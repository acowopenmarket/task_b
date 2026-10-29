"""Full-image RGB-D object localization without a table or box model.

Depth components propose search windows; RGB GrabCut refines each window.
Agreement is a consistency check, not independent recognition or object identity.
"""
import cv2
import numpy as np


def depth_metres(frames, encoding):
    d = np.asarray(frames, dtype=np.float32).copy()
    if encoding == '16UC1':
        d *= .001
    elif encoding != '32FC1':
        raise ValueError('UNSUPPORTED_DEPTH_ENCODING')
    d[~np.isfinite(d) | (d <= 0)] = np.nan
    return d


def rays_for(info):
    k = info['k']; y, x = np.indices((info['height'], info['width']))
    return np.stack(((x-k[2])/k[0], (y-k[5])/k[4], np.ones_like(x)), -1)


def filled(mask):
    out = np.zeros(mask.shape, np.uint8)
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, contours, -1, 1, cv2.FILLED)
    return out.astype(bool)


def continuous_surfaces(depth, valid, max_jump, minimum_area):
    """Connect measured neighbours, allowing total depth range on a curved face.

    Pixel vertices and their four-neighbour edges occupy separate graph cells.
    Invalid pixels never create edges, and diagonals cannot bypass a depth jump.
    """
    h,w=depth.shape
    graph=np.zeros((2*h-1,2*w-1),np.uint8)
    graph[::2,::2]=valid
    graph[::2,1::2]=(valid[:,:-1] & valid[:,1:] &
                     (np.abs(depth[:,1:]-depth[:,:-1])<=max_jump))
    graph[1::2,::2]=(valid[:-1,:] & valid[1:,:] &
                     (np.abs(depth[1:,:]-depth[:-1,:])<=max_jump))
    count,labels=cv2.connectedComponents(graph,connectivity=4)
    labels=labels[::2,::2]
    sizes=np.bincount(labels.ravel(),minlength=count)
    for i in np.flatnonzero((sizes>=minimum_area) & (sizes<=depth.size*.65)):
        if i:
            yield labels==i


def locate(frames, encoding, info, rgb, cfg, diagnostics=None):
    if diagnostics is not None:
        diagnostics.update(required_rgb_depth_iou=cfg.get('rgb_depth_min_iou',.75),candidates=[])
    depths = depth_metres(frames, encoding)
    if depths.ndim != 3 or depths.shape[1:] != rgb.shape[:2]:
        raise ValueError('DEPTH_SIZE_MISMATCH')
    # Use measured depths only. Holes are filled in masks, never in 3-D data.
    d = depths[-1]; valid = np.isfinite(d)
    step = cfg.get('depth_layer_m', .012)
    min_points = cfg.get('min_depth_points', 30)
    minimum_area = max(40, int(d.size * cfg.get('min_object_area_ratio', .001)))
    debug = {}; candidates = []
    if valid.sum() < min_points:
        return None, debug, 'INSUFFICIENT_TARGET_DEPTH'
    # First propose whole continuous surfaces: a curved object's total depth
    # span may be much larger than a band, while neighbour differences are small.
    proposals=[]
    def propose(region):
        mask=filled(region)
        x,y,w,h=cv2.boundingRect(mask.astype(np.uint8))
        area=np.count_nonzero(mask)
        if area<minimum_area or area>d.size*.65 or min(w,h)<5:
            return
        if any(np.count_nonzero(mask & old)/max(1,np.count_nonzero(mask | old)) > .65 for old in proposals):
            return
        proposals.append(mask)
    for region in continuous_surfaces(d,valid,step,minimum_area):
        propose(region)
    # Keep overlapping bands as additional proposals for measured surfaces
    # connected to their surroundings; all proposals face the same RGB checks.
    bins = np.unique(np.floor(d[valid]/step).astype(int))
    for b in bins:
        layer = valid & (d >= b*step) & (d < (b+2)*step)
        layer = cv2.morphologyEx(layer.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(layer)
        for i in range(1, count):
            x,y,w,h,area = stats[i]
            if area < minimum_area or area > d.size*.65 or min(w,h) < 5:
                continue
            propose(labels == i)
    for depth_mask in proposals:
        ys,xs = np.where(depth_mask)
        x,y,w,h = cv2.boundingRect(depth_mask.astype(np.uint8))
        pad = max(6, int(min(w,h)*.15))
        x0,y0 = max(0,x-pad),max(0,y-pad)
        x1,y1 = min(d.shape[1],x+w+pad),min(d.shape[0],y+h+pad)
        # RGB classification uses the whole rectangle, not a depth mask pasted
        # into RGB. Depth still supplies the proposal, so agreement is conditional.
        crop=rgb[y0:y1,x0:x1]
        scale=min(1.,240/max(crop.shape[:2]))
        small=cv2.resize(crop,None,fx=scale,fy=scale)
        gc = np.full(small.shape[:2], cv2.GC_PR_FGD, np.uint8)
        if y>y0: gc[:2]=0
        if y+h<y1: gc[-2:]=0
        if x>x0: gc[:,:2]=0
        if x+w<x1: gc[:,-2:]=0
        if not np.any(gc == cv2.GC_BGD):
            continue
        try:
            cv2.setRNGSeed(0)
            cv2.grabCut(small, gc, None, np.zeros((1,65)), np.zeros((1,65)), 3, cv2.GC_INIT_WITH_MASK)
        except cv2.error:
            continue
        rgb_mask=np.zeros(d.shape,bool)
        rgb_mask[y0:y1,x0:x1]=cv2.resize(((gc == cv2.GC_FGD) | (gc == cv2.GC_PR_FGD)).astype(np.uint8),
            (x1-x0,y1-y0),interpolation=cv2.INTER_NEAREST)>0
        rgb_mask = filled(rgb_mask)
        union = np.count_nonzero(rgb_mask | depth_mask)
        overlap = np.count_nonzero(rgb_mask & depth_mask)/max(1,union)
        check=dict(bbox=[int(x),int(y),int(w),int(h)],rgb_depth_iou=float(overlap),
            depth_area_px=int(np.count_nonzero(depth_mask)),rgb_area_px=int(np.count_nonzero(rgb_mask)),
            rejected_by=None)
        if diagnostics is not None: diagnostics['candidates'].append(check)
        if overlap < cfg.get('rgb_depth_min_iou', .75):
            check['rejected_by']='RGB_DEPTH_IOU_BELOW_THRESHOLD'
            continue
        mask = rgb_mask & depth_mask
        measured = mask & valid
        if measured.sum() < min_points:
            check['rejected_by']='INSUFFICIENT_TARGET_DEPTH'
            continue
        # A target surface should stand forward of its immediate surroundings.
        # This compares neighbouring measurements; it fits no support plane.
        ring=(cv2.dilate(mask.astype(np.uint8),np.ones((31,31),np.uint8))>0) & ~mask & valid
        if ring.sum()<min_points:
            check['rejected_by']='INSUFFICIENT_SURROUNDING_DEPTH'
            continue
        relief=float(np.median(d[ring])-np.median(d[measured]))
        check['depth_separation_m']=relief
        if relief <= cfg.get('min_depth_separation_m', .004):
            check['rejected_by']='INSUFFICIENT_DEPTH_SEPARATION'
            continue
        score = overlap**2 * np.sqrt(np.count_nonzero(mask))
        candidates.append((score, mask, depth_mask, rgb_mask, overlap))
    if not candidates:
        if diagnostics is not None:
            checks=diagnostics['candidates']
            diagnostics['failure_detail']=('RGB_DEPTH_IOU_BELOW_THRESHOLD' if checks and
                all(c['rejected_by']=='RGB_DEPTH_IOU_BELOW_THRESHOLD' for c in checks)
                else 'NO_VALID_RGBD_CANDIDATE')
        return None, debug, 'NO_MATCHING_RGBD_OBJECT'
    _,mask,depth_mask,rgb_mask,overlap = max(candidates, key=lambda a:a[0])
    medians=[]
    for f in depths:
        z=f[mask & np.isfinite(f)]
        if len(z)<min_points:
            return None, debug, 'TARGET_DEPTH_MISSING_IN_FRAME'
        medians.append(float(np.median(z)))
    if np.ptp(medians)>cfg.get('target_temporal_span_m', .006):
        return None, debug, 'TARGET_DEPTH_UNSTABLE'
    # Median accepted surface depth; not a volumetric object centre.
    x,y,w,h=cv2.boundingRect(mask.astype(np.uint8)); u=x+(w-1)/2; v=y+(h-1)/2
    z=medians[-1]; k=info['k']
    debug.update(object_mask=mask.astype(np.uint8),depth_object=depth_mask.astype(np.uint8),
                 rgb_object=rgb_mask.astype(np.uint8))
    return dict(success=True,reason='OK',pixel=[u,v],object_bbox=[x,y,w,h],
        point_camera_m=[(u-k[2])*z/k[0],(v-k[5])*z/k[4],z],
        point_kind='bbox_center_target_depth_median',depth_m=z,rgb_depth_iou=overlap,
        frame_depth_medians_m=medians),debug,'OK'
