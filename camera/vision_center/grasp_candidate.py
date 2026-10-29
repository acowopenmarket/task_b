"""Bounding-rectangle center + valid enclosed-region depth median; no robot I/O.

The measured point is NOT an internal object center or a final gripping center.
Returns explicit failure with no coordinates when evidence is insufficient.
"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class CandidateConfig:
    table_distance_m: float = .003
    min_table_ratio: float = .35
    min_table_border_ratio: float = .35
    min_object_height_m: float = .008
    max_object_height_m: float = .4
    min_object_area_px: int = 250
    min_region_valid_pixels: int = 20
    max_temporal_span_m: float = .006
    min_frames: int = 3
    rgb_border_guard_px: int = 5


def aligned_profiles(color, depth):
    if any(color[k] != depth[k] for k in ('width','height','frame_id')): return False
    if not np.allclose(color['k'],depth['k'],atol=1e-4,rtol=0): return False
    models=('plumb_bob','rational_polynomial')
    if any(p['distortion_model'] not in models or len(p['d'])>8 for p in (color,depth)): return False
    return bool(np.allclose(np.pad(color['d'],(0,8-len(color['d']))),
                            np.pad(depth['d'],(0,8-len(depth['d']))),atol=1e-8,rtol=0))


def depth_metres(raw, encoding):
    if encoding not in ('16UC1','32FC1'): raise ValueError('UNKNOWN_DEPTH_ENCODING')
    d=np.asarray(raw,dtype=float)*(.001 if encoding=='16UC1' else 1.)
    d[(d<=0)|~np.isfinite(d)]=np.nan
    return d


def rays_for(info):
    w,h=info['width'],info['height']; k=np.array(info['k'],float).reshape(3,3)
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0: raise ValueError('INVALID_INTRINSICS')
    yy,xx=np.indices((h,w)); pixels=np.stack((xx,yy),-1).astype(float).reshape(-1,1,2)
    d=np.asarray(info['d'],float)
    if not np.isfinite(d).all(): raise ValueError('INVALID_DISTORTION')
    if info['distortion_model'] in ('plumb_bob','rational_polynomial'):
        uv=cv2.undistortPoints(pixels,k,d if d.size else None)
    elif info['distortion_model']=='equidistant':
        uv=cv2.fisheye.undistortPoints(pixels,k,d)
    else: raise ValueError('UNSUPPORTED_DISTORTION_MODEL')
    return np.dstack((uv.reshape(h,w,2),np.ones((h,w))))


def fit_table(points, cfg):
    sample=points[::4,::4].reshape(-1,3); sample=sample[np.isfinite(sample).all(axis=1)]
    if len(sample)<200: raise ValueError('INSUFFICIENT_TABLE_DEPTH')
    rng=np.random.default_rng(42)
    if len(sample)>3000: sample=sample[rng.choice(len(sample),3000,replace=False)]
    best=None; best_count=0
    for _ in range(160):
        a,b,c=sample[rng.choice(len(sample),3,replace=False)]
        n=np.cross(b-a,c-a); length=np.linalg.norm(n)
        if length<1e-9: continue
        n/=length; offset=-float(n@a)
        inside=np.abs(sample@n+offset)<cfg.table_distance_m
        if int(inside.sum())>best_count: best=inside; best_count=int(inside.sum())
    if best is None or best_count/len(sample)<cfg.min_table_ratio:
        raise ValueError('TABLE_PLANE_NOT_RELIABLE')
    support=sample[best]; mean=support.mean(axis=0)
    _,_,vt=np.linalg.svd(support-mean,full_matrices=False)
    n=vt[-1]; offset=-float(n@mean)
    if offset<0: n=-n; offset=-offset
    # A dominant object face must not silently become the tabletop.
    ring=np.zeros(points.shape[:2],bool); width=max(3,min(ring.shape)//12)
    ring[:width]=ring[-width:]=True; ring[:,:width]=ring[:,-width:]=True
    border=points[ring]; border=border[np.isfinite(border).all(axis=1)]
    if len(border)<100: raise ValueError('INSUFFICIENT_VISIBLE_TABLE_BORDER')
    ratio=float(np.mean(np.abs(border@n+offset)<cfg.table_distance_m))
    if ratio<cfg.min_table_border_ratio: raise ValueError('TABLE_BORDER_SUPPORT_TOO_LOW')
    rms=float(np.sqrt(np.mean((support@n+offset)**2)))
    return n,offset,dict(normal_camera=n.tolist(),offset_m=offset,inlier_ratio=best_count/len(sample),
                         border_inlier_ratio=ratio,rms_m=rms)


def region_depth(frame, rays, region_mask, plane, cfg):
    """All valid pixels enclosed by the object outer contour, including interior.

    Valid means finite, positive, above the fitted tabletop and in the configured
    object-height range. All these samples contribute to the median: no depth
    averaging, invented values, MAD trimming or substitution of a local ROI.
    """
    n,offset=plane; heights=(rays*frame[...,None])@n+offset
    accepted=(region_mask>0)&np.isfinite(frame)&(frame>0)&(heights>cfg.min_object_height_m)&(heights<cfg.max_object_height_m)
    values=frame[accepted]
    if len(values)<cfg.min_region_valid_pixels: raise ValueError('INSUFFICIENT_VALID_REGION_DEPTH')
    z=float(np.median(values))
    return z,dict(valid_count=int(len(values)),region_pixel_count=int(np.count_nonzero(region_mask)),
                  median_m=z,mad_m=float(np.median(np.abs(values-z))),
                  depth_min_m=float(values.min()),depth_max_m=float(values.max())),accepted


def ray_at_pixel(info, pixel):
    k=np.array(info['k'],float).reshape(3,3); d=np.asarray(info['d'],float)
    p=np.array([[pixel]],float)
    if info['distortion_model']=='equidistant': uv=cv2.fisheye.undistortPoints(p,k,d)
    else: uv=cv2.undistortPoints(p,k,d if d.size else None)
    return np.r_[uv.reshape(2),1.]


def object_regions(foreground, rgb, cfg, debug):
    """Use RGB appearance to group fragmented depth support, without class labels.

    GrabCut changes only segmentation. It never synthesizes measured depth.
    Regions intersecting the image/background guard are incomplete, so cannot
    be counted as complete measurable objects (e.g. a cable entering the image).
    """
    grouped=cv2.morphologyEx(foreground.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    raw_contours,_=cv2.findContours(grouped,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    raw_count=sum(cv2.contourArea(c)>=cfg.min_object_area_px for c in raw_contours)
    guard=1; method='depth_connected_regions'
    if rgb is not None:
        if rgb.shape!=(*foreground.shape,3) or rgb.dtype!=np.uint8:
            raise ValueError('RGB_DEPTH_GEOMETRY_MISMATCH')
        core=cv2.distanceTransform(foreground.astype(np.uint8),cv2.DIST_L2,5)>2
        if int(core.sum())<cfg.min_region_valid_pixels: raise ValueError('NO_RELIABLE_RGBD_FOREGROUND_SEEDS')
        guard=cfg.rgb_border_guard_px
        mask=np.full(foreground.shape,cv2.GC_PR_BGD,np.uint8)
        mask[cv2.dilate(foreground.astype(np.uint8),np.ones((9,9),np.uint8))>0]=cv2.GC_PR_FGD
        mask[core]=cv2.GC_FGD
        mask[:guard]=mask[-guard:]=cv2.GC_BGD; mask[:,:guard]=mask[:,-guard:]=cv2.GC_BGD
        cv2.setRNGSeed(42)
        cv2.grabCut(rgb,mask,None,np.zeros((1,65),float),np.zeros((1,65),float),5,cv2.GC_INIT_WITH_MASK)
        grouped=np.isin(mask,[cv2.GC_FGD,cv2.GC_PR_FGD]).astype(np.uint8)
        grouped=cv2.morphologyEx(grouped,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        method='depth_seeded_rgb_grabcut_regions'
    debug['segmented_region_mask']=grouped
    contours,_=cv2.findContours(grouped,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted([c for c in contours if cv2.contourArea(c)>=cfg.min_object_area_px],key=cv2.contourArea,reverse=True)
    eligible=[]; review=[]
    for i,c in enumerate(contours):
        x,y,w,h=cv2.boundingRect(c)
        clipped=bool(x<=guard or y<=guard or x+w>=foreground.shape[1]-guard or y+h>=foreground.shape[0]-guard)
        region=np.zeros(foreground.shape,np.uint8); cv2.drawContours(region,[c],-1,1,-1)
        support=int(np.count_nonzero((region>0)&foreground))
        decision='OBJECT_REGION_CLIPPED' if clipped else ('INSUFFICIENT_DEPTH_SUPPORT' if support<cfg.min_region_valid_pixels else 'eligible_complete_region')
        review.append(dict(id=i+1,bbox=[int(x),int(y),int(w),int(h)],contour_area_px=float(cv2.contourArea(c)),
                           measured_foreground_pixels=support,decision=decision))
        if decision=='eligible_complete_region': eligible.append(c)
    if not eligible:
        reason='OBJECT_REGION_CLIPPED' if any(r['decision']=='OBJECT_REGION_CLIPPED' for r in review) else 'NO_SINGLE_OBJECT_REGION'
    elif len(eligible)>1: reason='MULTIPLE_OBJECTS: single-object input required'
    else: reason=None
    return eligible,dict(method=method,raw_depth_region_count=int(raw_count),regions=review),reason


def compute_candidate(raw_frames, encoding, depth_info, config=None, rgb=None):
    """Fixed image bbox center XY and enclosed-region median camera-depth Z.

    No contour centroid, center-neighborhood depth, interior point search or
    fallback. Missing center depth does not move XY. Insufficient region depth fails.
    """
    cfg=config or CandidateConfig(); debug={}; segmentation={}
    kind='bbox_center_region_depth_candidate'
    try:
        frames=depth_metres(raw_frames,encoding)
        if frames.ndim!=3 or len(frames)<cfg.min_frames: raise ValueError('INSUFFICIENT_TEMPORAL_FRAMES')
        if frames.shape[1:]!=(depth_info['height'],depth_info['width']): raise ValueError('DEPTH_SIZE_MISMATCH')
        rays=rays_for(depth_info)
        frame=frames[-1]; points=rays*frame[...,None]
        n,offset,table=fit_table(points,cfg); height=points@n+offset
        foreground=np.isfinite(frame)&(height>cfg.min_object_height_m)&(height<cfg.max_object_height_m)
        debug['foreground_mask']=foreground.astype(np.uint8)
        contours,segmentation,failure=object_regions(foreground,rgb,cfg,debug)
        if failure: raise ValueError(failure)
        contour=contours[0]; x,y,w,h=cv2.boundingRect(contour)
        silhouette=np.zeros(frame.shape,np.uint8); cv2.drawContours(silhouette,[contour],-1,1,-1)
        debug['object_mask']=silhouette
        if x<=1 or y<=1 or x+w>=frame.shape[1]-1 or y+h>=frame.shape[0]-1:
            raise ValueError('OBJECT_REGION_CLIPPED')
        # Bounds refer to the first/last enclosed pixel centers, inclusive.
        xmin,ymin,xmax,ymax=int(x),int(y),int(x+w-1),int(y+h-1)
        pixel=[(xmin+xmax)/2.,(ymin+ymax)/2.]
        edge=np.zeros(frame.shape,np.uint8)
        cv2.drawContours(edge,[contour],-1,1,1,lineType=cv2.LINE_8)
        debug['object_edge_mask']=edge
        marker=np.zeros(frame.shape,np.uint8)
        cv2.drawMarker(marker,tuple(np.rint(pixel).astype(int)),1,cv2.MARKER_CROSS,20,2)
        debug['bbox_center_marker']=marker
        measurements=[]; quality=[]; accepted_frames=[]; all_values=[]
        for number,f in enumerate(frames):
            try: z,q,accepted=region_depth(f,rays,silhouette,(n,offset),cfg)
            except ValueError as exc: raise ValueError(f'FRAME_{number}: {exc}') from exc
            measurements.append(z); quality.append(q); accepted_frames.append(accepted)
            all_values.append(f[accepted])
        span=float(np.ptp(measurements))
        if span>cfg.max_temporal_span_m: raise ValueError('TEMPORAL_REGION_DEPTH_UNSTABLE')
        # Z uses the current frame's whole-region median, paired with the current RGB
        # rectangle and capture pose. Previous frames only verify stability.
        z=measurements[-1]
        pc=ray_at_pixel(depth_info,pixel)*z
        candidate_height=float(pc@n+offset)
        debug['region_valid_depth_mask']=accepted_frames[-1].astype(np.uint8)
        debug['region_all_frames_valid_mask']=np.logical_and.reduce(accepted_frames).astype(np.uint8)
        values=all_values[-1]
        result=dict(success=True,reason='OK',point_kind=kind,
            pixel=pixel,selection_reason='axis_aligned_bbox_center_with_valid_enclosed_region_depth_median',
            center_method='axis_aligned_bounding_rectangle_pixel_extents_center',
            center_relocated=False,center_pixel_depth_used=False,
            depth_median_source='all_valid_pixels_enclosed_by_external_contour_last_frame',
            bbox_pixel_bounds=[xmin,ymin,xmax,ymax],object_bbox=[int(x),int(y),int(w),int(h)],
            object_area_px=int(silhouette.sum()),depth_m=z,point_camera_m=pc.tolist(),
            height_above_table_m=candidate_height,
            height_method='constructed_bbox_center_ray_at_region_median_depth_to_table_plane',
            valid_point_count=quality[-1]['valid_count'],minimum_frame_valid_count=min(q['valid_count'] for q in quality),
            region_depth=quality[-1],region_valid_depth_values_m=values.tolist(),
            frame_count=len(frames),temporal_region_depth_m=measurements,temporal_span_m=span,
            table_plane=table,segmentation=segmentation,
            note='BBox image center projected at measured whole-region median depth; not a direct center-pixel depth measurement or verified gripping center.',
            executable=False,motion_sent=False)
        return result,debug
    except (ValueError,KeyError,cv2.error,np.linalg.LinAlgError) as exc:
        return dict(success=False,reason=str(exc),point_kind=kind,segmentation=segmentation,
                    executable=False,motion_sent=False),debug
