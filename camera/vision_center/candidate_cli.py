"""Offline single-object candidate verification. Does not launch hardware."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from configs import load,save,DEFAULT_CONFIG
from coordinate_transform.geometry import RobotConvention
from .grasp_candidate import compute_candidate
from .candidate_pipeline import add_base_coordinates


def read_profile(path):
    path=Path(path)
    if path.suffix.lower() in ('.yaml','.yml'):
        import yaml
        info=yaml.safe_load(path.read_text(encoding='utf-8').split('\n---')[0])
    else: info=load(path)
    if 'header' in info:
        return dict(width=info['width'],height=info['height'],k=info['k'],d=info['d'],
                    distortion_model=info['distortion_model'],frame_id=info['header']['frame_id'])
    return info


def render(image,result,debug):
    display=image.copy()
    for region in result.get('segmentation',{}).get('regions',[]):
        x,y,w,h=region['bbox']; eligible=region['decision']=='eligible_complete_region'
        color=(0,255,0) if eligible else (0,0,255)
        cv2.rectangle(display,(x,y),(x+w-1,y+h-1),color,1)
        cv2.putText(display,f"R{region['id']} "+('accepted' if eligible else 'excluded'),
                    (x,max(15,y-4)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
    if 'object_mask' in debug:
        contours,_=cv2.findContours(debug['object_mask'],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(display,contours,-1,(0,255,0),2)
    if 'region_valid_depth_mask' in debug:
        selected=debug['region_valid_depth_mask']>0
        display[selected]=(255,255,0)
    for name,color in [('bbox_center_marker',(0,0,255))]:
        if name in debug: display[debug[name]>0]=color
    if result['success']:
        u,v=result['pixel']; cv2.drawMarker(display,tuple(np.rint([u,v]).astype(int)),(0,0,255),cv2.MARKER_CROSS,20,2)
        lines=[f"pixel {u},{v}  depth {result['depth_m']:.4f} m  valid {result['valid_point_count']}",
               'camera(m): '+', '.join(f'{x:.4f}' for x in result['point_camera_m']),
               'base(m): '+', '.join(f'{x:.4f}' for x in result['point_base_m']),
               f"height above table {result['height_above_table_m']:.4f} m",
               result['selection_reason']]
        lines+=['BBOX CENTER + REGION MEDIAN / REPLAY / NO MOTION']
    else: lines=['FAILED: '+result['reason'],'NO COORDINATES / NO MOTION']
    # Put readings below the image so the complete object contour stays visible.
    image_height=display.shape[0]
    display=cv2.copyMakeBorder(display,0,18*len(lines)+12,0,0,cv2.BORDER_CONSTANT,value=(20,20,20))
    for i,line in enumerate(lines):
        y=image_height+20+i*18
        cv2.putText(display,line[:105],(5,y),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
    return display


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',required=True); p.add_argument('--depth-series',required=True)
    p.add_argument('--image',required=True); p.add_argument('--depth-info',required=True)
    p.add_argument('--calibration',required=True); p.add_argument('--output',required=True)
    p.add_argument('--show',action='store_true')
    args=p.parse_args(); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    # Do not let screenshots from a previous successful measurement look like
    # evidence for a new failed run. Only touch this module's known artifacts.
    artifact_names=['region_depth_support','edge_depth_region','overview','local_depth_region','foreground_mask','object_mask','valid_interior_mask',
                    'initial_centroid_marker','candidate_pixel_marker','local_roi_border','local_depth_mask','segmented_region_mask',
                    'object_edge_mask','bbox_center_marker','region_valid_depth_mask','region_all_frames_valid_mask']
    for name in artifact_names:
        path=out/(name+'.png')
        if path.is_file(): path.unlink()
    debug={}; image=None
    try:
        context=load(args.snapshot); info=read_profile(args.depth_info)
        image=cv2.imread(args.image)
        if image is None or image.shape[:2]!=(info['height'],info['width']):
            raise ValueError('RGB_IMAGE_SIZE_MISMATCH')
        frames=np.load(args.depth_series,allow_pickle=False)
        result,debug=compute_candidate(frames,context['depth_encoding'],info,rgb=image)
        result=add_base_coordinates(result,context,info,load(args.calibration),RobotConvention(),load(DEFAULT_CONFIG))
        result['evidence']=dict(snapshot=str(args.snapshot),depth_series=str(args.depth_series),depth_info=str(args.depth_info),
            depth_profile_scope=('Capture includes paired depth CameraInfo; frame records are saved alongside snapshot'
                                 if 'depth_camera_info' in context else
                                 'External saved profile; per-frame CameraInfo timestamps are not available for this legacy burst'))
    except (ValueError,KeyError,OSError,TypeError,ImportError) as exc:
        result=dict(success=False,reason=str(exc),executable=False,motion_sent=False)
    save(out/'measurement.json',result)
    for name,mask in debug.items(): cv2.imwrite(str(out/(name+'.png')),mask*255)
    if image is not None:
        display=render(image,result,debug); cv2.imwrite(str(out/'overview.png'),display)
        if result['success']:
            x,y,w,h=result['object_bbox']
            crop=image[max(0,y-12):y+h+12,max(0,x-12):x+w+12].copy()
            roi=debug['region_valid_depth_mask'][max(0,y-12):y+h+12,max(0,x-12):x+w+12]>0
            crop[roi]=(crop[roi]*.3+np.array([255,255,0])*.7).astype(np.uint8)
            cv2.imwrite(str(out/'region_depth_support.png'),crop)
        if args.show:
            cv2.imshow('Single object surface candidate - OFFLINE',display)
            if result['success']: cv2.imshow('Valid enclosed region depth support',cv2.imread(str(out/'region_depth_support.png')))
            cv2.waitKey(0); cv2.destroyAllWindows()
    print(json.dumps(result,indent=2,ensure_ascii=False))
    if not result['success']: raise SystemExit(2)


if __name__=='__main__': main()
