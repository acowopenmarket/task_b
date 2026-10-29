"""Trace saved RGB-D candidates offline; never imports hardware/ROS."""
import argparse
import inspect
import json
from pathlib import Path
import sys
import cv2
import numpy as np
from rgbd_candidate import locate

def main():
    p=argparse.ArgumentParser()
    p.add_argument('capture',type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rec=json.loads((a.capture/'records.json').read_text())[-1]
    cfg=json.loads((Path(__file__).parent/'config.json').read_text())['vision']
    checks=[];regions=[]
    lines,start=inspect.getsourcelines(locate)
    overlap_line=start+next(i for i,s in enumerate(lines) if 'if overlap <' in s)
    relief_line=start+next(i for i,s in enumerate(lines) if 'if relief <=' in s)
    def trace(frame,event,arg):
        if event=='line' and frame.f_code is locate.__code__:
            v=frame.f_locals
            if frame.f_lineno==overlap_line:
                i=len(checks)
                checks.append(dict(bbox=[int(v[k]) for k in ('x','y','w','h')],
                    iou=float(v['overlap']),depth_area=int(v['depth_mask'].sum()),rgb_area=int(v['rgb_mask'].sum())))
                cv2.imwrite(str(a.output/f'candidate_{i}_depth.png'),v['depth_mask'].astype(np.uint8)*255)
                cv2.imwrite(str(a.output/f'candidate_{i}_rgb.png'),v['rgb_mask'].astype(np.uint8)*255)
            if frame.f_lineno==relief_line: checks[-1]['relief_m']=float(v['relief'])
        if event=='return' and frame.f_code.co_name=='propose':
            v=frame.f_locals
            if 'area' in v:
                regions.append(dict(bbox=[int(v[k]) for k in ('x','y','w','h')],area=int(v['area'])))
        return trace
    diagnostics={}
    sys.settrace(trace)
    try:
        result,debug,reason=locate(np.load(a.capture/'depth.npy'),rec['depth_encoding'],
            rec['camera_info'],np.load(a.capture/'rgb.npy')[-1],cfg,diagnostics=diagnostics)
    finally: sys.settrace(None)
    for name,mask in debug.items(): cv2.imwrite(str(a.output/(name+'.png')),mask*255)
    report=dict(reason=reason,result=result,proposals=regions,candidates=checks,diagnostics=diagnostics,
        mode='offline_only',motion_sent=False)
    (a.output/'diagnostics.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report))

if __name__=='__main__':main()
