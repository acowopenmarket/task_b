"""Create synthetic five-position RGB-D QA images. No device or real calibration."""
import json
from pathlib import Path
import cv2
import numpy as np
from test_task_b import vision_fixture
from vision import analyze


def main():
    out=Path(__file__).resolve().parent/'offline_preview'
    out.mkdir(exist_ok=True)
    panels=[]; observations=[]
    positions=[('corner_1',(14,14)),('corner_2',(54,14)),('corner_3',(14,34)),('corner_4',(54,34)),('center',(34,24))]
    for name,xy in positions:
        c,cal,records,frames,rgb,empty=vision_fixture(xy)
        result,debug=analyze(frames,rgb,records,cal,c,empty)
        observations.append(dict(position=name,**result))
        image=cv2.resize(rgb[-1],(400,300),interpolation=cv2.INTER_NEAREST)
        contours,_=cv2.findContours(debug['roi'],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image,[x*5 for x in contours],-1,(0,200,0),2)
        cv2.rectangle(image,(xy[0]*5,xy[1]*5),((xy[0]+17)*5,(xy[1]+17)*5),(0,0,255),2)
        image=cv2.copyMakeBorder(image,60,0,0,0,cv2.BORDER_CONSTANT,value=(25,25,25))
        cv2.putText(image,name+' / '+result['presence'],(8,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
        cv2.putText(image,'SYNTHETIC | top=%.3fm'%result.get('z_top',-1),(8,48),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        cv2.imwrite(str(out/(name+'.png')),image); panels.append(image)
    legend=np.full_like(panels[0],25)
    for i,line in enumerate(['OFFLINE RGB-D ONLY','Green: usable box ROI','Red: synthetic object','Same default grasp direction','No hardware validation']):
        cv2.putText(legend,line,(10,70+i*35),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    panels.append(legend)
    grid=np.vstack((np.hstack(panels[:3]),np.hstack(panels[3:])))
    cv2.imwrite(str(out/'five_positions.png'),grid)
    (out/'observations.json').write_text(json.dumps(observations,indent=2,ensure_ascii=False),encoding='utf-8')
    print(out)


if __name__=='__main__': main()
