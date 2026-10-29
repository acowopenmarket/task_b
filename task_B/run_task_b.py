#!/usr/bin/env python3
"""默认模拟预览。真实任务必须 --execute；--capture 仅采集，不动作。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import paths
from core import Task, validate, compute_target
from estun_segmented.runner import Journal


def read(path): return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def save(path,value): Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=paths.ROOT/'config.json')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--capture',action='store_true',help='只读持续相机+只读bridge，采集一次')
    mode.add_argument('--vision-only',action='store_true',help='仅相机RGB-D识别；无需机械臂反馈，不转换基座坐标')
    mode.add_argument('--replay',type=Path,help='离线重算 capture_目录，绝不执行')
    mode.add_argument('--check-config',action='store_true')
    p.add_argument('--scenario',type=Path,help='仅模拟模式，JSON视觉观察序列')
    p.add_argument('--output',type=Path)
    a=p.parse_args(argv)
    live=a.execute or a.capture
    if a.scenario and (live or a.replay or a.vision_only): p.error('--scenario仅可用于模拟')
    run=a.output or paths.ROOT/('runs/'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
    run.mkdir(parents=True,exist_ok=False)
    j=Journal(run/'events.jsonl'); devices=None; result={'success':False}
    try:
        c=read(a.config)
        save(run/'input_config.json',c)
        # Snapshot exact reused source/calibration hashes for traceability.
        evidence=[paths.ROOT/f for f in ('core.py','vision.py','rgbd_candidate.py','capture_sync.py','hardware.py','run_task_b.py','config.json')]
        evidence += [paths.PROJECT/f for f in ('camera/coordinate_transform/geometry.py',
            'camera/vision_center/grasp_candidate.py','camera/vision_center/candidate_pipeline.py',
            'estun/ros2_ws/src/estun_segmented/estun_segmented/runner.py',
            'estun/ros2_ws/src/estun_segmented/estun_segmented/core.py',
            'estun/ros2_ws/src/estun_segmented/estun_segmented/bridge.py',
            'jia/ros2_ws/src/ms42_gripper/ms42_gripper/client.py',
            'jia/ros2_ws/src/ms42_gripper/ms42_gripper/protocol.py')]
        save(run/'source_hashes.json',{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in evidence if f.is_file()})
        if a.check_config:
            validate(c,live=True); result={'success':True,'mode':'config_only'}
        elif a.vision_only:
            validate(c)
            from hardware import CameraOnly
            devices=CameraOnly(c,run,j,False)
            devices.start()
            result=devices.capture()
            result['mode']='camera_only'
        elif a.replay:
            validate(c)
            import cv2,numpy as np
            from hardware import resolve
            from vision import analyze,tag
            result,debug=analyze(np.load(a.replay/'depth.npy',allow_pickle=False),
                np.load(a.replay/'rgb.npy',allow_pickle=False),read(a.replay/'records.json'),
                read(resolve(c['hardware']['calibration_path'])),c)
            result['mode']='replay_only'
            if result.get('localizable') and result.get('top_valid'):
                result['target']=compute_target(tag(result),c)
            for name,mask in debug.items(): cv2.imwrite(str(run/(name+'.png')),mask*255)
        elif live:
            validate(c,live=a.execute)
            from hardware import Devices,Motion,resolve
            import ipaddress
            for key in ('controller_ip','feedback_source_ip'):
                ip=ipaddress.ip_address(c['hardware'][key])
                if ip.is_unspecified or ip.is_multicast or str(ip) in ('192.0.2.10','192.0.2.100','198.51.100.1','203.0.113.1'):
                    raise ValueError('拒绝示例/非法IP: '+key)
            if 'REPLACE' in c['hardware']['gripper_port']:
                raise ValueError('拒绝USB占位符')
            calibration=read(resolve(c['hardware']['calibration_path']))
            if calibration.get('status')!='validated_real' or not calibration.get('heldout_consistency_verified'):
                raise ValueError('必须使用本机真实验证标定')
            if (calibration.get('example_notice') or calibration.get('synthetic')
                    or 'example' in str(c['hardware']['calibration_path']).lower()
                    or not calibration.get('dataset_sha256')):
                raise ValueError('拒绝示例/合成/无采集数据指纹标定；请提供本机真实标定')
            save(run/'calibration_used.json',calibration)
            if int(os.environ.get('ROS_DOMAIN_ID','-1'))!=c['hardware']['ros_domain_id']:
                raise ValueError('ROS_DOMAIN_ID与集中配置不一致')
            devices=Devices(c,run,j,a.execute)
            devices.start()
            if a.execute:
                result=Task(c,Motion(devices),devices,devices,j).run()
                result['mode']='hardware_execute'
            else:
                # Read-only acquisition is valid only at the configured observation pose.
                from estun_segmented.core import errors,Settings
                s=Settings(**c['motion']); pe,ae=errors(devices.node.read_start().pose,c['observation_pose'])
                if pe>s.position_tolerance or ae>s.angle_tolerance:
                    raise ValueError('请先用示教器到配置观察位并静止，再只读采集')
                result=devices.capture()
                result['mode']='live_readonly'
                if result.get('localizable') and result.get('top_valid'):
                    result['target']=compute_target(result,c)
        else:
            from sim import demo_config,Vision,Motion,Gripper
            c=demo_config(c); save(run/'simulation_config.json',c)
            j.write('SIMULATION_ONLY',note='使用虚构演示箱体和观察位；未连接设备')
            v=Vision(read(a.scenario) if a.scenario else None)
            result=Task(c,Motion(c,j),Gripper(),v,j).run()
            result['mode']='simulation_only'
    except (Exception,KeyboardInterrupt) as exc:
        result={'success':False,'aborted':True,'reason':str(exc)}
        j.write('ABORT',**result)
    finally:
        if devices:
            try: devices.close()
            except Exception as exc:
                result.update(cleanup_error=str(exc),success=False)
        save(run/'result.json',result); j.write('RESULT',**result); j.close()
    print('日志目录: '+str(run.resolve()))
    valid_readonly=(result.get('mode') in ('replay_only','live_readonly','camera_only')
                    and result.get('presence') in ('PRESENT','ABSENT'))
    return 0 if result.get('success') or valid_readonly else 2


if __name__=='__main__': raise SystemExit(main())
