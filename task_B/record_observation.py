"""Receive real controller UDP feedback only; no socket send or motion imports.

Windows direct feedback endpoint. Do not run concurrently with a feedback relay.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import socket
import time
import paths
from estun_segmented.wire import decode_feedback
from estun_segmented.core import errors


def select_stable(samples, duration, translation, rotation, max_gap):
    if len(samples)<10 or samples[-1]['received']-samples[0]['received']<duration:
        return None
    # Recording a stationary PowerOff pose is read-only configuration, not
    # permission to execute. The original motion adapter still requires AutoReady.
    if any(s['feedback']['fault_flag']!=0 or s['feedback']['state'] not in ('AutoReady','Idle','PowerOff') for s in samples):
        return None
    if any(b['received']-a['received']>max_gap for a,b in zip(samples,samples[1:])):
        return None
    pose=samples[-1]['feedback']['end_positions']
    for s in samples:
        p,a=errors(s['feedback']['end_positions'],pose)
        if p>translation or a>rotation:
            return None
    return list(pose)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=paths.ROOT/'config.json')
    p.add_argument('--listen-ip',default='192.0.2.20')
    p.add_argument('--timeout',type=float,default=12.)
    p.add_argument('--save',action='store_true',help='Save this fresh stable pose to observation_pose')
    a=p.parse_args()
    config=json.loads(a.config.read_text(encoding='utf-8-sig'))
    out=paths.ROOT/'runs'/('observation_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True)
    evidence=dict(success=False,read_only=True,motion_sent=False,controller_ip=config['hardware']['controller_ip'],
                  listen_address=[a.listen_ip,9006],samples=[],captured_at=None)
    try:
        start=time.monotonic(); window=[]; pose=None
        duration=max(.8,config['motion']['settle_seconds'])
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as rx:
            # Exclusive port ownership; never enable SO_REUSEADDR.
            if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):
                rx.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
            rx.bind((a.listen_ip,9006)); rx.settimeout(.5)
            while time.monotonic()-start<a.timeout:
                try: raw,source=rx.recvfrom(65535)
                except socket.timeout: continue
                if source[0]!=config['hardware']['controller_ip']: continue
                sample=dict(received=time.monotonic(),received_at=datetime.now().astimezone().isoformat(),
                            source=list(source),feedback=decode_feedback(raw))
                evidence['samples'].append(sample); window.append(sample)
                # Keep at least a full settle interval, bounded to 2x its length.
                window=[s for s in window if sample['received']-s['received']<=duration*2]
                if sample['feedback']['fault_flag']!=0:
                    raise RuntimeError('控制器故障，未写入观察位: '+str(sample['feedback']))
                pose=select_stable(window,duration,config['vision']['settle_translation_m'],
                    config['vision']['settle_rotation_rad'],config['motion']['feedback_timeout'])
                if pose is not None: break
        if pose is None:
            raise TimeoutError('未收到足够的新鲜、无故障且静止的控制器反馈，未修改配置')
        evidence.update(success=True,observation_pose=pose,units='robot_base_m_rad',
            controller_state=window[-1]['feedback']['state'],joint_positions=window[-1]['feedback']['joint_positions'],
            execution_ready=window[-1]['feedback']['state']=='AutoReady',
            captured_at=window[-1]['received_at'],stable_window_s=window[-1]['received']-window[0]['received'],
            selected_pose_policy='latest_real_feedback_in_stable_window_not_average')
        if a.save:
            # Preserve exact previous bytes and only replace the requested field.
            original=a.config.read_bytes()
            (out/'config_before.json').write_bytes(original)
            config['observation_pose']=pose
            a.config.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            evidence['config_updated']=str(a.config.resolve())
        print(json.dumps({k:v for k,v in evidence.items() if k!='samples'},ensure_ascii=False,indent=2))
        return 0
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        evidence.update(success=False,reason=str(exc))
        print(str(exc)); return 2
    finally:
        (out/'feedback_evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
        print('证据目录: '+str(out))


if __name__=='__main__': raise SystemExit(main())
