"""Own and clean up ONLY the camera/receive-only bridge children started here."""
import argparse
import json
import os
import signal
import subprocess
from pathlib import Path

from configs import load, DEFAULT_CONFIG, PROJECT_ROOT
from coordinate_transform.geometry import RobotConvention


def main():
    p=argparse.ArgumentParser(); p.add_argument('--dataset',required=True)
    p.add_argument('--feedback-source-ip',required=True); p.add_argument('--duration',type=float,default=1800)
    p.add_argument('--controller-ip',required=True)
    args=p.parse_args(); out=Path(args.dataset); out.mkdir(parents=True,exist_ok=True)
    children=[]; logs=[]; exitcodes={}
    try:
        commands=[('camera',['bash',str(PROJECT_ROOT/'scripts/run_camera.sh')]),
                  ('receive_only_bridge',['ros2','run','estun_segmented','bridge','--receive-only',
                    '--feedback-source-ip',args.feedback_source_ip,'--controller-ip',args.controller_ip])]
        for name,cmd in commands:
            log=(out/(name+'.log')).open('a'); logs.append(log)
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,child))
        from .capture import run
        cfg=load(DEFAULT_CONFIG)
        run(cfg,RobotConvention(cfg['robot_core']),args.dataset,args.duration)
    finally:
        for name,child in reversed(children):
            # Signal the entire owned process group even when ros2 run's parent remains.
            try: os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError: pass
            try: child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM); child.wait(timeout=5)
            exitcodes[name]=child.returncode
        for log in logs: log.close()
        (out/'session_cleanup.json').write_text(json.dumps(exitcodes,indent=2))


if __name__=='__main__': main()
