"""Run only pure offline suites; save exact output and counts in a new directory."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent


def main():
    out=ROOT/'runs'/('verification_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    for f in ROOT.glob('*.py'): ast.parse(f.read_text(encoding='utf-8'),filename=str(f))
    suites=[('task_b',ROOT,['-m','unittest','-v','test_capture_sync','test_analysis_feedback','test_rgbd_candidate','test_task_b','test_record_observation'],[]),
            ('camera_full',PROJECT,['-m','unittest','discover','-s','camera/tests','-v'],[PROJECT/'camera']),
            ('robot',PROJECT,['-m','unittest','discover','-s','estun/tests','-p','test_motion.py','-v'],[]),
            ('pick',PROJECT,['-m','unittest','discover','-s','try/tests','-p','test_pick.py','-v'],[]),
            ('gripper_protocol',PROJECT,['-m','unittest','discover','-s','jia/ros2_ws/src/ms42_gripper/test','-p','test_protocol.py','-v'],[PROJECT/'jia/ros2_ws/src/ms42_gripper']),
            ('gripper_client',PROJECT,['-m','unittest','discover','-s','jia/ros2_ws/src/ms42_gripper/test','-p','test_client.py','-v'],[PROJECT/'jia/ros2_ws/src/ms42_gripper'])]
    results=[]
    for name,cwd,args,extra in suites:
        env=dict(os.environ,PYTHONUTF8='1',PYTHONIOENCODING='utf-8',PYTHONDONTWRITEBYTECODE='1')
        env['PYTHONPATH']=os.pathsep.join(map(str,extra))+os.pathsep+env.get('PYTHONPATH','')
        command=[sys.executable]+args
        p=subprocess.run(command,cwd=cwd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf-8',errors='replace',timeout=120)
        (out/(name+'.txt')).write_text(p.stdout,encoding='utf-8')
        results.append(dict(suite=name,exit_code=p.returncode,command=command,cwd=str(cwd)))
        print(name+': '+('PASS' if p.returncode==0 else 'FAIL'))
    (out/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print(str(out)); return int(any(x['exit_code'] for x in results))


if __name__=='__main__': raise SystemExit(main())
