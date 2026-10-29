"""Observation return adapter. Uses existing motion validation and controller.

Executed only by an explicit panel action in WSL; importing does not move devices.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import paths
from core import check_workspace, validate
from estun_segmented.core import Settings, errors
from estun_segmented.runner import Journal, make_node
from hardware import Devices


class ObservationGripper(Devices):
    """Reuse serial exclusivity and completion checks without starting a camera."""
    def healthy(self,camera=True):
        if any(child.poll() is not None for child in self.children):
            raise RuntimeError('bridge子进程已退出')
        return super().healthy(camera=False)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    journal=Journal(args.output/'events.jsonl')
    node=ros=child=log=lock=devices=None
    result={'success':False,'mode':'return_observation','observation_reached':False,'gripper_open_completed':False}
    try:
        # Same process lock as task_B: another task or return cannot overlap.
        lock=open('/tmp/estun_task_b.lock','w')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        c=json.loads(args.config.read_text(encoding='utf-8-sig'))
        validate(c,live=True)
        check_workspace(c['observation_pose'],c)
        (args.output/'input_config.json').write_text(json.dumps(c,ensure_ascii=False,indent=2))
        import rclpy
        ros=rclpy
        ros.init(args=[])
        settings=Settings(**c['motion'])
        node=make_node(journal,settings)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            node.spin()
        if (node.count_publishers('/RobotInfo') or node.count_publishers('/estun_bridge/status')
                or node.count_publishers('/RobotMove')!=1 or node.publisher.get_subscription_count()):
            raise RuntimeError('已有运动程序或 bridge 会话，请结束旧会话后再运行')
        h=c['hardware']
        log=(args.output/'bridge.log').open('w')
        child=subprocess.Popen(['ros2','run','estun_segmented','bridge',
            '--controller-ip',h['controller_ip'],'--feedback-source-ip',h['feedback_source_ip'],
            '--bind-ip',h['bind_ip'],'--feedback-timeout',str(c['motion']['feedback_timeout'])],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        node.read_start()
        devices=ObservationGripper(c,args.output,journal,True)
        devices.node=node
        devices.children=[child]
        devices.connect_gripper()  # query only; refuse serial faults before arm movement
        journal.write('GO_OBSERVATION',target=c['observation_pose'],stage='all')
        # Plan, speed, stage order, readiness, timeout and arrival checks are
        # all implemented by the existing controller, not by the web layer.
        node.move_to(c['observation_pose'],stage='all',execute=True)
        actual=node.read_start().pose
        pe,ae=errors(actual,c['observation_pose'])
        if pe>settings.position_tolerance or ae>settings.angle_tolerance:
            raise RuntimeError('观察位到位复核失败')
        result.update(observation_reached=True,actual=list(actual),position_error_m=pe,angle_error_rad=ae)
        devices.action(False)  # existing completion loop; one open command, never resend
        journal.write('OPEN_COMPLETED',purpose='observation_initial_state')
        result.update(success=True,gripper_open_completed=True)
    except (Exception,KeyboardInterrupt) as exc:
        result.update(success=False,aborted=True,reason=str(exc) or type(exc).__name__)
        journal.write('ABORT',**result)
    finally:
        cleanup=[]
        for action in (lambda:devices.gripper.disconnect() if devices and devices.gripper else None,
                       lambda:node.destroy_node() if node else None,
                       lambda:ros.shutdown() if ros and ros.ok() else None):
            try: action()
            except Exception as exc: cleanup.append(str(exc))
        if child and child.poll() is None:
            for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
                try:
                    os.killpg(child.pid,sig)
                    child.wait(timeout=5)
                    break
                except ProcessLookupError: break
                except subprocess.TimeoutExpired: continue
                except Exception as exc:
                    cleanup.append(str(exc)); break
        if log: log.close()
        if lock: lock.close()
        if cleanup: result.update(success=False,cleanup_error='; '.join(cleanup))
        (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        journal.write('RESULT',**result)
        journal.close()
    return 0 if result['success'] else 2


if __name__=='__main__':
    raise SystemExit(main())
