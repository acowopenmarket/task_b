"""One camera and one bridge per session; repeat subscription capture after return.

Importing this file performs no hardware operations. Linux/WSL ROS2 only.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import uuid
import numpy as np
import cv2
import paths
from core import check_workspace
from vision import analyze, tag, unknown
from capture_sync import match_buffered


def resolve(value):
    p=Path(value)
    return p if p.is_absolute() else paths.ROOT/p


def save(path,data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


class Devices:
    def __init__(self,c,run,journal,execute=False):
        self.c,self.run,self.j,self.execute=c,run,journal,execute
        self.children=[]; self.logs=[]; self.node=None; self.ros=None; self.gripper=None; self.lock=None
        self.source_context=None  # this task only; never loaded from disk

    def start(self):
        if sys.platform!='linux':
            raise RuntimeError('设备模式须在已配置ROS2的Linux/WSL中运行')
        import fcntl
        self.lock=open('/tmp/estun_task_b.lock','w')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        import rclpy
        from estun_segmented.runner import make_node
        from estun_segmented.core import Settings
        from sensor_msgs.msg import Image, CameraInfo
        from codroid_msgs.msg import RobotInfo
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from cv_bridge import CvBridge
        self.ros=rclpy; rclpy.init(args=[])
        self.node=make_node(self.j,Settings(**self.c['motion']))
        self.cv=CvBridge(); self.buffers={k:deque(maxlen=100) for k in ('color','depth','ci','di','robot')}
        self.subs=[]
        topics=self.c['hardware']['topics']
        for key,typ in [('color',Image),('depth',Image),('ci',CameraInfo),('di',CameraInfo),('robot',RobotInfo)]:
            # BEST_EFFORT subscription accepts reliable and sensor-data publishers.
            self.subs.append(self.node.create_subscription(typ,topics[key],
                lambda msg,k=key:self.buffers[k].append((msg,time.monotonic())),
                QoSProfile(depth=40,reliability=ReliabilityPolicy.BEST_EFFORT)))
        deadline=time.monotonic()+1.
        while time.monotonic()<deadline:
            self.node.spin()
        if any(self.node.count_publishers(topics[k]) for k in topics) or self.node.count_publishers('/estun_bridge/status'):
            raise RuntimeError('已有相机/反馈bridge发布者；请先结束旧会话，避免重复驱动或bridge')
        if self.node.count_publishers('/RobotMove')!=1 or self.node.publisher.get_subscription_count()!=0:
            raise RuntimeError('已有运动发布者或bridge订阅者')
        h=self.c['hardware']
        commands=[('camera',['bash',str(resolve(h['camera_script']))]),
                  ('bridge',['ros2','run','estun_segmented','bridge',
                    '--controller-ip',h['controller_ip'],'--feedback-source-ip',h['feedback_source_ip'],
                    '--bind-ip',h['bind_ip'],'--feedback-timeout',str(self.c['motion']['feedback_timeout'])])]
        if not self.execute:
            commands[-1][1].append('--receive-only')
        for name,command in commands:
            log=(self.run/(name+'.log')).open('w'); self.logs.append(log)
            self.children.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        self.node.read_start()
        self.wait_camera_ready()
        if self.execute:
            self.connect_gripper()
        return self

    def connect_gripper(self):
        if not self.execute:
            raise RuntimeError('只读模式不能连接执行夹爪')
        import fcntl
        h=self.c['hardware']
        sys.path.insert(0,str(paths.PROJECT/'jia/ros2_ws/src/ms42_gripper'))
        from ms42_gripper.client import GripperClient
        import serial
        p=h['gripper']
        if any('ms42_gripper' in name for name,namespace in self.node.get_node_names_and_namespaces()):
            raise RuntimeError('已有夹爪ROS节点；任务B直接复用串口驱动，禁止并行占用')
        serial_path=Path(h['gripper_port']).resolve(strict=True)
        # Check same-user tty holders in addition to advisory flock and TIOCEXCL.
        # The legacy node did not request kernel exclusivity on its own opens.
        for fd in Path('/proc').glob('[0-9]*/fd/*'):
            try:
                if fd.resolve()==serial_path:
                    raise RuntimeError('夹爪串口已被进程占用: '+str(fd))
            except (PermissionError,FileNotFoundError,OSError):
                continue
        # Kernel exclusive open; refuse an already opened tty. Do not run jia node too.
        def exclusive_serial(**kwargs):
            import termios
            port=serial.Serial(exclusive=True,**kwargs)
            try:
                fcntl.ioctl(port.fileno(),termios.TIOCEXCL)
            except Exception:
                port.close(); raise
            return port
        self.gripper=GripperClient(port=h['gripper_port'],baudrate=p['baudrate'],motor_id=p['motor_id'],
            subdivision=p['subdivision'],max_angle_deg=math.degrees(p['travel_rad']),
            max_speed_rad_s=p['max_speed_rad_s'],feedback_timeout=p['feedback_timeout_s'],
            dry_run=False,serial_factory=exclusive_serial)
        self.gripper.connect()
        self.gripper.query()  # before any arm motion

    def wait_camera_ready(self):
        deadline=time.monotonic()+max(15.,self.c['vision']['capture_timeout_s'])
        topics=self.c['hardware']['topics']
        while time.monotonic()<deadline:
            self.healthy(camera=False)
            if self.children[0].poll() is not None:
                raise RuntimeError('相机进程在初始化期间退出')
            counts=[self.node.count_publishers(topics[k]) for k in ('color','depth','ci','di')]
            if any(n>1 for n in counts):
                raise RuntimeError('相机主题有重复发布者')
            if all(n==1 for n in counts) and all(self.buffers[k] for k in ('color','depth','ci','di')):
                return
        raise TimeoutError('相机初始化等待超时，未收到唯一的RGB-D与内参')

    def healthy(self,camera=True):
        monitored=self.children if camera else self.children[1:]
        if any(p.poll() is not None for p in monitored):
            raise RuntimeError('相机或bridge子进程已退出' if camera else 'bridge子进程已退出')
        self.node.spin(); self.node.current().check()
        if not self.node.connected():
            raise RuntimeError('反馈/通信/运动发布者不唯一')
        for k,t in self.c['hardware']['topics'].items():
            if not camera and k!='robot': continue
            if self.node.count_publishers(t)!=1:
                raise RuntimeError('主题发布者不唯一: '+t)

    def action(self,close):
        if not self.execute or self.gripper is None:
            raise RuntimeError('只读模式不能发送夹爪指令')
        self.healthy(); self.node.read_start()
        p=self.c['hardware']['gripper']; h=self.c['hardware']
        baseline=self.gripper.query()
        self.j.write('GRIPPER_SEND',close=close,travel_rad=p['travel_rad'],
                     speed_rad_s=p['speed_rad_s'],baseline_raw_angle=baseline.raw_angle)
        self.gripper.move(close,math.degrees(p['travel_rad']),p['speed_rad_s'])  # boundary conversion, once
        sent=time.monotonic(); deadline=sent+h['gripper_action_timeout_s']
        wait=max(h['gripper_min_wait_s'],p['travel_rad']/p['speed_rad_s'])
        stable=0; previous=None
        while time.monotonic()<deadline:
            self.healthy()
            fb=self.gripper.query()  # new serial query, parser flushes old frames
            self.j.write('GRIPPER_FEEDBACK',close=close,target_reached=fb.target_reached,
                         raw_speed=fb.raw_speed,raw_angle=fb.raw_angle)
            done=time.monotonic()-sent>=wait and fb.target_reached and fb.raw_speed==0
            stable=stable+1 if done and previous==fb.raw_angle else (1 if done else 0)
            previous=fb.raw_angle
            if stable>=h['gripper_stable_samples']:
                self.j.write('GRIPPER_MOTOR_COMPLETE',close=close,note='真实电机反馈稳定；不验证持物')
                return
            # Spin while waiting; keep bridge feedback fresh and fault checks live.
            next_poll=time.monotonic()+1/p['poll_hz']
            while time.monotonic()<next_poll:
                self.healthy()
        raise TimeoutError('夹爪真实动作完成反馈超时；不重发，不继续运动')

    def capture(self,outer_deadline=None):
        # Data/decode failures are UNKNOWN, never a negative detection. Motion
        # faults from healthy()/read_start() propagate and latch the task abort.
        try:
            return self._capture(outer_deadline)
        except (ValueError,KeyError,TypeError,cv2.error) as exc:
            return tag(unknown('CAPTURE_DATA_ERROR: '+str(exc)))

    def analyze_with_feedback(self,frames,rgbs,records,calibration):
        # Image processing can take longer than the feedback freshness limit.
        # Only computation runs in the worker; ROS callbacks and all device
        # checks stay on the calling thread. Never extend the feedback timeout.
        with ThreadPoolExecutor(max_workers=1) as worker:
            pending=worker.submit(analyze,frames,rgbs,records,calibration,self.c,
                                  source_context=self.source_context)
            while not pending.done():
                self.healthy(camera=False)
            self.healthy(camera=False)
            return pending.result()

    def _capture(self,outer_deadline=None):
        from handeye_calibration.capture import stamp,camera_profile
        from vision_center.grasp_candidate import aligned_profiles
        self.healthy(camera=False)
        from estun_segmented.core import errors,Settings
        fb=self.node.read_start(); settings=Settings(**self.c['motion'])
        pe,ae=errors(fb.pose,self.c['observation_pose'])
        if pe>settings.position_tolerance or ae>settings.angle_tolerance:
            return tag(unknown('NOT_AT_CONFIGURED_OBSERVATION_POSE'))
        # Discard all buffered frames from travel and previous captures.
        for q in self.buffers.values(): q.clear()
        start=time.monotonic(); cfg=self.c['vision']
        deadline=min(outer_deadline or float('inf'),start+cfg['capture_timeout_s'])
        frames=[]; rgbs=[]; records=[]; seen=set(); last_reason='NO_SYNCHRONIZED_BURST'
        sync_checks=[]
        while time.monotonic()<deadline:
            self.healthy(camera=False)
            topics=self.c['hardware']['topics']
            counts=[self.node.count_publishers(topics[k]) for k in ('color','depth','ci','di')]
            if self.children[0].poll() is not None or any(n>1 for n in counts):
                last_reason='CAMERA_UNAVAILABLE_OR_DUPLICATE'; break
            if any(n==0 for n in counts):
                last_reason='CAMERA_TOPICS_NOT_READY'; continue
            if not all(self.buffers.values()): continue
            bundle,diagnostic=match_buffered(self.buffers,('depth','ci','di','robot'),seen,time.monotonic(),cfg)
            if diagnostic is not None: sync_checks.append(diagnostic)
            if bundle is None:
                last_reason='STALE_OR_UNSYNCHRONIZED'; continue
            image,pairs,span=bundle
            depth,ci,di,robot=[pairs[k][0] for k in ('depth','ci','di','robot')]
            info=camera_profile(ci); dinfo=camera_profile(di)
            if (not aligned_profiles(info,dinfo) or len({m.header.frame_id for m in (image,depth,ci,di)})!=1
                    or len({(m.width,m.height) for m in (image,depth,ci,di)})!=1):
                last_reason='PROFILE_ALIGNMENT_MISMATCH'; continue
            seen.add(stamp(image))
            frames.append(self.cv.imgmsg_to_cv2(depth,'passthrough').copy())
            rgbs.append(self.cv.imgmsg_to_cv2(image,'bgr8').copy())
            records.append(dict(camera_info=info,depth_camera_info=dinfo,camera_frame=image.header.frame_id,
                robot_pose_at_capture=list(robot.end_positions),robot_fault=int(robot.fault_flag),robot_state=robot.state,
                image_stamp=stamp(image),robot_stamp=stamp(robot),depth_stamp=stamp(depth),
                capture_sync_error_sec=span,depth_encoding=depth.encoding))
            if len(frames)>=cfg['frames']:
                break
        cid=str(uuid.uuid4()); out=self.run/('capture_'+cid); out.mkdir()
        save(out/'records.json',records)
        save(out/'sync_diagnostics.json',dict(checks=len(sync_checks),
            best=min(sync_checks,key=lambda x:x['span_s']) if sync_checks else None,
            last=sync_checks[-1] if sync_checks else None))
        if len(frames)<cfg['frames']:
            result=tag(unknown(last_reason),cid,start)
        else:
            np.save(out/'depth.npy',np.stack(frames)); np.save(out/'rgb.npy',np.stack(rgbs))
            cv2.imwrite(str(out/'color.png'),rgbs[-1])
            calibration=json.loads(resolve(self.c['hardware']['calibration_path']).read_text(encoding='utf-8'))
            result,debug=self.analyze_with_feedback(frames,rgbs,records,calibration)
            context=result.pop('source_context',None)
            if context is not None and result.get('localizable'):
                self.source_context=context
                save(out/'task_source_context.json',context)
            for name,mask in debug.items(): cv2.imwrite(str(out/(name+'.png')),mask*255)
            result=tag(result,cid,start)
        result['evidence_dir']=str(out); save(out/'observation.json',result)
        return result

    def close(self):
        cleanup_errors=[]
        for action in (
            lambda:self.gripper.disconnect() if self.gripper is not None else None,
            lambda:self.node.destroy_node() if self.node is not None else None,
            lambda:self.ros.shutdown() if self.ros is not None and self.ros.ok() else None):
            try: action()
            except Exception as exc: cleanup_errors.append(str(exc))
        for child in reversed(self.children):
            try:
                if child.poll() is None:
                    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
                        try: os.killpg(child.pid,sig)
                        except ProcessLookupError: break
                        try:
                            child.wait(timeout=5); break
                        except subprocess.TimeoutExpired: continue
            except Exception as exc: cleanup_errors.append(str(exc))
        for log in self.logs:
            try: log.close()
            except Exception as exc: cleanup_errors.append(str(exc))
        if self.lock:
            try: self.lock.close()
            except Exception as exc: cleanup_errors.append(str(exc))
        if cleanup_errors: raise RuntimeError('会话清理异常: '+'; '.join(cleanup_errors))


class CameraOnly(Devices):
    """RGB-D recognition without robot topics, calibration, bridge or serial."""
    def start(self):
        if sys.platform!='linux':
            raise RuntimeError('相机模式须在已配置ROS2的Linux/WSL中运行')
        import fcntl
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from sensor_msgs.msg import Image,CameraInfo
        from cv_bridge import CvBridge
        self.lock=open('/tmp/estun_task_b.lock','w')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.ros=rclpy; rclpy.init(args=[])
        self.node=Node('task_b_camera_only')
        self.dispatcher=SingleThreadedExecutor(context=self.node.context)
        self.dispatcher.add_node(self.node)
        self.cv=CvBridge()
        self.buffers={k:deque(maxlen=100) for k in ('color','depth','ci','di')}
        self.subs=[]
        for key,typ in [('color',Image),('depth',Image),('ci',CameraInfo),('di',CameraInfo)]:
            self.subs.append(self.node.create_subscription(typ,self.c['hardware']['topics'][key],
                lambda msg,k=key:self.buffers[k].append((msg,time.monotonic())),
                QoSProfile(depth=40,reliability=ReliabilityPolicy.BEST_EFFORT)))
        deadline=time.monotonic()+1
        while time.monotonic()<deadline: self.spin_camera()
        if any(self.node.count_publishers(self.c['hardware']['topics'][k]) for k in self.buffers):
            raise RuntimeError('已有相机发布者；请先结束旧相机会话，避免重复驱动')
        log=(self.run/'camera.log').open('w'); self.logs.append(log)
        self.children.append(subprocess.Popen(['bash',str(resolve(self.c['hardware']['camera_script']))],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+max(15.,self.c['vision']['capture_timeout_s'])
        while time.monotonic()<deadline:
            self.healthy()
            if all(self.buffers[k] for k in self.buffers): return self
        raise TimeoutError('相机初始化等待超时，未收到唯一的RGB-D与内参')

    def spin_camera(self):
        self.dispatcher.spin_once(timeout_sec=.02)
        for _ in range(2*len(self.subs)+4):
            self.dispatcher.spin_once(timeout_sec=0.)

    def healthy(self,camera=True):
        self.spin_camera()
        if any(p.poll() is not None for p in self.children):
            raise RuntimeError('相机子进程已退出')
        if any(self.node.count_publishers(self.c['hardware']['topics'][k])>1 for k in self.buffers):
            raise RuntimeError('相机主题有重复发布者')

    def _capture(self,outer_deadline=None):
        from handeye_calibration.capture import stamp,camera_profile
        from vision_center.grasp_candidate import aligned_profiles
        from rgbd_candidate import locate
        for q in self.buffers.values(): q.clear()
        cfg=self.c['vision']; start=time.monotonic()
        deadline=min(outer_deadline or float('inf'),start+cfg['capture_timeout_s'])
        frames=[]; rgbs=[]; records=[]; seen=set(); checks=[]
        reason='NO_SYNCHRONIZED_RGBD_BURST'
        while time.monotonic()<deadline:
            self.healthy()
            bundle,diagnostic=match_buffered(self.buffers,('depth','ci','di'),seen,time.monotonic(),cfg)
            if diagnostic is not None: checks.append(diagnostic)
            if bundle is None: continue
            image,pairs,span=bundle
            depth,ci,di=[pairs[k][0] for k in ('depth','ci','di')]
            info=camera_profile(ci); dinfo=camera_profile(di)
            if (not aligned_profiles(info,dinfo) or len({m.header.frame_id for m in (image,depth,ci,di)})!=1
                    or len({(m.width,m.height) for m in (image,depth,ci,di)})!=1):
                reason='PROFILE_ALIGNMENT_MISMATCH'; continue
            seen.add(stamp(image))
            frames.append(self.cv.imgmsg_to_cv2(depth,'passthrough').copy())
            rgbs.append(self.cv.imgmsg_to_cv2(image,'bgr8').copy())
            records.append(dict(camera_info=info,depth_camera_info=dinfo,depth_encoding=depth.encoding,
                camera_frame=image.header.frame_id,image_stamp=stamp(image),depth_stamp=stamp(depth),
                capture_sync_error_sec=span))
            if len(frames)>=cfg['frames']: break
        cid=str(uuid.uuid4()); out=self.run/('capture_'+cid); out.mkdir()
        save(out/'records.json',records)
        save(out/'sync_diagnostics.json',dict(checks=len(checks),
            best=min(checks,key=lambda x:x['span_s']) if checks else None,
            last=checks[-1] if checks else None))
        result=unknown(reason)
        if len(frames)>=cfg['frames']:
            np.save(out/'depth.npy',np.stack(frames)); np.save(out/'rgb.npy',np.stack(rgbs))
            cv2.imwrite(str(out/'color.png'),rgbs[-1])
            if any(r['camera_info']!=records[-1]['camera_info'] or
                   r['depth_encoding']!=records[-1]['depth_encoding'] for r in records):
                raise ValueError('PROFILE_CHANGED_DURING_CAPTURE')
            if any(np.mean(np.max(rgb,axis=-1)<10)>.9 or np.mean(np.min(rgb,axis=-1)>250)>.9 for rgb in rgbs):
                raise ValueError('RGB_BLACKOUT_OR_OVEREXPOSURE')
            diagnostics={}
            candidate,debug,reason=locate(frames,records[-1]['depth_encoding'],records[-1]['camera_info'],rgbs[-1],cfg,
                diagnostics=diagnostics)
            save(out/'candidate_diagnostics.json',diagnostics)
            if candidate is not None:
                result=dict(presence='PRESENT',localizable=True,reason=reason,
                    coordinate_frame='camera_optical_m',rgbd_candidate=candidate,
                    point_camera_m=candidate['point_camera_m'],pixel=candidate['pixel'],
                    object_bbox=candidate['object_bbox'],rgb_depth_iou=candidate['rgb_depth_iou'])
            else: result=unknown(reason,failure_detail=diagnostics.get('failure_detail'))
            for name,mask in debug.items(): cv2.imwrite(str(out/(name+'.png')),mask*255)
        result=tag(dict(result,robot_feedback_required=False,motion_sent=False),cid,start)
        result['evidence_dir']=str(out); save(out/'observation.json',result)
        return result

    def close(self):
        try:
            if getattr(self,'dispatcher',None) is not None:
                if self.node is not None: self.dispatcher.remove_node(self.node)
                self.dispatcher.shutdown(timeout_sec=0.)
        finally:
            super().close()


class Motion:
    def __init__(self,devices): self.d=devices
    def read_start(self):
        self.d.healthy(camera=False)
        return self.d.node.read_start()
    def move_to(self,p,stage='all',execute=False):
        if not self.d.execute or not execute: raise RuntimeError('只读模式禁止运动')
        self.d.healthy(); check_workspace(p,self.d.c)
        return self.d.node.move_to(p,stage=stage,execute=True)
