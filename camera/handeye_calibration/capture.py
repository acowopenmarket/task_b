"""Only ROS subscriptions and keyboard file saves; no publishers or robot sockets."""
import json
import math
import time
import uuid
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo
from codroid_msgs.msg import RobotInfo
from cv_bridge import CvBridge

from coordinate_transform.geometry import rotation_angle, camera_to_base
from .calibration import board_points, detect_board


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def camera_profile(msg):
    return dict(width=msg.width,height=msg.height,k=list(msg.k),d=list(msg.d),
                distortion_model=msg.distortion_model,frame_id=msg.header.frame_id)


class Collector(Node):
    def __init__(self, config, robot, directory, duration=0):
        super().__init__('handeye_readonly_collector')
        self.cfg=config; self.robot=robot; self.out=Path(directory)
        self.out.mkdir(parents=True,exist_ok=True)
        manifest=self.out/'session.json'
        if manifest.exists():
            old=json.loads(manifest.read_text())
            if old['config']!=config or old['convention']!=robot.evidence():
                raise ValueError('Existing session configuration differs; use a new directory')
            self.session_id=old['session_id']
        else:
            self.session_id=str(uuid.uuid4())
            manifest.write_text(json.dumps(dict(session_id=self.session_id,config=config,
                                 convention=robot.evidence()),indent=2))
        self.bridge=CvBridge(); cv2.setNumThreads(1)
        self.feedback=deque(maxlen=500); self.image=None; self.info=None; self.depth=None
        self.images=deque(maxlen=12); self.infos=deque(maxlen=50); self.depths=deque(maxlen=12)
        self.cached_stamp=None; self.detection=None; self.error='Waiting for image, CameraInfo and RobotInfo'
        self.reverse=False; self.quit=False; self.started=time.monotonic(); self.duration=duration
        self.last_status=0.
        self.visual_cache=None; self.visual_key=None
        self.ui_notice=None
        self.worker_result=None; self.worker_error='Waiting for checkerboard detection'
        self.worker_stop=threading.Event()
        roles=[json.loads(p.read_text())['role'] for p in self.out.glob('sample_*.json')]
        self.sample_counts={'train':roles.count('train'),'validation':roles.count('validation')}
        qos=QoSProfile(depth=10)
        self.subs=[self.create_subscription(Image,'/camera/color/image_raw',self.on_image,qos),
                   self.create_subscription(Image,'/camera/depth/image_raw',self.on_depth,qos),
                   self.create_subscription(CameraInfo,'/camera/color/camera_info',self.on_info,qos),
                   self.create_subscription(RobotInfo,'/RobotInfo',self.on_robot,QoSProfile(depth=50))]
        self.create_timer(.2,self.preview)
        self.worker=threading.Thread(target=self.detect_worker,daemon=True)
        self.worker.start()
        print('READ ONLY: s=train, v=held-out, c=reverse corner order, g=confirm current grasp orientation, '
              'p=save fixed-board-center target observation, q=quit',flush=True)
        print('Keep board fixed. s/v confirms corner 0 is the SAME physically marked corner in every view.',flush=True)

    def on_image(self,msg):
        self.image=(msg,time.monotonic()); self.images.append(self.image)
    def on_depth(self,msg):
        self.depth=(msg,time.monotonic()); self.depths.append(self.depth)
    def on_info(self,msg):
        self.info=(msg,time.monotonic()); self.infos.append(self.info)
    def on_robot(self,msg):
        try:
            self.robot.pose_matrix(msg.end_positions)
            self.feedback.append(dict(robot_pose=list(msg.end_positions),robot_stamp=stamp(msg),
                received=time.monotonic(),robot_fault=int(msg.fault_flag),robot_state=msg.state))
        except ValueError as exc: self.error='INVALID_ROBOT_INFO: '+str(exc)

    def image_pair(self):
        if not self.image or not self.info: raise ValueError('NO_IMAGE_OR_CAMERA_INFO')
        # Pair by message stamps, not callback arrival order. CameraInfo may arrive
        # one callback/frame later than the large image, even under reliable QoS.
        pair=None
        images=list(self.images); infos=list(self.infos)
        for candidate,arrived in reversed(images):
            matching=min(infos,key=lambda entry:abs(stamp(entry[0])-stamp(candidate)))
            if abs(stamp(candidate)-stamp(matching[0]))<=self.cfg['max_sync_sec']:
                pair=(candidate,arrived,matching[0]); break
        if pair is None: raise ValueError('IMAGE_INFO_UNSYNCHRONIZED')
        return pair

    def matched(self,pair=None):
        now=time.monotonic()
        image,arrival,info=self.image_pair() if pair is None else pair
        if now-arrival>self.cfg['max_image_age_sec']: raise ValueError('STALE_IMAGE')
        if image.header.frame_id!=self.cfg['camera_frame'] or info.header.frame_id!=image.header.frame_id:
            raise ValueError('CAMERA_FRAME_MISMATCH')
        if abs(stamp(image)-stamp(info))>self.cfg['max_sync_sec']: raise ValueError('IMAGE_INFO_UNSYNCHRONIZED')
        recent=[f for f in self.feedback if now-f['received']<=self.cfg['settle_window_sec']+0.15]
        if len(recent)<10 or recent[-1]['received']-recent[0]['received']<self.cfg['settle_window_sec']*.8:
            raise ValueError('NEED_FRESH_STATIONARY_ROBOT_WINDOW')
        if now-recent[-1]['received']>self.cfg['max_robot_age_sec']: raise ValueError('STALE_ROBOT_INFO')
        if any(f['robot_fault']!=0 or f['robot_state'] not in ('Idle','AutoReady') for f in recent):
            raise ValueError('ROBOT_MOVING_OR_FAULTED: save only after manually stopping')
        last=self.robot.pose_matrix(recent[-1]['robot_pose'])
        for f in recent:
            t=self.robot.pose_matrix(f['robot_pose'])
            if (np.linalg.norm(t[:3,3]-last[:3,3])>self.cfg['settle_translation_m'] or
                rotation_angle(t[:3,:3].T@last[:3,:3])>self.cfg['settle_rotation_rad']):
                raise ValueError('ROBOT_NOT_SETTLED')
        f=min(recent,key=lambda x:abs(x['robot_stamp']-stamp(image)))
        delta=abs(f['robot_stamp']-stamp(image))
        if delta>self.cfg['max_sync_sec']: raise ValueError('IMAGE_ROBOT_UNSYNCHRONIZED')
        return image,info,f,delta

    def detect_worker(self):
        previous=None
        while not self.worker_stop.is_set():
            try:
                image,arrival,info=self.image_pair(); key=(stamp(image),self.reverse)
                if key==previous:
                    self.worker_stop.wait(.02); continue
                previous=key
                bgr=self.bridge.imgmsg_to_cv2(image,'bgr8')
                t,corners,rms=detect_board(bgr,camera_profile(info),self.cfg['board'],key[1])
                if rms>self.cfg['max_reprojection_px']: raise ValueError(f'REPROJECTION_TOO_HIGH: {rms:.3f}px')
                self.worker_result=(image,arrival,info,bgr,t,corners,rms,key[1])
                self.worker_error=''
            except (ValueError,cv2.error) as exc:
                self.worker_error=str(exc); self.worker_result=None
            self.worker_stop.wait(.01)

    def detected(self):
        result=self.worker_result
        if result is None: raise ValueError(self.worker_error or 'WAIT_FOR_DETECTION')
        image,arrival,info,bgr,t,corners,rms,reversed_order=result
        if reversed_order!=self.reverse: raise ValueError('WAIT_FOR_UPDATED_CORNER_ORDER')
        image,info,f,delta=self.matched((image,arrival,info))
        return image,info,f,delta,(bgr,t,corners,rms)

    def save(self,role):
        image,info,f,delta,(bgr,t,corners,rms)=self.detected()
        existing=list(self.out.glob('sample_*.json'))
        current=self.robot.pose_matrix(f['robot_pose'])
        for p in existing:
            old=json.loads(p.read_text())
            other=self.robot.pose_matrix(old['robot_pose'])
            if (np.linalg.norm(current[:3,3]-other[:3,3])<.01 and
                rotation_angle(current[:3,:3].T@other[:3,:3])<math.radians(5)):
                raise ValueError('DUPLICATE_POSE: move manually to a different viewpoint')
        name=f'sample_{time.time_ns()}'
        if not cv2.imwrite(str(self.out/(name+'.png')),bgr):
            raise OSError('IMAGE_WRITE_FAILED: sample was not saved')
        sample=dict(id=name,session_id=self.session_id,data_kind='real_checkerboard',role=role,
            board=self.cfg['board'],camera_frame=self.cfg['camera_frame'],camera_info=camera_profile(info),
            convention=self.robot.evidence(),robot_pose=f['robot_pose'],robot_fault=f['robot_fault'],
            robot_state=f['robot_state'],robot_stamp=f['robot_stamp'],image_stamp=stamp(image),
            sync_error_sec=delta,origin_confirmed=True,corner_order_reversed=self.reverse,
            T_camera_board=t.tolist(),corners_px=corners.reshape(-1,2).tolist(),
            reprojection_rms_px=rms,image_file=name+'.png',
            timestamp_note='RobotInfo stamp is host receive time, not controller hardware time')
        try: sample['depth_board_center']=self.depth_center(image,info,t)
        except ValueError as exc: sample['depth_board_center_error']=str(exc)
        (self.out/(name+'.json')).write_text(json.dumps(sample,indent=2))
        self.sample_counts[role]+=1
        self.ui_notice=(True,time.monotonic(),f'SAVED {role.upper()} #{self.sample_counts[role]:03}',name)
        print(f'SAVED {role}: {name}, reprojection={rms:.3f}px, sync={delta*1000:.1f}ms',flush=True)

    def depth_center(self,image,info,t):
        if not self.depth: raise ValueError('NO_ALIGNED_DEPTH')
        depth,arrival=min(list(self.depths),key=lambda entry:abs(stamp(entry[0])-stamp(image)))
        if time.monotonic()-arrival>.5 or abs(stamp(depth)-stamp(image))>self.cfg['max_sync_sec']:
            raise ValueError('DEPTH_NOT_SYNCHRONIZED')
        if depth.header.frame_id!=image.header.frame_id or (depth.width,depth.height)!=(image.width,image.height):
            raise ValueError('DEPTH_NOT_ALIGNED')
        # Board physical center = [100,70,0] mm relative to first inner corner.
        pboard=np.mean(board_points(self.cfg['board']),axis=0)
        pc=(t@np.r_[pboard,1])[:3]
        k=np.array(info.k).reshape(3,3); d=np.array(info.d)
        px,_=cv2.projectPoints(pc.reshape(1,3),np.zeros(3),np.zeros(3),k,d)
        u,v=np.rint(px.reshape(2)).astype(int)
        if not (3<=u<depth.width-3 and 3<=v<depth.height-3): raise ValueError('TARGET_OUTSIDE_IMAGE')
        raw=self.bridge.imgmsg_to_cv2(depth,'passthrough')
        if depth.encoding=='16UC1': dm=raw.astype(float)*.001
        elif depth.encoding=='32FC1': dm=raw.astype(float)
        else: raise ValueError('UNKNOWN_DEPTH_ENCODING')
        roi=dm[v-3:v+4,u-3:u+4]; valid=roi[np.isfinite(roi)&(roi>0)]
        if valid.size<13: raise ValueError('INVALID_TARGET_DEPTH')
        z=float(np.median(valid))
        ray=cv2.undistortPoints(np.array([[[u,v]]],dtype=float),k,d).reshape(2)
        return dict(point_camera_m=[float(ray[0]*z),float(ray[1]*z),z],pixel=[int(u),int(v)],
                    valid_pixels=int(valid.size),pnp_center_camera_m=pc.tolist(),
                    expected_point_board_m=pboard.tolist(),depth_stamp=stamp(depth))

    def observation(self):
        image,info,f,delta,(_,t,_,_)=self.detected()
        data=self.depth_center(image,info,t)
        result=dict(data_kind='real_fixed_board_center',camera_frame=self.cfg['camera_frame'],
            camera_info=camera_profile(info),point_camera_m=data['point_camera_m'],
            robot_pose_at_capture=f['robot_pose'],robot_stamp=f['robot_stamp'],image_stamp=stamp(image),
            robot_fault=f['robot_fault'],convention=self.robot.evidence(),session_id=self.session_id,
            note='Measured aligned depth at fixed board center; not medical object recognition',
            capture_sync_error_sec=max(delta,abs(data['depth_stamp']-stamp(image))),details=data)
        path=self.out/'target_observation.json'; path.write_text(json.dumps(result,indent=2))
        print('SAVED actual fixed-board depth target observation: '+str(path),flush=True)

    def grasp(self):
        _,_,f,_=self.matched()
        result=dict(grasp_rpy_rad=f['robot_pose'][3:],operator_confirmed_for_calculation=True,
                    motion_verified=False,source='operator pressed g at physically inspected stationary pose',
                    convention=self.robot.evidence())
        path=self.out/'grasp_orientation.json'; path.write_text(json.dumps(result,indent=2))
        print('SAVED chosen grasp orientation (no movement/trajectory validation): '+str(path),flush=True)

    def preview(self):
        if self.duration and time.monotonic()-self.started>=self.duration: self.quit=True
        if not self.image: return
        try:
            detection=self.worker_result
            if detection is not None and time.monotonic()-detection[1]<=self.cfg['max_image_age_sec']:
                preview_image=detection[0]; preview_info=detection[2]
            else:
                preview_image=self.image[0]; preview_info=None
            canvas=self.bridge.imgmsg_to_cv2(preview_image,'bgr8').copy()
            # Visual tracking is independent of RobotInfo readiness. A temporarily
            # unmatched robot packet must not make a valid corner overlay disappear.
            if preview_info is not None:
                try:
                    visible_corners=detection[5]
                    cv2.drawChessboardCorners(canvas,(11,8),visible_corners,True)
                    x,y=np.rint(visible_corners[0,0]).astype(int)
                    cv2.circle(canvas,(int(x),int(y)),12,(0,0,255),3)
                    cv2.putText(canvas,'0',(int(x)+15,int(y)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
                except (ValueError,cv2.error):
                    self.visual_cache=None; self.visual_key=None
            try:
                _,_,_,dt,(_,t,corners,rms)=self.detected()
                self.error=f'READY rms={rms:.3f}px sync={dt*1000:.1f}ms; check physical corner 0'
            except (ValueError,cv2.error) as exc: self.error=str(exc)
            cv2.putText(canvas,self.error[:95],(8,24),cv2.FONT_HERSHEY_SIMPLEX,.46,(0,255,255),1)
            cv2.putText(canvas,'s train | v held-out | c reverse | g grasp | p fixed target | q quit',
                        (8,canvas.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
            count_text=f'TRAIN: {self.sample_counts["train"]} / 15+     VALIDATION: {self.sample_counts["validation"]} / 5+'
            cv2.rectangle(canvas,(0,32),(canvas.shape[1],64),(35,35,35),-1)
            cv2.putText(canvas,count_text,(10,54),cv2.FONT_HERSHEY_SIMPLEX,.58,(255,255,255),1)
            if self.ui_notice and time.monotonic()-self.ui_notice[1]<3.5:
                ok,_,headline,detail=self.ui_notice
                color=(20,180,20) if ok else (0,0,210)
                top=canvas.shape[0]//2-52
                cv2.rectangle(canvas,(8,top),(canvas.shape[1]-8,top+108),color,-1)
                cv2.rectangle(canvas,(2,2),(canvas.shape[1]-3,canvas.shape[0]-3),color,6)
                cv2.putText(canvas,headline[:45],(20,top+36),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
                cv2.putText(canvas,detail[:80],(20,top+69),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                cv2.putText(canvas,count_text,(20,top+94),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            cv2.imshow('Read-only hand-eye: mark physical corner 0',canvas)
            if time.monotonic()-self.last_status>1:
                cv2.imwrite(str(self.out/'live_preview.png'),canvas)
                files=list(self.out.glob('sample_*.json'))
                roles=[json.loads(p.read_text())['role'] for p in files]
                (self.out/'live_status.json').write_text(json.dumps(dict(message=self.error,
                    training=roles.count('train'),validation=roles.count('validation'),
                    latest_robot=self.feedback[-1] if self.feedback else None,
                    elapsed_sec=time.monotonic()-self.started),indent=2))
                self.last_status=time.monotonic()
            key=cv2.waitKey(1)&0xff
            try:
                if key==ord('q'): self.quit=True
                elif key==ord('c'): self.reverse=not self.reverse; self.cached_stamp=None
                elif key==ord('s'): self.save('train')
                elif key==ord('v'): self.save('validation')
                elif key==ord('g'): self.grasp()
                elif key==ord('p'): self.observation()
            except (ValueError,cv2.error,OSError) as exc:
                self.ui_notice=(False,time.monotonic(),'NOT SAVED',str(exc))
                print('SAVE REFUSED: '+str(exc),flush=True)
        except cv2.error as exc:
            print('DISPLAY_ERROR: '+str(exc),flush=True); self.quit=True


def run(config,robot,directory,duration=0):
    rclpy.init(); node=None
    try:
        node=Collector(config,robot,directory,duration)
        while rclpy.ok() and not node.quit: rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt: pass
    finally:
        if node:
            node.worker_stop.set(); node.worker.join(timeout=3)
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok(): rclpy.shutdown()
