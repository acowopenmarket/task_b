"""Task B state machine. SI base coordinates; adapters never replay commands."""
import copy
import math
import time
import numpy as np
import paths
from coordinate_transform.geometry import RobotConvention, target_components, finite_vector
from estun_segmented.core import Settings, errors, MotionError

POST_GRASP_NEAR_DEPTH_M = .171
CRITERION = '源区域未检测到目标，或识别到的目标距相机深度小于171mm；不代表传感器已验证持物或全过程未滑落'


def review_success_reason(obs):
    """Only a valid post-grasp observation can satisfy either success rule."""
    if obs['presence'] == 'ABSENT':
        return 'SOURCE_TARGET_ABSENT'
    if obs['presence'] == 'PRESENT':
        depth = obs.get('rgbd_candidate', {}).get('depth_m')
        if (isinstance(depth, (int, float)) and not isinstance(depth, bool)
                and math.isfinite(depth) and 0 < depth < POST_GRASP_NEAR_DEPTH_M):
            return 'TARGET_CAMERA_DEPTH_BELOW_171MM'
    return None


def validate(c, live=False):
    missing = []
    required = ['observation_pose', 'safety.clearance_m']
    for key in required:
        value = c
        for part in key.split('.'):
            value = value.get(part) if isinstance(value, dict) else None
        if value is None or value == '':
            missing.append(key)
    if live:
        missing += ['confirmed.'+k for k,v in c['confirmed'].items() if v is not True]
    if missing:
        raise ValueError('现场必填/确认缺失: '+', '.join(missing))
    if c['units'] != 'robot_base_m_rad':
        raise ValueError('仅允许 robot_base_m_rad')
    finite_vector(c['observation_pose'], 6)
    finite_vector(c['flange_to_gripper_m'], 3)
    finite_vector(c['grasp_rpy_rad'], 3)
    Settings(**c['motion'])
    for k in ('max_extra_retries',):
        if type(c[k]) is not int or c[k] < 0:
            raise ValueError(k)
    for k in ('frames','confirm_bursts','min_depth_points'):
        if type(c['vision'][k]) is not int or c['vision'][k] < 1:
            raise ValueError('vision.'+k)
    if c['vision']['frames'] < 3:
        raise ValueError('多帧顶部检查至少3帧')
    for section in (c['vision'], c['safety']):
        for key,value in section.items():
            if isinstance(value, (int,float)) and (not math.isfinite(value) or value <= 0):
                raise ValueError(key+' 必须为有限正数')
    if not math.isfinite(c['max_insert_depth_m']) or c['max_insert_depth_m'] <= 0:
        raise ValueError('max_insert_depth_m')
    if not math.isfinite(c['min_flange_z_m']) or c['min_flange_z_m'] < .205:
        raise ValueError('不得降低原法兰Z下限0.205m')
    workspace=c['safety'].get('workspace_bounds')
    if workspace is not None:
        finite_vector(workspace,6)
        if any(workspace[i] >= workspace[i+1] for i in (0,2,4)):
            raise ValueError('workspace_bounds=[xmin,xmax,ymin,ymax,zmin,zmax]')
    # Observation flange must be high enough even while rotating the offset.
    rim=c['box'].get('rim_z_m')
    if rim is not None and (not math.isfinite(rim)):
        raise ValueError('rim_z_m')
    safe = rim + c['safety']['clearance_m'] + np.linalg.norm(c['flange_to_gripper_m']) if rim is not None else c['min_flange_z_m']
    if c['observation_pose'][2] < max(safe,c['min_flange_z_m']):
        raise ValueError('初始观察位Z不足以在箱口上方安全调姿态/抬升')
    gp=c['hardware']['gripper']; gh=c['hardware']
    if not (0.1 <= gp['speed_rad_s'] <= gp['max_speed_rad_s'] <= 42
            and 0 < gp['travel_rad'] <= math.radians(6553.5)):
        raise ValueError('夹爪行程/速度超出原协议范围')
    if (type(gh['gripper_stable_samples']) is not int or gh['gripper_stable_samples']<2
            or gh['gripper_min_wait_s']<=0 or not math.isfinite(gh['gripper_action_timeout_s'])
            or gh['gripper_action_timeout_s'] <= max(gh['gripper_min_wait_s'],gp['travel_rad']/gp['speed_rad_s'])
                + gh['gripper_stable_samples']/gp['poll_hz']):
        raise ValueError('夹爪完成等待/超时配置不足')


def in_xy(p,b):
    return b[0] <= p[0] <= b[1] and b[2] <= p[1] <= b[3]


def compute_target(obs,c):
    if obs['presence'] != 'PRESENT' or not obs.get('localizable') or not obs.get('top_valid'):
        raise ValueError('目标存在性/定位/顶部估计不满足运动条件')
    original = finite_vector(obs['candidate_base_m'],3).tolist()
    top = float(obs['z_top'])
    if not math.isfinite(top) or top < original[2] - c['vision']['top_temporal_span_m']:
        raise ValueError('顶部与候选点不一致')
    selected=original.copy()
    selected[2]=max(original[2],top-c['max_insert_depth_m'])
    robot=RobotConvention()
    target=target_components(selected,c['grasp_rpy_rad'],robot,
                             c['flange_to_gripper_m'],min_flange_z=c['min_flange_z_m'])
    final=np.array(target['gripper_center_reconstructed_m'])
    target.update(candidate_base_m=original,selected_grasp_base_m=selected,z_top=top,
                  grasp_policy='direct_localization_fixed_orientation',rpy_rad=c['grasp_rpy_rad'],
                  layer1_adjustment_m=selected[2]-original[2],
                  final_reference_delta_from_candidate_m=(final-original).tolist(),
                  capture_id=obs['capture_id'],top_quality=obs.get('top_quality'))
    if target['flange_pose'][2] >= c['observation_pose'][2]:
        raise ValueError('最终法兰Z不低于观察位，不能按规定下降/抬升')
    for p in (target['flange_pose'],c['observation_pose']):
        check_workspace(p,c)
    return target


def check_workspace(p,c):
    finite_vector(p,6)
    bounds=c['safety'].get('workspace_bounds')
    if bounds is not None and not all(bounds[2*i] <= p[i] <= bounds[2*i+1] for i in range(3)):
        raise ValueError('目标超出现场确认工作空间')
    if p[2] < c['min_flange_z_m']:
        raise ValueError('运动低于法兰Z下限')


class Task:
    def __init__(self,c,motion,gripper,vision,journal,clock=time.monotonic):
        self.c,self.m,self.g,self.v,self.j,self.clock=c,motion,gripper,vision,journal,clock
        self.seen=set(); self.used=set(); self.s=Settings(**c['motion'])

    def state(self,name,**data):
        self.j.write(name,**data)

    def move(self,p,stage,name):
        check_workspace(p,self.c)
        self.state(name,target=p,stage=stage)
        self.m.move_to(p,stage=stage,execute=True)
        fb=self.m.read_start()
        pe,ae=errors(fb.pose,p)
        if pe>self.s.position_tolerance or ae>self.s.angle_tolerance:
            raise MotionError('到位复核失败: '+name)

    def observe(self,locate=False):
        deadline=self.clock()+self.c['vision']['timeout_s']; previous=None; count=0
        while self.clock()<deadline:
            self.m.read_start()  # stable + fresh + unique bridge gate before each burst
            obs=self.v.capture(deadline)
            cid=obs['capture_id']
            if cid in self.seen:
                raise ValueError('重复采集ID，禁止复用旧数据')
            self.seen.add(cid)
            age=self.clock()-obs['captured_monotonic']
            if not 0 <= age <= self.c['safety']['max_capture_age_s']:
                raise ValueError('采集数据过期')
            self.state('OBSERVATION',**obs)
            status=obs['presence']
            if status not in ('PRESENT','ABSENT','UNKNOWN'):
                raise ValueError('非法视觉状态')
            if status=='UNKNOWN':
                previous=None; count=0
                continue
            # capture() already validates a multi-frame RGB-D burst. One valid
            # near-target burst suffices only for the post-grasp review.
            if not locate and review_success_reason(obs)=='TARGET_CAMERA_DEPTH_BELOW_171MM':
                return obs
            count=count+1 if status==previous else 1; previous=status
            if count>=self.c['vision']['confirm_bursts']:
                if locate and status=='PRESENT' and not (obs.get('localizable') and obs.get('top_valid')):
                    self.state('PRESENT_CANNOT_LOCALIZE',reason=obs.get('reason'))
                    continue
                return obs
        raise TimeoutError('有效观察/定位超时；UNKNOWN不能成功或盲目重抓')

    def run(self):
        try:
            validate(self.c)
            observation=self.c['observation_pose']
            # Existing segmented all moves XY before Z. Explicit lift first is essential.
            start=list(self.m.read_start().pose)
            if self.c['box'].get('rim_z_m') is None:
                pe,ae=errors(start,observation)
                if pe>self.s.position_tolerance or ae>self.s.angle_tolerance:
                    raise MotionError('自动场景模式须先用示教器到观察位；未观察环境前不自动移位')
            if start[2] < observation[2]:
                start[2]=observation[2]; self.move(start,'z','INITIAL_VERTICAL_LIFT')
            self.move(observation,'all','GO_OBSERVATION')
            self.g.action(False); self.state('OPEN_COMPLETED',purpose='initialize')
            for attempt in range(self.c['max_extra_retries']+1):
                self.state('ATTEMPT',attempt=attempt,extra_retries=attempt)
                obs=self.observe(locate=True)
                if obs['presence']!='PRESENT':
                    raise ValueError('抓取前未确认目标存在；不报告抓取成功')
                target=compute_target(obs,self.c)
                if obs['capture_id'] in self.used or self.clock()-obs['captured_monotonic'] > self.c['safety']['max_capture_age_s']:
                    raise ValueError('抓取数据已用或过期')
                self.used.add(obs['capture_id'])
                self.state('HEIGHT_AND_TARGET',**target)
                final=target['flange_pose']; above=list(final); above[2]=observation[2]
                rotate=list(observation); rotate[3:]=final[3:]
                self.move(rotate,'orientation','ROTATE_ABOVE_RIM')
                self.move(above,'xy','MOVE_ABOVE_TARGET')
                self.move(final,'z','DESCEND')
                self.g.action(True); self.state('CLOSE_COMPLETED',attempt=attempt)
                self.move(above,'z','LIFT_CLOSED')
                back=list(observation); back[3:]=final[3:]
                self.move(back,'xy','RETURN_CLOSED')
                self.move(observation,'orientation','RESTORE_OBSERVATION_CLOSED')
                review=self.observe()
                success_reason=review_success_reason(review)
                if success_reason:
                    self.g.action(False); self.state('OPEN_COMPLETED',purpose='success_release')
                    evidence=dict(success_reason=success_reason,review_capture_id=review['capture_id'],
                                  target_camera_depth_m=review.get('rgbd_candidate',{}).get('depth_m'),
                                  success_depth_threshold_m=POST_GRASP_NEAR_DEPTH_M)
                    self.state('SUCCESS',criterion=CRITERION,attempt=attempt,object_held_verified=False,**evidence)
                    return dict(success=True,attempts=attempt+1,criterion=CRITERION,object_held_verified=False,**evidence)
                self.g.action(False); self.state('OPEN_COMPLETED',purpose='present_before_relocalize')
                if attempt==self.c['max_extra_retries']:
                    self.state('RETRY_LIMIT',attempt=attempt)
                    return {'success':False,'attempts':attempt+1,'reason':'目标仍在，达到重试上限'}
                self.state('RELOCALIZE_REQUIRED',discard_capture=obs['capture_id'])
        except Exception as exc:
            self.state('ABORT',reason=str(exc),note='停止后续发送；不重发运动/夹爪指令')
            return {'success':False,'reason':str(exc),'aborted':True}
