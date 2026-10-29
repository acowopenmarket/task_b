"""Panel tests use a dry-run controller only; never connect hardware."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from control_panel_server import Controller, PanelServer


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        (self.root/'config.json').write_text(json.dumps({
            'observation_pose':[.284,.085,.421,-3.076,0,-1.571],
            'motion':{'linear_speed':.05,'angular_speed':.10471975511965977},
            'min_flange_z_m':.213,'max_extra_retries':2,
            'hardware':{'controller_ip':'192.0.2.10','feedback_source_ip':'198.51.100.1'}}))
        self.controller=Controller(self.root,dry_run=True)
        self.server=PanelServer(('127.0.0.1',0),self.controller)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.origin='http://127.0.0.1:'+str(self.server.server_port)
    def tearDown(self):
        deadline=time.monotonic()+5
        while self.controller.snapshot()['busy'] and time.monotonic()<deadline:
            time.sleep(.05)
        self.server.shutdown();self.server.server_close();self.thread.join()
        self.temp.cleanup()
    def request(self,method,path,body=None,extra=None):
        c=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=3)
        headers={'Origin':self.origin,'Content-Type':'application/json','X-Control-Token':self.server.token}
        headers.update(extra or {})
        c.request(method,path,body=json.dumps(body) if body is not None else None,headers=headers)
        r=c.getresponse();raw=r.read();c.close()
        return r.status,raw
    def test_page_load_never_starts_hardware(self):
        with patch('control_panel_server.subprocess.Popen') as start:
            status,body=self.request('GET','/')
            self.assertEqual(status,200)
            self.assertIn('回观察位'.encode(),body)
            self.assertIn('开始 task_B'.encode(),body)
            self.assertIsNone(self.controller.snapshot()['job'])
            start.assert_not_called()
    def test_double_click_and_two_actions_cannot_overlap(self):
        with patch('control_panel_server.subprocess.Popen') as start:
            a,_=self.request('POST','/api/action',{'action':'observation'})
            job=self.controller.snapshot()['job']['id']
            b,_=self.request('POST','/api/action',{'action':'task_b'})
            self.assertEqual((a,b),(202,409))
            self.assertEqual(job,self.controller.snapshot()['job']['id'])
            while self.controller.snapshot()['busy']: time.sleep(.05)
            self.assertEqual(self.controller.snapshot()['job']['status'],'succeeded')
            self.assertFalse(self.controller.snapshot()['job']['result']['motion_sent'])
            start.assert_not_called()
    def test_second_explicit_action_after_completion(self):
        self.request('POST','/api/action',{'action':'observation'})
        while self.controller.snapshot()['busy']: time.sleep(.05)
        code,_=self.request('POST','/api/action',{'action':'task_b'})
        self.assertEqual(code,202)
        self.assertEqual(self.controller.snapshot()['job']['action'],'task_b')
    def test_rejects_cross_origin_missing_token_and_dns_rebinding(self):
        for headers in ({'Origin':'https://untrusted.example'},
                        {'X-Control-Token':''},{'Host':'untrusted.example'}):
            code,_=self.request('POST','/api/action',{'action':'task_b'},headers)
            self.assertEqual(code,403)
        self.assertIsNone(self.controller.snapshot()['job'])
    def test_unknown_action_and_bad_input_never_start(self):
        for body in ({'action':'shell'},{'action':['task_b']},{},[]):
            code,_=self.request('POST','/api/action',body)
            self.assertEqual(code,400)
        self.assertIsNone(self.controller.snapshot()['job'])
    def test_state_survives_refresh_but_test_and_live_are_separate(self):
        self.request('POST','/api/action',{'action':'observation'})
        while self.controller.snapshot()['busy']: time.sleep(.05)
        old=self.controller.snapshot()['job']['id']
        code,raw=self.request('GET','/api/state')
        self.assertEqual(code,200)
        self.assertEqual(json.loads(raw)['job']['id'],old)
        self.assertEqual(Controller(self.root,True).snapshot()['job']['id'],old)
        self.assertIsNone(Controller(self.root,False).snapshot()['job'])
    def test_interrupted_state_does_not_auto_resume(self):
        path=self.root/'runs'/'control_panel_last_test.json'
        path.parent.mkdir()
        path.write_text(json.dumps({'status':'running','action':'task_b','id':'old'}))
        with patch('control_panel_server.subprocess.Popen') as start:
            loaded=Controller(self.root,True).snapshot()
            self.assertEqual(loaded['job']['status'],'interrupted')
            self.assertFalse(loaded['busy'])
            start.assert_not_called()

    def test_real_dispatch_uses_original_task_and_reports_failure_without_retry(self):
        self.check_dispatch('task_b',False)

    def test_observation_dispatch_is_separate_from_gripper_task(self):
        self.check_dispatch('observation',True)

    def check_dispatch(self,action,success):
        controller=Controller(self.root,False)
        commands=[]
        class FakeProcess:
            def __init__(self,code=None):
                self.code=code;self.stdout=iter(['{"event":"RESULT"}\n'])
            def poll(self): return self.code
            def wait(self,timeout=None): return self.code
            def terminate(self): self.code=-15
        relay=FakeProcess()
        def process(command,**kwargs):
            commands.append(command)
            if command[0]!='wsl.exe': return relay
            folder=Path(controller.job['run_dir'])/'program';folder.mkdir()
            (folder/'result.json').write_text(json.dumps({'success':success,'reason':'' if success else '原程序拒绝：Idle'}))
            return FakeProcess(0 if success else 2)
        with patch('control_panel_server.sys.platform','win32'), \
             patch.object(controller,'run_checked',return_value='198.51.100.2\ndefault via 198.51.100.1 dev eth0'), \
             patch.object(controller,'ensure_usb') as usb, \
             patch('control_panel_server.subprocess.Popen',side_effect=process):
            controller.start(action)
            deadline=time.monotonic()+5
            while controller.snapshot()['busy'] and time.monotonic()<deadline: time.sleep(.02)
        job=controller.snapshot()['job']
        self.assertEqual(job['status'],'succeeded' if success else 'failed')
        if not success:self.assertEqual(job['reason'],'原程序拒绝：Idle')
        self.assertEqual(len(commands),2)  # one relay, one program, no retries
        self.assertEqual(relay.poll(),-15)
        command=commands[1][-1]
        if action=='task_b':
            self.assertIn('bash run_task_b.sh --execute',command);usb.assert_called_once()
        else:
            self.assertIn('python3 control_panel_action.py',command)
            self.assertNotIn('run_task_b.sh',command);usb.assert_called_once_with(('1a86:55d4',))


if __name__=='__main__': unittest.main(verbosity=2)
