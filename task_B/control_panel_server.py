"""Loopback-only Windows control panel for existing WSL robot programs.

Starting this server never starts hardware. Actions require a same-origin POST,
session token and an idle job slot. --dry-run never starts WSL or devices.
"""
import argparse
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import secrets
import shlex
import subprocess
import sys
import threading
import time
import uuid
import webbrowser

ROOT=Path(__file__).resolve().parent
ACTIONS={'observation':'返回初始观察位','task_b':'执行一次 task_B'}


def wsl_path(path):
    p=Path(path).resolve()
    if not p.drive or len(p.drive)!=2:
        raise ValueError('控制面板需从 Windows 本地磁盘运行')
    return '/mnt/'+p.drive[0].lower()+'/'+p.as_posix()[3:]


def hidden_process_options():
    return {'creationflags':subprocess.CREATE_NO_WINDOW} if sys.platform=='win32' else {}


class Controller:
    def __init__(self,root=ROOT,dry_run=False):
        self.root=Path(root)
        self.dry_run=dry_run
        self.guard=threading.Lock()
        self.log=deque(maxlen=200)
        self.job=None
        self.state_file=self.root/'runs'/('control_panel_last_test.json' if dry_run else 'control_panel_last.json')
        if self.state_file.is_file():
            try:
                self.job=json.loads(self.state_file.read_text(encoding='utf-8'))
                if self.job.get('status') in ('preparing','running'):
                    self.job.update(status='interrupted',reason='上次面板服务中断，请核实设备状态；不会自动重试')
            except (ValueError,OSError): pass

    def append(self,message):
        with self.guard:
            self.log.append({'time':datetime.now().strftime('%H:%M:%S'),'message':str(message)[:3000]})

    def save_locked(self):
        self.state_file.parent.mkdir(parents=True,exist_ok=True)
        temporary=self.state_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.job,ensure_ascii=False,indent=2),encoding='utf-8')
        temporary.replace(self.state_file)

    def snapshot(self):
        try:
            c=json.loads((self.root/'config.json').read_text(encoding='utf-8-sig'))
            config={'observation_pose':c['observation_pose'],'motion':c['motion'],
                    'min_flange_z_m':c['min_flange_z_m'],'max_extra_retries':c['max_extra_retries']}
        except (OSError,ValueError,KeyError) as exc:
            config={'error':str(exc)}
        with self.guard:
            return {'dry_run':self.dry_run,'job':dict(self.job) if self.job else None,
                    'busy':bool(self.job and self.job['status'] in ('preparing','running')),
                    'log':list(self.log),'config':config}

    def start(self,action):
        if action not in ACTIONS: raise ValueError('未知操作')
        with self.guard:
            if self.job and self.job['status'] in ('preparing','running'):
                return None
            self.log.clear()
            self.job={'id':uuid.uuid4().hex,'action':action,'label':ACTIONS[action],
                      'status':'preparing','started_at':datetime.now().astimezone().isoformat(),
                      'reason':'','result':None,'run_dir':''}
            self.save_locked()
            thread=threading.Thread(target=self.work,args=(action,),daemon=False)
            thread.start()
            return dict(self.job)

    def run_checked(self,command,timeout=25):
        p=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            encoding='utf-8',errors='replace',timeout=timeout,**hidden_process_options())
        if p.returncode: raise RuntimeError(p.stdout.strip() or '命令退出码 '+str(p.returncode))
        return p.stdout.strip()

    def ensure_usb(self,devices=('2bc5:0800','1a86:55d4')):
        listing=self.run_checked(['usbipd','list'])
        for device in devices:
            matches=[line for line in listing.splitlines() if device in line.lower()]
            if len(matches)!=1: raise RuntimeError('USB设备缺失或不唯一: '+device)
            line=matches[0]
            if 'Attached' in line: continue
            if 'Not shared' in line:
                raise RuntimeError('USB设备尚未共享，请先按原设备配置共享: '+device)
            self.append('挂载USB设备 '+device)
            self.run_checked(['usbipd','attach','--wsl','--busid',line.split()[0]],timeout=40)

    def work(self,action):
        relay=None
        child=None
        relay_log=None
        run=None
        result=None
        failed=None
        try:
            if self.dry_run:
                self.append('测试模式：不会连接 WSL、机械臂、相机或夹爪')
                with self.guard: self.job['status']='running'
                time.sleep(.7)
                result={'success':True,'mode':'ui_test_only','motion_sent':False}
                return
            if sys.platform!='win32': raise RuntimeError('请在 Windows 中使用启动面板.cmd')
            c=json.loads((self.root/'config.json').read_text(encoding='utf-8-sig'))
            # These are existing communication endpoints, never browser inputs.
            ipaddress.IPv4Address(c['hardware']['controller_ip'])
            probe=self.run_checked(['wsl.exe','--','bash','-lc','hostname -I; ip -4 route show default'])
            lines=probe.splitlines()
            wsl_ip=next(str(ipaddress.IPv4Address(x)) for x in lines[0].split() if '.' in x)
            route=next(line.split()[2] for line in lines if line.startswith('default via '))
            if route!=c['hardware']['feedback_source_ip']:
                raise RuntimeError('WSL网关 '+route+' 与配置反馈源不一致，请核对 config.json')
            if action=='task_b': self.ensure_usb()
            else: self.ensure_usb(('1a86:55d4',))
            run=self.root/'runs'/('panel_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
            run.mkdir(parents=True)
            with self.guard:
                self.job['run_dir']=str(run)
                self.save_locked()
            # Bind failure (an old relay owns UDP9006) must abort, not coexist.
            relay_log=(run/'relay.log').open('w',encoding='utf-8')
            relay=subprocess.Popen([sys.executable,'-u',str(self.root.parent/'estun'/'windows_feedback_relay.py'),
                '--wsl-ip',wsl_ip,'--controller-ip',c['hardware']['controller_ip']],
                stdout=relay_log,stderr=subprocess.STDOUT,**hidden_process_options())
            time.sleep(.5)
            if relay.poll() is not None:
                relay_log.flush()
                raise RuntimeError('反馈转发器启动失败，检查旧会话占用。'+(run/'relay.log').read_text(encoding='utf-8',errors='replace')[-1500:])
            base=wsl_path(self.root)
            output=wsl_path(run/'program')
            if action=='task_b':
                command='cd '+shlex.quote(base)+' && bash run_task_b.sh --execute --output '+shlex.quote(output)
            else:
                command=('cd '+shlex.quote(base)+' && export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1'
                    ' && source ../camera/scripts/handeye_env.sh'
                    ' && python3 control_panel_action.py --config config.json --output '+shlex.quote(output))
            self.append(ACTIONS[action]+'：启动现有程序')
            with self.guard:
                self.job['status']='running'
                self.save_locked()
            with (run/'console.log').open('w',encoding='utf-8') as console:
                child=subprocess.Popen(['wsl.exe','--','bash','-lc',command],stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,encoding='utf-8',errors='replace',bufsize=1,**hidden_process_options())
                for line in child.stdout:
                    console.write(line); console.flush()
                    self.append(line.rstrip())
                code=child.wait()
            path=run/'program'/'result.json'
            if not path.is_file(): raise RuntimeError('程序未生成结果，退出码 '+str(code)+'；查看 console.log')
            result=json.loads(path.read_text(encoding='utf-8-sig'))
            if code!=0 or not result.get('success'):
                failed=result.get('reason') or result.get('cleanup_error') or '程序未报告成功；退出码 '+str(code)
        except Exception as exc:
            failed=str(exc) or type(exc).__name__
            self.append('操作结束：'+failed)
        finally:
            # Never free the action slot while a started motion program is
            # still alive, even if the panel's logging or result handling fails.
            if child and child.poll() is None:
                self.append('等待已启动程序结束；不会并发启动其他操作')
                child.wait()
            if relay:
                try:
                    if relay.poll() is None:
                        relay.terminate(); relay.wait(timeout=5)
                except Exception as exc: failed=(failed or '')+'；转发器清理失败：'+str(exc)
            if relay_log: relay_log.close()
            with self.guard:
                self.job.update(status='failed' if failed else 'succeeded',reason=failed or '',result=result,
                    ended_at=datetime.now().astimezone().isoformat())
                self.save_locked()


class PanelServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,controller):
        self.controller=controller
        self.token=secrets.token_urlsafe(32)
        super().__init__(address,Handler)
        self.origins={f'http://127.0.0.1:{self.server_port}',f'http://localhost:{self.server_port}'}


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args): pass
    def respond(self,code,data,content_type='application/json; charset=utf-8'):
        raw=json.dumps(data,ensure_ascii=False).encode() if isinstance(data,(dict,list)) else data
        self.send_response(code)
        self.send_header('Content-Type',content_type)
        self.send_header('Content-Length',str(len(raw)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.send_header('X-Frame-Options','DENY')
        self.end_headers()
        self.wfile.write(raw)
    def valid_host(self):
        return 'http://'+self.headers.get('Host','') in self.server.origins
    def do_GET(self):
        if not self.valid_host(): return self.respond(403,{'error':'仅允许本机访问'})
        if self.path in ('/','/index.html'):
            return self.respond(200,(ROOT/'control_panel'/'index.html').read_bytes(),'text/html; charset=utf-8')
        if self.path=='/api/state':
            return self.respond(200,dict(self.server.controller.snapshot(),token=self.server.token))
        return self.respond(404,{'error':'未找到'})
    def do_POST(self):
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=1024: raise ValueError('请求长度无效')
            raw=self.rfile.read(size)
        except ValueError as exc:
            return self.respond(400,{'error':str(exc)})
        if (not self.valid_host() or self.headers.get('Origin') not in self.server.origins
                or not secrets.compare_digest(self.headers.get('X-Control-Token',''),self.server.token)):
            return self.respond(403,{'error':'页面会话无效，请刷新本机页面'})
        if self.path!='/api/action': return self.respond(404,{'error':'未找到'})
        try:
            if self.headers.get('Content-Type')!='application/json':
                raise ValueError('请求格式无效')
            body=json.loads(raw)
            job=self.server.controller.start(body['action'])
            if job is None: return self.respond(409,{'error':'已有操作运行中，请等待结束'})
            self.respond(202,{'job':job})
        except (ValueError,KeyError,TypeError) as exc:
            self.respond(400,{'error':str(exc)})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--dry-run',action='store_true',help='仅测试页面，绝不启动设备')
    parser.add_argument('--open-browser',action='store_true')
    args=parser.parse_args()
    controller=Controller(dry_run=args.dry_run)
    try:
        server=PanelServer(('127.0.0.1',args.port),controller)
    except OSError as exc:
        print('无法启动服务，端口可能已占用：'+str(exc),flush=True)
        if args.open_browser:
            webbrowser.open('http://127.0.0.1:'+str(args.port))
        return
    url='http://127.0.0.1:'+str(server.server_port)
    print('Task B control panel: '+url,flush=True)
    print('UI TEST ONLY' if args.dry_run else 'Ready; no hardware starts until a button is clicked.',flush=True)
    if args.open_browser: threading.Timer(.5,lambda:webbrowser.open(url)).start()
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()


if __name__=='__main__': main()
