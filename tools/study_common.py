"""Sequential native-query orchestration; Python does not perform planning."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,platform,subprocess,threading,time
import psutil

def arguments(diagnostic=False):
 p=argparse.ArgumentParser()
 if not diagnostic:p.add_argument('mode',choices=['comparison','components','selection'])
 p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--exe',type=Path);p.add_argument('--cpu',type=int);p.add_argument('--audit',type=Path)
 a=p.parse_args();a.root=a.root.resolve();a.audit=a.audit or a.root/'results/environment';a.audit.mkdir(parents=True,exist_ok=True)
 if a.exe is None:
  names=['build/miningpocket.exe','build/Release/miningpocket.exe','build/miningpocket'];a.exe=next((a.root/n for n in names if (a.root/n).exists()),None)
 if a.exe is None or not a.exe.exists():p.error('Build the native executable or supply --exe.')
 a.exe=a.exe.resolve()
 if a.cpu is not None:psutil.Process().cpu_affinity([a.cpu])
 return a

class Study:
 def __init__(self,args,mode):
  self.a=args;self.mode=mode;self.stop=threading.Event();self.load=[]
  root=args.root
  manifest={str(p.relative_to(root)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['src','include','benchmark'] for p in (root/folder).rglob('*') if p.is_file()}
  build=root/'build/build_info.json'
  self.meta=dict(start=datetime.datetime.now().isoformat(),platform=platform.platform(),cpu=platform.processor(),logical_cpus=psutil.cpu_count(),physical_cpus=psutil.cpu_count(logical=False),ram=psutil.virtual_memory().total,threads=1,affinity=psutil.Process().cpu_affinity() if hasattr(psutil.Process(),'cpu_affinity') else None,exe_sha256=hashlib.sha256(args.exe.read_bytes()).hexdigest(),source_manifest=manifest,build=json.loads(build.read_text(encoding='utf-8-sig')) if build.exists() else {'description':'Supply compiler/build identification for measurements.'})
 def __enter__(self):
  (self.a.audit/f'{self.mode}_environment.json').write_text(json.dumps(self.meta,indent=2),encoding='utf-8');self.thread=threading.Thread(target=self.monitor,daemon=True);self.thread.start();return self
 def monitor(self):
  previous={};t0=time.perf_counter();psutil.cpu_percent(None)
  while not self.stop.wait(1):
   now=time.perf_counter();procs=[]
   for proc in psutil.process_iter(['pid','name','cpu_times']):
    try:
     c=proc.info['cpu_times'];cpu=c.user+c.system;old=previous.get(proc.pid,(cpu,now));pct=100*(cpu-old[0])/max(1e-6,now-old[1]);previous[proc.pid]=(cpu,now)
     if pct>.1 and proc.pid:procs.append(dict(pid=proc.pid,name=proc.info['name'],cpu_percent=pct))
    except (psutil.NoSuchProcess,psutil.AccessDenied):pass
   self.load.append(dict(elapsed=now-t0,total_cpu_percent=psutil.cpu_percent(None),processes=sorted(procs,key=lambda x:x['cpu_percent'],reverse=True)[:12]))
 def __exit__(self,*exc):
  self.stop.set();self.thread.join();(self.a.audit/f'{self.mode}_load.json').write_text(json.dumps(self.load,separators=(',',':')),encoding='utf-8')
 def query(self,case,method,extra):
  cp=subprocess.run([str(self.a.exe),'run',str(self.a.root),'--case',str(case),'--method',method]+extra,text=True,capture_output=True,check=True)
  r=json.loads(cp.stdout);r['recorded_at']=datetime.datetime.now().isoformat();return r
