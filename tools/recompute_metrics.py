from pathlib import Path
import json,statistics
P=Path(__file__).resolve().parents[1]
weight=json.loads((P/'experiment_settings.json').read_text(encoding='utf-8-sig'))['lambda_s']
rows=json.loads((P/'results/recorded/comparison100.json').read_text(encoding='utf-8-sig'))
methods=['Ours','HA','BIAGT','GHA'];ids=sorted({r['id'] for r in rows})
assert len(rows)==400 and len(ids)==100
for r in rows:
 r['extraStops']=int(r['cusps']+r['steeringStops']);r['J']=r['totalTime']+weight*r['extraStops']
for i in ids:
 rr=[r for r in rows if r['id']==i];ok=[r for r in rr if r['success']];assert len(ok)==4
 jb=min(r['J'] for r in ok);tb=min(r['totalPlanningSeconds'] for r in ok)
 for r in rr:r.update(bestJ=jb,qualityGapPercent=100*(r['J']/jb-1),runtimeOverheadPercent=100*(r['totalPlanningSeconds']/tb-1))
out=[]
for m in methods:
 rr=[r for r in rows if r['method']==m];s=dict(method=m,n=len(rr),successes=sum(r['success'] for r in rr))
 for k in ['J','extraStops','totalTime','cusps','steeringStops','qualityGapPercent','runtimeOverheadPercent','totalPlanningSeconds']:s['mean_'+k]=statistics.mean(r[k] for r in rr)
 s['bestCount']=sum(r['qualityGapPercent']<1e-8 for r in rr);s['medianGap']=statistics.median(r['qualityGapPercent'] for r in rr);out.append(s)
(P/'results/reproduced').mkdir(exist_ok=True)
(P/'results/reproduced/metrics100.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
(P/'results/reproduced/summary.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))

expected=json.loads((P/'results/recorded/summary.json').read_text())
for a,b in zip(out,expected):
 assert a['method']==b['method']
 for k in b:
  if isinstance(b[k],(int,float)): assert abs(a[k]-b[k]) < 1e-8,(k,a[k],b[k])
print('PASS: all recorded aggregate metrics reproduced.')
