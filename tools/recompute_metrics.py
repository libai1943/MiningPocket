from pathlib import Path
import json,statistics,math
R=Path(__file__).resolve().parents[1]
weight=json.loads((R/'experiment_settings.json').read_text(encoding='utf-8-sig'))['lambda_s']
rows=json.loads((R/'results/recorded/comparison100.json').read_text(encoding='utf-8-sig'))
methods=['BRS','HA','BIAGT','GHA','CC-A*'];ids=sorted({r['id'] for r in rows});assert len(rows)==500 and len(ids)==100
lookup={(r['id'],r['method']):r for r in rows};assert len(lookup)==500
common=[i for i in ids if all(lookup[i,m]['success'] for m in methods)]
for i in ids:
 valid=[lookup[i,m] for m in methods if lookup[i,m]['success']]
 if not valid:continue
 for r in valid:
  value=r['totalTime']+weight*r['internalStops'];assert abs(value-r['J'])<1e-7
 jb=min(r['J'] for r in valid);tb=min(r['totalPlanningSeconds'] for r in valid)
 for r in valid:r.update(qualityGapPercent=100*(r['J']/jb-1),runtimeOverheadPercent=100*(r['totalPlanningSeconds']/tb-1),bestCost=abs(r['J']-jb)<1e-7)
out=[]
for m in methods:
 rr=[lookup[i,m] for i in common];allm=[lookup[i,m] for i in ids]
 d={'method':m,'successes':sum(r['success'] for r in allm),'common_count':len(rr)}
 for k in ['J','totalTime','internalStops','qualityGapPercent','runtimeOverheadPercent']:d['mean_'+k]=statistics.mean(r[k] for r in rr) if rr else None
 d['mean_wall_all_attempts']=statistics.mean(r['totalPlanningSeconds'] for r in allm);d['best_count']=sum(r.get('bestCost',False) for r in allm if r['success']);out.append(d)
(R/'results/reproduced').mkdir(exist_ok=True)
(R/'results/reproduced/metrics100.json').write_text(json.dumps(rows,indent=2))
(R/'results/reproduced/summary.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
