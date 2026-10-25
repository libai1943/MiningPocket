from pathlib import Path
import json,statistics
R=Path(__file__).resolve().parents[1];M=['BRS','HA','BIAGT','GHA','CC-A*'];rows=[json.loads(x) for x in (R/'results/comparison100.jsonl').read_text().splitlines()];assert len(rows)==500
lookup={(r['case'],r['method']):r for r in rows};common=[i for i in range(1,101) if all(lookup[i,m]['success'] for m in M)]
weight=json.loads((R/'benchmark/parameters.json').read_text())['stop_penalty']
for i in range(1,101):
 valid=[lookup[i,m] for m in M if lookup[i,m]['success']]
 if not valid:continue
 for r in valid:assert abs(r['J']-r['execution_seconds']-weight*r['extra_stops'])<1e-7
 jb=min(r['J'] for r in valid);tb=min(r['planning_seconds'] for r in valid)
 for r in valid:r['gap']=100*(r['J']/jb-1);r['overhead']=100*(r['planning_seconds']/tb-1);r['best']=abs(r['J']-jb)<1e-7
out=[]
for m in M:
 a=[lookup[i,m] for i in range(1,101)];v=[lookup[i,m] for i in common]
 out.append(dict(method=m,success=sum(r['success'] for r in a),common_tasks=len(common),J=statistics.mean(r['J'] for r in v),Tf=statistics.mean(r['execution_seconds'] for r in v),Ns=statistics.mean(r['extra_stops'] for r in v),relative_cost_gap=statistics.mean(r['gap'] for r in v),planning_mean=statistics.mean(r['planning_seconds'] for r in a),planning_median=statistics.median(r['planning_seconds'] for r in a),planning_max=max(r['planning_seconds'] for r in a),best=sum(r.get('best',False) for r in a)))
print(json.dumps(out,indent=2))
