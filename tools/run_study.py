from study_common import arguments,Study
import json
args=arguments();mode=args.mode
methods=['BRS','HA','BIAGT','GHA','CC-A*']
settings=[(m,[]) for m in methods] if mode=='comparison' else [('goal_side',['--policy','2']),('start_side',['--policy','3']),('fixed_order',['--fixed-order']),('bounded_refinement',['--refine'])]
if mode=='selection':settings=[('first',['--candidates','1','--improve-seconds','0']),('three',['--candidates','3','--improve-seconds','.1']),('ten',['--candidates','10','--improve-seconds','.25']),('thirty',['--candidates','30','--improve-seconds','.5'])]
target=args.root/'results'/f'{mode}100.jsonl';target.parent.mkdir(exist_ok=True);rows=[]
with Study(args,mode) as study,target.open('w',encoding='utf-8') as out:
 for case in range(1,101):
  order=settings[(case-1)%len(settings):]+settings[:(case-1)%len(settings)]
  for label,extra in order:
   rec=study.query(case,label if mode=='comparison' else 'BRS',extra);rec['configuration']=label;out.write(json.dumps(rec,separators=(',',':'))+'\n');out.flush();rows.append(rec)
   if not rec['success']:print('FAIL',case,label,rec['planning_seconds'],flush=True)
  if case%10==0:print(mode,case,'complete;',sum(x['success'] for x in rows),'/',len(rows),flush=True)
print('DONE',target,flush=True)
