from study_common import arguments,Study
import json
args=arguments(diagnostic=True);target=args.root/'results/diagnostic100.jsonl'
source=[json.loads(x) for x in (args.root/'results/comparison100.jsonl').read_text().splitlines()]
assert len(source)==500
with Study(args,'diagnostic') as study,target.open('w',encoding='utf-8') as out:
 for original in source:
  if original['success']:continue
  case=original['case'];method=original['method'];print('START diagnostic',case,method,flush=True)
  rec=study.query(case,method,['--budget','500','--first-only']);rec['configuration']='first_feasible_diagnostic';rec['main_success']=False;out.write(json.dumps(rec,separators=(',',':'))+'\n');out.flush()
  print('DONE diagnostic',case,method,'success',rec['success'],'seconds',rec['planning_seconds'],flush=True)
print('DONE',target,flush=True)
