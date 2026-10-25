from pathlib import Path
import hashlib,json,zipfile
R=Path(__file__).resolve().parents[1]
manifest=json.loads((R/'RELEASE_SHA256.json').read_text())
for name,digest in manifest.items():assert hashlib.sha256((R/name).read_bytes()).hexdigest()==digest,name
cases=json.loads((R/'benchmark/manifest.json').read_text());assert len(cases)==100
with zipfile.ZipFile(R/'benchmark/cases.zip') as z:
 assert len(z.namelist())==100
 for row in cases:
  name=f"case_{row['id']:03d}.json";data=z.read(name);assert hashlib.sha256(data).hexdigest()==row['json_sha256']
  if (R/'benchmark'/name).exists():assert (R/'benchmark'/name).read_bytes()==data
rows=[json.loads(x) for x in (R/'results/comparison100.jsonl').read_text().splitlines()];assert len(rows)==500
for m in ['BRS','HA','BIAGT','GHA','CC-A*']:
 rr=[r for r in rows if r['method']==m];assert {r['case'] for r in rr}==set(range(1,101));assert all(r['planning_seconds']<=10 for r in rr);print(m,sum(r['success'] for r in rr),'/ 100')
print('PASS: release hashes, exact 100 tasks, and complete 10-second comparison records.')
