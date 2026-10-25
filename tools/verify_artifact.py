from pathlib import Path
import hashlib,json,zipfile
R=Path(__file__).resolve().parents[1]
spec=json.loads((R/'qa/INPUT_RESULT_SHA256.json').read_text());seen=set()
for archive,members in spec['archives'].items():
 with zipfile.ZipFile(R/archive) as z:
  assert sorted(z.namelist())==sorted(members)
  for name,v in members.items():
   data=z.read(name);assert len(data)==v['bytes'] and hashlib.sha256(data).hexdigest()==v['sha256'],(archive,name)
   seen.add((archive,name))
for name,sha in spec['files'].items():assert hashlib.sha256((R/name).read_bytes()).hexdigest()==sha,name
rows=json.loads((R/'results/recorded/comparison100.json').read_text());assert len(rows)==500
for method in ['BRS','HA','BIAGT','GHA','CC-A*']:
 rr=[r for r in rows if r['method']==method];assert len(rr)==100
 print(method,':',sum(r['success'] for r in rr),'/ 100')
print('PASS: archive members and source hashes verified; failure records retained.')
