from pathlib import Path
import hashlib,json,zipfile
root=Path(__file__).resolve().parents[1]
expected={r['path']:r for r in json.loads((root/'qa/INPUT_RESULT_SHA256.json').read_text())}
seen=set()
archives=[root/'benchmark/cases.zip',*sorted((root/'results/recorded').glob('*.zip'))]
for archive in archives:
    prefix='benchmark/' if archive.name=='cases.zip' else 'results/recorded/'
    with zipfile.ZipFile(archive) as z:
        for name in z.namelist():
            if name.endswith('/'): continue
            path=prefix+name
            assert path in expected,('Unexpected archive member',path)
            data=z.read(name); row=expected[path]
            assert len(data)==row['bytes'] and hashlib.sha256(data).hexdigest()==row['sha256'],path
            assert path not in seen,path
            seen.add(path)
assert seen==set(expected),(len(seen),len(expected))
assert len(seen)==500
rows=json.loads((root/'results/recorded/comparison100.json').read_text())
assert len(rows)==400
for method in ['Ours','HA','BIAGT','GHA']:
    rr=[r for r in rows if r['method']==method]
    assert len(rr)==100 and all(r['success'] for r in rr),method
print('PASS: 100 exact inputs and 400 exact planning outputs; all archive hashes match.')
