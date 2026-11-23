import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
def main():
 manifest=json.loads((root/'SHA256SUMS.json').read_text(encoding='utf-8'));errors=[]
 for r in manifest['files']:
  p=root/r['path']
  if not p.is_file():errors.append('MISSING '+r['path']);continue
  h=hashlib.sha256()
  with p.open('rb') as f:
   for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
  if p.stat().st_size!=r['size_bytes'] or h.hexdigest()!=r['sha256']:errors.append('MISMATCH '+r['path'])
 if errors:raise SystemExit('\n'.join(errors))
 print('归档核验通过：',len(manifest['files']),'个文件。此结果不表示模型训练或现场验收已完成。')
if __name__=='__main__':main()
