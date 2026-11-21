"""Recreate selected unchanged video frames and reviewed YOLO labels.

Input is the author's five original videos, obtained separately from GitHub.
Frame/label manifests are versioned; source-video checksums are checked first.
"""
from pathlib import Path
import argparse,json,hashlib
from collections import defaultdict
import cv2
ROOT=Path(__file__).resolve().parents[1]
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--raw',type=Path,required=True)
    p.add_argument('--output',type=Path,default=Path('local_data'));args=p.parse_args()
    integrity=json.loads((ROOT/'dataset/video_integrity.json').read_text(encoding='utf-8'))
    for clip in integrity['clips']:
        if sha(args.raw/clip['file'])!=clip['sha256']:raise ValueError('Source video hash mismatch: '+clip['file'])
    for kind in ('ball','marker'):
        manifest=json.loads((ROOT/f'dataset/{kind}_manifest.json').read_text(encoding='utf-8'))
        rows=manifest['reviewed'] if kind=='ball' else manifest['images'];groups=defaultdict(list)
        for row in rows:groups[row.get('source_file',row.get('source'))].append(row)
        for filename,group in groups.items():
            cap=cv2.VideoCapture(str(args.raw/filename))
            try:
                for row in sorted(group,key=lambda r:r['source_frame_index_0based']):
                    cap.set(cv2.CAP_PROP_POS_FRAMES,row['source_frame_index_0based']);ok,frame=cap.read()
                    if not ok:raise RuntimeError('Cannot decode '+filename)
                    dest=args.output/kind/row['image'];dest.parent.mkdir(parents=True,exist_ok=True)
                    cv2.imencode('.png',frame)[1].tofile(str(dest))
                    # Decoding libraries can change compressed PNG bytes; raw BGR hash is checked if available.
                    rawhash=row.get('decoded_bgr_sha256')
                    if rawhash and hashlib.sha256(frame.tobytes()).hexdigest()!=rawhash:raise ValueError('Decoded pixel hash mismatch')
                    labels=[(0,b) for b in row['final_boxes']] if kind=='ball' else [(b['class_id'],b['bbox']) for b in row['labels']]
                    label=args.output/kind/'labels'/row['split']/(dest.stem+'.txt');label.parent.mkdir(parents=True,exist_ok=True)
                    label.write_text('\n'.join(f'{c} {(b[0]+b[2])/2560:.8f} {(b[1]+b[3])/1440:.8f} {(b[2]-b[0])/1280:.8f} {(b[3]-b[1])/720:.8f}' for c,b in labels)+ ('\n' if labels else ''))
            finally:cap.release()
        config={'path':str((args.output/kind).resolve()),'train':'images/train',
            'val':'images/val' if kind=='ball' else 'images/train','names':['ball'] if kind=='ball' else ['LCross','TCross','XCross','PenaltyPoint']}
        if kind=='ball':config['test']='images/test'
        (args.output/kind/'data.yaml').write_text(json.dumps(config,indent=2),encoding='utf-8')
        print(kind,len(rows),'images reconstructed')
if __name__=='__main__':main()
