"""Inspect measured/organizer map bytes; create only a verified adapter after real confirmation."""
import argparse,hashlib,json,shutil
from pathlib import Path
import yaml
from geometry import read_map

def main():
 p=argparse.ArgumentParser();p.add_argument('--field',required=True,type=Path);p.add_argument('--geometry',required=True,type=Path);p.add_argument('--version',required=True);p.add_argument('--out',type=Path,default=Path('config'));p.add_argument('--confirm-measured-map','--confirm-official-match',dest='confirm_measured_map',action='store_true');a=p.parse_args();field=yaml.safe_load(a.field.read_text());geom=yaml.safe_load(a.geometry.read_text());sha=hashlib.sha256(a.field.read_bytes()).hexdigest();print('field SHA256:',sha);print('field keys:',list(field) if isinstance(field,dict) else type(field).__name__);print('geometry:',json.dumps(geom,ensure_ascii=False,indent=2));markers=read_map(a.field);print('Parsed map points:',len(markers))
 if not a.confirm_measured_map:print('仅检查，未写入运行配置。请先核验实测坐标与场地一致，原点为中圈中心，X朝球门、Y朝左、Z朝上，单位m。');return
 if isinstance(field,dict) and (field.get('verified') is False or field.get('usage')=='reference_draft_only_not_verified_for_live_localization'):raise SystemExit('整份地图未确认：不能生成正式verified适配器。先现场贴设并实测点球点，在新版本中记录确认依据。')
 if isinstance(field,dict) and any(r.get('verified') is False for r in field.get('landmarks',[]) if isinstance(r,dict)):raise SystemExit('存在未确认地图点：不能生成正式verified适配器。')
 a.out.mkdir(parents=True,exist_ok=True)
 if (a.out/'field.yaml').exists() or (a.out/'field_geometry.yaml').exists():raise SystemExit('运行地图已存在，停止以保留已有版本。使用新输出目录。')
 shutil.copy2(a.field,a.out/'field.yaml');shutil.copy2(a.geometry,a.out/'field_geometry.source.yaml');adapter=dict(schema='cquick_field_adapter_v1',verified=True,units='m',axes='x_to_goal_y_left_z_up_yaw_ccw',version=a.version,field_sha256=sha,source_geometry_sha256=hashlib.sha256(a.geometry.read_bytes()).hexdigest(),confirmation='Operator confirmed measured map, meters, leader coordinate convention and actual-field match. Input coordinates unchanged.');(a.out/'field_geometry.yaml').write_text(yaml.safe_dump(adapter,sort_keys=False));print('已保留输入地图字节和几何原文，写入适配元数据。坐标未修改。')
if __name__=='__main__':main()
