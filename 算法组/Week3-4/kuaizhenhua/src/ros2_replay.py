"""Publish saved detection records as explicit OFFLINE REPLAY, never live input."""
import argparse,json,time
from pathlib import Path
from ros2_output import RosOutput
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('records',type=Path)
    p.add_argument('--limit',type=int,default=0);p.add_argument('--rate',type=float,default=5)
    args=p.parse_args()
    if args.rate<=0:p.error('rate must be positive')
    publisher=RosOutput();time.sleep(1)
    try:
        with args.records.open(encoding='utf-8') as f:
            for i,line in enumerate(f):
                if args.limit and i>=args.limit:break
                record=json.loads(line);record['input_mode']='OFFLINE REPLAY / ROS transport check'
                publisher.publish(record);time.sleep(1/args.rate)
    finally:publisher.close()
if __name__=='__main__':main()
