"""Explicit JSON contracts carried by the two ROS2 std_msgs/String topics."""
from datetime import datetime, timezone
import math

def _header(packet, frame_id):
 stamp=datetime.fromisoformat(packet['timestamp']).astimezone(timezone.utc)
 sec=int(stamp.timestamp())
 return dict(stamp=dict(sec=sec,nanosec=stamp.microsecond*1000),
             stamp_iso_utc=stamp.isoformat(),frame_id=frame_id,seq=int(packet['frame']))

def _pose_fields(packet, prefix):
 x=packet.get('x_'+prefix+'_map')
 y=packet.get('y_'+prefix+'_map')
 yaw=packet.get('yaw_'+prefix+'_map')
 if x is None or y is None or yaw is None:return None
 if not all(math.isfinite(float(v)) for v in (x,y,yaw)):return None
 return dict(x_m=float(x),y_m=float(y),yaw_deg=float(yaw),yaw_rad=math.radians(float(yaw)))

def pose_message(packet):
 """One fresh message per frame. Invalid messages carry no prior pose coordinates."""
 valid=bool(packet.get('valid',False))
 carrier=_pose_fields(packet,'carrier') if valid else None
 camera=_pose_fields(packet,'camera') if valid else None
 valid=valid and carrier is not None and camera is not None
 residual=packet.get('residual')
 if residual is not None and not math.isfinite(float(residual)):residual=None
 return dict(schema='cquick.field_pose.v1',
             header=_header(packet,'field_map'),
             source=packet['source'],input_kind=packet['source'],
             source_transport=packet.get('source_transport'),
             map_axes='+X_to_goal,+Y_left,+Z_up; yaw_ccw_from_+X',
             position_unit='m',yaw_deg_unit='deg',yaw_rad_unit='rad',residual_unit='m',
             valid=valid,status=packet.get('status','invalid'),
             marker_count=int(packet.get('marker_count',0)),
             residual_m=float(residual) if residual is not None else None,
             camera_optical_center_in_map=camera if valid else None,
             carrier_origin_in_map=carrier if valid else None,
             test_point_label=packet.get('test_point'))

def ball_message(packet, balls):
 detections=[]
 for b in balls:
  box=b['bbox']
  if len(box)!=4 or not all(math.isfinite(float(v)) for v in box):raise ValueError('Invalid ball bbox')
  detections.append(dict(class_id=0,class_name='ball',confidence=float(b['confidence']),
                         bbox_xyxy_px=[float(v) for v in box],
                         center_uv_px=[float(b['center_u']),float(b['center_v'])]))
 return dict(schema='cquick.ball.v1',
             header=_header(packet,'camera_optical'),
             source=packet['source'],source_transport=packet.get('source_transport'),
             image_width_px=int(packet['width']) if 'width' in packet else None,
             image_height_px=int(packet['height']) if 'height' in packet else None,
             coordinate_system='pixel u right, v down from image top-left',
             detections=detections)
