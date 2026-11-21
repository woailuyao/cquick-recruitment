"""Numeric geometry checks, not evidence of physical measurement accuracy."""
from pathlib import Path
import unittest,math
import numpy as np
import cv2
from relative_localization import RelativeLocalizer,junction
from field_map_matching import Landmark,match_ground_points
ROOT=Path(__file__).resolve().parents[1]

class GeometryTests(unittest.TestCase):
    def setUp(self):self.loc=RelativeLocalizer(ROOT/'config/field_geometry.yaml',ROOT/'config/field.yaml')
    def test_reference_roundtrip_and_unknown_scale(self):
        points=[{'class_name':key[:-1],'pixel':value} for key,value in self.loc.geometry['marker_pixels'].items()]
        result=self.loc.estimate_points(points)
        self.assertTrue(result['relative_valid']);self.assertFalse(result['valid'])
        self.assertIsNone(result['x_camera_map'])
        np.testing.assert_allclose([result['x_camera_map_units'],result['y_camera_map_units']],self.loc.geometry['camera_center_map_units'][:2],atol=1e-5)
    def test_no_stale_pose(self):
        result=self.loc.estimate_points([])
        self.assertFalse(result['relative_valid']);self.assertIsNone(result['x_camera_map_units'])
    def test_ray_facing_away_from_ground(self):
        self.loc.R=np.eye(3)
        self.assertIsNone(self.loc.ground_point([640,360]))
    def test_symmetric_map_rejects_ambiguity(self):
        points=[{'class_name':'LCross','x':0,'y':0},{'class_name':'TCross','x':1,'y':0},{'class_name':'XCross','x':0,'y':1}]
        marks=[Landmark(c+str(k),c,x+dx,y) for k,dx in [(1,0),(2,4)] for c,x,y in [('LCross',0,0),('TCross',1,0),('XCross',0,1)]]
        result=match_ground_points(points,marks)
        self.assertFalse(result['valid']);self.assertEqual(result['status'],'ambiguous')
    def test_joint_not_box_centre(self):
        image=np.zeros((120,120,3),np.uint8);image[:]=(30,65,30)
        cv2.line(image,(30,30),(30,80),(50,230,240),8);cv2.line(image,(30,80),(90,80),(50,230,240),8)
        point=junction(image,[24,24,97,87])
        self.assertIsNotNone(point);self.assertLess(np.linalg.norm(np.array(point)-[30,80]),5)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):match_ground_points([{'class_name':'LCross','x':float('nan'),'y':0}]*3,self.loc.landmarks)
if __name__=='__main__':unittest.main()
