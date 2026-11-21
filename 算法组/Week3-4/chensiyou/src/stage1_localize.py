import math
import numpy as np
import yaml


field_map_data = {
    'TCross1': [5.955, 2.930],
    'TCross2': [4.945, 2.930],
    'TCross3': [1.000, 2.930],
    'TCross4': [0.000, 2.930],
    'LCross1': [7.465, 2.930],
    'LCross2': [-1.485, 2.930],
    'LCross3': [4.945, 2.020],
    'LCross4': [1.000, 2.020],
    'LCross5': [5.955, 0.000],
    'XCross1': [4.505, -4.010],
    'XCross2': [1.565, -4.010],
    'PenaltyPoint': [2.965, 0.885]
}



def solve_pose_from_two_points(p1_obs, p2_obs, p1_map, p2_map):
    """
    利用两个观测点和两个地图点，解算相机位姿 (x, y, yaw)
    p_obs: 观测点 (X前方, Y右侧)
    p_map: 地图点 (X绝对, Y绝对)
    """

    vec_obs = np.array([p2_obs[0] - p1_obs[0], p2_obs[1] - p1_obs[1]])
    vec_map = np.array([p2_map[0] - p1_map[0], p2_map[1] - p1_map[1]])


    angle_obs = math.atan2(vec_obs[1], vec_obs[0])
    angle_map = math.atan2(vec_map[1], vec_map[0])
    yaw_rad = angle_map - angle_obs



    cos_a, sin_a = math.cos(yaw_rad), math.sin(yaw_rad)
    R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    p1_map_np = np.array(p1_map)
    p1_obs_np = np.array(p1_obs)

    t = p1_map_np - np.dot(R, p1_obs_np)

    return t[0], t[1], math.degrees(yaw_rad)



def match_and_localize(observed_markers):
    """
    observed_markers: 列表，形如 [('TCross', 1.19, -0.60), ('LCross', 2.5, 1.2)]
    返回最佳位姿 (x, y, yaw) 和 最小误差
    """
    best_pose = None
    min_error = float('inf')


    if len(observed_markers) < 2:
        return None, None


    for i in range(len(observed_markers)):
        for j in range(i + 1, len(observed_markers)):
            m1_type, m1_x, m1_y = observed_markers[i]
            m2_type, m2_x, m2_y = observed_markers[j]


            map_points_1 = {k: v for k, v in field_map_data.items() if k.startswith(m1_type)}
            map_points_2 = {k: v for k, v in field_map_data.items() if k.startswith(m2_type)}


            for name1, coord1 in map_points_1.items():
                for name2, coord2 in map_points_2.items():
                    if name1 == name2: continue


                    cam_x, cam_y, cam_yaw = solve_pose_from_two_points(
                        (m1_x, m1_y), (m2_x, m2_y),
                        coord1, coord2
                    )


                    total_error = 0
                    for m_type, mx, my in observed_markers:

                        cos_a, sin_a = math.cos(math.radians(cam_yaw)), math.sin(math.radians(cam_yaw))
                        R = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
                        p_obs = np.array([mx, my])
                        p_map_pred = np.dot(R, p_obs) + np.array([cam_x, cam_y])


                        min_dist = float('inf')
                        for k, v in field_map_data.items():
                            if k.startswith(m_type):
                                dist = np.linalg.norm(p_map_pred - np.array(v))
                                if dist < min_dist:
                                    min_dist = dist
                        total_error += min_dist


                    if total_error < min_error:
                        min_error = total_error
                        best_pose = (cam_x, cam_y, cam_yaw)

    return best_pose, min_error



if __name__ == '__main__':

    observed = [
        ('TCross', 1.19, 0.60),
        ('LCross', 2.50, -1.20),
        ('XCross', 3.00, 0.00)  
    ]

    print("观测到的特征:", observed)
    print("正在地图库中暴力搜索匹配...")

    pose, err = match_and_localize(observed)

    if pose:
        print("\n 匹配成功！ ")
        print(f" 相机在地图中的位置: X = {pose[0]:.3f} m, Y = {pose[1]:.3f} m")
        print(f" 相机朝向 (Yaw): {pose[2]:.1f} 度")
        print(f" 匹配残差 (越小越好): {err:.3f} m")
        print("")
    else:
        print(" 匹配失败，特征点不足或误差太大。")