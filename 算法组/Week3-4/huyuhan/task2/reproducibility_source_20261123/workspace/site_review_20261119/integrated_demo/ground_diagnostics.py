"""Read-only ray/plane diagnostics derived after a valid PnP result."""
import cv2
import numpy as np


def project_used_points(session):
    """Return solved extrinsics and ground hits without modifying the pose or points."""
    pose = session.pose
    if pose is None:
        return None, []
    rotation = pose["rotation"]
    center = pose["center"]
    translation = -rotation @ center
    values = {
        "R_camera_from_map": rotation.tolist(),
        "t_camera_from_map_m": translation.tolist(),
        "optical_center_map_m": center.tolist(),
        "ground_plane_map": "Z=0 m",
        "origin": "current estimated optical-center plumb projection",
        "method": "derived_from_current_pnp_not_fixed_installation_extrinsics",
    }
    ids = list(session.points)
    if not ids:
        return values, []
    pixels = np.asarray([session.points[key] for key in ids], dtype=np.float64).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(pixels, session.intr, session.dist).reshape(-1, 2)
    rays_map = np.column_stack((normalized, np.ones(len(ids)))) @ rotation
    forward = pose["direction"][:2] / np.linalg.norm(pose["direction"][:2])
    left = np.array([-forward[1], forward[0]])
    results = []
    for key, ray in zip(ids, rays_map):
        known = np.asarray(session.markers[key]["xyz"][:2])
        row = dict(map_id=key, class_name=session.markers[key]["type"],
                   pixel_uv=[float(v) for v in session.points[key]], matched_map_xy_m=known.tolist(),
                   point_origin="optical_flow_from_manually_selected_map_id",
                   status="ray_no_forward_ground_intersection", ground_map_xy_m=None,
                   ground_local_pnp_xy_m=None, map_difference_m=None)
        if np.isfinite(ray).all() and ray[2] < -1e-8:
            distance = -center[2] / ray[2]
            hit = center[:2] + distance * ray[:2]
            delta = hit - center[:2]
            row.update(status="pnp_derived_ground_intersection", ground_map_xy_m=hit.tolist(),
                       ground_local_pnp_xy_m=[float(delta @ forward), float(delta @ left)],
                       map_difference_m=float(np.linalg.norm(hit - known)))
        results.append(row)
    return values, results
