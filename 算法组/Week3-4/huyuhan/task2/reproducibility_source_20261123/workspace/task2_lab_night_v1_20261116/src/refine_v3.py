"""Hybrid candidate: preserve current accepted fits; recover only strong v2 consensus."""
import numpy as np
from refine import refine as refine_current
from refine_v2 import refine_v2


def refine_v3(image, box, cls):
    old = refine_current(image, box, cls)
    if old is not None:
        return old
    point, debug = refine_v2(image, box, cls, return_debug=True)
    if point is None:
        return None
    point = np.asarray(point, float)
    x1, y1, x2, y2 = map(float, box)
    bw, bh = max(1.0, x2 - x1), max(1.0, y2 - y1)
    scale = max(1.0, min(bw, bh))
    tolerance = max(7.0, scale * .14)
    near = [x for x in debug if x["fit"] is not None and
            np.linalg.norm(np.asarray(x["fit"], float) - point) <= tolerance]
    spread = max([np.linalg.norm(np.asarray(x["fit"], float) - point) for x in near], default=999.0)
    center_distance = np.linalg.norm((point - [(x1+x2)/2, (y1+y2)/2]) / [bw, bh])
    if (len(near) < 3 or len({x["margin"] for x in near}) < 2 or
            len({x["mode"] for x in near}) < 2 or spread > 12.0 or center_distance > .20):
        return None
    return point.tolist()
