import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


def is_in_polygon(point, polygon):
    """Check if a 2D point is inside any of the polygons"""
    p = Point(point[0], point[1])
    poly = Polygon(polygon)
    return poly.contains(p)


def endpoints_to_edges(corners, closed=False):
    """ Convert corner points [p1, p2, p3, ..., pn]
        to closed loop polygon edges [(p1, p2), (p2, p3), ..., (pn, p1)]
    """
    # Get the edges from endpoints
    edges = []
    for i in range(len(corners) - 1):
        edges.append((corners[i], corners[i + 1]))
    # If to close the loop
    if closed:
        edges.append((corners[-1], corners[0]))

    return edges


def angle_diff(angle1, angle2, absolute=True):
    """Calculate the min difference between two angles ranged in [-pi, pi]
    arguments:
        angle1: from angle1
        angle2: to angle2
        abs: if return the absolute value of the difference,
             if so, the result is always positive and ranges in [0, pi]
             else, it will return the signed difference from angle1 to angle2
    """
    angle_diff = angle2 - angle1

    # Calculate the absolute difference
    min_diff = np.min(
        [np.abs(angle_diff),
         2 * np.pi - np.abs(angle_diff)]
    )
    if absolute:
        return min_diff

    # Determine if the difference is
    # in the positive or negative direction
    is_pos = angle_diff % (2 * np.pi) < np.pi
    if not is_pos:
        min_diff = -min_diff

    return min_diff

def interpolate_angle(angle1, angle2, num):
    """Interpolate between two angles"""
    # Calculate the step size from angle1 to angle2
    step_size = angle_diff(angle1, angle2, absolute=False) / (num - 1)

    # Interpolate
    interpolated_angles = [
        wrap_to_pi(angle1 + i * step_size) 
        for i in range(num)
    ]

    return interpolated_angles


def wrap_to_pi(angle):
    """Wrap an angle to [-pi, pi]"""
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return angle


def line_intersection(line1, line2):
    """Check if two lines intersect
    arguments:
        line1: a line defined by two endpoints [(x1, y1), (x2, y2)]
        line2: a line defined by two endpoints [(x1, y1), (x2, y2)]

    return:
        Intersection point (x, y) if intersect
        None if not intersect
    """
    # Intersection between line(p1, p2) and line(p3, p4)
    x1, y1 = line1[0][0], line1[0][1]
    x2, y2 = line1[1][0], line1[1][1]
    x3, y3 = line2[0][0], line2[0][1]
    x4, y4 = line2[1][0], line2[1][1]

    # Parallel
    denom = (y4-y3) * (x2-x1) - (x4-x3) * (y2-y1)
    if denom == 0:
        return None

    ua = ((x4-x3) * (y1-y3) - (y4-y3) * (x1-x3)) / denom
    ub = ((x2-x1) * (y1-y3) - (y2-y1) * (x1-x3)) / denom
    # Out of range
    if ua < 0 or ua > 1:
        return None
    if ub < 0 or ub > 1:
        return None

    # Intersection
    x = x1 + ua * (x2-x1)
    y = y1 + ua * (y2-y1)
    return (x, y)


def is_intersecting(L1, L2):
    """Find if an intersection point exists between two set of lines
    arguments:
        L1: a list of lines
        L2: a list of lines

    returns:
        a list of intersection points
    """
    x = []
    y = []
    for l1 in L1:
        for l2 in L2:
            point = line_intersection(l1, l2)
            if point:
                return True
    return False
