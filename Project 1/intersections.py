import time
from sortedcontainers import SortedList

def segment_pair_intersection(line1, line2):
    '''Return the intersection point between 2 line segments or None, if they do not intersect. 

    arguments:
    line1 - is the first line segment and has the following form ((x1,y1), (x2,y2)) where x1, y1 is
      the first point of the line segment and x2,y2 is the second point of the line segment.
    line2 - is the second line segment and has the same format as line1.

    return:
    ipoint - A tuple (x,y) which is the point of intersection or None if it does not exist 
    '''
    ### YOUR CODE STARTS HERE ###
    # Assign coordinates to line1 and line2
    ((x1, y1), (x2, y2)) = line1
    ((x3, y3), (x4, y4)) = line2

    # Compute denominator based on cross product of two lines
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)

    # Parallel lines
    if denom == 0:
        return None

    # Compute numerators
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

    # Check if intersection is within the segments
    # If 'ua' and 'ub' are both within 0-1, then two lines intersect with each other
    if 0 <= ua <= 1 and 0 <= ub <= 1:
        ix = x1 + ua * (x2 - x1)
        iy = y1 + ua * (y2 - y1)
        return (ix, iy)

    return None

    # intersect = True 

    # if intersect:
    #     # intersection point between line1 and line2
    #     return (0,0)
    # else:
    #     return None

    ## YOUR CODE ENDS HERE ###

def efficient_intersections(L1, L2):

    #This is the Bonus part of the assignment 

    ### YOUR CODE STARTS HERE ###

    # return intersections

    return ([], []) 
    ### YOUR CODE ENDS HERE ###

def all_pairs_intersections(L1, L2):
    x = []
    y = []
    for l1 in L1: 
        for l2 in L2: 
          point =  segment_pair_intersection(l1, l2)
          if point: 
            x.append(point[0])
            y.append(point[1])

    return (x,y)