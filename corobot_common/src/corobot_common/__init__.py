import math

def distance(x, y):
    """The distance from the origin to (x, y)."""
    return math.sqrt(x * x + y * y)

def point_distance(p1, p2):
    """Distance between two point-like objects in the Euclidean plane."""
    return distance(p2.x - p1.x, p2.y - p1.y)

def point_equals(p1, p2):
    """Checks if two point-like objects have the same x and y."""
    return p1.x == p2.x and p1.y == p2.y

def bresenham(x1, y1, x2, y2, f):
    """Can I straight line nav to this wp from given point?"""
    x1, y1 = int(x1), int(y1)
    x2, y2 = int(x2), int(y2)
    dx = x2 - x1
    dy = y2 - y1
    inc_x = int(math.copysign(1, dx))
    inc_y = int(math.copysign(2, dy))
    dx *= inc_x
    dy *= inc_y
    x = x1
    y = y1
    if dx > dy:
        inc_side = 2 * dy
        inc_diag = inc_side - 2 * dx
        d = 2 * dy - dx
        while x <= x2 if inc_x > 0 else x >= x2:
            v = f(x, y)
            if v is not None:
                return v
            if d <= 0:
                d += inc_side
            else:
                d += inc_diag
                y += inc_y
            x += inc_x
    else:
        inc_side = 2 * dx
        inc_diag = inc_side - 2 * dy
        d = 2 * dx - dy
        while y <= y2 if inc_y > 0 else y >= y2:
            v = f(x, y)
            if v is not None:
                return v
            if d <= 0:
                d += inc_side
            else:
                d += inc_diag
                x += inc_x
            y += inc_y
