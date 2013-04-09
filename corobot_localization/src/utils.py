from math import cos, sin

from numpy.matlib import matrix

def column_vector(*args):
    """Utility function to construct a column vector (single-column matrix)."""
    return matrix([[e] for e in args])

def rotation_matrix(theta):
    return matrix([
        [cos(theta), -sin(theta), 0],
        [sin(theta),  cos(theta), 0],
        [         0,           0, 1]])

def coord_transform(state, offset):
    """Perform a coordinate transform on a state vector.

    state: a column vector of x, y, theta
    offset: a column vector of dx, dy, dtheta

    """
    dt = offset.item(2, 0)
    rotation = rotation_matrix(dt)
    return rotation * state + offset

def get_offset(state_a, state_b):
    """Get the origin of reference frame B in A."""
    dt = state_a.item(2, 0) - state_b.item(2, 0)
    rotation = rotation_matrix(dt)
    return state_a - rotation * state_b
