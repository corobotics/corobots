import math
from collections import namedtuple
from heapq import heappop, heappush

def distance(x, y):
    """The distance from the origin to (x, y)."""
    return math.sqrt(x * x + y * y)

def point_distance(p1, p2):
    """Distance between two point-like objects in the Euclidean plane."""
    return distance(p2.x - p1.x, p2.y - p1.y)

def point_equals(p1, p2):
    """Checks if two point-like objects have the same x and y."""
    return p1.x == p2.x and p1.y == p2.y

# A simple named tuple class to make a_star() more readable.
a_star_data = namedtuple("a_star_data", ("cost", "parent"))

def a_star(start, is_goal, neighbors, get_cost, heuristic):
    """Perform the A* algorithm to find a path from start to goal.

    A node can be any object with a hashable "name" attribute.

    Arguments:
      start          - The node to start from.
      is_goal(x)     - Function to test whether node x is a goal.
      neighbors(x)   - Function that returns the neighbors of node x.
      get_cost(x, y) - Function to obtain the cost from node x to node y.
      heuristic(x)   - Calculate approximate cost from node x to the goal.

    Returns a list of nodes that form a path to a goal, or an empty list.
    The path returned does not include the start node.

    """
    # A heap of (score, cost, node) tuples to visit, where
    # score is defined as: cost + heuristic(node)
    frontier = [(heuristic(start), 0, start)]
    # Stores the cost along the best known path from start to
    # each node and the parent of the node along that path.
    node_data = {start.name: a_star_data(0, None)}
    # Main A* loop.
    while frontier:
        _, cost, node = heappop(frontier)
        best = node_data.get(node.name)
        if best is not None and best.cost < cost:
            # This node has already been improved upon since it was added
            # to the heap; skip it.
            continue
        if is_goal(node):
            # We found the goal! Construct the path and return.
            path = []
            parent = best.parent
            while parent is not None:
                path.append(node)
                node = parent
                parent = node_data[node.name].parent
            # Since we traverse from goal back to start, reverse the path.
            path.reverse()
            # Fin.
            return path
        # Extend our frontier with the neighbors of this node.
        for neighbor in neighbors(node):
            # Find the total cost along this path to the neighbor.
            neighbor_cost = cost + get_cost(node, neighbor)
            # Get the best previous data for the neighbor.
            neighbor_best = node_data.get(neighbor.name)
            # Only add the new node if it improves on the previous best cost.
            if neighbor_best is None or neighbor_cost < neighbor_best.cost:
                node_data[neighbor.name] = a_star_data(neighbor_cost, node)
                score = neighbor_cost + heuristic(neighbor)
                heappush(frontier, (score, neighbor_cost, neighbor))
    return []

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
