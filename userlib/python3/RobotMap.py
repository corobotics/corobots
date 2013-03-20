'''
This module represents the map for use in navigation.

MapNode class will be used and returned by several functions.

Author: Z. Butler
'''
class MapNode():
    __slots__ = ('x','y','name','nbrs')

    def __init__(self,name,x,y):
        self.x = x
        self.y = y
        self.name = name
        self.nbrs = []

def _make_map():
    '''
    Internal function, creates the map object from the file
    '''
    nodes = {}
    first = True
    for line in open('../../corobot_map/map/waypoints.csv'):
        if first:
            first = False
            continue
        parts = line.split(',')
        node = MapNode(parts[0].upper(),float(parts[3]),float(parts[4]))
        node.nbrs = parts[6:]
        nodes[node.name] = node
    return nodes
    

def get_closest_node(pos):
    '''
    Returns the closest node to the given position
    
    pos - (x,y) location desired

    returns the name of the closest node
    '''
    global __theMap__
    minsqdist = float('+infinity')
    closest = None
    for nodename in __theMap__:
        node = __theMap__[nodename]
        sqdist = (node.x-pos[0])*(node.x-pos[0]) + (node.y-pos[1])*(node.y-pos[1])
        if sqdist < minsqdist:
            minsqdist = sqdist
            closest = node.name
    return closest

def get_node_names():
    '''
    Returns names of all nodes in the map
    '''
    global __theMap__
    return __theMap__.keys()

def get_node(name):
    '''
    Returns a MapNode object for the given name
    '''
    global __theMap__
    return __theMap__[name.upper()]

def is_node(name):
    ''' 
    Returns true/false whether the given name represents a node in the map
    
    name - node name to test
    '''
    global __theMap__
    return name in __theMap__


__theMap__ = _make_map()

