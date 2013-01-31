class MapNode():
    __slots__ = ('x','y','name','nbrs')

    def __init__(self,name,x,y):
        self.x = x
        self.y = y
        self.name = name
        self.nbrs = []

def _makeMap():
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
    

def getClosestNode(pos):
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

def getNodeNames():
    global __theMap__
    return __theMap__.keys()

def getNode(name):
    global __theMap__
    return __theMap__[name.upper()]

def isNode(name):
    global __theMap__
    return name in __theMap__


__theMap__ = _makeMap()

