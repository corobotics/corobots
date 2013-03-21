'''
Simple example showing usage of the Python version of the corobot user API.

author: Z. Butler
'''
from Robot import *

def main():
    '''
      Sample program, drives the robot around a bit.
    '''
    r = Robot()
    pos = r.get_pos()
    print('Robot is at',pos)
    loc = r.get_closest_loc()
    print('Closest node is',loc)
    #r.go_to_location(loc,True)
    if r.go_to_XY(pos[0]-1.5, pos[1]+1.0, True):
        print('Made it!')
    else:
        print("Didn't make it.")

main()
