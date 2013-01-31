from Robot import *

def main():
    r = Robot()
    pos = r.getPos()
    print('Robot is at',pos)
    loc = r.getClosestLoc()
    print('Closest node is',loc)
    #r.goToLocation(loc,True)
    if r.goToXY(pos[0]-1.5, pos[1]+1.0, True):
        print('Made it!')
    else:
        print("Didn't make it.")

main()
