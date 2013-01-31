'''
Python 3 user library for corobots project

Based on Java library

Z. Butler, Jan 2013
'''
import socket
import os
import RobotMap

class RobotConnectionException(Exception):
    pass

class Robot():
    __slots__ = ('sockin', 'sockout', 'USER_PORT')

    def __init__(self):
        '''
           Creates connection to robot
        '''
        print('Connecting to robot...')
        self.USER_PORT = 15001
        self._openSocket()

    def _openSocket(self):
        try:
            robotaddr = os.getenv('ROBOT')
            sock = socket.create_connection((robotaddr,self.USER_PORT))
            self.sockout = sock.makefile('w')
            self.sockin = sock.makefile('r')
        except OSError:
            print('Error connecting to assigned robot. Please try again')
            raise RobotConnectionException()
    
    def navigateToLocation(self,location,block):
        location = location.upper()
        if RobotMap.isNode(location):
            msg = 'NAVTOLOC ' + location + '\n'
            self.sockout.write(msg)
            self.sockout.flush()
            if block:
                return self._queryArrive()
            else:
                return True
        else:
            return False

    def goToLocation(self,location,block):
        location = location.upper()
        if RobotMap.isNode(location):
            msg = 'GOTOLOC ' + location + '\n'
            self.sockout.write(msg)
            self.sockout.flush()
            if block:
                return self._queryArrive()
            else:
                return True
        else:
            return False

    def goToXY(self,x,y,block):
        msg = 'GOTOXY ' + str(x) + ' ' + str(y) + '\n'
        self.sockout.write(msg)
        self.sockout.flush()
        if block:
            return self._queryArrive()
        else:
            return True

    def _queryArrive(self):
        self.sockout.write('QUERY_ARRIVE\n')
        self.sockout.flush()
        gotback = self.sockin.readline()
        return gotback[:6] == 'ARRIVE'

    def getPos(self):
        ''' Asks robot for its position, returns an (x,y) tuple
        '''
        self.sockout.write('GETPOS\n')
        self.sockout.flush()
        posstr = self.sockin.readline()
        posparts = posstr.split(' ')
        if posparts[0] != 'POS':
            raise Exception()
        else:
            return (float(posparts[1]),float(posparts[2]))

    def getClosestLoc(self):
        pos = self.getPos()
        return RobotMap.getClosestNode(pos)
