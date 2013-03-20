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
        self._open_socket()

    def _open_socket(self):
        '''
        Creates the connection to the robot.
        '''
        try:
            robotaddr = os.getenv('ROBOT')
            sock = socket.create_connection((robotaddr,self.USER_PORT))
            self.sockout = sock.makefile('w')
            self.sockin = sock.makefile('r')
        except OSError:
            print('Error connecting to assigned robot. Please try again')
            raise RobotConnectionException()
    
    def navigate_to_location(self,location,block):
        '''
        Drives the robot to the given location, including a full path plan.

        location - destination: name of a map node (waypoint)
        block - whether or not to block (wait for destination to be reached before returning)

        Returns whether or not location was reached successfully
        '''
        location = location.upper()
        if RobotMap.is_node(location):
            msg = 'NAVTOLOC ' + location + '\n'
            self.sockout.write(msg)
            self.sockout.flush()
            if block:
                return self._query_arrive()
            else:
                return True
        else:
            return False

    def go_to_location(self,location,block):
        '''
        Drives the robot in a straight line to the given location.

        location - destination: name of a map node (waypoint)
        block - whether or not to block (wait for destination to be reached before returning)

        Returns whether or not location was reached successfully
        '''
        location = location.upper()
        if RobotMap.is_node(location):
            msg = 'GOTOLOC ' + location + '\n'
            self.sockout.write(msg)
            self.sockout.flush()
            if block:
                return self._query_arrive()
            else:
                return True
        else:
            return False

    def go_to_XY(self,x,y,block):
        '''
        Drives the robot in a straight line to the given coordinates.
        x, y - location to drive to
        block - whether or not to block (wait for destination to be reached before returning)

        Returns whether or not location was reached successfully
        '''
        msg = 'GOTOXY ' + str(x) + ' ' + str(y) + '\n'
        self.sockout.write(msg)
        self.sockout.flush()
        if block:
            return self._query_arrive()
        else:
            return True

    def _query_arrive(self):
        '''
        Internal method used for blocking gotos
        '''
        self.sockout.write('QUERY_ARRIVE\n')
        self.sockout.flush()
        gotback = self.sockin.readline()
        return gotback[:6] == 'ARRIVE'

    def get_pos(self):
        ''' 
        Asks robot for its position, returns an (x,y) tuple
        '''
        self.sockout.write('GETPOS\n')
        self.sockout.flush()
        posstr = self.sockin.readline()
        posparts = posstr.split(' ')
        if posparts[0] != 'POS':
            raise Exception()
        else:
            return (float(posparts[1]),float(posparts[2]))

    def get_closest_loc(self):
        '''
        Returns the closest node to the current robot location
        '''
        pos = self.get_pos()
        return RobotMap.get_closest_node(pos)
