#!/usr/bin/env python
# CommandListener.py
# author - Jaydeep Untwal (jmu8722@cs.rit.edu)
import socket               # Import socket module
from DataManager import DataManager

def query(c, commands):
	sql = ''
	for i in range(1,len(commands)):
		sql = sql + commands[i] + ' '
	c.send(dm.query(sql))
	return

dm = DataManager("http://vhost7.cs.rit.edu/Corobot/CorobotMain.php","corobot","corobot!@#")
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM,0)         # Create a socket object
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = '' # Get local machine name
port = 8888                 # Reserve a port for your service.

dm.updateInformation(socket.gethostname()+".rit.edu",port)

s.bind((host, port))        # Bind to the port
s.listen(5)                 # Now wait for client connection.
while True:
	c, addr = s.accept()     # Establish connection with client.
	input = str(c.recv(1024))
	commands = input.split( );
	options = {'query':query}
	
	try:
		options.get(commands[0])(c,commands)
	except:
		c.send("Invalid Command!")
	c.close()                # Close the connection
