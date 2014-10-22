#!/usr/bin/env python
# MessageManager.py
# author - Jaydeep Untwal (jmu8722@cs.rit.edu)
import time
import os
from DataManager import DataManager

# Set Server, User and Password
dm = DataManager("http://vhost7.cs.rit.edu/Corobot/CorobotMain.php", "corobot", "corobot!@#")

while True:

    #Check connection
    conn = dm.checkConnectionToServer()
    if conn is True:
	#Get and Send messages
	dm.getMessages(dm.hostName)
	dm.pushLocalData()
	#dm.printLocalData()
    #Sleep
    time.sleep(5)


