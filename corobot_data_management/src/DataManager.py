# DataManager.py
# authors - Jaydeep Untwal (jmu8722@cs.rit.edu)
import sqlite3 as sql
import urllib2
import urllib
import datetime
import socket
import json

# Class to send and receive data from the server or local database
class DataManager:

    # Self Variables
    # localDB
    # server
    # hostName
    # user
    # password

    def __init__(self, server, user, password):
        self.localDB = 'Corobot_Local_DB'
        self.server = server
        self.hostName = self.getHostName()
        self.user = user
        self.password = password
        self.createLocalDatabase()
        return

    def checkConnectionToServer(self):
       try:
            request = urllib2.urlopen(self.server)
            return True
       except Exception as e:
           return False

    def createLocalDatabase(self):
        sqlCon = sql.connect(self.localDB)
        cur = sqlCon.cursor()
        #cur.execute("DROP TABLE IF EXISTS corobot_messages;")
	cur.execute('''CREATE TABLE IF NOT EXISTS corobot_messages (
                        Message_ID INTEGER PRIMARY KEY AUTOINCREMENT,
                        Sender varchar(20),
                        Receiver varchar(20),
                        Topic varchar(20),
                        Data Text,
                        Timestamp DateTime,
                        Sync INTEGER);''')
        sqlCon.commit()
        sqlCon.close()
        return

    def postRequest(self, postData):
        postData = postData.encode('utf-8')
        request = urllib2.Request(self.server,postData)
        request.add_header("Content-Type","application/x-www-form-urlencoded;charset=utf-8")
        f = urllib2.urlopen(request, postData)
        return str(f.read().decode('utf-8'))

    def query(self, q):
	sqlCon = sql.connect(self.localDB)
	cur = sqlCon.cursor()
	cur.execute(q)
	temp = cur.fetchall()
	sqlCon.close()
	return str(temp)

    def updateInformation(self, hostaddr, port):
	postData = urllib.urlencode({'Method': 'updateInformation', 'CorobotID': self.hostName, 'HostAddress': hostaddr, 'Port': port, 'User': self.user, 'Pass': self.password})
	return self.postRequest(postData)

    def sendMessage(self, sender, receiver, topic, data, dT):

        if dT is None:
            dT = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        postData = urllib.urlencode({'Method': 'sendMessage', 'Sender': sender, 'Receiver': receiver,
                                           'Topic': topic, 'Data': data, 'Timestamp': dT, 'User': self.user, 'Pass': self.password})
        return self.postRequest(postData)

    def getMessages(self, receiver):
	sqlCon = sql.connect(self.localDB)
	cur = sqlCon.cursor()
            
	postData = urllib.urlencode({'Method': 'getMessages', 'Receiver': receiver, 'User': self.user, 'Pass': self.password})
	jsonStr = self.postRequest(postData)

    	jsonData = json.loads(jsonStr)

	for i in range(len(jsonData)):
		cur.execute('''INSERT INTO corobot_messages (Sender,Receiver,Topic,Data,Timestamp,Sync) VALUES(?,?,?,?,?,?)''',
		(jsonData[i]["Sender"], jsonData[i]["Receiver"], jsonData[i]["Topic"], jsonData[i]["Data"], jsonData[i]["Timestamp"], 1))

	sqlCon.commit()
	#sqlCon.close()
        return

    def saveLocal(self, sender, receiver, topic, data):
        sqlCon = sql.connect(self.localDB)
        cur = sqlCon.cursor()
        dT = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cur.execute('''INSERT INTO corobot_messages (Sender,Receiver,Topic,Data,Timestamp,Sync) VALUES(?,?,?,?,?,?)''',
                    (sender, receiver, topic, data, dT,0))
        sqlCon.commit()
        sqlCon.close()
        return

    def printLocalData(self):
        sqlCon = sql.connect(self.localDB)
        cur = sqlCon.cursor()
        cur.execute("SELECT * FROM corobot_messages")
        data = cur.fetchall()

        for d in data:
            print(d)

        sqlCon.close()
        return

    def pushLocalData(self):
        sqlCon = sql.connect(self.localDB)
        cur = sqlCon.cursor()
        cur.execute("SELECT * FROM corobot_messages WHERE Sync = 0")
        data = cur.fetchall()

        for d in data:
            res = self.sendMessage(d[1], d[2], d[3], d[4], d[5])
            if res == "1":
                cur.execute("UPDATE corobot_messages SET Sync = 1 WHERE Message_ID = %d" % (d[0]))
                sqlCon.commit()

	sqlCon.commit()
        sqlCon.close()
        return

    def printLocalData(self):
        sqlCon = sql.connect(self.localDB)
        cur = sqlCon.cursor()
        cur.execute("SELECT * FROM corobot_messages")
        data = cur.fetchall()

        for d in data:
            print(d)

        sqlCon.close()
        return


    def getHostName(self):
        hostname = str(socket.gethostname())
        return hostname
