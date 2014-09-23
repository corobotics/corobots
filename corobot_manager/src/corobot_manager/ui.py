from Tkinter import *
#from corobot import Robot
import math
import rospy

class CorobotMonitorUI(Tk):

    def __init__(self):
        """ Corobot general info monitor 
        """
        Tk.__init__(self)
        
        self.title("Corobot Monitor")
        
        self.Nav = [0,0]
        self.obsCache = []
        self.pos = [0,0,0]

        self.map = PhotoImage(file = '~corobot/corobot_ws/src/corobot_manager/src/corobot_manager/robotMap-1024.gif')

        self.frame = Frame(self, width = 1000)
        self.imgframe = Frame(self)
        self.lazframe = Frame(self)

        self.frame.grid(padx=8, pady=8, row = 0)
        self.imgframe.grid(padx = 8, pady = 8, row = 1)
        self.lazframe.grid(padx = 8, pady = 8, row = 0, column = 1, rowspan = 2)

        self.label = Label(self.frame, text="Current Pose", font=("Helvetica", 24))
        self.label.grid(column = 0, row = 0, columnspan = 3)
        
	self.canvas = Canvas(self.imgframe, width = self.map.width(), height = self.map.height())
	self.canvas.create_image(self.map.width()/2,self.map.height()/2, image = self.map)
	self.canvas.grid(sticky = 'NE')

        self.marker = self.canvas.create_oval(-5,5,5,-5,fill = 'red')
        self.goal = self.canvas.create_oval(-5,5,5,-5, fill = 'green')

        self.lazcanvas = Canvas(self.lazframe, width = 400, height = 700, bg = 'white')
        self.lazcanvas.grid(sticky = 'NE')
        self.robotoval = self.lazcanvas.create_oval(195, 605, 205, 595, fill = 'red')
        self.lasercolor = 'red'
        
        self.xinfo = Label(self.frame, text="X: 0.0", font=("Helvetica", 24))
        self.xinfo.grid(column = 0, row =2, sticky = 'W')
        self.yinfo = Label(self.frame, text="Y: 0.0", font=("Helvetica", 24))
        self.yinfo.grid(column = 1, row = 2)
        self.thinfo = Label(self.frame, text="Theta: 0.0", font=("Helvetica", 24))
        self.thinfo.grid(column = 2, row = 2, sticky = 'E')
        self.covinfo = Label(self.frame, text="StDev: (0.0, 0.0, 0.0)", font=("Helvetica",18))
        self.covinfo.grid(column = 0, row = 5, sticky = 'W')

        self.absGoalInfo = Label(self.frame, text="AbsGoal: -", font=("Helvetica", 24))
        self.absGoalInfo.grid(column = 0, row = 3, sticky = 'W')

        self.rawnavinfo = Label(self.frame, text="RawNav: -", font=("Helvetica", 24))
        self.rawnavinfo.grid(column = 1, row = 3)

        self.velCmdInfo = Label(self.frame, text="ActVel: -", font=("Helvetica", 24))
        self.velCmdInfo.grid(column = 2, row = 3, sticky = 'E')

        self.obsinfo = Label(self.frame, text="ObsLoc: -", font=("Helvetica", 24))
        self.obsinfo.grid(column = 0, row = 4, columnspan = 3)

        self.netForceInfo = Label(self.frame, text="NetForce: -", font=("Helvetica", 24))
        self.netForceInfo.grid(column = 0, row =6, columnspan = 3)

        self.qrCountInfo = Label(self.frame, text="QRLeft: 0 ; QRRight: 0", font=("Helvetica", 24))
        self.qrCountInfo.grid(column = 2, row = 5, sticky = 'E')

        self.batteryInfo = Label(self.frame, text="Battery: -", font=("Helvetica", 24))
        self.batteryInfo.grid(column = 0, row = 7, sticky = 'W')

        self.laptopBatteryInfo = Label(self.frame, text="Laptop Battery: -", font=("Helvetica", 24))
        self.laptopBatteryInfo.grid(column = 2, row = 7, sticky = 'E')

        self.recoveryInfo = Label(self.frame, text="Recovery: -", font=("Helvetica", 24))
        self.recoveryInfo.grid(column = 1, row = 7)

        self.tBox = Entry(self.frame, width=10)
        self.tBox.grid(column = 1, row = 8)

        #Button(self.frame, text = 'Submit', command = navigate).pack(side=LEFT)

    def navigate():
        pass
    """with Robot("127.0.0.1", 15001) as r:
            r.nav_to(self.tBox.get())"""

    def setPose(self, x, y, theta, cov):
	self.canvas.delete(self.marker)
	self.pose = [x, y, theta]
	self.marker = self.canvas.create_oval((x/.1312)-5, self.map.height()-(y/.1312) + 5, (x/.1312) + 5, self.map.height()-(y/.1312)-5, fill = 'red')
	self.canvas.create_oval((x/.1312)-1, self.map.height()-(y/.1312) + 1, (x/.1312) + 1, self.map.height()-(y/.1312)-1, fill = 'blue', outline = 'blue')
	self.xinfo.configure(text="X: {0:6.3f}".format(x))
        self.yinfo.configure(text="Y: {0:6.3f}".format(y))
        self.thinfo.configure(text="Theta: {0:+4.2f}".format(theta))
        self.covinfo.configure(text="StDev: ({0:6.3g},{1:6.3g},{2:6.3g})".format(math.sqrt(cov[0]),math.sqrt(cov[4]),math.sqrt(cov[8])))

    def setRawnavMsg(self, txt):
        self.lazcanvas.delete("force")
        txt = txt.split()
        self.Nav = [float(txt[0][1:-1]),float(txt[1][:-1])]
        self.lazcanvas.create_line(200, 600, math.sin(self.Nav[1]) * self.Nav[0] * -100 + 200,600 - math.cos(self.Nav[1])* self.Nav[0] * 100, arrow = LAST, tags = "force" )
        text = "RawNav: <{0:4.2f}".format(float(txt[0][1:-1]))
        text += ", {0:4.2f}>".format(float(txt[1][:-1]))
        self.rawnavinfo.configure(text=text)

    def setObsMsg(self, txt):
        text = txt.split()
        rospy.loginfo(text)
        if(text[0] != 'obs'):        
            if(text[2] == 'add'):
            	pos = [float(text[0][1:-1]), float(text[1][:-2])]
                while(int(text[-1]) >= len(self.obsCache) + 1):
                    self.obsCache = self.obsCache + [" "];
            	self.obsCache.append(pos)
	    elif(text[2] == 'rem'):
                self.obsCache.pop(int(text[-1]) - 1)
	else:
            self.obsCache[:] = []
	rospy.loginfo(len(self.obsCache))
        self.drawObs()
        txt = "Obs: " + txt
        self.obsinfo.configure(text=txt)

    def setAbsGoalMsg(self, txt):
	txt = txt.split()
	pos = [float(txt[0][1:-1]), float(txt[1][:-1])]
	txt = "AbsGoal: <{0:4.2f}".format(pos[0])
	txt += ", {0:4.2f}>".format(pos[1])
	self.canvas.delete(self.goal)
	self.goal = self.canvas.create_oval((pos[0]/.1312)-5, self.map.height() - (pos[1]/.1312) +5, (pos[0]/.1312) + 5, self.map.height() - (pos[1]/.1312) - 5, fill ='green')
        self.absGoalInfo.configure(text=txt)

    def setNetForceMsg(self, txt):
        txt = "NetForce: " + txt
        self.netForceInfo.configure(text=txt)

    def setVelCmdMsg(self, txt):
        txt = "ActVel: " + txt
        self.velCmdInfo.configure(text=txt)

    def setQrCountMsg(self, txt):
        """qrInfoText = self.qrCountInfo.cget("text")
        if txt[0] == 'L':
            rightPart = qrInfoText[qrInfoText.index(";") : ]
            txt = "QRLeft: " + txt[1 : ] + " " + rightPart
        else:
            leftPart = qrInfoText[0 : qrInfoText.index(";")]
            txt = leftPart + "; QRRight: " + txt[1 : ]"""
        self.qrCountInfo.configure(text=txt)

    def setRecoveryMsg(self, txt):
        self.recoveryInfo.configure(text=txt)

    def setBatteryMsg(self, txt):
        txt = "Battery: " + txt + "%"
        self.batteryInfo.configure(text=txt)

    def setLaptopBatteryMsg(self, txt):
        txt = "Laptop Battery: " + txt
        self.laptopBatteryInfo.configure(text=txt)

    def setLaserMap(self, Omin, rmin, rmax, Oinc, dist):
        self.lazcanvas.delete('kinectVision')
        for i in range(0, len(dist)):
            if dist[i] < rmin or dist[i] > rmax or math.isnan(dist[i]):
                continue
            theta = Omin + Oinc * i
            pos = [math.sin(theta) * dist[i]*-100 + 200, math.cos(theta) * dist[i]*100]
            self.lazcanvas.create_oval(pos[0] - 5,600 - pos[1]+5, pos[0] + 5,600 - pos[1] -5, tag = 'kinectVision')
        self.drawObs()
        self.lazcanvas.itemconfigure(self.robotoval,fill=self.lasercolor)
        
    def drawObs(self):
        self.lazcanvas.delete('obstacles')
        for obs in self.obsCache:
            if(obs == ' '):
                continue
            posPol = [math.hypot(obs[0] - self.pose[0], obs[1] - self.pose[1]), math.atan2(obs[1] - self.pose[1], obs[0] - self.pose[0]) - self.pose[2]]
            while(posPol[1] < 0):
                posPol[1] += 2* math.pi
            while(posPol[1] > 2 * math.pi):
                posPol[1] -= 2 * math.pi
            posRel = [posPol[0] * math.cos(posPol[1]), posPol[0] * -math.sin(posPol[1])]
            self.lazcanvas.create_oval(posRel[1]*100 + 200 - 10,600 - posRel[0] * 100 + 10, posRel[1]*100 + 200  + 10, 600 - posRel[0] * 100 - 10, fill = 'red', tags = 'obstacles')
                             
class CorobotUIMessage(Tk):

    def __init__(self, display_text, timeout, confirm, okay_text="Okay"):
        """ Corobot popup message ui. 

        Arguments:
        display_text -- Message to display in the popup.
        timeout --- Timeout in seconds after which the popup closes and is assumed unconfirmed.
        confirm --- True if message requires confirmation, False if not.
        okay_text --- Text for the confirmation button, defaults to "Okay"
        """
        Tk.__init__(self)
        self.title("Corobot Message")

        # Confirm indicates whether a response is needed, with the response defaulting to false.
        self.confirm = confirm
        self.response = False

        frame = Frame(self)
        frame.pack(padx=8, pady=8)

        # Text wraps at half of screen width.
        label = Label(frame, text=display_text, font=("Helvetica", 24), wraplength=(self.winfo_screenwidth()/2))
        label.pack(side='top')

        if self.confirm:
            btn1 = Button(frame, text=okay_text, command=self.okay, font=("Helvetica", 16))
            btn1.pack(side='bottom', padx=4, pady=4)

        self.update()
        width = self.winfo_width()
        height = self.winfo_height()
        # Center popup and set minimum window size to smallest pack of 
        yp = (self.winfo_screenheight() / 2) - (height / 2)

        self.geometry("%dx%d%+d%+d" % (width, height, xp, yp))
        self.minsize(width, height)

        # Prevent closing via the 'X' button, forcing either confirmation
        # or timeout.
        self.protocol("WM_DELETE_WINDOW", self.ignore)
        # Set timeout and disable window decorations
        self.after(int(timeout*1000), self.timeout_destroy)
        self.update_idletasks()

    def okay(self):
        """Message confirmed, used with 'Okay' button"""
        self.response = True
        self.destroy()

    def timeout_destroy(self):
        """Destroy on timeout method, means the message was not confirmed."""
        if self.confirm:
            self.response = False
        self.destroy()

    def was_confirmed(self):
        """Getter for whether the message was confirmed"""
        return self.response

    def ignore(self):
        """Used to override close behavior of 'X' button"""
        pass
