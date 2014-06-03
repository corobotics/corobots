from Tkinter import *
#from corobot import Robot
import math

class CorobotMonitorUI(Tk):

    def __init__(self):
        """ Corobot general info monitor 
        """
        Tk.__init__(self)
        self.title("Corobot Monitor")

        self.frame = Frame(self)
        self.frame.pack(padx=8, pady=8)

        self.label = Label(self.frame, text="Current Pose", font=("Helvetica", 24))
        self.label.pack(side='top')

        self.xinfo = Label(self.frame, text="X: 0.0", font=("Helvetica", 24))
        self.xinfo.pack()
        self.yinfo = Label(self.frame, text="Y: 0.0", font=("Helvetica", 24))
        self.yinfo.pack()
        self.thinfo = Label(self.frame, text="Theta: 0.0", font=("Helvetica", 24))
        self.thinfo.pack()
	self.covinfo = Label(self.frame, text="StDev: (0.0, 0.0, 0.0)", font=("Helvetica",18))
	self.covinfo.pack()

        self.absGoalInfo = Label(self.frame, text="AbsGoal: -", font=("Helvetica", 24))
        self.absGoalInfo.pack()

        self.rawnavinfo = Label(self.frame, text="RawNav: -", font=("Helvetica", 24))
        self.rawnavinfo.pack()

        self.velCmdInfo = Label(self.frame, text="ActVel: -", font=("Helvetica", 24))
        self.velCmdInfo.pack()

        self.obsinfo = Label(self.frame, text="ObsLoc: -", font=("Helvetica", 24))
        self.obsinfo.pack()

        self.netForceInfo = Label(self.frame, text="NetForce: -", font=("Helvetica", 24))
        self.netForceInfo.pack()

        self.qrCountInfo = Label(self.frame, text="QRLeft: 0 ; QRRight: 0", font=("Helvetica", 24))
        self.qrCountInfo.pack()

        self.batteryInfo = Label(self.frame, text="Battery: -", font=("Helvetica", 24))
        self.batteryInfo.pack()

        self.laptopBatteryInfo = Label(self.frame, text="Laptop Battery: -", font=("Helvetica", 24))
        self.laptopBatteryInfo.pack()

        self.recoveryInfo = Label(self.frame, text="Recovery: -", font=("Helvetica", 24))
        self.recoveryInfo.pack()

        self.tBox = Entry(self.frame, width=10)
        self.tBox.pack()

        #Button(self.frame, text = 'Submit', command = navigate).pack(side=LEFT)

    def navigate():
        pass
    """with Robot("127.0.0.1", 15001) as r:
            r.nav_to(self.tBox.get())"""

    def setPose(self, x, y, theta, cov):
        self.xinfo.configure(text="X: {0:6.3f}".format(x))
        self.yinfo.configure(text="Y: {0:6.3f}".format(y))
        self.thinfo.configure(text="Theta: {0:+4.2f}".format(theta))
	self.covinfo.configure(text="StDev: ({0:6.3g},{1:6.3g},{2:6.3g})".format(math.sqrt(cov[0]),math.sqrt(cov[4]),math.sqrt(cov[8])))

    def setRawnavMsg(self, txt):
        txt = "RawNav: " + txt
        self.rawnavinfo.configure(text=txt)

    def setObsMsg(self, txt):
        txt = "Obs: " + txt
        self.obsinfo.configure(text=txt)

    def setAbsGoalMsg(self, txt):
        txt = "AbsGoal: " + txt
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
        # ui elements.
        xp = (self.winfo_screenwidth() / 2) - (width / 2)
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
