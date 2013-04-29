from Tkinter import *

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
