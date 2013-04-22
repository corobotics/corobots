from Tkinter import *

class CorobotUIMessage(Tk):

    def __init__(self, display_text, timeout, okay_text="Okay", confirm=False):
        Tk.__init__(self)

        self.confirm = confirm
        frame = Frame(self)
        frame.pack()
        label = Label(frame, text=display_text)
        label.pack()
        if self.confirm:
            Button(frame, text=okay_text, command=self.okay).pack(side=LEFT)

        #Set timeout and disable window decorations
        self.after(int(timeout*1000), self.ui_destroy())
#        self.overrideredirect(1)

    def okay(self):
        self.respond = True
        self.destroy()

    def ui_destroy(self):
        if self.confirm:
            self.respond = False
        self.destroy()
            
