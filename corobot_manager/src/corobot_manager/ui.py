import Tkinter

class CorobotUIMessage(Tkinter.Tk):

    def __init__(self, display_text, timeout, okay_text="Okay", confirm_pub=None):
        CorobotUIMessage.__init__(self, display_text, timeout)

        self.respond = confirm_pub

        if self.respond is not None:
            Button(self, text=okay_text, command=self.okay).pack(side=LEFT)

        #Set timeout and disable window decorations
        self.after(int(timeout*1000), self.ui_destroy())
        self.overrideredirect(1)

    def okay(self):
        self.respond.publish(confirm="TRUE")
        self.destroy()

    def ui_destroy(self):
        if self.respond is not None:
            self.respond.publish(confirm="FALSE")
        self.destroy()
            
