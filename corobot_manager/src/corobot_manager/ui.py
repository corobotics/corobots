from Tkinter import *

class CorobotUIMessage(Tk):

    def __init__(self, display_text, timeout, okay_text="Okay", confirm=False):
        Tk.__init__(self)

        self.confirm = confirm
        self.respond = False
        frame = Frame(self)
        frame.pack()
        label = Label(frame, text=display_text, font=("Helvetica", 24), wraplength=500)
        label.pack(padx=8, pady=8)
        if self.confirm:
            Button(frame, text=okay_text, command=self.okay).pack(padx=8, pady=8)

        width = max((self.winfo_screenwidth() / 2), self.winfo_width())
        height = max((self.winfo_screenwidth() / 4), self.winfo_height())
        
        xp = (self.winfo_screenwidth() / 2) - (width / 2) - 8
        yp = (self.winfo_screenheight() / 4) - (height / 2) - 20

        self.geometry("%dx%d%+d%+d" % (width, height, xp, yp))
        self.overrideredirect(1)
       
        self.update_idletasks()
        #Set timeout and disable window decorations
        self.after(int(timeout*1000), self.quit())
        self.mainloop()

    def okay(self):
        self.respond = True
        self.quit()

    def ui_destroy(self):
        if self.confirm:
            self.respond = False
 
        self.quit()

    def was_confirmed(self):
        return self.respond
            
