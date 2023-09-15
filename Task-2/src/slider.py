import tkinter as tk
import threading
import numpy as np

#tkinder slider
class SliderInput(tk.Tk):
    def __init__(self, name = 'None', controller = None):
        super().__init__()
        self.name=  name   
        self.controller = controller          #controller object 
        self.sliderLength = 10      #length of slider
        self.range = [0, 10000]     #range of slider
        self.title(name)    #set window title
        self.length = 500   #set window size
        self.geometry(str(int(self.length + 100))+"x200")       #set window size
        
        try:
            self.prev_values()
        except:
            print('No previous val found ', name)
            self.curr_val = [0,0,0]

        self.slider1 = tk.Scale(self, from_=self.range[0], to=self.range[1], orient="horizontal", sliderlength=self.sliderLength, label= 'kp', length=self.length)
        self.slider1.set(self.curr_val[0])      #set initial value
        self.slider1.pack()            #pack to make visible
        self.slider1.bind("<ButtonRelease-1>", self.updateParam)         #bind to update function

        self.slider2 = tk.Scale(self, from_=self.range[0], to=self.range[1], orient="horizontal", sliderlength=self.sliderLength, label= 'kd', length=self.length)
        self.slider2.set(self.curr_val[1])       #set initial value
        self.slider2.pack()         #pack to make visible
        self.slider2.bind("<ButtonRelease-1>", self.updateParam)         #bind to update function

        self.slider3 = tk.Scale(self, from_=self.range[0], to=self.range[1], orient="horizontal", sliderlength=self.sliderLength, label= 'ki', length=self.length)
        self.slider3.set(self.curr_val[2])       #set initial value
        self.slider3.pack()          #pack to make visible
        self.slider3.bind("<ButtonRelease-1>", self.updateParam)         #bind to update function

        print("slider init done "+ name)

    #update function to updateParam the value from slider
    def updateParam(self, *args):
        self.curr_val = np.array([self.slider1.get(), self.slider2.get(), self.slider3.get()])
        self.controller.update_K(self.curr_val/10, self.name)       #update PID gains in controller object from slider
        self.save_values()
        

    #save values to file using numpy
    def save_values(self):
        np.save(self.name + 'saved.npy', self.curr_val)

    #load values from file using numpy
    def prev_values(self):
        k_value=np.load(self.name + 'saved.npy')
        self.curr_val = k_value
        print(self.curr_val, 'prev val')
        self.controller.update_K(self.curr_val/10, self.name)    #update PID gains in controller object from slider

#target function for thread
def start_slider(name, controller):
    sr = SliderInput(name, controller)
    sr.mainloop()

if __name__ == "__main__":
    threading.Thread(target=start_slider).start()
