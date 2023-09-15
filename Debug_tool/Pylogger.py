
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from drawnow import drawnow
class plotter():
    
    def __init__(self,controller):

        self.controller = controller
        self.x_vals = [i for i in range(200)]
        self.y_vals = [0 for i in range(200)]
        self.y1_vals = [0 for i in range(200)]
        self.y2_vals = [0 for i in range(200)]
        self.zero_vals=[0 for i in range(200)]   
  


    def animate(self):
        plt.style.use('fivethirtyeight')
        plt.tight_layout() 
        self.ax = plt.subplot(311)
        self.ax.set_title("X_Error")
        self.ax.axes.get_xaxis().set_visible(False)
        self.ax1 = plt.subplot(312)
        self.ax1.set_title("X_Error")
        self.ax1.axes.get_xaxis().set_visible(False)
        self.ax2 = plt.subplot(313)
        self.ax2.set_title("X_Error")
        self.ax2.axes.get_xaxis().set_visible(False)             
        self.y_vals.pop(0)
        self.y1_vals.pop(0)
        self.y2_vals.pop(0)
        self.y_vals.append(self.controller.x_error)
        self.y1_vals.append(self.controller.y_error)
        self.y2_vals.append(self.controller.z_error)
        self.ax.plot(self.x_vals,self.y_vals)
        self.ax.plot(self.x_vals,self.zero_vals)  
        self.ax.set_title("X Error",fontsize=15)
        self.ax1.plot(self.x_vals,self.y1_vals)  
        self.ax1.plot(self.x_vals,self.zero_vals)  
        self.ax1.set_title("Y Error",fontsize=15)     
        self.ax2.plot(self.x_vals,self.y2_vals)
        self.ax2.plot(self.x_vals,self.zero_vals)  
        self.ax2.set_title("Z Error",fontsize=15)
        self.ax.set_facecolor('#DEDEDE')
        self.ax1.set_facecolor('#DEDEDE')
        self.ax2.set_facecolor('#DEDEDE')
        plt.show() 
    def __call__(self):
        plt.ion()
        drawnow(self.animate) 
       

        

      