import qi
import numpy as np
import time
import zmq
import sys
import zmq
from tkinter import *

class ALMotionUnity():
    def __init__(self, hw_ip, motionPost_service):
        self.hw_ip = hw_ip
        self.hw_context = zmq.Context()
        self.hw_socket = self.hw_context.socket(zmq.PUB)
        self.hw_socket.bind(self.hw_ip)
        self.motionPost_service = motionPost_service
    
    def setUnityPepperAngle(self, jointName, jointValue):
        self.jointName = jointName
        self.UnityjointValue = jointValue
        
    def setRealPepperAngle(self, jointName, jointValue):
        self.jointName = jointName
        self.RealjointValue = jointValue
     
    def RealPepperOpenHand(self):
        self.motionPost_service.openHand('RHand')

    def RealPepperClosedHand(self):
        self.motionPost_service.openHand('RHand')
         
    def RealPepperMotion(self):
        fractionMaxSpeed  = 0.1
        for i in range(len(self.jointName)):
            self.motionPost_service.setAngles(self.jointName[i], self.RealjointValue[i], fractionMaxSpeed)
            
    def RealPepperOrigin(self):
        fractionMaxSpeed  = 0.1
        origin_angle = [1.3, 0, 0.2, 0]
        for i in range(len(self.jointName)):
            self.motionPost_service.setAngles(self.jointName[i], origin_angle[i], fractionMaxSpeed)       
    
    def UnityPepperMotion(self):
        for i in range(len(self.jointName)):
            self.hw_socket.send_string("motion" + " " + str((self.jointName[i])) + " " + str((self.UnityjointValue)[i]))
            time.sleep(2)
            
    def UnityPepeprOpenHand(self):
        rightFinger = "RThumb1"
        self.hw_socket.send_string("motion" + " " +rightFinger + " " + str(-75))

    def UnityPepperCloseHand(self):
        self.motionPost_service.closeHand('RHand')
        rightFinger = "RThumb1"
        self.hw_socket.send_string("motion" + " " +rightFinger + " " + str(1))
   
    def UnityPepperOrigin(self):
        for i in range(len(self.jointName)):
            self.hw_socket.send_string("origin" + " " + str((self.jointName[i])) + " " + str(0))

    def combinedMotionFunction(self,evt=None):
        self.RealPepperMotion()
        self.UnityPepperMotion()

    def combinedOpenHandFunction(self,evt=None):
        self.UnityPepeprOpenHand()
        self.RealPepperOpenHand()

    def combinedClosedHandFunction(self,evt=None):
        self.UnityPepperCloseHand()
        self.RealPepperClosedHand()
        
    def combinedOriginFunction(self, evt=None):
        self.UnityPepperOrigin()
        self.RealPepperOrigin()
        
    # def DroppingSender(self):
    #     pos_bicep_angle = 60
    #     motionType = 3
    #     self.hw_socket.send_string(str(motionType) + " " + str(pos_bicep_angle))
        
    # def ReturnArmToOrigin(self):
    #     motionType = 4 
    #     self.hw_socket.send_string(str(motionType))
           
    def GrashGUI(self):
        master= Tk()
        master.title("Unity Motion Console")
        master.geometry("300x200")
        button1= Button(master, text="Raise arm", command = self.combinedMotionFunction)
        button1.place(x=25, y=50)
        button2= Button(master, text="Open Hand", command = self.combinedOpenHandFunction)
        button2.place(x=25, y=100)
        button3= Button(master, text="Close Hand", command = self.combinedClosedHandFunction)
        button3.place(x=150, y=100)   
        button4= Button(master, text="Return to Origin", command = self.combinedOriginFunction)
        button4.place(x=150, y=50)           
        
        master.mainloop()
