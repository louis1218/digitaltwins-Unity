#! /usr/local/bin/python3
import zmq
import time
import random
import io
import cv2
import numpy as np
import qi
import sys
from tkinter import *
import threading

class ALNavigationUnity:
	def __init__(self, ip, motion_service):
		print("__init__ in cmd_vel")
		self.ip = ip
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.PUB)
		self.socket.bind(self.ip)
		self.motion_service = motion_service

	def setVelocityUnity(self, x,y,theta):
		self.xUnity = x
		self.yUnity = y
		self.thetaUnity = theta

	def setVelocityReal(self, x,y,theta):
		self.xReal = x
		self.yReal = y
		self.thetaReal = theta
			
	def UnitySocketSender(self, x, y, theta):
		vector3 = str(x)+" "+str(y)+ " "+str(theta)
		self.socket.send_string(str(vector3))
  
	def RealPepperCmd(self, x, y , theta):
		self.motion_service.moveToward(x, y, theta)
		time.sleep(0.5)
		self.motion_service.stopMove()
  
	def forwardUnityCmd(self):
		start_time = time.time()

		while (time.time() - start_time < 1.3):
			print(" V Pepper moving forward")
			self.UnitySocketSender(self.xUnity,0,0)
		
	def leftUnityCmd(self):
		start_time = time.time()
		while (time.time() - start_time < 1.3):
			print(" V Pepper turning left")
			self.UnitySocketSender(0,0,(self.thetaUnity*-1))
		
	def rightUnityCmd(self):
		start_time = time.time()
		while (time.time() - start_time < 1.3):
			print(" V Pepper turning right")
			self.UnitySocketSender(0,0,self.thetaUnity)
  
	def navigationUnityCmd(self):
		start_time = time.time()
		while (time.time() - start_time < 10):
			print(" V Pepper moving forward")
			self.UnitySocketSender(self.xUnity,0,0)
   
		start_time = time.time()
		while (time.time() - start_time < 2):
			print(" V Pepper moving forward")
			self.UnitySocketSender(0,0,self.thetaUnity)
   
	def navigationRealCmd(self):
		self.motion_service.moveToward(self.xReal, 0, 0)
		time.sleep(10)
		self.motion_service.stopMove()

		self.motion_service.moveToward(0, 0, (self.thetaReal*-1))
		time.sleep(1.5)
		self.motion_service.stopMove()
  
	def forwardRealCmd(self):
		self.RealPepperCmd(self.xReal,0,0)
		
	def leftRealCmd(self):
		self.RealPepperCmd(0,0,(self.thetaReal))
		
	def rightRealCmd(self):
		self.RealPepperCmd(0,0,(self.thetaReal*-1))
  
	def combinedNavigationFunction(self, evt=None):
		p1 = threading.Thread(target=self.navigationUnityCmd)
		p2 = threading.Thread(target=self.navigationRealCmd)
		p1.start()
		p2.start()     
  
	def combinedForwardFunction(self,evt=None):
     
		p1 = threading.Thread(target=self.forwardUnityCmd)
		p2 = threading.Thread(target=self.forwardRealCmd)
		p1.start()
		p2.start()
		
	def combinedLeftFunction(self,evt=None):
		p1 = threading.Thread(target=self.leftUnityCmd)
		p2 = threading.Thread(target=self.leftRealCmd)
		p1.start()
		p2.start()

	def combinedRightFunction(self,evt=None):
		p1 = threading.Thread(target=self.rightUnityCmd)
		p2 = threading.Thread(target=self.rightRealCmd)
		p1.start()
		p2.start()

	def ControlGUI(self):
		master= Tk()
		master.title("Unity Navigation Console")
		master.geometry("300x200")
		button1= Button(master, text="Forward", command = self.combinedForwardFunction)
		button1.place(x=100, y=25)

		button2=Button(master, text="Left", command = self.combinedLeftFunction)
		button2.place(x=50, y=100)

		button3=Button(master, text="Right", command = self.combinedRightFunction)
		button3.place(x=175, y=100)
  
		button4=Button(master, text="Navigate", command = self.combinedNavigationFunction)
		button4.place(x=100, y=150)
  
  		
		master.mainloop()
   
# if __name__ == "__main__":
	
# 	app = qi.Application(sys.argv)
# 	cmd_vel_ip = "tcp://127.0.0.1:5003"
# 	app.start()
# 	app.session.registerService("ALNavigationUnity_cmdvel", ALNavigationUnity(cmd_vel_ip))
# 	app.run() 
