#! /usr/local/bin/python3
import zmq
import time
import numpy as np
import qi

class ALVideoDeviceUnity_RGBCamera:
	def __init__(self, ip):
		print("__init__ in ALVideoDeviceUnity_rgb")
		self.ip = ip
		self.unity_image_data = []
  
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.REQ)
		self.socket.connect(self.ip)
		# self.socket.subscribe("")
		self.TIMEOUT = 10000
  
	def getCameraInfo(self, width, height):
		if (width == 640 and height == 480):
			CameraInfo = {"CameraType": "rgbcamera", "Width": 640, "Height":480, "distortion_model": "plumb_bob", "d": [0.0,0.0,0.0,0.0,0.0], "k": [360.54601759654855, 0.0, 320.0, 0.0,360.54601759654855, 240.0,0.0,0.0,1.0],
                 "r": [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], "p": [360.54601759654855, 0.0, 320.0, -0.0, 0.0, 360.54601759654855, 240.0,0.0,0.0,0.0,1.0,0.0]}
			return CameraInfo
		elif (width == 320 and height == 240):
			CameraInfo = {"CameraType": "rgbcamera", "Width": 320, "Height":240, "distortion_model": "plumb_bob", "d": [0.0,0.0,0.0,0.0,0.0], "k": [180.27300879827428, 0.0, 160.0, 0.0,180.27300879827428, 120.0,0.0,0.0,1.0],
                 "r": [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], "p": [180.27300879827428, 0.0, 160.0, -0.0, 0.0, 180.27300879827428, 120.0,0.0,0.0,0.0,1.0,0.0]}		
			return CameraInfo	
		else:
			print("resolution size is not compatible")

   
	def getImageList(self):
		
		self.getImageRemote()
  
		return self.unity_image_data
			
		
	def getImageRemote(self):
 
		time_start = time.time()
		self.socket.send_string("request_rgb")
		time.sleep(0.03)
		# poller = zmq.Poller()
		# poller.register(self.socket, zmq.POLLIN)
		# evt = dict(poller.poll(self.TIMEOUT))
		# if evt:
			# if evt.get(self.socket) == zmq.POLLIN:
		unity_byte_data = self.socket.recv()
		if unity_byte_data != None: # if NEW data has been received since last ReadReceivedData function call
			self.unity_image_data = list(unity_byte_data)
			time_end = time.time()
			time_diff = time_end-time_start
	 
class ALVideoDeviceUnity_DepthCamera:
	def __init__(self, ip):
		print("__init__ in ALVideoDeviceUnity_depth")
		self.ip = ip
		self.unity_image_data = []
  
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.REQ)
		self.socket.connect(self.ip)
		self.TIMEOUT = 10000

	def getCameraInfo(self, width, height):
		if (width == 640 and height == 480):
			CameraInfo = {"CameraType": "depthcamera", "Width": 640, "Height":480, "distortion_model": "plumb_bob", "d": [0.0,0.0,0.0,0.0,0.0], "k": [415.69219771027923, 0.0, 320.0, 0.0,415.69219771027923, 240.0,0.0,0.0,1.0],
                 "r": [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], "p": [415.69219771027923, 0.0, 320.0, -0.0, 0.0, 415.69219771027923, 240.0,0.0,0.0,0.0,1.0,0.0]}
			return CameraInfo
		elif (width == 320 and height == 240):
			CameraInfo = {"CameraType": "depthcamera", "Width": 320, "Height":240, "distortion_model": "plumb_bob", "d": [0.0,0.0,0.0,0.0,0.0], "k": [207.84609885513962, 0.0, 160.0, 0.0,207.84609885513962, 120.0,0.0,0.0,1.0],
                 "r": [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], "p": [207.84609885513962, 0.0, 160.0, -0.0, 0.0, 207.84609885513962, 120.0,0.0,0.0,0.0,1.0,0.0]}			
			return CameraInfo		
		else:
			print("resolution size is not compatible")
  
	def getImageList(self):
		
		self.getImageRemote()
  
		return self.unity_image_data
			
		
	def getImageRemote(self):

		time_start = time.time()
		self.socket.send_string("request_depth")
		time.sleep(0.05)
		# poller = zmq.Poller()
		# poller.register(self.socket, zmq.POLLIN)
		# evt = dict(poller.poll(self.TIMEOUT))
		# if evt:
			# if evt.get(self.socket) == zmq.POLLIN:
		unity_byte_data = self.socket.recv()
		if unity_byte_data != None: # if NEW data has been received since last ReadReceivedData function call
			self.unity_image_data = list(unity_byte_data)
			time_end = time.time()
			time_diff = time_end-time_start
   
# if __name__ == "__main__":
	
# 	app = qi.Application(sys.argv)
# 	rgb_ip = "tcp://127.0.0.1:5001"
# 	depth_ip = "tcp://127.0.0.1:5002"
# 	app.start()
# 	app.session.registerService("ALVideoDeviceUnity_rgb", ALVideoDeviceUnity_rgb(rgb_ip))
# 	app.session.registerService("ALVideoDeviceUnity_depth", ALVideoDeviceUnity_depth(depth_ip))
# 	app.run()  
