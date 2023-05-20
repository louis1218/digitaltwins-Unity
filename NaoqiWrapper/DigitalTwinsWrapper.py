#! /usr/local/bin/python3
import time
import io
import cv2
import numpy as np
import qi
import signal
import sys
from tkinter import *
import threading
from camera.ALVideoDeviceUnity import ALVideoDeviceUnity_RGBCamera
from camera.ALVideoDeviceUnity import ALVideoDeviceUnity_DepthCamera
from navigation.ALNavigationUnity import ALNavigationUnity
from motionPlanning.ALMotionUnity import ALMotionUnity
from speechToText.ALSpeechRecognition import ALSpeechRecognition
from speechToText.ALTextToSpeech import ALTextToSpeech


#from yolov8 import YOLOv8
#from yolov8.utils import draw_bounding_box_opencv
#from yolov8.utils import class_names as CLASSES

class Dialogue_TTS(threading.Thread):
    def __init__(self, qiApp):
        threading.Thread.__init__(self)
        self.app = qiApp
        
    def run(self):
        texttospeech = self.app.session.service("ALTextToSpeechUnity")
        speech_recg = self.app.session.service("ALSpeechRecognitionUnity")
        if (speech_recg.getMsgPubStatus):
            print(speech_recg.getMsgPubStatus())
            texttospeech.TextToSpeech()           

class Dialogue_STT(threading.Thread):
    def __init__(self, qiApp):
        threading.Thread.__init__(self)
        self.app = qiApp

    def run(self):
        speech_recg = self.app.session.service("ALSpeechRecognitionUnity")
        while True:
            # print(speech_recg.getMsgPubStatus())
            speech_recg.HumanSpeech()

class MotionPlanning(threading.Thread):
    def __init__(self, qiApp, motion_ip, motionPost_service):
        threading.Thread.__init__(self)
        self.motion_ip = motion_ip
        self.app = qiApp
        self.jointName = ""
        self.UnityjointValue = 0
        self.RealjointValue = 0
        self.motionPost_service = motionPost_service
    
    def setUnityPepperAngle(self, jointName, jointValue):
        self.jointName = jointName
        self.UnityjointValue = jointValue
        
    def setRealPepperAngle(self, jointName, jointValue):
        self.jointName = jointName
        self.RealjointValue = jointValue
        
    def run(self):
        
        unity_grasp = self.app.session.service("ALMotionUnity")
        unity_grasp.setUnityPepperAngle(self.jointName, self.UnityjointValue)
        unity_grasp.setRealPepperAngle(self.jointName, self.RealjointValue)
        unity_grasp.GrashGUI()


class Navigation(threading.Thread):
    def __init__(self, qiApp, cmd_vel_ip):
        threading.Thread.__init__(self)
        self.cmd_vel_ip = cmd_vel_ip
        self.app = qiApp

    def setVelocityUnity(self, x,y,theta):
        self.xUnity = x
        self.yUnity = y
        self.thetaUnity = theta
  
    def setVelocityReal(self, x,y,theta):
        self.xReal = x
        self.yReal = y
        self.thetaReal = theta
    
    def run(self):
        unity_cmd_vel = self.app.session.service("ALNavigationUnity_cmdvel")
        unity_cmd_vel.setVelocityUnity(self.xUnity, self.yUnity, self.thetaUnity)
        unity_cmd_vel.setVelocityReal(self.xReal, self.yReal, self.thetaReal)        
        unity_cmd_vel.ControlGUI()
        
class Camera(threading.Thread):
    def __init__(self, qiApp, rgb_ip, depth_ip, pepper_session, UNITY, PEPPER_CAMERA, video_service, videosClientRGB, videosClientDepth ):
        threading.Thread.__init__(self)
        self.rgb_ip = rgb_ip
        self.depth_ip = depth_ip
        self.app = qiApp
        self.pepper_session = pepper_session
        self.UNITY = UNITY
        self.PEPPER_CAMERA = PEPPER_CAMERA
        self.video_service = video_service
        self.videosClientRGB = videosClientRGB
        self.videosClientDepth = videosClientDepth
        
        #self.model = "/home/crossing/Desktop/naoqi_driver/NaoqiUnityWrapper/models/yolov8-l_640.onnx"
        #self.cv2_detector = cv2.dnn.readNetFromONNX(self.model)
        #self.yolov8_detector = YOLOv8(self.model, conf_thres=0.5, iou_thres=0.5)
            
    def UnityImgtoCV2(self, session):
        start_time = time.time()
        unityimg = session.getImageList()
        byte_img = io.BytesIO(bytearray(unityimg))
        numpy_img = np.asarray(bytearray(byte_img.read()), dtype=np.uint8)
        cv2_img = cv2.imdecode(numpy_img, cv2.IMREAD_UNCHANGED)
        camerainfo = session.getCameraInfo(cv2_img.shape[1], cv2_img.shape[0])
        resized = cv2.resize(cv2_img, (640,480), interpolation = cv2.INTER_AREA)
                
        return start_time, camerainfo, resized
    

    def detect_opencv(self, orig_image):
        time_1 = time.time()

        [height, width, _] = orig_image.shape
        length = max((height, width))
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = orig_image
        scale = length / 640

        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640))
        self.cv2_detector.setInput(blob)
        outputs = self.cv2_detector.forward()

        outputs = np.array([cv2.transpose(outputs[0])])
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2], outputs[0][i][3]]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)

        detections = []
        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]
            detection = {
                'class_id': class_ids[index],
                'class_name': CLASSES[class_ids[index]],
                'confidence': scores[index],
                'box': box,
                'scale': scale}
            detections.append(detection)
            img = draw_bounding_box_opencv(orig_image, class_ids[index], scores[index], round(box[0] * scale), round(box[1] * scale),
                            round((box[0] + box[2]) * scale), round((box[1] + box[3]) * scale))

        time_2=time.time()
        print("Detection time OPENCV:", time_2 - time_1)
        print("Object detected OPENCV: ", detections)
        return img   
        
    def run(self):
        unity_rgbCamera = self.app.session.service("ALVideoDeviceUnity_rgb")
        unity_depthCamera = self.app.session.service("ALVideoDeviceUnity_depth")
        # writer_unity = cv2.VideoWriter('unity_ycb.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (1280,960))
        # writer_real = cv2.VideoWriter('real_ycb.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (1280,960))


        while True:
            if (self.UNITY == 1):
                start_time_rgb, rgb_camerainfo, rgbresized = self.UnityImgtoCV2(unity_rgbCamera)


                # yolo_image_unity = self.detect_opencv(rgbresized)
                #print("RGB FPS: ", 1.0 / (time.time() - start_time_rgb))
                #print(rgb_camerainfo)
                #print("\n")
                
                time.sleep(0.05)

                start_time_depth, depth_camerainfo, depthresized = self.UnityImgtoCV2(unity_depthCamera)  
                
                depthresized = 255 - depthresized

                # print("Depth FPS: ", 1.0 / (time.time() - start_time_depth))
                # print(depth_camerainfo)
                # print("\n")
                
                time.sleep(0.05)
                
                unity_img = cv2.hconcat([rgbresized, depthresized])

                # writer_unity.write(unity_img)
                cv2.imshow("Unity Camera", unity_img)
                cv2.waitKey(1)
                time.sleep(0.05)
                
            if (self.PEPPER_CAMERA == 1):
                naoImageRGB = self.video_service.getImageRemote(self.videosClientRGB)
                imageWidth = naoImageRGB[0]
                imageHeight = naoImageRGB[1]
                array = naoImageRGB[6]
                frame = np.frombuffer(naoImageRGB[6], np.uint8).reshape(naoImageRGB[1], naoImageRGB[0], 3)
                pepper_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                
                # yolo_image_real = self.detect_opencv(pepper_rgb)

                naoImageDepth = self.video_service.getImageRemote(self.videosClientDepth)
                DepthimageWidth = naoImageDepth[0]
                DepthimageHeight = naoImageDepth[1]
                Deptharray = naoImageDepth[6]
                
                frameDepth = np.frombuffer(naoImageDepth[6], np.uint8).reshape(naoImageDepth[1], naoImageDepth[0], 3)
                pepper_gray = cv2.cvtColor(frameDepth, cv2.COLOR_BGR2GRAY)
                pepper_gray = cv2.resize(pepper_gray, (640 ,480))
                
                pepper_gray_stack = np.dstack((pepper_gray,pepper_gray))
                pepper_gray_stack = np.dstack((pepper_gray_stack,pepper_gray))
                
                pepper_img = cv2.hconcat([pepper_rgb, pepper_gray_stack])
                
                # writer_real.write(pepper_img)
                cv2.imshow("Pepper Camera", pepper_img)

                cv2.waitKey(1)
                time.sleep(0.05)

            
class NaoqiUnityWrapper():
    def __init__(self, qiApp, rgb_ip, depth_ip, cmd_vel_ip, motion_ip, stt_ip, tts_ip, pepper_session, UNITY, PEPPER_CAMERA,PEPPER_POSTURE,PEPPER_MOTION):
        self.rgb_ip = rgb_ip
        self.depth_ip = depth_ip
        self.cmd_vel_ip = cmd_vel_ip
        self.motion_ip = motion_ip
        self.stt_ip = stt_ip
        self.tts_ip = tts_ip
        self.pepper_session = pepper_session
        self.UNITY = UNITY
        self.PEPPER_CAMERA = PEPPER_CAMERA
        self.PEPPER_POSTURE = PEPPER_POSTURE
        self.PEPPER_MOTION = PEPPER_MOTION

        if (self.PEPPER_POSTURE == 1):
            self.initPostureNaoQi()
        if (self.PEPPER_MOTION == 1):
            self.initMotionNaoQi()
            
        if (self.PEPPER_CAMERA == 1):
            self.initCamerasNaoQi()
        if (self.PEPPER_CAMERA == 0):
            self.video_service = ""
            self.videosClientRGB = ""
            self.videosClientDepth = ""            
        
        qiApp.session.registerService("ALVideoDeviceUnity_rgb", ALVideoDeviceUnity_RGBCamera(self.rgb_ip))
        qiApp.session.registerService("ALVideoDeviceUnity_depth", ALVideoDeviceUnity_DepthCamera(self.depth_ip))
        qiApp.session.registerService("ALNavigationUnity_cmdvel", ALNavigationUnity(self.cmd_vel_ip, self.motion_service))
        qiApp.session.registerService("ALMotionUnity", ALMotionUnity(self.motion_ip, self.motionPost_service))
        qiApp.session.registerService("ALSpeechRecognitionUnity", ALSpeechRecognition(self.stt_ip))
        qiApp.session.registerService("ALTextToSpeechUnity", ALTextToSpeech(self.tts_ip))

        self.CameraThread = Camera(qiApp, rgb_ip, depth_ip, self.pepper_session, self.UNITY, self.PEPPER_CAMERA, self.video_service, self.videosClientRGB,self.videosClientDepth )
        self.NavigationThread = Navigation(qiApp, cmd_vel_ip)
        self.MotionPlanningThread = MotionPlanning(qiApp, motion_ip, self.motionPost_service)
        self.SpeechToTextThread = Dialogue_STT(qiApp)
        self.TextToSpeechThread = Dialogue_TTS(qiApp)

    def initCamerasNaoQi(self):
        self.video_service = self.pepper_session.service("ALVideoDevice")
        fps = 20
        resolution = 2  	# 2 = Image of 640*480px ; 3 = Image of 1280*960px
        colorSpace = 11  	# RGB
        resolutionD = 1
        colorSpaceD = 17

        if (self.PEPPER_CAMERA == 1):
            self.videosClientRGB = self.video_service.subscribeCamera("cameras_pepper", 0, resolution, colorSpace, fps)
            self.videosClientDepth = self.video_service.subscribeCamera("cameras_pepper", 2, resolutionD, colorSpace, fps)
        else:
            self.videosClientRGB = ""
            self.videosClientDepth = ""
    
    def initPostureNaoQi(self):
        self.motionPost_service = self.pepper_session.service("ALMotion")
        
        print(self.motionPost_service.getSummary())

        time.sleep(2.0)
    
    def initMotionNaoQi(self):
        self.motion_service = self.pepper_session.service("ALMotion")
        self.posture_service = self.pepper_session.service("ALRobotPosture")
        
    def cleanup(self):
        self.video_service.unsubscribe(self.videosClientRGB)
        self.video_service.unsubscribe(self.videosClientDepth)
        self.pepper_session.close()
        print("cleanup naoqi")

    def setUnityPepperAngle(self, jointName, jointValue):
        self.jointName = jointName
        self.UnityjointValue = jointValue
        
    def setRealPepperAngle(self, jointName, jointValue):
        self.jointName = jointName
        self.RealjointValue = jointValue
        
    def setVelocityUnity(self, x,y,theta):
        self.xUnity = x
        self.yUnity = y
        self.thetaUnity = theta
  
    def setVelocityReal(self, x,y,theta):
        self.xReal = x
        self.yReal = y
        self.thetaReal = theta
            
    def StartCamera(self):
        self.CameraThread.start()
        
    def StartNavigation(self):
        self.NavigationThread.setVelocityUnity(self.xUnity, self.yUnity, self.thetaUnity)
        self.NavigationThread.setVelocityReal(self.xReal, self.yReal, self.thetaReal)
        self.NavigationThread.start()
        
    def StartMotion(self):
        self.MotionPlanningThread.setUnityPepperAngle(self.jointName, self.UnityjointValue)
        self.MotionPlanningThread.setRealPepperAngle(self.jointName, self.RealjointValue)
        self.MotionPlanningThread.start()
        
    def StartSTT(self):
        self.SpeechToTextThread.start()
        
    def StartTTS(self):
        self.TextToSpeechThread.start()
 
    def signal_handler(self, sig, frame):
        self.cleanup()
        print('You pressed Ctrl+C!')
        sys.exit(0)
           
if __name__ == '__main__': 
    
    # use this ip for local computer only
    
    # rgb_ip = "tcp://127.0.0.1:5001"
    # depth_ip = "tcp://127.0.0.1:5002"
    # cmd_vel_ip = "tcp://127.0.0.1:5003"
    # motion_ip = "tcp://127.0.0.1:5004"
    # stt_ip = "tcp://127.0.0.1:5005"
    # tts_ip = "tcp://127.0.0.1:5006"

    # Assign your local network with the Windows laptop running Unity
    rgb_ip = "tcp://192.168.50.92:5001"
    depth_ip = "tcp://192.168.50.92:5002"
    cmd_vel_ip = "tcp://192.168.50.42:5003"
    motion_ip = "tcp://192.168.50.42:5009"
    stt_ip = "tcp://192.168.50.42:5005"
    tts_ip = "tcp://192.168.50.42:5006"    
    
    # Connect to your Pepper robot via qi session
    pepper_session = qi.Session()
    pepper_session.connect("tcp://192.168.50.44")
    
    # Choose whether you wan to access Virtual Robot or Physical Robot or both simultaneously
    UNITY = 1
    PEPPER = 1
    
    if PEPPER == 1:
        PEPPER_MOTION = 1
        PEPPER_POSTURE = 1
        PEPPER_CAMERA = 1
    
    qiApp = qi.Application(sys.argv)
    qiApp.start()
    wrap = NaoqiUnityWrapper(qiApp, rgb_ip,depth_ip,cmd_vel_ip, motion_ip, stt_ip, tts_ip, pepper_session, UNITY, PEPPER_CAMERA, PEPPER_POSTURE, PEPPER_MOTION)
    
    # To unplug the qi socket properly
    signal.signal(signal.SIGINT, wrap.signal_handler)
    
    # Start functions to access virtual and real Pepper
    wrap.StartCamera()
    wrap.setVelocityUnity(0.5,0,5) # linear x, linear y ,Theta # at the moment, Our virtual pepper in Unity is unable to move in Y axis.
    wrap.setVelocityReal(0.5,0,0.5) # linear x, linear y ,Theta
    wrap.StartNavigation()
    joint = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RWristYaw"]
    wrap.setUnityPepperAngle(joint, [-60,-10, 25, 104.5])
    wrap.setRealPepperAngle(joint, [0.2,-0.5, 0.2, 3.0])    
    wrap.StartMotion()
    
    # For speech to text only
    #wrap.StartSTT()
    #wrap.StartTTS()

    # Exit properly
    print('Press Ctrl+C')
    signal.pause()
    wrap.close()
