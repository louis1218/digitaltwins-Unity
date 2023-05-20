import zmq
from vosk import Model, KaldiRecognizer
import pyaudio
import gtts
from playsound import playsound
import qi
from . import ALSpeechRecognition 
import time


class ALTextToSpeech():
    def __init__(self, ip):
        self.ip = ip
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        # Binds the socket to a predefined port on localhost
        self.socket.connect(ip)
        self.socket.subscribe("")
        self.prev_data = ""

    def TextToSpeech(self):
        while True:
            poller = zmq.Poller()
            poller.register(self.socket, zmq.POLLIN)
            evt = dict(poller.poll(10000))
            if evt:
                if evt.get(self.socket) == zmq.POLLIN:
                    # time.sleep(0.05)
                    data = self.socket.recv(zmq.NOBLOCK)
            # data = self.socket.recv_string()
                    if (str(data) != "b''" and data != self.prev_data):
                        # print(len(data))
                        # print(data)
                        if (len(data)>0):
                            print("Virtual Pepper: ", str(data))
                            tts = gtts.gTTS(str(data))
                            tts.save("./speechToText/pepper_message.mp3")
                            playsound("./speechToText/pepper_message.mp3")
                        else:
                            print("No message from Virtual Pepper")     
                    self.prev_data = data           
            
            # self.socket.send_string("request")
            # poller = zmq.Poller()
            # poller.register(self.socket, zmq.POLLIN)
            # evt = dict(poller.poll(10000))
            # if evt:
            #     if evt.get(self.socket) == zmq.POLLIN:
            #         data = self.socket.recv(zmq.NOBLOCK)
            #         # print(data)
            #         if (str(data) != "b''" and data != self.prev_data):
            #             print(len(data))
            #             print(data)
            #             tts = gtts.gTTS(str(data))
            #             tts.save("./speechToText/pepper_message.mp3")
            #             playsound("./speechToText/pepper_message.mp3")
            #         self.prev_data = data
