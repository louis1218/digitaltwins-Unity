import zmq
from vosk import Model, KaldiRecognizer
import pyaudio
import qi

class ALSpeechRecognition():
    def __init__(self, sr_ip):
        self.sr_ip = sr_ip
        self.sr_context = zmq.Context()
        self.sr_socket = self.sr_context.socket(zmq.PUB)
        self.sr_socket.bind(sr_ip)
        self.msgpub = False

        self.model = Model("./speechToText/vosk_model")
        self.recognizer = KaldiRecognizer(self.model, 16000)

        self.mic = pyaudio.PyAudio()
        self.stream = self.mic.open(rate = 16000, channels =1 , format = pyaudio.paInt16, input = True, frames_per_buffer=8192)
        self.stream.start_stream()

    def getMsgPubStatus(self):
        return self.msgpub
    
    def HumanSpeech(self):
        self.msgpub = False
        data = self.stream.read(4096)
        if self.recognizer.AcceptWaveform(data):
            text = self.recognizer.Result()
            speech = text[14:-3]
            print(text)
            print(text[14:-3])
            self.sr_socket.send_string(speech)
            self.msgpub = True
    
