import zmq
from vosk import Model, KaldiRecognizer
import pyaudio


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5557")

model = Model("./vosk_model")
recognizer = KaldiRecognizer(model, 16000)

mic = pyaudio.PyAudio()
stream = mic.open(rate = 16000, channels =1 , format = pyaudio.paInt16, input = True, frames_per_buffer=8192)
stream.start_stream()

while True:
    
    data = stream.read(4096)
    
    if recognizer.AcceptWaveform(data):
        text = recognizer.Result()
        speech = text[14:-3]
        print(text)
        print(text[14:-3])
        socket.send_string(speech)



