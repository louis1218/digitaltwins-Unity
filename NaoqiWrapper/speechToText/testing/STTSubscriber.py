import zmq
from vosk import Model, KaldiRecognizer
import pyaudio
import gtts
from playsound import playsound

host_sub = "127.0.0.1"
port_sub = "5556"
# Creates a socket instance
context_sub = zmq.Context()
socket_sub = context_sub.socket(zmq.REQ)
# Binds the socket to a predefined port on localhost
socket_sub.connect("tcp://{}:{}".format(host_sub, port_sub))

prev_data = ""

while True:
    socket_sub.send_string("request")
    poller = zmq.Poller()
    poller.register(socket_sub, zmq.POLLIN)
    evt = dict(poller.poll(10000))
    if evt:
        if evt.get(socket_sub) == zmq.POLLIN:
            data = socket_sub.recv(zmq.NOBLOCK)
            # print(data)
            if (str(data) != "b''" and data != prev_data):
                print(len(data))
                print(data)
                tts = gtts.gTTS(str(data))
                tts.save("pepper_message.mp3")
                playsound("pepper_message.mp3")
            prev_data = data