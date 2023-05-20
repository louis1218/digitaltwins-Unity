import argparse
import queue
import sys
import sounddevice as sd
import qi
import zmq
from vosk import Model, KaldiRecognizer

class VOSKSpeechToText():
    def __init__(self, args, model, dump_fn):
        self.q = queue.Queue()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5557")
        self.speech = ""
        self.args = args
        self.model = model
        self.dump_fn = dump_fn

    def int_or_str(self,text):
        """Helper function for argument parsing."""
        try:
            return int(text)
        except ValueError:
            return text

    def callback(self,indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))

    # host = "127.0.0.1"
    # port = "5555"
    # # Creates a socket instance
    # context = zmq.Context()
    # socket = context.socket(zmq.PUB)
    # # Binds the socket to a predefined port on localhost
    # socket.bind("tcp://{}:{}".format(host, port))


    # host_sub = "127.0.0.1"
    # port_sub = "5556"
    # # Creates a socket instance
    # context_sub = zmq.Context()
    # socket_sub = context_sub.socket(zmq.SUB)
    # # Binds the socket to a predefined port on localhost
    # socket_sub.connect("tcp://{}:{}".format(host_sub, port_sub))


        # if (len(message)==0):
        #     break
        # else:
        #     continue


    # parser = argparse.ArgumentParser(add_help=False)
    # parser.add_argument(
    #     "-l", "--list-devices", action="store_true",
    #     help="show list of audio devices and exit")
    # args, remaining = parser.parse_known_args()
    # if args.list_devices:
    #     print(sd.query_devices())
    #     parser.exit(0)
    # parser = argparse.ArgumentParser(
    #     description=__doc__,
    #     formatter_class=argparse.RawDescriptionHelpFormatter,
    #     parents=[parser])
    # parser.add_argument(
    #     "-f", "--filename", type=str, metavar="FILENAME",
    #     help="audio file to store recording to")
    # parser.add_argument(
    #     "-d", "--device", type=int_or_str,
    #     help="input device (numeric ID or substring)")
    # parser.add_argument(
    #     "-r", "--samplerate", type=int, help="sampling rate")
    # parser.add_argument(
    #     "-m", "--model", type=str, help="language model; e.g. en-us, fr, nl; default is en-us")
    # args = parser.parse_args(remaining)

    # def initialiseRawInputStream(self):
    #     with sd.RawInputStream(samplerate=self.args.samplerate, blocksize = 8000, device=self.args.device,
    #             dtype="int16", channels=1, callback=self.callback):
    #         print("#" * 80)
    #         print("Press Ctrl+C to stop the recording")
    #         print("#" * 80)
            
            
    def getSpeechData(self):
        return self.speech

    def SpeechPublisher(self, rec):
        # try:
            # while True:
        
        data = self.q.get()
        if rec.AcceptWaveform(data):
            print("Accepting Data")
        else:
            print(rec.PartialResult())
            self.speech = (rec.PartialResult())[15:]
            # print(self.speech)
            # socket.send_string(speech_data)
            # speechFromPepper = socket_sub.recv()
            # print(speechFromPepper)
            self.socket.send_string(self.speech)
        # if self.dump_fn is not None:
        #     self.dump_fn.write(data)
    
        # socket.send(b"World")

                # speechFromPepper = socket_sub.recv()
                # print(speechFromPepper)

    # except KeyboardInterrupt:
    #     print("\nDone")
    #     self.parser.exit(0)
    # except Exception as e:
    #     self.parser.exit(type(e).__name__ + ": " + str(e))


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "-l", "--list-devices", action="store_true",
        help="show list of audio devices and exit")
    args, remaining = parser.parse_known_args()
    if args.list_devices:
        print(sd.query_devices())
        parser.exit(0)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[parser])
    parser.add_argument(
        "-f", "--filename", type=str, metavar="FILENAME",
        help="audio file to store recording to")
    parser.add_argument(
        "-d", "--device", type=str,
        help="input device (numeric ID or substring)")
    parser.add_argument(
        "-r", "--samplerate", type=int, help="sampling rate")
    parser.add_argument(
        "-m", "--model", type=str, help="language model; e.g. en-us, fr, nl; default is en-us")
    args = parser.parse_args(remaining)
    
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, "input")
        # soundfile expects an int, sounddevice provides a float:
        args.samplerate = int(device_info["default_samplerate"])
        
        print (device_info)
    if args.model is None:
        model = Model(lang="en-us")
    else:
        model = Model(lang=args.model)

    if args.filename:
        dump_fn = open(args.filename, "wb")
    else:
        dump_fn = None
            
    sst = VOSKSpeechToText(args, model, dump_fn)

    data = ""
    
    with sd.RawInputStream(samplerate=args.samplerate, blocksize = 8000, device=args.device,
    dtype="int16", channels=1, callback=sst.callback):
    
        print("#" * 80)
        print("Press Ctrl+C to stop the recording")
        print("#" * 80)
        rec = KaldiRecognizer(model, args.samplerate)
        while True:
            print(len(data))
            sst.SpeechPublisher(rec)
            
            data = sst.getSpeechData()
            # if (len(data) % 50 == 0):
            #     data = data[50:]
            # else:
            #     continue
            print(data[15:-3])
            if (len(data) % 50 == 0):
                rec = KaldiRecognizer(model, args.samplerate)
                continue
