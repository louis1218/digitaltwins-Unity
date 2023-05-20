using UnityEngine;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using System.Diagnostics;

public class NaoqiSpeechToTextSubscriber : MonoBehaviour
{
    private SpeechToTextNetMqSubscriber _SpeechToTextNetMqSubscriber;
    public bool Connected;
    public string pepper_message;
    public string human_message;


    public NaoqiSpeechToTextSubscriber(){}

    public string getHumanMessage(){
        return human_message;
    }

    public void setHumanMessage(string input){
        human_message = input;
    }

    private void HandleMessageSubscriber(string message)
    {

        setHumanMessage(message);
        var splitted_human_Strings = message.Split(' ');
        print(message);

        if ((string.Equals(splitted_human_Strings[0], "what")) && (string.Equals(splitted_human_Strings[splitted_human_Strings.Length-1], "doing"))){
                setPepperMessage("Hi I am virtual Pepper. I am here to show a virtual demonstration of myself in Unity Game Engine");
            }
        else{
            for(int i=0;i<splitted_human_Strings.Length;i++){
                if (string.Equals(splitted_human_Strings[i], "hi")){
                    setPepperMessage("Hi I am Pepper. I am virtual in Unity Game Engine.");
                }
                else{
                    setPepperMessage("");
                }
            }
        }
    }

    public void setPepperMessage(string input){
        pepper_message = input;
    }

    public string getPepperMessage(){
        return this.pepper_message;
    }

    void Start()
    {
        _SpeechToTextNetMqSubscriber = new SpeechToTextNetMqSubscriber(HandleMessageSubscriber);
        _SpeechToTextNetMqSubscriber.Start();
        print("SpeechToText Subscriber initialised");
    }


    void FixedUpdate()
    {
        _SpeechToTextNetMqSubscriber.Update();
    }

    private void OnDestroy()
    {
        _SpeechToTextNetMqSubscriber.Stop();
    }

}

public class SpeechToTextNetMqSubscriber
{
    private readonly Thread _listenerWorker;
    public bool _listenerCancelled;
    public delegate void MessageDelegate(string message);
    private readonly MessageDelegate _messageDelegate;
    private readonly ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();

    public SpeechToTextNetMqSubscriber(){}

    private void ListenerWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var subSocket = new SubscriberSocket())
        {
            subSocket.Options.ReceiveHighWatermark = 1000;
            subSocket.Connect("tcp://192.168.50.42:5005");
            subSocket.Subscribe("");
            while (!_listenerCancelled)
            {
                string frameString;
                if (!subSocket.TryReceiveFrameString(out frameString)) continue;
                _messageQueue.Enqueue(frameString);
                // UnityEngine.Debug.Log(frameString);
            }
            subSocket.Close();
        }
        NetMQConfig.Cleanup();
    }

    public void Update()
    {
        while (!_messageQueue.IsEmpty)
        {
            string message;
            if (_messageQueue.TryDequeue(out message))
            {
                _messageDelegate(message);
            }
            else
            {
                break;
            }
        }
    }

    public SpeechToTextNetMqSubscriber(MessageDelegate messageDelegate)
    {
        _messageDelegate = messageDelegate;
        _listenerWorker = new Thread(ListenerWork);
    }

    public void Start()
    {
        _listenerCancelled = false;
        _listenerWorker.Start();
    }

    public void Stop()
    {
        _listenerCancelled = true;
        _listenerWorker.Join();
    }
}