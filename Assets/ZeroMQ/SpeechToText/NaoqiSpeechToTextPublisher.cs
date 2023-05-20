using UnityEngine;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using System.Diagnostics;

public class NaoqiSpeechToTextPublisher : MonoBehaviour
{
    public bool Connected;
    public SpeechToTextNetMqPublisher _SpeechToTextNetMqPublisher;
    private NaoqiSpeechToTextSubscriber naoqiSST;
    public string human_messageFromSubscriber;
    public string pepper_messageFromSubscriber;

    public string getHumanMessage(){
        return human_messageFromSubscriber;
    }

    public void setHumanMessage(string input){
        human_messageFromSubscriber = input;
    }

    public string getPepperMessage(){
        return pepper_messageFromSubscriber;
    }

    public void setPepperMessage(string input){
        pepper_messageFromSubscriber = input;
    }

    public NaoqiSpeechToTextPublisher(){}

    // public string getMessage(){
    //     return messageFromSubscriber;
    // }

    public string HandleMessage(string message)
    {
        return pepper_messageFromSubscriber;
    }

    public void Start()
    {
        _SpeechToTextNetMqPublisher = new SpeechToTextNetMqPublisher(HandleMessage);
        // _SpeechToTextNetMqPublisher = new SpeechToTextNetMqPublisher();
        _SpeechToTextNetMqPublisher.Start();
    }


    void Update()
    {
        string pep_words = gameObject.GetComponent<NaoqiSpeechToTextSubscriber>().pepper_message;
        setPepperMessage(pep_words);
        // print(pepper_messageFromSubscriber);

        string human_words = gameObject.GetComponent<NaoqiSpeechToTextSubscriber>().human_message;
        setHumanMessage(human_words);

        // print(human_messageFromSubscriber);
    }

    public void OnDestroy()
    {
        _SpeechToTextNetMqPublisher.Stop();
    }
}

public class SpeechToTextNetMqPublisher
{
    private readonly Thread _listenerWorker;
    private bool _listenerCancelled;
    public delegate string MessageDelegate(string message);

    private readonly MessageDelegate _messageDelegate;
    private readonly Stopwatch _contactWatch;
    private const long ContactThreshold = 1000;
    public bool Connected;
    public NaoqiSpeechToTextPublisher _NaoqiSpeechToTextPublisher;

    private void ListenerWork()
    {
        
        AsyncIO.ForceDotNet.Force();
        using (var server = new PublisherSocket())
        {
            server.Bind("tcp://*:5006");
            _NaoqiSpeechToTextPublisher = new NaoqiSpeechToTextPublisher();

            while (true)
            {
                var response = _messageDelegate("checking");
                // UnityEngine.Debug.Log(response);
                // var msg = _NaoqiSpeechToTextPublisher.pepper_messageFromSubscriber;
                // UnityEngine.Debug.Log(msg);
                // UnityEngine.Debug.Log(_NaoqiSpeechToTextPublisher.getPepperMessage());
                server.SendFrame(response);
            }
        }
        NetMQConfig.Cleanup();
    }


    public SpeechToTextNetMqPublisher(MessageDelegate messageDelegate)
    {
        _messageDelegate = messageDelegate;
        _contactWatch = new Stopwatch();
        _contactWatch.Start();
        _listenerWorker = new Thread(ListenerWork);
    }

    // public SpeechToTextNetMqPublisher(){
    //     _listenerWorker = new Thread(ListenerWork);
    // }

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




