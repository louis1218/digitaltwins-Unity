using System.Diagnostics;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;
using System;
using System.Collections.Generic;
using UnityEngine.Serialization;
using System.Collections;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using Unity.Robotics.Core;
using UnityEngine.Rendering;
using System.IO;

public class ImageStorage
{
   public byte[] rgb_image;
} 

public class RGBCamera : MonoBehaviour
{
    // The game object
    public Camera ImageCamera;
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    [Range(0, 100)]
    public int qualityLevel = 50;
    private Texture2D texture2D;
    private Rect rect;
    // Publish the cube's position and rotation every N seconds
    private float publishMessageFrequency = 0.05f;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    public bool Connected;
    private NetMqPublisher _netMqPublisher;
    // private UnityCamera _camera;
    private byte[] _response;
    // public ImageStorage image_storage;


    private void Start()
    {
        // _camera = new UnityCamera(ImageCamera, resolutionWidth, resolutionHeight, qualityLevel, 
        // texture2D, rect, publishMessageFrequency, timeElapsed);
        // _camera.Start();

        // Initialize game Object
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        Camera.onPostRender += UpdateImage;
    
        _netMqPublisher = new NetMqPublisher(HandleMessage);
        _netMqPublisher.Start();
        print("RGBCamera initialised");

    }

    public void UpdateImage(Camera _camera)
    {
        if (texture2D != null && _camera == this.ImageCamera)
            UpdateMessage();
    }

    public void UpdateMessage()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            texture2D.ReadPixels(rect, 0, 0);
            // print("RGB Camera Connected");
            _response = texture2D.EncodeToJPG(qualityLevel);

            // Camera Info message
            //CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, message.header, 0.0f, 0.01f);
            //ros.Publish(cameraInfoTopicName, cameraInfoMessage);

            timeElapsed = 0;
        }
    }

    private void OnDestroy()
    {
        if (texture2D != null) {
            Destroy(texture2D);
            texture2D = null;
        }
        NetMQConfig.Cleanup();
        _netMqPublisher.Stop();
    }

    private void Update()
    {
        // print(_response);
        // image_storage = new ImageStorage();
        Connected = _netMqPublisher.Connected;
        // image_storage.rgb_image = _response;
        // print(image_storage.rgb_image);
    }

    private byte[] HandleMessage(byte[] message)
    {
        // Not on main thread
        return _response;
    }
}

// public class NetMqPublisher
// {
//     private readonly Thread _listenerWorker;
//     private bool _listenerCancelled;
//     public delegate byte[] MessageDelegate(byte[] message);


//     private readonly MessageDelegate _messageDelegate;
//     private readonly Stopwatch _contactWatch;
//     private const long ContactThreshold = 1000;
//     public bool Connected;
//     private byte[] pub_rgb_image;

//     private void ListenerWork()
//     {
//         AsyncIO.ForceDotNet.Force();
//         using (var server = new PublisherSocket())
//         {
//             server.Bind("tcp://127.0.0.1:5001");

//             while (true)
//             {
//                 // byte[] rgb_message = new byte[] { 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 };
//                 // byte[] message;
//                 // var response = _messageDelegate(message);
//                 // UnityEngine.Debug.Log(response);
//                 // var msg = _NaoqiSpeechToTextPublisher.pepper_messageFromSubscriber;
//                 // UnityEngine.Debug.Log(pub_rgb_image);
//                 // UnityEngine.Debug.Log(_NaoqiSpeechToTextPublisher.getPepperMessage());
//                 server.SendFrame(pub_rgb_image);
//             }
//         }
//         NetMQConfig.Cleanup();
//     }

//     // public NetMqPublisher(byte[] image){
//     //     pub_rgb_image = image;
//     //     UnityEngine.Debug.Log(image);
//     //     _listenerWorker = new Thread(ListenerWork);
//     // }

//     public NetMqPublisher(MessageDelegate messageDelegate)
//     {
//         _messageDelegate = messageDelegate;
//         UnityEngine.Debug.Log(messageDelegate);
//         _contactWatch = new Stopwatch();
//         _contactWatch.Start();
//         _listenerWorker = new Thread(ListenerWork);
//     }

//     public void Start()
//     {
//         _listenerCancelled = false;
//         _listenerWorker.Start();
//     }

//     public void Stop()
//     {
//         _listenerCancelled = true;
//         _listenerWorker.Join();
//     }
// }


public class NetMqPublisher
{
    private readonly Thread _listenerWorker;
    private bool _listenerCancelled;
    public delegate byte[] MessageDelegate(byte[] message);


    private readonly MessageDelegate _messageDelegate;
    private readonly Stopwatch _contactWatch;
    private const long ContactThreshold = 1000;
    public bool Connected;
    // public ImageStorage image_storage;

    private void ListenerWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var server = new ResponseSocket())
        {
            server.Bind("tcp://*:5001");
            
            while (!_listenerCancelled)
            {
                Connected = _contactWatch.ElapsedMilliseconds < ContactThreshold;
                byte[] message;
                // image_storage = new ImageStorage();
                if (!server.TryReceiveFrameBytes(out message)) continue;
                _contactWatch.Restart();
                var response = _messageDelegate(message);

                // UnityEngine.Debug.Log(image_storage.rgb_image);
                server.SendFrame(response);
                // server.SendMoreFrame("topic1").SendFrame(response);
                // server.SendMoreFrame("topic2").SendFrame("haha");
            }
        }
        NetMQConfig.Cleanup();
    }

    public NetMqPublisher(MessageDelegate messageDelegate)
    {
        _messageDelegate = messageDelegate;
        _contactWatch = new Stopwatch();
        _contactWatch.Start();
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