using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;
using UnityEngine;
using System;
using UnityEngine.Serialization;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using Unity.Robotics.Core;
using UnityEngine.Rendering;
using System.IO;

public class depth_script : MonoBehaviour
{
    public Material material;
    public int height= 480; // Change to desired height
	public int width = 640; // Change to desired width
	public Camera cam;
	public RenderTexture renderTexture;
    public Texture2D screenShot;
    public Rect rect;
    private byte[] jpg_depth_img;
    // Start is called before the first frame update
    public DepthCamera_NetMqPublisher depth_netMqPublisher;

    private byte[] HandleMessage(byte[] message)
    {
        return jpg_depth_img;
    }

    public void Start()
    {
        rect = new Rect(0, 0, width, height);
        renderTexture = new RenderTexture(width, height, 24);
        screenShot = new Texture2D(width, height, TextureFormat.RGBA32, false);
        screenShot.hideFlags = HideFlags.HideAndDontSave;
        cam = GetComponent<Camera>(); 
        cam.depthTextureMode = DepthTextureMode.Depth;
        depth_netMqPublisher = new DepthCamera_NetMqPublisher(HandleMessage);
        depth_netMqPublisher.Start();
        print("DepthCamera initialised");
    }

    public void OnRenderImage (RenderTexture source, RenderTexture dest){
		Graphics.Blit(source, dest, material);
	}

    // Update is called once per frame
    public void Update()
    {
        cam.targetTexture = renderTexture;
        cam.Render();

        RenderTexture.active = renderTexture;
        screenShot.ReadPixels(rect, 0, 0);
        screenShot.Apply();
        // print(screenShot.GetRawTextureData().Length);
        cam.targetTexture = null;
        RenderTexture.active = null;
        // print("Depth Camera Connected");
        jpg_depth_img = screenShot.EncodeToJPG();

    }
}


public class DepthCamera_NetMqPublisher
{
    private readonly Thread _listenerWorker;
    private bool _listenerCancelled;
    public delegate byte[] MessageDelegate(byte[] message);

    private readonly MessageDelegate _messageDelegate;
    private readonly Stopwatch _contactWatch;
    private const long ContactThreshold = 1000;
    public bool Connected;

    public DepthCamera_NetMqPublisher(){}

    private void ListenerWork()
    {
        
        AsyncIO.ForceDotNet.Force();
        using (var server = new ResponseSocket())
        {
            server.Bind("tcp://*:5002");

            while (!_listenerCancelled)
            {
                Connected = _contactWatch.ElapsedMilliseconds < ContactThreshold;
                byte[] message;
                if (!server.TryReceiveFrameBytes(out message)) continue;
                _contactWatch.Restart();
                var response = _messageDelegate(message);
                server.SendFrame(response);
            }
        }
        NetMQConfig.Cleanup();
    }

    public DepthCamera_NetMqPublisher(MessageDelegate messageDelegate)
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




