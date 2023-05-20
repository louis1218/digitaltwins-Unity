// using System.Diagnostics;
// using System.Threading;
// using NetMQ;
// using NetMQ.Sockets;
// using UnityEngine;
// using System;
// using System.Collections.Generic;
// using UnityEngine.Serialization;
// using System.Collections;
// using System.Text;
// using System.Net;
// using System.Net.Sockets;
// using System.Threading.Tasks;
// using Unity.Robotics.Core;
// using UnityEngine.Rendering;
// using System.IO;

// public class DepthCamera : MonoBehaviour
// {

//     public Camera ImageCamera;
//     public string FrameId = "CameraDepth_frame";
//     public int resolutionWidth = 640;
//     public int resolutionHeight = 480;
//     // [Range(0, 100)]
//     // public int qualityLevel = 50;
//     // private Texture2D texture2D;
//     // private Rect rect;
//     // Publish the cube's position and rotation every N seconds
//     public float publishMessageFrequency = 0.005f;

//     // Used to determine how much time has elapsed since the last message was published
//     private float timeElapsed;

//     [Header("Shader Setup")]
//     public Shader uberReplacementShader;
//     // public Shader opticalFlowShader;
//     public float opticalFlowSensitivity = 1.0f;

//     // pass configuration
//     private CapturePass capturePass = new CapturePass() { name = "_depth" };
//     private Texture2D texture2D;
//     private Rect rect;

//     public bool Connected;
//     private DepthCamera_NetMqPublisher depth_netMqPublisher;
//     // private byte[] _response;
//     private byte[] jpg_depth_img;

//     private byte[] HandleMessage(byte[] message)
//     {
//         // Not on main thread
//         return jpg_depth_img;
//     }

//     public DepthCamera(){}

//     struct CapturePass
//     {
//         // configuration
//         public string name;
//         public bool supportsAntialiasing;
//         public bool needsRescale;
//         public CapturePass(string name_) { name = name_; supportsAntialiasing = true; needsRescale = false; camera = null; }
//         public Camera camera;
//     };

//     static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode, Color clearColor)
//     {
//         var cb = new CommandBuffer();
//         cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
//         cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
//         cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
//         cam.SetReplacementShader(shader, "");
//         cam.backgroundColor = clearColor;
//         cam.clearFlags = CameraClearFlags.SolidColor;
//         cam.allowHDR = false;
//         cam.allowMSAA = false;
//     }

//     public enum ReplacementMode
//     {
//         ObjectId = 0,
//         CatergoryId = 1,
//         DepthCompressed = 2,
//         DepthMultichannel = 3,
//         Normals = 4
//     };
    
//     public void Start()
//     {
//         // Initialize game Object
//         // texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
//         // rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
//         // ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
//         texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RFloat, false);
//         rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
//         // default fallbacks, if shaders are unspecified
//         if (!uberReplacementShader)
//             uberReplacementShader = Shader.Find("Hidden/UberReplacement");

//         // if (!opticalFlowShader)
//         //     opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

//         //set up camera shader
//         SetupCameraWithReplacementShader(ImageCamera, uberReplacementShader, ReplacementMode.DepthCompressed, Color.white);


//         capturePass.camera = ImageCamera;

//         // on scene change
//         var renderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
//         var mpb = new MaterialPropertyBlock();
//         foreach (var r in renderers)
//         {
//             var id = r.gameObject.GetInstanceID();
//             var layer = r.gameObject.layer;
//             var tag = r.gameObject.tag;

//             mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
//             mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
//             r.SetPropertyBlock(mpb);
//         }

//         depth_netMqPublisher = new DepthCamera_NetMqPublisher(HandleMessage);
//         depth_netMqPublisher.Start();
//         //SAVE
//         // Camera.onPostRender += UpdateImage;
//     }
//     // private void UpdateImage(Camera _camera)
//     // {
//     //     if (texture2D != null && _camera == this.ImageCamera)
//     //         UpdateMessage();
//     // }

//     public void Update()
//     {
//         if (texture2D != null)
//             // execute as coroutine to wait for the EndOfFrame before starting capture
//             StartCoroutine(
//                 WaitForEndOfFrameAndSave());

//         Connected = depth_netMqPublisher.Connected;

//     }

//     public IEnumerator WaitForEndOfFrameAndSave()
//     {
//         yield return new WaitForEndOfFrame();
//         UpdateMessage();
//     }

//     public void UpdateMessage()
//     {
//         timeElapsed += Time.deltaTime;

//         if (timeElapsed > publishMessageFrequency)
//         {
//             //save
//             bool supportsAntialiasing = true;
//             bool needsRescale = false;
//             var depth = 32;
//             var format = RenderTextureFormat.Default;
//             var readWrite = RenderTextureReadWrite.Default;
//             var antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

//             var finalRT =
//                 RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, depth, format, readWrite, antiAliasing);
//             var renderRT = (!needsRescale) ? finalRT :
//                 RenderTexture.GetTemporary(ImageCamera.pixelWidth, ImageCamera.pixelHeight, depth, format, readWrite, antiAliasing);
//             // var tex = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);

//             var prevActiveRT = RenderTexture.active;
//             var prevCameraRT = ImageCamera.targetTexture;

//             // render to offscreen texture (readonly from CPU side)
//             RenderTexture.active = renderRT;
//             ImageCamera.targetTexture = renderRT;
//             //
//             // ImageCamera.Render();
//             texture2D.ReadPixels(rect, 0, 0);
//             texture2D.Apply();
//             print("Depth Camera Connected");
//             jpg_depth_img = texture2D.EncodeToJPG();

//             // Camera Info message
//             // CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, message.header, 0.0f, 0.01f);
//             // ros.Publish(cameraInfoTopicName, cameraInfoMessage);
//             RenderTexture.active = null;
//             timeElapsed = 0;            
//         }
//     }
    
//     public void DestroyImage(){
//         Destroy(texture2D);
//         print("Destroying Image");
//     }

//     private void OnDestroy()
//     {
//         depth_netMqPublisher.Stop();
//     }
// }

// public class DepthCamera_NetMqPublisher
// {
//     private readonly Thread _listenerWorker;
//     private bool _listenerCancelled;
//     public delegate byte[] MessageDelegate(byte[] message);
//     public DepthCamera depthCamera;

//     private readonly MessageDelegate _messageDelegate;
//     private readonly Stopwatch _contactWatch;
//     private const long ContactThreshold = 1000;
//     public bool Connected;
//     public bool msg_sent = false;

//     public DepthCamera_NetMqPublisher(){}

//     public bool checkMsgStatus(){
//         return msg_sent;
//     }

//     private void ListenerWork()
//     {
        
//         AsyncIO.ForceDotNet.Force();
//         using (var server = new ResponseSocket())
//         {
//             server.Bind("tcp://127.0.0.1:5002");

//             while (!_listenerCancelled)
//             {
//                 depthCamera = new DepthCamera();
//                 Connected = _contactWatch.ElapsedMilliseconds < ContactThreshold;
//                 byte[] message;
//                 if (!server.TryReceiveFrameBytes(out message)) continue;
//                 _contactWatch.Restart();
//                 var response = _messageDelegate(message);
//                 server.SendFrame(response);
//             }
//         }
//         NetMQConfig.Cleanup();
//     }

//     public DepthCamera_NetMqPublisher(MessageDelegate messageDelegate)
//     {
//         _messageDelegate = messageDelegate;
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


