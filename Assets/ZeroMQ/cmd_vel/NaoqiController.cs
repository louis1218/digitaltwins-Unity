using UnityEngine;
using Unity.Robotics.UrdfImporter.Control;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public enum ControlMode {Naoqi};

public class NaoqiController : MonoBehaviour
{
    public GameObject wheel1;
    public GameObject wheel2;
    public ControlMode mode = ControlMode.Naoqi;

    private ArticulationBody wA1;
    private ArticulationBody wA2;

    public float maxLinearSpeed = 2; //  m/s
    public float maxRotationalSpeed = 10;//
    public float minRotationalSpeed = 6;//
    public float wheelRadius = 0.1f; //meters
    public float trackWidth = 0.1f; // meters Distance between tyres
    public float forceLimit = 100;
    public float damping = 10;

    public float Timeout = 0.5f;
    private float lastCmdReceived = 0f;

    private RotationDirection direction;

    private float linear_x ;
    private float angular_z;

    private NetMqSubscriber _netMqSubscriber;

    private void HandleMessage(string message)
    {
        // print(message);
        var splittedStrings = message.Split(' ');
        // print(splittedStrings);
        // if (splittedStrings.Length != 3) return;
        linear_x = float.Parse(splittedStrings[0]);
        angular_z = float.Parse(splittedStrings[2]);
        lastCmdReceived = Time.time;
    }

    private void OnDestroy()
    {
        _netMqSubscriber.Stop();
    }

    void Start()
    {
        wA1 = wheel1.GetComponent<ArticulationBody>();
        wA2 = wheel2.GetComponent<ArticulationBody>();
        SetParameters(wA1);
        SetParameters(wA2);
        _netMqSubscriber = new NetMqSubscriber(HandleMessage);
        _netMqSubscriber.Start();
        print("cmd_vel initialised");
    }

    void FixedUpdate()
    {
        _netMqSubscriber.Update();
        if (mode == ControlMode.Naoqi)
        {
            NaoqiUpdate();
        }
    }

    private void SetParameters(ArticulationBody joint)
    {
        ArticulationDrive drive = joint.xDrive;
        drive.forceLimit = forceLimit;
        drive.damping = damping;
        joint.xDrive = drive;
    }

    private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
    {
        ArticulationDrive drive = joint.xDrive;
        if (float.IsNaN(wheelSpeed))
        {
            drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
        }
        else
        {
            drive.targetVelocity = wheelSpeed;
        }
        joint.xDrive = drive;
    }

    private void NaoqiUpdate()
    {
        if (Time.time - lastCmdReceived > Timeout)
        {
            linear_x = 0f;
            angular_z = 0f;
        }
        RobotInput(linear_x, angular_z);
    }


    private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
    {
        if (speed > maxLinearSpeed)
        {
            speed = maxLinearSpeed;
        }

        if (rotSpeed >0){
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }

            if (rotSpeed < minRotationalSpeed)
            {
                rotSpeed = minRotationalSpeed;
            }
        }

        if (rotSpeed <0){
            if ((rotSpeed*-1) > maxRotationalSpeed)
            {
                rotSpeed = (maxRotationalSpeed*-1);
            }

            if ((rotSpeed*-1) < minRotationalSpeed)
            {
                rotSpeed = (minRotationalSpeed*-1);
            }
        }

        float wheel1Rotation = (speed / wheelRadius);
        float wheel2Rotation = wheel1Rotation;
        float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
        if (rotSpeed != 0)
        {
            wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
        }
        else
        {
            wheel1Rotation *= Mathf.Rad2Deg;
            wheel2Rotation *= Mathf.Rad2Deg;
        }
        SetSpeed(wA1, wheel1Rotation);
        SetSpeed(wA2, wheel2Rotation);
    }
}


public class NetMqSubscriber
{
    private readonly Thread _listenerWorker;

    private bool _listenerCancelled;

    public delegate void MessageDelegate(string message);

    private readonly MessageDelegate _messageDelegate;

    private readonly ConcurrentQueue<string> _messageQueue = new ConcurrentQueue<string>();

    private void ListenerWork()
    {
        AsyncIO.ForceDotNet.Force();
        using (var subSocket = new SubscriberSocket())
        {
            subSocket.Options.ReceiveHighWatermark = 1000;
            subSocket.Connect("tcp://192.168.50.42:5003");
            subSocket.Subscribe("");
            while (!_listenerCancelled)
            {
                string frameString;
                if (!subSocket.TryReceiveFrameString(out frameString)) continue;
                // Debug.Log(frameString);
                _messageQueue.Enqueue(frameString);
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

    public NetMqSubscriber(MessageDelegate messageDelegate)
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