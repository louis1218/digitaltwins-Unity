using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Int32Msg = RosMessageTypes.Std.Int32Msg;

public class ROS_Chatter : MonoBehaviour{

    ROSConnection ros;
    // Start is called before the first frame update
    void Start(){
        ros = ROSConnection.instance;
        ros.RegisterPublisher<Int32Msg>("test2");
    }

    // Update is called once per frame
    void Update(){
        Int32Msg msg_data = new Int32Msg(12345);
        ros.Send("test2", msg_data);
    }

}