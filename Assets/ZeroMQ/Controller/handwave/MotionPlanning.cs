using System;
using Unity.Robotics;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.UrdfImporter.Control;
using System.Collections.Concurrent;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

namespace Unity.Robotics.UrdfImporter.Control
{

    public enum ControlMode {Naoqi};
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };

    public class MotionPlanning : MonoBehaviour
    {

        private ArticulationBody[] LShoulderChain;
        private ArticulationBody[] RShoulderChain;

        public ControlMode mode = ControlMode.Naoqi;
        private HandWavingNetMqSubscriber _HandWavingNetMqSubscriber;

        [InspectorReadOnly(hideInEditMode: true)]
        public string selectedJoint;
        [HideInInspector]

        public ControlType control = ControlType.PositionControl;
        public float stiffness;
        public float damping;
        private float target;
        public float forceLimit;
        public float speed; // Units: degree/s
        public float torque; // Units: Nm or N
        public float acceleration;// Units: m/s^2 / degree/s^2
        private int motionPlan;
        private int defDyanmicVal = 10;
        float timer = 0;
        bool timerReached = false;
        private bool NaoqiMessageReceived = false;

        // Arm
        public GameObject Rshoulder; 
        public GameObject RBicep;
        public GameObject RForearm;
        public GameObject Rwrist;

        public GameObject Lshoulder;

        private ArticulationBody rshoulder_ab;
        private ArticulationBody rforearm_ab;
        private ArticulationBody rwrist_ab;
        private ArticulationBody rbicep_ab;

        private ArticulationBody lshoulder_ab;
        
        private int shoulderIndex;
        private int bicepIndex;
        private int forearmIndex;
        private int wristIndex;

        private float shoulder_target;
        private float forearm_target;
        private float wrist_target;
        private float bicep_target;

        //Fingers
        public GameObject RThumb1; 
        public GameObject RFinger11;
        public GameObject RFinger21;
        public GameObject RFinger31;
        public GameObject RFinger41;

        private ArticulationBody rthumb1_ab;
        private ArticulationBody rfinger11_ab;
        private ArticulationBody rfinger21_ab;
        private ArticulationBody rfinger31_ab;
        private ArticulationBody rfinger41_ab;

        private int rthumb1Index;
        private int rfinger11Index;
        private int rfinger21Index;
        private int rfinger31Index;
        private int rfinger41Index;

        private float rthumb1_target;
        private float rfinger11_target;
        private float rfinger21_target;
        private float rfinger31_target;
        private float rfinger41_target; 

        private int jointPart;
        private string jointName;
        private float jointMoveTarget;

        private string motionAction = "";


        private void InitialiseJointPhysics(ArticulationBody[] bodyChain){
            foreach (ArticulationBody joint in bodyChain)
            {
                joint.gameObject.AddComponent<JointControl>();
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = forceLimit;
                joint.xDrive = currentDrive;
            }            
        }

        void Start()
        {
            rshoulder_ab = Rshoulder.GetComponent<ArticulationBody>();
            rforearm_ab = RForearm.GetComponent<ArticulationBody>();
            rwrist_ab = Rwrist.GetComponent<ArticulationBody>();
            rbicep_ab = RBicep.GetComponent<ArticulationBody>();
            rthumb1_ab = RThumb1.GetComponent<ArticulationBody>();
            rfinger11_ab = RFinger11.GetComponent<ArticulationBody>();
            rfinger21_ab = RFinger21.GetComponent<ArticulationBody>();
            rfinger31_ab = RFinger31.GetComponent<ArticulationBody>();
            rfinger41_ab = RFinger41.GetComponent<ArticulationBody>();

            lshoulder_ab = Lshoulder.GetComponent<ArticulationBody>();

            shoulderIndex = 0;
            bicepIndex = 1;
            forearmIndex = 3;
            wristIndex = 4;
            rthumb1Index = 5;
            rfinger11Index = 7;
            rfinger21Index = 10;
            rfinger31Index = 13;
            rfinger41Index = 16;

            jointPart = -1;

            this.gameObject.AddComponent<FKRobot>();
            RShoulderChain = rshoulder_ab.GetComponentsInChildren<ArticulationBody>();
            LShoulderChain = lshoulder_ab.GetComponentsInChildren<ArticulationBody>();

            _HandWavingNetMqSubscriber = new HandWavingNetMqSubscriber(HandleMessage);
            _HandWavingNetMqSubscriber.Start();

            InitialiseJointPhysics(RShoulderChain);
            InitialiseJointPhysics(LShoulderChain);
        }

        private void HandleMessage(string message)
        {
            var splittedStrings = message.Split(' ');
            // print(splittedStrings);
            // if (splittedStrings.Length != 3) return;
            motionAction = splittedStrings[0];
            string NaoqijointName = splittedStrings[1];
            float NaoqijointMoveTarget = float.Parse(splittedStrings[2]);
            NaoqiMessageReceived = true;
            timer = Time.deltaTime;

            if (motionAction.Equals("origin")){
                print("return to origin");
            }
            else{
                // print(jointName);
                // print(jointMoveTarget);
                if (NaoqijointName.Equals("RShoulderPitch")){
                    setJointPart(shoulderIndex);
                    setJointTarget(NaoqijointMoveTarget);
                }

                else if (NaoqijointName.Equals("RShoulderRoll")){
                    setJointPart(bicepIndex);
                    setJointTarget(NaoqijointMoveTarget);
                }

                else if (NaoqijointName.Equals("RElbowRoll")){
                    setJointPart(forearmIndex);
                    setJointTarget(NaoqijointMoveTarget);
                }   

                else if (NaoqijointName.Equals("RWristYaw")){
                    setJointPart(wristIndex);
                    setJointTarget(NaoqijointMoveTarget);
                } 

                else if (NaoqijointName.Equals("RThumb1")){
                    setJointPart(rthumb1Index);
                    setJointTarget(NaoqijointMoveTarget);
                }  


                else if (NaoqijointName.Equals("RFinger11")){
                    setJointPart(rfinger11Index);
                    setJointTarget(NaoqijointMoveTarget);
                }     


                else if (NaoqijointName.Equals("RFinger21")){
                    setJointPart(rfinger21Index);
                    setJointTarget(NaoqijointMoveTarget);
                }     


                else if (NaoqijointName.Equals("RFinger31")){
                    setJointPart(rfinger31Index);
                    setJointTarget(NaoqijointMoveTarget);
                }     


                else if (NaoqijointName.Equals("RFinger41")){
                    setJointPart(rfinger41Index);
                    setJointTarget(NaoqijointMoveTarget);
                }                     
            }
        }

        private void setJointPart(int jointIndex){
            this.jointPart = jointIndex;
        }        

        private void setJointTarget(float jointTarget){
            this.jointMoveTarget = jointTarget;
        }        
        
        void Update()
        {
            _HandWavingNetMqSubscriber.Update();

            // Hand waving 
            timer += Time.deltaTime;

            if (motionAction.Equals("origin")){
                ShoulderReturnToOrigin(shoulderIndex);
                BicepReturnToOrigin(bicepIndex);
                WristReturnToOrigin(wristIndex);
                ForeArmReturnToOrigin(forearmIndex);
                StartCoroutine(FingerRolling(rthumb1Index, 0, rthumb1_ab));
                StartCoroutine(FingerRolling(rfinger11Index, 0, rfinger11_ab));
                StartCoroutine(FingerRolling(rfinger21Index, 0, rfinger21_ab));
                StartCoroutine(FingerRolling(rfinger31Index, 0, rfinger31_ab));
                StartCoroutine(FingerRolling(rfinger41Index, 0, rfinger41_ab));
            }

            else{
                if (jointPart == shoulderIndex){
                    StartCoroutine(ShoulderRaise(jointPart, jointMoveTarget));
                }

                if (jointPart == bicepIndex){
                    StartCoroutine(BicepExtending(jointPart, jointMoveTarget));
                }          

                if (jointPart == forearmIndex){
                    StartCoroutine(ForearmExtending(jointPart, jointMoveTarget));
                }

                if(jointPart == wristIndex){
                    StartCoroutine(WristRolling(jointPart, jointMoveTarget));
                }     

                if(jointPart == rthumb1Index){
                    StartCoroutine(FingerRolling(rthumb1Index, jointMoveTarget, rthumb1_ab));
                    StartCoroutine(FingerRolling(rfinger11Index, jointMoveTarget, rfinger11_ab));
                    StartCoroutine(FingerRolling(rfinger21Index, jointMoveTarget, rfinger21_ab));
                    StartCoroutine(FingerRolling(rfinger31Index, jointMoveTarget, rfinger31_ab));
                    StartCoroutine(FingerRolling(rfinger41Index, jointMoveTarget, rfinger41_ab));
                }
            }
        }

        private void OnDestroy()
        {
            _HandWavingNetMqSubscriber.Stop();
        }

        /// <summary>
        /// Sets the direction of movement of the joint on every update
        /// </summary>
        /// <param name="jointIndex">Index of the link selected in the Articulation Chain</param>
        IEnumerator ShoulderRaise(int jointIndex, float shoulder_target)
        {
            
            JointControl current = RShoulderChain[jointIndex].GetComponent<JointControl>();

            // print(current);
            if (current.controltype != control) 
            {
                UpdateControlType(current);
            }

            ArticulationDrive shoulderDrive = rshoulder_ab.xDrive;
            var current_target  = shoulderDrive.target;
            
            if (shoulder_target!=0){
                if (current_target > shoulder_target){
                    current.direction = RotationDirection.Negative;
                }

                if (current_target < shoulder_target || current_target == shoulder_target){
                    current.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }
            }

            if (shoulder_target == 0){
                if (current_target < 0){
                    current.direction = RotationDirection.Positive;
                }

                if (current_target > 0 || current_target == 0){
                    current.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }                
            }
        }
        

        IEnumerator FingerRolling(int jointIndex, float finger_target, ArticulationBody finger_ab){
            JointControl current_right = RShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive fingerDrive_right = finger_ab.xDrive;
            // ArticulationDrive wristDrive_left = lwrist_ab.xDrive;

            var current_finger_target  = fingerDrive_right.target;
            // var current_Lwrist_target  = wristDrive_left.target;
            current_right.speed = 10;
            current_right.acceleration = 10;

            if (finger_target < 0){

                if(current_finger_target > finger_target){
                    current_right.direction = RotationDirection.Negative;
                }



                if (current_finger_target < finger_target || current_finger_target == finger_target){
                    current_right.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);

                }
            }

            if (finger_target > 0){

                if(current_finger_target < finger_target){
                    current_right.direction = RotationDirection.Positive;
                }

                if (current_finger_target > finger_target || current_finger_target == finger_target){
                    current_right.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }
            }

            if (finger_target == 0){
                if(current_finger_target < 0){
                    current_right.direction = RotationDirection.Positive;
                }

                if (current_finger_target > 0 || current_finger_target == 0){
                    current_right.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }                
            }
        }


        IEnumerator WristRolling(int jointIndex, float wrist_target){
            JointControl current_right = RShoulderChain[jointIndex].GetComponent<JointControl>();
            // JointControl current_left = LShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive wristDrive_right = rwrist_ab.xDrive;
            // ArticulationDrive wristDrive_left = lwrist_ab.xDrive;

            var current_Rwrist_target  = wristDrive_right.target;
            // var current_Lwrist_target  = wristDrive_left.target;


            if (wrist_target != 0){
                if(current_Rwrist_target<wrist_target){
                    current_right.direction = RotationDirection.Positive;
                }


                // if(current_Lwrist_target>(predefined_target_hold_positive*-1)){
                //     current_left.direction = RotationDirection.Negative;
                // }           

                if (current_Rwrist_target > wrist_target || current_Rwrist_target == wrist_target){
                    current_right.direction = RotationDirection.None;
                    // current_left.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }
            }

            if (wrist_target == 0){

                if(current_Rwrist_target<wrist_target){
                    current_right.direction = RotationDirection.Positive;
                }

                if (current_Rwrist_target > wrist_target || current_Rwrist_target == wrist_target){
                    current_right.direction = RotationDirection.None;
                    // current_left.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }                
            }
        }

        IEnumerator BicepExtending(int jointIndex, float bicep_target){
            JointControl current_right = RShoulderChain[jointIndex].GetComponent<JointControl>();
            // JointControl current_left = LShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive bicepDrive_right = rbicep_ab.xDrive;
            // ArticulationDrive bicepDrive_left = lbicep_ab.xDrive;

            var current_Rbicep_target  = bicepDrive_right.target;
            // var current_Lbicep_target  = bicepDrive_left.target;

            if (bicep_target != 0){
                if(current_Rbicep_target > bicep_target){
                    current_right.direction = RotationDirection.Negative;
                }

                // if(current_Lbicep_target< predefined_target_drop_positive){
                //     current_left.direction = RotationDirection.Positive;
                // }           

                if (current_Rbicep_target < bicep_target || current_Rbicep_target == bicep_target){
                    current_right.direction = RotationDirection.None;
                    // current_left.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }
            }

            if (bicep_target == 0){
                if(current_Rbicep_target < 0){
                    current_right.direction = RotationDirection.Positive;
                }

                // if(current_Lbicep_target< predefined_target_drop_positive){
                //     current_left.direction = RotationDirection.Positive;
                // }           

                if (current_Rbicep_target > 0 || current_Rbicep_target == 0){
                    current_right.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }                
            }
        }  

        IEnumerator ForearmExtending(int jointIndex, float forearm_target){
            JointControl current_right = RShoulderChain[jointIndex].GetComponent<JointControl>();
            // JointControl current_left = LShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive forearmDrive_right = rforearm_ab.xDrive;
            // ArticulationDrive bicepDrive_left = lbicep_ab.xDrive;

            var current_Rforearm_target  = forearmDrive_right.target;
            // var current_Lbicep_target  = bicepDrive_left.target;

            if (forearm_target!=0){
                if(current_Rforearm_target < forearm_target){
                    current_right.direction = RotationDirection.Positive;
                }      

                if (current_Rforearm_target > forearm_target || current_Rforearm_target == forearm_target){
                    current_right.direction = RotationDirection.None;
                    // current_left.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }
            }
            
            if (forearm_target == 0){
                if(current_Rforearm_target > 0){
                    current_right.direction = RotationDirection.Negative;
                }     

                if (current_Rforearm_target < 0 || current_Rforearm_target == 0){
                    current_right.direction = RotationDirection.None;
                    yield return new WaitForSeconds(0.3f);
                }
            }
        }    

        private void WristReturnToOrigin(int jointIndex){
            
            JointControl current_wrist_right = RShoulderChain[jointIndex].GetComponent<JointControl>();
            // JointControl current_wrist_left = LShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive wristDrive_right = rwrist_ab.xDrive;
            var current_target_right  = wristDrive_right.target;

            // ArticulationDrive wristDrive_left = lwrist_ab.xDrive;
            // var current_target_left  = wristDrive_left.target;

            if (current_target_right >0 ){
                current_wrist_right.direction = RotationDirection.Negative;
                // current_wrist_left.direction = RotationDirection.Positive;
            }

            if (current_target_right < 0 ){
                current_wrist_right.direction = RotationDirection.Positive;
                // current_wrist_left.direction = RotationDirection.Negative;
            }

            if (current_target_right< 0 || current_target_right == 0){
                // current_wrist_left.direction = RotationDirection.None;
                current_wrist_right.direction = RotationDirection.None;
            }      
        }

        private void BicepReturnToOrigin(int jointIndex){

            JointControl current_bicep_right = RShoulderChain[jointIndex].GetComponent<JointControl>();
            // JointControl current_bicep_left = LShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive bicepDrive_right = rbicep_ab.xDrive;
            // ArticulationDrive bicepDrive_left = lbicep_ab.xDrive;

            var current_target_right  = bicepDrive_right.target;
            // var current_target_left  = bicepDrive_left.target;

            if (current_target_right < 0 ){
                // current_bicep_left.direction = RotationDirection.Negative;
                current_bicep_right.direction = RotationDirection.Positive;
            }

            else if (current_target_right > 0 ){
                // current_bicep_left.direction = RotationDirection.Positive;
                current_bicep_right.direction = RotationDirection.Negative;
            }

            else if (current_target_right == 0){
                // current_bicep_left.direction = RotationDirection.None;
                current_bicep_right.direction = RotationDirection.None;
            }       
        }

        private void ForeArmReturnToOrigin(int jointIndex){

            JointControl current_hw = RShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive forearmDrive = rforearm_ab.xDrive;
            var current_target  = forearmDrive.target;

            if (current_target > 0 ){
                current_hw.direction = RotationDirection.Negative;
            }

            if (current_target < 0 ){
                current_hw.direction = RotationDirection.Positive;
            }

            if (current_target == 0 ){
                current_hw.direction = RotationDirection.None;
                // ArmMotionDone = true;
            }
        }

        private void ShoulderReturnToOrigin(int jointIndex){

            JointControl current_hw = RShoulderChain[jointIndex].GetComponent<JointControl>();

            ArticulationDrive shoulderDrive = rshoulder_ab.xDrive;
            var current_target  = shoulderDrive.target;

            if (current_target < 0){
                current_hw.direction = RotationDirection.Positive;
            }

            else if (current_target > 0){
                current_hw.direction = RotationDirection.Negative;
            }

            else if (current_target == 0){
                current_hw.direction = RotationDirection.None;
            }
        }

        public void UpdateControlType(JointControl joint)
        {
            joint.controltype = control;
            if (control == ControlType.PositionControl)
            {
                ArticulationDrive drive = joint.joint.xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                joint.joint.xDrive = drive;
            }
        }

        void DestroyScriptInstance()
        {
            // Removes this script instance from the game object
            Destroy(this);
            print("MotionPlan Done");
        }
    }
}


public class HandWavingNetMqSubscriber
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
            subSocket.Connect("tcp://192.168.50.42:5009");
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

    public HandWavingNetMqSubscriber(MessageDelegate messageDelegate)
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