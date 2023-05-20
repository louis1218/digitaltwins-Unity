using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using StringMsg = RosMessageTypes.Std.StringMsg;
using System;

public class VoskResultText : MonoBehaviour 
{
    public VoskSpeechToText VoskSpeechToText;
    public Text ResultText;
    public int count = 0;

    ROSConnection ros;
    // Start is called before the first frame update
    void Start(){
        ros = ROSConnection.instance;
        ros.RegisterPublisher<StringMsg>("unity_chatter");
    }

    void Awake()
    {
        VoskSpeechToText.OnTranscriptionResult += OnTranscriptionResult;
    }

    private void OnTranscriptionResult(string obj)
    {

        Debug.Log(obj);
        var result = new RecognitionResult(obj);

        Debug.Log(ResultText.text.Length); 

        if ((ResultText.text.Length)>100)
        {
            ResultText.text = "";
            count = 1;
        }

        else
        {
            ResultText.text += count;
            
            for (int i = 0; i < result.Phrases.Length; i++)
            {
                if (i > 0)
                {
                    ResultText.text += ", ";
                }
            ResultText.text += result.Phrases[i].Text;
            }
        }
    	ResultText.text += "\n";
        count = count+1;
    }

    // Update is called once per frame
    void Update(){
        StringMsg msg_data = new StringMsg(ResultText.text);
        ros.Send("unity_chatter", msg_data);
    }
}
