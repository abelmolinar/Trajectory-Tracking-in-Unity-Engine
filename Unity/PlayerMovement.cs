using System;
using System.Net.Sockets;
using System.Text;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public class PlayerMovements : MonoBehaviour
{
    private TcpClient client;
    private NetworkStream stream;
    private string receivedData;
    public Rigidbody rb;
    public Transform player;

    void Start()
    {
        // Ensure Unity runs at full speed even when not in focus
        Application.runInBackground = true;

        // Set a target frame rate
        Application.targetFrameRate = 60;

        // Connect to the Python server
        client = new TcpClient("127.0.0.1", 5000);
        stream = client.GetStream();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
            if (true)
        {
        // Block that sends player coords from Unity to Python
        // Send the sphere's current position (x, y, z) to Python
        Vector3 currentPosition = transform.position;
        Vector3 currentVelocity = rb.linearVelocity;
        string Data = $"{currentPosition.x},{currentPosition.y},{currentPosition.z},{currentVelocity.x},{currentVelocity.y},{currentVelocity.z}";
        byte[] positionBytes = Encoding.UTF8.GetBytes(Data);
        stream.Write(positionBytes, 0, positionBytes.Length);
        Debug.Log("Data Sent: " + Data);

        // Block that receives player velocities to be applied
        byte[] buffer = new byte[1024];
        int bytesRead = stream.Read(buffer, 0, buffer.Length);
        string controlData = Encoding.UTF8.GetString(buffer, 0, bytesRead).Trim();
        Debug.Log("Received control inputs: " + controlData);

        // Parse the control inputs
        string[] controlInputs = controlData.Split(',');
        float velocityX = float.Parse(controlInputs[0]);
        float velocityY = float.Parse(controlInputs[1]);
        float velocityZ = float.Parse(controlInputs[2]);

        // Apply the control inputs to the sphere
        rb.AddForce(velocityX, velocityY, velocityZ, ForceMode.VelocityChange);
        }
        
    }

    void OnApplicationQuit()
    {
        // Close the connection when the application quits
        stream.Close();
        client.Close();
    }
}
