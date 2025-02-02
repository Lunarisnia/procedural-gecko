using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    public Vector3 StartOffset;

    public Vector3 AllowedRotationalAxis;

    public float MinAngle;
    public float MaxAngle;

    private void Awake()
    {
        StartOffset = transform.localPosition;
    }
}