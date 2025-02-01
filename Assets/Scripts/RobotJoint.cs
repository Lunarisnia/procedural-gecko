using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    public Vector3 StartOffset;

    public Vector3 AllowedRotationalAxis;

    private void Awake()
    {
        StartOffset = transform.localPosition;
    }
}