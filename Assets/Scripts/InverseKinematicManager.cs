using UnityEngine;

public class InverseKinematicManager : MonoBehaviour
{
    public RobotJoint[] Joints;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private void Start()
    {
        RobotJoint[] joints = GetComponentsInChildren<RobotJoint>();
        Joints = joints;
    }

    // Update is called once per frame
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.F))
        {
            float[] angles = new float[Joints.Length];
            for (int i = 0; i < Joints.Length; i++)
                angles[i] = Quaternion.Angle(Quaternion.identity, Joints[i].transform.localRotation);
            Debug.Log("FK: " + ForwardKinematics(angles));
        }

        if (Input.GetKeyDown(KeyCode.G))
            foreach (RobotJoint j in Joints)
                Debug.Log("(" + j.name + ")" + " Global Position: " + j.transform.position);
    }

    // This function essentially calculate the global position of the last joint after accounting all the rotation
    private Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].AllowedRotationalAxis);

            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;

            prevPoint = nextPoint;
        }

        return prevPoint;
    }

    private float DistanceFromTarget(Vector3 target, float[] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        return (point - target).magnitude;
    }
}