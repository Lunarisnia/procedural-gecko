using UnityEngine;

public class InverseKinematicManager : MonoBehaviour
{
    public RobotJoint[] Joints;
    public float SamplingDistance = 1.0f;
    public float LearningRate = 0.01f;

    public Transform Target;

    public float DistanceThreshold;

    private float[] angles;

    private void Awake()
    {
        angles = new float[Joints.Length];
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

        InverseKinematics(Target.position, angles);
        for (int i = 0; i < Joints.Length; i++)
            Joints[i].transform.localRotation = Quaternion.AngleAxis(angles[i], Joints[i].AllowedRotationalAxis);
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

    private float PartialGradient(Vector3 target, float[] angles, int i)
    {
        float angle = angles[i];

        float fX = DistanceFromTarget(target, angles);

        angles[i] += SamplingDistance;
        float fXPlusD = DistanceFromTarget(target, angles);

        float gradient = (fXPlusD - fX) / SamplingDistance;

        angles[i] = angle;

        return gradient;
    }

    public void InverseKinematics(Vector3 target, float[] angles)
    {
        if (DistanceFromTarget(target, angles) < DistanceThreshold) return;
        for (int i = 0; i < Joints.Length; i++)
        {
            float gradient = PartialGradient(target, angles, i);
            angles[i] -= LearningRate * gradient;

            angles[i] = Mathf.Clamp(angles[i], Joints[i].MinAngle, Joints[i].MaxAngle);

            if (DistanceFromTarget(target, angles) < DistanceThreshold) return;
        }
    }
}