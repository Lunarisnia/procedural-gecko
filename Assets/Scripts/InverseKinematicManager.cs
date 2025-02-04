using UnityEngine;

public class InverseKinematicManager : MonoBehaviour
{
    public RobotJoint[] Joints;
    public float SamplingDistance = 1.0f;
    public float LearningRate = 0.01f;

    public Transform Target;

    public float DistanceThreshold;

    public int IKMode;

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

        if (IKMode == 0)
        {
            InverseKinematics(Target.position, angles);
            for (int i = 0; i < Joints.Length; i++)
                Joints[i].transform.localRotation = Quaternion.AngleAxis(angles[i], Joints[i].AllowedRotationalAxis);
        }
        else if (IKMode == 1)
        {
            FABRIK(Target.position);
        }
        else if (IKMode == 2)
        {
            RobloxFABRIK(Target.position);
        }
        else
        {
            LocalFABRIK(Target.position);
        }
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

    public void FABRIK(Vector3 target)
    {
        float jointSum = 0.0f;
        float[] jointDistances = new float[Joints.Length];
        for (int i = 0; i < jointDistances.Length - 1; i++)
        {
            jointDistances[i] = Mathf.Abs((Joints[i + 1].transform.position - Joints[i].transform.position).magnitude);
            jointSum += jointDistances[i];
        }

        float targetDistance = Mathf.Abs((Joints[0].transform.position - target).magnitude);

        if (targetDistance > jointSum)
            // Debug.Log("Unreachable");
            // Target is unreachable
        {
            for (int i = 0; i < jointDistances.Length - 1; i++)
            {
                float r = Mathf.Abs((target - Joints[i].transform.position).magnitude);
                float lambda = jointDistances[i] / r;

                // Find new joint positions
                Joints[i + 1].transform.position = (1.0f - lambda) * Joints[i].transform.position + lambda * target;
            }
        }
        else
        {
            Vector3 b = Joints[0].transform.position;
            float tipDistance = Mathf.Abs((Joints[Joints.Length - 1].transform.position - target).magnitude);
            if (tipDistance < DistanceThreshold) return;

            Joints[Joints.Length - 1].transform.position = target;
            for (int i = Joints.Length - 2; i >= 0; i--)
            {
                float r = Mathf.Abs((Joints[i + 1].transform.position - Joints[i].transform.position).magnitude);
                float lambda = jointDistances[i] / r;

                Joints[i].transform.position = (1f - lambda) * Joints[i + 1].transform.position +
                                               lambda * Joints[i].transform.position;
            }

            // Stage 2: Backward Reaching
            Joints[0].transform.position = b;
            for (int i = 0; i < Joints.Length - 1; i++)
            {
                float r = Mathf.Abs((Joints[i + 1].transform.position - Joints[i].transform.position).magnitude);
                float lambda = jointDistances[i] / r;

                Joints[i + 1].transform.position = (1.0f - lambda) * Joints[i].transform.position +
                                                   lambda * Joints[i + 1].transform.position;
            }
        }
    }

    public void RobloxFABRIK(Vector3 target)
    {
        float jointSum = 0.0f;
        float[] jointDistances = new float[Joints.Length];
        for (int i = 0; i < jointDistances.Length - 1; i++)
        {
            jointDistances[i] = Mathf.Abs((Joints[i + 1].transform.position - Joints[i].transform.position).magnitude);
            jointSum += jointDistances[i];
        }

        float targetDistance = Mathf.Abs((Joints[0].transform.position - target).magnitude);

        if (targetDistance > jointSum)
        {
            // Target is unreachable
            for (int i = 0; i < jointDistances.Length - 1; i++)
            {
                Transform j = Joints[i].transform;
                Vector3 targetDir = (target - j.position).normalized;
                Vector3 nextPosition = targetDir * jointDistances[i] + Joints[i].transform.position;

                Joints[i + 1].transform.position = nextPosition;
                Joints[i].transform.localRotation = Quaternion.LookRotation(targetDir, transform.up);
            }

            Vector3 t = (target - Joints[Joints.Length - 1].transform.position).normalized;
            Joints[Joints.Length - 1].transform.localRotation = Quaternion.LookRotation(t, transform.up);
        }
        else
        {
            Vector3 b = Joints[0].transform.position;
            Joints[Joints.Length - 1].transform.position = target;
            // Forward Reaching
            for (int i = Joints.Length - 2; i >= 0; i--)
            {
                Transform prevJoint = Joints[i + 1].transform;
                Transform nextJoint = Joints[i].transform;

                Vector3 newDir = (nextJoint.position - prevJoint.position).normalized;
                Vector3 nextPosition = newDir * jointDistances[i] + prevJoint.transform.position;

                Joints[i].transform.position = nextPosition;
            }

            Joints[0].transform.position = b;
            // Backward Reaching
            for (int i = 0; i < Joints.Length - 1; i++)
            {
                Transform nextJoint = Joints[i + 1].transform;
                Transform prevJoint = Joints[i].transform;

                Vector3 newDir = (nextJoint.position - prevJoint.position).normalized;
                Vector3 nextPosition = newDir * jointDistances[i] + prevJoint.transform.position;

                Joints[i + 1].transform.position = nextPosition;

                // Look at the next joint
                Joints[i].transform.localRotation = Quaternion.LookRotation(newDir, transform.up);
            }

            Joints[Joints.Length - 1].transform.localRotation = Quaternion.LookRotation(target, transform.up);
        }
    }

    // NOTE: I am stuck with how to account for parenting
    public void LocalFABRIK(Vector3 target)
    {
        // Debug.Log("GlobalPosition: " + Joints[Joints.Length - 1].transform.position + "-- Local Position: " +
        //           Joints[Joints.Length - 1].transform.worldToLocalMatrix *
        //           Joints[Joints.Length - 1].transform.position);
        // Debug.Log("LocalPosition: " + Joints[Joints.Length - 1].transform.localPosition +
        //           "Global Position (CONVERT): " + Joints[Joints.Length - 2].transform
        //               .InverseTransformPoint(Joints[Joints.Length - 1].transform.position) +
        //           "Global Position (REAL): " + Joints[Joints.Length - 1].transform.position);
        float jointSum = 0.0f;
        float[] jointDistances = new float[Joints.Length];
        for (int i = 0; i < jointDistances.Length - 1; i++)
        {
            jointDistances[i] = Mathf.Abs((Joints[i + 1].transform.position - Joints[i].transform.position).magnitude);
            jointSum += jointDistances[i];
        }

        float targetDistance = Mathf.Abs((Joints[0].transform.position - target).magnitude);

        if (targetDistance > jointSum)
        {
            // Target is unreachable
            for (int i = 0; i < jointDistances.Length - 1; i++)
            {
                Transform j = Joints[i].transform;
                Vector3 targetDir = (target - j.position).normalized;
                Vector3 nextPosition = targetDir * jointDistances[i] + Joints[i].transform.position;

                Joints[i + 1].transform.position = nextPosition;
                Joints[i].transform.localRotation = Quaternion.LookRotation(targetDir, transform.up);
            }

            Vector3 t = (target - Joints[Joints.Length - 1].transform.position).normalized;
            Joints[Joints.Length - 1].transform.localRotation = Quaternion.LookRotation(t, transform.up);
        }
        else
        {
            Vector3 b = Joints[0].transform.position;
            Joints[Joints.Length - 1].transform.localPosition =
                Joints[Joints.Length - 2].transform.InverseTransformPoint(target);
            // Forward Reaching
            for (int i = Joints.Length - 2; i >= 0; i--)
            {
                Transform prevJoint = Joints[i + 1].transform;
                Transform nextJoint = Joints[i].transform;

                Vector3 newDir = (nextJoint.position - prevJoint.position).normalized;
                Vector3 nextPosition = newDir * jointDistances[i] + prevJoint.transform.position;

                if (i > 0)
                    Joints[i].transform.localPosition = Joints[i - 1].transform.InverseTransformPoint(nextPosition);
                else
                    Joints[i].transform.position = nextPosition;
            }

            // Joints[0].transform.position = b;
            // // Backward Reaching
            // for (int i = 0; i < Joints.Length - 1; i++)
            // {
            //     Transform nextJoint = Joints[i + 1].transform;
            //     Transform prevJoint = Joints[i].transform;
            //
            //     Vector3 newDir = (nextJoint.position - prevJoint.position).normalized;
            //     Vector3 nextPosition = newDir * jointDistances[i] + prevJoint.transform.position;
            //
            //     Joints[i + 1].transform.position = nextPosition;
            //
            //     // Look at the next joint
            //     Joints[i].transform.localRotation = Quaternion.LookRotation(newDir, transform.up);
            // }
            //
            Joints[Joints.Length - 1].transform.localRotation = Quaternion.LookRotation(target, transform.up);
        }
    }
}