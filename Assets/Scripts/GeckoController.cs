using UnityEngine;

public class GeckoController : MonoBehaviour
{
    [SerializeField] private Transform target;

    // Gecko's Neck
    [SerializeField] private Transform neckBone;
    public bool activateHeadTracking = true;
    public float neckTurnSpeed = 1.0f;
    public float maxTurnDegree = 90.0f;

    [Header("Eyes")] [SerializeField] private Transform leftEye;
    public float leftEyeMaxY = 90.0f;
    public float leftEyeMinY = 90.0f;

    [SerializeField] private Transform rightEye;
    public float eyeTrackingSpeed = 5.0f;
    public float rightEyeMaxY = 90.0f;
    public float rightEyeMinY = 90.0f;


    private bool zeroed;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Z)) zeroed = !zeroed;
    }

    private void LateUpdate()
    {
        if (activateHeadTracking)
            HeadTrackingUpdate();
        EyeTrackingUpdate();
    }

    private void HeadTrackingUpdate()
    {
        if (zeroed)
        {
            neckBone.rotation = Quaternion.identity;
        }
        else
        {
            Quaternion currentLocalRotation = neckBone.localRotation;
            neckBone.localRotation = Quaternion.identity;

            Vector3 lookDirection = (target.position - neckBone.position).normalized;
            Vector3 lookDirectionLocal = neckBone.InverseTransformDirection(lookDirection);
            lookDirectionLocal =
                Vector3.RotateTowards(Vector3.forward, lookDirectionLocal, maxTurnDegree * Mathf.Deg2Rad, 0);

            Quaternion targetRotation = Quaternion.LookRotation(lookDirectionLocal);
            neckBone.localRotation = FrameIndependentSlerp(currentLocalRotation, targetRotation, neckTurnSpeed);
        }
    }

    private void EyeTrackingUpdate()
    {
        Vector3 lookDirection = (target.position - neckBone.position).normalized;

        Vector3 leftEyeLookDirection =
            leftEye.parent.InverseTransformDirection(lookDirection);
        Quaternion leftEyeRotation = Quaternion.LookRotation(leftEyeLookDirection);
        leftEye.localRotation = FrameIndependentSlerp(leftEye.localRotation, leftEyeRotation, eyeTrackingSpeed);


        Vector3 rightEyeLookDirection =
            rightEye.parent.InverseTransformDirection(lookDirection);
        Quaternion rightEyeDirection = Quaternion.LookRotation(rightEyeLookDirection);
        rightEye.localRotation = FrameIndependentSlerp(rightEye.localRotation, rightEyeDirection, eyeTrackingSpeed);

        float leftEyeCurrentYRotation = leftEye.localEulerAngles.y;
        float rightEyeCurrentYRotation = rightEye.localEulerAngles.y;
        if (leftEyeCurrentYRotation > 180) leftEyeCurrentYRotation -= 360;
        if (rightEyeCurrentYRotation > 180) rightEyeCurrentYRotation -= 360;


        float leftEyeClampedY = Mathf.Clamp(leftEyeCurrentYRotation, leftEyeMinY, leftEyeMaxY);
        float rightEyeClampedY = Mathf.Clamp(rightEyeCurrentYRotation, rightEyeMinY, rightEyeMaxY);
        leftEye.localEulerAngles = new Vector3(
            leftEye.localEulerAngles.x,
            leftEyeClampedY,
            leftEye.localEulerAngles.z
        );
        rightEye.localEulerAngles = new Vector3(
            rightEye.localEulerAngles.x,
            rightEyeClampedY,
            rightEye.localEulerAngles.z
        );
    }

    private Quaternion FrameIndependentSlerp(Quaternion a, Quaternion b, float t)
    {
        return Quaternion.Slerp(a, b, 1 - Mathf.Exp(-t * Time.deltaTime));
    }
}