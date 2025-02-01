using UnityEngine;

public class AngleRotateTest : MonoBehaviour
{
    public float offsetAngle = 90.0f;

    private float _angle;


    // Update is called once per frame
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            _angle += offsetAngle;
            Debug.Log("Angle Increased: " + _angle);
        }

        if (Input.GetKeyDown(KeyCode.W))
        {
            _angle -= offsetAngle;
            Debug.Log("Angle Decreased: " + _angle);
        }

        if (Input.GetKeyDown(KeyCode.R)) transform.localRotation = Quaternion.AngleAxis(_angle, Vector3.up);
    }
}