using UnityEngine;

public class Teleporter : MonoBehaviour
{
    public float Radius = 2.0f;

    // Update is called once per frame
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.K)) transform.position = Random.insideUnitCircle * Radius;
    }
}