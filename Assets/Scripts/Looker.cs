using UnityEngine;

public class Looker : MonoBehaviour
{
    public Transform Target;

    // Update is called once per frame
    private void Update()
    {
        transform.localRotation = Quaternion.LookRotation(Target.position, Vector3.up);
    }
}