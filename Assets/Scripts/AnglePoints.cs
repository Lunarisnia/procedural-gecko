using UnityEngine;

public class AnglePoints : MonoBehaviour
{
    public Transform[] Points;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.A))
        {
            // Vector2 p1 = new(Points[0].position.x, Points[0].position.z);
            // Vector2 p2 = new(Points[1].position.x, Points[1].position.z);
            // float angle = Mathf.Atan2(p2.y - p1.y, p2.x - p1.x) * Mathf.Rad2Deg;
            // Debug.Log("Angle: " + angle);
        }

        Vector2 p1 = new(Points[0].position.x, Points[0].position.z);
        Vector2 p2 = new(Points[1].position.x, Points[1].position.z);
        float angle = Mathf.Atan2(p2.y - p1.y, p2.x - p1.x) * Mathf.Rad2Deg;
        // if (angle < 180)
        //     Points[1].position = new Vector3(Mathf.Cos(Points[1].position.x + Time.time), 0.0f,
        //         Mathf.Sin(Points[1].position.z + Time.time));
    }
}