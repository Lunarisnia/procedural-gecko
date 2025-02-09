using System.Collections;
using UnityEngine;

public class LegStepper : MonoBehaviour
{
    public Transform LegTarget;
    public Transform Home;
    public float maxDistanceFromHome = 1.0f;
    public float MoveDuration = 1.0f;

    private bool IsMoving;

    // Update is called once per frame
    private void Update()
    {
        if (IsMoving) return;

        Vector3 targetToHome = Home.position - LegTarget.position;

        float distanceToHome = targetToHome.magnitude;
        if (distanceToHome > maxDistanceFromHome)
            // LegTarget.position = Home.position;
            StartCoroutine(MoveToHome());
    }

    private IEnumerator MoveToHome()
    {
        IsMoving = true;
        float timeElapsed = 0;

        Vector3 startPoint = LegTarget.position;
        Vector3 endPoint = Home.position;

        do
        {
            timeElapsed += Time.deltaTime;
            float normalizedTime = timeElapsed / MoveDuration;

            LegTarget.position = Vector3.Lerp(startPoint, endPoint, normalizedTime);

            yield return null;
        } while (timeElapsed < MoveDuration);

        IsMoving = false;
    }
}