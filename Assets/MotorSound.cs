using UnityEngine;

public class MotorSound : MonoBehaviour
{
    public QuadPhysics physics;
    [Range(0, 1)] public float volume = 0.35f;
    AudioSource src;

    void Start()
    {
        src = GetComponent<AudioSource>();
        if (src == null) src = gameObject.AddComponent<AudioSource>();
        src.loop = true; src.spatialBlend = 1f; src.minDistance = 2f; src.maxDistance = 40f;
        src.playOnAwake = false;

        const int n = 44100;
        var data = new float[n];
        var rnd = new System.Random(7);
        for (int i = 0; i < n; i++)
        {
            float t = i / (float)n;
            float saw = 2f * (t * 120f - Mathf.Floor(t * 120f + 0.5f));
            float harm = Mathf.Sin(t * 2f * Mathf.PI * 240f) * 0.4f;
            data[i] = (saw * 0.5f + harm + (float)(rnd.NextDouble() * 2 - 1) * 0.25f) * 0.5f;
        }
        var clip = AudioClip.Create("motors", n, 1, n, false);
        clip.SetData(data, 0);
        src.clip = clip;
        src.Play();
    }

    void Update()
    {
        if (physics == null || src == null) return;
        float rpm = 0f;
        foreach (var m in physics.motors) rpm += m.rpm;
        rpm /= 4f;
        float norm = Mathf.Clamp01(rpm / Mathf.Max(1f, physics.maxRpm));
        src.pitch = 0.4f + norm * 2.2f;
        src.volume = volume * Mathf.Clamp01(norm * 3f);
    }
}