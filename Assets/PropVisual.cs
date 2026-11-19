using UnityEngine;

public class PropVisual : MonoBehaviour
{
    public QuadPhysics physics;
    public int motorIndex;
    [Tooltip("Корень лопастей — скрывается целиком на высоких оборотах")]
    public GameObject bladesRoot;
    public Renderer blurDisc;
    public Renderer blades;
    [Tooltip("Обороты, выше которых лопасти скрываются в диск")]
    public float blurRpm = 3500f;

    float angle; Color discColor;

    void Start() { if (blurDisc) discColor = blurDisc.material.color; }

    void Update()
    {
        if (physics == null || motorIndex < 0 || motorIndex >= physics.motors.Length) return;
        var m = physics.motors[motorIndex];
        angle += -m.spinDir * m.rpm / 60f * 360f * Time.deltaTime;
        transform.localRotation = Quaternion.Euler(0, angle % 360f, 0);

        bool showBlades = m.rpm < blurRpm;
        if (bladesRoot != null) { if (bladesRoot.activeSelf != showBlades) bladesRoot.SetActive(showBlades); }
        else if (blades != null) blades.enabled = showBlades;

        if (blurDisc)
        {
            bool showDisc = m.rpm > blurRpm * 0.5f;
            if (blurDisc.enabled != showDisc) blurDisc.enabled = showDisc;
            float t = Mathf.Clamp01((m.rpm - blurRpm * 0.5f) / blurRpm);
            var c = discColor; c.a = t * 0.35f; blurDisc.material.color = c;
        }
    }
}