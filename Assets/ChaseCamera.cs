using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public enum CamMode { Chase, Fpv, Orbit }

public class ChaseCamera : MonoBehaviour
{
    [Header("Камеры")]
    public Transform target;
    public DroneInputManager input;
    public CamMode mode = CamMode.Chase;

    [Header("Chase")]
    public float chaseDist = 4f;
    public float chaseHeight = 1.6f;
    public float chaseSmooth = 8f;

    [Header("FPV")]
    public Vector3 fpvMount = new Vector3(0, 0.04f, 0.1f);
    [Range(0, 60)] public float fpvTiltBase = 15f;
    [Range(0, 60)] public float fpvTiltAuxRange = 40f;
    [Range(60, 130)] public float fpvFov = 100f;

    [Header("Orbit")]
    public float orbitDist = 6f;
    public float orbitPitch = 25f;
    public float orbitYaw = 0f;

    Camera cam;
    void Awake() { cam = GetComponent<Camera>(); }

    void Update()
    {
        if (input != null && input.ConsumeCameraCycle())
            mode = (CamMode)(((int)mode + 1) % 3);
#if ENABLE_INPUT_SYSTEM
        var m = Mouse.current;
        if (m != null && mode == CamMode.Orbit)
        {
            orbitDist = Mathf.Clamp(orbitDist - m.scroll.ReadValue().y * 0.005f, 1.5f, 30f);
            if (m.rightButton.isPressed)
            {
                orbitYaw += m.delta.x.ReadValue() * 0.2f;
                orbitPitch = Mathf.Clamp(orbitPitch - m.delta.y.ReadValue() * 0.2f, -10f, 85f);
            }
        }
#endif
    }

    void LateUpdate()
    {
        if (target == null) return;
        float tilt = fpvTiltBase;
        if (input != null && input.JoyConnected) tilt += (input.Sticks.aux1 + 1f) * 0.5f * fpvTiltAuxRange;

        switch (mode)
        {
            case CamMode.Fpv:
                transform.SetPositionAndRotation(target.TransformPoint(fpvMount), target.rotation * Quaternion.Euler(tilt, 0, 0));
                if (cam) cam.fieldOfView = fpvFov;
                break;
            case CamMode.Chase:
                Vector3 fwd = Quaternion.Euler(0, target.eulerAngles.y, 0) * Vector3.forward;
                Vector3 want = target.position - fwd * chaseDist + Vector3.up * chaseHeight;
                transform.position = Vector3.Lerp(transform.position, want, 1f - Mathf.Exp(-chaseSmooth * Time.deltaTime));
                transform.LookAt(target.position + Vector3.up * 0.2f);
                if (cam) cam.fieldOfView = 65f;
                break;
            case CamMode.Orbit:
                Quaternion rot = Quaternion.Euler(orbitPitch, orbitYaw, 0);
                transform.position = target.position + rot * (Vector3.back * orbitDist);
                transform.LookAt(target.position);
                if (cam) cam.fieldOfView = 60f;
                break;
        }
    }
}