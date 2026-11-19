using System;
using UnityEngine;

[Serializable]
public class MotorUnit
{
    [Tooltip("Локальная позиция мотора")] public Vector3 localPosition;
    [Tooltip("+1 = CW (по часовой сверху), -1 = CCW")] public int spinDir = 1;
    [HideInInspector] public float rpm, cmd, thrustN, torqueNm, groundH, dw;
    [HideInInspector] public bool vrs;
}
public struct CrashInfo
{
    public Vector3 point;             // точка контакта
    public Vector3 relativeVelocity;  // скорость удара
    public float speed;               // её модуль
    public Collider other;            // во что ударились
}
public class QuadPhysics : MonoBehaviour
{
    [Header("Рама")]
    [Tooltip("Масса, кг")] [Range(0.2f, 5f)] public float massKg = 1.2f;
    [Tooltip("Полуразмер X-рамы: моторы в (±d,0,±d), м")] [Range(0.08f, 0.5f)] public float motorOffset = 0.16f;
    [Tooltip("Пересчитать инерцию автоматически из массы и рамы")] public bool autoInertia = true;
    public Vector3 inertia = new Vector3(0.015f, 0.028f, 0.015f);
    [Tooltip("Порядок: FL, FR, BR, BL (как Betaflight X)")]
    public MotorUnit[] motors =
    {
        new MotorUnit { spinDir = 1 }, new MotorUnit { spinDir = -1 },
        new MotorUnit { spinDir = 1 }, new MotorUnit { spinDir = -1 }
    };

    [Header("Моторы и винты")]
    [Tooltip("Максимальные обороты при полном напряжении")] public float maxRpm = 26000f;
    [Tooltip("Максимальная статическая тяга одного мотора, г")] public float maxThrustGrams = 1300f;
    [Tooltip("Радиус винта, м (5 дюймов ≈ 0.0635)")] public float propRadius = 0.0635f;
    [Tooltip("Отношение момента сопротивления к тяге, м (kQ/kT)")] [Range(0.005f, 0.04f)] public float torqueRatio = 0.016f;
    [Tooltip("Момент инерции винта+ротора, кг·м²")] public float propInertia = 2.5e-5f;
    [Tooltip("Постоянная раскрутки, с")] [Range(0.005f, 0.3f)] public float spoolUpTime = 0.035f;
    [Tooltip("Постоянная торможения, с")] [Range(0.005f, 0.4f)] public float spoolDownTime = 0.055f;
    [Tooltip("Холостые обороты при armed, доля")] [Range(0, 0.15f)] public float idlePercent = 0.055f;
    [Tooltip("Эффективность мотор+винт, г/Вт (для тока батареи)")] [Range(2, 12)] public float gramsPerWatt = 6f;

    [Header("Аэродинамика винтов")]
    [Tooltip("Потеря тяги при наборе высоты / рост при снижении (осевой обдув)")] [Range(0, 0.6f)] public float inflowCoef = 0.28f;
    [Tooltip("H-force: боковое торможение диском винта, доля тяги на м/с")] [Range(0, 0.05f)] public float hForceCoef = 0.012f;
    [Tooltip("Экранирование землёй (ground effect)")] public bool groundEffect = true;
    [Range(1f, 1.4f)] public float groundEffectMax = 1.22f;
    [Tooltip("Vortex Ring State — провал тяги при быстром вертикальном снижении")] public bool vortexRingState = true;
    [Tooltip("Гироскопическая прецессия винтов")] public bool gyroscopicEffect = true;

    [Header("Аэродинамика корпуса")]
    [Tooltip("Cd·A по осям корпуса (X бок, Y верх/низ, Z перед), м²")]
    public Vector3 bodyCdA = new Vector3(0.012f, 0.03f, 0.012f);
    [Tooltip("Демпфирование вращения по осям корпуса")]
    public Vector3 angularCd = new Vector3(0.0015f, 0.002f, 0.0015f);

    [Header("Краши")]
    [Tooltip("Скорость удара для краша, м/с")] [Range(1, 15)] public float crashSpeed = 6f;
    public bool drawDebugForces = false;

    public bool Crashed { get; private set; }
    public bool Wrecked { get; private set; }   // дрон разобран на обломки
    public bool GroundContact { get; private set; }
    public bool VrsActive { get; private set; }
    public float PowerW { get; private set; }
    public float TotalThrustN { get; private set; }
    public float Airspeed { get; private set; }
    public float GroundEffectFactor { get; private set; } = 1f;
    public event Action OnCrash;
    public event Action<CrashInfo> OnCrashDetailed;
    public event Action OnReset;

   [Header("Ссылки")]
    [Tooltip("Система ветра; если пусто — найдётся сама при старте")]
    public WindSystem wind;

    Rigidbody rb;
    float kT, kQ;
    readonly RaycastHit[] hits = new RaycastHit[8];

    public Rigidbody Rb => rb != null ? rb : GetComponent<Rigidbody>();

    void Awake()
    {
        if (wind == null) wind = FindObjectOfType<WindSystem>();
        SetupRigidbody();
        RecalcCoeffs();
        // AutoLayout() больше НЕ вызывается автоматически:
        // позиции моторов, заданные в редакторе, сохраняются.
    }

    [ContextMenu("Применить параметры рамы (Rigidbody)")]
    public void SetupRigidbody()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        ApplyFrame();
    }

    public void ApplyFrame()
    {
        if (rb == null) rb = GetComponent<Rigidbody>();
        if (rb == null) return;
        rb.mass = massKg;
        rb.linearDamping = 0; rb.angularDamping = 0;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.solverIterations = 12;
        rb.centerOfMass = Vector3.zero;
        rb.inertiaTensor = autoInertia
            ? massKg * new Vector3(motorOffset * motorOffset * 0.5f, motorOffset * motorOffset * 0.9f, motorOffset * motorOffset * 0.5f)
            : inertia;
        rb.inertiaTensorRotation = Quaternion.identity;
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        foreach (var m in motors)
        {
            Vector3 wp = transform.TransformPoint(m.localPosition);
            Gizmos.DrawWireSphere(wp, propRadius);
            Gizmos.DrawLine(transform.position, wp);
        }
    }

    [ContextMenu("Auto Layout X")]
    public void AutoLayout()
    {
        float d = motorOffset;
        motors[0].localPosition = new Vector3(-d, 0, d);  // FL
        motors[1].localPosition = new Vector3(d, 0, d);   // FR
        motors[2].localPosition = new Vector3(d, 0, -d);  // BR
        motors[3].localPosition = new Vector3(-d, 0, -d); // BL
    }

    void RecalcCoeffs()
    {
        float wMax = maxRpm * 2f * Mathf.PI / 60f;
        kT = (maxThrustGrams * 0.001f * 9.81f) / (wMax * wMax);
        kQ = kT * torqueRatio;
    }
    void OnValidate() { RecalcCoeffs(); }

    float Density => wind != null ? wind.airDensity : 1.225f;
    float DensityRatio => Density / 1.225f;

    public float MaxThrustPerMotorN(float vbat, float fullV)
        => maxThrustGrams * 0.001f * 9.81f * Mathf.Clamp(vbat / fullV, 0.3f, 1f) * DensityRatio;

    public float HoverThrottle01(float vbat, float fullV)
    {
        float t = massKg * Mathf.Abs(Physics.gravity.y) / (4f * Mathf.Max(0.1f, MaxThrustPerMotorN(vbat, fullV)));
        return Mathf.Clamp01(Mathf.Sqrt(t));
    }

    public void SetCommands(float[] cmd)
    {
        for (int i = 0; i < 4; i++) motors[i].cmd = Mathf.Clamp01(cmd[i]);
    }

    // Интеграция оборотов (вызывается несколько раз за FixedUpdate полётным контроллером)
    public void StepMotors(float dt, float vbat, float fullV, bool enabled)
    {        if (Wrecked) return;
        float vf = Mathf.Clamp(vbat / fullV, 0.3f, 1f);
        float wMax = maxRpm * vf;
        foreach (var m in motors)
        {
            float target = enabled && !Crashed ? (idlePercent + m.cmd * (1f - idlePercent)) * wMax : 0f;
            float tau = target > m.rpm ? spoolUpTime : spoolDownTime;
            float prev = m.rpm;
            m.rpm += (target - m.rpm) * (1f - Mathf.Exp(-dt / Mathf.Max(0.001f, tau)));
            m.dw = (m.rpm - prev) * 2f * Mathf.PI / 60f / dt;
        }
    }

    // Силы — один раз за физический шаг
    public void ApplyAero(Vector3 windVec)
    {        if (Wrecked)
        {
            TotalThrustN = 0; PowerW = 0; Airspeed = 0;
            VrsActive = false; GroundEffectFactor = 1f; GroundContact = false;
            return;
        }
        RecalcCoeffs();
        float rho = Density, dr = DensityRatio;
        float area = Mathf.PI * propRadius * propRadius;
        TotalThrustN = 0; PowerW = 0; VrsActive = false; GroundContact = false;
        float geSum = 0;
        Vector3 up = transform.up;
        Vector3 omega = rb.angularVelocity;

        foreach (var m in motors)
        {
            Vector3 wp = transform.TransformPoint(m.localPosition);
            float w = m.rpm * 2f * Mathf.PI / 60f;
            float tStatic = kT * w * w * dr;
            Vector3 vrel = rb.GetPointVelocity(wp) - windVec;
            float va = Vector3.Dot(vrel, up);
            float vi = Mathf.Sqrt(Mathf.Max(0.5f, m.thrustN) / (2f * rho * area));

            // осевой обдув
            float fIn = Mathf.Clamp(1f - inflowCoef * va / vi, 0.35f, 1.25f);
            // VRS
            m.vrs = false;
            if (vortexRingState && va < 0 && -va > 0.6f * vi && -va < 1.6f * vi && m.cmd > 0.1f)
            { fIn *= 0.72f; m.vrs = true; VrsActive = true; }
            // ground effect (Cheeseman–Bennett)
            float gh = GroundHeightAt(wp);
            m.groundH = wp.y - gh;
            if (m.groundH < 0.06f) GroundContact = true;
            float ge = 1f;
            if (groundEffect && m.groundH < 4f * propRadius)
            {
                float h = Mathf.Max(m.groundH, propRadius * 0.35f);
                float r = propRadius / (4f * h);
                ge = Mathf.Clamp(1f / (1f - r * r), 1f, groundEffectMax);
            }
            geSum += ge;

            float T = Mathf.Max(0, tStatic * fIn * ge);
            m.thrustN = T;
            m.torqueNm = kQ * w * w * dr;
            TotalThrustN += T;
            float eff = gramsPerWatt * (1.15f - 0.5f * m.cmd);
            PowerW += T / 0.00981f / Mathf.Max(1f, eff);

            // H-force: торможение диском при боковом движении
            Vector3 lat = vrel - up * va;
            Vector3 force = up * T - lat * (hForceCoef * T);
            rb.AddForceAtPosition(force, wp);

            // реактивный момент + переходный момент раскрутки
            float yawT = m.spinDir * (m.torqueNm + propInertia * m.dw);
            rb.AddTorque(Vector3.up * yawT);

            // гироскопическая прецессия: tau = omega_body x L_prop
            if (gyroscopicEffect)
                rb.AddTorque(Vector3.Cross(omega, up * (m.spinDir * propInertia * w)));
        }
        GroundEffectFactor = geSum / 4f;

        // drag корпуса (в осях тела)
        Vector3 vBody = transform.InverseTransformDirection(rb.linearVelocity - windVec);
        Vector3 dragLocal = new Vector3(
            -0.5f * rho * bodyCdA.x * vBody.x * Mathf.Abs(vBody.x),
            -0.5f * rho * bodyCdA.y * vBody.y * Mathf.Abs(vBody.y),
            -0.5f * rho * bodyCdA.z * vBody.z * Mathf.Abs(vBody.z));
        rb.AddForce(transform.TransformDirection(dragLocal));
        Vector3 wBody = transform.InverseTransformDirection(omega);
        Vector3 adLocal = new Vector3(
            -angularCd.x * wBody.x * Mathf.Abs(wBody.x),
            -angularCd.y * wBody.y * Mathf.Abs(wBody.y),
            -angularCd.z * wBody.z * Mathf.Abs(wBody.z));
        rb.AddTorque(transform.TransformDirection(adLocal));

        Airspeed = (rb.linearVelocity - windVec).magnitude;
    }

    public float GroundHeightAt(Vector3 p)
    {
        int n = Physics.RaycastNonAlloc(p + Vector3.up * 0.05f, Vector3.down, hits, 60f);
        float best = -100f;
        for (int i = 0; i < n; i++)
        {
            if (hits[i].rigidbody == rb) continue;
            if (hits[i].point.y > best) best = hits[i].point.y;
        }
        return best > -99f ? best : 0f;
    }

    void OnCollisionEnter(Collision c)
    {
        GroundContact = true;
        if (Wrecked || Crashed) return;
        float speed = c.relativeVelocity.magnitude;
        if (speed > crashSpeed)
        {
            Crashed = true;
            Vector3 p = c.contactCount > 0 ? c.GetContact(0).point : transform.position;
            OnCrash?.Invoke();                       // FlightController дизармит
            OnCrashDetailed?.Invoke(new CrashInfo    // DroneBreakup разваливает
            { point = p, relativeVelocity = c.relativeVelocity, speed = speed, other = c.collider });
        }
    }
    void OnCollisionStay() { GroundContact = true; }

    public void ResetState()
    {
        OnReset?.Invoke();      // DroneBreakup соберёт обломки обратно
        ExitWreckedState();
        Crashed = false;
        foreach (var m in motors) { m.rpm = 0; m.cmd = 0; m.thrustN = 0; m.dw = 0; m.vrs = false; }
        PowerW = TotalThrustN = 0;
    }

    // Вызывается из DroneBreakup: корень остаётся маркером места крушения
    public void EnterWreckedState()
    {
        Wrecked = true;
        Crashed = true;
        foreach (var m in motors) { m.rpm = 0; m.cmd = 0; m.thrustN = 0; m.dw = 0; m.vrs = false; }
        PowerW = 0; TotalThrustN = 0; VrsActive = false;
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.isKinematic = true;      // не уничтожаем: HUD/телеметрия продолжают работать
        }
    }

    public void ExitWreckedState()
    {
        Wrecked = false;
        if (rb != null)
        {
            rb.isKinematic = false;
            SetupRigidbody();
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }

    void OnDrawGizmos()
    {
        if (!drawDebugForces || !Application.isPlaying) return;
        Gizmos.color = Color.green;
        foreach (var m in motors)
        {
            Vector3 wp = transform.TransformPoint(m.localPosition);
            Gizmos.DrawRay(wp, transform.up * m.thrustN * 0.1f);
        }
    }
}