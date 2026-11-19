using UnityEngine;

public class FlightController : MonoBehaviour
{
    [Header("Ссылки (авто с этого же объекта)")]
    public DroneInputManager input;
    public QuadPhysics physics;
    public BatteryModel battery;
    public WindSystem wind;
    public UdpLink udp;

    [Header("Режимы")]
    public FlightMode defaultMode = FlightMode.Angle;
    [Tooltip("SwA (AUX2) переключает Acro/Acro↔Angle, как на реальной аппаратуре")]
    public bool useAuxModeSwitch = true;
    [Tooltip("Частота внутреннего цикла PID, Гц")] [Range(100, 1000)] public int loopHz = 400;

    [Header("PID угловых скоростей (выход — доля оборотов моторов)")]
    public PidController rateRoll = new PidController { kp = 0.0018f, ki = 0.0035f, kd = 0.000045f, iLimit = 0.25f, outputLimit = 0.55f };
    public PidController ratePitch = new PidController { kp = 0.0018f, ki = 0.0035f, kd = 0.000045f, iLimit = 0.25f, outputLimit = 0.55f };
    public PidController rateYaw = new PidController { kp = 0.004f, ki = 0.008f, kd = 0f, iLimit = 0.3f, outputLimit = 0.5f };

    [Header("Angle (самовыравнивание)")]
    [Tooltip("deg/s на градус ошибки наклона")] [Range(1, 15)] public float angleP = 7.5f;
    [Tooltip("Максимальный наклон в Angle, град")] [Range(10, 60)] public float angleLimit = 45f;

    [Header("AltHold")]
    [Range(0.2f, 4)] public float altP = 1.3f;
    public PidController altVelPid = new PidController { kp = 0.22f, ki = 0.12f, kd = 0f, iLimit = 0.25f, outputLimit = 0.45f };
    [Range(0.5f, 8)] public float maxClimbRate = 3f;
    [Tooltip("Скорость подстройки оценки газа висения")] [Range(0, 1)] public float hoverLearnRate = 0.15f;

    [Header("PosHold (удержание точки)")]
    public PidController posVelFwd = new PidController { kp = 3.2f, ki = 0.8f, kd = 0f, iLimit = 3f, outputLimit = 8f };
    public PidController posVelRight = new PidController { kp = 3.2f, ki = 0.8f, kd = 0f, iLimit = 3f, outputLimit = 8f };
    [Range(1, 20)] public float posMaxSpeed = 8f;
    [Range(5, 35)] public float posMaxTilt = 25f;

    [Header("Airmode и газ")]
    [Tooltip("Сохранять управляемость на нулевом газе")] public bool airmode = true;
    [Range(0, 0.15f)] public float airmodeIdle = 0.06f;
    [Range(0.3f, 1)] public float throttleLimit = 1f;

    [Header("Шум датчиков (реализм)")]
    [Tooltip("Шум гироскопа, deg/s (СКО)")] [Range(0, 5)] public float gyroNoiseDps = 0.4f;
    [Tooltip("Дрейф нуля гироскопа, deg/s")] [Range(0, 2)] public float gyroBiasDps = 0.15f;
    [Range(0, 1)] public float attitudeNoiseDeg = 0.15f;
    [Range(0, 0.5f)] public float baroNoiseM = 0.05f;
    [Range(0, 0.5f)] public float gpsNoiseM = 0.08f;

    [Header("Арм и безопасность")]
    public bool requireLowThrottleToArm = true;
    public bool autoDisarmOnCrash = true;
    [Tooltip("Автодизарм на земле с низким газом, с")] [Range(0.5f, 10)] public float groundDisarmTime = 2.5f;
    public FailsafeAction failsafeAction = FailsafeAction.AutoLand;
    [Tooltip("Максимальный газ, при котором разрешён ARM (0.08 = 8%)")]
    [Range(0f, 0.3f)] public float armMaxThrottle = 0.08f;
    [Tooltip("Арм стиками как в Betaflight: газ вниз + рыскание вправо. Дизарм: газ вниз + рыскание влево")]
    public bool armStickCombo = true;
    [Range(0.2f, 2f)] public float stickComboHold = 0.7f;
    [Tooltip("Автодизарм на земле с низким газом. Для отладки поставь 10 с")]
    public string ArmBlockReason { get; private set; } = "";
    float comboTimer;
    // ---- состояние ----
    public FlightMode Mode { get; private set; }
    public bool Armed { get; private set; }
    public bool Failsafe { get; private set; }
    public float FlightTime { get; private set; }
    public float MaxAlt { get; private set; }
    public float MaxGs { get; private set; }
    public float HoverEstimate { get; private set; } = 0.5f;
    public Vector3 HomePos { get; private set; }

    public float AttRoll => -SimMath.Norm180(transform.eulerAngles.z);     // + вправо    // + вправо
    public float AttPitchUp => -SimMath.Norm180(transform.eulerAngles.x);  // + нос вверх
    public float Heading => transform.eulerAngles.y;
    public float AltAgl => transform.position.y - physics.GroundHeightAt(transform.position);
    public float Vs => physics.Rb.linearVelocity.y;
    public float Gs => new Vector2(physics.Rb.linearVelocity.x, physics.Rb.linearVelocity.z).magnitude;
    public float HomeDist => Vector3.Distance(new Vector3(transform.position.x, 0, transform.position.z), new Vector3(HomePos.x, 0, HomePos.z));
    public float HomeBearing => Mathf.Atan2(HomePos.x - transform.position.x, HomePos.z - transform.position.z) * Mathf.Rad2Deg;

    Vector3 initPos, gyroBias; Quaternion initRot;
    float altTarget, groundTimer;
    readonly float[] cmd = new float[4];
    Vector3 windNow;

    void Awake()
    {
        if (!input) input = GetComponent<DroneInputManager>();
        if (!physics) physics = GetComponent<QuadPhysics>();
        if (!battery) battery = GetComponent<BatteryModel>();
        if (!udp) udp = GetComponent<UdpLink>();
        if (!wind) wind = FindObjectOfType<WindSystem>();
        Mode = defaultMode;
        physics.OnCrash += () => { if (autoDisarmOnCrash) Disarm(); };
        if (udp != null) udp.TelemetryJsonProvider = BuildTelemetryJson;
    }

    void Start()
    {
        initPos = transform.position; initRot = transform.rotation;
        HomePos = initPos;
        HoverEstimate = physics.HoverThrottle01(battery.Voltage, battery.FullVoltage);
    }

    void Update()
    {
        if (input.ConsumeArm()) TryArm();
        if (input.ConsumeDisarm()) Disarm();
        if (input.ConsumeReset(out bool full)) ResetDrone(full);
        if (input.ConsumeModeCycle()) SetMode((FlightMode)(((int)Mode + 1) % 4));
        if (udp != null)
        {
            int a = udp.ConsumeArmRequest(); if (a == 1) TryArm(); else if (a == 2) Disarm();
            int m = udp.ConsumeModeRequest(); if (m >= 1 && m <= 4) SetMode((FlightMode)(m - 1));
            if (udp.ConsumeResetRequest()) ResetDrone(true);
        }
        var s = input.Sticks;
        if (useAuxModeSwitch && Mathf.Abs(s.aux2) > 0.5f)
            Mode = s.aux2 > 0 ? FlightMode.Angle : FlightMode.Acro;
        if (Armed) FlightTime += Time.deltaTime;
        MaxAlt = Mathf.Max(MaxAlt, AltAgl);
        MaxGs = Mathf.Max(MaxGs, Gs);

        // причина отказа арма для HUD
        if (!Armed) { string r; CanArm(out r); ArmBlockReason = r; } else ArmBlockReason = "";

        // арм/дизарм стиками
        if (armStickCombo)
        {
            var st = input.Sticks;
            bool low = (st.throttle + 1f) * 0.5f <= armMaxThrottle;
            bool cArm = low && st.yaw > 0.85f;
            bool cDis = low && st.yaw < -0.85f;
            if (cArm || cDis)
            {
                if (comboTimer >= 0f)
                {
                    comboTimer += Time.deltaTime;
                    if (comboTimer >= stickComboHold)
                    {
                        if (cArm) TryArm(); else Disarm();
                        comboTimer = -1f;   // не повторять, пока стики не отпущены
                    }
                }
            }
            else comboTimer = 0f;
        }
    }

    public void SetMode(FlightMode m)
    {
        if (m == Mode) return;
        Mode = m;
        altTarget = AltAgl;
        altVelPid.Reset(); posVelFwd.Reset(); posVelRight.Reset();
    }

    public bool CanArm(out string reason)
    {
        reason = "";
        if (Armed) { reason = "уже ARMED"; return false; }
        if (physics.Crashed) { reason = "CRASHED — нажми R для сброса"; return false; }
        if (battery.IsEmpty) { reason = "BATTERY DEAD — Shift+R"; return false; }
        float thr01 = (input.Sticks.throttle + 1f) * 0.5f;
        if (requireLowThrottleToArm && thr01 > armMaxThrottle)
        { reason = $"ГАЗ {thr01 * 100f:0}% — опусти ниже {armMaxThrottle * 100f:0}%"; return false; }
        if (Vector3.Angle(transform.up, Vector3.up) > 30f)
        { reason = "НАКЛОН > 30° — поставь дрон ровно"; return false; }
        return true;
    }

    public void TryArm()
    {
        string reason;
        if (!CanArm(out reason)) { ArmBlockReason = reason; return; }
        ArmBlockReason = "";
        Armed = true; FlightTime = 0; HomePos = transform.position;
        rateRoll.Reset(); ratePitch.Reset(); rateYaw.Reset();
        altVelPid.Reset(); posVelFwd.Reset(); posVelRight.Reset();
        altTarget = AltAgl; groundTimer = 0;
    }

    public void Disarm() { Armed = false; Failsafe = false; }

    public void ResetDrone(bool full)
    {
        Disarm();
        physics.Rb.linearVelocity = Vector3.zero; physics.Rb.angularVelocity = Vector3.zero;
        transform.SetPositionAndRotation(initPos, initRot);
        physics.ResetState();
        input.ResetKeyboardThrottle();
        if (full) battery.ResetState();
        MaxAlt = MaxGs = FlightTime = 0;
        SetMode(defaultMode);
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        windNow = wind != null ? wind.GetWind(transform.position) : Vector3.zero;
        if (wind != null) wind.Current = windNow;

        Failsafe = Armed && input.Failsafe;
        bool motorsEnabled = Armed && !physics.Crashed && !battery.IsEmpty && !(Failsafe && failsafeAction == FailsafeAction.Drop);
        if (battery.IsEmpty && Armed) Disarm();

        float thrust01 = 0, rOut = 0, pOut = 0, yOut = 0;
        int sub = Mathf.Clamp(Mathf.CeilToInt(dt * loopHz), 1, 10);
        float h = dt / sub;

        for (int i = 0; i < sub; i++)
        {
            if (motorsEnabled)
            {
                ComputeControl(h, out thrust01, out rOut, out pOut, out yOut);
                Mix(thrust01, rOut, pOut, yOut);
            }
            else for (int k = 0; k < 4; k++) cmd[k] = 0;

            physics.SetCommands(cmd);
            physics.StepMotors(h, battery.Voltage, battery.FullVoltage, motorsEnabled);
        }
        physics.ApplyAero(windNow);
        battery.Step(dt, motorsEnabled ? physics.PowerW : 0f);

        // автодизарм на земле
        if (Armed && physics.GroundContact && thrust01 < 0.12f && Mathf.Abs(Vs) < 0.3f)
        {
            groundTimer += dt;
            if (groundTimer > groundDisarmTime) Disarm();
        }
        else groundTimer = 0;

        // дрейф нуля гироскопа
        gyroBias += new Vector3(SimMath.Gaussian(), SimMath.Gaussian(), SimMath.Gaussian()) * (gyroBiasDps * 0.05f * dt);
        gyroBias = Vector3.ClampMagnitude(gyroBias, gyroBiasDps);
    }

    void ComputeControl(float dt, out float thrust01, out float rOut, out float pOut, out float yOut)
    {
        var s = input.Sticks;
        thrust01 = Mathf.Clamp01((s.throttle + 1f) * 0.5f) * throttleLimit;
        float spRoll, spPitch, spYaw;
        spYaw = SimMath.AcroRate(s.yaw, input.yawRcRate, input.yawSuperRate);

        FlightMode eff = Mode;
        if (Failsafe && failsafeAction == FailsafeAction.AutoLand)
        {
            eff = FlightMode.AltHold;
            s = ControlFrame.Neutral; s.throttle = 0f;
            altTarget = Mathf.Max(0f, altTarget - 0.7f * dt);
            spYaw = 0;
        }

        Vector3 euler = transform.eulerAngles;
        float rollMeas = -SimMath.Norm180(euler.z) + SimMath.Gaussian() * attitudeNoiseDeg;
        float pitchDownMeas = SimMath.Norm180(euler.x) + SimMath.Gaussian() * attitudeNoiseDeg;

        if (eff == FlightMode.Acro)
        {
            spRoll = SimMath.AcroRate(s.roll, input.rcRate, input.superRate);
            spPitch = SimMath.AcroRate(s.pitch, input.rcRate, input.superRate);
        }
        else
        {
            float tRoll = s.roll * angleLimit;
            float tPitchDown = s.pitch * angleLimit;

            if (eff == FlightMode.PosHold)
            {
                Quaternion yawRot = Quaternion.Euler(0, Heading, 0);
                Vector3 vYaw = Quaternion.Inverse(yawRot) * physics.Rb.linearVelocity;
                Vector3 gpsErr = new Vector3(SimMath.Gaussian(), 0, SimMath.Gaussian()) * gpsNoiseM;
                float aF = posVelFwd.Update(s.pitch * -posMaxSpeed, vYaw.z + gpsErr.z, dt);
                float aR = posVelRight.Update(s.roll * posMaxSpeed, vYaw.x + gpsErr.x, dt);
                float g = Mathf.Abs(Physics.gravity.y);
                tPitchDown = Mathf.Clamp(Mathf.Atan2(-aF, g) * Mathf.Rad2Deg, -posMaxTilt, posMaxTilt);
                
                tRoll = Mathf.Clamp(Mathf.Atan2(aR, g) * Mathf.Rad2Deg, -posMaxTilt, posMaxTilt);
            }
            spRoll = angleP * (tRoll - rollMeas);
            spPitch = angleP * (tPitchDown - pitchDownMeas);

            if (eff == FlightMode.AltHold || eff == FlightMode.PosHold)
            {
                float thrStick = s.throttle;
                if (Mathf.Abs(thrStick) > 0.12f) altTarget += thrStick * maxClimbRate * dt;
                else if (!(Failsafe)) { /* удержание */ }
                float alt = AltAgl + SimMath.Gaussian() * baroNoiseM;
                float velT = Mathf.Clamp(altP * (altTarget - alt), -maxClimbRate, maxClimbRate);
                float hover = physics.HoverThrottle01(battery.Voltage, battery.FullVoltage);
                HoverEstimate += (hover - HoverEstimate) * 0.01f;
                thrust01 = Mathf.Clamp01(HoverEstimate + altVelPid.Update(velT, Vs, dt));
                if (Mathf.Abs(Vs) < 0.5f)
                    HoverEstimate += (thrust01 - HoverEstimate) * hoverLearnRate * dt;
            }
        }

        // гироскоп в осях тела + шум
        Vector3 wBody = transform.InverseTransformDirection(physics.Rb.angularVelocity) * Mathf.Rad2Deg;
        wBody += new Vector3(SimMath.Gaussian(), SimMath.Gaussian(), SimMath.Gaussian()) * gyroNoiseDps + gyroBias;
        pOut = ratePitch.Update(spPitch, wBody.x, dt);   // + nose down
        yOut = rateYaw.Update(spYaw, wBody.y, dt);       // + yaw right
        rOut = rateRoll.Update(spRoll, -wBody.z, dt);    // + roll right
    }

    // Микшер X: FL, FR, BR, BL; spinDirs FL+ FR- BR+ BL-
    void Mix(float t, float r, float p, float y)
    {
        if (airmode) t = Mathf.Max(t, airmodeIdle);
        cmd[0] = t + r - p + y; // FL
        cmd[1] = t - r - p - y; // FR
        cmd[2] = t - r + p + y; // BR
        cmd[3] = t + r + p - y; // BL
        float min = Mathf.Min(Mathf.Min(cmd[0], cmd[1]), Mathf.Min(cmd[2], cmd[3]));
        float max = Mathf.Max(Mathf.Max(cmd[0], cmd[1]), Mathf.Max(cmd[2], cmd[3]));
        if (airmode && min < 0) for (int i = 0; i < 4; i++) cmd[i] -= min;
        max = Mathf.Max(Mathf.Max(cmd[0], cmd[1]), Mathf.Max(cmd[2], cmd[3]));
        if (max > 1f) for (int i = 0; i < 4; i++) cmd[i] -= max - 1f;
        for (int i = 0; i < 4; i++) cmd[i] = Mathf.Clamp01(cmd[i]);
    }

    public string BuildTelemetryJson()
    {
        var d = new TelemetryData
        {
            t = Time.time, armed = Armed, failsafe = Failsafe, crashed = physics.Crashed, vrs = physics.VrsActive,
            mode = Mode.ToString(),
            vbat = battery.Voltage, amps = battery.Current, soc = battery.Soc, usedMah = battery.UsedMah,
            alt = AltAgl, gs = Gs, airspeed = physics.Airspeed, vs = Vs, hdg = Heading, dist = HomeDist,
            ge = physics.GroundEffectFactor
        };
        Vector3 p = transform.position, v = physics.Rb.linearVelocity;
        d.pos = new[] { p.x, p.y, p.z }; d.vel = new[] { v.x, v.y, v.z };
        d.att = new[] { AttRoll, AttPitchUp, Heading };
        d.wind = new[] { windNow.x, windNow.y, windNow.z };
        for (int i = 0; i < 4; i++)
        {
            d.rpm[i] = physics.motors[i].rpm;
            d.cmd[i] = physics.motors[i].cmd;
            d.thrustG[i] = physics.motors[i].thrustN / 0.00981f;
        }
        return JsonUtility.ToJson(d);
    }
}