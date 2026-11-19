### USER
я хочу сделать реалистичный симулятор квадрокоптера, помоги мне написать этот код на unity 6.3 в C# чтобы управлять этим квадрокоптером можно было одновременно с клавиатуры и с джойстика квадрокоптера sm001 этот джойстик должен иметь интерфейс и возможность калибровки, а так же связь по протоколу UDP по UdpClient чтобы можно было управлять с внешнего приложения, добавь в симулятор подробно настройки ветра и физику вращающихся винтов чтобы было похоже на управление реальным квадрокоптером, дополни код необходимыми деталями по своему усмотрению в соответствии со списком необходимых модулей в настоящем симуляторе, мне нужна просто сама логика управления квадрокоптером, выведи на экран в unity3d интерфейс и различную телеметрию управления дроном, состояние его систем, авиагоризонт и прочее. а так же сделай чтобы скрипты в инспекторе unity выглядели понятными выведи много параметров для настройки прямо в редакторе

### ASSISTANT
Отличная задача — соберём полноценный каркас симулятора: физика винтов (тяга ∝ ω², реакция момента, спин-ап моторов, ground effect, VRS, H-force), полётный контроллер с PID и режимами Acro/Angle/AltHold/PosHold, три источника ввода одновременно (клавиатура + SM001 через Input System с мастером калибровки + UDP), ветер с порывами/сдвигом/турбулентностью, модель батареи, HUD с авиагоризонтом и телеметрией, камеры Chase/FPV/Orbit и авто-сборка тестовой сцены.

## Настройка проекта (Unity 6.3)

1. Установи пакет **Input System** (Package Manager). В `Project Settings → Player → Active Input Handling` поставь **Both** (или Input System New).
2. Создай пустой GameObject `Simulator`, повесь на него **`SimBootstrap`** — нажми Play: сцена (земля, ворота, дрон, камера, HUD) соберётся сама. Все компоненты можно потом настроить вручную в инспекторе.
3. Создай 10 файлов ниже в `Assets/Scripts/`.

---

### 1. `SimCore.cs` — общие типы, PID, математика, хоткеи

```csharp
using System;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

public enum FlightMode { Acro, Angle, AltHold, PosHold }
public enum InputMixMode { MaxAbs, Sum, Priority }
public enum InputSourceKind { None, Keyboard, Joystick, Udp }
public enum FailsafeAction { Drop, AutoLand }
public enum CalWizardState { Off, Intro, Centers, Extremes, Verify }

[Serializable]
public struct ControlFrame
{
    // Все каналы -1..+1. Throttle: -1 = 0%, +1 = 100%.
    public float roll, pitch, throttle, yaw, aux1, aux2;
    public static ControlFrame Neutral => new ControlFrame { throttle = -1f };
    public float Get(int i) => i switch { 0 => roll, 1 => pitch, 2 => throttle, 3 => yaw, 4 => aux1, _ => aux2 };
}

[Serializable]
public class CalibrationData
{
    public int version = 1;
    public int[] sourceIndex = { 0, 1, 2, 3, 4, 5 };   // индекс оси HID
    public int[] buttonIndex = { -1, -1, -1, -1, -1, -1 }; // >=0 -> канал с кнопки (SwA)
    public bool[] invert = { false, true, true, false, false, false };
    public bool[] centered = { true, true, false, true, true, true }; // throttle без центра
    public float[] min = { -1, -1, -1, -1, -1, -1 };
    public float[] max = { 1, 1, 1, 1, 1, 1 };
    public float[] center = { 0, 0, 0, 0, 0, 0 };

    public float Normalize(int ch, float raw)
    {
        float v;
        if (!centered[ch])
        {
            float span = Mathf.Max(0.05f, max[ch] - min[ch]);
            v = 2f * Mathf.Clamp01((raw - min[ch]) / span) - 1f;
        }
        else
        {
            float c = center[ch];
            float span = raw >= c ? Mathf.Max(0.05f, max[ch] - c) : Mathf.Max(0.05f, c - min[ch]);
            v = Mathf.Clamp((raw - c) / span, -1f, 1f);
        }
        return invert[ch] ? -v : v;
    }
}

[Serializable]
public class PidController
{
    public float kp = 0.002f, ki = 0.004f, kd = 0.00005f;
    public float iLimit = 0.3f, outputLimit = 0.6f, dFilterHz = 90f;
    public float P, I, D, Output;
    float prevMeas, dFilt; bool hasPrev;

    public float Update(float setpoint, float meas, float dt)
    {
        if (dt <= 0f) return Output;
        float err = setpoint - meas;
        P = kp * err;
        float iStep = ki * err * dt;
        I = Mathf.Clamp(I + iStep, -iLimit, iLimit);
        float dRaw = hasPrev ? (meas - prevMeas) / dt : 0f;
        prevMeas = meas; hasPrev = true;
        float rc = 1f / (2f * Mathf.PI * Mathf.Max(1f, dFilterHz));
        dFilt += (dRaw - dFilt) * (dt / (rc + dt));
        D = -kd * dFilt;
        float o = P + I + D;
        if (Mathf.Abs(o) > outputLimit)
        {
            if (Mathf.Sign(err) == Mathf.Sign(o)) I -= iStep; // anti-windup
            o = Mathf.Clamp(o, -outputLimit, outputLimit);
        }
        Output = o;
        return o;
    }
    public void Reset() { P = I = D = Output = 0; prevMeas = dFilt = 0; hasPrev = false; }
}

[Serializable]
public class TelemetryData
{
    public float t; public bool armed, failsafe, crashed, vrs;
    public string mode;
    public float[] pos = new float[3], vel = new float[3], att = new float[3], wind = new float[3];
    public float[] rpm = new float[4], cmd = new float[4], thrustG = new float[4];
    public float vbat, amps, soc, usedMah, alt, gs, airspeed, vs, hdg, dist, ge;
}

public static class SimMath
{
    public static float Norm180(float a) { a %= 360f; if (a > 180f) a -= 360f; if (a < -180f) a += 360f; return a; }
    public static float Deadzone(float v, float dz)
    {
        if (Mathf.Abs(v) < dz) return 0f;
        return Mathf.Sign(v) * (Mathf.Abs(v) - dz) / (1f - dz);
    }
    public static float Expo(float v, float e) => v * (1f - e) + v * v * v * e;
    // Betaflight Actual Rates: возврат deg/s
    public static float AcroRate(float s, float rcRate, float superRate)
    {
        float denom = Mathf.Max(0.01f, 1f - Mathf.Abs(s) * superRate);
        return 200f * rcRate * s / denom;
    }
    static readonly System.Random rng = new System.Random();
    public static float Gaussian()
    {
        double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2));
    }
}

public enum SimKey
{
    ThrUp, ThrDn, YawL, YawR, PitchFwd, PitchBack, RollL, RollR, Fine,
    Arm, Disarm, Reset, ResetBattery, Mode, Camera,
    Help, Calib, Env, Inputs, Tune, HudToggle
}

// Единый слой хоткеев: работает и с новым Input System, и с legacy.
public static class Hotkeys
{
    public static bool Held(SimKey k)
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb == null) return false;
        switch (k)
        {
            case SimKey.ThrUp: return kb.wKey.isPressed;
            case SimKey.ThrDn: return kb.sKey.isPressed;
            case SimKey.YawL: return kb.aKey.isPressed;
            case SimKey.YawR: return kb.dKey.isPressed;
            case SimKey.PitchFwd: return kb.upArrowKey.isPressed;
            case SimKey.PitchBack: return kb.downArrowKey.isPressed;
            case SimKey.RollL: return kb.leftArrowKey.isPressed;
            case SimKey.RollR: return kb.rightArrowKey.isPressed;
            case SimKey.Fine: return kb.leftShiftKey.isPressed || kb.rightShiftKey.isPressed;
        }
        return false;
#elif ENABLE_LEGACY_INPUT_MANAGER
        switch (k)
        {
            case SimKey.ThrUp: return Input.GetKey(KeyCode.W);
            case SimKey.ThrDn: return Input.GetKey(KeyCode.S);
            case SimKey.YawL: return Input.GetKey(KeyCode.A);
            case SimKey.YawR: return Input.GetKey(KeyCode.D);
            case SimKey.PitchFwd: return Input.GetKey(KeyCode.UpArrow);
            case SimKey.PitchBack: return Input.GetKey(KeyCode.DownArrow);
            case SimKey.RollL: return Input.GetKey(KeyCode.LeftArrow);
            case SimKey.RollR: return Input.GetKey(KeyCode.RightArrow);
            case SimKey.Fine: return Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
        }
        return false;
#else
        return false;
#endif
    }

    public static bool Down(SimKey k)
    {
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb == null) return false;
        bool shift = kb.leftShiftKey.isPressed || kb.rightShiftKey.isPressed;
        switch (k)
        {
            case SimKey.Arm: return kb.spaceKey.wasPressedThisFrame;
            case SimKey.Disarm: return kb.backspaceKey.wasPressedThisFrame;
            case SimKey.Reset: return kb.rKey.wasPressedThisFrame && !shift;
            case SimKey.ResetBattery: return kb.rKey.wasPressedThisFrame && shift;
            case SimKey.Mode: return kb.mKey.wasPressedThisFrame;
            case SimKey.Camera: return kb.cKey.wasPressedThisFrame;
            case SimKey.Help: return kb.f1Key.wasPressedThisFrame;
            case SimKey.Calib: return kb.f2Key.wasPressedThisFrame;
            case SimKey.Env: return kb.f3Key.wasPressedThisFrame;
            case SimKey.Inputs: return kb.f4Key.wasPressedThisFrame;
            case SimKey.Tune: return kb.f5Key.wasPressedThisFrame;
            case SimKey.HudToggle: return kb.f6Key.wasPressedThisFrame;
        }
        return false;
#elif ENABLE_LEGACY_INPUT_MANAGER
        bool shift = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
        switch (k)
        {
            case SimKey.Arm: return Input.GetKeyDown(KeyCode.Space);
            case SimKey.Disarm: return Input.GetKeyDown(KeyCode.Backspace);
            case SimKey.Reset: return Input.GetKeyDown(KeyCode.R) && !shift;
            case SimKey.ResetBattery: return Input.GetKeyDown(KeyCode.R) && shift;
            case SimKey.Mode: return Input.GetKeyDown(KeyCode.M);
            case SimKey.Camera: return Input.GetKeyDown(KeyCode.C);
            case SimKey.Help: return Input.GetKeyDown(KeyCode.F1);
            case SimKey.Calib: return Input.GetKeyDown(KeyCode.F2);
            case SimKey.Env: return Input.GetKeyDown(KeyCode.F3);
            case SimKey.Inputs: return Input.GetKeyDown(KeyCode.F4);
            case SimKey.Tune: return Input.GetKeyDown(KeyCode.F5);
            case SimKey.HudToggle: return Input.GetKeyDown(KeyCode.F6);
        }
        return false;
#else
        return false;
#endif
    }
}
```

---

### 2. `DroneInputManager.cs` — клавиатура + SM001 + UDP, калибровка, микширование

```csharp
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(-50)]
public class DroneInputManager : MonoBehaviour
{
    public const int Channels = 6;
    public static readonly string[] ChannelNames = { "ROLL", "PITCH", "THR", "YAW", "AUX1(VrA)", "AUX2(SwA)" };

    [Header("Источники и микширование")]
    [Tooltip("MaxAbs — по каждому каналу побеждает источник с наибольшим отклонением (одновременная работа). Sum — сумма. Priority — последний активный источник целиком.")]
    public InputMixMode mixMode = InputMixMode.MaxAbs;
    public bool enableKeyboard = true;
    public bool enableJoystick = true;
    public bool enableUdp = true;

    [Header("Клавиатура")]
    [Tooltip("Скорость нарастания виртуального стика, ед/с")] [Range(0.5f, 10f)] public float kbAttack = 4.5f;
    [Tooltip("Скорость возврата стика в ноль, ед/с")] [Range(0.5f, 15f)] public float kbRelease = 7f;
    [Tooltip("Скорость изменения газа W/S, ед/с")] [Range(0.1f, 2f)] public float kbThrottleSpeed = 0.7f;
    [Tooltip("Множитель при зажатом Shift")] [Range(0.1f, 1f)] public float kbFineFactor = 0.35f;

    [Header("Ощущение стиков (после микса)")]
    [Range(0f, 0.2f)] public float deadzone = 0.02f;
    [Range(0f, 1f)] public float expoRollPitch = 0.25f;
    [Range(0f, 1f)] public float expoYaw = 0.2f;
    [Range(0f, 1f)] public float throttleExpo = 0.15f;
    [Tooltip("Betaflight Actual Rates: базовая скорость, x200 deg/s")] [Range(0.2f, 3f)] public float rcRate = 1.0f;
    [Tooltip("Ускорение к краям стика (0.7 ≈ 670 deg/s максимум)")] [Range(0f, 0.95f)] public float superRate = 0.7f;
    [Range(0.2f, 3f)] public float yawRcRate = 1.0f;
    [Range(0f, 0.95f)] public float yawSuperRate = 0.6f;

    [Header("Джойстик SM001")]
    [Tooltip("Индекс устройства в списке HID-джойстиков")] public int deviceIndex = 0;
    public CalibrationData cal = new CalibrationData();

    [Header("Failsafe")]
    [Tooltip("Считать потерей связи, если UDP-пакеты были, но пропали")] public bool failsafeOnUdpLost = true;
    [Tooltip("Failsafe при отключении джойстика, если он был основным")] public bool failsafeOnJoyLost = true;

    // ---- runtime ----
    public ControlFrame Sticks { get; private set; } = ControlFrame.Neutral;
    public float[] Mixed = new float[Channels];
    public float[] JoyCal = new float[Channels];
    public InputSourceKind[] ActiveSrc = new InputSourceKind[Channels];
    public bool Failsafe { get; private set; }
    public float KbThr01 => kbThr;
    public UdpLink udp;

    public List<string> DeviceNames = new List<string>();
    public List<string> AxisLabels = new List<string>();
    public List<float> AxisValues = new List<float>();
    public List<bool> ButtonValues = new List<bool>();
    public bool JoyConnected => joyDevice != null;
    public CalWizardState Wizard { get; private set; } = CalWizardState.Off;

    float kbThr; float[] kbAxes = new float[4];
    float lastKbActive, lastJoyActive, lastUdpActive, lastJoyDominant;
    bool udpEverFresh, joyEverActive;
    bool armFlag, disarmFlag, resetFlag, resetFullFlag, modeFlag, camFlag;
    float refreshTimer;
#if ENABLE_INPUT_SYSTEM
    Joystick joyDevice;
    List<AxisControl> axes = new List<AxisControl>();
    List<ButtonControl> buttons = new List<ButtonControl>();
#endif
    string CalPath => Path.Combine(Application.persistentDataPath, "sm001_calibration.json");

    void Awake()
    {
        udp = GetComponent<UdpLink>();
        LoadCal();
#if !ENABLE_INPUT_SYSTEM
        Debug.LogWarning("[Input] Пакет Input System не активен — джойстик недоступен, работает клавиатура.");
#endif
    }

    void Update()
    {
        float dt = Time.unscaledDeltaTime;
        refreshTimer -= dt;
        if (refreshTimer <= 0f) { RefreshDevices(); refreshTimer = 0.5f; }
        ReadJoystick();
        ReadKeyboard(dt);
        if (udp != null) udp.Poll();

        // one-shot команды
        if (Hotkeys.Down(SimKey.Arm)) armFlag = true;
        if (Hotkeys.Down(SimKey.Disarm)) disarmFlag = true;
        if (Hotkeys.Down(SimKey.Reset)) resetFlag = true;
        if (Hotkeys.Down(SimKey.ResetBattery)) resetFullFlag = true;
        if (Hotkeys.Down(SimKey.Mode)) modeFlag = true;
        if (Hotkeys.Down(SimKey.Camera)) camFlag = true;

        Mix();
        UpdateFailsafe();
        if (Wizard == CalWizardState.Extremes) CaptureExtremes();
    }

    // ---------- устройства ----------
    void RefreshDevices()
    {
#if ENABLE_INPUT_SYSTEM
        DeviceNames.Clear();
        var list = new List<Joystick>();
        foreach (var d in InputSystem.devices) if (d is Joystick j) { list.Add(j); DeviceNames.Add(j.displayName); }
        deviceIndex = Mathf.Clamp(deviceIndex, 0, Mathf.Max(0, list.Count - 1));
        var dev = list.Count > 0 ? list[deviceIndex] : null;
        if (dev != joyDevice)
        {
            joyDevice = dev; axes.Clear(); buttons.Clear(); AxisLabels.Clear();
            if (dev != null)
                foreach (var c in dev.allControls)
                {
                    if (c is AxisControl ax && !(c.parent is ButtonControl)) { axes.Add(ax); AxisLabels.Add(c.displayName); }
                    else if (c is ButtonControl b && !(b.parent is StickControl)) buttons.Add(b);
                }
        }
#endif
    }

    void ReadJoystick()
    {
#if ENABLE_INPUT_SYSTEM
        AxisValues.Clear(); ButtonValues.Clear();
        if (joyDevice == null) { for (int i = 0; i < Channels; i++) JoyCal[i] = 0; return; }
        foreach (var a in axes) AxisValues.Add(a.ReadValue());
        foreach (var b in buttons) ButtonValues.Add(b.isPressed);
        for (int i = 0; i < Channels; i++)
        {
            int bi = cal.buttonIndex[i];
            float raw = bi >= 0
                ? (bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f)
                : (cal.sourceIndex[i] < AxisValues.Count ? AxisValues[cal.sourceIndex[i]] : 0f);
            JoyCal[i] = cal.Normalize(i, raw);
        }
        joyEverActive |= joyDevice != null;
#endif
    }

    public void CycleDevice() { deviceIndex++; refreshTimer = 0; }
    public float RawAxis(int i) => i >= 0 && i < AxisValues.Count ? AxisValues[i] : 0f;
    public float RawChannel(int ch)
    {
        int bi = cal.buttonIndex[ch];
        if (bi >= 0) return bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f;
        return RawAxis(cal.sourceIndex[ch]);
    }

    // ---------- клавиатура ----------
    void ReadKeyboard(float dt)
    {
        if (!enableKeyboard) { for (int i = 0; i < 4; i++) kbAxes[i] = 0; return; }
        float fine = Hotkeys.Held(SimKey.Fine) ? kbFineFactor : 1f;
        float[] target =
        {
            (Hotkeys.Held(SimKey.RollR) ? 1 : 0) - (Hotkeys.Held(SimKey.RollL) ? 1 : 0),
            (Hotkeys.Held(SimKey.PitchFwd) ? 1 : 0) - (Hotkeys.Held(SimKey.PitchBack) ? 1 : 0),
            0,
            (Hotkeys.Held(SimKey.YawR) ? 1 : 0) - (Hotkeys.Held(SimKey.YawL) ? 1 : 0)
        };
        for (int i = 0; i < 4; i++)
        {
            if (i == 2) continue;
            float t = target[i] * fine;
            float speed = Mathf.Abs(t) > 0.01f ? kbAttack : kbRelease;
            kbAxes[i] = Mathf.MoveTowards(kbAxes[i], t, speed * dt);
        }
        if (Hotkeys.Held(SimKey.ThrUp)) kbThr += kbThrottleSpeed * fine * dt;
        if (Hotkeys.Held(SimKey.ThrDn)) kbThr -= kbThrottleSpeed * fine * dt;
        kbThr = Mathf.Clamp01(kbThr);
    }
    public void ResetKeyboardThrottle() { kbThr = 0; Array.Clear(kbAxes, 0, 4); }

    // ---------- микс ----------
    void Mix()
    {
        float t = Time.realtimeSinceStartup;
        float[] kb = { kbAxes[0], kbAxes[1], kbThr * 2f - 1f, kbAxes[3], 0, 0 };
        float[] joy = JoyCal;
        ControlFrame u = udp != null ? udp.Control : ControlFrame.Neutral;
        float[] net = { u.roll, u.pitch, u.throttle, u.yaw, u.aux1, u.aux2 };

        bool kbOn = enableKeyboard, joyOn = enableJoystick && JoyConnected, udpOn = enableUdp && udp != null && udp.Fresh;
        if (AnyActive(kb, 0.05f) || kbThr > 0.01f) lastKbActive = t;
        if (joyOn && AnyActive(joy, 0.05f)) lastJoyActive = t;
        if (udpOn) { lastUdpActive = t; udpEverFresh = true; }

        int prio = 0; // 0 kb,1 joy,2 udp
        if (lastJoyActive >= lastKbActive && lastJoyActive >= lastUdpActive) prio = 1;
        if (lastUdpActive >= lastKbActive && lastUdpActive >= lastJoyActive) prio = 2;

        for (int ch = 0; ch < Channels; ch++)
        {
            float v = 0; InputSourceKind src = InputSourceKind.None;
            float kv = kbOn ? kb[ch] : 0, jv = joyOn ? joy[ch] : 0, uv = udpOn ? net[ch] : 0;
            if (mixMode == InputMixMode.Priority)
            {
                v = prio == 2 ? uv : prio == 1 ? jv : kv;
                src = prio == 2 ? InputSourceKind.Udp : prio == 1 ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
            }
            else if (mixMode == InputMixMode.Sum)
            {
                v = Mathf.Clamp(kv + jv + uv, -1, 1);
                src = Mathf.Abs(uv) >= Mathf.Abs(jv) && Mathf.Abs(uv) >= Mathf.Abs(kv) ? InputSourceKind.Udp
                    : Mathf.Abs(jv) >= Mathf.Abs(kv) ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
            }
            else // MaxAbs; газ — максимум значения (любой источник может поднять газ)
            {
                if (ch == 2)
                {
                    v = -1;
                    if (kbOn && kv > v) { v = kv; src = InputSourceKind.Keyboard; }
                    if (joyOn && jv > v) { v = jv; src = InputSourceKind.Joystick; }
                    if (udpOn && uv > v) { v = uv; src = InputSourceKind.Udp; }
                }
                else
                {
                    if (Mathf.Abs(kv) > Mathf.Abs(v)) { v = kv; src = InputSourceKind.Keyboard; }
                    if (Mathf.Abs(jv) > Mathf.Abs(v)) { v = jv; src = InputSourceKind.Joystick; }
                    if (Mathf.Abs(uv) > Mathf.Abs(v)) { v = uv; src = InputSourceKind.Udp; }
                }
            }
            Mixed[ch] = v; ActiveSrc[ch] = src;
            if (src == InputSourceKind.Joystick && Mathf.Abs(v) > 0.05f) lastJoyDominant = t;
        }

        float dz = deadzone;
        var s = ControlFrame.Neutral;
        s.roll = SimMath.Expo(SimMath.Deadzone(Mixed[0], dz), expoRollPitch);
        s.pitch = SimMath.Expo(SimMath.Deadzone(Mixed[1], dz), expoRollPitch);
        s.yaw = SimMath.Expo(SimMath.Deadzone(Mixed[3], dz), expoYaw);
        s.throttle = SimMath.Expo(Mixed[2], throttleExpo);
        s.aux1 = Mixed[4]; s.aux2 = Mixed[5];
        Sticks = s;
    }
    static bool AnyActive(float[] a, float thr)
    { for (int i = 0; i < a.Length; i++) if (i != 2 && Mathf.Abs(a[i]) > thr) return true; return false; }

    void UpdateFailsafe()
    {
        bool udpLost = failsafeOnUdpLost && enableUdp && udpEverFresh && udp != null && !udp.Fresh;
        bool joyLost = failsafeOnJoyLost && enableJoystick && joyEverActive && !JoyConnected &&
                       Time.realtimeSinceStartup - lastJoyDominant < 3f;
        Failsafe = udpLost || joyLost;
    }

    // ---------- one-shot ----------
    public bool ConsumeArm() { bool f = armFlag; armFlag = false; return f; }
    public bool ConsumeDisarm() { bool f = disarmFlag; disarmFlag = false; return f; }
    public bool ConsumeReset(out bool full) { full = resetFullFlag; bool f = resetFlag || resetFullFlag; resetFlag = resetFullFlag = false; return f; }
    public bool ConsumeModeCycle() { bool f = modeFlag; modeFlag = false; return f; }
    public bool ConsumeCameraCycle() { bool f = camFlag; camFlag = false; return f; }

    // ---------- калибровка ----------
    public void BeginCal() { Wizard = CalWizardState.Intro; }
    public void CalCancel() { Wizard = CalWizardState.Off; LoadCal(); }
    public void CalNext()
    {
        switch (Wizard)
        {
            case CalWizardState.Intro: Wizard = CalWizardState.Centers; break;
            case CalWizardState.Centers:
                for (int i = 0; i < Channels; i++)
                {
                    float r = RawChannel(i);
                    cal.center[i] = r; cal.min[i] = r; cal.max[i] = r;
                }
                Wizard = CalWizardState.Extremes; break;
            case CalWizardState.Extremes: Wizard = CalWizardState.Verify; break;
            case CalWizardState.Verify: SaveCal(); Wizard = CalWizardState.Off; break;
        }
    }
    void CaptureExtremes()
    {
        for (int i = 0; i < Channels; i++)
        {
            float r = RawChannel(i);
            cal.min[i] = Mathf.Min(cal.min[i], r);
            cal.max[i] = Mathf.Max(cal.max[i], r);
        }
    }
    public void ResetCal() { cal = new CalibrationData(); SaveCal(); }
    public void SaveCal() { try { File.WriteAllText(CalPath, JsonUtility.ToJson(cal, true)); } catch (Exception e) { Debug.LogWarning(e.Message); } }
    public void LoadCal()
    {
        try { if (File.Exists(CalPath)) cal = JsonUtility.FromJson<CalibrationData>(File.ReadAllText(CalPath)); }
        catch (Exception e) { Debug.LogWarning(e.Message); }
        if (cal == null || cal.sourceIndex == null || cal.sourceIndex.Length != Channels) cal = new CalibrationData();
    }
}
```

---

### 3. `UdpLink.cs` — приём управления и отправка телеметрии по UDP

```csharp
using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

// Протокол (порт 14551 приём, 14550 телеметрия):
//  JSON: {"channels":[roll,pitch,thr,yaw,aux1,aux2],"arm":1,"mode":2,"reset":false}
//        channels: -1..1 (thr -1=0%). arm: 0=нет,1=arm,2=disarm. mode: 0=нет,1=Acro..4=PosHold.
//  CSV PWM: "1500,1500,1000,1500,1000,1000" (мкс, 4 или 6 значений).
[DefaultExecutionOrder(-40)]
public class UdpLink : MonoBehaviour
{
    [Header("Сеть")]
    public bool enableRx = true;
    public bool enableTx = true;
    [Range(1024, 65535)] public int listenPort = 14551;
    public string remoteHost = "127.0.0.1";
    [Range(1024, 65535)] public int remotePort = 14550;
    [Tooltip("Частота отправки телеметрии, Гц")] [Range(1, 100)] public int telemetryHz = 30;
    [Tooltip("Таймаут свежести управляющего пакета, с")] [Range(0.1f, 3f)] public float timeout = 0.5f;

    [Serializable]
    class RxPacket { public float[] channels; public int arm; public int mode; public bool reset; }

    public ControlFrame Control { get; private set; } = ControlFrame.Neutral;
    public bool Fresh => enableRx && gotAny && Time.realtimeSinceStartup - lastRxTime < timeout;
    public long RxCount, TxCount; public string LastError = "";
    public Func<string> TelemetryJsonProvider;

    UdpClient client; Thread rxThread; volatile bool running;
    readonly object locker = new object();
    RxPacket latest; bool gotAny; float lastRxTime, txTimer;
    int armReq, modeReq; bool resetReq;

    void OnEnable() { StartLink(); }
    void OnDisable() { StopLink(); }
    void OnDestroy() { StopLink(); }
    void OnApplicationQuit() { StopLink(); }

    void StartLink()
    {
        if (running) return;
        try
        {
            client = new UdpClient(listenPort);
            running = true;
            rxThread = new Thread(RxLoop) { IsBackground = true };
            rxThread.Start();
        }
        catch (Exception e) { LastError = e.Message; Debug.LogWarning("[UDP] " + e.Message); }
    }
    void StopLink()
    {
        running = false;
        try { client?.Close(); } catch { }
        client = null;
    }

    void RxLoop()
    {
        var any = new IPEndPoint(IPAddress.Any, 0);
        while (running)
        {
            try
            {
                byte[] data = client.Receive(ref any);
                string txt = Encoding.UTF8.GetString(data).Trim();
                RxPacket p = Parse(txt);
                if (p != null) lock (locker) { latest = p; }
            }
            catch (SocketException) { }
            catch (Exception) { if (!running) return; }
        }
    }

    RxPacket Parse(string txt)
    {
        try
        {
            if (txt.StartsWith("{")) return JsonUtility.FromJson<RxPacket>(txt);
            var parts = txt.Split(',');
            if (parts.Length >= 4)
            {
                var p = new RxPacket { channels = new float[6] };
                for (int i = 0; i < 6; i++)
                {
                    float us = i < parts.Length ? float.Parse(parts[i], CultureInfo.InvariantCulture) : 1500f;
                    p.channels[i] = Mathf.Clamp((us - 1500f) / 500f, -1f, 1f);
                }
                return p;
            }
        }
        catch (Exception e) { LastError = e.Message; }
        return null;
    }

    // Вызывается менеджером ввода на главном потоке
    public void Poll()
    {
        lock (locker)
        {
            if (latest == null) return;
            var p = latest; latest = null;
            gotAny = true; lastRxTime = Time.realtimeSinceStartup; RxCount++;
            if (p.channels != null && p.channels.Length >= 4)
            {
                var f = ControlFrame.Neutral;
                f.roll = Mathf.Clamp(p.channels[0], -1, 1);
                f.pitch = Mathf.Clamp(p.channels[1], -1, 1);
                f.throttle = Mathf.Clamp(p.channels[2], -1, 1);
                f.yaw = Mathf.Clamp(p.channels[3], -1, 1);
                if (p.channels.Length > 4) f.aux1 = Mathf.Clamp(p.channels[4], -1, 1);
                if (p.channels.Length > 5) f.aux2 = Mathf.Clamp(p.channels[5], -1, 1);
                Control = f;
            }
            if (p.arm != 0) armReq = p.arm;
            if (p.mode != 0) modeReq = p.mode;
            if (p.reset) resetReq = true;
        }
    }

    public int ConsumeArmRequest() { int r = armReq; armReq = 0; return r; }
    public int ConsumeModeRequest() { int r = modeReq; modeReq = 0; return r; }
    public bool ConsumeResetRequest() { bool r = resetReq; resetReq = false; return r; }

    void Update()
    {
        if (!enableTx || client == null || TelemetryJsonProvider == null) return;
        txTimer -= Time.unscaledDeltaTime;
        if (txTimer > 0) return;
        txTimer = 1f / Mathf.Max(1, telemetryHz);
        try
        {
            byte[] b = Encoding.UTF8.GetBytes(TelemetryJsonProvider());
            client.Send(b, b.Length, remoteHost, remotePort);
            TxCount++;
        }
        catch (Exception e) { LastError = e.Message; }
    }
}
```

---

### 4. `WindSystem.cs` — ветер: база, порывы, турбулентность, сдвиг по высоте

```csharp
using UnityEngine;

public class WindSystem : MonoBehaviour
{
    [Header("Базовый ветер")]
    [Tooltip("Скорость, м/с")] [Range(0, 25)] public float baseSpeed = 3f;
    [Tooltip("Направление, КУДА дует, градусы (0 = +Z / север)")] [Range(0, 360)] public float directionDeg = 270f;
    [Tooltip("Медленное блуждание направления, ±град")] [Range(0, 45)] public float dirWander = 12f;

    [Header("Порывы и турбулентность")]
    [Tooltip("Амплитуда порывов, м/с")] [Range(0, 15)] public float gustAmp = 2f;
    [Tooltip("Частота порывов")] [Range(0.02f, 2)] public float gustFreq = 0.25f;
    [Tooltip("Амплитуда мелкой турбулентности, м/с")] [Range(0, 8)] public float turbAmp = 0.8f;
    [Tooltip("Пространственный масштаб турбулентности, м")] [Range(1, 100)] public float turbScale = 25f;
    [Tooltip("Вертикальные порывы, м/с")] [Range(0, 5)] public float verticalAmp = 0.5f;

    [Header("Сдвиг ветра по высоте")]
    [Tooltip("Степень профиля (0 — нет сдвига, 0.14 — типичный атмосферный)")] [Range(0, 0.6f)] public float shearExponent = 0.14f;
    [Tooltip("Высота замера базовой скорости, м")] public float referenceHeight = 10f;

    [Header("Среда")]
    [Tooltip("Плотность воздуха, кг/м³ (влияет на тягу и drag)")] [Range(0.6f, 1.4f)] public float airDensity = 1.225f;
    [Tooltip("Гравитация, м/с²")] [Range(1, 20)] public float gravity = 9.81f;
    public float randomSeed = 17f;

    public Vector3 Current { get; set; } // последний сэмпл у дрона (для HUD)

    void Update()
    {
        if (!Mathf.Approximately(Physics.gravity.y, -gravity))
            Physics.gravity = new Vector3(0, -gravity, 0);
    }

    public Vector3 GetWind(Vector3 pos)
    {
        float t = Time.time;
        float dir = directionDeg + dirWander * (Mathf.PerlinNoise(randomSeed, t * 0.05f) * 2f - 1f);
        float h = Mathf.Max(0.1f, pos.y);
        float shear = Mathf.Pow(h / Mathf.Max(0.5f, referenceHeight), shearExponent);
        shear = Mathf.Clamp(shear, 0.2f, 2.5f);
        float gust = gustAmp * (Mathf.PerlinNoise(randomSeed + 100f, t * gustFreq) * 2f - 1f);
        float speed = Mathf.Max(0, baseSpeed * shear + gust);
        Vector3 horiz = Quaternion.Euler(0, dir, 0) * Vector3.forward * speed;
        Vector3 turb = new Vector3(
            Mathf.PerlinNoise(pos.x / turbScale + randomSeed, t * 0.7f) * 2f - 1f,
            (Mathf.PerlinNoise(pos.z / turbScale + randomSeed + 50f, t * 0.9f) * 2f - 1f) * verticalAmp,
            Mathf.PerlinNoise(pos.y / turbScale + randomSeed + 90f, t * 0.8f) * 2f - 1f) * turbAmp;
        return horiz + turb;
    }

    public void PresetCalm() { baseSpeed = 0; gustAmp = 0; turbAmp = 0.1f; verticalAmp = 0; }
    public void PresetBreeze() { baseSpeed = 4; gustAmp = 2; turbAmp = 0.8f; verticalAmp = 0.4f; }
    public void PresetStrong() { baseSpeed = 10; gustAmp = 5; turbAmp = 2.2f; verticalAmp = 1.2f; }
    public void PresetStorm() { baseSpeed = 16; gustAmp = 8; turbAmp = 3.5f; verticalAmp = 2f; }

    void OnDrawGizmosSelected()
    {
        Vector3 dir = Quaternion.Euler(0, directionDeg, 0) * Vector3.forward;
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(Vector3.up * 2, dir * (1f + baseSpeed * 0.3f));
    }
}
```

---

### 5. `BatteryModel.cs` — LiPo: просадка, расход, отсечка

```csharp
using UnityEngine;

public class BatteryModel : MonoBehaviour
{
    [Header("Пакет LiPo")]
    [Tooltip("Число банок (S)")] [Range(1, 8)] public int cells = 4;
    [Tooltip("Ёмкость, мАч")] public float capacityMah = 1500f;
    [Tooltip("Внутреннее сопротивление одной банки, Ом")] [Range(0.001f, 0.05f)] public float cellResistance = 0.008f;
    [Tooltip("Кривая напряжения банки от SoC (0..1)")]
    public AnimationCurve cellCurve = new AnimationCurve(
        new Keyframe(0, 3.0f), new Keyframe(0.1f, 3.3f), new Keyframe(0.3f, 3.55f),
        new Keyframe(0.6f, 3.75f), new Keyframe(0.9f, 4.0f), new Keyframe(1, 4.2f));
    [Tooltip("Потребление полётного контроллера/FPV, Вт")] public float idlePowerW = 3f;

    public float Voltage { get; private set; }
    public float Current { get; private set; }
    public float Soc { get; private set; } = 1f;
    public float UsedMah { get; private set; }
    public float CellVoltage => Voltage / cells;
    public float FullVoltage => cells * 4.2f;
    public bool IsLow => CellVoltage < 3.5f;
    public bool IsCritical => CellVoltage < 3.3f;
    public bool IsEmpty => CellVoltage < 3.0f;

    void Awake() { Voltage = FullVoltage; }

    public void Step(float dt, float motorPowerW)
    {
        float power = motorPowerW + idlePowerW;
        float ocv = cells * cellCurve.Evaluate(Soc);
        float i = power / Mathf.Max(1f, Voltage);
        float target = Mathf.Max(0f, ocv - i * cellResistance * cells);
        Voltage += (target - Voltage) * Mathf.Clamp01(dt * 12f); // динамика просадки
        Current = i;
        UsedMah += i * dt / 3.6f;
        Soc = Mathf.Clamp01(1f - UsedMah / capacityMah);
    }

    public void ResetState() { Soc = 1; UsedMah = 0; Current = 0; Voltage = FullVoltage; }
}
```

---

### 6. `QuadPhysics.cs` — моторы, винты, аэродинамика, краши

```csharp
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
    public bool GroundContact { get; private set; }
    public bool VrsActive { get; private set; }
    public float PowerW { get; private set; }
    public float TotalThrustN { get; private set; }
    public float Airspeed { get; private set; }
    public float GroundEffectFactor { get; private set; } = 1f;
    public event Action OnCrash;

    Rigidbody rb; WindSystem wind;
    float kT, kQ;
    readonly RaycastHit[] hits = new RaycastHit[8];

    public Rigidbody Rb => rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        wind = FindObjectOfType<WindSystem>();
        ApplyFrame();
        AutoLayout();
        RecalcCoeffs();
    }

    public void ApplyFrame()
    {
        rb.mass = massKg;
        rb.drag = 0; rb.angularDrag = 0; // свой drag
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.solverIterations = 12;
        rb.centerOfMass = Vector3.zero;
        rb.inertiaTensor = autoInertia
            ? massKg * new Vector3(motorOffset * motorOffset * 0.5f, motorOffset * motorOffset * 0.9f, motorOffset * motorOffset * 0.5f)
            : inertia;
        rb.inertiaTensorRotation = Quaternion.identity;
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
    {
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
    {
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
        Vector3 vBody = transform.InverseTransformDirection(rb.velocity - windVec);
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

        Airspeed = (rb.velocity - windVec).magnitude;
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
        if (!Crashed && c.relativeVelocity.magnitude > crashSpeed)
        {
            Crashed = true;
            OnCrash?.Invoke();
        }
    }
    void OnCollisionStay() { GroundContact = true; }

    public void ResetState()
    {
        Crashed = false;
        foreach (var m in motors) { m.rpm = 0; m.cmd = 0; m.thrustN = 0; m.dw = 0; m.vrs = false; }
        PowerW = TotalThrustN = 0;
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
```

---

### 7. `FlightController.cs` — режимы, PID, микшер, арм, failsafe, оркестрация

```csharp
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

    // ---- состояние ----
    public FlightMode Mode { get; private set; }
    public bool Armed { get; private set; }
    public bool Failsafe { get; private set; }
    public float FlightTime { get; private set; }
    public float MaxAlt { get; private set; }
    public float MaxGs { get; private set; }
    public float HoverEstimate { get; private set; } = 0.5f;
    public Vector3 HomePos { get; private set; }

    public float AttRoll => SimMath.Norm180(transform.eulerAngles.z);      // + вправо
    public float AttPitchUp => -SimMath.Norm180(transform.eulerAngles.x);  // + нос вверх
    public float Heading => transform.eulerAngles.y;
    public float AltAgl => transform.position.y - physics.GroundHeightAt(transform.position);
    public float Vs => physics.Rb.velocity.y;
    public float Gs => new Vector2(physics.Rb.velocity.x, physics.Rb.velocity.z).magnitude;
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
    }

    public void SetMode(FlightMode m)
    {
        if (m == Mode) return;
        Mode = m;
        altTarget = AltAgl;
        altVelPid.Reset(); posVelFwd.Reset(); posVelRight.Reset();
    }

    public void TryArm()
    {
        if (Armed || physics.Crashed || battery.IsEmpty) return;
        if (requireLowThrottleToArm && (input.Sticks.throttle + 1f) * 0.5f > 0.05f) return;
        if (Vector3.Angle(transform.up, Vector3.up) > 30f) return;
        Armed = true; FlightTime = 0; HomePos = transform.position;
        rateRoll.Reset(); ratePitch.Reset(); rateYaw.Reset();
        altVelPid.Reset(); posVelFwd.Reset(); posVelRight.Reset();
        altTarget = AltAgl; groundTimer = 0;
    }

    public void Disarm() { Armed = false; Failsafe = false; }

    public void ResetDrone(bool full)
    {
        Disarm();
        physics.Rb.velocity = Vector3.zero; physics.Rb.angularVelocity = Vector3.zero;
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
        float rollMeas = SimMath.Norm180(euler.z) + SimMath.Gaussian() * attitudeNoiseDeg;
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
                Vector3 vYaw = Quaternion.Inverse(yawRot) * physics.Rb.velocity;
                Vector3 gpsErr = new Vector3(SimMath.Gaussian(), 0, SimMath.Gaussian()) * gpsNoiseM;
                float aF = posVelFwd.Update(s.pitch * -posMaxSpeed, vYaw.z + gpsErr.z, dt);
                float aR = posVelRight.Update(s.roll * posMaxSpeed, vYaw.x + gpsErr.x, dt);
                float g = Mathf.Abs(Physics.gravity.y);
                tPitchDown = Mathf.Clamp(Mathf.Atan2(-aF, g) * Mathf.Rad2Deg, -posMaxTilt, posMaxTilt);
                tRoll = Mathf.Clamp(-Mathf.Atan2(aR, g) * Mathf.Rad2Deg, -posMaxTilt, posMaxTilt);
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
        rOut = rateRoll.Update(spRoll, wBody.z, dt);     // + roll right
    }

    // Микшер X: FL, FR, BR, BL; spinDirs FL+ FR- BR+ BL-
    void Mix(float t, float r, float p, float y)
    {
        if (airmode) t = Mathf.Max(t, airmodeIdle);
        cmd[0] = t - r - p + y; // FL
        cmd[1] = t + r - p - y; // FR
        cmd[2] = t + r + p + y; // BR
        cmd[3] = t - r + p - y; // BL
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
        Vector3 p = transform.position, v = physics.Rb.velocity;
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
```

---

### 8. `DronePresentation.cs` — камеры, вращение винтов, звук моторов

```csharp
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
    [Header("Chase")] public float chaseDist = 4f; public float chaseHeight = 1.6f; public float chaseSmooth = 8f;
    [Header("FPV")]
    public Vector3 fpvMount = new Vector3(0, 0.04f, 0.1f);
    [Tooltip("Базовый наклон камеры FPV, град")] [Range(0, 60)] public float fpvTiltBase = 15f;
    [Tooltip("Добавка наклона от VrA (AUX1), град")] [Range(0, 60)] public float fpvTiltAuxRange = 40f;
    [Range(60, 130)] public float fpvFov = 100f;
    [Header("Orbit")] public float orbitDist = 6f; public float orbitPitch = 25f; public float orbitYaw = 0f;

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

public class PropVisual : MonoBehaviour
{
    public QuadPhysics physics; public int motorIndex;
    public Renderer blurDisc; public Renderer blades;
    [Tooltip("Обороты, выше которых лопасти скрываются в диск")] public float blurRpm = 3500f;
    float angle; Color discColor;

    void Start() { if (blurDisc) discColor = blurDisc.material.color; }

    void Update()
    {
        if (physics == null) return;
        var m = physics.motors[motorIndex];
        angle += -m.spinDir * m.rpm / 60f * 360f * Time.deltaTime;
        transform.localRotation = Quaternion.Euler(0, angle % 360f, 0);
        float t = Mathf.Clamp01((m.rpm - blurRpm * 0.5f) / blurRpm);
        if (blades) blades.enabled = m.rpm < blurRpm;
        if (blurDisc)
        {
            blurDisc.enabled = m.rpm > blurRpm * 0.5f;
            var c = discColor; c.a = t * 0.35f; blurDisc.material.color = c;
        }
    }
}

public class MotorSound : MonoBehaviour
{
    public QuadPhysics physics;
    [Range(0, 1)] public float volume = 0.35f;
    AudioSource src;

    void Start()
    {
        src = gameObject.AddComponent<AudioSource>();
        src.loop = true; src.spatialBlend = 1f; src.minDistance = 2f; src.maxDistance = 40f;
        const int n = 44100; var data = new float[n];
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
        src.clip = clip; src.Play();
    }

    void Update()
    {
        if (physics == null || src == null) return;
        float rpm = 0; foreach (var m in physics.motors) rpm += m.rpm;
        rpm /= 4f;
        float norm = Mathf.Clamp01(rpm / physics.maxRpm);
        src.pitch = 0.4f + norm * 2.2f;
        src.volume = volume * Mathf.Clamp01(norm * 3f);
    }
}
```

---

### 9. `SimHud.cs` — HUD: авиагоризонт, телеметрия, окна калибровки/ветра/тюнинга

```csharp
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(100)]
public class SimHud : MonoBehaviour
{
    FlightController fc; QuadPhysics ph; BatteryModel bat; WindSystem wind;
    DroneInputManager inp; UdpLink udp; ChaseCamera cam;

    Texture2D white;
    readonly Dictionary<int, GUIStyle> labels = new Dictionary<int, GUIStyle>();
    GUIStyle winStyle;
    bool showHud = true;
    bool wHelp, wCal, wEnv, wInp, wTune;
    Rect rHelp = new Rect(420, 120, 640, 560);
    Rect rCal = new Rect(460, 160, 560, 460);
    Rect rEnv = new Rect(980, 140, 420, 520);
    Rect rInp = new Rect(300, 140, 620, 640);
    Rect rTune = new Rect(760, 120, 460, 680);
    Vector2 scrollInp, scrollTune, scrollHelp;
    float fps;

    static readonly Color Cbg = new Color(0.02f, 0.04f, 0.07f, 0.78f);
    static readonly Color Cacc = new Color(0.25f, 0.9f, 1f);
    static readonly Color Cok = new Color(0.3f, 1f, 0.45f);
    static readonly Color Cwarn = new Color(1f, 0.75f, 0.2f);
    static readonly Color Cerr = new Color(1f, 0.3f, 0.3f);
    static readonly Color Cdim = new Color(0.75f, 0.8f, 0.85f);
    static readonly Color Sky = new Color(0.22f, 0.5f, 0.78f);
    static readonly Color Ground = new Color(0.45f, 0.34f, 0.2f);

    void Start()
    {
        fc = FindObjectOfType<FlightController>(); ph = FindObjectOfType<QuadPhysics>();
        bat = FindObjectOfType<BatteryModel>(); wind = FindObjectOfType<WindSystem>();
        inp = FindObjectOfType<DroneInputManager>(); udp = FindObjectOfType<UdpLink>();
        cam = FindObjectOfType<ChaseCamera>();
        white = new Texture2D(1, 1); white.SetPixel(0, 0, Color.white); white.Apply();
        enabled = fc != null;
    }

    void Update()
    {
        fps = Mathf.Lerp(fps, 1f / Mathf.Max(0.0001f, Time.unscaledDeltaTime), 0.05f);
        if (Hotkeys.Down(SimKey.Help)) wHelp = !wHelp;
        if (Hotkeys.Down(SimKey.Calib)) { wCal = !wCal; if (wCal && inp != null && inp.Wizard == CalWizardState.Off) inp.BeginCal(); }
        if (Hotkeys.Down(SimKey.Env)) wEnv = !wEnv;
        if (Hotkeys.Down(SimKey.Inputs)) wInp = !wInp;
        if (Hotkeys.Down(SimKey.Tune)) wTune = !wTune;
        if (Hotkeys.Down(SimKey.HudToggle)) showHud = !showHud;
    }

    // ---------- helpers ----------
    void Fill(Rect r, Color c) { var old = GUI.color; GUI.color = c; GUI.DrawTexture(r, white); GUI.color = old; }
    GUIStyle L(int size, bool bold = false)
    {
        int key = size * 2 + (bold ? 1 : 0);
        if (!labels.TryGetValue(key, out var s))
        {
            s = new GUIStyle(GUI.skin.label) { fontSize = size, fontStyle = bold ? FontStyle.Bold : FontStyle.Normal, wordWrap = false };
            labels[key] = s;
        }
        return s;
    }
    void Txt(Rect r, string t, int size, Color c, TextAnchor a = TextAnchor.MiddleLeft, bool bold = false)
    {
        var s = L(size, bold); s.alignment = a; var old = GUI.color; GUI.color = c;
        GUI.Label(r, t, s); GUI.color = old;
    }
    void Panel(Rect r, string title)
    {
        Fill(r, Cbg);
        Fill(new Rect(r.x, r.y, r.width, 2), Cacc * 0.6f);
        if (title != null) Txt(new Rect(r.x + 8, r.y + 4, r.width - 16, 20), title, 13, Cacc, TextAnchor.UpperLeft, true);
    }
    void Bar(Rect r, float v01, Color c)
    {
        Fill(r, new Color(1, 1, 1, 0.08f));
        Fill(new Rect(r.x, r.y, r.width * Mathf.Clamp01(v01), r.height), c);
    }
    void SignedBar(Rect r, float v, Color c)
    {
        Fill(r, new Color(1, 1, 1, 0.08f));
        float mid = r.x + r.width / 2;
        float w = Mathf.Abs(v) * r.width / 2;
        Fill(new Rect(v >= 0 ? mid : mid - w, r.y, w, r.height), c);
        Fill(new Rect(mid - 1, r.y, 2, r.height), new Color(1, 1, 1, 0.3f));
    }
    static Color SrcColor(InputSourceKind k) => k switch
    { InputSourceKind.Keyboard => Cwarn, InputSourceKind.Joystick => Cok, InputSourceKind.Udp => new Color(0.7f, 0.5f, 1f), _ => Cdim };

    void OnGUI()
    {
        if (fc == null) return;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        float W = Screen.width / scale, H = 1080;

        if (showHud)
        {
            DrawTopBar(W);
            DrawADI(new Rect(60, 210, 300, 300));
            DrawTapes(new Rect(60, 210, 300, 300));
            DrawRightPanel(W);
            DrawSticks(W, H);
            DrawWarnings(W);
            Txt(new Rect(20, H - 34, 700, 24), "F1 Помощь   F2 Калибровка   F3 Ветер   F4 Ввод   F5 Тюнинг   F6 HUD   C Камера", 14, Cdim);
            if (cam != null && cam.mode == CamMode.Fpv)
            {
                Fill(new Rect(W / 2 - 14, H / 2 - 1, 28, 2), Cacc);
                Fill(new Rect(W / 2 - 1, H / 2 - 14, 2, 28), Cacc);
            }
        }

        winStyle = winStyle ?? new GUIStyle(GUI.skin.window);
        if (wHelp) rHelp = GUI.Window(11, rHelp, WinHelp, "ПОМОЩЬ / УПРАВЛЕНИЕ", winStyle);
        if (wCal) rCal = GUI.Window(12, rCal, WinCal, "КАЛИБРОВКА SM001", winStyle);
        if (wEnv) rEnv = GUI.Window(13, rEnv, WinEnv, "ВЕТЕР И СРЕДА", winStyle);
        if (wInp) rInp = GUI.Window(14, rInp, WinInput, "ВВОД / КАНАЛЫ", winStyle);
        if (wTune) rTune = GUI.Window(15, rTune, WinTune, "ТЮНИНГ PID / РАМЫ", winStyle);
    }

    // ---------- HUD ----------
    void DrawTopBar(float W)
    {
        Panel(new Rect(0, 0, W, 52), null);
        Color armC = fc.Armed ? Cok : Cdim;
        Txt(new Rect(20, 8, 260, 36), $"{fc.Mode.ToString().ToUpper()}  |  {(fc.Armed ? "ARMED" : "DISARMED")}", 22, armC, TextAnchor.MiddleLeft, true);
        string batS = $"{bat.Voltage:0.0}V  {bat.Soc * 100:0}%  {bat.Current:0.0}A";
        Txt(new Rect(300, 8, 300, 36), "BAT " + batS, 20, bat.IsCritical ? Cerr : bat.IsLow ? Cwarn : Cok);
        Txt(new Rect(620, 8, 200, 36), $"T {System.TimeSpan.FromSeconds(fc.FlightTime):mm\\:ss}", 20, Cdim);
        Txt(new Rect(780, 8, 140, 36), $"FPS {fps:0}", 16, Cdim);
        float x = 940;
        Txt(new Rect(x, 8, 60, 36), "KB", 16, inp.enableKeyboard ? Cwarn : Cdim * 0.5f); x += 50;
        Txt(new Rect(x, 8, 60, 36), "JS", 16, inp.JoyConnected ? Cok : Cdim * 0.5f); x += 50;
        Txt(new Rect(x, 8, 70, 36), "UDP", 16, udp != null && udp.Fresh ? new Color(0.7f, 0.5f, 1f) : Cdim * 0.5f); x += 80;
        if (wind != null)
            Txt(new Rect(x, 8, 380, 36), $"WIND {wind.Current.magnitude:0.0} m/s", 18, wind.Current.magnitude > 10 ? Cwarn : Cdim);
    }

    void DrawADI(Rect r)
    {
        Panel(new Rect(r.x - 8, r.y - 46, r.width + 16, r.height + 62), null);
        // компас сверху
        Rect cr = new Rect(r.x, r.y - 40, r.width, 30);
        GUI.BeginGroup(cr);
        float hdg = fc.Heading;
        for (int d = -60; d <= 60; d += 15)
        {
            int mark = ((Mathf.RoundToInt(hdg / 15f) * 15 + d) % 360 + 360) % 360;
            float x = cr.width / 2 + (Mathf.RoundToInt(hdg / 15f) * 15 + d - hdg) * 3f;
            if (x < 0 || x > cr.width) continue;
            Fill(new Rect(x - 1, 18, 2, 8), Cdim);
            string s = mark switch { 0 => "N", 90 => "E", 180 => "S", 270 => "W", _ => mark.ToString() };
            Txt(new Rect(x - 20, 0, 40, 18), s, 13, Cdim, TextAnchor.MiddleCenter);
        }
        Fill(new Rect(cr.width / 2 - 1, 24, 2, 6), Cacc);
        GUI.EndGroup();

        float roll = fc.AttRoll, pitch = fc.AttPitchUp;
        GUI.BeginGroup(r);
        Rect inner = new Rect(0, 0, r.width, r.height);
        GUI.BeginGroup(inner);
        Vector2 piv = new Vector2(r.width / 2, r.height / 2);
        var mat = GUI.matrix;
        GUIUtility.RotateAroundPivot(-roll, piv);
        float ppd = 4f, hy = piv.y + pitch * ppd;
        Fill(new Rect(-600, hy - 1600, 1500, 1600), Sky);
        Fill(new Rect(-600, hy, 1500, 1600), Ground);
        Fill(new Rect(-600, hy - 1, 1500, 2), Color.white);
        for (int p = -90; p <= 90; p += 5)
        {
            if (p == 0) continue;
            float y = hy - p * ppd;
            bool major = p % 10 == 0;
            float w = major ? 70 : 34;
            Fill(new Rect(piv.x - w / 2, y - 1, w, 2), new Color(1, 1, 1, 0.85f));
            if (major)
            {
                Txt(new Rect(piv.x - w / 2 - 34, y - 9, 30, 18), Mathf.Abs(p).ToString(), 12, Color.white, TextAnchor.MiddleRight);
                Txt(new Rect(piv.x + w / 2 + 4, y - 9, 30, 18), Mathf.Abs(p).ToString(), 12, Color.white);
            }
        }
        GUI.matrix = mat;
        // шкала крена
        for (int a = -60; a <= 60; a += 15)
        {
            var m2 = GUI.matrix;
            GUIUtility.RotateAroundPivot(-roll + a, piv);
            Fill(new Rect(piv.x - 1, 8, 2, a % 30 == 0 ? 14 : 8), Color.white);
            GUI.matrix = m2;
        }
        GUIUtility.RotateAroundPivot(-roll, piv);
        GUI.matrix = mat;
        // символ самолёта
        Fill(new Rect(piv.x - 60, piv.y - 2, 40, 4), Color.yellow);
        Fill(new Rect(piv.x + 20, piv.y - 2, 40, 4), Color.yellow);
        Fill(new Rect(piv.x - 3, piv.y - 3, 6, 6), Color.yellow);
        GUI.EndGroup();
        Txt(new Rect(4, r.height - 24, 120, 20), $"R {roll:+0;-0}°", 14, Color.white);
        Txt(new Rect(r.width - 124, r.height - 24, 120, 20), $"P {pitch:+0;-0}°", 14, Color.white, TextAnchor.MiddleRight);
        GUI.EndGroup();
        // стрелка домой
        float rel = fc.HomeBearing - fc.Heading;
        Txt(new Rect(r.x, r.y + r.height + 4, r.width, 18), $"HOME {fc.HomeDist:0} m", 13, Cacc, TextAnchor.MiddleCenter);
        var mm = GUI.matrix;
        GUIUtility.RotateAroundPivot(rel, new Vector2(r.x + r.width / 2 + 60, r.y + r.height + 13));
        Txt(new Rect(r.x + r.width / 2 + 52, r.y + r.height + 4, 16, 18), "^", 14, Cacc, TextAnchor.MiddleCenter);
        GUI.matrix = mm;
    }

    void DrawTapes(Rect adi)
    {
        // скорость слева
        Rect sp = new Rect(adi.x - 58, adi.y, 50, adi.height);
        Panel(sp, null);
        Txt(new Rect(sp.x, sp.y + 4, sp.width, 18), "GS", 12, Cdim, TextAnchor.MiddleCenter);
        GUI.BeginGroup(sp);
        float gs = fc.Gs;
        for (int v = Mathf.FloorToInt(gs) - 8; v <= gs + 8; v++)
        {
            if (v < 0 || v % 2 != 0) continue;
            float y = sp.height / 2 - (v - gs) * 14f;
            if (y < 24 || y > sp.height - 8) continue;
            Fill(new Rect(sp.width - 10, y, 8, 1), Cdim);
            Txt(new Rect(0, y - 8, sp.width - 12, 16), v.ToString(), 11, Cdim, TextAnchor.MiddleRight);
        }
        GUI.EndGroup();
        Fill(new Rect(sp.x, sp.y + sp.height / 2 - 12, sp.width, 24), Cbg);
        Txt(new Rect(sp.x, sp.y + sp.height / 2 - 12, sp.width, 24), gs.ToString("0.0"), 15, Cacc, TextAnchor.MiddleCenter, true);

        // высота справа
        Rect al = new Rect(adi.xMax + 8, adi.y, 58, adi.height);
        Panel(al, null);
        Txt(new Rect(al.x, al.y + 4, al.width, 18), "ALT", 12, Cdim, TextAnchor.MiddleCenter);
        GUI.BeginGroup(al);
        float alt = fc.AltAgl;
        for (int v = Mathf.FloorToInt(alt) - 12; v <= alt + 12; v++)
        {
            if (v < 0 || v % 5 != 0) continue;
            float y = al.height / 2 - (v - alt) * 6f;
            if (y < 24 || y > al.height - 8) continue;
            Fill(new Rect(2, y, 8, 1), Cdim);
            Txt(new Rect(12, y - 8, al.width - 12, 16), v.ToString(), 11, Cdim);
        }
        GUI.EndGroup();
        Fill(new Rect(al.x, al.y + al.height / 2 - 12, al.width, 24), Cbg);
        Txt(new Rect(al.x, al.y + al.height / 2 - 12, al.width, 24), alt.ToString("0.0"), 15, Cacc, TextAnchor.MiddleCenter, true);
        // VS
        float vs = fc.Vs;
        Txt(new Rect(al.x, al.yMax + 4, al.width + 40, 20), $"VS {vs:+0.0;-0.0}", 13, Mathf.Abs(vs) > 4 ? Cwarn : Cdim);
        Txt(new Rect(sp.x - 10, sp.yMax + 4, 100, 20), $"AS {ph.Airspeed:0.0}", 13, Cdim);
    }

    void DrawRightPanel(float W)
    {
        Rect p = new Rect(W - 320, 90, 300, 470);
        Panel(p, "СИСТЕМЫ");
        float y = p.y + 28;
        // моторы X
        string[] names = { "FL", "FR", "BR", "BL" };
        Vector2[] pos = { new Vector2(40, 40), new Vector2(180, 40), new Vector2(180, 130), new Vector2(40, 130) };
        for (int i = 0; i < 4; i++)
        {
            var m = ph.motors[i];
            Rect mr = new Rect(p.x + pos[i].x, y + pos[i].y, 84, 44);
            Fill(mr, new Color(1, 1, 1, 0.06f));
            Txt(new Rect(mr.x, mr.y, 84, 16), $"{names[i]} {(m.spinDir > 0 ? "CW" : "CCW")}", 11, Cdim, TextAnchor.MiddleCenter);
            Txt(new Rect(mr.x, mr.y + 14, 84, 18), $"{m.rpm:0}", 14, m.vrs ? Cwarn : Cacc, TextAnchor.MiddleCenter, true);
            Bar(new Rect(mr.x + 4, mr.y + 34, 76, 6), m.cmd, Cok);
        }
        Txt(new Rect(p.x + 10, y + 180, 280, 20), $"Тяга {ph.TotalThrustN / 0.00981f:0} г   GE x{ph.GroundEffectFactor:0.00}", 14, Cdim);
        y += 210;
        Txt(new Rect(p.x + 10, y, 280, 20), $"Батарея {bat.Voltage:0.00} V ({bat.CellVoltage:0.02}/cell)", 14, bat.IsCritical ? Cerr : bat.IsLow ? Cwarn : Cok); y += 24;
        Bar(new Rect(p.x + 10, y, 280, 10), bat.Soc, bat.IsLow ? Cwarn : Cok); y += 18;
        Txt(new Rect(p.x + 10, y, 280, 20), $"Израсходовано {bat.UsedMah:0} mAh   {bat.Current:0.0} A", 13, Cdim); y += 26;
        if (wind != null)
        {
            Vector3 wv = wind.Current;
            float wd = (Mathf.Atan2(wv.x, wv.z) * Mathf.Rad2Deg + 360f) % 360f;
            Txt(new Rect(p.x + 10, y, 280, 20), $"Ветер {wv.magnitude:0.0} m/s  {wd:0}°  верт {wv.y:+0.0;-0.0}", 14, wv.magnitude > 10 ? Cwarn : Cdim); y += 24;
        }
        Txt(new Rect(p.x + 10, y, 280, 20), $"Макс: ALT {fc.MaxAlt:0.0} m   GS {fc.MaxGs:0.0} m/s", 13, Cdim); y += 24;
        Txt(new Rect(p.x + 10, y, 280, 20), $"FC {fc.loopHz} Hz   Hover {(fc.HoverEstimate * 100):0}%", 13, Cdim); y += 24;
        if (udp != null)
            Txt(new Rect(p.x + 10, y, 280, 20), $"UDP rx:{udp.RxCount} tx:{udp.TxCount} :{udp.listenPort}", 12, Cdim);
    }

    void DrawSticks(float W, float H)
    {
        float bx = W / 2 - 320, by = H - 190;
        Panel(new Rect(bx - 10, by - 26, 660, 180), "КАНАЛЫ");
        // стики
        DrawStickBox(new Rect(bx, by, 130, 130), inp.Mixed[3], -inp.Mixed[2], "YAW / THR");
        DrawStickBox(new Rect(bx + 150, by, 130, 130), inp.Sticks.roll, -inp.Sticks.pitch, "ROLL / PITCH");
        // 6 каналов
        for (int i = 0; i < 6; i++)
        {
            Rect r = new Rect(bx + 300, by + i * 22, 330, 18);
            Txt(new Rect(r.x, r.y, 90, 18), DroneInputManager.ChannelNames[i], 11, Cdim);
            SignedBar(new Rect(r.x + 92, r.y + 4, 160, 10), inp.Mixed[i], SrcColor(inp.ActiveSrc[i]));
            Txt(new Rect(r.x + 258, r.y, 70, 18), inp.Mixed[i].ToString("0.00"), 11, SrcColor(inp.ActiveSrc[i]));
        }
    }
    void DrawStickBox(Rect r, float x, float y, string title)
    {
        Fill(r, new Color(1, 1, 1, 0.05f));
        Fill(new Rect(r.x + r.width / 2 - 1, r.y, 2, r.height), new Color(1, 1, 1, 0.15f));
        Fill(new Rect(r.x, r.y + r.height / 2 - 1, r.width, 2), new Color(1, 1, 1, 0.15f));
        float px = r.x + r.width / 2 + x * (r.width / 2 - 8);
        float py = r.y + r.height / 2 + y * (r.height / 2 - 8);
        Fill(new Rect(px - 5, py - 5, 10, 10), Cacc);
        Txt(new Rect(r.x, r.yMax + 2, r.width, 16), title, 10, Cdim, TextAnchor.MiddleCenter);
    }

    void DrawWarnings(float W)
    {
        float y = 90; bool blink = Time.realtimeSinceStartup % 0.6f < 0.35f;
        void Warn(string t, Color c, bool force = false)
        {
            if (!force && !blink) return;
            Txt(new Rect(W / 2 - 300, y, 600, 34), t, 26, c, TextAnchor.MiddleCenter, true); y += 38;
        }
        if (ph.Crashed) Warn("CRASHED — R для сброса", Cerr, true);
        if (fc.Failsafe) Warn("FAILSAFE — " + fc.failsafeAction, Cerr);
        if (bat.IsEmpty) Warn("BATTERY DEAD", Cerr, true);
        else if (bat.IsCritical) Warn("BATTERY CRITICAL", Cerr);
        else if (bat.IsLow) Warn("LOW BATTERY", Cwarn);
        if (ph.VrsActive) Warn("VORTEX RING STATE", Cwarn);
        if (!fc.Armed && !ph.Crashed)
            Txt(new Rect(W / 2 - 300, 1000 - 230, 600, 26), "DISARMED — Space для арма (газ в нуле)", 17, Cdim, TextAnchor.MiddleCenter);
    }

    // ---------- окна ----------
    float Slider(string name, float v, float min, float max)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(name, GUILayout.Width(170));
        v = GUILayout.Slider(v, min, max);
        GUILayout.Label(v.ToString("0.###"), GUILayout.Width(60));
        GUILayout.EndHorizontal();
        return v;
    }
    void PidRows(string name, PidController p, float pMax, float iMax, float dMax)
    {
        GUILayout.Label(name, GUI.skin.box);
        p.kp = Slider("P", p.kp, 0, pMax);
        p.ki = Slider("I", p.ki, 0, iMax);
        p.kd = Slider("D", p.kd, 0, dMax);
    }

    void WinHelp(int id)
    {
        scrollHelp = GUILayout.BeginScrollView(scrollHelp);
        GUILayout.Label(
@"КЛАВИАТУРА
W/S — газ (фиксация), A/D — рыскание
Стрелки — крен/тангаж, Shift — точный режим
Space — ARM, Backspace — DISARM
R — сброс позиции, Shift+R — сброс + батарея
M — режим (Acro/Angle/AltHold/PosHold), C — камера

ДЖОЙСТИК SM001
F2 — мастер калибровки, F4 — монитор каналов и маппинг.
SwA (AUX2): верх — Angle, низ — Acro. VrA (AUX1) — наклон FPV-камеры.
Режим микса MaxAbs: клавиатура и джойстик работают ОДНОВРЕМЕННО.

UDP (порт 14551 приём / 14550 телеметрия)
JSON: {""channels"":[r,p,t,y,a1,a2],""arm"":1,""mode"":2,""reset"":false}
каналы -1..1 (t: -1=0%). arm: 1=arm 2=disarm. mode: 1..4.
Или CSV PWM: 1500,1500,1000,1500,1000,1000

РЕЖИМЫ
Acro — угловые скорости, Angle — самовыравнивание,
AltHold — высота, PosHold — удержание точки (скоростной).");
        GUILayout.EndScrollView();
        if (GUILayout.Button("Закрыть")) wHelp = false;
        GUI.DragWindow();
    }

    void WinCal(int id)
    {
        if (inp == null) return;
        GUILayout.Label($"Устройство: {(inp.JoyConnected ? inp.DeviceNames[Mathf.Min(inp.deviceIndex, inp.DeviceNames.Count - 1)] : "НЕ ПОДКЛЮЧЕН")}");
        if (GUILayout.Button("Сменить устройство")) inp.CycleDevice();
        GUILayout.Space(6);
        switch (inp.Wizard)
        {
            case CalWizardState.Off:
                GUILayout.Label("Мастер калибровки готов.");
                if (GUILayout.Button("НАЧАТЬ КАЛИБРОВКУ")) inp.BeginCal();
                if (GUILayout.Button("Сбросить к значениям по умолчанию")) inp.ResetCal();
                break;
            case CalWizardState.Intro:
                GUILayout.Label("Шаг 1/4. Подключите SM001 по USB.\nДалее потребуется поставить стики в центр,\nзатем прокрутить все оси по полным диапазонам\n(включая VrA и переключатель SwA).");
                if (GUILayout.Button("ДАЛЕЕ →")) inp.CalNext();
                break;
            case CalWizardState.Centers:
                GUILayout.Label("Шаг 2/4. Стики — в ЦЕНТР, газ — ВНИЗ, VrA — в центр.\nНажмите «Зафиксировать центр».");
                DrawRawChannels();
                if (GUILayout.Button("ЗАФИКСИРОВАТЬ ЦЕНТР →")) inp.CalNext();
                break;
            case CalWizardState.Extremes:
                GUILayout.Label("Шаг 3/4. Медленно прокрутите ОБА стика по кругу,\nгаз снизу-вверх, VrA по краям, SwA во все положения.\nСмотрите, чтобы диапазоны ниже заполнились.");
                DrawCalRanges();
                if (GUILayout.Button("ГОТОВО →")) inp.CalNext();
                break;
            case CalWizardState.Verify:
                GUILayout.Label("Шаг 4/4. Проверьте: стики в центре ≈ 0,\nкрая ≈ ±1, газ 0% = -1, 100% = +1.");
                for (int i = 0; i < 6; i++)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(DroneInputManager.ChannelNames[i], GUILayout.Width(90));
                    GUILayout.Label(inp.JoyCal[i].ToString("0.00"), GUILayout.Width(60));
                    inp.cal.invert[i] = GUILayout.Toggle(inp.cal.invert[i], "инверсия", GUILayout.Width(90));
                    GUILayout.EndHorizontal();
                }
                if (GUILayout.Button("СОХРАНИТЬ")) inp.CalNext();
                break;
        }
        if (inp.Wizard != CalWizardState.Off && GUILayout.Button("Отмена")) inp.CalCancel();
        GUI.DragWindow();
    }
    void DrawRawChannels()
    {
        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(DroneInputManager.ChannelNames[i], GUILayout.Width(90));
            GUILayout.Label(inp.RawChannel(i).ToString("0.000"), GUILayout.Width(80));
            GUILayout.EndHorizontal();
        }
    }
    void DrawCalRanges()
    {
        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(DroneInputManager.ChannelNames[i], GUILayout.Width(90));
            GUILayout.Label($"min {inp.cal.min[i]:0.00}  max {inp.cal.max[i]:0.00}", GUILayout.Width(180));
            float span = Mathf.Clamp01((inp.cal.max[i] - inp.cal.min[i]) / 1.5f);
            GUILayout.Box("", GUILayout.Width(150 * span + 1), GUILayout.Height(12));
            GUILayout.EndHorizontal();
        }
    }

    void WinEnv(int id)
    {
        if (wind == null) return;
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Штиль")) wind.PresetCalm();
        if (GUILayout.Button("Бриз")) wind.PresetBreeze();
        if (GUILayout.Button("Сильный")) wind.PresetStrong();
        if (GUILayout.Button("Шторм")) wind.PresetStorm();
        GUILayout.EndHorizontal();
        wind.baseSpeed = Slider("Скорость, m/s", wind.baseSpeed, 0, 25);
        wind.directionDeg = Slider("Направление, °", wind.directionDeg, 0, 360);
        wind.dirWander = Slider("Блуждание напр., °", wind.dirWander, 0, 45);
        wind.gustAmp = Slider("Порывы, m/s", wind.gustAmp, 0, 15);
        wind.gustFreq = Slider("Частота порывов", wind.gustFreq, 0.02f, 2);
        wind.turbAmp = Slider("Турбулентность", wind.turbAmp, 0, 8);
        wind.turbScale = Slider("Масштаб турб., m", wind.turbScale, 1, 100);
        wind.verticalAmp = Slider("Верт. порывы", wind.verticalAmp, 0, 5);
        wind.shearExponent = Slider("Сдвиг по высоте", wind.shearExponent, 0, 0.6f);
        GUILayout.Space(8);
        wind.airDensity = Slider("Плотность воздуха", wind.airDensity, 0.6f, 1.4f);
        wind.gravity = Slider("Гравитация", wind.gravity, 1, 20);
        Time.timeScale = Slider("Time scale", Time.timeScale, 0.1f, 2);
        var snd = FindObjectOfType<MotorSound>();
        if (snd) snd.volume = Slider("Громкость моторов", snd.volume, 0, 1);
        GUI.DragWindow();
    }

    void WinInput(int id)
    {
        if (inp == null) return;
        scrollInp = GUILayout.BeginScrollView(scrollInp);
        GUILayout.Label("Микс источников (одновременная работа):");
        GUILayout.BeginHorizontal();
        foreach (InputMixMode m in System.Enum.GetValues(typeof(InputMixMode)))
        {
            GUI.color = inp.mixMode == m ? Cacc : Color.white;
            if (GUILayout.Button(m.ToString())) inp.mixMode = m;
        }
        GUI.color = Color.white;
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        inp.enableKeyboard = GUILayout.Toggle(inp.enableKeyboard, "Клавиатура");
        inp.enableJoystick = GUILayout.Toggle(inp.enableJoystick, "Джойстик");
        inp.enableUdp = GUILayout.Toggle(inp.enableUdp, "UDP");
        GUILayout.EndHorizontal();
        GUILayout.Space(6);
        GUILayout.Label($"Джойстик: {(inp.JoyConnected ? "подключен" : "нет")}  [{inp.deviceIndex}]");
        if (GUILayout.Button("Сменить устройство")) inp.CycleDevice();
        GUILayout.Label("Сырые оси HID (индекс : значение):");
        for (int i = 0; i < inp.AxisLabels.Count && i < 14; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"[{i}] {inp.AxisLabels[i]}", GUILayout.Width(220));
            GUILayout.Label(inp.AxisValues[i].ToString("0.000"), GUILayout.Width(70));
            GUILayout.Box("", GUILayout.Width(Mathf.Abs(inp.AxisValues[i]) * 150 + 1), GUILayout.Height(10));
            GUILayout.EndHorizontal();
        }
        GUILayout.Label($"Кнопки: {inp.ButtonValues.Count}");
        GUILayout.Space(6);
        GUILayout.Label("Маппинг каналов (ось / кнопка / инверсия):");
        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(DroneInputManager.ChannelNames[i], GUILayout.Width(90));
            if (GUILayout.Button("-", GUILayout.Width(24))) inp.cal.sourceIndex[i] = Mathf.Max(0, inp.cal.sourceIndex[i] - 1);
            GUILayout.Label("ось " + inp.cal.sourceIndex[i], GUILayout.Width(50));
            if (GUILayout.Button("+", GUILayout.Width(24))) inp.cal.sourceIndex[i]++;
            if (GUILayout.Button("-", GUILayout.Width(24))) inp.cal.buttonIndex[i] = Mathf.Max(-1, inp.cal.buttonIndex[i] - 1);
            GUILayout.Label("кн " + inp.cal.buttonIndex[i], GUILayout.Width(46));
            if (GUILayout.Button("+", GUILayout.Width(24))) inp.cal.buttonIndex[i]++;
            inp.cal.invert[i] = GUILayout.Toggle(inp.cal.invert[i], "inv", GUILayout.Width(44));
            GUILayout.Label(inp.JoyCal[i].ToString("0.00"), GUILayout.Width(50));
            GUILayout.EndHorizontal();
        }
        if (GUILayout.Button("Сохранить маппинг")) inp.SaveCal();
        GUILayout.Space(6);
        inp.deadzone = Slider("Deadzone", inp.deadzone, 0, 0.2f);
        inp.expoRollPitch = Slider("Expo R/P", inp.expoRollPitch, 0, 1);
        inp.expoYaw = Slider("Expo Yaw", inp.expoYaw, 0, 1);
        inp.throttleExpo = Slider("Expo Thr", inp.throttleExpo, 0, 1);
        if (udp != null)
        {
            GUILayout.Space(6);
            GUILayout.Label($"UDP :{udp.listenPort} → {udp.remoteHost}:{udp.remotePort}  fresh={udp.Fresh}");
            GUILayout.Label($"rx {udp.RxCount}  tx {udp.TxCount}  err: {udp.LastError}");
        }
        GUILayout.EndScrollView();
        GUI.DragWindow();
    }

    void WinTune(int id)
    {
        if (fc == null) return;
        scrollTune = GUILayout.BeginScrollView(scrollTune);
        PidRows("RATE ROLL", fc.rateRoll, 0.006f, 0.02f, 0.0003f);
        PidRows("RATE PITCH", fc.ratePitch, 0.006f, 0.02f, 0.0003f);
        PidRows("RATE YAW", fc.rateYaw, 0.012f, 0.03f, 0.0002f);
        fc.angleP = Slider("Angle P", fc.angleP, 1, 15);
        fc.angleLimit = Slider("Angle limit, °", fc.angleLimit, 10, 60);
        inp.rcRate = Slider("RC rate", inp.rcRate, 0.2f, 3);
        inp.superRate = Slider("Super rate", inp.superRate, 0, 0.95f);
        GUILayout.Space(6);
        ph.maxThrustGrams = Slider("Тяга/мотор, г", ph.maxThrustGrams, 200, 4000);
        ph.maxRpm = Slider("Max RPM", ph.maxRpm, 5000, 50000);
        ph.spoolUpTime = Slider("Spool up, с", ph.spoolUpTime, 0.005f, 0.3f);
        ph.spoolDownTime = Slider("Spool down, с", ph.spoolDownTime, 0.005f, 0.4f);
        ph.massKg = Slider("Масса, кг", ph.massKg, 0.2f, 5);
        if (GUILayout.Button("Применить массу/раму")) ph.ApplyFrame();
        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("ARM")) fc.TryArm();
        if (GUILayout.Button("DISARM")) fc.Disarm();
        if (GUILayout.Button("RESET")) fc.ResetDrone(true);
        GUILayout.EndHorizontal();
        GUILayout.EndScrollView();
        GUI.DragWindow();
    }
}
```

---

### 10. `SimBootstrap.cs` — авто-сборка сцены (повесь на пустой GameObject)

```csharp
using UnityEngine;

[DefaultExecutionOrder(-100)]
public class SimBootstrap : MonoBehaviour
{
    [Header("Сборка демо-сцены")]
    [Tooltip("Собрать сцену, только если в ней нет дрона")] public bool buildIfEmpty = true;
    [Tooltip("Частота физики, Гц (рекомендуется 200)")] [Range(50, 400)] public int physicsHz = 200;
    [Range(30, 240)] public int targetFps = 144;
    public Vector3 spawnPos = new Vector3(0, 0.12f, 0);

    void Awake()
    {
        Time.fixedDeltaTime = 1f / physicsHz;
        Time.maximumDeltaTime = 0.1f;
        Application.targetFrameRate = targetFps;
        var existing = FindObjectOfType<QuadPhysics>();
        if (existing != null) { EnsureCamera(existing.transform); EnsureHud(); return; }
        if (!buildIfEmpty) return;
        BuildEnvironment();
        var drone = BuildDrone();
        EnsureCamera(drone.transform);
        EnsureHud();
    }

    static Material Mat(Color c)
    {
        Shader sh = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
        var m = new Material(sh); m.color = c; return m;
    }
    static GameObject Prim(PrimitiveType t, string name, Transform parent, Vector3 lp, Vector3 scale, Color c, bool collider = true)
    {
        var g = GameObject.CreatePrimitive(t);
        g.name = name; g.transform.SetParent(parent, false);
        g.transform.localPosition = lp; g.transform.localScale = scale;
        g.GetComponent<Renderer>().material = Mat(c);
        if (!collider) Object.Destroy(g.GetComponent<Collider>());
        return g;
    }

    void BuildEnvironment()
    {
        var env = new GameObject("Environment");
        env.AddComponent<WindSystem>();
        var ground = Prim(PrimitiveType.Cube, "Ground", env.transform, new Vector3(0, -0.5f, 0), new Vector3(400, 1, 400), new Color(0.22f, 0.3f, 0.22f));
        ground.isStatic = true;
        Prim(PrimitiveType.Cube, "HomePad", env.transform, new Vector3(0, 0.01f, 0), new Vector3(2, 0.02f, 2), new Color(0.9f, 0.5f, 0.1f));
        // ворота для FPV
        for (int i = 0; i < 2; i++)
        {
            var gate = new GameObject("Gate" + i); gate.transform.SetParent(env.transform);
            gate.transform.position = new Vector3(i * 10, 0, 14 + i * 12);
            gate.transform.rotation = Quaternion.Euler(0, i * 30f, 0);
            Color gc = i == 0 ? new Color(0.2f, 0.8f, 1f) : new Color(1f, 0.3f, 0.6f);
            Prim(PrimitiveType.Cube, "L", gate.transform, new Vector3(-1.5f, 1.25f, 0), new Vector3(0.15f, 2.5f, 0.15f), gc);
            Prim(PrimitiveType.Cube, "R", gate.transform, new Vector3(1.5f, 1.25f, 0), new Vector3(0.15f, 2.5f, 0.15f), gc);
            Prim(PrimitiveType.Cube, "T", gate.transform, new Vector3(0, 2.5f, 0), new Vector3(3.15f, 0.15f, 0.15f), gc);
        }
        var rnd = new System.Random(42);
        for (int i = 0; i < 6; i++)
        {
            float h = 1f + (float)rnd.NextDouble() * 4f;
            Prim(PrimitiveType.Cube, "Box" + i, env.transform,
                new Vector3((float)rnd.NextDouble() * 60 - 30, h / 2, 20 + (float)rnd.NextDouble() * 50),
                new Vector3(2, h, 2), new Color(0.4f, 0.42f, 0.45f));
        }
        if (FindObjectOfType<Light>() == null)
        {
            var lg = new GameObject("Sun");
            var l = lg.AddComponent<Light>(); l.type = LightType.Directional; l.intensity = 1.2f;
            lg.transform.rotation = Quaternion.Euler(50, -30, 0);
            RenderSettings.ambientLight = new Color(0.45f, 0.48f, 0.52f);
        }
    }

    GameObject BuildDrone()
    {
        var d = new GameObject("Quadcopter");
        d.transform.position = spawnPos;
        Color body = new Color(0.12f, 0.13f, 0.15f), arm = new Color(0.2f, 0.21f, 0.24f);
        Prim(PrimitiveType.Cube, "Body", d.transform, Vector3.zero, new Vector3(0.14f, 0.05f, 0.22f), body);

        var input = d.AddComponent<DroneInputManager>();
        d.AddComponent<UdpLink>();
        d.AddComponent<BatteryModel>();
        var ph = d.AddComponent<QuadPhysics>();
        ph.AutoLayout();
        d.AddComponent<FlightController>();
        d.AddComponent<MotorSound>().physics = ph;

        for (int i = 0; i < 4; i++)
        {
            Vector3 mp = ph.motors[i].localPosition;
            // луч
            var beam = Prim(PrimitiveType.Cube, "Arm" + i, d.transform, mp * 0.5f, new Vector3(0.03f, 0.02f, mp.magnitude), arm);
            beam.transform.localRotation = Quaternion.LookRotation(mp.normalized);
            // мотор
            Prim(PrimitiveType.Cylinder, "Motor" + i, d.transform, mp + Vector3.up * 0.02f, new Vector3(0.05f, 0.02f, 0.05f), body);
            var pod = d.transform.Find("Motor" + i);
            Object.Destroy(pod.GetComponent<CapsuleCollider>());
            pod.gameObject.AddComponent<SphereCollider>().radius = 0.6f;
            // винт
            var prop = new GameObject("Prop" + i);
            prop.transform.SetParent(d.transform, false);
            prop.transform.localPosition = mp + Vector3.up * 0.045f;
            var b1 = Prim(PrimitiveType.Cube, "Blade1", prop.transform, Vector3.zero, new Vector3(0.125f, 0.003f, 0.012f), new Color(0.85f, 0.2f, 0.2f), false);
            var b2 = Prim(PrimitiveType.Cube, "Blade2", prop.transform, Vector3.zero, new Vector3(0.012f, 0.003f, 0.125f), new Color(0.85f, 0.2f, 0.2f), false);
            var disc = Prim(PrimitiveType.Cylinder, "Disc", prop.transform, Vector3.zero, new Vector3(0.13f, 0.001f, 0.13f), new Color(0.6f, 0.6f, 0.6f, 0.3f), false);
            var pv = prop.AddComponent<PropVisual>();
            pv.physics = ph; pv.motorIndex = i;
            pv.blades = b1.GetComponent<Renderer>();
            pv.blurDisc = disc.GetComponent<Renderer>();
            // вторая лопасть скрывается вместе с первой через тот же renderer-трюк: делаем её дочерней к Blade1
            b2.transform.SetParent(b1.transform, true);
        }
        return d;
    }

    void EnsureCamera(Transform drone)
    {
        var c = Camera.main;
        GameObject cg;
        if (c != null) cg = c.gameObject;
        else
        {
            cg = new GameObject("SimCamera"); cg.tag = "MainCamera";
            cg.AddComponent<Camera>(); cg.AddComponent<AudioListener>();
        }
        var cc = cg.GetComponent<ChaseCamera>() ?? cg.AddComponent<ChaseCamera>();
        cc.target = drone;
        cc.input = FindObjectOfType<DroneInputManager>();
    }
    void EnsureHud()
    {
        if (FindObjectOfType<SimHud>() == null) new GameObject("HUD").AddComponent<SimHud>();
    }
}
```

---

## Как пользоваться

**Полёт:** `Space` — арм (газ в нуле), `W` — газ ~50% и дрон висит в Angle. `M` — режимы, `C` — Chase → FPV → Orbit, `R` — сброс. Клавиатура и SM001 работают **одновременно** (микс MaxAbs: побеждает большее отклонение по каждому каналу; газ — кто выше).

**Калибровка SM001 (F2):** центр стиков → фиксация → полные круги стиками + газ + VrA + SwA → проверка → сохранение (JSON в `persistentDataPath`). Если оси HID идут в другом порядке — поправь маппинг в **F4**, там видны все сырые оси с индексами. SwA назначь на «кн 0/1» канала AUX2 — получишь переключатель Acro/Angle как на настоящей аппаратуре.

**UDP-тест из внешнего приложения** (Python):

```python
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ch = [0.0, 0.0, -0.2, 0.0, 0.0, 1.0]   # газ 40%, SwA -> Angle
s.sendto(json.dumps({"channels": ch, "arm": 1, "mode": 2}).encode(), ("127.0.0.1", 14551))
# телеметрия приходит на порт 14550 — bind и recvfrom там
```

**Тюнинг (F5):** если дрон «дрыгается» — уменьши P/D угловых скоростей или увеличь `spoolUpTime`; если вялый — подними P. Ветер крути в **F3** (пресеты Штиль→Шторм). Все ключевые параметры продублированы в инспекторе с русскими подсказками.

Что можно добавить следующим шагом: логирование полёта в CSV/blackbox, повреждение отдельных моторов при ударе винтом, рейсы по воротам с таймером круга, запись/воспроизведение полёта. Скажи, что из этого нужно — расширю соответствующий модуль.

### USER
все сделал но возникли ошибки помоги исправить:   1 . Assets\DroneInputManager.cs(69,10): error CS0246: The type or namespace name 'AxisControl' could not be found (are you missing a using directive or an assembly reference?)

2.  Assets\DroneInputManager.cs(70,10): error CS0246: The type or namespace name 'ButtonControl' could not be found (are you missing a using directive or an assembly reference?)



### ASSISTANT
Причина простая: классы контролов (`AxisControl`, `ButtonControl`, `StickControl`) живут не в `UnityEngine.InputSystem`, а во вложенном пространстве имён **`UnityEngine.InputSystem.Controls`**. Поэтому `Joystick` компилируется, а эти два типа — нет.

## Исправление

Открой `DroneInputManager.cs` и замени шапку файла (первые строки) на эту — добавлена одна строка `using`:

```csharp
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;   // <-- ДОБАВИТЬ ЭТУ СТРОКУ
#endif
```

Больше в этом файле ничего менять не нужно — этот `using` закроет обе ошибки и все последующие упоминания `AxisControl`, `ButtonControl` и `StickControl` (в `RefreshDevices()` они тоже используются, компилятор просто показал первые ошибки).

Остальные файлы править не надо: в `SimCore.cs` и `DronePresentation.cs` типы контролов явно не называются (там только `Keyboard.current`, `Mouse.current` и обращения вида `kb.wKey.isPressed`), им хватает `using UnityEngine.InputSystem;`.

Сохрани файл, дождись рекомпиляции в Unity — ошибки CS0246 уйдут, и можно жать Play. Если после этого появятся другие ошибки — кидай список, разберём.

### USER
исправил ошибки но возникла новая:  Assets\SimHud.cs(339,23): error CS0117: 'GUILayout' does not contain a definition for 'Slider'



### ASSISTANT
Это моя опечатка в API: в IMGUI нет метода `GUILayout.Slider` — горизонтальный слайдер называется **`GUILayout.HorizontalSlider`**. Все окна (F3/F4/F5) используют один helper `Slider()`, поэтому правка в одном месте чинит всё сразу.

## Исправление

В `SimHud.cs` найди метод `Slider` (строка ~339) и замени его целиком на:

```csharp
float Slider(string name, float v, float min, float max)
{
    GUILayout.BeginHorizontal();
    GUILayout.Label(name, GUILayout.Width(170));
    v = GUILayout.HorizontalSlider(v, min, max);   // было GUILayout.Slider — такого метода не существует
    GUILayout.Label(v.ToString("0.###"), GUILayout.Width(60));
    GUILayout.EndHorizontal();
    return v;
}
```

Фактически меняется одна строка: `GUILayout.Slider(...)` → `GUILayout.HorizontalSlider(...)`.

Я перепроверил остальные файлы на подобные несовпадения API (`GUILayout.Box/Label/Button/Toggle/BeginScrollView`, `GUI.Window`, `GUIUtility.RotateAroundPivot`, `Physics.RaycastNonAlloc`, `AudioClip.Create` и т.д.) — там всё корректно, так что после этой правки проект должен скомпилироваться. Сохрани файл, дождись рекомпиляции и жми Play. Если Unity выдаст ещё что-то — присылай текст ошибки, сразу поправлю.

### USER
Ура все работает спасибо!  а можно доработать код так чтобы сцену можно было создать в редакторе и она бы оставалась статичной а не генерировалась при запуске?

### ASSISTANT
Да, это правильнее: сцену нужно строить **в редакторе** (объекты, материалы-ассеты, сохранённые ссылки), а при Play только применять настройки времени и ничего не создавать. Ниже — рефакторинг: общая функция постройки сцены вынесена в отдельный класс, который вызывают и редактор, и (опционально) рантайм.

## Что получится

- Меню **Tools ▸ Drone Sim ▸ Build Demo Scene** (и кнопки прямо в инспекторе `SimBootstrap`) — сцена строится в edit-mode, сохраняется, объекты можно двигать/удалять/заменять своими моделями.
- Материалы создаются как ассеты в `Assets/DroneSim/Materials` (не «розовые» после перезагрузки сцены).
- Окружение помечается `Static`, дрон — нет.
- Все ссылки (`FlightController`, `PropVisual`, камера, HUD) прописываются в сериализованные поля — видны в инспекторе.
- `SimBootstrap` при Play больше **ничего не генерирует** (флаг `autoBuildAtRuntime` выключен по умолчанию).

---

## 1. Новый файл `Assets/Scripts/DroneSceneBuilder.cs`

```csharp
using System;
using System.Collections.Generic;
using UnityEngine;

// Контекст постройки: в редакторе материалы становятся ассетами + Undo, в рантайме — обычные new Material.
public class DroneBuildContext
{
    public Vector3 spawn = new Vector3(0f, 0.12f, 0f);
    public bool markEnvironmentStatic = true;
    public Func<string, Color, Material> materialFactory;
    public Action<GameObject> register;

    readonly Dictionary<string, Material> cache = new Dictionary<string, Material>();

    public Material Get(string name, Color c)
    {
        if (cache.TryGetValue(name, out var m) && m != null) return m;
        m = materialFactory != null ? materialFactory(name, c) : DefaultMaterial(c);
        cache[name] = m;
        return m;
    }

    public static Material DefaultMaterial(Color c)
    {
        Shader sh = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
        var m = new Material(sh); m.color = c; return m;
    }

    public static DroneBuildContext Runtime(Vector3 spawn)
        => new DroneBuildContext { spawn = spawn, markEnvironmentStatic = false };
}

public static class DroneSceneBuilder
{
    public static readonly Dictionary<string, Color> Palette = new Dictionary<string, Color>
    {
        { "Ground",   new Color(0.22f, 0.30f, 0.22f) },
        { "Pad",      new Color(0.90f, 0.50f, 0.10f) },
        { "GateA",    new Color(0.20f, 0.80f, 1.00f) },
        { "GateB",    new Color(1.00f, 0.30f, 0.60f) },
        { "Obstacle", new Color(0.40f, 0.42f, 0.45f) },
        { "Body",     new Color(0.12f, 0.13f, 0.15f) },
        { "Arm",      new Color(0.20f, 0.21f, 0.24f) },
        { "Motor",    new Color(0.30f, 0.31f, 0.34f) },
        { "Blade",    new Color(0.85f, 0.20f, 0.20f) },
        { "Disc",     new Color(0.60f, 0.60f, 0.60f, 0.30f) },
    };

    public static GameObject BuildAll(DroneBuildContext ctx)
    {
        BuildEnvironment(ctx);
        var drone = BuildDrone(ctx);
        BuildCameraAndHud(ctx, drone.transform);
        EnsureBootstrap(drone);
        WireReferences();
        return drone;
    }

    // ---------- окружение ----------
    public static GameObject BuildEnvironment(DroneBuildContext ctx)
    {
        var env = new GameObject("Environment");
        ctx.register?.Invoke(env);
        env.AddComponent<WindSystem>();

        Prim(PrimitiveType.Cube, "Ground", env.transform, new Vector3(0, -0.5f, 0),
            new Vector3(400, 1, 400), ctx, "Ground", true);
        Prim(PrimitiveType.Cube, "HomePad", env.transform, new Vector3(0, 0.01f, 0),
            new Vector3(2, 0.02f, 2), ctx, "Pad", true);

        for (int i = 0; i < 2; i++)
        {
            var gate = new GameObject("Gate" + i);
            gate.transform.SetParent(env.transform, false);
            gate.transform.position = new Vector3(i * 10f, 0, 14f + i * 12f);
            gate.transform.rotation = Quaternion.Euler(0, i * 30f, 0);
            string mat = i == 0 ? "GateA" : "GateB";
            Prim(PrimitiveType.Cube, "L", gate.transform, new Vector3(-1.5f, 1.25f, 0), new Vector3(0.15f, 2.5f, 0.15f), ctx, mat, true);
            Prim(PrimitiveType.Cube, "R", gate.transform, new Vector3(1.5f, 1.25f, 0), new Vector3(0.15f, 2.5f, 0.15f), ctx, mat, true);
            Prim(PrimitiveType.Cube, "T", gate.transform, new Vector3(0, 2.5f, 0), new Vector3(3.15f, 0.15f, 0.15f), ctx, mat, true);
        }

        var rnd = new System.Random(42);
        for (int i = 0; i < 6; i++)
        {
            float h = 1f + (float)rnd.NextDouble() * 4f;
            Prim(PrimitiveType.Cube, "Box" + i, env.transform,
                new Vector3((float)rnd.NextDouble() * 60f - 30f, h / 2f, 20f + (float)rnd.NextDouble() * 50f),
                new Vector3(2, h, 2), ctx, "Obstacle", true);
        }

        if (UnityEngine.Object.FindObjectOfType<Light>() == null)
        {
            var lg = new GameObject("Sun");
            ctx.register?.Invoke(lg);
            lg.transform.SetParent(env.transform, false);
            var l = lg.AddComponent<Light>();
            l.type = LightType.Directional; l.intensity = 1.2f;
            lg.transform.rotation = Quaternion.Euler(50, -30, 0);
            RenderSettings.ambientLight = new Color(0.45f, 0.48f, 0.52f);
        }

        if (ctx.markEnvironmentStatic)
            foreach (Transform t in env.transform) t.gameObject.isStatic = true;

        return env;
    }

    // ---------- дрон ----------
    public static GameObject BuildDrone(DroneBuildContext ctx)
    {
        var d = new GameObject("Quadcopter");
        ctx.register?.Invoke(d);
        d.transform.position = ctx.spawn;

        Prim(PrimitiveType.Cube, "Body", d.transform, Vector3.zero, new Vector3(0.14f, 0.05f, 0.22f), ctx, "Body", true);

        var input = d.AddComponent<DroneInputManager>();
        var udp = d.AddComponent<UdpLink>();
        var bat = d.AddComponent<BatteryModel>();
        var ph = d.AddComponent<QuadPhysics>();
        ph.AutoLayout();
        ph.SetupRigidbody();                       // масса/инерция сохраняются в сцене
        var fc = d.AddComponent<FlightController>();
        var audio = d.AddComponent<AudioSource>();
        audio.playOnAwake = false; audio.spatialBlend = 1f;
        var ms = d.AddComponent<MotorSound>(); ms.physics = ph;

        for (int i = 0; i < 4; i++)
        {
            Vector3 mp = ph.motors[i].localPosition;

            var beam = Prim(PrimitiveType.Cube, "Arm" + i, d.transform, mp * 0.5f,
                new Vector3(0.03f, 0.02f, mp.magnitude), ctx, "Arm", true);
            beam.transform.localRotation = Quaternion.LookRotation(mp.normalized);

            var pod = Prim(PrimitiveType.Cylinder, "Motor" + i, d.transform, mp + Vector3.up * 0.02f,
                new Vector3(0.05f, 0.02f, 0.05f), ctx, "Motor", true);
            RemoveCollider(pod);
            pod.AddComponent<SphereCollider>().radius = 0.6f;

            var prop = new GameObject("Prop" + i);
            prop.transform.SetParent(d.transform, false);
            prop.transform.localPosition = mp + Vector3.up * 0.045f;

            var blades = new GameObject("Blades");
            blades.transform.SetParent(prop.transform, false);
            Prim(PrimitiveType.Cube, "Blade1", blades.transform, Vector3.zero, new Vector3(0.125f, 0.003f, 0.012f), ctx, "Blade", false);
            Prim(PrimitiveType.Cube, "Blade2", blades.transform, Vector3.zero, new Vector3(0.012f, 0.003f, 0.125f), ctx, "Blade", false);
            var disc = Prim(PrimitiveType.Cylinder, "Disc", prop.transform, Vector3.zero, new Vector3(0.13f, 0.001f, 0.13f), ctx, "Disc", false);

            var pv = prop.AddComponent<PropVisual>();
            pv.physics = ph; pv.motorIndex = i;
            pv.bladesRoot = blades;
            pv.blurDisc = disc.GetComponent<Renderer>();
        }

        // сериализованные ссылки
        var wind = UnityEngine.Object.FindObjectOfType<WindSystem>();
        input.udp = udp;
        ph.wind = wind;
        fc.input = input; fc.physics = ph; fc.battery = bat; fc.udp = udp; fc.wind = wind;
        return d;
    }

    // ---------- камера и HUD ----------
    public static void BuildCameraAndHud(DroneBuildContext ctx, Transform drone)
    {
        var c = Camera.main;
        GameObject cg;
        if (c != null) cg = c.gameObject;
        else
        {
            cg = new GameObject("SimCamera");
            ctx.register?.Invoke(cg);
            cg.tag = "MainCamera";
            cg.AddComponent<Camera>();
            cg.transform.position = drone.position + new Vector3(0, 2f, -5f);
        }
        if (UnityEngine.Object.FindObjectOfType<AudioListener>() == null) cg.AddComponent<AudioListener>();
        var cc = cg.GetComponent<ChaseCamera>();
        if (cc == null) cc = cg.AddComponent<ChaseCamera>();
        cc.target = drone;
        cc.input = drone.GetComponent<DroneInputManager>();

        if (UnityEngine.Object.FindObjectOfType<SimHud>() == null)
        {
            var hg = new GameObject("HUD");
            ctx.register?.Invoke(hg);
            hg.AddComponent<SimHud>();
        }
    }

    static void EnsureBootstrap(GameObject drone)
    {
        var boot = UnityEngine.Object.FindObjectOfType<SimBootstrap>();
        if (boot == null)
        {
            var g = new GameObject("Simulator");
            boot = g.AddComponent<SimBootstrap>();
        }
        boot.autoBuildAtRuntime = false;
        boot.drone = drone.GetComponent<QuadPhysics>();
    }

    // ---------- перепривязка всех ссылок ----------
    public static void WireReferences()
    {
        var ph = UnityEngine.Object.FindObjectOfType<QuadPhysics>();
        var wind = UnityEngine.Object.FindObjectOfType<WindSystem>();
        var inp = UnityEngine.Object.FindObjectOfType<DroneInputManager>();
        var udp = UnityEngine.Object.FindObjectOfType<UdpLink>();
        var bat = UnityEngine.Object.FindObjectOfType<BatteryModel>();
        var fc = UnityEngine.Object.FindObjectOfType<FlightController>();
        var cam = UnityEngine.Object.FindObjectOfType<ChaseCamera>();
        var hud = UnityEngine.Object.FindObjectOfType<SimHud>();
        var ms = UnityEngine.Object.FindObjectOfType<MotorSound>();

        if (fc != null) { fc.physics = ph; fc.input = inp; fc.udp = udp; fc.battery = bat; fc.wind = wind; }
        if (inp != null) inp.udp = udp;
        if (ph != null) ph.wind = wind;
        if (ms != null) ms.physics = ph;
        foreach (var pv in UnityEngine.Object.FindObjectsOfType<PropVisual>()) pv.physics = ph;
        if (cam != null && ph != null) { cam.target = ph.transform; cam.input = inp; }
        if (hud != null)
        { hud.fc = fc; hud.ph = ph; hud.bat = bat; hud.wind = wind; hud.inp = inp; hud.udp = udp; hud.cam = cam; }

        var boot = UnityEngine.Object.FindObjectOfType<SimBootstrap>();
        if (boot != null) { boot.drone = ph; boot.input = inp; boot.chaseCamera = cam; boot.hud = hud; }
    }

    // ---------- утилиты ----------
    static GameObject Prim(PrimitiveType t, string name, Transform parent, Vector3 lp, Vector3 scale,
                           DroneBuildContext ctx, string mat, bool collider)
    {
        var g = GameObject.CreatePrimitive(t);
        g.name = name;
        g.transform.SetParent(parent, false);
        g.transform.localPosition = lp;
        g.transform.localScale = scale;
        g.GetComponent<Renderer>().sharedMaterial = ctx.Get(mat, Palette[mat]);
        if (!collider) RemoveCollider(g);
        return g;
    }

    static void RemoveCollider(GameObject g)
    {
        var c = g.GetComponent<Collider>();
        if (c == null) return;
        if (Application.isPlaying) UnityEngine.Object.Destroy(c);
        else UnityEngine.Object.DestroyImmediate(c);
    }
}
```

---

## 2. Новый файл `Assets/Editor/DroneSimEditorTools.cs`

> Обязательно в папке **Editor** — иначе Unity не соберёт билд.

```csharp
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class DroneSimEditorTools
{
    const string MatFolder = "Assets/DroneSim/Materials";
    const string PrefabFolder = "Assets/DroneSim/Prefabs";

    [MenuItem("Tools/Drone Sim/Build Demo Scene %#d")]
    public static void BuildScene()
    {
        var existing = Object.FindObjectOfType<QuadPhysics>();
        if (existing != null)
        {
            if (!EditorUtility.DisplayDialog("Дрон уже есть в сцене",
                "Удалить существующие объекты симулятора и собрать заново?", "Пересобрать", "Отмена")) return;
            ClearSimObjects(false);
        }

        Undo.IncrementCurrentGroup();
        Undo.SetCurrentGroupName("Build Drone Scene");
        var drone = DroneSceneBuilder.BuildAll(EditorContext());
        ApplyPhysicsSettings(200);
        MarkDirty();
        Selection.activeGameObject = drone;
        EditorGUIUtility.PingObject(drone);
        Debug.Log("[DroneSim] Сцена создана в редакторе. Сохраните сцену (Ctrl+S) — при Play она больше не генерируется.");
    }

    [MenuItem("Tools/Drone Sim/Rebuild Drone Only")]
    public static void RebuildDrone()
    {
        var old = Object.FindObjectOfType<QuadPhysics>();
        Vector3 spawn = old != null ? old.transform.position : new Vector3(0, 0.12f, 0);
        if (old != null) Undo.DestroyObjectImmediate(old.gameObject);

        Undo.IncrementCurrentGroup();
        Undo.SetCurrentGroupName("Rebuild Drone");
        var ctx = EditorContext(); ctx.spawn = spawn;
        var drone = DroneSceneBuilder.BuildDrone(ctx);
        DroneSceneBuilder.BuildCameraAndHud(ctx, drone.transform);
        DroneSceneBuilder.WireReferences();
        MarkDirty();
        Selection.activeGameObject = drone;
    }

    [MenuItem("Tools/Drone Sim/Wire References")]
    public static void Wire()
    {
        DroneSceneBuilder.WireReferences();
        MarkDirty();
        Debug.Log("[DroneSim] Ссылки перепривязаны.");
    }

    [MenuItem("Tools/Drone Sim/Clear Sim Objects")]
    public static void ClearSimMenu() => ClearSimObjects(true);

    public static void ClearSimObjects(bool ask)
    {
        if (ask && !EditorUtility.DisplayDialog("Удалить объекты симулятора",
            "Будут удалены Environment, Quadcopter, SimCamera, HUD. Продолжить?", "Удалить", "Отмена")) return;

        string[] roots = { "Quadcopter", "Environment", "SimCamera", "HUD" };
        foreach (var name in roots)
        {
            var go = GameObject.Find(name);
            if (go != null) Undo.DestroyObjectImmediate(go);
        }
        MarkDirty();
    }

    [MenuItem("Tools/Drone Sim/Apply Physics Settings 200 Hz")]
    public static void ApplyPhysicsMenu() { ApplyPhysicsSettings(200); Debug.Log("[DroneSim] Fixed Timestep = 0.005 (200 Hz)."); }

    [MenuItem("Tools/Drone Sim/Save Drone As Prefab")]
    public static void SavePrefab()
    {
        var ph = Object.FindObjectOfType<QuadPhysics>();
        if (ph == null) { EditorUtility.DisplayDialog("Нет дрона", "В сцене нет Quadcopter.", "OK"); return; }
        if (!AssetDatabase.IsValidFolder("Assets/DroneSim")) AssetDatabase.CreateFolder("Assets", "DroneSim");
        if (!AssetDatabase.IsValidFolder(PrefabFolder)) AssetDatabase.CreateFolder("Assets/DroneSim", "Prefabs");
        PrefabUtility.SaveAsPrefabAssetAndConnect(ph.gameObject, PrefabFolder + "/Quadcopter.prefab", InteractionMode.UserAction);
        Debug.Log("[DroneSim] Префаб сохранён: " + PrefabFolder + "/Quadcopter.prefab");
    }

    // ---------- внутреннее ----------
    static DroneBuildContext EditorContext()
    {
        var boot = Object.FindObjectOfType<SimBootstrap>();
        return new DroneBuildContext
        {
            spawn = boot != null ? boot.spawnPos : new Vector3(0, 0.12f, 0),
            markEnvironmentStatic = true,
            materialFactory = EditorMaterial,
            register = go => Undo.RegisterCreatedObjectUndo(go, "Build Drone Scene")
        };
    }

    static Material EditorMaterial(string name, Color c)
    {
        if (!AssetDatabase.IsValidFolder("Assets/DroneSim")) AssetDatabase.CreateFolder("Assets", "DroneSim");
        if (!AssetDatabase.IsValidFolder(MatFolder)) AssetDatabase.CreateFolder("Assets/DroneSim", "Materials");

        string path = $"{MatFolder}/M_{name}.mat";
        var m = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (m == null)
        {
            Shader sh = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
            m = new Material(sh) { color = c };
            AssetDatabase.CreateAsset(m, path);
            AssetDatabase.SaveAssets();
        }
        else { m.color = c; EditorUtility.SetDirty(m); }
        return m;
    }

    static void ApplyPhysicsSettings(int hz)
    {
        try
        {
            var tm = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TimeManager.asset")[0];
            var so = new SerializedObject(tm);
            so.FindProperty("Fixed Timestep").floatValue = 1f / hz;
            so.FindProperty("Maximum Allowed Timestep").floatValue = 0.1f;
            so.ApplyModifiedProperties();
        }
        catch (System.Exception e) { Debug.LogWarning("[DroneSim] TimeManager: " + e.Message); }

        var boot = Object.FindObjectOfType<SimBootstrap>();
        if (boot != null) { boot.physicsHz = hz; EditorUtility.SetDirty(boot); }
    }

    static void MarkDirty()
    {
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
    }
}

[CustomEditor(typeof(SimBootstrap))]
public class SimBootstrapInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        EditorGUILayout.Space(8);
        EditorGUILayout.LabelField("Создание сцены в редакторе", EditorStyles.boldLabel);

        if (GUILayout.Button("СОЗДАТЬ ДЕМО-СЦЕНУ (статичную)", GUILayout.Height(30)))
            DroneSimEditorTools.BuildScene();
        if (GUILayout.Button("Пересобрать только дрон"))
            DroneSimEditorTools.RebuildDrone();
        if (GUILayout.Button("Привязать ссылки"))
            DroneSimEditorTools.Wire();
        if (GUILayout.Button("Сохранить дрон как префаб"))
            DroneSimEditorTools.SavePrefab();
        if (GUILayout.Button("Удалить объекты симулятора"))
            DroneSimEditorTools.ClearSimMenu();

        EditorGUILayout.HelpBox(
            "Сцена строится в редакторе и сохраняется вместе с проектом. " +
            "autoBuildAtRuntime должен быть ВЫКЛЮЧЕН — тогда при Play ничего не генерируется.",
            MessageType.Info);
    }
}
```

---

## 3. Полная замена `SimBootstrap.cs`

```csharp
using UnityEngine;

[DefaultExecutionOrder(-100)]
public class SimBootstrap : MonoBehaviour
{
    [Header("Режим сборки")]
    [Tooltip("ВЫКЛ — сцена создана в редакторе и не трогается при Play (рекомендуется). ВКЛ — старое поведение: генерация при запуске.")]
    public bool autoBuildAtRuntime = false;
    [Tooltip("При авто-сборке: не строить, если дрон уже есть в сцене")]
    public bool buildOnlyIfEmpty = true;
    [Tooltip("Точка спавна для авто-сборки и для кнопки построения сцены")]
    public Vector3 spawnPos = new Vector3(0f, 0.12f, 0f);

    [Header("Настройки времени выполнения")]
    public bool applyFixedTimestep = true;
    [Tooltip("Частота физики, Гц")] [Range(50, 400)] public int physicsHz = 200;
    [Range(30, 240)] public int targetFps = 144;
    [Range(0.02f, 0.3f)] public float maxDeltaTime = 0.1f;

    [Header("Ссылки на объекты сцены")]
    public QuadPhysics drone;
    public DroneInputManager input;
    public ChaseCamera chaseCamera;
    public SimHud hud;

    void Awake()
    {
        if (applyFixedTimestep)
        {
            Time.fixedDeltaTime = 1f / physicsHz;
            Time.maximumDeltaTime = maxDeltaTime;
        }
        Application.targetFrameRate = targetFps;

        bool needBuild = autoBuildAtRuntime &&
                         (!buildOnlyIfEmpty || (drone == null && FindObjectOfType<QuadPhysics>() == null));
        if (needBuild) DroneSceneBuilder.BuildAll(DroneBuildContext.Runtime(spawnPos));

        Wire();

        if (drone == null)
            Debug.LogWarning("[SimBootstrap] Дрон не найден. Постройте сцену: Tools ▸ Drone Sim ▸ Build Demo Scene " +
                             "(или кнопка в инспекторе SimBootstrap).");
    }

    public void Wire()
    {
        if (drone == null)
        {
            var p = FindObjectOfType<QuadPhysics>();
            if (p != null) drone = p;
        }
        if (drone == null) return;

        if (input == null) input = drone.GetComponent<DroneInputManager>();
        if (chaseCamera == null) chaseCamera = FindObjectOfType<ChaseCamera>();
        if (chaseCamera != null)
        {
            chaseCamera.target = drone.transform;
            if (chaseCamera.input == null) chaseCamera.input = input;
        }
        if (hud == null) hud = FindObjectOfType<SimHud>();
    }
}
```

---

## 4. Четыре маленьких патча

### 4.1 `QuadPhysics.cs` — настройка Rigidbody в редакторе + сохранение позиций моторов

Найди блок от `Rigidbody rb; WindSystem wind;` до конца метода `ApplyFrame()` и замени на:

```csharp
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
        rb.drag = 0; rb.angularDrag = 0;
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
```

### 4.2 `SimHud.cs` — ссылки в инспекторе

Замени первые строки класса (поля ссылок) на:

```csharp
    [Header("Ссылки (заполняются в редакторе, иначе найдутся сами)")]
    public FlightController fc;
    public QuadPhysics ph;
    public BatteryModel bat;
    public WindSystem wind;
    public DroneInputManager inp;
    public UdpLink udp;
    public ChaseCamera cam;
```

И метод `Start()`:

```csharp
    void Start()
    {
        if (fc == null) fc = FindObjectOfType<FlightController>();
        if (ph == null) ph = FindObjectOfType<QuadPhysics>();
        if (bat == null) bat = FindObjectOfType<BatteryModel>();
        if (wind == null) wind = FindObjectOfType<WindSystem>();
        if (inp == null) inp = FindObjectOfType<DroneInputManager>();
        if (udp == null) udp = FindObjectOfType<UdpLink>();
        if (cam == null) cam = FindObjectOfType<ChaseCamera>();

        white = new Texture2D(1, 1); white.SetPixel(0, 0, Color.white); white.Apply();
        enabled = fc != null;
    }
```

### 4.3 `DronePresentation.cs` — класс `PropVisual` целиком (заодно чинит вторую лопасть, которая раньше не пряталась на высоких оборотах)

```csharp
public class PropVisual : MonoBehaviour
{
    public QuadPhysics physics; public int motorIndex;
    [Tooltip("Корень лопастей — скрывается целиком на высоких оборотах")]
    public GameObject bladesRoot;
    public Renderer blurDisc; public Renderer blades;
    [Tooltip("Обороты, выше которых лопасти скрываются в диск")] public float blurRpm = 3500f;
    float angle; Color discColor;

    void Start() { if (blurDisc) discColor = blurDisc.material.color; }

    void Update()
    {
        if (physics == null) return;
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
```

### 4.4 `DronePresentation.cs` — начало `MotorSound.Start()` (переиспользуем AudioSource из сцены)

```csharp
    void Start()
    {
        src = GetComponent<AudioSource>();
        if (src == null) src = gameObject.AddComponent<AudioSource>();
        src.loop = true; src.spatialBlend = 1f; src.minDistance = 2f; src.maxDistance = 40f;
        // ...дальше без изменений (генерация клипа и src.Play())
```

И в `DroneInputManager.Awake()` первая строка — чтобы не перетирать ссылку из редактора:

```csharp
        if (udp == null) udp = GetComponent<UdpLink>();
```

---

## Как пользоваться

1. Удали из сцены всё, что осталось от старых запусков (в рантайме ничего не сохранялось, так что сцена обычно пустая — только твой `Simulator`).
2. Выдели `Simulator` → в инспекторе кнопка **«СОЗДАТЬ ДЕМО-СЦЕНУ»** (или `Tools ▸ Drone Sim ▸ Build Demo Scene`, `Ctrl+Alt+D`).
3. **Ctrl+S** — сцена сохранена: `Environment` (static), `Quadcopter`, `SimCamera`, `HUD`, материалы в `Assets/DroneSim/Materials`.
4. Правь что угодно в редакторе: двигай ворота, меняй меши/материалы дрона, крути `motorOffset` и позиции моторов (жёлтые gizmos видны при выделении дрона), меняй массу → контекст-меню компонента `QuadPhysics` → *«Применить параметры рамы»*.
5. Если что-то переставил вручную и ссылки поехали — кнопка **«Привязать ссылки»**.
6. Play — `SimBootstrap` только выставит `fixedDeltaTime`/FPS и подхватит ссылки, ничего не создавая.

Пара нюансов: `Fixed Timestep = 0.005` теперь записан в Project Settings (меню *Apply Physics Settings 200 Hz*), поэтому симуляция стабильна и без рантайм-переопределения; дрон можно сохранить префабом (*Save Drone As Prefab*) и переиспользовать в других сценах — HUD и камера найдут его по ссылкам или автоматически.

### USER
я все сделал спасибо работает!  теперь надо доработать режим крушения пусть наш дрон состоящий из множества обьектов разваливается при ударе просто сделай так чтобы на каждый gameobject применялся свой boxcollider и Rigidbody с физикой и гравитацией и оно само разлетится реалистично по сцене доработай то место кода с крушением!  

### ASSISTANT
Отлично, сделаем «разбор на обломки»: при сильном ударе каждая деталь отсоединяется от дрона, получает собственный `BoxCollider` (по границам меша) и `Rigidbody` с гравитацией, наследует предударную скорость дрона + тангенциальную скорость от вращения, получает импульс от точки удара и кувыркается по сцене. По клавише **R** дрон собирается обратно в исходную иерархию.

---

## 1. Новый файл `Assets/Scripts/DroneBreakup.cs`

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum BreakupGranularity { EveryRenderer, TopLevelParts, Manual }

// Разбирает дрон на физические обломки при краше и собирает обратно по Reset.
public class DroneBreakup : MonoBehaviour
{
    [System.Serializable]
    class Piece
    {
        public Transform tr;
        public Transform parent;
        public Vector3 localPos, localScale;
        public Quaternion localRot;
        public Rigidbody rb;
        public BoxCollider box;
        public bool boxAdded;
        public bool wasActive;
        public float volume;
        public List<Collider> offColliders = new List<Collider>();
        public List<MonoBehaviour> offScripts = new List<MonoBehaviour>();
    }

    [Header("Ссылка")]
    public QuadPhysics physics;

    [Header("Условие разрушения")]
    [Tooltip("Минимальная скорость удара для развала, м/с. Выше crashSpeed — разваливается только при жёстких ударах")]
    [Range(1f, 30f)] public float minImpactSpeed = 7f;
    [Tooltip("Разваливать при любом краше, игнорируя порог скорости")]
    public bool breakOnEveryCrash = false;

    [Header("На что разваливается")]
    [Tooltip("EveryRenderer — каждая деталь с мешом; TopLevelParts — только прямые дети (винт целиком); Manual — список вручную")]
    public BreakupGranularity granularity = BreakupGranularity.EveryRenderer;
    public List<Transform> manualParts = new List<Transform>();
    [Tooltip("Объекты с этими подстроками в имени обломками не станут и будут скрыты (например blur-диск винта)")]
    public string[] excludeNames = { "Disc" };
    [Tooltip("Отключать сторонние скрипты на обломках (PropVisual и т.п.)")]
    public bool disableScriptsOnPieces = true;

    [Header("Разлёт")]
    [Tooltip("Насколько обломки наследуют предударную скорость дрона (1 = полностью)")]
    [Range(0f, 1f)] public float velocityInheritance = 0.85f;
    [Tooltip("Учитывать вращение дрона: дальние детали получают тангенциальную скорость")]
    public bool usePointVelocity = true;
    [Tooltip("Базовая скорость разлёта от точки удара, м/с")]
    [Range(0f, 20f)] public float explosionForce = 3f;
    [Tooltip("Добавка к разлёту за каждый м/с скорости удара")]
    [Range(0f, 1f)] public float impactForceScale = 0.25f;
    [Tooltip("Радиус затухания импульса от точки удара, м")]
    [Range(0.05f, 3f)] public float explosionRadius = 0.6f;
    [Tooltip("Подброс вверх")] [Range(0f, 2f)] public float upwardBias = 0.35f;
    [Tooltip("Случайный разброс направления")] [Range(0f, 1f)] public float randomness = 0.35f;
    [Tooltip("Случайное вращение обломков, рад/с")] [Range(0f, 60f)] public float spinTorque = 18f;

    [Header("Физика обломков")]
    [Tooltip("Суммарная масса обломков, кг (0 = взять массу дрона)")]
    public float totalMassOverride = 0f;
    [Tooltip("Масса распределяется по объёму деталей, но не меньше этого, кг")]
    public float minPieceMass = 0.01f;
    [Range(0f, 2f)] public float debrisDrag = 0.08f;
    [Range(0f, 3f)] public float debrisAngularDrag = 0.35f;
    [Range(0f, 1f)] public float bounciness = 0.25f;
    [Range(0f, 1f)] public float friction = 0.6f;
    [Tooltip("Свой физматериал; если пусто — создаётся автоматически")]
    public PhysicMaterial debrisMaterial;
    public bool continuousCollision = true;
    [Range(7f, 100f)] public float maxAngularVelocity = 45f;

    [Header("Столкновения обломков между собой")]
    [Tooltip("Детали изначально пересекаются — без задержки будет ложный 'взрыв'")]
    public bool debrisSelfCollision = false;
    [Tooltip("Через сколько секунд включить столкновения обломков друг с другом")]
    public float selfCollisionDelay = 0.4f;

    [Header("Прочее")]
    [Tooltip("Камера переключается на самый тяжёлый обломок")]
    public bool cameraFollowsLargestPiece = true;
    [Tooltip("Скрыть обломки через N секунд (0 = оставить навсегда)")]
    public float debrisLifetime = 0f;
    public AudioClip crashSound;
    [Range(0f, 1f)] public float crashSoundVolume = 0.8f;
    public GameObject crashParticles;
    public bool logBreakup = true;

    public bool IsBroken { get; private set; }
    public int PieceCount => pieces.Count;

    readonly List<Piece> pieces = new List<Piece>();
    readonly List<GameObject> hidden = new List<GameObject>();
    readonly List<Collider> pieceColliders = new List<Collider>();
    Vector3 lastVelocity, lastAngular;
    Transform savedCamTarget;
    ChaseCamera cam;
    MotorSound sound;
    PhysicMaterial runtimeMat;

    void Awake()
    {
        if (physics == null) physics = GetComponent<QuadPhysics>();
        cam = FindObjectOfType<ChaseCamera>();
        sound = GetComponent<MotorSound>();
        if (debrisMaterial == null)
        {
            runtimeMat = new PhysicMaterial("DebrisPhys")
            {
                bounciness = bounciness,
                dynamicFriction = friction,
                staticFriction = friction,
                bounceCombine = PhysicMaterialCombine.Maximum,
                frictionCombine = PhysicMaterialCombine.Average
            };
            debrisMaterial = runtimeMat;
        }
    }

    void OnEnable()
    {
        if (physics == null) physics = GetComponent<QuadPhysics>();
        if (physics != null)
        {
            physics.OnCrashDetailed += HandleCrash;
            physics.OnReset += Reassemble;
        }
    }
    void OnDisable()
    {
        if (physics != null)
        {
            physics.OnCrashDetailed -= HandleCrash;
            physics.OnReset -= Reassemble;
        }
    }

    void FixedUpdate()
    {
        if (IsBroken || physics == null || physics.Rb == null) return;
        lastVelocity = physics.Rb.velocity;   // предударная скорость (OnCollisionEnter приходит уже после solver'а)
        lastAngular = physics.Rb.angularVelocity;
    }

    void HandleCrash(CrashInfo info)
    {
        if (IsBroken) return;
        if (!breakOnEveryCrash && info.speed < minImpactSpeed) return;
        BreakApart(info);
    }

    [ContextMenu("Развалить (тест)")]
    public void BreakApartTest()
    {
        if (IsBroken) return;
        BreakApart(new CrashInfo
        {
            point = transform.position,
            relativeVelocity = lastVelocity,
            speed = Mathf.Max(minImpactSpeed, lastVelocity.magnitude)
        });
    }

    // ---------- РАЗБОР ----------
    public void BreakApart(CrashInfo info)
    {
        if (IsBroken || physics == null) return;
        var targets = CollectPieces();
        if (targets.Count == 0) { Debug.LogWarning("[Breakup] Не найдено деталей для разбора."); return; }

        Rigidbody rootRb = physics.Rb;
        Vector3 rootVel = lastVelocity.sqrMagnitude > 0.01f ? lastVelocity : (rootRb ? rootRb.velocity : Vector3.zero);
        Vector3 rootAng = lastAngular;

        // 1. Снимок исходного состояния + скорости ДО отсоединения
        float totalVolume = 0f;
        foreach (var t in targets)
        {
            var p = new Piece
            {
                tr = t,
                parent = t.parent,
                localPos = t.localPosition,
                localRot = t.localRotation,
                localScale = t.localScale,
                wasActive = t.gameObject.activeSelf
            };
            Bounds lb = LocalBounds(t);
            Vector3 ws = Vector3.Scale(lb.size, t.lossyScale);
            p.volume = Mathf.Max(1e-6f, ws.x * ws.y * ws.z);
            totalVolume += p.volume;
            pieces.Add(p);
        }

        float totalMass = totalMassOverride > 0f ? totalMassOverride : physics.massKg;
        IsBroken = true;

        // 2. Отсоединение, коллайдеры, rigidbody, импульсы
        foreach (var p in pieces)
        {
            Transform t = p.tr;
            Vector3 worldPos = t.position;

            Vector3 pointVel = rootVel;
            if (usePointVelocity && rootRb != null) pointVel = rootRb.GetPointVelocity(worldPos);
            else if (usePointVelocity) pointVel = rootVel + Vector3.Cross(rootAng, worldPos - transform.position);

            foreach (var c in t.GetComponentsInChildren<Collider>(true))
            { if (c.enabled) { p.offColliders.Add(c); c.enabled = false; } }

            if (disableScriptsOnPieces)
                foreach (var mb in t.GetComponentsInChildren<MonoBehaviour>(true))
                { if (mb != null && mb.enabled && mb != this) { p.offScripts.Add(mb); mb.enabled = false; } }

            t.SetParent(null, true);

            var box = t.GetComponent<BoxCollider>();
            if (box == null) { box = t.gameObject.AddComponent<BoxCollider>(); p.boxAdded = true; }
            Bounds lb = LocalBounds(t);
            box.center = lb.center;
            box.size = lb.size;
            box.enabled = true;
            box.material = debrisMaterial;
            p.box = box;
            pieceColliders.Add(box);

            var rb = t.gameObject.AddComponent<Rigidbody>();
            rb.mass = Mathf.Max(minPieceMass, totalMass * (p.volume / totalVolume));
            rb.useGravity = true;
            rb.drag = debrisDrag;
            rb.angularDrag = debrisAngularDrag;
            rb.maxAngularVelocity = maxAngularVelocity;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = continuousCollision
                ? CollisionDetectionMode.ContinuousDynamic : CollisionDetectionMode.Discrete;
            p.rb = rb;

            // импульс от точки удара
            Vector3 dir = worldPos - info.point;
            float dist = dir.magnitude;
            dir = dist > 0.001f ? dir / dist : (worldPos - transform.position).normalized;
            if (dir.sqrMagnitude < 0.001f) dir = Random.onUnitSphere;
            dir = Vector3.Slerp(dir, Random.onUnitSphere, randomness);
            dir += Vector3.up * upwardBias;
            dir.Normalize();

            float falloff = Mathf.Lerp(1f, 0.35f, Mathf.Clamp01(dist / Mathf.Max(0.05f, explosionRadius)));
            float force = (explosionForce + info.speed * impactForceScale) * falloff;

            rb.velocity = pointVel * velocityInheritance + dir * force;
            rb.angularVelocity = rootAng * 0.5f + Random.insideUnitSphere * spinTorque;
        }

        // 3. Обломки не должны расталкивать друг друга из исходного перекрытия
        if (!debrisSelfCollision) IgnoreAll(true);
        else StartCoroutine(SelfCollisionLater());

        // 4. Скрыть исключённую визуальщину (blur-диски и т.п.)
        foreach (var r in GetComponentsInChildren<Renderer>(true))
            if (IsExcluded(r.name) && r.gameObject.activeSelf)
            { hidden.Add(r.gameObject); r.gameObject.SetActive(false); }

        foreach (var pv in GetComponentsInChildren<PropVisual>(true)) pv.enabled = false;
        if (sound != null) sound.enabled = false;

        // 5. Корень остаётся маркером места крушения, физика дрона выключается
        physics.EnterWreckedState();

        // 6. Камера, звук, эффекты
        if (cameraFollowsLargestPiece && cam != null)
        {
            Piece biggest = null;
            foreach (var p in pieces) if (biggest == null || p.rb.mass > biggest.rb.mass) biggest = p;
            if (biggest != null) { savedCamTarget = cam.target; cam.target = biggest.tr; }
        }
        if (crashSound != null) AudioSource.PlayClipAtPoint(crashSound, info.point, crashSoundVolume);
        if (crashParticles != null) Destroy(Instantiate(crashParticles, info.point, Quaternion.identity), 6f);
        if (debrisLifetime > 0f) StartCoroutine(HideDebrisLater());

        if (logBreakup)
            Debug.Log($"[Breakup] Удар {info.speed:0.0} м/с → дрон разобран на {pieces.Count} обломков.");
    }

    IEnumerator SelfCollisionLater()
    {
        yield return new WaitForSeconds(selfCollisionDelay);
        if (IsBroken) IgnoreAll(false);
    }
    IEnumerator HideDebrisLater()
    {
        yield return new WaitForSeconds(debrisLifetime);
        if (!IsBroken) yield break;
        foreach (var p in pieces) if (p.tr != null) p.tr.gameObject.SetActive(false);
    }

    void IgnoreAll(bool ignore)
    {
        for (int i = 0; i < pieceColliders.Count; i++)
            for (int j = i + 1; j < pieceColliders.Count; j++)
            {
                var a = pieceColliders[i]; var b = pieceColliders[j];
                if (a != null && b != null) Physics.IgnoreCollision(a, b, ignore);
            }
    }

    // ---------- СБОРКА ----------
    [ContextMenu("Собрать обратно")]
    public void Reassemble()
    {
        if (!IsBroken) return;
        StopAllCoroutines();
        IgnoreAll(false);

        foreach (var p in pieces)
        {
            if (p.tr == null) continue;
            if (p.rb != null)
            {
                p.rb.isKinematic = true;
                p.rb.detectCollisions = false;
                Destroy(p.rb);
            }
            if (p.box != null)
            {
                p.box.enabled = false;
                if (p.boxAdded) Destroy(p.box);
                else p.box.enabled = true;
            }
            p.tr.gameObject.SetActive(p.wasActive);
            p.tr.SetParent(p.parent != null ? p.parent : transform, false);
            p.tr.localPosition = p.localPos;
            p.tr.localRotation = p.localRot;
            p.tr.localScale = p.localScale;
            foreach (var c in p.offColliders) if (c != null) c.enabled = true;
            foreach (var mb in p.offScripts) if (mb != null) mb.enabled = true;
        }
        foreach (var g in hidden) if (g != null) g.SetActive(true);
        foreach (var pv in GetComponentsInChildren<PropVisual>(true)) pv.enabled = true;
        if (sound != null) sound.enabled = true;

        pieces.Clear(); hidden.Clear(); pieceColliders.Clear();
        IsBroken = false;

        physics.ExitWreckedState();
        if (cam != null && savedCamTarget != null) { cam.target = savedCamTarget; savedCamTarget = null; }
        if (logBreakup) Debug.Log("[Breakup] Дрон собран обратно.");
    }

    // ---------- утилиты ----------
    List<Transform> CollectPieces()
    {
        var list = new List<Transform>();
        switch (granularity)
        {
            case BreakupGranularity.Manual:
                foreach (var t in manualParts)
                    if (t != null && t != transform && t.IsChildOf(transform) && !IsExcluded(t.name)) list.Add(t);
                break;

            case BreakupGranularity.TopLevelParts:
                foreach (Transform child in transform)
                    if (!IsExcluded(child.name) && child.GetComponentsInChildren<Renderer>(true).Length > 0) list.Add(child);
                break;

            default:
                foreach (var r in GetComponentsInChildren<Renderer>(true))
                {
                    if (r.transform == transform || IsExcluded(r.name)) continue;
                    if (r is ParticleSystemRenderer || r is TrailRenderer || r is LineRenderer) continue;
                    if (!list.Contains(r.transform)) list.Add(r.transform);
                }
                break;
        }
        // если предок уже в списке — вложенную деталь отдельно не разбираем
        list.RemoveAll(t =>
        {
            var par = t.parent;
            while (par != null && par != transform) { if (list.Contains(par)) return true; par = par.parent; }
            return false;
        });
        return list;
    }

    bool IsExcluded(string n)
    {
        if (excludeNames == null) return false;
        foreach (var s in excludeNames)
            if (!string.IsNullOrEmpty(s) && n.IndexOf(s, System.StringComparison.OrdinalIgnoreCase) >= 0) return true;
        return false;
    }

    // Границы в ЛОКАЛЬНЫХ координатах детали (mesh.bounds точнее, чем Renderer.bounds)
    Bounds LocalBounds(Transform t)
    {
        var mf = t.GetComponent<MeshFilter>();
        if (mf != null && mf.sharedMesh != null) return mf.sharedMesh.bounds;

        var rends = t.GetComponentsInChildren<Renderer>(true);
        bool has = false;
        Bounds b = new Bounds(Vector3.zero, Vector3.zero);
        foreach (var r in rends)
        {
            if (IsExcluded(r.name)) continue;
            var wb = r.bounds;
            for (int i = 0; i < 8; i++)
            {
                Vector3 corner = wb.center + Vector3.Scale(wb.extents, new Vector3(
                    (i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                Vector3 lp = t.InverseTransformPoint(corner);
                if (!has) { b = new Bounds(lp, Vector3.zero); has = true; }
                else b.Encapsulate(lp);
            }
        }
        return has ? b : new Bounds(Vector3.zero, Vector3.one * 0.05f);
    }

    void OnDestroy() { if (runtimeMat != null) Destroy(runtimeMat); }
}
```

---

## 2. Патчи `QuadPhysics.cs` (4 куска)

### 2.1 Добавь структуру **перед** `public class QuadPhysics` (рядом с `MotorUnit`)

```csharp
public struct CrashInfo
{
    public Vector3 point;             // точка контакта
    public Vector3 relativeVelocity;  // скорость удара
    public float speed;               // её модуль
    public Collider other;            // во что ударились
}
```

### 2.2 Замени блок публичных свойств/событий

```csharp
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
```

### 2.3 Замени `OnCollisionEnter`

```csharp
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
```

### 2.4 Guards + состояние «разобран» + сброс

В **начало** `StepMotors(...)` первой строкой:

```csharp
        if (Wrecked) return;
```

В **начало** `ApplyAero(...)` первой строкой:

```csharp
        if (Wrecked)
        {
            TotalThrustN = 0; PowerW = 0; Airspeed = 0;
            VrsActive = false; GroundEffectFactor = 1f; GroundContact = false;
            return;
        }
```

И замени `ResetState()` целиком, добавив рядом два новых метода:

```csharp
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
            rb.velocity = Vector3.zero;
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
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }
```

> `FlightController`, `SimHud` и остальной код менять **не нужно** — корневой `Rigidbody` остаётся жив (кинематический), поэтому телеметрия, HUD и `R` продолжают работать как раньше.

---

## 3. Чтобы компонент появлялся в новых сборках

В `DroneSceneBuilder.BuildDrone(...)` после строки `var ms = d.AddComponent<MotorSound>(); ms.physics = ph;` добавь:

```csharp
        var br = d.AddComponent<DroneBreakup>(); br.physics = ph;
```

А в `DroneSceneBuilder.WireReferences()` рядом с остальными привязками:

```csharp
        var br = UnityEngine.Object.FindObjectOfType<DroneBreakup>();
        if (br != null) br.physics = ph;
```

Для **уже созданной** сцены просто добавь компонент вручную: выдели `Quadcopter` → *Add Component ▸ Drone Breakup* (ссылка на `QuadPhysics` подтянется сама). Либо добавь в `DroneSimEditorTools` пункт меню:

```csharp
    [MenuItem("Tools/Drone Sim/Add Breakup To Drone")]
    public static void AddBreakup()
    {
        var ph = Object.FindObjectOfType<QuadPhysics>();
        if (ph == null) { EditorUtility.DisplayDialog("Нет дрона", "В сцене нет Quadcopter.", "OK"); return; }
        var br = ph.GetComponent<DroneBreakup>();
        if (br == null) br = Undo.AddComponent<DroneBreakup>(ph.gameObject);
        br.physics = ph;
        EditorUtility.SetDirty(br);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = ph.gameObject;
    }
```

---

## Как проверить и настроить

1. Добавь компонент, Play, арм (`Space`), газ `W` ~60%, разгонись стрелкой вперёд и влети в стену/бокс на 12–15 м/с — дрон разлетится на 17 деталей (корпус, 4 луча, 4 мотора, 8 лопастей), blur-диски скроются, камера переключится на самый тяжёлый обломок.
2. **R** — всё собирается обратно в исходную иерархию, полёт как обычно.
3. Тест без краша: контекст-меню компонента (шестерёнка) → **«Развалить (тест)»** / **«Собрать обратно»**.

Ключевые ручки в инспекторе:

| Параметр | Эффект |
|---|---|
| `minImpactSpeed` | 7 — развал только при жёстком ударе; поставь 6 (= `crashSpeed`) — при любом краше |
| `granularity` | `EveryRenderer` — по детали; `TopLevelParts` — винт отлетает целиком (12 частей) |
| `velocityInheritance` | 0.85 — обломки летят по инерции вперёд, как настоящий дрон на скорости |
| `explosionForce` / `impactForceScale` | сила «взрыва»; для реализма без фантастики держи 2–4 и 0.2–0.3 |
| `spinTorque` | кувырки обломков |
| `debrisSelfCollision` | включи + `selfCollisionDelay` 0.4, если хочешь, чтобы обломки сталкивались друг с другом после разлёта |
| `debrisLifetime` | 0 — лежат навсегда; 10 — исчезнут через 10 с |
| `crashSound` / `crashParticles` | сюда можно кинуть свой клип удара и префаб искр/пыла |

Пара нюансов: массы распределяются по объёму деталей (корпус ~0.7 кг, лопасть ~0.01 кг), поэтому лёгкие лопасти планируют, а корпус падает камнем — это и даёт реалистичную картину. Если Unity 6.3 выдаст предупреждение/ошибку на `PhysicMaterial` (в новых версиях тип переименован в `PhysicsMaterial`) — просто замени оба упоминания в `DroneBreakup.cs` на `PhysicsMaterial`, логика не меняется.

### USER
джойстик к сожалению работает неправильно тумблеры и кнопки дают действие но стики не изменяют ничего можно ли это поправить или подскажи в чем дело может где то неправильные настройки просто?

### ASSISTANT
Да, это почти наверняка **не поломка физики, а маппинг HID-осей**: у SM001 порядок аналоговых осей в Unity часто не совпадает с нашим дефолтом `{0,1,2,3}`, а тумблер SwA попадает на кнопку/ось и поэтому «работает». Плюс часть RC-аппаратур Unity видит как generic **HID**, где оси приходят целыми числами (`IntegerControl`), а наш старый код читал только `AxisControl` — отсюда «стики ничего не меняют».

Ниже — обновлённый `DroneInputManager`, который: читает **ось любого типа** (float/int/HID), поддерживает **Joystick + HID**, ставит стики первыми, умеет **авто-назначать Mode 2** и позволяет **назначить канал кликом** прямо в мониторе F4. Калибровка из JSON сохранится.

---

## 1. Сначала быстрая диагностика (30 секунд)

1. Play → **F4** → блок «Сырые оси HID». Подвигай каждый стик.
   - **Если какая-то строка `[i]` меняет значение** — устройство читается, просто каналы смотрят не на те индексы. Лечится маппингом (ниже).
   - **Если ни одна строка не меняется, а кнопки/тумблеры меняются** — оси приходят как HID-int или устройство не Joystick. Новый код это чинит.
2. Заодно проверь в Windows: `Win+R` → `joy.cpl` → Свойства. Если там стики тоже не двигаются — проблема в драйвере/режиме USB аппаратуры (переключи режим USB на SM001 / переподключи), Unity тут ни при чём.

---

## 2. Полная замена `DroneInputManager.cs`

```csharp
using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using UnityEngine.InputSystem.HID;
#endif

[DefaultExecutionOrder(-50)]
public class DroneInputManager : MonoBehaviour
{
    public const int Channels = 6;
    public static readonly string[] ChannelNames = { "ROLL", "PITCH", "THR", "YAW", "AUX1(VrA)", "AUX2(SwA)" };

    [Header("Источники и микширование")]
    [Tooltip("MaxAbs — по каждому каналу побеждает источник с наибольшим отклонением (одновременная работа). Sum — сумма. Priority — последний активный источник целиком.")]
    public InputMixMode mixMode = InputMixMode.MaxAbs;
    public bool enableKeyboard = true;
    public bool enableJoystick = true;
    public bool enableUdp = true;

    [Header("Клавиатура")]
    [Range(0.5f, 10f)] public float kbAttack = 4.5f;
    [Range(0.5f, 15f)] public float kbRelease = 7f;
    [Range(0.1f, 2f)] public float kbThrottleSpeed = 0.7f;
    [Range(0.1f, 1f)] public float kbFineFactor = 0.35f;

    [Header("Ощущение стиков")]
    [Range(0f, 0.2f)] public float deadzone = 0.02f;
    [Range(0f, 1f)] public float expoRollPitch = 0.25f;
    [Range(0f, 1f)] public float expoYaw = 0.2f;
    [Range(0f, 1f)] public float throttleExpo = 0.15f;
    [Range(0.2f, 3f)] public float rcRate = 1.0f;
    [Range(0f, 0.95f)] public float superRate = 0.7f;
    [Range(0.2f, 3f)] public float yawRcRate = 1.0f;
    [Range(0f, 0.95f)] public float yawSuperRate = 0.6f;

    [Header("Джойстик SM001")]
    public int deviceIndex = 0;
    public CalibrationData cal = new CalibrationData();
    [Tooltip("Печатать список осей/кнопок при подключении устройства в Console")]
    public bool logDeviceOnConnect = true;

    [Header("Failsafe")]
    public bool failsafeOnUdpLost = true;
    public bool failsafeOnJoyLost = true;

    public ControlFrame Sticks { get; private set; } = ControlFrame.Neutral;
    public float[] Mixed = new float[Channels];
    public float[] JoyCal = new float[Channels];
    public InputSourceKind[] ActiveSrc = new InputSourceKind[Channels];
    public bool Failsafe { get; private set; }
    public float KbThr01 => kbThr;
    public UdpLink udp;

    public List<string> DeviceNames = new List<string>();
    public List<string> AxisLabels = new List<string>();
    public List<float> AxisValues = new List<float>();
    public List<bool> ButtonValues = new List<bool>();
    public bool JoyConnected => joyDevice != null;
    public string DeviceLayout => joyDevice != null ? joyDevice.layout : "none";
    public CalWizardState Wizard { get; private set; } = CalWizardState.Off;

    float kbThr; float[] kbAxes = new float[4];
    float lastKbActive, lastJoyActive, lastUdpActive, lastJoyDominant;
    bool udpEverFresh, joyEverActive;
    bool armFlag, disarmFlag, resetFlag, resetFullFlag, modeFlag, camFlag;
    float refreshTimer;
#if ENABLE_INPUT_SYSTEM
    InputDevice joyDevice;
    readonly List<InputControl> axisCtrls = new List<InputControl>();
    readonly List<ButtonControl> buttons = new List<ButtonControl>();
#endif
    string CalPath => Path.Combine(Application.persistentDataPath, "sm001_calibration.json");

    void Awake()
    {
        if (udp == null) udp = GetComponent<UdpLink>();
        LoadCal();
#if !ENABLE_INPUT_SYSTEM
        Debug.LogWarning("[Input] Пакет Input System не активен — джойстик недоступен.");
#endif
    }

    void Update()
    {
        float dt = Time.unscaledDeltaTime;
        refreshTimer -= dt;
        if (refreshTimer <= 0f) { RefreshDevices(); refreshTimer = 0.5f; }
        ReadJoystick();
        ReadKeyboard(dt);
        if (udp != null) udp.Poll();

        if (Hotkeys.Down(SimKey.Arm)) armFlag = true;
        if (Hotkeys.Down(SimKey.Disarm)) disarmFlag = true;
        if (Hotkeys.Down(SimKey.Reset)) resetFlag = true;
        if (Hotkeys.Down(SimKey.ResetBattery)) resetFullFlag = true;
        if (Hotkeys.Down(SimKey.Mode)) modeFlag = true;
        if (Hotkeys.Down(SimKey.Camera)) camFlag = true;

        Mix();
        UpdateFailsafe();
        if (Wizard == CalWizardState.Extremes) CaptureExtremes();
    }

    // ---------- устройства ----------
    void RefreshDevices()
    {
#if ENABLE_INPUT_SYSTEM
        DeviceNames.Clear();
        var list = new List<InputDevice>();
        foreach (var d in InputSystem.devices)
            if (d is Joystick || d is HID) { list.Add(d); DeviceNames.Add($"{d.displayName} [{d.layout}]"); }

        deviceIndex = Mathf.Clamp(deviceIndex, 0, Mathf.Max(0, list.Count - 1));
        var dev = list.Count > 0 ? list[deviceIndex] : null;
        if (dev == joyDevice) return;

        joyDevice = dev;
        axisCtrls.Clear(); buttons.Clear(); AxisLabels.Clear();
        if (dev == null) return;

        // Стики (дочерние оси StickControl) ставим первыми, потом прочие аналоговые оси.
        var stickAxes = new List<InputControl>();
        var otherAxes = new List<InputControl>();
        foreach (var c in dev.allControls)
        {
            if (c is ButtonControl b && !(b.parent is StickControl)) { buttons.Add(b); continue; }
            if ((c is AxisControl || c is IntegerControl) && !(c.parent is ButtonControl))
            {
                if (c.parent is StickControl) stickAxes.Add(c); else otherAxes.Add(c);
            }
        }
        foreach (var c in stickAxes) { axisCtrls.Add(c); AxisLabels.Add(c.displayName); }
        foreach (var c in otherAxes) { axisCtrls.Add(c); AxisLabels.Add(c.displayName); }

        if (logDeviceOnConnect)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[Input] Устройство: {dev.displayName}  layout={dev.layout}");
            for (int i = 0; i < axisCtrls.Count; i++) sb.AppendLine($"  ось [{i}] {axisCtrls[i].displayName} ({axisCtrls[i].GetType().Name})");
            sb.AppendLine($"  кнопок: {buttons.Count}");
            Debug.Log(sb.ToString());
        }
#endif
    }

    static float ReadFloat(InputControl c)
    {
        object o = c.ReadValueAsObject();
        if (o is float f) return f;
        if (o is int i) return i;
        if (o is double d) return (float)d;
        if (o is byte by) return by;
        if (o is short sh) return sh;
        return 0f;
    }

    void ReadJoystick()
    {
#if ENABLE_INPUT_SYSTEM
        AxisValues.Clear(); ButtonValues.Clear();
        if (joyDevice == null) { for (int i = 0; i < Channels; i++) JoyCal[i] = 0; return; }
        foreach (var c in axisCtrls) AxisValues.Add(ReadFloat(c));
        foreach (var b in buttons) ButtonValues.Add(b.isPressed);
        for (int i = 0; i < Channels; i++)
        {
            int bi = cal.buttonIndex[i];
            float raw = bi >= 0
                ? (bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f)
                : (cal.sourceIndex[i] < AxisValues.Count ? AxisValues[cal.sourceIndex[i]] : 0f);
            JoyCal[i] = cal.Normalize(i, raw);
        }
        joyEverActive = true;
#endif
    }

    public void CycleDevice() { deviceIndex++; refreshTimer = 0; }
    public float RawAxis(int i) => i >= 0 && i < AxisValues.Count ? AxisValues[i] : 0f;
    public float RawChannel(int ch)
    {
        int bi = cal.buttonIndex[ch];
        if (bi >= 0) return bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f;
        return RawAxis(cal.sourceIndex[ch]);
    }

    // Клик-назначение из монитора F4
    public void AssignAxisToChannel(int axisIndex, int channel)
    {
        cal.sourceIndex[channel] = axisIndex;
        cal.buttonIndex[channel] = -1;
        SaveCal();
    }
    public void AssignButtonToChannel(int buttonIndex, int channel)
    {
        cal.buttonIndex[channel] = buttonIndex;
        SaveCal();
    }

    // Авто Mode 2: правый стик = ROLL/PITCH, левый = THR/YAW
    public void AutoMapSticksMode2()
    {
#if ENABLE_INPUT_SYSTEM
        if (joyDevice == null) { Debug.LogWarning("[Input] Джойстик не подключен."); return; }
        var sticks = new List<StickControl>();
        foreach (var c in joyDevice.allControls) if (c is StickControl s) sticks.Add(s);
        if (sticks.Count >= 2)
        {
            int lx = axisCtrls.IndexOf(sticks[0].x), ly = axisCtrls.IndexOf(sticks[0].y);
            int rx = axisCtrls.IndexOf(sticks[1].x), ry = axisCtrls.IndexOf(sticks[1].y);
            if (lx >= 0 && ly >= 0 && rx >= 0 && ry >= 0)
            {
                cal.sourceIndex[0] = rx; cal.sourceIndex[1] = ry;
                cal.sourceIndex[2] = ly; cal.sourceIndex[3] = lx;
                for (int i = 0; i < 4; i++) cal.buttonIndex[i] = -1;
                SaveCal();
                Debug.Log($"[Input] Авто Mode2: ROLL=ось{rx} PITCH=ось{ry} THR=ось{ly} YAW=ось{lx}");
                return;
            }
        }
        Debug.Log("[Input] Два StickControl не найдены (generic HID). Назначьте оси вручную кнопками R/P/T/Y в F4.");
#endif
    }

    // ---------- клавиатура / микс / failsafe / калибровка (без изменений) ----------
    void ReadKeyboard(float dt)
    {
        if (!enableKeyboard) { for (int i = 0; i < 4; i++) kbAxes[i] = 0; return; }
        float fine = Hotkeys.Held(SimKey.Fine) ? kbFineFactor : 1f;
        float[] target =
        {
            (Hotkeys.Held(SimKey.RollR) ? 1 : 0) - (Hotkeys.Held(SimKey.RollL) ? 1 : 0),
            (Hotkeys.Held(SimKey.PitchFwd) ? 1 : 0) - (Hotkeys.Held(SimKey.PitchBack) ? 1 : 0),
            0,
            (Hotkeys.Held(SimKey.YawR) ? 1 : 0) - (Hotkeys.Held(SimKey.YawL) ? 1 : 0)
        };
        for (int i = 0; i < 4; i++)
        {
            if (i == 2) continue;
            float t = target[i] * fine;
            kbAxes[i] = Mathf.MoveTowards(kbAxes[i], t, (Mathf.Abs(t) > 0.01f ? kbAttack : kbRelease) * dt);
        }
        if (Hotkeys.Held(SimKey.ThrUp)) kbThr += kbThrottleSpeed * fine * dt;
        if (Hotkeys.Held(SimKey.ThrDn)) kbThr -= kbThrottleSpeed * fine * dt;
        kbThr = Mathf.Clamp01(kbThr);
    }
    public void ResetKeyboardThrottle() { kbThr = 0; Array.Clear(kbAxes, 0, 4); }

    void Mix()
    {
        float t = Time.realtimeSinceStartup;
        float[] kb = { kbAxes[0], kbAxes[1], kbThr * 2f - 1f, kbAxes[3], 0, 0 };
        float[] joy = JoyCal;
        ControlFrame u = udp != null ? udp.Control : ControlFrame.Neutral;
        float[] net = { u.roll, u.pitch, u.throttle, u.yaw, u.aux1, u.aux2 };

        bool kbOn = enableKeyboard, joyOn = enableJoystick && JoyConnected, udpOn = enableUdp && udp != null && udp.Fresh;
        if (AnyActive(kb, 0.05f) || kbThr > 0.01f) lastKbActive = t;
        if (joyOn && AnyActive(joy, 0.05f)) lastJoyActive = t;
        if (udpOn) { lastUdpActive = t; udpEverFresh = true; }

        int prio = 0;
        if (lastJoyActive >= lastKbActive && lastJoyActive >= lastUdpActive) prio = 1;
        if (lastUdpActive >= lastKbActive && lastUdpActive >= lastJoyActive) prio = 2;

        for (int ch = 0; ch < Channels; ch++)
        {
            float v = 0; InputSourceKind src = InputSourceKind.None;
            float kv = kbOn ? kb[ch] : 0, jv = joyOn ? joy[ch] : 0, uv = udpOn ? net[ch] : 0;
            if (mixMode == InputMixMode.Priority)
            {
                v = prio == 2 ? uv : prio == 1 ? jv : kv;
                src = prio == 2 ? InputSourceKind.Udp : prio == 1 ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
            }
            else if (mixMode == InputMixMode.Sum)
            {
                v = Mathf.Clamp(kv + jv + uv, -1, 1);
                src = Mathf.Abs(uv) >= Mathf.Abs(jv) && Mathf.Abs(uv) >= Mathf.Abs(kv) ? InputSourceKind.Udp
                    : Mathf.Abs(jv) >= Mathf.Abs(kv) ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
            }
            else
            {
                if (ch == 2)
                {
                    v = -1;
                    if (kbOn && kv > v) { v = kv; src = InputSourceKind.Keyboard; }
                    if (joyOn && jv > v) { v = jv; src = InputSourceKind.Joystick; }
                    if (udpOn && uv > v) { v = uv; src = InputSourceKind.Udp; }
                }
                else
                {
                    if (Mathf.Abs(kv) > Mathf.Abs(v)) { v = kv; src = InputSourceKind.Keyboard; }
                    if (Mathf.Abs(jv) > Mathf.Abs(v)) { v = jv; src = InputSourceKind.Joystick; }
                    if (Mathf.Abs(uv) > Mathf.Abs(v)) { v = uv; src = InputSourceKind.Udp; }
                }
            }
            Mixed[ch] = v; ActiveSrc[ch] = src;
            if (src == InputSourceKind.Joystick && Mathf.Abs(v) > 0.05f) lastJoyDominant = t;
        }

        var s = ControlFrame.Neutral;
        s.roll = SimMath.Expo(SimMath.Deadzone(Mixed[0], deadzone), expoRollPitch);
        s.pitch = SimMath.Expo(SimMath.Deadzone(Mixed[1], deadzone), expoRollPitch);
        s.yaw = SimMath.Expo(SimMath.Deadzone(Mixed[3], deadzone), expoYaw);
        s.throttle = SimMath.Expo(Mixed[2], throttleExpo);
        s.aux1 = Mixed[4]; s.aux2 = Mixed[5];
        Sticks = s;
    }
    static bool AnyActive(float[] a, float thr)
    { for (int i = 0; i < a.Length; i++) if (i != 2 && Mathf.Abs(a[i]) > thr) return true; return false; }

    void UpdateFailsafe()
    {
        bool udpLost = failsafeOnUdpLost && enableUdp && udpEverFresh && udp != null && !udp.Fresh;
        bool joyLost = failsafeOnJoyLost && enableJoystick && joyEverActive && !JoyConnected &&
                       Time.realtimeSinceStartup - lastJoyDominant < 3f;
        Failsafe = udpLost || joyLost;
    }

    public bool ConsumeArm() { bool f = armFlag; armFlag = false; return f; }
    public bool ConsumeDisarm() { bool f = disarmFlag; disarmFlag = false; return f; }
    public bool ConsumeReset(out bool full) { full = resetFullFlag; bool f = resetFlag || resetFullFlag; resetFlag = resetFullFlag = false; return f; }
    public bool ConsumeModeCycle() { bool f = modeFlag; modeFlag = false; return f; }
    public bool ConsumeCameraCycle() { bool f = camFlag; camFlag = false; return f; }

    public void BeginCal() { Wizard = CalWizardState.Intro; }
    public void CalCancel() { Wizard = CalWizardState.Off; LoadCal(); }
    public void CalNext()
    {
        switch (Wizard)
        {
            case CalWizardState.Intro: Wizard = CalWizardState.Centers; break;
            case CalWizardState.Centers:
                for (int i = 0; i < Channels; i++) { float r = RawChannel(i); cal.center[i] = r; cal.min[i] = r; cal.max[i] = r; }
                Wizard = CalWizardState.Extremes; break;
            case CalWizardState.Extremes: Wizard = CalWizardState.Verify; break;
            case CalWizardState.Verify: SaveCal(); Wizard = CalWizardState.Off; break;
        }
    }
    void CaptureExtremes()
    {
        for (int i = 0; i < Channels; i++) { float r = RawChannel(i); cal.min[i] = Mathf.Min(cal.min[i], r); cal.max[i] = Mathf.Max(cal.max[i], r); }
    }
    public void ResetCal() { cal = new CalibrationData(); SaveCal(); }
    public void SaveCal() { try { File.WriteAllText(CalPath, JsonUtility.ToJson(cal, true)); } catch (Exception e) { Debug.LogWarning(e.Message); } }
    public void LoadCal()
    {
        try { if (File.Exists(CalPath)) cal = JsonUtility.FromJson<CalibrationData>(File.ReadAllText(CalPath)); }
        catch (Exception e) { Debug.LogWarning(e.Message); }
        if (cal == null || cal.sourceIndex == null || cal.sourceIndex.Length != Channels) cal = new CalibrationData();
    }
}
```

> Если Unity ругнётся на `using UnityEngine.InputSystem.HID;` — убери эту строку и замени `d is HID` на `d.layout == "HID"` (смысл тот же).

---

## 3. Замена метода `WinInput` в `SimHud.cs` (кликабельный маппинг)

```csharp
    void WinInput(int id)
    {
        if (inp == null) return;
        scrollInp = GUILayout.BeginScrollView(scrollInp);
        GUILayout.Label("Микс источников (одновременная работа):");
        GUILayout.BeginHorizontal();
        foreach (InputMixMode m in System.Enum.GetValues(typeof(InputMixMode)))
        {
            GUI.color = inp.mixMode == m ? Cacc : Color.white;
            if (GUILayout.Button(m.ToString())) inp.mixMode = m;
        }
        GUI.color = Color.white;
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        inp.enableKeyboard = GUILayout.Toggle(inp.enableKeyboard, "Клавиатура");
        inp.enableJoystick = GUILayout.Toggle(inp.enableJoystick, "Джойстик");
        inp.enableUdp = GUILayout.Toggle(inp.enableUdp, "UDP");
        GUILayout.EndHorizontal();
        GUILayout.Space(6);
        GUILayout.Label($"Джойстик: {(inp.JoyConnected ? "подключен" : "нет")}  layout={inp.DeviceLayout}");
        if (GUILayout.Button("Сменить устройство")) inp.CycleDevice();
        if (GUILayout.Button("Авто-назначить стики (Mode 2)")) inp.AutoMapSticksMode2();
        GUILayout.Space(4);
        GUILayout.Label("Сырые оси: двинь стик → строка меняется → нажми канал справа (R/P/T/Y):");
        for (int i = 0; i < inp.AxisLabels.Count && i < 16; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"[{i}] {inp.AxisLabels[i]}", GUILayout.Width(170));
            GUILayout.Label(inp.AxisValues[i].ToString("0.000"), GUILayout.Width(60));
            GUILayout.Box("", GUILayout.Width(Mathf.Abs(inp.AxisValues[i]) * 60 + 1), GUILayout.Height(10));
            if (GUILayout.Button("R", GUILayout.Width(22))) inp.AssignAxisToChannel(i, 0);
            if (GUILayout.Button("P", GUILayout.Width(22))) inp.AssignAxisToChannel(i, 1);
            if (GUILayout.Button("T", GUILayout.Width(22))) inp.AssignAxisToChannel(i, 2);
            if (GUILayout.Button("Y", GUILayout.Width(22))) inp.AssignAxisToChannel(i, 3);
            GUILayout.EndHorizontal();
        }
        GUILayout.Label($"Кнопок: {inp.ButtonValues.Count} (назначь SwA на AUX2 через «кн +» ниже)");
        GUILayout.Space(6);
        GUILayout.Label("Текущий маппинг каналов:");
        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(DroneInputManager.ChannelNames[i], GUILayout.Width(90));
            if (GUILayout.Button("-", GUILayout.Width(24))) inp.cal.sourceIndex[i] = Mathf.Max(0, inp.cal.sourceIndex[i] - 1);
            GUILayout.Label("ось " + inp.cal.sourceIndex[i], GUILayout.Width(50));
            if (GUILayout.Button("+", GUILayout.Width(24))) inp.cal.sourceIndex[i]++;
            if (GUILayout.Button("-", GUILayout.Width(24))) inp.cal.buttonIndex[i] = Mathf.Max(-1, inp.cal.buttonIndex[i] - 1);
            GUILayout.Label("кн " + inp.cal.buttonIndex[i], GUILayout.Width(46));
            if (GUILayout.Button("+", GUILayout.Width(24))) inp.cal.buttonIndex[i]++;
            inp.cal.invert[i] = GUILayout.Toggle(inp.cal.invert[i], "inv", GUILayout.Width(44));
            GUILayout.Label(inp.JoyCal[i].ToString("0.00"), GUILayout.Width(50));
            GUILayout.EndHorizontal();
        }
        if (GUILayout.Button("Сохранить маппинг")) inp.SaveCal();
        if (GUILayout.Button("Сбросить калибровку к умолчанию")) inp.ResetCal();
        GUILayout.Space(6);
        inp.deadzone = Slider("Deadzone", inp.deadzone, 0, 0.2f);
        inp.expoRollPitch = Slider("Expo R/P", inp.expoRollPitch, 0, 1);
        inp.expoYaw = Slider("Expo Yaw", inp.expoYaw, 0, 1);
        inp.throttleExpo = Slider("Expo Thr", inp.throttleExpo, 0, 1);
        if (udp != null)
        {
            GUILayout.Space(6);
            GUILayout.Label($"UDP :{udp.listenPort} → {udp.remoteHost}:{udp.remotePort}  fresh={udp.Fresh}");
            GUILayout.Label($"rx {udp.RxCount}  tx {udp.TxCount}  err: {udp.LastError}");
        }
        GUILayout.EndScrollView();
        GUI.DragWindow();
    }
```

---

## 4. Как назначить правильно (порядок действий)

1. Play → **F4**. Нажми **«Авто-назначить стики (Mode 2)»** — если Unity видит два стика, всё встанет само.
2. Если не помогло: двинь **правый стик вправо** → в списке осей найдёшь строку, где значение поехало → нажми в ней **R**. Двинь **правый стик вверх** → **P**. **Левый стик вверх** (газ) → **T**. **Левый стик влево/вправо** → **Y**.
3. Проверь знак: газ вниз должен давать `THR = -1`, вверх `+1`; правый стик вправо `ROLL = +1`. Если наоборот — поставь галочку **inv** в строке канала.
4. Тумблер SwA: в строке **AUX2(SwA)** кнопкой «кн +» выставь индекс кнопки, которая меняется при щелчке тумблера (видишь в Console/по поведению). VrA — это аналоговая ось, назначь её на AUX1 тем же кликом по оси (кнопки R/P/T/Y только для стиков; для AUX1 используй «ось +/−» в строке канала).
5. **F2** → пройди калибровку заново (центр → полные диапазоны → сохранить). После的正确ного маппинга калибровка встанет ровно.
6. «Сохранить маппинг» — всё пишется в JSON, при следующем запуске подхватится.

---

## 5. Если сырые оси вообще не двигаются

- Проверь `joy.cpl` в Windows (см. п.1). Если там тоже мертво — переключи **режим USB** на аппаратуре/донгле (у SM001/FlySky бывает несколько USB-режимов: HID-joystick vs simulator dongle), переподключи, попробуй другой порт.
- Пришли мне текст из Console, который новый код печатает при подключении (`[Input] Устройство: ... layout=... ось [i] ...`) — по нему я точно скажу, какие индексы соответствуют стикам твоего экземпляра SM001, и дам готовый маппинг без угадывания.

В 90% случаев проблема именно в индексах осей (стики «живут» под другими номерами, чем дефолтные 0–3), и кликабельный маппинг в F4 закрывает её за минуту.

### USER
возникли ошибки помоги исправить:  1 Assets\DroneBreakup.cs(245,38): error CS0104: 'Random' is an ambiguous reference between 'UnityEngine.Random' and 'System.Random'

2  Assets\DroneBreakup.cs(283,13): error CS0104: 'Debug' is an ambiguous reference between 'UnityEngine.Debug' and 'System.Diagnostics.Debug'



### ASSISTANT
Это конфликт имён: в файл попали `using System;` и/или `using System.Diagnostics;` (часто их автоматически добавляет IDE при автоисправлении), а в Unity есть свои `Random` и `Debug`. Компилятор не знает, какой из них брать.

## Исправление (одна правка в шапке файла)

Открой `DroneBreakup.cs` и приведи **самые первые строки** к такому виду — добавлены два alias-а в конце:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;   // убирает конфликт с System.Random
using Debug = UnityEngine.Debug;     // убирает конфликт с System.Diagnostics.Debug
```

Alias имеет приоритет над обычными `using`, поэтому остальные строки (`using System;`, `using System.Diagnostics;`, если они там есть) можно оставить — ошибки CS0104 уйдут, и все `Random.onUnitSphere`, `Random.insideUnitSphere`, `Debug.Log` в файле снова будут указывать на Unity-версии. Больше ничего в коде менять не нужно.

## Альтернатива (если хочешь без alias)

Просто удали из шапки `DroneBreakup.cs` строки:

```csharp
using System;                 // удалить, если есть
using System.Diagnostics;     // удалить, если есть
```

В моём исходном коде они не использовались — там везде стоит полностью квалифицированное `System.StringComparison.OrdinalIgnoreCase`, так что удаление безопасно. Но вариант с alias надёжнее: IDE не сможет снова «сломать» файл автоимпортом.

## На будущее

Если такие же ошибки CS0104 появятся в других файлах симулятора (`Random`, `Debug`, иногда `Object` — конфликт с `System.Object`), лечится тем же способом — alias-строкой в шапке:

```csharp
using Object = UnityEngine.Object;
```

Сохрани файл, дождись рекомпиляции — и можно снова тестировать развал дрона на скорости. Если после этого вылезет что-то ещё (например, по `PhysicMaterial` в Unity 6.3) — присылай, поправлю.

### USER
я поправил шапку скрипта но все равно ошибка остается? Assets\DroneSimEditorTools.cs(18,24): error CS0104: 'Object' is an ambiguous reference between 'UnityEngine.Object' and 'object'



### ASSISTANT
Да, это тот же класс проблемы, просто теперь в редакторном файле: IDE добавила `using System;`, и `Object` стал неоднозначным (`UnityEngine.Object` vs `System.Object`). Лечится так же — alias-ом.

## Исправление

Приведи шапку `Assets/Editor/DroneSimEditorTools.cs` к такому виду:

```csharp
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;   // убирает конфликт UnityEngine.Object / System.Object
using Debug = UnityEngine.Debug;     // на случай, если IDE потом добавит System.Diagnostics
```

Этого достаточно — все `Object.FindObjectOfType<...>()` в файле (строка 18 и остальные в `RebuildDrone`, `SavePrefab`, `AddBreakup` и т.д.) снова будут указывать на Unity-версию. Больше ничего менять не нужно.

## Универсальный набор «от всех таких ошибок»

Если хочешь закрыть тему заранее, добавь в шапку **каждого** файла симулятора (после обычных `using`, отдельными строками) вот эти три alias-а:

```csharp
using Random = UnityEngine.Random;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;
```

Они безвредны, даже если конфликта нет, и не дают IDE «сломать» файл автоимпортом `using System;` / `using System.Diagnostics;`. Особенно актуально для `DroneBreakup.cs`, `DroneSceneBuilder.cs`, `SimHud.cs`, `DroneSimEditorTools.cs` — там чаще всего случаются автоимпорты.

## Альтернатива

Просто удали из шапки `DroneSimEditorTools.cs` строку `using System;`, если она там есть — в моём исходном коде она не нужна (там нет типов из корня `System`, всё либо UnityEngine/UnityEditor, либо полностью квалифицировано). Но alias надёжнее, так что рекомендую первый вариант.

Сохрани, дождись рекомпиляции — иerrors CS0104 должны исчезнуть во всём проекте. Если вылезет ещё что-то — присылай, добьём.

### USER
я внес исправления в скрипт по джостику теперь заработали стики в вертикальном направлении а в горизонтальном направлении оба стика вообще никак не реагируют в режиме F4

### ASSISTANT
Да, это очень показательный симптом: **вертикальные оси читаются, а горизонтальные обоих стиков — нет**. Значит, горизонтальные каналы SM001 приходят в Unity как **другой тип контрола** (часто это `Vector2`/`HatSwitch`/HID-int, либо ось «спрятана» под кнопкой), а наш список осей их просто не видел. Ниже патч, который собирает **все** аналоговые контролы любого типа, показывает их в F4 и даёт назначить кликом. Заодно добавим live-просмотр кнопок — если горизонтальные стики окажутся POV-кнопками, мы это сразу увидим.

---

## Патч 1 — поля джойстика в `DroneInputManager.cs`

Найди блок:

```csharp
#if ENABLE_INPUT_SYSTEM
    InputDevice joyDevice;
    readonly List<InputControl> axisCtrls = new List<InputControl>();
    readonly List<ButtonControl> buttons = new List<ButtonControl>();
#endif
```

Замени на:

```csharp
#if ENABLE_INPUT_SYSTEM
    InputDevice joyDevice;
    class AxisEntry { public InputControl ctrl; public int comp; public string label; } // comp: -1 scalar, 0=x, 1=y
    readonly List<AxisEntry> axisEntries = new List<AxisEntry>();
    readonly List<ButtonControl> buttons = new List<ButtonControl>();
#endif
```

---

## Патч 2 — замени `RefreshDevices()` целиком

```csharp
    void RefreshDevices()
    {
#if ENABLE_INPUT_SYSTEM
        DeviceNames.Clear();
        var list = new List<InputDevice>();
        foreach (var d in InputSystem.devices)
            if (d is Joystick || d is HID) { list.Add(d); DeviceNames.Add($"{d.displayName} [{d.layout}]"); }

        deviceIndex = Mathf.Clamp(deviceIndex, 0, Mathf.Max(0, list.Count - 1));
        var dev = list.Count > 0 ? list[deviceIndex] : null;
        if (dev == joyDevice) return;

        joyDevice = dev;
        axisEntries.Clear(); buttons.Clear(); AxisLabels.Clear();
        if (dev == null) return;

        var stick = new List<AxisEntry>();
        var other = new List<AxisEntry>();
        var extra = new List<AxisEntry>();
        var scalar = new HashSet<InputControl>();

        // 1) Все скалярные оси без фильтра по родителю (раньше часть осей могла отсеиваться)
        foreach (var c in dev.allControls)
        {
            if (c is ButtonControl b && !(b.parent is StickControl)) buttons.Add(b);
            if (c is AxisControl || c is IntegerControl)
            {
                var e = new AxisEntry { ctrl = c, comp = -1, label = $"{c.displayName} <{c.GetType().Name}>" };
                if (c.parent is StickControl) stick.Add(e); else other.Add(e);
                scalar.Add(c);
            }
        }
        // 2) Виртуальные оси из Vector2 / HatSwitch, если их дети не попали в скаляры
        foreach (var c in dev.allControls)
        {
            if (c is Vector2Control v2)
            {
                if (!scalar.Contains(v2.x)) extra.Add(new AxisEntry { ctrl = v2, comp = 0, label = c.displayName + ".x <vec2>" });
                if (!scalar.Contains(v2.y)) extra.Add(new AxisEntry { ctrl = v2, comp = 1, label = c.displayName + ".y <vec2>" });
            }
            else if (c is HatSwitchControl hat)
            {
                extra.Add(new AxisEntry { ctrl = hat, comp = 0, label = c.displayName + ".x <hat>" });
                extra.Add(new AxisEntry { ctrl = hat, comp = 1, label = c.displayName + ".y <hat>" });
            }
        }
        foreach (var e in stick) axisEntries.Add(e);
        foreach (var e in other) axisEntries.Add(e);
        foreach (var e in extra) axisEntries.Add(e);
        foreach (var e in axisEntries) AxisLabels.Add(e.label);

        if (logDeviceOnConnect) DumpAllControls();
#endif
    }
```

---

## Патч 3 — замени `ReadFloat(...)` и `ReadJoystick()` на это

```csharp
    static float ToFloat(object o)
    {
        if (o is float f) return f;
        if (o is int i) return i;
        if (o is double d) return (float)d;
        if (o is byte b) return b;
        if (o is short s) return s;
        if (o is uint ui) return ui;
        return 0f;
    }

#if ENABLE_INPUT_SYSTEM
    static float ReadEntry(AxisEntry e)
    {
        object o = e.ctrl.ReadValueAsObject();
        if (e.comp >= 0 && o is Vector2 v) return e.comp == 0 ? v.x : v.y;
        return ToFloat(o);
    }
#endif

    void ReadJoystick()
    {
#if ENABLE_INPUT_SYSTEM
        AxisValues.Clear(); ButtonValues.Clear();
        if (joyDevice == null) { for (int i = 0; i < Channels; i++) JoyCal[i] = 0; return; }
        foreach (var e in axisEntries) AxisValues.Add(ReadEntry(e));
        foreach (var b in buttons) ButtonValues.Add(b.isPressed);
        for (int i = 0; i < Channels; i++)
        {
            int bi = cal.buttonIndex[i];
            float raw = bi >= 0
                ? (bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f)
                : (cal.sourceIndex[i] < AxisValues.Count ? AxisValues[cal.sourceIndex[i]] : 0f);
            JoyCal[i] = cal.Normalize(i, raw);
        }
        joyEverActive = true;
#endif
    }
```

---

## Патч 4 — замени `AutoMapSticksMode2()` и добавь два новых метода

```csharp
    public void AutoMapSticksMode2()
    {
#if ENABLE_INPUT_SYSTEM
        if (joyDevice == null) { UnityEngine.Debug.LogWarning("[Input] Джойстик не подключен."); return; }
        var sticks = new List<StickControl>();
        foreach (var c in joyDevice.allControls) if (c is StickControl s) sticks.Add(s);
        if (sticks.Count >= 2)
        {
            int lx = FindAxis(sticks[0].x), ly = FindAxis(sticks[0].y);
            int rx = FindAxis(sticks[1].x), ry = FindAxis(sticks[1].y);
            if (lx >= 0 && ly >= 0 && rx >= 0 && ry >= 0)
            {
                cal.sourceIndex[0] = rx; cal.sourceIndex[1] = ry;
                cal.sourceIndex[2] = ly; cal.sourceIndex[3] = lx;
                for (int i = 0; i < 4; i++) cal.buttonIndex[i] = -1;
                SaveCal();
                UnityEngine.Debug.Log($"[Input] Авто Mode2: ROLL=ось{rx} PITCH=ось{ry} THR=ось{ly} YAW=ось{lx}");
                return;
            }
        }
        UnityEngine.Debug.Log("[Input] Два StickControl не найдены. Назначьте оси вручную кнопками R/P/T/Y в F4.");
#endif
    }

#if ENABLE_INPUT_SYSTEM
    int FindAxis(InputControl c)
    {
        for (int i = 0; i < axisEntries.Count; i++) if (axisEntries[i].ctrl == c) return i;
        return -1;
    }
#endif

    // Печатает в Console вообще все контролы устройства — если что-то не читается, пришли мне этот лог
    public void DumpAllControls()
    {
#if ENABLE_INPUT_SYSTEM
        if (joyDevice == null) { UnityEngine.Debug.Log("[Input] Устройство не подключено."); return; }
        var sb = new System.Text.StringBuilder();
        sb.AppendLine($"[Input] ALL CONTROLS {joyDevice.displayName} layout={joyDevice.layout}");
        foreach (var c in joyDevice.allControls)
            sb.AppendLine($"{c.path} | {c.GetType().Name} | {c.ReadValueAsObject()}");
        UnityEngine.Debug.Log(sb.ToString());
#endif
    }
```

---

## Патч 5 — в `SimHud.cs` (окно F4)

**5a.** Найди строку с баром оси (в `WinInput`) и замени её, чтобы HID-int значения не растягивали окно на весь экран:

```csharp
            GUILayout.Box("", GUILayout.Width(Mathf.Clamp(Mathf.Abs(inp.AxisValues[i]) * 60f, 1f, 60f)), GUILayout.Height(10));
```

**5b.** Найди строку `GUILayout.Label($"Кнопок: {inp.ButtonValues.Count} ...");` и замени её на блок:

```csharp
        GUILayout.Label($"Кнопок: {inp.ButtonValues.Count} (двигай стики/тумблеры — смотри, что нажимается):");
        for (int i = 0; i < inp.ButtonValues.Count && i < 32; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"[{i}] {(inp.ButtonValues[i] ? "ON" : "off")}", GUILayout.Width(90));
            if (GUILayout.Button("→AUX1", GUILayout.Width(60))) inp.AssignButtonToChannel(i, 4);
            if (GUILayout.Button("→AUX2", GUILayout.Width(60))) inp.AssignButtonToChannel(i, 5);
            GUILayout.EndHorizontal();
        }
        if (GUILayout.Button("Показать ВСЕ контролы устройства в Console")) inp.DumpAllControls();
```

---

## После патча — порядок действий

1. Play → **F4**. Список осей теперь содержит типы (`<AxisControl>`, `<vec2>`, `<hat>`, `<IntegerControl>`). Индексы могли сдвинуться, поэтому **переназначь заново кликом**: двинь правый стик вправо → в строке, где поехало значение, нажми **R**; правый вверх → **P**; левый вверх (газ) → **T**; левый влево/вправо → **Y**. Знак поправь галочкой **inv**.
2. Подвигай горизонтально и посмотри секцию **кнопок**: если при горизонтальном движении загораются `ON` у каких-то кнопок — значит, в текущем USB-режиме аппаратура отдаёт горизонтальные оси как POV/кнопки (аналогового значения там нет). Тогда либо переключи **режим USB** на SM001 (HID-joystick / simulator), либо используй их как дискретные каналы.
3. Если горизонтальные по-прежнему нигде не появляются — нажми **«Показать ВСЕ контролы устройства в Console»** и пришли мне этот лог. По нему я точно скажу, под каким именем/типом живут горизонтальные оси твоего экземпляра SM001, и дам готовый маппинг без угадывания.
4. Заодно проверь `Win+R` → `joy.cpl` → Свойства: если там горизонтальные оси тоже мертвы — проблема в режиме USB/драйвере аппаратуры, и Unity тут ни при чём.

В большинстве случаев после этого патча горизонтальные оси находятся либо как `<vec2>`/`<hat>` в конце списка, либо как ранее отфильтрованный `AxisControl`, и назначаются кликом за минуту.

### USER
выходит ошибка: Assets\DroneInputManager.cs(150,27): error CS0246: The type or namespace name 'HatSwitchControl' could not be found (are you missing a using directive or an assembly reference?)



### ASSISTANT
`HatSwitchControl` отсутствует в вашей версии пакета Input System (в разных версиях hat switch представлен как `DpadControl`/`Vector2Control` либо вообще другим типом). Поэтому вместо жёсткой привязки к классу сделаем **универсальную проверку по типу значения**: любой контрол, возвращающий `Vector2`, разкладывается на две оси. Работает во всех версиях пакета.

## Исправление

В `DroneInputManager.cs` внутри `RefreshDevices()` найди **второй цикл** (тот, что с комментарием «Виртуальные оси из Vector2 / HatSwitch…») и замени его целиком на:

```csharp
        // 2) Виртуальные оси из любых контролов со значением Vector2 (стики, D-pad, hat switch)
        foreach (var c in dev.allControls)
        {
            if (c is Vector2Control v2)
            {
                if (v2.x != null && !scalar.Contains(v2.x))
                    extra.Add(new AxisEntry { ctrl = v2, comp = 0, label = c.displayName + ".x <vec2>" });
                if (v2.y != null && !scalar.Contains(v2.y))
                    extra.Add(new AxisEntry { ctrl = v2, comp = 1, label = c.displayName + ".y <vec2>" });
            }
            else if (!(c is ButtonControl) && !(c is AxisControl) && !(c is IntegerControl))
            {
                // универсально: тип класса может отличаться между версиями пакета,
                // поэтому смотрим на фактическое значение контрола
                if (c.ReadValueAsObject() is Vector2)
                {
                    extra.Add(new AxisEntry { ctrl = c, comp = 0, label = c.displayName + ".x <vec2>" });
                    extra.Add(new AxisEntry { ctrl = c, comp = 1, label = c.displayName + ".y <vec2>" });
                }
            }
        }
```

Больше ничего править не нужно — имя `HatSwitchControl` в файле больше нигде не встречается, ошибка CS0246 уйдёт.

## Дальше

После рекомпиляции: Play → **F4** → подвигай горизонтально оба стика и посмотри, появилась ли строка с `<vec2>`, где меняется значение. Если да — назначь её кликом **R** (правый стик) / **Y** (левый стик).

Если горизонтальные по-прежнему нигде не шевелятся — нажми в F4 кнопку **«Показать ВСЕ контролы устройства в Console»** и пришли мне вывод: там будет видно каждое имя контрола, его тип и текущее значение, и я скажу точно, что именно отдаёт ваш SM001 по горизонтали (или подтвердим, что оси не приходят из USB вовсе — тогда дело в режиме USB аппаратуры).

### USER
к сожалению по прежнему горизонтальные оси не реагируют а некоторые углы в стиках управления на основном интерфейсе симулятора сдвигают сразу две оси, можно ли это исправить

### ASSISTANT
Понял картину. Тут две разные причины, и обе лечатся одним решением — **читать аппаратуру напрямую через Windows-джойстик-API (winmm), минуя Input System**:

1. **Горизонтальные оси мертвы** — Unity Input System криво разбирает HID-дескриптор SM001 и просто не создаёт контролы для этих каналов (мы уже перебрали все типы контролов — их там нет).
2. **«Два канала шевелятся от одного стика»** — это нормализация Unity: `StickControl` применяет deadzone/normalize **к вектору целиком**, т.е. пересчитывает X и Y вместе. Плюс наши виртуальные `<vec2>`-записи — это те же самые композиты. Отсюда и связанность по диагонали.

winmm отдаёт **6 независимых сырых осей** (X, Y, Z, R, U, V — ровно 6 каналов SM001) + 32 кнопки + POV, без всякой обработки. Это стандартный путь для RC-аппаратуры в Unity под Windows.

---

## 0. Сначала 30 секунд проверки железа

`Win+R` → `joy.cpl` → Свойства:
- **Если горизонтальные оси там тоже мертвы** — код ни при чём: переключи USB-режим на аппаратуре, попробуй другой кабель/порт, и обязательно **отключи микширование** в модели (Delta/Elevon/V-tail) — именно оно даёт «два канала от одного стика».
- **Если там всё двигается** (обычно так и есть) — значит Unity теряет оси, и патч ниже это чинит.
- Заодно там же: Свойства → Параметры → **Откалибровать** в Windows.

---

## 1. Новый файл `Assets/Scripts/WindowsJoystick.cs`

```csharp
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

// Прямое чтение USB-джойстика через winmm.dll (Windows).
// Обходит layout/нормализацию Unity Input System: 6 сырых осей + 32 кнопки + POV.
public sealed class WindowsJoystickReader
{
    const uint JOY_RETURNALL = 0x000000FF;

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    struct JOYCAPS
    {
        public ushort wMid;
        public ushort wPid;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)] public string szPname;
        public uint wXmin, wXmax, wYmin, wYmax, wZmin, wZmax;
        public uint wNumButtons;
        public uint wPeriodMin, wPeriodMax;
        public uint wRmin, wRmax, wUmin, wUmax, wVmin, wVmax;
        public uint wCaps, wMaxAxes, wNumAxes, wMaxButtons;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)] public string szRegKey;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 260)] public string szOEMVxD;
    }

    [StructLayout(LayoutKind.Sequential)]
    struct JOYINFOEX
    {
        public uint dwSize, dwFlags;
        public uint dwXpos, dwYpos, dwZpos, dwRpos, dwUpos, dwVpos;
        public uint dwButtons, dwButtonNumber, dwPOV, dwReserved1, dwReserved2;
    }

    [DllImport("winmm.dll")] static extern uint joyGetNumDevs();
    [DllImport("winmm.dll", EntryPoint = "joyGetDevCapsW", CharSet = CharSet.Unicode)]
    static extern uint joyGetDevCaps(UIntPtr id, ref JOYCAPS caps, uint size);
    [DllImport("winmm.dll")] static extern uint joyGetPosEx(uint id, ref JOYINFOEX info);

    public sealed class Device
    {
        public int id;
        public string name;
        public int axisCount, buttonCount;
        public readonly float[] axes = new float[8];   // 0..5 = X,Y,Z,R,U,V ; 6,7 = POV X/Y
        public readonly uint[] raw = new uint[8];
        public readonly bool[] buttons = new bool[32];
        public float povDegrees = -1f;
        internal readonly uint[] min = new uint[6];
        internal readonly uint[] max = new uint[6];
    }

    public readonly List<Device> devices = new List<Device>();
    public int index;
    public Device Current => index >= 0 && index < devices.Count ? devices[index] : null;

    public static bool Supported
    {
        get
        {
#if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
            return true;
#else
            return false;
#endif
        }
    }

    public void Refresh()
    {
        devices.Clear();
        if (!Supported) return;
        uint n = joyGetNumDevs();
        for (uint id = 0; id < Math.Min(n, 16u); id++)
        {
            var caps = new JOYCAPS();
            if (joyGetDevCaps((UIntPtr)id, ref caps, (uint)Marshal.SizeOf(typeof(JOYCAPS))) != 0) continue;
            var info = new JOYINFOEX { dwSize = (uint)Marshal.SizeOf(typeof(JOYINFOEX)), dwFlags = JOY_RETURNALL };
            if (joyGetPosEx(id, ref info) != 0) continue;   // не подключён

            var d = new Device
            {
                id = (int)id,
                name = string.IsNullOrEmpty(caps.szPname) ? ("Joystick " + id) : caps.szPname,
                axisCount = (int)caps.wNumAxes,
                buttonCount = (int)caps.wNumButtons
            };
            d.min[0] = caps.wXmin; d.max[0] = caps.wXmax;
            d.min[1] = caps.wYmin; d.max[1] = caps.wYmax;
            d.min[2] = caps.wZmin; d.max[2] = caps.wZmax;
            d.min[3] = caps.wRmin; d.max[3] = caps.wRmax;
            d.min[4] = caps.wUmin; d.max[4] = caps.wUmax;
            d.min[5] = caps.wVmin; d.max[5] = caps.wVmax;
            devices.Add(d);
            Poll(d);
        }
        index = Mathf.Clamp(index, 0, Mathf.Max(0, devices.Count - 1));
    }

    public void PollAll() { foreach (var d in devices) Poll(d); }

    void Poll(Device d)
    {
        var info = new JOYINFOEX { dwSize = (uint)Marshal.SizeOf(typeof(JOYINFOEX)), dwFlags = JOY_RETURNALL };
        if (joyGetPosEx((uint)d.id, ref info) != 0) return;

        d.raw[0] = info.dwXpos; d.raw[1] = info.dwYpos; d.raw[2] = info.dwZpos;
        d.raw[3] = info.dwRpos; d.raw[4] = info.dwUpos; d.raw[5] = info.dwVpos;
        for (int i = 0; i < 6; i++) d.axes[i] = Norm(d.raw[i], d.min[i], d.max[i]);

        if (info.dwPOV >= 36000)   // 0xFFFF = центр
        {
            d.povDegrees = -1f; d.axes[6] = 0f; d.axes[7] = 0f; d.raw[6] = d.raw[7] = 0;
        }
        else
        {
            d.povDegrees = info.dwPOV / 100f;
            float a = d.povDegrees * Mathf.Deg2Rad;
            d.axes[6] = Mathf.Sin(a);   // вправо +
            d.axes[7] = Mathf.Cos(a);   // вверх +
        }
        for (int i = 0; i < 32; i++) d.buttons[i] = (info.dwButtons & (1u << i)) != 0;
    }

    static float Norm(uint v, uint min, uint max)
    {
        if (max <= min) { min = 0; max = 65535; }
        float t = ((float)v - min) / ((float)max - min);
        return Mathf.Clamp(t * 2f - 1f, -1f, 1f);
    }
}
```

---

## 2. Патчи `DroneInputManager.cs` (6 вставок)

**2.1** В самом верху файла, **после `using`-ов и перед** `[DefaultExecutionOrder(-50)]`, добавь:

```csharp
public enum JoystickBackend { Windows, InputSystem }
```

**2.2** Замени блок полей «Джойстик SM001» на:

```csharp
    [Header("Джойстик SM001")]
    [Tooltip("Windows — прямое чтение winmm: 6 сырых осей X/Y/Z/R/U/V без нормализации Unity (рекомендуется для SM001). InputSystem — пакет Unity.")]
    public JoystickBackend backend = JoystickBackend.Windows;
    public int deviceIndex = 0;
    public CalibrationData cal = new CalibrationData();
    [Tooltip("Печатать список осей/кнопок при подключении устройства в Console")]
    public bool logDeviceOnConnect = true;

    readonly WindowsJoystickReader winJoy = new WindowsJoystickReader();
    bool UseWin => backend == JoystickBackend.Windows && WindowsJoystickReader.Supported;
    static readonly string[] WinAxisLabels = { "X", "Y", "Z", "R", "U", "V", "POV X", "POV Y" };
```

**2.3** Замени две строки-свойства:

```csharp
    public bool JoyConnected => UseWin ? winJoy.Current != null : joyDevice != null;
    public string DeviceLayout => UseWin ? "winmm (raw)" : (joyDevice != null ? joyDevice.layout : "none");
```

**2.4** В **начало** `RefreshDevices()` (сразу после открывающей скобки, до `#if ENABLE_INPUT_SYSTEM`) вставь:

```csharp
        if (UseWin)
        {
            winJoy.Refresh();
            DeviceNames.Clear();
            foreach (var d in winJoy.devices) DeviceNames.Add($"{d.name} [winmm #{d.id}, осей {d.axisCount}]");
            deviceIndex = Mathf.Clamp(deviceIndex, 0, Mathf.Max(0, winJoy.devices.Count - 1));
            winJoy.index = deviceIndex;
            AxisLabels.Clear();
            AxisLabels.AddRange(WinAxisLabels);
            return;
        }
```

**2.5** В **начало** `ReadJoystick()` (до `#if ENABLE_INPUT_SYSTEM`) вставь:

```csharp
        if (UseWin)
        {
            AxisValues.Clear(); ButtonValues.Clear();
            winJoy.PollAll();
            var d = winJoy.Current;
            if (d == null) { for (int i = 0; i < Channels; i++) JoyCal[i] = 0; return; }
            for (int i = 0; i < 8; i++) AxisValues.Add(d.axes[i]);
            for (int i = 0; i < 32; i++) ButtonValues.Add(d.buttons[i]);
            for (int ch = 0; ch < Channels; ch++)
            {
                int bi = cal.buttonIndex[ch];
                float raw = bi >= 0
                    ? (bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f)
                    : (cal.sourceIndex[ch] < AxisValues.Count ? AxisValues[cal.sourceIndex[ch]] : 0f);
                JoyCal[ch] = cal.Normalize(ch, raw);
            }
            joyEverActive = true;
            return;
        }
```

**2.6** В **начало** `AutoMapSticksMode2()` и `DumpAllControls()` вставь соответственно:

```csharp
        // в AutoMapSticksMode2():
        if (UseWin)
        {
            for (int i = 0; i < 6; i++) { cal.sourceIndex[i] = i; cal.buttonIndex[i] = -1; }
            SaveCal();
            UnityEngine.Debug.Log("[winmm] Каналы = X,Y,Z,R,U,V. Проверь в F4 и переназначь кликом R/P/T/Y.");
            return;
        }
```

```csharp
        // в DumpAllControls():
        if (UseWin)
        {
            var d = winJoy.Current;
            if (d == null) { UnityEngine.Debug.Log("[winmm] устройство не подключено"); return; }
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[winmm] {d.name} id={d.id} осей={d.axisCount} кнопок={d.buttonCount} POV={d.povDegrees}");
            for (int i = 0; i < 8; i++) sb.AppendLine($"  [{i}] {WinAxisLabels[i]} raw={d.raw[i]} norm={d.axes[i]:0.000}");
            UnityEngine.Debug.Log(sb.ToString());
            return;
        }
```

И добавь рядом с `CycleDevice()` новый метод:

```csharp
    public void ToggleBackend()
    {
        backend = backend == JoystickBackend.Windows ? JoystickBackend.InputSystem : JoystickBackend.Windows;
        refreshTimer = 0f;
    }
```

---

## 3. Патч `SimHud.cs` (окно F4)

Найди строку `GUILayout.Label($"Джойстик: {(inp.JoyConnected ? "подключен" : "нет")}  layout={inp.DeviceLayout}");` и сразу **после неё** вставь:

```csharp
        GUILayout.BeginHorizontal();
        GUILayout.Label("Бэкенд: " + inp.backend, GUILayout.Width(180));
        if (GUILayout.Button("Переключить Windows / InputSystem")) inp.ToggleBackend();
        GUILayout.EndHorizontal();
```

---

## 4. Порядок настройки после патча

1. Play → **F4** → бэкенд должен быть **Windows**, в списке осей теперь ровно `X, Y, Z, R, U, V, POV X, POV Y` — это сырые независимые каналы, никакой связанности.
2. Двигай стики по одному:
   - правый вправо/влево → строка (обычно **X**) → кнопка **R**;
   - правый вверх/вниз → (**Y**) → **P**;
   - левый вверх/вниз (газ) → (**Z** или **R**) → **T**;
   - левый влево/вправо → (**R** или **Z**) → **Y**;
   - крутилка VrA → (**U** или **V**) → в строке AUX1 «ось +/−»;
   - тумблер SwA → если это кнопка, в строке AUX2 выставь «кн N».
3. Знаки поправь галочкой **inv** (газ вниз = −1, вверх = +1; крен вправо = +1).
4. **F2** → калибровка заново (центр → полные диапазоны всех осей и крутилки → сохранить).
5. «Сохранить маппинг».

Каждый канал теперь читает **свою** сырую ось, поэтому «два канала от одного стика» исчезнут, а горизонтальные заработают — winmm отдаёт все 6 каналов SM001 напрямую, без участия Unity-лейаута.

Если после этого в F4 при горизонтальном движении всё равно ни одна из `X/Y/Z/R/U/V` не шевелится — нажми «Показать ВСЕ контролы устройства в Console» (теперь печатает сырые winmm-значения) и пришли вывод плюс то, что показывает `joy.cpl`. Это однозначно отделит проблему аппаратуры/USB-режима от софта.

### USER
сделал эти 6 правок и возникла новая ошибка: Assets\DroneInputManager.cs(324,17): error CS0136: A local or parameter named 'sb' cannot be declared in this scope because that name is used in an enclosing local scope to define a local or parameter



### ASSISTANT
Классический CS0136: в `DumpAllControls()` теперь **две** переменные `sb` — одна внутри блока `if (UseWin) { ... }`, другая ниже в теле метода. C# запрещает повторное использование имени во вложенной и охватывающей областях видимости. Лечится переименованием.

## Исправление

Замени метод `DumpAllControls()` в `DroneInputManager.cs` целиком на этот (переменные разведены: `sbWin` и `sbHid`):

```csharp
    // Печатает в Console все контролы устройства — если что-то не читается, пришли мне этот лог
    public void DumpAllControls()
    {
        if (UseWin)
        {
            var d = winJoy.Current;
            if (d == null) { UnityEngine.Debug.Log("[winmm] устройство не подключено"); return; }
            var sbWin = new System.Text.StringBuilder();
            sbWin.AppendLine($"[winmm] {d.name} id={d.id} осей={d.axisCount} кнопок={d.buttonCount} POV={d.povDegrees}");
            for (int i = 0; i < 8; i++)
                sbWin.AppendLine($"  [{i}] {WinAxisLabels[i]} raw={d.raw[i]} norm={d.axes[i]:0.000}");
            UnityEngine.Debug.Log(sbWin.ToString());
            return;
        }
#if ENABLE_INPUT_SYSTEM
        if (joyDevice == null) { UnityEngine.Debug.Log("[Input] Устройство не подключено."); return; }
        var sbHid = new System.Text.StringBuilder();
        sbHid.AppendLine($"[Input] ALL CONTROLS {joyDevice.displayName} layout={joyDevice.layout}");
        foreach (var c in joyDevice.allControls)
            sbHid.AppendLine($"{c.path} | {c.GetType().Name} | {c.ReadValueAsObject()}");
        UnityEngine.Debug.Log(sbHid.ToString());
#endif
    }
```

Остальные вставки править не нужно — в `AutoMapSticksMode2()` счётчики `i` живут в отдельных `for`-областях и не конфликтуют.

Если вдруг такой же CS0136 вылезет ещё где-то после вставок — правило то же: просто дай переменной во внутреннем блоке уникальное имя (`dWin`, `rawWin` и т.п.).

После рекомпиляции: Play → **F4**, бэкенд **Windows**, и проверь по списку `X, Y, Z, R, U, V` — какие строки шевелятся при движении каждого стика. Если горизонтальные теперь ожили — назначай кликом R/P/T/Y и прогони калибровку F2. Если нет — жми «Показать ВСЕ контролы устройства в Console» и присылай вывод вместе с тем, что показывает `joy.cpl`.

### USER
джостик заработал спасибо!   теперь последний способ управления это Протокол Udp подскажи какие там команды на управление дроном и какие команды на взлет и прочее?

### ASSISTANT
Отлично! UDP у нас работает как «виртуальная аппаратура»: внешнее приложение шлёт каналы + разовые команды, а симулятор в ответ льёт телеметрию.

## Порты

| Направление | Порт | Что |
|---|---|---|
| Приложение → симулятор | **14551** (`listenPort`) | команды управления |
| Симулятор → приложение | **14550** (`remotePort` на `remoteHost`) | телеметрия JSON, 30 Гц |

Настраивается в компоненте `UdpLink` на дроне. Сокет один: слушает 14551 и с него же шлёт телеметрию — приложению достаточно `bind(14550)` и `sendto(..., 14551)`.

---

## 1. Каналы (постоянное управление)

```json
{"channels":[roll, pitch, throttle, yaw, aux1, aux2]}
```

Все значения **−1…+1**. Порядок и смысл:

| # | Канал | −1 | 0 | +1 |
|---|---|---|---|---|
| 0 | ROLL | крен влево | центр | крен вправо |
| 1 | PITCH | нос вверх (назад) | центр | нос вниз (вперёд) |
| 2 | THROTTLE | **газ 0 %** | газ 50 % | газ 100 % |
| 3 | YAW | рыскание влево | центр | рыскание вправо |
| 4 | AUX1 (VrA) | доп. канал | — | — |
| 5 | AUX2 (SwA) | Acro | **не трогать режим** | Angle |

⚠️ **AUX2 держи 0**, если хочешь управлять режимом полем `mode`: в `FlightController` включён `useAuxModeSwitch`, и при `|aux2| > 0.5` он каждый кадр принудительно ставит Acro/Angle, перекрывая твою команду.

Массив может быть длиной 4 (тогда aux = 0). Можно вообще прислать пакет **без** `channels` — тогда применятся только команды.

### Что означают стики в разных режимах

| mode | Название | ROLL / PITCH | YAW | THROTTLE |
|---|---|---|---|---|
| 1 | **Acro** | угловая скорость (до ~670 °/с) | угловая скорость | газ напрямую |
| 2 | **Angle** | наклон до ±45° | угловая скорость | газ напрямую |
| 3 | **AltHold** | наклон до ±45° | угловая скорость | **скорость набора**: центр = держать высоту, ±1 = ±3 м/с |
| 4 | **PosHold** | горизонтальная скорость до ±8 м/с в системе курса | угловая скорость | как AltHold |

Для автономного взлёта/посадки используй **mode 3 (AltHold)** или **4 (PosHold)** — там газ замкнут по высоте.

---

## 2. Разовые команды

| Поле | Значения | Действие |
|---|---|---|
| `arm` | `0` — нет команды, `1` — **ARM**, `2` — **DISARM** | завести/заглушить моторы |
| `mode` | `0` — нет, `1` Acro, `2` Angle, `3` AltHold, `4` PosHold | переключить режим |
| `reset` | `true` | полный сброс: позиция, батарея, ремонт после краша, disarm |

**Важно:** это одноразовые команды. Отправил `arm:1` → получил в телеметрии `armed:true` → верни `arm:0`. Если слать `arm:1`/`mode:3`/`reset:true` в каждом пакете, ты заблокируешь ручное управление с клавиатуры и будешь постоянно ресетить дрон.

### Условия арма (из `FlightController.TryArm`)
- газ **≤ 5 %**, т.е. канал throttle **≤ −0.95** (шли ровно `-1.0`);
- наклон дрона < 30°;
- не `crashed` (после краша сначала `reset:true`);
- батарея не пуста.

---

## 3. Взлёт и посадка — готовая последовательность

```
1. {"reset":true}                       один пакет, потом reset:false
2. {"channels":[0,0,-1,0,0,0],"arm":1}  ждём tel.armed == true → arm:0
3. {"mode":3}                           один пакет → mode:0   (AltHold)
4. channels throttle = +0.6             набор ~1.8 м/с, ждём tel.alt > 3
5. channels throttle =  0.0             висение (высота держится сама)
6. channels yaw = +0.4                  поворот ~55 °/с, потом yaw = 0
7. channels throttle = -0.5             снижение ~1.5 м/с до tel.alt < 0.2
8. channels throttle = -1.0             газ в ноль → автодизарм через 2.5 с
   (или явный {"arm":2})
```

В Acro/Angle газ «висения» ≈ **0.0** (50 %), но точное значение зависит от массы и заряда — там придётся держать высоту самому, поэтому для программного полёта бери AltHold.

---

## 4. Альтернативный формат: CSV PWM

Если удобно мыслить «микросекундами приёмника», можно слать просто строку:

```
1500,1500,1000,1500,1000,1000
```

`1000 → −1`, `1500 → 0`, `2000 → +1`. Допустимо 4 значения (aux подставятся 1500). Команд arm/mode/reset в этом формате нет — только каналы.

---

## 5. Телеметрия (приходит на 14550)

```json
{"t":42.7,"armed":true,"failsafe":false,"crashed":false,"vrs":false,"mode":"AltHold",
 "pos":[1.2,3.05,-4.8],"vel":[0.1,0.0,-0.3],"att":[-1.2,2.4,91.0],"wind":[2.1,0.0,0.4],
 "rpm":[12500,12480,12510,12495],"cmd":[0.48,0.48,0.49,0.48],"thrustG":[295,294,297,295],
 "vbat":15.62,"amps":11.3,"soc":0.87,"usedMah":194.0,
 "alt":3.02,"gs":0.32,"airspeed":2.4,"vs":0.01,"hdg":91.0,"dist":4.95,"ge":1.0}
```

| Поле | Смысл |
|---|---|
| `armed / failsafe / crashed / vrs` | состояние систем (VRS = вихревое кольцо) |
| `mode` | текущий режим строкой |
| `pos`, `vel` | мировые координаты Unity (м) и скорость (м/с) |
| `att` | `[крен +вправо, тангаж +нос вверх, курс 0…360]` (град) |
| `alt`, `vs`, `gs`, `airspeed` | высота AGL, верт. скорость, путевая, воздушная (м/с) |
| `hdg`, `dist` | курс и дистанция до точки дома (м) |
| `rpm`, `cmd`, `thrustG` | по 4 моторам в порядке **FL, FR, BR, BL** |
| `vbat`, `amps`, `soc`, `usedMah` | батарея: В, А, доля 0…1, израсходовано мАч |
| `wind`, `ge` | вектор ветра у дрона и коэффициент экранного эффекта |

---

## 6. Рабочий пример на Python (миссия: взлёт → висение → поворот → посадка)

```python
import socket, json, threading, time

SIM = ("127.0.0.1", 14551)   # куда шлём команды
TEL_PORT = 14550             # где слушаем телеметрию

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", TEL_PORT))
sock.settimeout(0.5)

tel = {}
cmd = {"channels": [0.0, 0.0, -1.0, 0.0, 0.0, 0.0], "arm": 0, "mode": 0, "reset": False}
running = True

def rx():
    while running:
        try:
            data, _ = sock.recvfrom(65535)
            tel.update(json.loads(data.decode()))
        except socket.timeout:
            pass
        except Exception as e:
            print("rx error:", e)

def tx():
    while running:
        sock.sendto(json.dumps(cmd).encode(), SIM)
        time.sleep(0.02)                      # 50 Гц — обязательно чаще, чем timeout 0.5 с

threading.Thread(target=rx, daemon=True).start()
threading.Thread(target=tx, daemon=True).start()

def stick(roll=0.0, pitch=0.0, thr=-1.0, yaw=0.0):
    cmd["channels"] = [roll, pitch, thr, yaw, 0.0, 0.0]   # aux2 = 0, чтобы не перебивать mode

def wait(cond, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond(): return True
        time.sleep(0.05)
    print("timeout!"); return False

time.sleep(0.3)
if not tel: print("НЕТ ТЕЛЕМЕТРИИ — проверь enableTx и порт 14550")

# 1. сброс + арм
cmd["reset"] = True; time.sleep(0.15); cmd["reset"] = False
stick(thr=-1.0); cmd["arm"] = 1
wait(lambda: tel.get("armed") is True, 5); cmd["arm"] = 0
print("armed:", tel.get("armed"))

# 2. взлёт в AltHold до 3 м
cmd["mode"] = 3; time.sleep(0.15); cmd["mode"] = 0
stick(thr=0.6)
wait(lambda: tel.get("alt", 0) > 3.0, 20)
stick(thr=0.0)
print("высота:", round(tel.get("alt", 0), 2)); time.sleep(4)

# 3. рыскание на 90°
h0 = tel.get("hdg", 0)
stick(thr=0.0, yaw=0.4)
wait(lambda: abs((tel.get("hdg", 0) - h0 + 180) % 360 - 180) > 85, 12)
stick(thr=0.0, yaw=0.0); time.sleep(2)

# 4. посадка
stick(thr=-0.5)
wait(lambda: tel.get("alt", 9) < 0.2, 25)
stick(thr=-1.0); time.sleep(3)
cmd["arm"] = 2; time.sleep(0.15); cmd["arm"] = 0

print("готово. батарея:", round(tel.get("vbat", 0), 2), "В, израсходовано:",
      round(tel.get("usedMah", 0)), "мАч")
running = False; time.sleep(0.1); sock.close()
```

Быстрый тест «жив ли канал» (5 строк):

```python
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(200):
    s.sendto(json.dumps({"channels":[0,0,-1,0,0,0],"arm":1}).encode(), ("127.0.0.1",14551))
    time.sleep(0.02)   # дрон должен заармиться (газ в нуле)
```

---

## 7. Мини-пример на C# (для другого приложения/Unity)

```csharp
using System.Globalization;
using System.Net.Sockets;
using System.Text;

public class DroneLink
{
    readonly UdpClient udp = new UdpClient();
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public void Send(float roll, float pitch, float thr, float yaw,
                     int arm = 0, int mode = 0, bool reset = false)
    {
        string json = string.Format(Inv,
            "{{\"channels\":[{0:F3},{1:F3},{2:F3},{3:F3},0,0],\"arm\":{4},\"mode\":{5},\"reset\":{6}}}",
            roll, pitch, thr, yaw, arm, mode, reset ? "true" : "false");
        var b = Encoding.UTF8.GetBytes(json);
        udp.Send(b, b.Length, "127.0.0.1", 14551);
    }
}
```

Обязательно `InvariantCulture` — иначе в русской локали числа уедут с запятой и JSON сломается.

---

## 8. Частые грабли

1. **Дрон не армится** — газ не в нуле. шли `throttle = -1.0`, а не `0` (0 = 50 %!).
2. **Режим сам переключается** — aux2 ≠ 0. Держи его нулём.
3. **Через полсекунды срабатывает FAILSAFE** — пакеты идут реже, чем `timeout` (0.5 с). Нужен поток 20–50 Гц, даже когда ничего не меняется.
4. **UDP «не работает», а джойстик подключён** — микс `MaxAbs` берёт большее отклонение по каждому каналу. Для чистого UDP выключи в **F4** галочки «Клавиатура» и «Джойстик» (или поставь `mixMode = Priority`).
5. **После краша ничего не слушается** — сначала `{"reset":true}` одним пакетом.
6. **Нет телеметрии** — проверь `enableTx` в `UdpLink` и что приложение слушает именно 14550; при работе с другой машины поменяй `remoteHost` на её IP и открой порты в брандмауэре.
7. Один датаграм = один JSON (или одна CSV-строка), склеивать несколько команд в пакет нельзя.

Если хочешь, могу добавить в протокол высокоуровневые команды вроде `{"cmd":"takeoff","alt":5}`, `{"cmd":"land"}`, `{"cmd":"goto","x":..,"z":..}` с автопилотом внутри симулятора — тогда внешнему приложению не придётся самому крутить стики.

### USER
еще проблемма с джостиком как мне послать Арм с джостика как запустить дрон, так же когда подключен джостик с клавиатуры его тоже не запустить можно ли это поправить?

### ASSISTANT
Разберёмся — тут три накладывающиеся причины:

1. **Арм был только с клавиатуры (`Space`) и по UDP** — кнопки джойстика ни на что не назначались.
2. **Газ микшировался по принципу «кто больше»** (`MaxAbs`). Если у джойстика канал газа не откалиброван и «висит» на 0 (= 50 %), он держит газ высоким, и клавиатура уже **не может** опустить его в ноль → `TryArm` блокируется навсегда.
3. **Порог арма был жёстко 5 %**, а после калибровки газ внизу часто даёт −0.9 (≈5–7 %) — арм не проходит. Плюс через 2.5 с на земле с низким газом срабатывает автодизарм, и кажется, что «дрон не запускается».

Ниже патч: **три способа арма** (кнопка джойстика, стик-комбо как в Betaflight, клавиша), **приоритет газа** вместо «максимума» и **понятная причина отказа прямо на HUD**.

---

## 1. `SimCore.cs` — кнопки действий в калибровке

В класс `CalibrationData` (в конец, перед закрывающей скобкой) добавь:

```csharp
    [Header("Кнопки действий (индекс кнопки, -1 = не назначено)")]
    public int armButton = -1;
    public int disarmButton = -1;
    public int resetButton = -1;
    public int modeButton = -1;
    public int cameraButton = -1;
```

Сохранится в тот же JSON автоматически.

---

## 2. `DroneInputManager.cs`

**2.1** Рядом с `public enum JoystickBackend ...` вверху файла добавь:

```csharp
public enum ThrottleMixMode { Priority, Max }
```

**2.2** В поля класса (например, после блока «Джойстик SM001») добавь:

```csharp
    [Header("Газ: кто главный")]
    [Tooltip("Priority — газ у источника, который последним двигал ручку газа (клавиатура и джойстик не мешают друг другу). Max — берётся максимальное значение.")]
    public ThrottleMixMode throttleMix = ThrottleMixMode.Priority;

    float lastKbThrT = -1f, lastJoyThrT = -1f, lastUdpThrT = -1f;
    float prevJoyThr, prevUdpThr;
    int thrOwner = 0;
    readonly bool[] prevButtons = new bool[32];
```

**2.3** В `ReadKeyboard()` замени две строки газа на:

```csharp
        if (Hotkeys.Held(SimKey.ThrUp)) { kbThr += kbThrottleSpeed * fine * dt; lastKbThrT = Time.realtimeSinceStartup; }
        if (Hotkeys.Held(SimKey.ThrDn)) { kbThr -= kbThrottleSpeed * fine * dt; lastKbThrT = Time.realtimeSinceStartup; }
```

**2.4** В `Update()` сразу после строки `if (udp != null) udp.Poll();` вставь:

```csharp
        float nowT = Time.realtimeSinceStartup;
        if (Mathf.Abs(JoyCal[2] - prevJoyThr) > 0.01f) { prevJoyThr = JoyCal[2]; lastJoyThrT = nowT; }
        float ut = udp != null ? udp.Control.throttle : -1f;
        if (Mathf.Abs(ut - prevUdpThr) > 0.01f) { prevUdpThr = ut; lastUdpThrT = nowT; }
        PollActionButtons();
```

**2.5** Добавь два новых метода в класс:

```csharp
    // Фронт нажатия кнопки джойстика -> команда
    void PollActionButtons()
    {
        for (int i = 0; i < ButtonValues.Count && i < prevButtons.Length; i++)
        {
            bool now = ButtonValues[i], was = prevButtons[i];
            prevButtons[i] = now;
            if (!now || was) continue;
            if (i == cal.armButton) armFlag = true;
            if (i == cal.disarmButton) disarmFlag = true;
            if (i == cal.resetButton) resetFlag = true;
            if (i == cal.modeButton) modeFlag = true;
            if (i == cal.cameraButton) camFlag = true;
        }
    }

    // Назначение из окна F4
    public void AssignAction(string action, int button)
    {
        switch (action)
        {
            case "arm": cal.armButton = button; break;
            case "disarm": cal.disarmButton = button; break;
            case "reset": cal.resetButton = button; break;
            case "mode": cal.modeButton = button; break;
            case "camera": cal.cameraButton = button; break;
        }
        SaveCal();
    }
```

**2.6** В `Mix()` найди внутри ветки `else` блок `if (ch == 2) { ... }` и замени **весь этот if-блок** на:

```csharp
                if (ch == 2)
                {
                    if (throttleMix == ThrottleMixMode.Max)
                    {
                        v = -1;
                        if (kbOn && kv > v) { v = kv; src = InputSourceKind.Keyboard; }
                        if (joyOn && jv > v) { v = jv; src = InputSourceKind.Joystick; }
                        if (udpOn && uv > v) { v = uv; src = InputSourceKind.Udp; }
                    }
                    else
                    {
                        // хозяин газа — тот, кто последним двигал ручку; если никто — клавиатура (газ 0)
                        int owner = -1; float best = -1f;
                        if (kbOn && lastKbThrT > best) { best = lastKbThrT; owner = 0; }
                        if (joyOn && lastJoyThrT > best) { best = lastJoyThrT; owner = 1; }
                        if (udpOn && lastUdpThrT > best) { best = lastUdpThrT; owner = 2; }
                        if (owner < 0) owner = 0;
                        if (owner != thrOwner)
                        {
                            if (owner == 0) kbThr = Mathf.Clamp01((Mixed[2] + 1f) * 0.5f); // плавная передача
                            thrOwner = owner;
                        }
                        v = owner == 2 ? uv : owner == 1 ? jv : (kbThr * 2f - 1f);
                        src = owner == 2 ? InputSourceKind.Udp : owner == 1 ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
                    }
                }
```

---

## 3. `FlightController.cs`

**3.1** В блок «Арм и безопасность» добавь поля и свойство:

```csharp
    [Tooltip("Максимальный газ, при котором разрешён ARM (0.08 = 8%)")]
    [Range(0f, 0.3f)] public float armMaxThrottle = 0.08f;
    [Tooltip("Арм стиками как в Betaflight: газ вниз + рыскание вправо. Дизарм: газ вниз + рыскание влево")]
    public bool armStickCombo = true;
    [Range(0.2f, 2f)] public float stickComboHold = 0.7f;
    [Tooltip("Автодизарм на земле с низким газом. Для отладки поставь 10 с")]
    public string ArmBlockReason { get; private set; } = "";
    float comboTimer;
```

**3.2** Замени `TryArm()` целиком и добавь `CanArm`:

```csharp
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
```

**3.3** В `Update()` в самый конец добавь:

```csharp
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
```

---

## 4. `SimHud.cs`

**4.1** В `DrawWarnings()` замени блок `if (!fc.Armed && !ph.Crashed) ...` на:

```csharp
        if (!fc.Armed && !ph.Crashed)
        {
            Txt(new Rect(W / 2 - 360, 770, 720, 24),
                "ARM: Space / кнопка джойстика / стики: газ вниз + YAW вправо (0.7 с)",
                15, Cdim, TextAnchor.MiddleCenter);
            bool ok = string.IsNullOrEmpty(fc.ArmBlockReason);
            Txt(new Rect(W / 2 - 360, 794, 720, 24),
                ok ? "готов к арму" : fc.ArmBlockReason,
                16, ok ? Cok : Cwarn, TextAnchor.MiddleCenter, true);
        }
```

**4.2** В `WinInput()` в цикле кнопок (там, где `→AUX1 / →AUX2`) добавь кнопки действий:

```csharp
            if (GUILayout.Button("ARM", GUILayout.Width(40))) inp.AssignAction("arm", i);
            if (GUILayout.Button("DIS", GUILayout.Width(36))) inp.AssignAction("disarm", i);
            if (GUILayout.Button("RST", GUILayout.Width(36))) inp.AssignAction("reset", i);
            if (GUILayout.Button("MODE", GUILayout.Width(46))) inp.AssignAction("mode", i);
            if (GUILayout.Button("CAM", GUILayout.Width(40))) inp.AssignAction("camera", i);
```

И расширь окно, чтобы влезло — в полях класса:

```csharp
    Rect rInp = new Rect(300, 140, 780, 640);
```

---

## Как теперь запускать дрон

**Способ 1 — стиками (как на настоящем квадрокоптере):**
газ **в самый низ**, рыскание (левый стик по горизонтали) **вправо до упора**, подержи **0.7 с** → ARM. Влево до упора → DISARM. Работает и на джойстике, и на клавиатуре (`S` + `D`).

**Способ 2 — кнопкой джойстика:**
F4 → раздел кнопок → щёлкни нужную кнопку/тумблер на пульте (строка загорится `ON`) → нажми в этой строке **ARM**. Аналогично **DIS** (дизарм), **RST** (сброс), **MODE**, **CAM**. Всё сохраняется в JSON.

**Способ 3 — клавиша `Space`** (как раньше).

**Взлёт:** после арма сразу поднимай газ (`W` или левый стик вверх) примерно до середины — дрон висит. Проще: переключи `M` в **AltHold** и дай газ чуть выше центра — высота держится сама.

---

## Два обязательных момента

- **Автодизарм на земле.** Если после арма газ ниже ~12 % дольше `groundDisarmTime` (2.5 с) — дрон сам дизармится. Пока осваиваешься, поставь в инспекторе `FlightController → Ground Disarm Time = 10`.
- **Калибровка газа (F2).** Внизу стик должен давать ровно `THR = −1.00`, вверху `+1.00`. Если внизу, например, −0.85 — арм не пройдёт даже с новым порогом 8 %. Теперь причина будет написана на экране: «ГАЗ 7 % — опусти ниже 8 %», так что сразу видно, что чинить.

Клавиатура и джойстик теперь не конфликтуют: газ принадлежит тому, кто последним трогал ручку, а при переходе к клавиатуре значение подхватывается плавно, без скачка тяги.

### USER
газ с пульта работает хорошо теперь сломалось управление дрон не наклоняется и не поворачивет ни с клавиатуры ни с джостика!

### ASSISTANT


### USER
газ с пульта работает хорошо теперь сломалось управление дрон не наклоняется и не поворачивет ни с клавиатуры ни с джостика!

### ASSISTANT


### USER
газ с пульта работает хорошо теперь сломалось управление дрон не наклоняется и не поворачивет ни с клавиатуры ни с джостика, помоги починить это

### ASSISTANT
Судя по симптомам («газ работает, а крен/тангаж/рыскание мертвы у **обоих** источников»), при замене блока `if (ch == 2)` в `Mix()` потерялась соседняя ветка `else`, которая считает **остальные каналы**. В итоге в цикле присваивается только газ, а `Mixed[0]`, `Mixed[1]`, `Mixed[3]` навсегда остаются нулями.

## Быстрая проверка (10 секунд)

Play → **F4** → подвигай стики/стрелки и смотри на строки каналов внизу HUD:
- **ROLL / PITCH / YAW стоят по 0.00**, хотя сырые оси шевелятся → точно `Mix()`, лечится ниже.
- Каналы шевелятся, но дрон не реагирует → смотри чек-лист в конце.

---

## Исправление: замени метод `Mix()` целиком

Чтобы больше не ловить ошибки при вставках, вот **полный метод** — просто выдели старый `void Mix() { ... }` в `DroneInputManager.cs` и замени на этот. Логика приоритета газа сохранена.

```csharp
    void Mix()
    {
        float t = Time.realtimeSinceStartup;
        float[] kb = { kbAxes[0], kbAxes[1], kbThr * 2f - 1f, kbAxes[3], 0, 0 };
        float[] joy = JoyCal;
        ControlFrame u = udp != null ? udp.Control : ControlFrame.Neutral;
        float[] net = { u.roll, u.pitch, u.throttle, u.yaw, u.aux1, u.aux2 };

        bool kbOn = enableKeyboard;
        bool joyOn = enableJoystick && JoyConnected;
        bool udpOn = enableUdp && udp != null && udp.Fresh;

        if (AnyActive(kb, 0.05f) || kbThr > 0.01f) lastKbActive = t;
        if (joyOn && AnyActive(joy, 0.05f)) lastJoyActive = t;
        if (udpOn) { lastUdpActive = t; udpEverFresh = true; }

        int prio = 0;
        if (lastJoyActive >= lastKbActive && lastJoyActive >= lastUdpActive) prio = 1;
        if (lastUdpActive >= lastKbActive && lastUdpActive >= lastJoyActive) prio = 2;

        for (int ch = 0; ch < Channels; ch++)
        {
            float v = 0f;
            InputSourceKind src = InputSourceKind.None;
            float kv = kbOn ? kb[ch] : 0f;
            float jv = joyOn ? joy[ch] : 0f;
            float uv = udpOn ? net[ch] : 0f;

            if (mixMode == InputMixMode.Priority)
            {
                v = prio == 2 ? uv : prio == 1 ? jv : kv;
                src = prio == 2 ? InputSourceKind.Udp : prio == 1 ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
            }
            else if (mixMode == InputMixMode.Sum)
            {
                v = Mathf.Clamp(kv + jv + uv, -1f, 1f);
                src = Mathf.Abs(uv) >= Mathf.Abs(jv) && Mathf.Abs(uv) >= Mathf.Abs(kv) ? InputSourceKind.Udp
                    : Mathf.Abs(jv) >= Mathf.Abs(kv) ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
            }
            else // MaxAbs — одновременная работа источников
            {
                if (ch == 2)
                {
                    // ГАЗ: хозяин — тот, кто последним двигал ручку
                    if (throttleMix == ThrottleMixMode.Max)
                    {
                        v = -1f;
                        if (kbOn && kv > v) { v = kv; src = InputSourceKind.Keyboard; }
                        if (joyOn && jv > v) { v = jv; src = InputSourceKind.Joystick; }
                        if (udpOn && uv > v) { v = uv; src = InputSourceKind.Udp; }
                    }
                    else
                    {
                        int owner = -1; float best = -1f;
                        if (kbOn && lastKbThrT > best) { best = lastKbThrT; owner = 0; }
                        if (joyOn && lastJoyThrT > best) { best = lastJoyThrT; owner = 1; }
                        if (udpOn && lastUdpThrT > best) { best = lastUdpThrT; owner = 2; }
                        if (owner < 0) owner = 0;
                        if (owner != thrOwner)
                        {
                            if (owner == 0) kbThr = Mathf.Clamp01((Mixed[2] + 1f) * 0.5f);
                            thrOwner = owner;
                        }
                        v = owner == 2 ? uv : owner == 1 ? jv : (kbThr * 2f - 1f);
                        src = owner == 2 ? InputSourceKind.Udp : owner == 1 ? InputSourceKind.Joystick : InputSourceKind.Keyboard;
                    }
                }
                else
                {
                    // КРЕН / ТАНГАЖ / РЫСКАНИЕ / AUX — побеждает большее отклонение
                    if (Mathf.Abs(kv) > Mathf.Abs(v)) { v = kv; src = InputSourceKind.Keyboard; }
                    if (Mathf.Abs(jv) > Mathf.Abs(v)) { v = jv; src = InputSourceKind.Joystick; }
                    if (Mathf.Abs(uv) > Mathf.Abs(v)) { v = uv; src = InputSourceKind.Udp; }
                }
            }

            Mixed[ch] = v;
            ActiveSrc[ch] = src;
            if (src == InputSourceKind.Joystick && Mathf.Abs(v) > 0.05f) lastJoyDominant = t;
        }

        var s = ControlFrame.Neutral;
        s.roll = SimMath.Expo(SimMath.Deadzone(Mixed[0], deadzone), expoRollPitch);
        s.pitch = SimMath.Expo(SimMath.Deadzone(Mixed[1], deadzone), expoRollPitch);
        s.yaw = SimMath.Expo(SimMath.Deadzone(Mixed[3], deadzone), expoYaw);
        s.throttle = SimMath.Expo(Mixed[2], throttleExpo);
        s.aux1 = Mixed[4];
        s.aux2 = Mixed[5];
        Sticks = s;
    }
```

Ключевые признаки, что метод теперь правильный:
- внутри цикла есть **обе** ветки: `if (ch == 2) { …газ… }` **и** `else { …крен/тангаж/рыскание… }`;
- после цикла обязательно идут строки `s.roll = …`, `s.pitch = …`, `s.yaw = …` и `Sticks = s;`.

---

## Если каналы шевелятся, но дрон всё равно не наклоняется

Проверь по порядку — всё видно на HUD:

1. **ARMED?** Без арма моторы только на холостых, стики ничего не делают. Смотри строку состояния и причину отказа арма (мы её добавили).
2. **Нет ли красной надписи FAILSAFE.** Если ты раньше в этой же сессии гонял Python-скрипт по UDP и остановил его, срабатывает защёлка «UDP был и пропал» — все стики принудительно обнуляются. Лечится: сними галочку `Failsafe On Udp Lost` в `DroneInputManager` (или перезапусти Play).
3. **Режим.** В `PosHold` на земле с малым газом наклона почти не будет — проверь в `Angle` (`M`).
4. **F5 → RATE ROLL / PITCH / YAW:** ползунки **P** не должны стоять в нуле (их легко сдвинуть мышью в окне тюнинга). Дефолты: P ≈ 0.0018 для крена/тангажа и 0.004 для рыскания.
5. **AUX2 ≠ 0** постоянно держит режим Acro/Angle — само по себе управление не ломает, но если ты ждал Angle, а стоит Acro, ощущения будут другими.

После замены `Mix()` крен/тангаж/рыскание должны ожить сразу и с пульта, и со стрелок — при этом газ останется у того источника, который последним трогал ручку.

### USER
При нажатии F4 пропал весь интерфейс помоги его вернуть, он мне нужен для настройки управления

### ASSISTANT
Это классическая беда IMGUI: **внутри окна F4 вылетает исключение**, Unity обрывает `OnGUI` целиком — и пропадает не только окно, а весь HUD. Хоткеи при этом продолжают работать.

## Вернуть интерфейс прямо сейчас

- Нажми **F4 ещё раз** — окно закроется, HUD вернётся.
- Если экран пустой — **F6** (переключатель HUD).
- Если GUI уже «сломался» до конца кадра — стоп Play и снова Play.

## Почему падает

В winmm-ветке `ReadJoystick()` при отсутствии устройства списки очищаются и метод выходит: `AxisValues.Count = 0`, а `AxisLabels.Count = 8` (метки-то всегда добавляются). Окно F4 рисует цикл по меткам и лезет в `AxisValues[i]` → `ArgumentOutOfRangeException` → весь `OnGUI` обрывается. Плюс меняющееся между Layout/Repaint количество контролов само по себе ломает GUILayout.

Ниже три патча: первый чинит причину, второй и третий делают так, чтобы HUD **больше никогда не умирал** из-за окна.

---

## Патч 1 — `DroneInputManager.cs`, winmm-ветка в `ReadJoystick()`

Замени блок `if (UseWin) { ... }` в начале метода на:

```csharp
        if (UseWin)
        {
            AxisValues.Clear(); ButtonValues.Clear();
            winJoy.PollAll();
            var d = winJoy.Current;

            // длины списков ВСЕГДА постоянны — иначе окно F4 падает по индексу
            for (int i = 0; i < 8; i++) AxisValues.Add(d != null ? d.axes[i] : 0f);
            for (int i = 0; i < 32; i++) ButtonValues.Add(d != null && d.buttons[i]);

            for (int ch = 0; ch < Channels; ch++)
            {
                if (d == null) { JoyCal[ch] = 0f; continue; }
                int bi = cal.buttonIndex[ch];
                float raw = bi >= 0
                    ? (bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f)
                    : (cal.sourceIndex[ch] < AxisValues.Count ? AxisValues[cal.sourceIndex[ch]] : 0f);
                JoyCal[ch] = cal.Normalize(ch, raw);
            }
            if (d != null) joyEverActive = true;
            return;
        }
```

---

## Патч 2 — `SimHud.cs`, окно `WinInput()`

Найди строку цикла осей:

```csharp
        for (int i = 0; i < inp.AxisLabels.Count && i < 16; i++)
```

Замени на:

```csharp
        int axisCount = Mathf.Min(inp.AxisLabels.Count, inp.AxisValues.Count);
        for (int i = 0; i < axisCount && i < 16; i++)
```

---

## Патч 3 — `SimHud.cs`, защита `OnGUI()`

Замени метод `OnGUI()` целиком и добавь рядом вспомогательный метод:

```csharp
    void OnGUI()
    {
        if (fc == null) return;
        var saved = GUI.matrix;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        float W = Screen.width / scale, H = 1080;

        try
        {
            if (showHud)
            {
                DrawTopBar(W);
                DrawADI(new Rect(60, 210, 300, 300));
                DrawTapes(new Rect(60, 210, 300, 300));
                DrawRightPanel(W);
                DrawSticks(W, H);
                DrawWarnings(W);
                Txt(new Rect(20, H - 34, 700, 24),
                    "F1 Помощь   F2 Калибровка   F3 Ветер   F4 Ввод   F5 Тюнинг   F6 HUD   C Камера", 14, Cdim);
                if (cam != null && cam.mode == CamMode.Fpv)
                {
                    Fill(new Rect(W / 2 - 14, H / 2 - 1, 28, 2), Cacc);
                    Fill(new Rect(W / 2 - 1, H / 2 - 14, 2, 28), Cacc);
                }
            }
        }
        catch (System.Exception e) { UnityEngine.Debug.LogError("[HUD] " + e); }

        winStyle = winStyle ?? new GUIStyle(GUI.skin.window);
        SafeWindow(11, ref rHelp, ref wHelp, WinHelp, "ПОМОЩЬ / УПРАВЛЕНИЕ");
        SafeWindow(12, ref rCal, ref wCal, WinCal, "КАЛИБРОВКА SM001");
        SafeWindow(13, ref rEnv, ref wEnv, WinEnv, "ВЕТЕР И СРЕДА");
        SafeWindow(14, ref rInp, ref wInp, WinInput, "ВВОД / КАНАЛЫ");
        SafeWindow(15, ref rTune, ref wTune, WinTune, "ТЮНИНГ PID / РАМЫ");

        GUI.matrix = saved;
    }

    // Сбойное окно закрывается само, HUD продолжает жить
    void SafeWindow(int id, ref Rect rect, ref bool open, GUI.WindowFunction func, string title)
    {
        if (!open) return;
        try { rect = GUI.Window(id, rect, func, title, winStyle); }
        catch (System.Exception e)
        {
            open = false;
            UnityEngine.Debug.LogError($"[HUD] Окно «{title}» закрыто из-за ошибки:\n{e}");
        }
    }
```

---

После этого F4 будет открываться нормально, а если в каком-то окне снова случится ошибка — закроется только оно, остальной интерфейс останется, и в Console будет точный текст причины. Если что-то опять упадёт — пришли мне строку ошибки из Console, поправлю прицельно.

### USER
наклоны вправо и влево с клавиатуры работают не правильно и инвертировано как их инвертировать в другую сторону?

### ASSISTANT
Причина не в клавиатуре, а в **знаке крена во всём тракте**: в Unity поворот вокруг **+Z поднимает правое крыло** (т.е. это крен ВЛЕВО), а у нас в микшере и в измерении угла он был помечен как «вправо». Поэтому команда «вправо» физически наклоняет влево — и с клавиатуры, и с пульта, и авиагоризонт тоже зеркалит.

Чинить лучше сразу в корне — тогда клавиатура, джойстик, UDP, телеметрия и авиагоризонт станут согласованными.

---

## Правильное исправление — 5 замен в `FlightController.cs`

**1. Измерение крена** (в `ComputeControl`). Найди:

```csharp
        float rollMeas = SimMath.Norm180(euler.z) + SimMath.Gaussian() * attitudeNoiseDeg;
```

Замени на:

```csharp
        float rollMeas = -SimMath.Norm180(euler.z) + SimMath.Gaussian() * attitudeNoiseDeg;
```

**2. Гироскоп по крену.** Найди:

```csharp
        rOut = rateRoll.Update(spRoll, wBody.z, dt);     // + roll right
```

Замени на:

```csharp
        rOut = rateRoll.Update(spRoll, -wBody.z, dt);    // + roll right
```

**3. Микшер** (метод `Mix` в `FlightController`). Найди четыре строки и замени на:

```csharp
        cmd[0] = t + r - p + y; // FL
        cmd[1] = t - r - p - y; // FR
        cmd[2] = t - r + p + y; // BR
        cmd[3] = t + r + p - y; // BL
```

**4. Телеметрия/HUD.** Найди свойство:

```csharp
    public float AttRoll => SimMath.Norm180(transform.eulerAngles.z);      // + вправо
```

Замени на:

```csharp
    public float AttRoll => -SimMath.Norm180(transform.eulerAngles.z);     // + вправо
```

**5. PosHold** (иначе удержание точки будет уводить вбок). Найди:

```csharp
                tRoll = Mathf.Clamp(-Mathf.Atan2(aR, g) * Mathf.Rad2Deg, -posMaxTilt, posMaxTilt);
```

Замени на:

```csharp
                tRoll = Mathf.Clamp(Mathf.Atan2(aR, g) * Mathf.Rad2Deg, -posMaxTilt, posMaxTilt);
```

> Важно: знаки меняются **одновременно** в измерении (п.1–2) и в воздействии (п.3) — обратная связь остаётся отрицательной, устойчивость PID не меняется. Если поменять что-то одно, дрон уйдёт в неуправляемое вращение.

---

## Если хочешь инвертировать только клавиатуру (быстрый вариант)

В `DroneInputManager.ReadKeyboard()` первая строка массива `target`:

```csharp
            (Hotkeys.Held(SimKey.RollR) ? 1 : 0) - (Hotkeys.Held(SimKey.RollL) ? 1 : 0),
```

замени на:

```csharp
            (Hotkeys.Held(SimKey.RollL) ? 1 : 0) - (Hotkeys.Held(SimKey.RollR) ? 1 : 0),
```

Но тогда джойстик и UDP останутся зеркальными, а авиагоризонт будет показывать крен наоборот — так что рекомендую вариант выше.

---

## Проверка после правки

1. Режим **Angle** (`M`), арм, газ ~50 %.
2. Стрелка **вправо** → дрон кренится **вправо**, на HUD `R` показывает **плюс**, авиагоризонт наклоняется в ту же сторону.
3. Правый стик пульта вправо → то же самое.

⚠️ Если ты раньше компенсировал инверсию галочкой **inv** у канала ROLL в **F4** — **сними её**, иначе получится двойная инверсия и крен снова уедет не туда. Галочка `inv` теперь нужна только если конкретный экземпляр пульта отдаёт ось с обратным знаком (проверяется просто: стик вправо → канал ROLL должен быть `+1`).

### USER
добавь открывающиеся меню вот с этими графиками телеметрии, а так же выведи туда обороты каждого винта

### ASSISTANT
Отлично, сделаем отдельную панель телеметрии в стиле твоих скриншотов: **вкладки LIVE TRACES / ROTOR BENCH / EVENT LOG**, сворачиваемые группы графиков и карточки моторов с крупными оборотами каждого винта.

## Что внутри

- **LIVE TRACES** — сворачиваемые (клик по заголовку) группы: высота/скороподъёмность, угловые скорости p/q/r, ветер X/Z + ground effect, **обороты всех 4 винтов**, duty моторов, батарея V/A, крен/тангаж. Пишутся в кольцевой буфер (по умолчанию 20 Гц × 30 с), рисуются в текстуру — дёшево даже с 7 графиками.
- **ROTOR BENCH** — 4 карточки: `M1 · CW`, статус `SATURATED / VRS / RUN / OFF`, крупные **RPM**, полоса duty, тяга T (Н), момент Q (Н·м), ток I (А), DUTY %, IGE (экранной эффект по каждому винту), VRS.
- **EVENT LOG** — ARM/DISARM, смена режима, CRASH, FAILSAFE, LOW/CRITICAL BATTERY, VRS, развал дрона.
- Кнопки **REC/PAUSE**, **CLEAR**, длина окна 10/30/60 с, **AUTOSCALE**. Хоткей **F7**.

---

## Новый файл `Assets/Scripts/SimTelemetryPanel.cs`

```csharp
using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(110)]
public class SimTelemetryPanel : MonoBehaviour
{
    [Header("Окно (F7 — показать/скрыть)")]
    public bool show = true;
    public Rect windowRect = new Rect(330, 110, 1010, 660);

    [Header("Запись")]
    [Tooltip("Частота выборки телеметрии, Гц")] [Range(5, 60)] public int sampleRateHz = 20;
    [Tooltip("Длина окна графиков, с")] [Range(10, 120)] public int windowSeconds = 30;
    public bool paused;

    [Header("Графики")]
    [Range(256, 1024)] public int graphWidth = 560;
    [Range(40, 160)] public int graphHeight = 70;
    public bool autoScale = true;

    // ---------- данные ----------
    class Sample
    {
        public float t, alt, vs, p, q, r, wx, wz, ge, roll, pitch, vbat, amps;
        public readonly float[] rpm = new float[4];
        public readonly float[] duty = new float[4];
    }
    class Series
    {
        public string name; public Color color;
        public System.Func<Sample, float> get;
        public float current;
    }
    class Group
    {
        public string title; public bool open; public Series[] series;
        public Texture2D tex; public Color32[] px;
        public float min, max; public bool scaled;
    }
    struct Evt { public float t; public string text; public Color c; }

    readonly Queue<Sample> samples = new Queue<Sample>();
    readonly List<Evt> events = new List<Evt>();
    List<Group> groups;
    int tab; float sampleTimer; Vector2 scrollTraces, scrollLog;
    GUIStyle winStyle, headStyle, bigStyle, smallStyle;

    FlightController fc; QuadPhysics ph; BatteryModel bat; WindSystem wind; DroneBreakup breakup;

    static readonly Color Cbg = new Color(0.02f, 0.04f, 0.07f, 0.92f);
    static readonly Color Cacc = new Color(0.25f, 0.9f, 1f);
    static readonly Color Cok = new Color(0.3f, 1f, 0.45f);
    static readonly Color Cwarn = new Color(1f, 0.75f, 0.2f);
    static readonly Color Cerr = new Color(1f, 0.3f, 0.3f);
    static readonly Color Cdim = new Color(0.72f, 0.78f, 0.84f);
    static readonly Color Cp = new Color(1f, 0.35f, 0.7f);
    static readonly Color Cb = new Color(0.4f, 0.6f, 1f);
    static readonly Color Cv = new Color(0.7f, 0.5f, 1f);
    static readonly Color Cy = new Color(1f, 0.85f, 0.2f);
    static readonly Color Co = new Color(1f, 0.6f, 0.2f);

    // состояние для журнала
    bool lastArmed, lastCrashed, lastFs, lastLow, lastCrit, lastVrs, lastBroken;
    FlightMode lastMode;

    void Start()
    {
        fc = FindObjectOfType<FlightController>();
        ph = FindObjectOfType<QuadPhysics>();
        bat = FindObjectOfType<BatteryModel>();
        wind = FindObjectOfType<WindSystem>();
        breakup = FindObjectOfType<DroneBreakup>();
        enabled = fc != null && ph != null;
        if (!enabled) return;

        lastMode = fc.Mode;
        if (ph.OnCrash != null) { } // событие подписывается ниже
        ph.OnCrash += () => Log("CRASH — удар", Cerr);

        groups = new List<Group>
        {
            new Group { title = "altitude (m) · climb rate (m/s)", open = true, series = new[]
            {
                new Series { name = "alt", color = Cacc, get = s => s.alt },
                new Series { name = "vs",  color = Cb,   get = s => s.vs  }
            }},
            new Group { title = "body rates p/q/r (°/s)", open = true, series = new[]
            {
                new Series { name = "p", color = Cp, get = s => s.p },
                new Series { name = "q", color = Cok, get = s => s.q },
                new Series { name = "r", color = Cb, get = s => s.r }
            }},
            new Group { title = "wind X/Z (m/s) · ground effect", open = true, series = new[]
            {
                new Series { name = "wx", color = Cv, get = s => s.wx },
                new Series { name = "wz", color = new Color(0.5f, 0.4f, 0.9f), get = s => s.wz },
                new Series { name = "GE", color = Cy, get = s => s.ge }
            }},
            new Group { title = "rotor rpm (об/мин)", open = true, series = new[]
            {
                new Series { name = "M1", color = Cacc, get = s => s.rpm[0] },
                new Series { name = "M2", color = Cok,  get = s => s.rpm[1] },
                new Series { name = "M3", color = Cp,   get = s => s.rpm[2] },
                new Series { name = "M4", color = Co,   get = s => s.rpm[3] }
            }},
            new Group { title = "motor duty (%)", open = false, series = new[]
            {
                new Series { name = "M1", color = Cacc, get = s => s.duty[0] },
                new Series { name = "M2", color = Cok,  get = s => s.duty[1] },
                new Series { name = "M3", color = Cp,   get = s => s.duty[2] },
                new Series { name = "M4", color = Co,   get = s => s.duty[3] }
            }},
            new Group { title = "battery V / A", open = false, series = new[]
            {
                new Series { name = "V", color = Cok,  get = s => s.vbat },
                new Series { name = "A", color = Cwarn, get = s => s.amps }
            }},
            new Group { title = "attitude roll/pitch (°)", open = false, series = new[]
            {
                new Series { name = "roll",  color = Cacc, get = s => s.roll },
                new Series { name = "pitch", color = Cp,   get = s => s.pitch }
            }},
        };
        Log("панель телеметрии запущена", Cdim);
    }

    void Update()
    {
        bool key = false;
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb != null) key = kb.f7Key.wasPressedThisFrame;
#elif ENABLE_LEGACY_INPUT_MANAGER
        key = Input.GetKeyDown(KeyCode.F7);
#endif
        if (key) show = !show;
        if (fc == null) return;

        PollEvents();

        sampleTimer -= Time.unscaledDeltaTime;
        if (sampleTimer > 0f) return;
        sampleTimer = 1f / Mathf.Max(1, sampleRateHz);

        if (!paused) PushSample();
        if (show && tab == 0)
            foreach (var g in groups) if (g.open) Plot(g);
    }

    int Capacity => Mathf.Max(16, sampleRateHz * windowSeconds);

    void PushSample()
    {
        var s = new Sample
        {
            t = Time.time,
            alt = fc.AltAgl,
            vs = fc.Vs,
            roll = fc.AttRoll,
            pitch = fc.AttPitchUp,
            vbat = bat != null ? bat.Voltage : 0f,
            amps = bat != null ? bat.Current : 0f,
            ge = ph.GroundEffectFactor,
            wx = wind != null ? wind.Current.x : 0f,
            wz = wind != null ? wind.Current.z : 0f
        };
        Vector3 w = ph.Rb != null
            ? ph.transform.InverseTransformDirection(ph.Rb.angularVelocity) * Mathf.Rad2Deg
            : Vector3.zero;
        s.p = -w.z;   // крен (после исправления знаков)
        s.q = w.x;    // тангаж
        s.r = w.y;    // рыскание
        for (int i = 0; i < 4; i++)
        {
            s.rpm[i] = ph.motors[i].rpm;
            s.duty[i] = ph.motors[i].cmd * 100f;
        }
        samples.Enqueue(s);
        while (samples.Count > Capacity) samples.Dequeue();
    }

    // ---------- отрисовка графика в текстуру ----------
    void EnsureTex(Group g)
    {
        if (g.tex != null && g.tex.width == graphWidth && g.tex.height == graphHeight) return;
        if (g.tex != null) Destroy(g.tex);
        g.tex = new Texture2D(graphWidth, graphHeight, TextureFormat.RGBA32, false)
        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
        g.px = new Color32[graphWidth * graphHeight];
    }

    void Plot(Group g)
    {
        EnsureTex(g);
        int W = g.tex.width, H = g.tex.height;
        var px = g.px;
        Color32 bg = new Color32(6, 10, 16, 255);
        for (int i = 0; i < px.Length; i++) px[i] = bg;
        Color32 grid = new Color32(22, 30, 40, 255);
        for (int gy = 1; gy < 4; gy++) { int y = H * gy / 4; for (int x = 0; x < W; x++) px[y * W + x] = grid; }

        int n = samples.Count;
        if (n < 2) { g.tex.SetPixels32(px); g.tex.Apply(false); return; }

        float tmin = float.MaxValue, tmax = float.MinValue;
        foreach (var s in samples)
            foreach (var se in g.series)
            {
                float v = se.get(s);
                if (v < tmin) tmin = v;
                if (v > tmax) tmax = v;
            }
        if (tmin > tmax) { tmin = 0; tmax = 1; }
        float pad = (tmax - tmin) * 0.15f + 0.001f;
        tmin -= pad; tmax += pad;
        if (autoScale)
        {
            if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
            else { g.min = Mathf.Lerp(g.min, tmin, 0.25f); g.max = Mathf.Lerp(g.max, tmax, 0.25f); }
        }
        else if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }

        float range = Mathf.Max(1e-4f, g.max - g.min);
        int cap = Capacity;
        foreach (var se in g.series)
        {
            Color32 col = se.color;
            int idx = 0, px0 = 0, py0 = 0;
            foreach (var s in samples)
            {
                int x = idx * (W - 1) / Mathf.Max(1, cap - 1);
                int y = Mathf.Clamp(Mathf.RoundToInt((se.get(s) - g.min) / range * (H - 1)), 0, H - 1);
                if (idx > 0) Line(px, W, H, px0, py0, x, y, col);
                px0 = x; py0 = y; idx++;
                se.current = se.get(s);
            }
        }
        g.tex.SetPixels32(px);
        g.tex.Apply(false);
    }

    static void Line(Color32[] px, int W, int H, int x0, int y0, int x1, int y1, Color32 c)
    {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (x0 >= 0 && x0 < W && y0 >= 0 && y0 < H) px[y0 * W + x0] = c;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ---------- журнал ----------
    void Log(string text, Color c)
    {
        events.Add(new Evt { t = Time.time, text = text, c = c });
        while (events.Count > 300) events.RemoveAt(0);
    }
    void PollEvents()
    {
        if (fc.Armed != lastArmed) { lastArmed = fc.Armed; Log(fc.Armed ? "ARM" : "DISARM", fc.Armed ? Cok : Cdim); }
        if (fc.Mode != lastMode) { lastMode = fc.Mode; Log("MODE → " + fc.Mode, Cacc); }
        if (ph.Crashed != lastCrashed) { lastCrashed = ph.Crashed; if (ph.Crashed) Log("CRASHED", Cerr); else Log("REPAIRED / RESET", Cok); }
        if (fc.Failsafe != lastFs) { lastFs = fc.Failsafe; Log(fc.Failsafe ? "FAILSAFE ON" : "FAILSAFE OFF", fc.Failsafe ? Cerr : Cok); }
        if (bat != null)
        {
            if (bat.IsLow != lastLow) { lastLow = bat.IsLow; if (bat.IsLow) Log("LOW BATTERY", Cwarn); }
            if (bat.IsCritical != lastCrit) { lastCrit = bat.IsCritical; if (bat.IsCritical) Log("BATTERY CRITICAL", Cerr); }
        }
        if (ph.VrsActive != lastVrs) { lastVrs = ph.VrsActive; if (ph.VrsActive) Log("VORTEX RING STATE", Cwarn); }
        if (breakup != null && breakup.IsBroken != lastBroken)
        { lastBroken = breakup.IsBroken; Log(lastBroken ? "BREAKUP — дрон разобран" : "REASSEMBLED", lastBroken ? Cerr : Cok); }
    }

    // ---------- GUI ----------
    void OnGUI()
    {
        if (!show || fc == null) return;
        var saved = GUI.matrix;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        winStyle = winStyle ?? new GUIStyle(GUI.skin.window);
        try { windowRect = GUI.Window(77, windowRect, DrawWindow, "ТЕЛЕМЕТРИЯ   [F7]", winStyle); }
        catch (System.Exception e) { show = false; UnityEngine.Debug.LogError("[Telemetry] " + e); }
        GUI.matrix = saved;
    }

    void DrawWindow(int id)
    {
        headStyle = headStyle ?? new GUIStyle(GUI.skin.button) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
        bigStyle = bigStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 30, fontStyle = FontStyle.Bold };
        smallStyle = smallStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 11 };

        GUILayout.BeginHorizontal();
        string[] tabs = { "LIVE TRACES", "ROTOR BENCH", "EVENT LOG" };
        for (int i = 0; i < tabs.Length; i++)
        {
            GUI.color = tab == i ? Cacc : Color.white;
            if (GUILayout.Button(tabs[i], GUILayout.Height(26))) tab = i;
        }
        GUI.color = Color.white;
        GUILayout.FlexibleSpace();
        GUI.color = paused ? Cwarn : Cok;
        if (GUILayout.Button(paused ? "● REC" : "⏸ PAUSE", GUILayout.Width(90), GUILayout.Height(26))) paused = !paused;
        GUI.color = Color.white;
        if (GUILayout.Button("CLEAR", GUILayout.Width(70), GUILayout.Height(26))) { samples.Clear(); events.Clear(); foreach (var g in groups) g.scaled = false; }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("окно:", GUILayout.Width(45));
        int[] secs = { 10, 30, 60 };
        foreach (var s in secs)
        {
            GUI.color = windowSeconds == s ? Cacc : Color.white;
            if (GUILayout.Button(s + " с", GUILayout.Width(56))) windowSeconds = s;
        }
        GUI.color = Color.white;
        autoScale = GUILayout.Toggle(autoScale, "AUTOSCALE", GUILayout.Width(110));
        GUILayout.Label($"  образцов: {samples.Count}/{Capacity}   {sampleRateHz} Гц", smallStyle);
        GUILayout.EndHorizontal();
        GUILayout.Space(4);

        if (tab == 0) DrawTraces();
        else if (tab == 1) DrawRotors();
        else DrawLog();

        GUI.DragWindow();
    }

    void DrawTraces()
    {
        scrollTraces = GUILayout.BeginScrollView(scrollTraces);
        foreach (var g in groups)
        {
            if (GUILayout.Button((g.open ? "▼  " : "►  ") + g.title, headStyle, GUILayout.Height(24)))
            { g.open = !g.open; if (g.open) Plot(g); }
            if (!g.open) continue;

            GUILayout.BeginHorizontal();
            foreach (var se in g.series)
            {
                GUI.color = se.color;
                GUILayout.Label(se.current.ToString("0.00") + " " + se.name, GUILayout.Width(130));
            }
            GUI.color = Cdim;
            GUILayout.FlexibleSpace();
            GUILayout.Label($"[{g.min:0.0} … {g.max:0.0}]", smallStyle, GUILayout.Width(150));
            GUI.color = Color.white;
            GUILayout.EndHorizontal();

            Rect r = GUILayoutUtility.GetRect(graphWidth, graphHeight, GUILayout.ExpandWidth(true));
            if (g.tex != null) GUI.DrawTexture(r, g.tex);
            GUILayout.Space(6);
        }
        GUILayout.EndScrollView();
    }

    void DrawRotors()
    {
        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        for (int i = 0; i < 4; i++)
        {
            var m = ph.motors[i];
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(232));

            string status; Color sc;
            if (m.cmd > 0.97f) { status = "SATURATED"; sc = Cwarn; }
            else if (m.vrs) { status = "VRS"; sc = Cwarn; }
            else if (m.rpm < 100f) { status = "OFF"; sc = Cdim; }
            else { status = "RUN"; sc = Cok; }

            GUILayout.BeginHorizontal();
            GUILayout.Label($"M{i + 1} · {(m.spinDir > 0 ? "CW" : "CCW")}", GUILayout.Width(110));
            GUI.color = sc; GUILayout.Label(status, smallStyle); GUI.color = Color.white;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(m.rpm.ToString("N0"), bigStyle, GUILayout.Width(140));
            GUILayout.Label("RPM", smallStyle, GUILayout.Width(40));
            GUILayout.EndHorizontal();

            Rect bar = GUILayoutUtility.GetRect(210, 8, GUILayout.ExpandWidth(false));
            GUI.color = new Color(1, 1, 1, 0.1f); GUI.DrawTexture(bar, Texture2D.whiteTexture);
            GUI.color = m.cmd > 0.97f ? Cwarn : Cok;
            GUI.DrawTexture(new Rect(bar.x, bar.y, bar.width * Mathf.Clamp01(m.cmd), bar.height), Texture2D.whiteTexture);
            GUI.color = Color.white;

            float eff = ph.gramsPerWatt * (1.15f - 0.5f * m.cmd);
            float power = (m.thrustN / 0.00981f) / Mathf.Max(1f, eff);
            float amps = bat != null ? power / Mathf.Max(1f, bat.Voltage) : 0f;
            float ige = Ige(m);

            GUILayout.Label($"T {m.thrustN:0.00} N     Q {m.torqueNm:0.0000}", smallStyle);
            GUILayout.Label($"I {amps:0.0} A       DUTY {m.cmd * 100f:0}%", smallStyle);
            GUILayout.Label($"IGE {ige:0.000}     VRS {(m.vrs ? 1f : 0f):0.00}", smallStyle);
            GUILayout.Label($"h {m.groundH:0.00} m    ω {m.rpm * 0.10472f:0} rad/s", smallStyle);
            GUILayout.EndVertical();
            if (i < 3) GUILayout.Space(4);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(10);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Суммарная тяга: {ph.TotalThrustN:0.0} Н  ({ph.TotalThrustN / 0.00981f:0} г)", GUILayout.Width(300));
        GUILayout.Label($"Мощность: {ph.PowerW:0} Вт", GUILayout.Width(200));
        if (bat != null) GUILayout.Label($"Батарея: {bat.Voltage:0.00} В  {bat.Current:0.0} А  {bat.Soc * 100f:0}%", GUILayout.Width(300));
        GUILayout.EndHorizontal();
        GUILayout.Label($"Ground effect (средний): x{ph.GroundEffectFactor:0.000}   Воздушная скорость: {ph.Airspeed:0.0} м/с", smallStyle);
    }

    float Ige(MotorUnit m)
    {
        if (!ph.groundEffect) return 1f;
        float R = ph.propRadius;
        if (m.groundH >= 4f * R) return 1f;
        float h = Mathf.Max(m.groundH, R * 0.35f);
        float r = R / (4f * h);
        return Mathf.Clamp(1f / (1f - r * r), 1f, ph.groundEffectMax);
    }

    void DrawLog()
    {
        scrollLog = GUILayout.BeginScrollView(scrollLog);
        for (int i = events.Count - 1; i >= 0; i--)
        {
            var e = events[i];
            GUILayout.BeginHorizontal();
            GUILayout.Label(e.t.ToString("0.0") + " s", smallStyle, GUILayout.Width(70));
            GUI.color = e.c; GUILayout.Label(e.text); GUI.color = Color.white;
            GUILayout.EndHorizontal();
        }
        GUILayout.EndScrollView();
    }

    void OnDestroy()
    {
        if (groups != null) foreach (var g in groups) if (g.tex != null) Destroy(g.tex);
    }
}
```

---

## Как подключить

1. Создай файл, дождись компиляции.
2. Выдели объект **HUD** (или любой другой) → **Add Component ▸ Sim Telemetry Panel**.
3. Play → **F7** открывает/скрывает окно. Окно таскается за заголовок.

Чтобы панель создавалась автоматически при сборке сцены, в `DroneSceneBuilder.BuildCameraAndHud(...)` рядом с созданием HUD добавь:

```csharp
        if (UnityEngine.Object.FindObjectOfType<SimTelemetryPanel>() == null)
            hg.AddComponent<SimTelemetryPanel>();
```

(внутри того же `if`, где создаётся `hg`). И в подсказку внизу `SimHud` можно дописать `F7 Телеметрия`.

---

## Как пользоваться

- **Сворачивание групп**: клик по строке `▼ altitude (m) · climb rate (m/s)` → группа складывается в `►`. Так можно оставить только нужные графики — заодно снижается нагрузка на отрисовку.
- **Обороты винтов**: группа `rotor rpm (об/мин)` — 4 цветные линии M1–M4 (FL, FR, BR, BL), плюс вкладка **ROTOR BENCH** с крупными цифрами RPM, тягой, моментом, током, duty, IGE и VRS по каждому мотору.
- **REC/PAUSE** — заморозить запись (графики остаются), **CLEAR** — очистить историю и журнал, **10/30/60 с** — длина окна, **AUTOSCALE** — автоподстройка вертикального масштаба (выключи, чтобы зафиксировать масштаб и сравнивать пики).
- **EVENT LOG** — хронология арма, режимов, крашей, фейлсейфа, просадки батареи, VRS и развала дрона.

Настройки частоты записи, размера графиков и стартового окна вынесены в инспектор. Если график кажется «рваным» — подними `Sample Rate Hz` до 40–50; если хочется длинную историю — увеличь `Window Seconds` (буфер = частота × секунды).

### USER
сделал все это и возникла ошибка Assets\SimTelemetryPanel.cs(80,16): error CS0070: The event 'QuadPhysics.OnCrash' can only appear on the left hand side of += or -= (except when used from within the type 'QuadPhysics')

 помоги исправвить вот этот скрипт целиком:  using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(110)]
public class SimTelemetryPanel : MonoBehaviour
{
    [Header("Окно (F7 — показать/скрыть)")]
    public bool show = true;
    public Rect windowRect = new Rect(330, 110, 1010, 660);

    [Header("Запись")]
    [Tooltip("Частота выборки телеметрии, Гц")] [Range(5, 60)] public int sampleRateHz = 20;
    [Tooltip("Длина окна графиков, с")] [Range(10, 120)] public int windowSeconds = 30;
    public bool paused;

    [Header("Графики")]
    [Range(256, 1024)] public int graphWidth = 560;
    [Range(40, 160)] public int graphHeight = 70;
    public bool autoScale = true;

    // ---------- данные ----------
    class Sample
    {
        public float t, alt, vs, p, q, r, wx, wz, ge, roll, pitch, vbat, amps;
        public readonly float[] rpm = new float[4];
        public readonly float[] duty = new float[4];
    }
    class Series
    {
        public string name; public Color color;
        public System.Func<Sample, float> get;
        public float current;
    }
    class Group
    {
        public string title; public bool open; public Series[] series;
        public Texture2D tex; public Color32[] px;
        public float min, max; public bool scaled;
    }
    struct Evt { public float t; public string text; public Color c; }

    readonly Queue<Sample> samples = new Queue<Sample>();
    readonly List<Evt> events = new List<Evt>();
    List<Group> groups;
    int tab; float sampleTimer; Vector2 scrollTraces, scrollLog;
    GUIStyle winStyle, headStyle, bigStyle, smallStyle;

    FlightController fc; QuadPhysics ph; BatteryModel bat; WindSystem wind; DroneBreakup breakup;

    static readonly Color Cbg = new Color(0.02f, 0.04f, 0.07f, 0.92f);
    static readonly Color Cacc = new Color(0.25f, 0.9f, 1f);
    static readonly Color Cok = new Color(0.3f, 1f, 0.45f);
    static readonly Color Cwarn = new Color(1f, 0.75f, 0.2f);
    static readonly Color Cerr = new Color(1f, 0.3f, 0.3f);
    static readonly Color Cdim = new Color(0.72f, 0.78f, 0.84f);
    static readonly Color Cp = new Color(1f, 0.35f, 0.7f);
    static readonly Color Cb = new Color(0.4f, 0.6f, 1f);
    static readonly Color Cv = new Color(0.7f, 0.5f, 1f);
    static readonly Color Cy = new Color(1f, 0.85f, 0.2f);
    static readonly Color Co = new Color(1f, 0.6f, 0.2f);

    // состояние для журнала
    bool lastArmed, lastCrashed, lastFs, lastLow, lastCrit, lastVrs, lastBroken;
    FlightMode lastMode;

    void Start()
    {
        fc = FindObjectOfType<FlightController>();
        ph = FindObjectOfType<QuadPhysics>();
        bat = FindObjectOfType<BatteryModel>();
        wind = FindObjectOfType<WindSystem>();
        breakup = FindObjectOfType<DroneBreakup>();
        enabled = fc != null && ph != null;
        if (!enabled) return;

        lastMode = fc.Mode;
        if (ph.OnCrash != null) { } // событие подписывается ниже
        ph.OnCrash += () => Log("CRASH — удар", Cerr);

        groups = new List<Group>
        {
            new Group { title = "altitude (m) · climb rate (m/s)", open = true, series = new[]
            {
                new Series { name = "alt", color = Cacc, get = s => s.alt },
                new Series { name = "vs",  color = Cb,   get = s => s.vs  }
            }},
            new Group { title = "body rates p/q/r (°/s)", open = true, series = new[]
            {
                new Series { name = "p", color = Cp, get = s => s.p },
                new Series { name = "q", color = Cok, get = s => s.q },
                new Series { name = "r", color = Cb, get = s => s.r }
            }},
            new Group { title = "wind X/Z (m/s) · ground effect", open = true, series = new[]
            {
                new Series { name = "wx", color = Cv, get = s => s.wx },
                new Series { name = "wz", color = new Color(0.5f, 0.4f, 0.9f), get = s => s.wz },
                new Series { name = "GE", color = Cy, get = s => s.ge }
            }},
            new Group { title = "rotor rpm (об/мин)", open = true, series = new[]
            {
                new Series { name = "M1", color = Cacc, get = s => s.rpm[0] },
                new Series { name = "M2", color = Cok,  get = s => s.rpm[1] },
                new Series { name = "M3", color = Cp,   get = s => s.rpm[2] },
                new Series { name = "M4", color = Co,   get = s => s.rpm[3] }
            }},
            new Group { title = "motor duty (%)", open = false, series = new[]
            {
                new Series { name = "M1", color = Cacc, get = s => s.duty[0] },
                new Series { name = "M2", color = Cok,  get = s => s.duty[1] },
                new Series { name = "M3", color = Cp,   get = s => s.duty[2] },
                new Series { name = "M4", color = Co,   get = s => s.duty[3] }
            }},
            new Group { title = "battery V / A", open = false, series = new[]
            {
                new Series { name = "V", color = Cok,  get = s => s.vbat },
                new Series { name = "A", color = Cwarn, get = s => s.amps }
            }},
            new Group { title = "attitude roll/pitch (°)", open = false, series = new[]
            {
                new Series { name = "roll",  color = Cacc, get = s => s.roll },
                new Series { name = "pitch", color = Cp,   get = s => s.pitch }
            }},
        };
        Log("панель телеметрии запущена", Cdim);
    }

    void Update()
    {
        bool key = false;
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb != null) key = kb.f7Key.wasPressedThisFrame;
#elif ENABLE_LEGACY_INPUT_MANAGER
        key = Input.GetKeyDown(KeyCode.F7);
#endif
        if (key) show = !show;
        if (fc == null) return;

        PollEvents();

        sampleTimer -= Time.unscaledDeltaTime;
        if (sampleTimer > 0f) return;
        sampleTimer = 1f / Mathf.Max(1, sampleRateHz);

        if (!paused) PushSample();
        if (show && tab == 0)
            foreach (var g in groups) if (g.open) Plot(g);
    }

    int Capacity => Mathf.Max(16, sampleRateHz * windowSeconds);

    void PushSample()
    {
        var s = new Sample
        {
            t = Time.time,
            alt = fc.AltAgl,
            vs = fc.Vs,
            roll = fc.AttRoll,
            pitch = fc.AttPitchUp,
            vbat = bat != null ? bat.Voltage : 0f,
            amps = bat != null ? bat.Current : 0f,
            ge = ph.GroundEffectFactor,
            wx = wind != null ? wind.Current.x : 0f,
            wz = wind != null ? wind.Current.z : 0f
        };
        Vector3 w = ph.Rb != null
            ? ph.transform.InverseTransformDirection(ph.Rb.angularVelocity) * Mathf.Rad2Deg
            : Vector3.zero;
        s.p = -w.z;   // крен (после исправления знаков)
        s.q = w.x;    // тангаж
        s.r = w.y;    // рыскание
        for (int i = 0; i < 4; i++)
        {
            s.rpm[i] = ph.motors[i].rpm;
            s.duty[i] = ph.motors[i].cmd * 100f;
        }
        samples.Enqueue(s);
        while (samples.Count > Capacity) samples.Dequeue();
    }

    // ---------- отрисовка графика в текстуру ----------
    void EnsureTex(Group g)
    {
        if (g.tex != null && g.tex.width == graphWidth && g.tex.height == graphHeight) return;
        if (g.tex != null) Destroy(g.tex);
        g.tex = new Texture2D(graphWidth, graphHeight, TextureFormat.RGBA32, false)
        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
        g.px = new Color32[graphWidth * graphHeight];
    }

    void Plot(Group g)
    {
        EnsureTex(g);
        int W = g.tex.width, H = g.tex.height;
        var px = g.px;
        Color32 bg = new Color32(6, 10, 16, 255);
        for (int i = 0; i < px.Length; i++) px[i] = bg;
        Color32 grid = new Color32(22, 30, 40, 255);
        for (int gy = 1; gy < 4; gy++) { int y = H * gy / 4; for (int x = 0; x < W; x++) px[y * W + x] = grid; }

        int n = samples.Count;
        if (n < 2) { g.tex.SetPixels32(px); g.tex.Apply(false); return; }

        float tmin = float.MaxValue, tmax = float.MinValue;
        foreach (var s in samples)
            foreach (var se in g.series)
            {
                float v = se.get(s);
                if (v < tmin) tmin = v;
                if (v > tmax) tmax = v;
            }
        if (tmin > tmax) { tmin = 0; tmax = 1; }
        float pad = (tmax - tmin) * 0.15f + 0.001f;
        tmin -= pad; tmax += pad;
        if (autoScale)
        {
            if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
            else { g.min = Mathf.Lerp(g.min, tmin, 0.25f); g.max = Mathf.Lerp(g.max, tmax, 0.25f); }
        }
        else if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }

        float range = Mathf.Max(1e-4f, g.max - g.min);
        int cap = Capacity;
        foreach (var se in g.series)
        {
            Color32 col = se.color;
            int idx = 0, px0 = 0, py0 = 0;
            foreach (var s in samples)
            {
                int x = idx * (W - 1) / Mathf.Max(1, cap - 1);
                int y = Mathf.Clamp(Mathf.RoundToInt((se.get(s) - g.min) / range * (H - 1)), 0, H - 1);
                if (idx > 0) Line(px, W, H, px0, py0, x, y, col);
                px0 = x; py0 = y; idx++;
                se.current = se.get(s);
            }
        }
        g.tex.SetPixels32(px);
        g.tex.Apply(false);
    }

    static void Line(Color32[] px, int W, int H, int x0, int y0, int x1, int y1, Color32 c)
    {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (x0 >= 0 && x0 < W && y0 >= 0 && y0 < H) px[y0 * W + x0] = c;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ---------- журнал ----------
    void Log(string text, Color c)
    {
        events.Add(new Evt { t = Time.time, text = text, c = c });
        while (events.Count > 300) events.RemoveAt(0);
    }
    void PollEvents()
    {
        if (fc.Armed != lastArmed) { lastArmed = fc.Armed; Log(fc.Armed ? "ARM" : "DISARM", fc.Armed ? Cok : Cdim); }
        if (fc.Mode != lastMode) { lastMode = fc.Mode; Log("MODE → " + fc.Mode, Cacc); }
        if (ph.Crashed != lastCrashed) { lastCrashed = ph.Crashed; if (ph.Crashed) Log("CRASHED", Cerr); else Log("REPAIRED / RESET", Cok); }
        if (fc.Failsafe != lastFs) { lastFs = fc.Failsafe; Log(fc.Failsafe ? "FAILSAFE ON" : "FAILSAFE OFF", fc.Failsafe ? Cerr : Cok); }
        if (bat != null)
        {
            if (bat.IsLow != lastLow) { lastLow = bat.IsLow; if (bat.IsLow) Log("LOW BATTERY", Cwarn); }
            if (bat.IsCritical != lastCrit) { lastCrit = bat.IsCritical; if (bat.IsCritical) Log("BATTERY CRITICAL", Cerr); }
        }
        if (ph.VrsActive != lastVrs) { lastVrs = ph.VrsActive; if (ph.VrsActive) Log("VORTEX RING STATE", Cwarn); }
        if (breakup != null && breakup.IsBroken != lastBroken)
        { lastBroken = breakup.IsBroken; Log(lastBroken ? "BREAKUP — дрон разобран" : "REASSEMBLED", lastBroken ? Cerr : Cok); }
    }

    // ---------- GUI ----------
    void OnGUI()
    {
        if (!show || fc == null) return;
        var saved = GUI.matrix;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        winStyle = winStyle ?? new GUIStyle(GUI.skin.window);
        try { windowRect = GUI.Window(77, windowRect, DrawWindow, "ТЕЛЕМЕТРИЯ   [F7]", winStyle); }
        catch (System.Exception e) { show = false; UnityEngine.Debug.LogError("[Telemetry] " + e); }
        GUI.matrix = saved;
    }

    void DrawWindow(int id)
    {
        headStyle = headStyle ?? new GUIStyle(GUI.skin.button) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
        bigStyle = bigStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 30, fontStyle = FontStyle.Bold };
        smallStyle = smallStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 11 };

        GUILayout.BeginHorizontal();
        string[] tabs = { "LIVE TRACES", "ROTOR BENCH", "EVENT LOG" };
        for (int i = 0; i < tabs.Length; i++)
        {
            GUI.color = tab == i ? Cacc : Color.white;
            if (GUILayout.Button(tabs[i], GUILayout.Height(26))) tab = i;
        }
        GUI.color = Color.white;
        GUILayout.FlexibleSpace();
        GUI.color = paused ? Cwarn : Cok;
        if (GUILayout.Button(paused ? "● REC" : "⏸ PAUSE", GUILayout.Width(90), GUILayout.Height(26))) paused = !paused;
        GUI.color = Color.white;
        if (GUILayout.Button("CLEAR", GUILayout.Width(70), GUILayout.Height(26))) { samples.Clear(); events.Clear(); foreach (var g in groups) g.scaled = false; }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("окно:", GUILayout.Width(45));
        int[] secs = { 10, 30, 60 };
        foreach (var s in secs)
        {
            GUI.color = windowSeconds == s ? Cacc : Color.white;
            if (GUILayout.Button(s + " с", GUILayout.Width(56))) windowSeconds = s;
        }
        GUI.color = Color.white;
        autoScale = GUILayout.Toggle(autoScale, "AUTOSCALE", GUILayout.Width(110));
        GUILayout.Label($"  образцов: {samples.Count}/{Capacity}   {sampleRateHz} Гц", smallStyle);
        GUILayout.EndHorizontal();
        GUILayout.Space(4);

        if (tab == 0) DrawTraces();
        else if (tab == 1) DrawRotors();
        else DrawLog();

        GUI.DragWindow();
    }

    void DrawTraces()
    {
        scrollTraces = GUILayout.BeginScrollView(scrollTraces);
        foreach (var g in groups)
        {
            if (GUILayout.Button((g.open ? "▼  " : "►  ") + g.title, headStyle, GUILayout.Height(24)))
            { g.open = !g.open; if (g.open) Plot(g); }
            if (!g.open) continue;

            GUILayout.BeginHorizontal();
            foreach (var se in g.series)
            {
                GUI.color = se.color;
                GUILayout.Label(se.current.ToString("0.00") + " " + se.name, GUILayout.Width(130));
            }
            GUI.color = Cdim;
            GUILayout.FlexibleSpace();
            GUILayout.Label($"[{g.min:0.0} … {g.max:0.0}]", smallStyle, GUILayout.Width(150));
            GUI.color = Color.white;
            GUILayout.EndHorizontal();

            Rect r = GUILayoutUtility.GetRect(graphWidth, graphHeight, GUILayout.ExpandWidth(true));
            if (g.tex != null) GUI.DrawTexture(r, g.tex);
            GUILayout.Space(6);
        }
        GUILayout.EndScrollView();
    }

    void DrawRotors()
    {
        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        for (int i = 0; i < 4; i++)
        {
            var m = ph.motors[i];
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(232));

            string status; Color sc;
            if (m.cmd > 0.97f) { status = "SATURATED"; sc = Cwarn; }
            else if (m.vrs) { status = "VRS"; sc = Cwarn; }
            else if (m.rpm < 100f) { status = "OFF"; sc = Cdim; }
            else { status = "RUN"; sc = Cok; }

            GUILayout.BeginHorizontal();
            GUILayout.Label($"M{i + 1} · {(m.spinDir > 0 ? "CW" : "CCW")}", GUILayout.Width(110));
            GUI.color = sc; GUILayout.Label(status, smallStyle); GUI.color = Color.white;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(m.rpm.ToString("N0"), bigStyle, GUILayout.Width(140));
            GUILayout.Label("RPM", smallStyle, GUILayout.Width(40));
            GUILayout.EndHorizontal();

            Rect bar = GUILayoutUtility.GetRect(210, 8, GUILayout.ExpandWidth(false));
            GUI.color = new Color(1, 1, 1, 0.1f); GUI.DrawTexture(bar, Texture2D.whiteTexture);
            GUI.color = m.cmd > 0.97f ? Cwarn : Cok;
            GUI.DrawTexture(new Rect(bar.x, bar.y, bar.width * Mathf.Clamp01(m.cmd), bar.height), Texture2D.whiteTexture);
            GUI.color = Color.white;

            float eff = ph.gramsPerWatt * (1.15f - 0.5f * m.cmd);
            float power = (m.thrustN / 0.00981f) / Mathf.Max(1f, eff);
            float amps = bat != null ? power / Mathf.Max(1f, bat.Voltage) : 0f;
            float ige = Ige(m);

            GUILayout.Label($"T {m.thrustN:0.00} N     Q {m.torqueNm:0.0000}", smallStyle);
            GUILayout.Label($"I {amps:0.0} A       DUTY {m.cmd * 100f:0}%", smallStyle);
            GUILayout.Label($"IGE {ige:0.000}     VRS {(m.vrs ? 1f : 0f):0.00}", smallStyle);
            GUILayout.Label($"h {m.groundH:0.00} m    ω {m.rpm * 0.10472f:0} rad/s", smallStyle);
            GUILayout.EndVertical();
            if (i < 3) GUILayout.Space(4);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(10);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Суммарная тяга: {ph.TotalThrustN:0.0} Н  ({ph.TotalThrustN / 0.00981f:0} г)", GUILayout.Width(300));
        GUILayout.Label($"Мощность: {ph.PowerW:0} Вт", GUILayout.Width(200));
        if (bat != null) GUILayout.Label($"Батарея: {bat.Voltage:0.00} В  {bat.Current:0.0} А  {bat.Soc * 100f:0}%", GUILayout.Width(300));
        GUILayout.EndHorizontal();
        GUILayout.Label($"Ground effect (средний): x{ph.GroundEffectFactor:0.000}   Воздушная скорость: {ph.Airspeed:0.0} м/с", smallStyle);
    }

    float Ige(MotorUnit m)
    {
        if (!ph.groundEffect) return 1f;
        float R = ph.propRadius;
        if (m.groundH >= 4f * R) return 1f;
        float h = Mathf.Max(m.groundH, R * 0.35f);
        float r = R / (4f * h);
        return Mathf.Clamp(1f / (1f - r * r), 1f, ph.groundEffectMax);
    }

    void DrawLog()
    {
        scrollLog = GUILayout.BeginScrollView(scrollLog);
        for (int i = events.Count - 1; i >= 0; i--)
        {
            var e = events[i];
            GUILayout.BeginHorizontal();
            GUILayout.Label(e.t.ToString("0.0") + " s", smallStyle, GUILayout.Width(70));
            GUI.color = e.c; GUILayout.Label(e.text); GUI.color = Color.white;
            GUILayout.EndHorizontal();
        }
        GUILayout.EndScrollView();
    }

    void OnDestroy()
    {
        if (groups != null) foreach (var g in groups) if (g.tex != null) Destroy(g.tex);
    }
}

### ASSISTANT
Ошибка в одной строке — `if (ph.OnCrash != null) { }`: событие в C# можно только подписывать/отписывать (`+=` / `-=`) извне класса, читать его нельзя. Эта строка была лишней (артефакт), её нужно просто удалить.

Заодно я укрепил скрипт: подписался на краш **именованным обработчиком** и отписываюсь в `OnDestroy` (иначе после уничтожения панели обработчик продолжал бы висеть на дроне), добавил null-защиты и убрал тяжёлую отрисовку текстур из `OnGUI` — теперь графики строятся только в `Update`.

## Полный исправленный `SimTelemetryPanel.cs`

```csharp
using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(110)]
public class SimTelemetryPanel : MonoBehaviour
{
    [Header("Окно (F7 — показать/скрыть)")]
    public bool show = true;
    public Rect windowRect = new Rect(330, 110, 1010, 660);

    [Header("Запись")]
    [Tooltip("Частота выборки телеметрии, Гц")] [Range(5, 60)] public int sampleRateHz = 20;
    [Tooltip("Длина окна графиков, с")] [Range(10, 120)] public int windowSeconds = 30;
    public bool paused;

    [Header("Графики")]
    [Range(256, 1024)] public int graphWidth = 560;
    [Range(40, 160)] public int graphHeight = 70;
    public bool autoScale = true;

    // ---------- данные ----------
    class Sample
    {
        public float t, alt, vs, p, q, r, wx, wz, ge, roll, pitch, vbat, amps;
        public readonly float[] rpm = new float[4];
        public readonly float[] duty = new float[4];
    }
    class Series
    {
        public string name; public Color color;
        public System.Func<Sample, float> get;
        public float current;
    }
    class Group
    {
        public string title; public bool open; public Series[] series;
        public Texture2D tex; public Color32[] px;
        public float min, max; public bool scaled;
    }
    struct Evt { public float t; public string text; public Color c; }

    readonly Queue<Sample> samples = new Queue<Sample>();
    readonly List<Evt> events = new List<Evt>();
    List<Group> groups;
    int tab; float sampleTimer; Vector2 scrollTraces, scrollLog;
    GUIStyle winStyle, headStyle, bigStyle, smallStyle;

    FlightController fc; QuadPhysics ph; BatteryModel bat; WindSystem wind; DroneBreakup breakup;
    System.Action crashHandler;
    bool subscribed;

    static readonly Color Cacc = new Color(0.25f, 0.9f, 1f);
    static readonly Color Cok = new Color(0.3f, 1f, 0.45f);
    static readonly Color Cwarn = new Color(1f, 0.75f, 0.2f);
    static readonly Color Cerr = new Color(1f, 0.3f, 0.3f);
    static readonly Color Cdim = new Color(0.72f, 0.78f, 0.84f);
    static readonly Color Cp = new Color(1f, 0.35f, 0.7f);
    static readonly Color Cb = new Color(0.4f, 0.6f, 1f);
    static readonly Color Cv = new Color(0.7f, 0.5f, 1f);
    static readonly Color Cy = new Color(1f, 0.85f, 0.2f);
    static readonly Color Co = new Color(1f, 0.6f, 0.2f);

    // состояние для журнала
    bool lastArmed, lastCrashed, lastFs, lastLow, lastCrit, lastVrs, lastBroken;
    FlightMode lastMode;

    void Start()
    {
        fc = FindObjectOfType<FlightController>();
        ph = FindObjectOfType<QuadPhysics>();
        bat = FindObjectOfType<BatteryModel>();
        wind = FindObjectOfType<WindSystem>();
        breakup = FindObjectOfType<DroneBreakup>();
        enabled = fc != null && ph != null;
        if (!enabled) return;

        lastMode = fc.Mode;
        crashHandler = () => Log("CRASH — удар", Cerr);
        ph.OnCrash += crashHandler;
        subscribed = true;

        groups = new List<Group>
        {
            new Group { title = "altitude (m) · climb rate (m/s)", open = true, series = new[]
            {
                new Series { name = "alt", color = Cacc, get = s => s.alt },
                new Series { name = "vs",  color = Cb,   get = s => s.vs  }
            }},
            new Group { title = "body rates p/q/r (°/s)", open = true, series = new[]
            {
                new Series { name = "p", color = Cp, get = s => s.p },
                new Series { name = "q", color = Cok, get = s => s.q },
                new Series { name = "r", color = Cb, get = s => s.r }
            }},
            new Group { title = "wind X/Z (m/s) · ground effect", open = true, series = new[]
            {
                new Series { name = "wx", color = Cv, get = s => s.wx },
                new Series { name = "wz", color = new Color(0.5f, 0.4f, 0.9f), get = s => s.wz },
                new Series { name = "GE", color = Cy, get = s => s.ge }
            }},
            new Group { title = "rotor rpm (об/мин)", open = true, series = new[]
            {
                new Series { name = "M1", color = Cacc, get = s => s.rpm[0] },
                new Series { name = "M2", color = Cok,  get = s => s.rpm[1] },
                new Series { name = "M3", color = Cp,   get = s => s.rpm[2] },
                new Series { name = "M4", color = Co,   get = s => s.rpm[3] }
            }},
            new Group { title = "motor duty (%)", open = false, series = new[]
            {
                new Series { name = "M1", color = Cacc, get = s => s.duty[0] },
                new Series { name = "M2", color = Cok,  get = s => s.duty[1] },
                new Series { name = "M3", color = Cp,   get = s => s.duty[2] },
                new Series { name = "M4", color = Co,   get = s => s.duty[3] }
            }},
            new Group { title = "battery V / A", open = false, series = new[]
            {
                new Series { name = "V", color = Cok,  get = s => s.vbat },
                new Series { name = "A", color = Cwarn, get = s => s.amps }
            }},
            new Group { title = "attitude roll/pitch (°)", open = false, series = new[]
            {
                new Series { name = "roll",  color = Cacc, get = s => s.roll },
                new Series { name = "pitch", color = Cp,   get = s => s.pitch }
            }},
        };
        Log("панель телеметрии запущена", Cdim);
    }

    void Update()
    {
        bool key = false;
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb != null) key = kb.f7Key.wasPressedThisFrame;
#elif ENABLE_LEGACY_INPUT_MANAGER
        key = Input.GetKeyDown(KeyCode.F7);
#endif
        if (key) show = !show;
        if (fc == null || ph == null || groups == null) return;

        PollEvents();

        sampleTimer -= Time.unscaledDeltaTime;
        if (sampleTimer > 0f) return;
        sampleTimer = 1f / Mathf.Max(1, sampleRateHz);

        if (!paused) PushSample();
        if (show && tab == 0)
            foreach (var g in groups) if (g.open) Plot(g);
    }

    int Capacity => Mathf.Max(16, sampleRateHz * windowSeconds);

    void PushSample()
    {
        var s = new Sample
        {
            t = Time.time,
            alt = fc.AltAgl,
            vs = fc.Vs,
            roll = fc.AttRoll,
            pitch = fc.AttPitchUp,
            vbat = bat != null ? bat.Voltage : 0f,
            amps = bat != null ? bat.Current : 0f,
            ge = ph.GroundEffectFactor,
            wx = wind != null ? wind.Current.x : 0f,
            wz = wind != null ? wind.Current.z : 0f
        };
        Vector3 w = ph.Rb != null
            ? ph.transform.InverseTransformDirection(ph.Rb.angularVelocity) * Mathf.Rad2Deg
            : Vector3.zero;
        s.p = -w.z;   // крен
        s.q = w.x;    // тангаж
        s.r = w.y;    // рыскание
        for (int i = 0; i < 4; i++)
        {
            s.rpm[i] = ph.motors[i].rpm;
            s.duty[i] = ph.motors[i].cmd * 100f;
        }
        samples.Enqueue(s);
        while (samples.Count > Capacity) samples.Dequeue();
    }

    // ---------- отрисовка графика в текстуру ----------
    void EnsureTex(Group g)
    {
        if (g.tex != null && g.tex.width == graphWidth && g.tex.height == graphHeight) return;
        if (g.tex != null) Destroy(g.tex);
        g.tex = new Texture2D(graphWidth, graphHeight, TextureFormat.RGBA32, false)
        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
        g.px = new Color32[graphWidth * graphHeight];
    }

    void Plot(Group g)
    {
        EnsureTex(g);
        int W = g.tex.width, H = g.tex.height;
        var px = g.px;
        Color32 bg = new Color32(6, 10, 16, 255);
        for (int i = 0; i < px.Length; i++) px[i] = bg;
        Color32 grid = new Color32(22, 30, 40, 255);
        for (int gy = 1; gy < 4; gy++) { int y = H * gy / 4; for (int x = 0; x < W; x++) px[y * W + x] = grid; }

        int n = samples.Count;
        if (n < 2) { g.tex.SetPixels32(px); g.tex.Apply(false); return; }

        float tmin = float.MaxValue, tmax = float.MinValue;
        foreach (var s in samples)
            foreach (var se in g.series)
            {
                float v = se.get(s);
                if (v < tmin) tmin = v;
                if (v > tmax) tmax = v;
            }
        if (tmin > tmax) { tmin = 0; tmax = 1; }
        float pad = (tmax - tmin) * 0.15f + 0.001f;
        tmin -= pad; tmax += pad;
        if (autoScale)
        {
            if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
            else { g.min = Mathf.Lerp(g.min, tmin, 0.25f); g.max = Mathf.Lerp(g.max, tmax, 0.25f); }
        }
        else if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }

        float range = Mathf.Max(1e-4f, g.max - g.min);
        int cap = Capacity;
        foreach (var se in g.series)
        {
            Color32 col = se.color;
            int idx = 0, px0 = 0, py0 = 0;
            foreach (var s in samples)
            {
                int x = idx * (W - 1) / Mathf.Max(1, cap - 1);
                int y = Mathf.Clamp(Mathf.RoundToInt((se.get(s) - g.min) / range * (H - 1)), 0, H - 1);
                if (idx > 0) Line(px, W, H, px0, py0, x, y, col);
                px0 = x; py0 = y; idx++;
                se.current = se.get(s);
            }
        }
        g.tex.SetPixels32(px);
        g.tex.Apply(false);
    }

    static void Line(Color32[] px, int W, int H, int x0, int y0, int x1, int y1, Color32 c)
    {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (x0 >= 0 && x0 < W && y0 >= 0 && y0 < H) px[y0 * W + x0] = c;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ---------- журнал ----------
    void Log(string text, Color c)
    {
        events.Add(new Evt { t = Time.time, text = text, c = c });
        while (events.Count > 300) events.RemoveAt(0);
    }

    void PollEvents()
    {
        if (fc.Armed != lastArmed) { lastArmed = fc.Armed; Log(fc.Armed ? "ARM" : "DISARM", fc.Armed ? Cok : Cdim); }
        if (fc.Mode != lastMode) { lastMode = fc.Mode; Log("MODE → " + fc.Mode, Cacc); }
        if (ph.Crashed != lastCrashed) { lastCrashed = ph.Crashed; if (ph.Crashed) Log("CRASHED", Cerr); else Log("REPAIRED / RESET", Cok); }
        if (fc.Failsafe != lastFs) { lastFs = fc.Failsafe; Log(fc.Failsafe ? "FAILSAFE ON" : "FAILSAFE OFF", fc.Failsafe ? Cerr : Cok); }
        if (bat != null)
        {
            if (bat.IsLow != lastLow) { lastLow = bat.IsLow; if (bat.IsLow) Log("LOW BATTERY", Cwarn); }
            if (bat.IsCritical != lastCrit) { lastCrit = bat.IsCritical; if (bat.IsCritical) Log("BATTERY CRITICAL", Cerr); }
        }
        if (ph.VrsActive != lastVrs) { lastVrs = ph.VrsActive; if (ph.VrsActive) Log("VORTEX RING STATE", Cwarn); }
        if (breakup != null && breakup.IsBroken != lastBroken)
        { lastBroken = breakup.IsBroken; Log(lastBroken ? "BREAKUP — дрон разобран" : "REASSEMBLED", lastBroken ? Cerr : Cok); }
    }

    // ---------- GUI ----------
    void OnGUI()
    {
        if (!show || fc == null || groups == null) return;
        var saved = GUI.matrix;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        winStyle = winStyle ?? new GUIStyle(GUI.skin.window);
        try { windowRect = GUI.Window(77, windowRect, DrawWindow, "ТЕЛЕМЕТРИЯ   [F7]", winStyle); }
        catch (System.Exception e) { show = false; UnityEngine.Debug.LogError("[Telemetry] " + e); }
        GUI.matrix = saved;
    }

    void DrawWindow(int id)
    {
        headStyle = headStyle ?? new GUIStyle(GUI.skin.button) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
        bigStyle = bigStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 30, fontStyle = FontStyle.Bold };
        smallStyle = smallStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 11 };

        GUILayout.BeginHorizontal();
        string[] tabs = { "LIVE TRACES", "ROTOR BENCH", "EVENT LOG" };
        for (int i = 0; i < tabs.Length; i++)
        {
            GUI.color = tab == i ? Cacc : Color.white;
            if (GUILayout.Button(tabs[i], GUILayout.Height(26))) tab = i;
        }
        GUI.color = Color.white;
        GUILayout.FlexibleSpace();
        GUI.color = paused ? Cwarn : Cok;
        if (GUILayout.Button(paused ? "● REC" : "⏸ PAUSE", GUILayout.Width(90), GUILayout.Height(26))) paused = !paused;
        GUI.color = Color.white;
        if (GUILayout.Button("CLEAR", GUILayout.Width(70), GUILayout.Height(26)))
        { samples.Clear(); events.Clear(); foreach (var g in groups) g.scaled = false; }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("окно:", GUILayout.Width(45));
        int[] secs = { 10, 30, 60 };
        foreach (var s in secs)
        {
            GUI.color = windowSeconds == s ? Cacc : Color.white;
            if (GUILayout.Button(s + " с", GUILayout.Width(56))) windowSeconds = s;
        }
        GUI.color = Color.white;
        autoScale = GUILayout.Toggle(autoScale, "AUTOSCALE", GUILayout.Width(110));
        GUILayout.Label($"  образцов: {samples.Count}/{Capacity}   {sampleRateHz} Гц", smallStyle);
        GUILayout.EndHorizontal();
        GUILayout.Space(4);

        if (tab == 0) DrawTraces();
        else if (tab == 1) DrawRotors();
        else DrawLog();

        GUI.DragWindow();
    }

    void DrawTraces()
    {
        scrollTraces = GUILayout.BeginScrollView(scrollTraces);
        foreach (var g in groups)
        {
            if (GUILayout.Button((g.open ? "▼  " : "►  ") + g.title, headStyle, GUILayout.Height(24)))
            { g.open = !g.open; g.scaled = false; }   // отрисовка произойдёт в Update
            if (!g.open) continue;

            GUILayout.BeginHorizontal();
            foreach (var se in g.series)
            {
                GUI.color = se.color;
                GUILayout.Label(se.current.ToString("0.00") + " " + se.name, GUILayout.Width(130));
            }
            GUI.color = Cdim;
            GUILayout.FlexibleSpace();
            GUILayout.Label($"[{g.min:0.0} … {g.max:0.0}]", smallStyle, GUILayout.Width(150));
            GUI.color = Color.white;
            GUILayout.EndHorizontal();

            Rect r = GUILayoutUtility.GetRect(graphWidth, graphHeight, GUILayout.ExpandWidth(true));
            if (g.tex != null) GUI.DrawTexture(r, g.tex);
            GUILayout.Space(6);
        }
        GUILayout.EndScrollView();
    }

    void DrawRotors()
    {
        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        for (int i = 0; i < 4; i++)
        {
            var m = ph.motors[i];
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(232));

            string status; Color sc;
            if (m.cmd > 0.97f) { status = "SATURATED"; sc = Cwarn; }
            else if (m.vrs) { status = "VRS"; sc = Cwarn; }
            else if (m.rpm < 100f) { status = "OFF"; sc = Cdim; }
            else { status = "RUN"; sc = Cok; }

            GUILayout.BeginHorizontal();
            GUILayout.Label($"M{i + 1} · {(m.spinDir > 0 ? "CW" : "CCW")}", GUILayout.Width(110));
            GUI.color = sc; GUILayout.Label(status, smallStyle); GUI.color = Color.white;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(m.rpm.ToString("N0"), bigStyle, GUILayout.Width(140));
            GUILayout.Label("RPM", smallStyle, GUILayout.Width(40));
            GUILayout.EndHorizontal();

            Rect bar = GUILayoutUtility.GetRect(210, 8, GUILayout.ExpandWidth(false));
            GUI.color = new Color(1, 1, 1, 0.1f); GUI.DrawTexture(bar, Texture2D.whiteTexture);
            GUI.color = m.cmd > 0.97f ? Cwarn : Cok;
            GUI.DrawTexture(new Rect(bar.x, bar.y, bar.width * Mathf.Clamp01(m.cmd), bar.height), Texture2D.whiteTexture);
            GUI.color = Color.white;

            float eff = ph.gramsPerWatt * (1.15f - 0.5f * m.cmd);
            float power = (m.thrustN / 0.00981f) / Mathf.Max(1f, eff);
            float amps = bat != null ? power / Mathf.Max(1f, bat.Voltage) : 0f;
            float ige = Ige(m);

            GUILayout.Label($"T {m.thrustN:0.00} N     Q {m.torqueNm:0.0000}", smallStyle);
            GUILayout.Label($"I {amps:0.0} A       DUTY {m.cmd * 100f:0}%", smallStyle);
            GUILayout.Label($"IGE {ige:0.000}     VRS {(m.vrs ? 1f : 0f):0.00}", smallStyle);
            GUILayout.Label($"h {m.groundH:0.00} m    ω {m.rpm * 0.10472f:0} rad/s", smallStyle);
            GUILayout.EndVertical();
            if (i < 3) GUILayout.Space(4);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(10);
        GUILayout.BeginHorizontal();
        GUILayout.Label($"Суммарная тяга: {ph.TotalThrustN:0.0} Н  ({ph.TotalThrustN / 0.00981f:0} г)", GUILayout.Width(300));
        GUILayout.Label($"Мощность: {ph.PowerW:0} Вт", GUILayout.Width(200));
        if (bat != null) GUILayout.Label($"Батарея: {bat.Voltage:0.00} В  {bat.Current:0.0} А  {bat.Soc * 100f:0}%", GUILayout.Width(300));
        GUILayout.EndHorizontal();
        GUILayout.Label($"Ground effect (средний): x{ph.GroundEffectFactor:0.000}   Воздушная скорость: {ph.Airspeed:0.0} м/с", smallStyle);
    }

    float Ige(MotorUnit m)
    {
        if (!ph.groundEffect) return 1f;
        float R = ph.propRadius;
        if (m.groundH >= 4f * R) return 1f;
        float h = Mathf.Max(m.groundH, R * 0.35f);
        float r = R / (4f * h);
        return Mathf.Clamp(1f / (1f - r * r), 1f, ph.groundEffectMax);
    }

    void DrawLog()
    {
        scrollLog = GUILayout.BeginScrollView(scrollLog);
        for (int i = events.Count - 1; i >= 0; i--)
        {
            var e = events[i];
            GUILayout.BeginHorizontal();
            GUILayout.Label(e.t.ToString("0.0") + " s", smallStyle, GUILayout.Width(70));
            GUI.color = e.c; GUILayout.Label(e.text); GUI.color = Color.white;
            GUILayout.EndHorizontal();
        }
        GUILayout.EndScrollView();
    }

    void OnDestroy()
    {
        if (subscribed && ph != null && crashHandler != null) ph.OnCrash -= crashHandler;
        subscribed = false;
        if (groups != null) foreach (var g in groups) if (g.tex != null) Destroy(g.tex);
    }
}
```

Что изменилось по сравнению с твоей версией:

- **удалена** строка `if (ph.OnCrash != null) { }` — именно она давала CS0070;
- подписка на краш идёт через поле `crashHandler` и снимается в `OnDestroy` (нет утечки обработчика и повторных записей в журнал);
- `Update`/`OnGUI` защищены от `groups == null` (если дрон не найден, панель просто молчит);
- при раскрытии группы больше не вызывается `Plot()` внутри GUI — только сброс масштаба, а перерисовка идёт в `Update` (меньше нагрузки и никаких сюрпризов IMGUI);
- убран неиспользуемый цвет `Cbg`.

Сохрани файл целиком, дождись компиляции — F7 откроет панель с графиками и оборотами каждого винта.

### USER
теперь еще возникает такая ошибка Assets\DroneSimEditorTools.cs(57,13): error CS0103: The name 'hg' does not exist in the current context

вот в этот методе:   public static void RebuildDrone()
 {
     var old = Object.FindObjectOfType<QuadPhysics>();
     Vector3 spawn = old != null ? old.transform.position : new Vector3(0, 0.12f, 0);
     if (old != null) Undo.DestroyObjectImmediate(old.gameObject);

     Undo.IncrementCurrentGroup();
     Undo.SetCurrentGroupName("Rebuild Drone");
     var ctx = EditorContext(); ctx.spawn = spawn;
     var drone = DroneSceneBuilder.BuildDrone(ctx);
     DroneSceneBuilder.BuildCameraAndHud(ctx, drone.transform);
     if (UnityEngine.Object.FindObjectOfType<SimTelemetryPanel>() == null)
         hg.AddComponent<SimTelemetryPanel>();
     DroneSceneBuilder.WireReferences();
     MarkDirty();
     Selection.activeGameObject = drone;
 }

### ASSISTANT
`hg` — это локальная переменная **внутри** `DroneSceneBuilder.BuildCameraAndHud(...)`, в методе `RebuildDrone()` её не существует. Я в прошлом сообщении неудачно сформулировал место вставки — панель нужно добавлять именно в `BuildCameraAndHud`, а не в `RebuildDrone`.

## 1. `DroneSimEditorTools.cs` — исправленный `RebuildDrone()`

Просто убери две строки с `hg`:

```csharp
    public static void RebuildDrone()
    {
        var old = Object.FindObjectOfType<QuadPhysics>();
        Vector3 spawn = old != null ? old.transform.position : new Vector3(0, 0.12f, 0);
        if (old != null) Undo.DestroyObjectImmediate(old.gameObject);

        Undo.IncrementCurrentGroup();
        Undo.SetCurrentGroupName("Rebuild Drone");
        var ctx = EditorContext(); ctx.spawn = spawn;
        var drone = DroneSceneBuilder.BuildDrone(ctx);
        DroneSceneBuilder.BuildCameraAndHud(ctx, drone.transform);
        DroneSceneBuilder.WireReferences();
        MarkDirty();
        Selection.activeGameObject = drone;
    }
```

## 2. `DroneSceneBuilder.cs` —正确的 место для панели

Замени метод `BuildCameraAndHud` целиком (панель добавляется в конце и работает даже если HUD уже есть в сцене):

```csharp
    public static void BuildCameraAndHud(DroneBuildContext ctx, Transform drone)
    {
        var c = Camera.main;
        GameObject cg;
        if (c != null) cg = c.gameObject;
        else
        {
            cg = new GameObject("SimCamera");
            ctx.register?.Invoke(cg);
            cg.tag = "MainCamera";
            cg.AddComponent<Camera>();
            cg.transform.position = drone.position + new Vector3(0, 2f, -5f);
        }
        if (UnityEngine.Object.FindObjectOfType<AudioListener>() == null) cg.AddComponent<AudioListener>();
        var cc = cg.GetComponent<ChaseCamera>();
        if (cc == null) cc = cg.AddComponent<ChaseCamera>();
        cc.target = drone;
        cc.input = drone.GetComponent<DroneInputManager>();

        if (UnityEngine.Object.FindObjectOfType<SimHud>() == null)
        {
            var hg = new GameObject("HUD");
            ctx.register?.Invoke(hg);
            hg.AddComponent<SimHud>();
        }

        // панель телеметрии — на тот же объект, что и HUD (или на отдельный)
        if (UnityEngine.Object.FindObjectOfType<SimTelemetryPanel>() == null)
        {
            var hud = UnityEngine.Object.FindObjectOfType<SimHud>();
            GameObject host = hud != null ? hud.gameObject : new GameObject("Telemetry");
            if (hud == null) ctx.register?.Invoke(host);
            host.AddComponent<SimTelemetryPanel>();
        }
    }
```

## 3. Бонус: меню для уже готовой статичной сцены

Так как сцена у тебя уже собрана и сохранена, проще добавить панель одним пунктом меню. Вставь этот метод в класс `DroneSimEditorTools` (рядом с `AddBreakup`):

```csharp
    [MenuItem("Tools/Drone Sim/Add Telemetry Panel")]
    public static void AddTelemetryPanel()
    {
        if (Object.FindObjectOfType<SimTelemetryPanel>() != null)
        { EditorUtility.DisplayDialog("Уже есть", "Панель телеметрии уже в сцене.", "OK"); return; }

        var hud = Object.FindObjectOfType<SimHud>();
        GameObject host = hud != null ? hud.gameObject : new GameObject("Telemetry");
        if (hud == null) Undo.RegisterCreatedObjectUndo(host, "Add Telemetry Panel");
        Undo.AddComponent<SimTelemetryPanel>(host);

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = host;
        Debug.Log("[DroneSim] Панель телеметрии добавлена. F7 — открыть/закрыть.");
    }
```

После компиляции: **Tools ▸ Drone Sim ▸ Add Telemetry Panel**, сохрани сцену (Ctrl+S) — и F7 откроет графики. Ошибка CS0103 уйдёт сразу после правки пункта 1.

### USER
Все отлично работает телеметрия отображается но вдруг стала ужасная производительность можно ли это исправить?  либо если нельзя может просто отключать исполнение фунций когда окно телеметрии скрыто?

### ASSISTANT
Да, это чинится — и заодно сделаем полную «заморозку» при скрытом окне.

## Что именно ело производительность

В прошлой версии каждый тик записи (20 Гц) для **каждой открытой группы** происходило:
- обход **всех** сэмплов × всех серий через делегаты `Func<Sample,float>` — причём `se.get(s)` вызывался **дважды** на сэмпл (для точки и для `current`);
- при 30 с × 20 Гц × 4 группы это ~40 000 вызовов делегатов за тик, плюс обход объектов `Sample` (кэш-недружелюбно);
- полная перекраска текстуры + `Apply()` (загрузка на GPU) 20 раз в секунду;
- форматирование строк в `OnGUI` каждый кадр → мусор для GC.

Плюс запись и опрос событий продолжались даже при скрытом окне.

## Что изменилось

- **Плоский кольцевой буфер** `float[20][cap]` вместо `Queue<Sample>` и делегатов — нулевые аллокации, последовательное чтение памяти (быстрее в разы).
- **Отдельная частота отрисовки** `Plot Hz` (по умолчанию 10 Гц) — запись может идти 20–50 Гц, а графики рисуются реже.
- **`Pause When Hidden`** — при закрытом окне (F7) не выполняется вообще ничего: ни запись, ни опрос событий, ни графики, ни GUI.
- **Кэш строк** (значения, диапазоны, карточки моторов) — обновляется по таймеру, а не в каждом `OnGUI`.
- Меньшие текстуры по умолчанию (480×60) и **счётчик затрат** `plot … ms` в панели, чтобы видеть реальную цену.

---

## Полный оптимизированный `SimTelemetryPanel.cs`

```csharp
using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(110)]
public class SimTelemetryPanel : MonoBehaviour
{
    [Header("Окно (F7 — показать/скрыть)")]
    public bool show = true;
    public Rect windowRect = new Rect(330, 110, 1010, 660);

    [Header("Запись")]
    [Tooltip("Частота выборки телеметрии, Гц")] [Range(5, 60)] public int sampleRateHz = 20;
    [Tooltip("Длина окна графиков, с")] [Range(10, 120)] public int windowSeconds = 30;
    public bool paused;
    [Tooltip("Окно скрыто (F7) — полностью остановить запись, опрос событий и отрисовку")]
    public bool pauseWhenHidden = true;

    [Header("Графики")]
    [Tooltip("Частота перерисовки графиков, Гц (запись может быть чаще)")] [Range(2, 30)] public int plotHz = 10;
    [Range(256, 1024)] public int graphWidth = 480;
    [Range(40, 160)] public int graphHeight = 60;
    public bool autoScale = true;

    // индексы каналов в плоском буфере
    const int ALT = 0, VS = 1, P = 2, Q = 3, R = 4, WX = 5, WZ = 6, GE = 7,
              ROLL = 8, PITCH = 9, VBAT = 10, AMPS = 11, RPM0 = 12, DUTY0 = 16, CHANS = 20;

    class Chan { public int id; public string name; public Color color; public float current; public string text = "0.00"; }
    class Group
    {
        public string title; public bool open; public Chan[] chans;
        public Texture2D tex; public Color32[] px;
        public float min, max; public bool scaled; public string rangeText = "";
    }
    class RotorCard
    {
        public string title = "", status = "", rpm = "", l1 = "", l2 = "", l3 = "", l4 = "";
        public Color statusColor; public float duty;
    }
    struct Evt { public string time; public string text; public Color c; }

    float[][] buf; int cap, head, count;
    readonly List<Evt> events = new List<Evt>();
    readonly RotorCard[] cards = { new RotorCard(), new RotorCard(), new RotorCard(), new RotorCard() };
    List<Group> groups;
    int tab; float sampleTimer, plotTimer; Vector2 scrollTraces, scrollLog;
    GUIStyle winStyle, headStyle, bigStyle, smallStyle;
    string infoText = "", summary1 = "", summary2 = "";
    float plotMs;
    static readonly System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

    FlightController fc; QuadPhysics ph; BatteryModel bat; WindSystem wind; DroneBreakup breakup;
    System.Action crashHandler; bool subscribed;

    static readonly Color Cacc = new Color(0.25f, 0.9f, 1f);
    static readonly Color Cok = new Color(0.3f, 1f, 0.45f);
    static readonly Color Cwarn = new Color(1f, 0.75f, 0.2f);
    static readonly Color Cerr = new Color(1f, 0.3f, 0.3f);
    static readonly Color Cdim = new Color(0.72f, 0.78f, 0.84f);
    static readonly Color Cp = new Color(1f, 0.35f, 0.7f);
    static readonly Color Cb = new Color(0.4f, 0.6f, 1f);
    static readonly Color Cv = new Color(0.7f, 0.5f, 1f);
    static readonly Color Cy = new Color(1f, 0.85f, 0.2f);
    static readonly Color Co = new Color(1f, 0.6f, 0.2f);

    bool lastArmed, lastCrashed, lastFs, lastLow, lastCrit, lastVrs, lastBroken;
    FlightMode lastMode;

    void Start()
    {
        fc = FindObjectOfType<FlightController>();
        ph = FindObjectOfType<QuadPhysics>();
        bat = FindObjectOfType<BatteryModel>();
        wind = FindObjectOfType<WindSystem>();
        breakup = FindObjectOfType<DroneBreakup>();
        enabled = fc != null && ph != null;
        if (!enabled) return;

        lastMode = fc.Mode;
        crashHandler = () => Log("CRASH — удар", Cerr);
        ph.OnCrash += crashHandler;
        subscribed = true;

        groups = new List<Group>
        {
            new Group { title = "altitude (m) · climb rate (m/s)", open = true, chans = new[]
            {
                new Chan { id = ALT, name = "alt", color = Cacc },
                new Chan { id = VS,  name = "vs",  color = Cb }
            }},
            new Group { title = "body rates p/q/r (°/s)", open = true, chans = new[]
            {
                new Chan { id = P, name = "p", color = Cp },
                new Chan { id = Q, name = "q", color = Cok },
                new Chan { id = R, name = "r", color = Cb }
            }},
            new Group { title = "wind X/Z (m/s) · ground effect", open = true, chans = new[]
            {
                new Chan { id = WX, name = "wx", color = Cv },
                new Chan { id = WZ, name = "wz", color = new Color(0.5f, 0.4f, 0.9f) },
                new Chan { id = GE, name = "GE", color = Cy }
            }},
            new Group { title = "rotor rpm (об/мин)", open = true, chans = new[]
            {
                new Chan { id = RPM0 + 0, name = "M1", color = Cacc },
                new Chan { id = RPM0 + 1, name = "M2", color = Cok },
                new Chan { id = RPM0 + 2, name = "M3", color = Cp },
                new Chan { id = RPM0 + 3, name = "M4", color = Co }
            }},
            new Group { title = "motor duty (%)", open = false, chans = new[]
            {
                new Chan { id = DUTY0 + 0, name = "M1", color = Cacc },
                new Chan { id = DUTY0 + 1, name = "M2", color = Cok },
                new Chan { id = DUTY0 + 2, name = "M3", color = Cp },
                new Chan { id = DUTY0 + 3, name = "M4", color = Co }
            }},
            new Group { title = "battery V / A", open = false, chans = new[]
            {
                new Chan { id = VBAT, name = "V", color = Cok },
                new Chan { id = AMPS, name = "A", color = Cwarn }
            }},
            new Group { title = "attitude roll/pitch (°)", open = false, chans = new[]
            {
                new Chan { id = ROLL,  name = "roll",  color = Cacc },
                new Chan { id = PITCH, name = "pitch", color = Cp }
            }},
        };
        EnsureBuffer();
        Log("панель телеметрии запущена", Cdim);
    }

    void Update()
    {
        bool key = false;
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb != null) key = kb.f7Key.wasPressedThisFrame;
#elif ENABLE_LEGACY_INPUT_MANAGER
        key = Input.GetKeyDown(KeyCode.F7);
#endif
        if (key) show = !show;

        if (fc == null || ph == null || groups == null) return;
        if (!show && pauseWhenHidden) return;          // полная заморозка в скрытом состоянии

        EnsureBuffer();
        PollEvents();

        sampleTimer -= Time.unscaledDeltaTime;
        if (sampleTimer <= 0f)
        {
            sampleTimer = 1f / Mathf.Max(1, sampleRateHz);
            if (!paused) Push();
        }

        plotTimer -= Time.unscaledDeltaTime;
        if (plotTimer <= 0f)
        {
            plotTimer = 1f / Mathf.Max(1, plotHz);
            if (show && tab == 0)
            {
                sw.Restart();
                foreach (var g in groups) if (g.open) Plot(g);
                sw.Stop();
                plotMs = Mathf.Lerp(plotMs, (float)sw.Elapsed.TotalMilliseconds, 0.3f);
            }
            if (show && tab == 1) UpdateCards();
            infoText = $"образцов {count}/{cap} · запись {sampleRateHz} Гц · отрисовка {plotHz} Гц · plot {plotMs:0.00} ms";
        }
    }

    void EnsureBuffer()
    {
        int want = Mathf.Clamp(sampleRateHz * windowSeconds, 16, 4000);
        if (buf != null && cap == want) return;
        cap = want;
        buf = new float[CHANS][];
        for (int i = 0; i < CHANS; i++) buf[i] = new float[cap];
        head = 0; count = 0;
        if (groups != null) foreach (var g in groups) g.scaled = false;
    }

    void Push()
    {
        buf[ALT][head] = fc.AltAgl;
        buf[VS][head] = fc.Vs;
        buf[ROLL][head] = fc.AttRoll;
        buf[PITCH][head] = fc.AttPitchUp;
        buf[VBAT][head] = bat != null ? bat.Voltage : 0f;
        buf[AMPS][head] = bat != null ? bat.Current : 0f;
        buf[GE][head] = ph.GroundEffectFactor;
        buf[WX][head] = wind != null ? wind.Current.x : 0f;
        buf[WZ][head] = wind != null ? wind.Current.z : 0f;

        Vector3 w = ph.Rb != null
            ? ph.transform.InverseTransformDirection(ph.Rb.angularVelocity) * Mathf.Rad2Deg
            : Vector3.zero;
        buf[P][head] = -w.z;
        buf[Q][head] = w.x;
        buf[R][head] = w.y;

        for (int i = 0; i < 4; i++)
        {
            buf[RPM0 + i][head] = ph.motors[i].rpm;
            buf[DUTY0 + i][head] = ph.motors[i].cmd * 100f;
        }
        head++; if (head >= cap) head = 0;
        if (count < cap) count++;
    }

    // ---------- графики ----------
    void EnsureTex(Group g)
    {
        if (g.tex != null && g.tex.width == graphWidth && g.tex.height == graphHeight) return;
        if (g.tex != null) Destroy(g.tex);
        g.tex = new Texture2D(graphWidth, graphHeight, TextureFormat.RGBA32, false)
        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
        g.px = new Color32[graphWidth * graphHeight];
    }

    void Plot(Group g)
    {
        EnsureTex(g);
        int W = g.tex.width, H = g.tex.height;
        var px = g.px;
        Color32 bg = new Color32(6, 10, 16, 255);
        for (int i = 0; i < px.Length; i++) px[i] = bg;
        Color32 grid = new Color32(22, 30, 40, 255);
        for (int gy = 1; gy < 4; gy++) { int y = H * gy / 4; for (int x = 0; x < W; x++) px[y * W + x] = grid; }

        if (count < 2) { g.tex.SetPixels32(px); g.tex.Apply(false); return; }

        int start = head - count; if (start < 0) start += cap;

        float tmin = float.MaxValue, tmax = float.MinValue;
        for (int c = 0; c < g.chans.Length; c++)
        {
            var arr = buf[g.chans[c].id];
            int idx = start;
            for (int i = 0; i < count; i++)
            {
                float v = arr[idx];
                if (v < tmin) tmin = v;
                if (v > tmax) tmax = v;
                idx++; if (idx >= cap) idx = 0;
            }
        }
        if (tmin > tmax) { tmin = 0; tmax = 1; }
        float pad = (tmax - tmin) * 0.15f + 0.001f;
        tmin -= pad; tmax += pad;
        if (autoScale)
        {
            if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
            else { g.min = Mathf.Lerp(g.min, tmin, 0.25f); g.max = Mathf.Lerp(g.max, tmax, 0.25f); }
        }
        else if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
        g.rangeText = "[" + g.min.ToString("0.0") + " … " + g.max.ToString("0.0") + "]";

        float range = Mathf.Max(1e-4f, g.max - g.min);
        int denom = Mathf.Max(1, cap - 1);
        for (int c = 0; c < g.chans.Length; c++)
        {
            var ch = g.chans[c];
            var arr = buf[ch.id];
            Color32 col = ch.color;
            int idx = start, px0 = 0, py0 = 0;
            float last = 0f;
            for (int i = 0; i < count; i++)
            {
                float v = arr[idx];
                int x = i * (W - 1) / denom;
                int y = Mathf.Clamp((int)((v - g.min) / range * (H - 1) + 0.5f), 0, H - 1);
                if (i > 0) Line(px, W, H, px0, py0, x, y, col);
                px0 = x; py0 = y; last = v;
                idx++; if (idx >= cap) idx = 0;
            }
            ch.current = last;
            ch.text = last.ToString("0.00");
        }
        g.tex.SetPixels32(px);
        g.tex.Apply(false);
    }

    static void Line(Color32[] px, int W, int H, int x0, int y0, int x1, int y1, Color32 c)
    {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (x0 >= 0 && x0 < W && y0 >= 0 && y0 < H) px[y0 * W + x0] = c;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ---------- карточки моторов (кэш строк) ----------
    void UpdateCards()
    {
        for (int i = 0; i < 4; i++)
        {
            var m = ph.motors[i];
            var cd = cards[i];
            cd.title = "M" + (i + 1) + " · " + (m.spinDir > 0 ? "CW" : "CCW");
            if (m.cmd > 0.97f) { cd.status = "SATURATED"; cd.statusColor = Cwarn; }
            else if (m.vrs) { cd.status = "VRS"; cd.statusColor = Cwarn; }
            else if (m.rpm < 100f) { cd.status = "OFF"; cd.statusColor = Cdim; }
            else { cd.status = "RUN"; cd.statusColor = Cok; }
            cd.rpm = m.rpm.ToString("N0");
            cd.duty = m.cmd;

            float eff = ph.gramsPerWatt * (1.15f - 0.5f * m.cmd);
            float power = (m.thrustN / 0.00981f) / Mathf.Max(1f, eff);
            float amps = bat != null ? power / Mathf.Max(1f, bat.Voltage) : 0f;
            cd.l1 = "T " + m.thrustN.ToString("0.00") + " N     Q " + m.torqueNm.ToString("0.0000");
            cd.l2 = "I " + amps.ToString("0.0") + " A       DUTY " + (m.cmd * 100f).ToString("0") + "%";
            cd.l3 = "IGE " + Ige(m).ToString("0.000") + "     VRS " + (m.vrs ? "1.00" : "0.00");
            cd.l4 = "h " + m.groundH.ToString("0.00") + " m    ω " + (m.rpm * 0.10472f).ToString("0") + " rad/s";
        }
        summary1 = "Суммарная тяга: " + ph.TotalThrustN.ToString("0.0") + " Н  (" +
                   (ph.TotalThrustN / 0.00981f).ToString("0") + " г)   Мощность: " + ph.PowerW.ToString("0") + " Вт";
        summary2 = bat != null
            ? "Батарея: " + bat.Voltage.ToString("0.00") + " В  " + bat.Current.ToString("0.0") + " А  " +
              (bat.Soc * 100f).ToString("0") + "%   GE x" + ph.GroundEffectFactor.ToString("0.000") +
              "   AS " + ph.Airspeed.ToString("0.0") + " м/с"
            : "GE x" + ph.GroundEffectFactor.ToString("0.000");
    }

    float Ige(MotorUnit m)
    {
        if (!ph.groundEffect) return 1f;
        float R = ph.propRadius;
        if (m.groundH >= 4f * R) return 1f;
        float h = Mathf.Max(m.groundH, R * 0.35f);
        float r = R / (4f * h);
        return Mathf.Clamp(1f / (1f - r * r), 1f, ph.groundEffectMax);
    }

    // ---------- журнал ----------
    void Log(string text, Color c)
    {
        events.Add(new Evt { time = Time.time.ToString("0.0") + " s", text = text, c = c });
        while (events.Count > 300) events.RemoveAt(0);
    }

    void PollEvents()
    {
        if (fc.Armed != lastArmed) { lastArmed = fc.Armed; Log(fc.Armed ? "ARM" : "DISARM", fc.Armed ? Cok : Cdim); }
        if (fc.Mode != lastMode) { lastMode = fc.Mode; Log("MODE → " + fc.Mode, Cacc); }
        if (ph.Crashed != lastCrashed) { lastCrashed = ph.Crashed; Log(ph.Crashed ? "CRASHED" : "REPAIRED / RESET", ph.Crashed ? Cerr : Cok); }
        if (fc.Failsafe != lastFs) { lastFs = fc.Failsafe; Log(fc.Failsafe ? "FAILSAFE ON" : "FAILSAFE OFF", fc.Failsafe ? Cerr : Cok); }
        if (bat != null)
        {
            if (bat.IsLow != lastLow) { lastLow = bat.IsLow; if (bat.IsLow) Log("LOW BATTERY", Cwarn); }
            if (bat.IsCritical != lastCrit) { lastCrit = bat.IsCritical; if (bat.IsCritical) Log("BATTERY CRITICAL", Cerr); }
        }
        if (ph.VrsActive != lastVrs) { lastVrs = ph.VrsActive; if (ph.VrsActive) Log("VORTEX RING STATE", Cwarn); }
        if (breakup != null && breakup.IsBroken != lastBroken)
        { lastBroken = breakup.IsBroken; Log(lastBroken ? "BREAKUP — дрон разобран" : "REASSEMBLED", lastBroken ? Cerr : Cok); }
    }

    // ---------- GUI ----------
    void OnGUI()
    {
        if (!show || fc == null || groups == null) return;
        var saved = GUI.matrix;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        winStyle = winStyle ?? new GUIStyle(GUI.skin.window);
        try { windowRect = GUI.Window(77, windowRect, DrawWindow, "ТЕЛЕМЕТРИЯ   [F7]", winStyle); }
        catch (System.Exception e) { show = false; UnityEngine.Debug.LogError("[Telemetry] " + e); }
        GUI.matrix = saved;
    }

    void DrawWindow(int id)
    {
        headStyle = headStyle ?? new GUIStyle(GUI.skin.button) { alignment = TextAnchor.MiddleLeft, fontStyle = FontStyle.Bold };
        bigStyle = bigStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 30, fontStyle = FontStyle.Bold };
        smallStyle = smallStyle ?? new GUIStyle(GUI.skin.label) { fontSize = 11 };

        GUILayout.BeginHorizontal();
        string[] tabs = { "LIVE TRACES", "ROTOR BENCH", "EVENT LOG" };
        for (int i = 0; i < tabs.Length; i++)
        {
            GUI.color = tab == i ? Cacc : Color.white;
            if (GUILayout.Button(tabs[i], GUILayout.Height(26))) tab = i;
        }
        GUI.color = Color.white;
        GUILayout.FlexibleSpace();
        GUI.color = paused ? Cwarn : Cok;
        if (GUILayout.Button(paused ? "● REC" : "⏸ PAUSE", GUILayout.Width(90), GUILayout.Height(26))) paused = !paused;
        GUI.color = Color.white;
        if (GUILayout.Button("CLEAR", GUILayout.Width(70), GUILayout.Height(26)))
        { head = 0; count = 0; events.Clear(); foreach (var g in groups) g.scaled = false; }
        GUILayout.EndHorizontal();

        GUILayout.BeginHorizontal();
        GUILayout.Label("окно:", GUILayout.Width(45));
        int[] secs = { 10, 30, 60 };
        foreach (var s in secs)
        {
            GUI.color = windowSeconds == s ? Cacc : Color.white;
            if (GUILayout.Button(s + " с", GUILayout.Width(56))) windowSeconds = s;
        }
        GUI.color = Color.white;
        autoScale = GUILayout.Toggle(autoScale, "AUTOSCALE", GUILayout.Width(100));
        GUILayout.Label(infoText, smallStyle);
        GUILayout.EndHorizontal();
        GUILayout.Space(4);

        if (tab == 0) DrawTraces();
        else if (tab == 1) DrawRotors();
        else DrawLog();

        GUI.DragWindow();
    }

    void DrawTraces()
    {
        scrollTraces = GUILayout.BeginScrollView(scrollTraces);
        foreach (var g in groups)
        {
            if (GUILayout.Button((g.open ? "▼  " : "►  ") + g.title, headStyle, GUILayout.Height(24)))
            { g.open = !g.open; g.scaled = false; }
            if (!g.open) continue;

            GUILayout.BeginHorizontal();
            foreach (var ch in g.chans)
            {
                GUI.color = ch.color;
                GUILayout.Label(ch.text + " " + ch.name, GUILayout.Width(130));
            }
            GUI.color = Cdim;
            GUILayout.FlexibleSpace();
            GUILayout.Label(g.rangeText, smallStyle, GUILayout.Width(150));
            GUI.color = Color.white;
            GUILayout.EndHorizontal();

            Rect r = GUILayoutUtility.GetRect(graphWidth, graphHeight, GUILayout.ExpandWidth(true));
            if (g.tex != null && Event.current.type == EventType.Repaint) GUI.DrawTexture(r, g.tex);
            GUILayout.Space(6);
        }
        GUILayout.EndScrollView();
    }

    void DrawRotors()
    {
        GUILayout.Space(6);
        GUILayout.BeginHorizontal();
        for (int i = 0; i < 4; i++)
        {
            var cd = cards[i];
            GUILayout.BeginVertical(GUI.skin.box, GUILayout.Width(232));
            GUILayout.BeginHorizontal();
            GUILayout.Label(cd.title, GUILayout.Width(110));
            GUI.color = cd.statusColor; GUILayout.Label(cd.status, smallStyle); GUI.color = Color.white;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(cd.rpm, bigStyle, GUILayout.Width(140));
            GUILayout.Label("RPM", smallStyle, GUILayout.Width(40));
            GUILayout.EndHorizontal();

            Rect bar = GUILayoutUtility.GetRect(210, 8, GUILayout.ExpandWidth(false));
            GUI.color = new Color(1, 1, 1, 0.1f); GUI.DrawTexture(bar, Texture2D.whiteTexture);
            GUI.color = cd.duty > 0.97f ? Cwarn : Cok;
            GUI.DrawTexture(new Rect(bar.x, bar.y, bar.width * Mathf.Clamp01(cd.duty), bar.height), Texture2D.whiteTexture);
            GUI.color = Color.white;

            GUILayout.Label(cd.l1, smallStyle);
            GUILayout.Label(cd.l2, smallStyle);
            GUILayout.Label(cd.l3, smallStyle);
            GUILayout.Label(cd.l4, smallStyle);
            GUILayout.EndVertical();
            if (i < 3) GUILayout.Space(4);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(10);
        GUILayout.Label(summary1);
        GUILayout.Label(summary2, smallStyle);
    }

    void DrawLog()
    {
        scrollLog = GUILayout.BeginScrollView(scrollLog);
        for (int i = events.Count - 1; i >= 0; i--)
        {
            var e = events[i];
            GUILayout.BeginHorizontal();
            GUILayout.Label(e.time, smallStyle, GUILayout.Width(70));
            GUI.color = e.c; GUILayout.Label(e.text); GUI.color = Color.white;
            GUILayout.EndHorizontal();
        }
        GUILayout.EndScrollView();
    }

    void OnDestroy()
    {
        if (subscribed && ph != null && crashHandler != null) ph.OnCrash -= crashHandler;
        subscribed = false;
        if (groups != null) foreach (var g in groups) if (g.tex != null) Destroy(g.tex);
    }
}
```

---

## Настройки для максимальной скорости

В инспекторе панели:
- **Plot Hz = 5–10** — графики всё равно выглядят плавно, а нагрузка падает линейно.
- **Graph Width 480 / Height 60** — цена перерисовки пропорциональна площади текстуры.
- **Window Seconds 10–30** — буфер = частота × секунды (жёстко ограничен 4000 сэмплов, чтобы нельзя было случайно убить FPS).
- **Pause When Hidden = true** — закрыл F7, и панель не делает вообще ничего.
- Сворачивай ненужные группы (`►`) — рисуются только раскрытые, и только на вкладке LIVE TRACES.

Смотри на счётчик **`plot … ms`** в шапке: нормально — это 0.2–1.5 мс. Если больше 3–4 мс, снижай Plot Hz и размер графиков.

## Если тормозит и без панели

Тогда дело в самом симуляторе, а не в телеметрии:
- **`SimBootstrap → Physics Hz`**: 200 Гц × 4 рейкаста на шаг = 800 рейкастов/с плюс PID-подшаги. Поставь **150** — на глаз разница минимальна, нагрузка заметно ниже.
- **`SimHud`** (основной HUD) рисует авиагоризонт с десятками линий каждый кадр — при работе с графиками его можно выключить **F6**.
- В редакторе IMGUI дороже, чем в билде: проверь FPS в собранной версии, прежде чем оптимизировать дальше.
- `Motor Sound` и `Prop Visual` дешёвые, их трогать не нужно.

Если после этого `plot ms` маленький, а FPS всё равно низкий — скажи, какой у тебя FPS с открытой панелью и с закрытой, и я подскажу, что профилировать дальше.

### USER
 все отлично работает большое спасибо! но теперь при билде из юнити под Windows в .exe сборку возникают следующие ошибки я думаю надо как то добавить в исключения скрипты редактора, помоги мне это сделать. Вот сами ошибки при сборке : Assets\DroneSimEditorTools.cs(2,19): error CS0234: The type or namespace name 'SceneManagement' does not exist in the namespace 'UnityEditor' (are you missing an assembly reference?)

Assets\DroneSimEditorTools.cs(169,38): error CS0246: The type or namespace name 'Editor' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(168,2): error CS0246: The type or namespace name 'CustomEditorAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(168,2): error CS0246: The type or namespace name 'CustomEditor' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(12,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(12,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(32,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(32,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(44,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(44,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(59,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(59,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(76,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(76,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(84,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(84,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(101,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(101,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(104,6): error CS0246: The type or namespace name 'MenuItemAttribute' could not be found (are you missing a using directive or an assembly reference?)

Assets\DroneSimEditorTools.cs(104,6): error CS0246: The type or namespace name 'MenuItem' could not be found (are you missing a using directive or an assembly reference?)

Error building Player because scripts had compiler errors

Build completed with a result of 'Failed' in 8 seconds (7982 ms) [19.11.2026 18:41:30 -> 19.11.2026 18:41:38, 07s]
21 errors
UnityEditor.EditorApplication:Internal_CallDelayFunctions ()



### ASSISTANT
Причина простая: файл лежит в корне — `Assets\DroneSimEditorTools.cs`, а не в папке `Editor`. В редакторе пространство имён `UnityEditor` доступно, а при сборке плеера его нет, поэтому все `MenuItem`, `Editor`, `CustomEditor`, `EditorSceneManager` исчезают.

## Способ 1 — правильный: перенести в папку `Editor`

1. В окне **Project** создай папку `Assets/Editor` (можно `Assets/Scripts/Editor` — важна именно папка с именем **Editor** на любом уровне).
2. Перетащи туда `DroneSimEditorTools.cs` **средствами Unity** (тогда `.meta` переедет вместе с файлом).
   Если переносишь через Проводник — переноси и `DroneSimEditorTools.cs.meta` тоже.
3. Unity автоматически исключает такие папки из сборки. Меню **Tools ▸ Drone Sim** и кнопки в инспекторе `SimBootstrap` продолжат работать как раньше.

Больше ничего менять не нужно — остальные скрипты рантайм-совместимы (я специально писал `DroneSceneBuilder` без ссылок на `UnityEditor`, чтобы он работал и в билде).

## Способ 2 — страховка: обернуть файл в `#if UNITY_EDITOR`

Полезно сделать **в дополнение** к способу 1 (или вместо, если не хочешь переносить). Тогда файл безопасно компилируется где угодно.

В самое начало файла:

```csharp
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;
using Debug = UnityEngine.Debug;
```

И в самый конец файла (после закрывающей скобки класса `SimBootstrapInspector`):

```csharp
#endif
```

Важно: **все `using` должны быть внутри `#if`**, иначе `using UnityEditor;` сам по себе даст ошибку при сборке. Оборачивается весь файл целиком, включая класс `SimBootstrapInspector : Editor`.

## Проверка остальных скриптов

Быстро: в IDE сделай поиск по проекту `using UnityEditor` — такие файлы должны быть **только** в папке `Editor` (или под `#if UNITY_EDITOR`). В нашем проекте это единственный файл:

| Файл | Где должен лежать |
|---|---|
| `DroneSimEditorTools.cs` | **Assets/Editor/** (или `#if UNITY_EDITOR`) |
| `SimCore.cs`, `DroneInputManager.cs`, `UdpLink.cs`, `WindSystem.cs`, `BatteryModel.cs`, `QuadPhysics.cs`, `FlightController.cs`, `DronePresentation.cs`, `SimHud.cs`, `SimBootstrap.cs`, `DroneSceneBuilder.cs`, `DroneBreakup.cs`, `WindowsJoystick.cs`, `SimTelemetryPanel.cs` | Assets/Scripts/ (рантайм) |

## Вариант для больших проектов — asmdef

Если позже разнесёшь код по сборкам: создай в папке `Editor` файл **Assembly Definition** (`Create ▸ Assembly Definition`) и в его инспекторе поставь **Platforms = Editor** (сними все остальные). Тогда сборка физически не попадёт в плеер, даже если кто-то случайно положит рядом рантайм-скрипт.

---

## Пара нюансов после успешной сборки

- **Материалы**: у тебя сцена статичная, материалы — ассеты в `Assets/DroneSim/Materials`, они попадут в билд по ссылкам ✓. Но если включишь `autoBuildAtRuntime` в билде, `Shader.Find("Sprites/Default")` может вернуть null — добавь шейдер в `Project Settings ▸ Graphics ▸ Always Included Shaders`.
- **Input System**: пакет должен быть в проекте (он у тебя есть),(define `ENABLE_INPUT_SYSTEM` подхватится автоматически). Джойстик через `winmm.dll` в Windows-билде работает так же, как в редакторе.
- **Хоткеи**: F1–F7 и IMGUI работают в билде; в полноэкранном режиме F-клавиши не перехватываются системой ✓.
- **FPS**: в билде IMGUI заметно дешевле, чем в редакторе — телеметрия будет идти легче.

После переноса файла в `Assets/Editor/` сборка должна пройти. Если появятся новые ошибки уже от других файлов — присылай список, разберём.

### USER
Прямо при запуске билда проекта в .exe сразу происходит вылет с ошибкой в первую же секунду, вот логи этой ошибки, помоги разобрать что поправить в проекте чтобы билд прошел успешно!  
логи:
Found 3 interfaces on host : 0) 192.168.0.103 1) 172.24.228.220 2) 10.6.7.1
Player connection [50544]  Target information:

Player connection [50544]  * "[IP] 192.168.0.103 [Port] 55000 [Flags] 2 [Guid] 3752700166 [EditorId] 1826496200 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 0 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [50544]  * "[IP] 172.24.228.220 [Port] 55000 [Flags] 2 [Guid] 3752700166 [EditorId] 1826496200 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 0 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [50544]  * "[IP] 10.6.7.1 [Port] 55000 [Flags] 2 [Guid] 3752700166 [EditorId] 1826496200 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 0 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [50544] Started UDP target info broadcast (1) on [225.0.0.222:54997].

Input System module state changed to: Initialized.
[Physics::Module] Initialized fallback backend.
[Physics::Module] Id: 0xdecafbad
Loading player data from C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/data.unity3d
Initialize engine version: 6000.3.28f1 (8f082d2fe476)
[Subsystems] Discovering subsystems at path C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/UnitySubsystems
[D3D12 Device Filter] Vendor Name: NVIDIA
[D3D12 Device Filter] Device Name: NVIDIA GeForce RTX 3090
[D3D12 Device Filter] Driver Version: 32.0.15.8108
[D3D12 Device Filter] Feature Level: 12.2
[D3D12 Device Filter] Graphics Memory: 24326 MB
[D3D12 Device Filter] Processor Count: 48
[D3D12 Device Filter] Device Type: Discrete
GfxDevice: creating device client; kGfxThreadingModeSplitJobs
d3d12: failed to query info queue interface (0x80004002).
Direct3D:
    Version:         Direct3D 12 [level 12.2]
    Renderer:        NVIDIA GeForce RTX 3090 (ID=0x2204)
    Vendor:          NVIDIA
    VRAM:            24326 MB
    App VRAM Budget: 23558 MB
    Driver:          32.0.15.8108
[Physics::Module] Selected backend.
[Physics::Module] Name: PhysX
[Physics::Module] Id: 0xf2b8ea05
[Physics::Module] SDK Version: 4.1.2
[Physics::Module] Integration Version: 1.0.0
[Physics::Module] Threading Mode: Multi-Threaded
<RI> Initializing input.

Using Windows.Gaming.Input
<RI> Input initialized.

The file 'C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/level0' is corrupted! Remove it and launch unity again!
[Position out of bounds!]
Crash!!!
SymInit: Symbol-SearchPath: '.;C:\WorkProject\WarDroneB\TestBuild01;C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/Plugins\x86_64;C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/Plugins;C:\WorkProject\WarDroneB\TestBuild01;C:\Windows;C:\Windows\system32;', symOptions: 534, UserName: 'KirikDK'
OS-Version: 10.0.0
C:\WorkProject\WarDroneB\TestBuild01\MyDroneA.exe:MyDroneA.exe (00007FF7D9180000), size: 679936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 6000.3.28.2093
C:\Windows\SYSTEM32\ntdll.dll:ntdll.dll (00007FF867030000), size: 2064384 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\KERNEL32.DLL:KERNEL32.DLL (00007FF865080000), size: 774144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1889
C:\Windows\System32\KERNELBASE.dll:KERNELBASE.dll (00007FF864860000), size: 2957312 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\WorkProject\WarDroneB\TestBuild01\UnityPlayer.dll:UnityPlayer.dll (00007FFF91720000), size: 92344320 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 6000.3.28.2093
C:\Windows\System32\USER32.dll:USER32.dll (00007FF8657A0000), size: 1691648 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\win32u.dll:win32u.dll (00007FF864FA0000), size: 139264 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\GDI32.dll:GDI32.dll (00007FF8652C0000), size: 176128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\gdi32full.dll:gdi32full.dll (00007FF864D90000), size: 1110016 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\msvcp_win.dll:msvcp_win.dll (00007FF864730000), size: 643072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.789
C:\Windows\System32\ucrtbase.dll:ucrtbase.dll (00007FF864EA0000), size: 1048576 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.789
C:\Windows\System32\ole32.dll:ole32.dll (00007FF865B80000), size: 1220608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1202
C:\Windows\System32\RPCRT4.dll:RPCRT4.dll (00007FF865410000), size: 1200128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1806
C:\Windows\System32\combase.dll:combase.dll (00007FF865E20000), size: 3489792 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\VERSION.dll:VERSION.dll (00007FF85E580000), size: 40960 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\SHLWAPI.dll:SHLWAPI.dll (00007FF865B20000), size: 348160 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\System32\msvcrt.dll:msvcrt.dll (00007FF865140000), size: 647168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.0.19041.546
C:\Windows\System32\SETUPAPI.dll:SETUPAPI.dll (00007FF866230000), size: 4620288 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\cfgmgr32.dll:cfgmgr32.dll (00007FF864BB0000), size: 319488 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\bcrypt.dll:bcrypt.dll (00007FF864D60000), size: 159744 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1023
C:\Windows\System32\ADVAPI32.dll:ADVAPI32.dll (00007FF866180000), size: 712704 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\sechost.dll:sechost.dll (00007FF865220000), size: 638976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\SHELL32.dll:SHELL32.dll (00007FF866820000), size: 7614464 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\System32\OLEAUT32.dll:OLEAUT32.dll (00007FF866750000), size: 839680 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.985
C:\Windows\System32\IMM32.dll:IMM32.dll (00007FF8651E0000), size: 204800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\OPENGL32.dll:OPENGL32.dll (00007FF8174C0000), size: 1200128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\WINMM.dll:WINMM.dll (00007FF8552A0000), size: 159744 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\CRYPT32.dll:CRYPT32.dll (00007FF864C00000), size: 1400832 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1889
C:\Windows\System32\WS2_32.dll:WS2_32.dll (00007FF865730000), size: 438272 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\IPHLPAPI.DLL:IPHLPAPI.DLL (00007FF863B30000), size: 241664 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\WINHTTP.dll:WINHTTP.dll (00007FF85D2A0000), size: 1097728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\HID.DLL:HID.DLL (00007FF862AB0000), size: 53248 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\d3d11.dll:d3d11.dll (00007FF8627F0000), size: 2502656 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dxgi.dll:dxgi.dll (00007FF862B10000), size: 995328 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\UIAutomationCore.DLL:UIAutomationCore.DLL (00007FF8113E0000), size: 3100672 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.2.19041.1741
C:\Windows\SYSTEM32\GLU32.dll:GLU32.dll (00007FF818AB0000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\Secur32.dll:Secur32.dll (00007FF84A270000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dwmapi.dll:dwmapi.dll (00007FF861040000), size: 192512 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\PROPSYS.dll:PROPSYS.dll (00007FF85E9F0000), size: 1007616 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.0.19041.1741
C:\Windows\SYSTEM32\SSPICLI.DLL:SSPICLI.DLL (00007FF864620000), size: 204800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll:cpcrypt.dll (00007FF864200000), size: 135168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.16728.0
C:\Windows\SYSTEM32\kernel.appcore.dll:kernel.appcore.dll (00007FF862AC0000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\bcryptPrimitives.dll:bcryptPrimitives.dll (00007FF8647D0000), size: 532480 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1415
C:\Windows\system32\uxtheme.dll:uxtheme.dll (00007FF860D70000), size: 647168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\shcore.dll:shcore.dll (00007FF865D70000), size: 708608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\clbcatq.dll:clbcatq.dll (00007FF8666A0000), size: 716800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 2001.12.10941.16384
C:\Windows\system32\wbem\wbemprox.dll:wbemprox.dll (00007FF84B4D0000), size: 69632 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1320
C:\Windows\SYSTEM32\wbemcomn.dll:wbemcomn.dll (00007FF84B0D0000), size: 589824 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\system32\wbem\wbemsvc.dll:wbemsvc.dll (00007FF84A350000), size: 81920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1320
C:\Windows\system32\wbem\fastprox.dll:fastprox.dll (00007FF84A440000), size: 1093632 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\amsi.dll:amsi.dll (00007FF849910000), size: 126976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\WorkProject\WarDroneB\TestBuild01\WinPixEventRuntime.dll:WinPixEventRuntime.dll (00007FF85E710000), size: 57344 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 1.0.2403.8001
C:\Windows\SYSTEM32\MSVCP140.dll:MSVCP140.dll (00007FF847190000), size: 643072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140_1.dll:VCRUNTIME140_1.dll (00007FF847260000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140.dll:VCRUNTIME140.dll (00007FF847230000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\windows.storage.dll:windows.storage.dll (00007FF861510000), size: 7933952 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\Wldp.dll:Wldp.dll (00007FF864160000), size: 196608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\profapi.dll:profapi.dll (00007FF864670000), size: 126976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.844
C:\WorkProject\WarDroneB\TestBuild01\GameAssembly.dll:GameAssembly.dll (00007FFF98B50000), size: 30552064 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\SYSTEM32\wintypes.dll:wintypes.dll (00007FF85FBA0000), size: 1392640 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dbghelp.dll:dbghelp.dll (00007FF862600000), size: 1982464 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.867
C:\WorkProject\WarDroneB\TestBuild01\baselib.dll:baselib.dll (00007FF8476B0000), size: 245760 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\system32\mswsock.dll:mswsock.dll (00007FF863E90000), size: 434176 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\NSI.dll:NSI.dll (00007FF865960000), size: 32768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.610
C:\Windows\SYSTEM32\dhcpcsvc6.DLL:dhcpcsvc6.DLL (00007FF85E590000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dhcpcsvc.DLL:dhcpcsvc.DLL (00007FF85E4F0000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\DNSAPI.dll:DNSAPI.dll (00007FF863B70000), size: 827392 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\CoreMessaging.dll:CoreMessaging.dll (00007FF860A40000), size: 991232 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\system32\napinsp.dll:napinsp.dll (00007FF83A9E0000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\pnrpnsp.dll:pnrpnsp.dll (00007FF83A9C0000), size: 110592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\wshbth.dll:wshbth.dll (00007FF83A9A0000), size: 86016 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\NLAapi.dll:NLAapi.dll (00007FF85FA30000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\winrnr.dll:winrnr.dll (00007FF83A980000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Program Files\Bonjour\mdnsNSP.dll:mdnsNSP.dll (00000000632E0000), size: 155648 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 3.0.0.10
C:\Windows\System32\MSCTF.dll:MSCTF.dll (00007FF8652F0000), size: 1134592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\d3d12.dll:d3d12.dll (00007FF855260000), size: 131072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1266
C:\WorkProject\WarDroneB\TestBuild01\D3D12\D3D12Core.dll:D3D12Core.dll (00007FFF9B680000), size: 4780032 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 1.618.1.0
C:\Windows\SYSTEM32\dxcore.dll:dxcore.dll (00007FF85B520000), size: 241664 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\msasn1.dll:msasn1.dll (00007FF864320000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\cryptnet.dll:cryptnet.dll (00007FF85E540000), size: 200704 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.906
C:\Windows\SYSTEM32\cryptbase.dll:cryptbase.dll (00007FF8640D0000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\drvstore.dll:drvstore.dll (00007FF85DEF0000), size: 1343488 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1949
C:\Windows\SYSTEM32\devobj.dll:devobj.dll (00007FF8644E0000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\wintrust.dll:wintrust.dll (0000022D93300000), size: 434176 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\imagehlp.dll:imagehlp.dll (00007FF865940000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1415
C:\Windows\SYSTEM32\CRYPTSP.dll:CRYPTSP.dll (00007FF8640B0000), size: 98304 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll:cpadvai.dll (00007FF864090000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.16639.0
C:\Program Files\Crypto Pro\CSP\cpcsp.dll:cpcsp.dll (00007FF8634A0000), size: 32768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.14960.0
C:\Program Files\Crypto Pro\CSP\cpcspi.dll:cpcspi.dll (00007FF863280000), size: 2187264 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.18680.0
C:\Program Files\Crypto Pro\CSP\cpsuprt.dll:cpsuprt.dll (00007FF8631E0000), size: 634880 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.15962.0
C:\Windows\system32\nvspcap64.dll:nvspcap64.dll (00007FF81A2E0000), size: 1339392 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 11.0.5.420
C:\Windows\SYSTEM32\ntmarta.dll:ntmarta.dll (00007FF863860000), size: 208896 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll:nvppex.dll (00007FFFF4690000), size: 1798144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\D3DSCache.dll:D3DSCache.dll (00007FFF3FFF0000), size: 176128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\USERENV.dll:USERENV.dll (00007FF8645F0000), size: 188416 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.572
C:\Windows\SYSTEM32\twinapi.appcore.dll:twinapi.appcore.dll (00007FF85DC80000), size: 2097152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll:nvldumdx.dll (00007FF85AFD0000), size: 802816 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll:nvgpucomp64.dll (00007FF8558C0000), size: 64565248 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll:NvMemMapStoragex.dll (00007FF8552D0000), size: 909312 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\SYSTEM32\POWRPROF.dll:POWRPROF.dll (00007FF863C50000), size: 307200 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\UMPDC.dll:UMPDC.dll (00007FF863B10000), size: 73728 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll:nvwgf2umx.dll (00007FF84DF90000), size: 92487680 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\DirectML.dll:DirectML.dll (00007FFF9CDB0000), size: 1282048 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 1.0.2007.1310
C:\Windows\SYSTEM32\Cabinet.dll:Cabinet.dll (00007FF854570000), size: 167936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.1.1
C:\Windows\SYSTEM32\d3d11on12.dll:d3d11on12.dll (00007FFF40060000), size: 618496 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1081
C:\Windows\SYSTEM32\dxilconv.dll:dxilconv.dll (00007FFF3F140000), size: 1282048 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1
C:\Windows\SYSTEM32\DXGIDebug.dll:DXGIDebug.dll (00007FFF40100000), size: 356352 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1
C:\Windows\System32\MMDevApi.dll:MMDevApi.dll (00007FF85D150000), size: 544768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1503
C:\Windows\SYSTEM32\AUDIOSES.DLL:AUDIOSES.DLL (00007FF8220A0000), size: 1581056 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\resourcepolicyclient.dll:resourcepolicyclient.dll (00007FF860F40000), size: 81920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.Input.dll:Windows.Gaming.Input.dll (00007FF85C990000), size: 864256 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\SYSTEM32\inputhost.dll:inputhost.dll (00007FF85B170000), size: 1384448 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1741
C:\Windows\System32\CoreUIComponents.dll:CoreUIComponents.dll (00007FF860360000), size: 3530752 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.UI.GameBar.dll:Windows.Gaming.UI.GameBar.dll (00007FFF3B6F0000), size: 110592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\System32\WININET.dll:WININET.dll (00007FF83A250000), size: 5083136 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 11.0.19041.2193
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll:cpwinet.dll (00007FF8388E0000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.15621.0
C:\Windows\System32\appresolver.dll:appresolver.dll (00007FF8204B0000), size: 589824 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\SLC.dll:SLC.dll (00007FF862D90000), size: 167936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1682
C:\Windows\System32\Bcp47Langs.dll:Bcp47Langs.dll (00007FF85B660000), size: 372736 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\System32\sppc.dll:sppc.dll (00007FF862D60000), size: 151552 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1682
C:\Windows\SYSTEM32\xinput1_3.dll:xinput1_3.dll (0000000000400000), size: 122880 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 9.18.944.0
C:\Windows\SYSTEM32\dcomp.dll:dcomp.dll (00007FF860170000), size: 1978368 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\textinputframework.dll:textinputframework.dll (00007FF85B2D0000), size: 1019904 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075

========== OUTPUTTING STACK TRACE ==================

0x00007FFF93977DAD (UnityPlayer) UnityMain
0x00007FFF939771C9 (UnityPlayer) UnityMain
SymInit: Symbol-SearchPath: '.;C:\WorkProject\WarDroneB\TestBuild01;C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/Plugins\x86_64;C:/WorkProject/WarDroneB/TestBuild01/MyDroneA_Data/Plugins;C:\WorkProject\WarDroneB\TestBuild01;C:\Windows;C:\Windows\system32;', symOptions: 534, UserName: 'KirikDK'
OS-Version: 10.0.0
C:\WorkProject\WarDroneB\TestBuild01\MyDroneA.exe:MyDroneA.exe (00007FF7D9180000), size: 679936 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild01\MyDroneA.exe', fileVersion: 6000.3.28.2093
C:\Windows\SYSTEM32\ntdll.dll:ntdll.dll (00007FF867030000), size: 2064384 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\ntdll.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\KERNEL32.DLL:KERNEL32.DLL (00007FF865080000), size: 774144 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\KERNEL32.DLL', fileVersion: 10.0.19041.1889
C:\Windows\System32\KERNELBASE.dll:KERNELBASE.dll (00007FF864860000), size: 2957312 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\KERNELBASE.dll', fileVersion: 10.0.19041.2193
C:\WorkProject\WarDroneB\TestBuild01\UnityPlayer.dll:UnityPlayer.dll (00007FFF91720000), size: 92344320 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild01\UnityPlayer.dll', fileVersion: 6000.3.28.2093
C:\Windows\System32\USER32.dll:USER32.dll (00007FF8657A0000), size: 1691648 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\USER32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\win32u.dll:win32u.dll (00007FF864FA0000), size: 139264 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\win32u.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\GDI32.dll:GDI32.dll (00007FF8652C0000), size: 176128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\GDI32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\gdi32full.dll:gdi32full.dll (00007FF864D90000), size: 1110016 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\gdi32full.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\msvcp_win.dll:msvcp_win.dll (00007FF864730000), size: 643072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\msvcp_win.dll', fileVersion: 10.0.19041.789
C:\Windows\System32\ucrtbase.dll:ucrtbase.dll (00007FF864EA0000), size: 1048576 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ucrtbase.dll', fileVersion: 10.0.19041.789
C:\Windows\System32\ole32.dll:ole32.dll (00007FF865B80000), size: 1220608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ole32.dll', fileVersion: 10.0.19041.1202
C:\Windows\System32\RPCRT4.dll:RPCRT4.dll (00007FF865410000), size: 1200128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\RPCRT4.dll', fileVersion: 10.0.19041.1806
C:\Windows\System32\combase.dll:combase.dll (00007FF865E20000), size: 3489792 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\combase.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\VERSION.dll:VERSION.dll (00007FF85E580000), size: 40960 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VERSION.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\SHLWAPI.dll:SHLWAPI.dll (00007FF865B20000), size: 348160 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SHLWAPI.dll', fileVersion: 10.0.19041.2075
C:\Windows\System32\msvcrt.dll:msvcrt.dll (00007FF865140000), size: 647168 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\msvcrt.dll', fileVersion: 7.0.19041.546
C:\Windows\System32\SETUPAPI.dll:SETUPAPI.dll (00007FF866230000), size: 4620288 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SETUPAPI.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\cfgmgr32.dll:cfgmgr32.dll (00007FF864BB0000), size: 319488 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\cfgmgr32.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\bcrypt.dll:bcrypt.dll (00007FF864D60000), size: 159744 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\bcrypt.dll', fileVersion: 10.0.19041.1023
C:\Windows\System32\ADVAPI32.dll:ADVAPI32.dll (00007FF866180000), size: 712704 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ADVAPI32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\sechost.dll:sechost.dll (00007FF865220000), size: 638976 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\sechost.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\SHELL32.dll:SHELL32.dll (00007FF866820000), size: 7614464 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SHELL32.dll', fileVersion: 10.0.19041.2075
C:\Windows\System32\OLEAUT32.dll:OLEAUT32.dll (00007FF866750000), size: 839680 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\OLEAUT32.dll', fileVersion: 10.0.19041.985
C:\Windows\System32\IMM32.dll:IMM32.dll (00007FF8651E0000), size: 204800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\IMM32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\OPENGL32.dll:OPENGL32.dll (00007FF8174C0000), size: 1200128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\OPENGL32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\WINMM.dll:WINMM.dll (00007FF8552A0000), size: 159744 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\WINMM.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\CRYPT32.dll:CRYPT32.dll (00007FF864C00000), size: 1400832 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CRYPT32.dll', fileVersion: 10.0.19041.1889
C:\Windows\System32\WS2_32.dll:WS2_32.dll (00007FF865730000), size: 438272 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\WS2_32.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\IPHLPAPI.DLL:IPHLPAPI.DLL (00007FF863B30000), size: 241664 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\IPHLPAPI.DLL', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\WINHTTP.dll:WINHTTP.dll (00007FF85D2A0000), size: 1097728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\WINHTTP.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\HID.DLL:HID.DLL (00007FF862AB0000), size: 53248 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\HID.DLL', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\d3d11.dll:d3d11.dll (00007FF8627F0000), size: 2502656 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\d3d11.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dxgi.dll:dxgi.dll (00007FF862B10000), size: 995328 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxgi.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\UIAutomationCore.DLL:UIAutomationCore.DLL (00007FF8113E0000), size: 3100672 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\UIAutomationCore.DLL', fileVersion: 7.2.19041.1741
C:\Windows\SYSTEM32\GLU32.dll:GLU32.dll (00007FF818AB0000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\GLU32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\Secur32.dll:Secur32.dll (00007FF84A270000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Secur32.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dwmapi.dll:dwmapi.dll (00007FF861040000), size: 192512 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dwmapi.dll', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\PROPSYS.dll:PROPSYS.dll (00007FF85E9F0000), size: 1007616 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\PROPSYS.dll', fileVersion: 7.0.19041.1741
C:\Windows\SYSTEM32\SSPICLI.DLL:SSPICLI.DLL (00007FF864620000), size: 204800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\SSPICLI.DLL', fileVersion: 10.0.19041.2130
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll:cpcrypt.dll (00007FF864200000), size: 135168 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll', fileVersion: 5.0.16728.0
C:\Windows\SYSTEM32\kernel.appcore.dll:kernel.appcore.dll (00007FF862AC0000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\kernel.appcore.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\bcryptPrimitives.dll:bcryptPrimitives.dll (00007FF8647D0000), size: 532480 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\bcryptPrimitives.dll', fileVersion: 10.0.19041.1415
C:\Windows\system32\uxtheme.dll:uxtheme.dll (00007FF860D70000), size: 647168 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\uxtheme.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\shcore.dll:shcore.dll (00007FF865D70000), size: 708608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\shcore.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\clbcatq.dll:clbcatq.dll (00007FF8666A0000), size: 716800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\clbcatq.dll', fileVersion: 2001.12.10941.16384
C:\Windows\system32\wbem\wbemprox.dll:wbemprox.dll (00007FF84B4D0000), size: 69632 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\wbemprox.dll', fileVersion: 10.0.19041.1320
C:\Windows\SYSTEM32\wbemcomn.dll:wbemcomn.dll (00007FF84B0D0000), size: 589824 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\wbemcomn.dll', fileVersion: 10.0.19041.1566
C:\Windows\system32\wbem\wbemsvc.dll:wbemsvc.dll (00007FF84A350000), size: 81920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\wbemsvc.dll', fileVersion: 10.0.19041.1320
C:\Windows\system32\wbem\fastprox.dll:fastprox.dll (00007FF84A440000), size: 1093632 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\fastprox.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\amsi.dll:amsi.dll (00007FF849910000), size: 126976 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\amsi.dll', fileVersion: 10.0.19041.2075
C:\WorkProject\WarDroneB\TestBuild01\WinPixEventRuntime.dll:WinPixEventRuntime.dll (00007FF85E710000), size: 57344 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild01\WinPixEventRuntime.dll', fileVersion: 1.0.2403.8001
C:\Windows\SYSTEM32\MSVCP140.dll:MSVCP140.dll (00007FF847190000), size: 643072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\MSVCP140.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140_1.dll:VCRUNTIME140_1.dll (00007FF847260000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VCRUNTIME140_1.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140.dll:VCRUNTIME140.dll (00007FF847230000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VCRUNTIME140.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\windows.storage.dll:windows.storage.dll (00007FF861510000), size: 7933952 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\windows.storage.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\Wldp.dll:Wldp.dll (00007FF864160000), size: 196608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Wldp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\profapi.dll:profapi.dll (00007FF864670000), size: 126976 (result: 0), SymType: '-nosymbols-', PDB: 'C:\Windows\SYSTEM32\profapi.dll', fileVersion: 10.0.19041.844
d3d12: upload buffer was too small for the requested resource! Size: 16777216. Requested: 33307392.
Detected one or more triangles where the distance between any 2 vertices is greater than 500 units. The resulting Triangle Mesh can impact simulation and query stability. It is recommended to tessellate meshes that have large triangles. Source mesh name: extrudeFront
Detected one or more triangles where the distance between any 2 vertices is greater than 500 units. The resulting Triangle Mesh can impact simulation and query stability. It is recommended to tessellate meshes that have large triangles. Source mesh name: FonBuildings
C:\WorkProject\WarDroneB\TestBuild01\GameAssembly.dll:GameAssembly.dll (00007FFF98B50000), size: 30552064 (result: 0), SymType: 'PDB', PDB: 'C:\WorkProject\WarDroneB\Library\Bee\artifacts\WinPlayerBuildProgram\cmv89\GameAssembly.pdb'
C:\Windows\SYSTEM32\wintypes.dll:wintypes.dll (00007FF85FBA0000), size: 1392640 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\wintypes.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dbghelp.dll:dbghelp.dll (00007FF862600000), size: 1982464 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dbghelp.dll', fileVersion: 10.0.19041.867
C:\WorkProject\WarDroneB\TestBuild01\baselib.dll:baselib.dll (00007FF8476B0000), size: 245760 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild01\baselib.dll'
C:\Windows\system32\mswsock.dll:mswsock.dll (00007FF863E90000), size: 434176 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\mswsock.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\NSI.dll:NSI.dll (00007FF865960000), size: 32768 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\NSI.dll', fileVersion: 10.0.19041.610
C:\Windows\SYSTEM32\dhcpcsvc6.DLL:dhcpcsvc6.DLL (00007FF85E590000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dhcpcsvc6.DLL', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dhcpcsvc.DLL:dhcpcsvc.DLL (00007FF85E4F0000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dhcpcsvc.DLL', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\DNSAPI.dll:DNSAPI.dll (00007FF863B70000), size: 827392 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\DNSAPI.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\CoreMessaging.dll:CoreMessaging.dll (00007FF860A40000), size: 991232 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CoreMessaging.dll', fileVersion: 10.0.19041.2193
C:\Windows\system32\napinsp.dll:napinsp.dll (00007FF83A9E0000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\napinsp.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\pnrpnsp.dll:pnrpnsp.dll (00007FF83A9C0000), size: 110592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\pnrpnsp.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\wshbth.dll:wshbth.dll (00007FF83A9A0000), size: 86016 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wshbth.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\NLAapi.dll:NLAapi.dll (00007FF85FA30000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\NLAapi.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\winrnr.dll:winrnr.dll (00007FF83A980000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\winrnr.dll', fileVersion: 10.0.19041.546
C:\Program Files\Bonjour\mdnsNSP.dll:mdnsNSP.dll (00000000632E0000), size: 155648 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Bonjour\mdnsNSP.dll', fileVersion: 3.0.0.10
C:\Windows\System32\MSCTF.dll:MSCTF.dll (00007FF8652F0000), size: 1134592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\MSCTF.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\d3d12.dll:d3d12.dll (00007FF855260000), size: 131072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\d3d12.dll', fileVersion: 10.0.19041.1266
C:\WorkProject\WarDroneB\TestBuild01\D3D12\D3D12Core.dll:D3D12Core.dll (00007FFF9B680000), size: 4780032 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild01\D3D12\D3D12Core.dll', fileVersion: 1.618.1.0
C:\Windows\SYSTEM32\dxcore.dll:dxcore.dll (00007FF85B520000), size: 241664 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxcore.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\msasn1.dll:msasn1.dll (00007FF864320000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\msasn1.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\cryptnet.dll:cryptnet.dll (00007FF85E540000), size: 200704 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\cryptnet.dll', fileVersion: 10.0.19041.906
C:\Windows\SYSTEM32\cryptbase.dll:cryptbase.dll (00007FF8640D0000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\cryptbase.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\drvstore.dll:drvstore.dll (00007FF85DEF0000), size: 1343488 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\drvstore.dll', fileVersion: 10.0.19041.1949
C:\Windows\SYSTEM32\devobj.dll:devobj.dll (00007FF8644E0000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\devobj.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\wintrust.dll:wintrust.dll (0000022D93300000), size: 434176 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\wintrust.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\imagehlp.dll:imagehlp.dll (00007FF865940000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\imagehlp.dll', fileVersion: 10.0.19041.1415
C:\Windows\SYSTEM32\CRYPTSP.dll:CRYPTSP.dll (00007FF8640B0000), size: 98304 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\CRYPTSP.dll', fileVersion: 10.0.19041.546
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll:cpadvai.dll (00007FF864090000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll', fileVersion: 5.0.16639.0
C:\Program Files\Crypto Pro\CSP\cpcsp.dll:cpcsp.dll (00007FF8634A0000), size: 32768 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpcsp.dll', fileVersion: 5.0.14960.0
C:\Program Files\Crypto Pro\CSP\cpcspi.dll:cpcspi.dll (00007FF863280000), size: 2187264 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpcspi.dll', fileVersion: 5.0.18680.0
C:\Program Files\Crypto Pro\CSP\cpsuprt.dll:cpsuprt.dll (00007FF8631E0000), size: 634880 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpsuprt.dll', fileVersion: 5.0.15962.0
C:\Windows\system32\nvspcap64.dll:nvspcap64.dll (00007FF81A2E0000), size: 1339392 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\nvspcap64.dll', fileVersion: 11.0.5.420
C:\Windows\SYSTEM32\ntmarta.dll:ntmarta.dll (00007FF863860000), size: 208896 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\ntmarta.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll:nvppex.dll (00007FFFF4690000), size: 1798144 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\D3DSCache.dll:D3DSCache.dll (00007FFF3FFF0000), size: 176128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\D3DSCache.dll', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\USERENV.dll:USERENV.dll (00007FF8645F0000), size: 188416 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\USERENV.dll', fileVersion: 10.0.19041.572
C:\Windows\SYSTEM32\twinapi.appcore.dll:twinapi.appcore.dll (00007FF85DC80000), size: 2097152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\twinapi.appcore.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll:nvldumdx.dll (00007FF85AFD0000), size: 802816 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll:nvgpucomp64.dll (00007FF8558C0000), size: 64565248 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll:NvMemMapStoragex.dll (00007FF8552D0000), size: 909312 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll'
C:\Windows\SYSTEM32\POWRPROF.dll:POWRPROF.dll (00007FF863C50000), size: 307200 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\POWRPROF.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\UMPDC.dll:UMPDC.dll (00007FF863B10000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\UMPDC.dll'
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll:nvwgf2umx.dll (00007FF84DF90000), size: 92487680 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\DirectML.dll:DirectML.dll (00007FFF9CDB0000), size: 1282048 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\DirectML.dll', fileVersion: 1.0.2007.1310
C:\Windows\SYSTEM32\Cabinet.dll:Cabinet.dll (00007FF854570000), size: 167936 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Cabinet.dll', fileVersion: 5.0.1.1
C:\Windows\SYSTEM32\d3d11on12.dll:d3d11on12.dll (00007FFF40060000), size: 618496 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\d3d11on12.dll', fileVersion: 10.0.19041.1081
C:\Windows\SYSTEM32\dxilconv.dll:dxilconv.dll (00007FFF3F140000), size: 1282048 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxilconv.dll', fileVersion: 10.0.19041.1
C:\Windows\SYSTEM32\DXGIDebug.dll:DXGIDebug.dll (00007FFF40100000), size: 356352 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\DXGIDebug.dll', fileVersion: 10.0.19041.1
C:\Windows\System32\MMDevApi.dll:MMDevApi.dll (00007FF85D150000), size: 544768 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\MMDevApi.dll', fileVersion: 10.0.19041.1503
C:\Windows\SYSTEM32\AUDIOSES.DLL:AUDIOSES.DLL (00007FF8220A0000), size: 1581056 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\AUDIOSES.DLL', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\resourcepolicyclient.dll:resourcepolicyclient.dll (00007FF860F40000), size: 81920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\resourcepolicyclient.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.Input.dll:Windows.Gaming.Input.dll (00007FF85C990000), size: 864256 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Windows.Gaming.Input.dll', fileVersion: 10.0.19041.1566
C:\Windows\SYSTEM32\inputhost.dll:inputhost.dll (00007FF85B170000), size: 1384448 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\inputhost.dll', fileVersion: 10.0.19041.1741
C:\Windows\System32\CoreUIComponents.dll:CoreUIComponents.dll (00007FF860360000), size: 3530752 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CoreUIComponents.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.UI.GameBar.dll:Windows.Gaming.UI.GameBar.dll (00007FFF3B6F0000), size: 110592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Windows.Gaming.UI.GameBar.dll', fileVersion: 10.0.19041.746
C:\Windows\System32\WININET.dll:WININET.dll (00007FF83A250000), size: 5083136 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\WININET.dll', fileVersion: 11.0.19041.2193
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll:cpwinet.dll (00007FF8388E0000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll', fileVersion: 5.0.15621.0
C:\Windows\System32\appresolver.dll:appresolver.dll (00007FF8204B0000), size: 589824 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\appresolver.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\SLC.dll:SLC.dll (00007FF862D90000), size: 167936 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SLC.dll', fileVersion: 10.0.19041.1682
C:\Windows\System32\Bcp47Langs.dll:Bcp47Langs.dll (00007FF85B660000), size: 372736 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Bcp47Langs.dll', fileVersion: 10.0.19041.1566
C:\Windows\System32\sppc.dll:sppc.dll (00007FF862D60000), size: 151552 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\sppc.dll', fileVersion: 10.0.19041.1682
C:\Windows\SYSTEM32\xinput1_3.dll:xinput1_3.dll (0000000000400000), size: 122880 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\xinput1_3.dll', fileVersion: 9.18.944.0
C:\Windows\SYSTEM32\dcomp.dll:dcomp.dll (00007FF860170000), size: 1978368 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dcomp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\textinputframework.dll:textinputframework.dll (00007FF85B2D0000), size: 1019904 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\textinputframework.dll', fileVersion: 10.0.19041.2075
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF923AADBF)
0x00007FFF923AADBF (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF923ADCE5)
0x00007FFF923ADCE5 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF91BA5C8A)
0x00007FFF91BA5C8A (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF9280FCD7)
0x00007FFF9280FCD7 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF9282307E)
0x00007FFF9282307E (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF928013D8)
0x00007FFF928013D8 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF928158E2)
0x00007FFF928158E2 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92815952)
0x00007FFF92815952 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92836F94)
0x00007FFF92836F94 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92839837)
0x00007FFF92839837 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF927FA5D6)
0x00007FFF927FA5D6 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92866002)
0x00007FFF92866002 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92863C36)
0x00007FFF92863C36 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92860868)
0x00007FFF92860868 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF9230B703)
0x00007FFF9230B703 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92306626)
0x00007FFF92306626 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92306C01)
0x00007FFF92306C01 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF92306C7A)
0x00007FFF92306C7A (UnityPlayer) (function-name not available)
0x00007FFF9363930F (UnityPlayer) UnityMain
0x00007FF865097034 (KERNEL32) BaseThreadInitThunk
0x00007FF8670826A1 (ntdll) RtlUserThreadStart

========== END OF STACKTRACE ===========

A crash has been intercepted by the crash handler. For call stack and other details, see the latest crash report generated in:
 * C:/Users/KirikDK/AppData/Local/Temp/WarDrone/MyDroneA/Crashes


### ASSISTANT
Разберём лог — там три независимые проблемы, и главная не в скриптах дрона.

## Что говорит лог

| Строка | Что значит |
|---|---|
| `The file '...MyDroneA_Data/level0' is corrupted! [Position out of bounds!]` | Движок **не смог десериализовать первую сцену** билда. Это происходит **до** запуска любых наших скриптов — значит логика дрона тут ни при чём. |
| `d3d12: upload buffer was too small for the requested resource! Size: 16777216. Requested: 33307392.` | D3D12 не смог загрузить ресурс на **33 МБ** (буфер 16 МБ). Прямая причина падения видеоподсистемы. |
| `Detected one or more triangles ... Source mesh name: extrudeFront / FonBuildings` | В сцене есть **гигантские меши** (город/фон) с треугольниками >500 юнитов — PhysX предупреждает о нестабильности. Это не наши примитивы дрона. |
| `GameAssembly.dll` в списке модулей | Сборка на **IL2CPP** (стриппинг кода). |
| `kGfxThreadingModeSplitJobs` | Включены **Graphics Jobs** — в связке с D3D12 в Unity 6 это частый источник крашей. |
| `cpcrypt.dll`, `cpcsp.dll`, `cpwinet.dll` (CryptoPro) в процессе | КриптоПро внедряется в процесс и перехватывает файловые операции — известны случаи порчи файлов сборки. |

---

## Шаг 1. Настройки плеера (это чинит краш в первую очередь)

`Edit ▸ Project Settings ▸ Player ▸ Other Settings`:

1. **Rendering ▸ Auto Graphics API for Windows** — снять галочку. В списке **Graphics APIs for Windows** удалить **Direct3D12**, оставить **Direct3D11** (перетащить наверх).
2. **Rendering ▸ Graphics Jobs** — **выключить**.
3. **Configuration ▸ Scripting Backend** — для теста поставь **Mono** (если с Mono запустится — виноват стриппинг IL2CPP, см. шаг 4).
4. **Optimization ▸ Managed Stripping Level** — **Minimal** (если остаёшься на IL2CPP).

В `File ▸ Build Profiles` (или Build Settings):
5. **Compression Method = Disabled** (или LZ4). LZMA-архив при обрыве записи даёт именно «corrupted level0».
6. Включи **Clean Build** и собирай в **новую пустую папку**, например `C:/WorkProject/WarDroneB/Build02`. Старую `TestBuild01` удали целиком — смешивать старые и новые данные нельзя.

---

## Шаг 2. Проверь, какая сцена вообще грузится

`level0` — это **первая сцена** в списке Build Profiles/Build Settings.

- Открой список сцен и убедись, что первой стоит **наша сцена с дроном**, а не тяжёлая городская.
- Для изоляции проблемы: временно оставь в списке **только сцену дрона** (без города) и собери. Если запустилось — проблема в городской геометрии (шаг 3).

Заодно посмотри размер файла `MyDroneA_Data/level0` в старой сборке:
- **0 КБ / неестественно маленький** → сборка физически оборвалась (антивирус/диск);
- **сотни МБ / ГБ** → сцена слишком тяжёлая, упирается в лимиты сериализации.

---

## Шаг 3. Городская геометрия (extrudeFront / FonBuildings)

Это фон, ему не нужна физика и не нужны гигантские треугольники:

1. Выдели объекты `extrudeFront`, `FonBuildings` → **удали Mesh Collider** (или замени на Box Collider). Оставь коллайдеры только там, где дрон реально может столкнуться.
2. У мешей в импорте: **Read/Write Enabled = Off**, **Mesh Compression = Medium/High** — это заметно уменьшит и память, и размер билда.
3. Если объект масштабирован до огромных размеров — лучше уменьшить масштаб и разбить меш на части, чем оставлять один треугольник на 500+ юнитов.
4. Проверь текстуры: 33 МБ за одну загрузку — это обычно огромная текстура или меш. Уменьши Max Size у фоновых текстур до 2048.

---

## Шаг 4. Страховка для IL2CPP — `Assets/link.xml`

Наши `JsonUtility`-типы (`CalibrationData`, `TelemetryData`, вложенный `UdpLink.RxPacket`) могут быть вырезаны стриппером. Создай файл **`Assets/link.xml`**:

```xml
<linker>
  <assembly fullname="Assembly-CSharp" preserve="all"/>
</linker>
```

Это полностью отключает стриппинг нашего кода — для симулятора размер не критичен, зато исчезает целый класс трудноуловимых багов в билде.

---

## Шаг 5. Антивирус / CryptoPro

Добавь в исключения Защитника Windows и любого другого антивируса:
- папку проекта `C:\WorkProject\WarDroneB\`
- папку сборок
- `Unity.exe`, `UnityShaderCompiler.exe`

CryptoPro (`cpcrypt.dll` и компания) внедряется в процесс и лезет в файловые операции — на время сборки его лучше отключить, это известный источник повреждённых файлов.

---

## Шаг 6. Если всё ещё падает — чистая пересборка

1. Закрой Unity.
2. Удали в проекте: `Library/Bee`, `Library/PlayerDataCache`, `Library/StateCache` (весь `Library` удалять не обязательно — будет долгий реимпорт).
3. Удали папку билда.
4. Открой Unity, дай переимпортировать, собери **Clean Build** в новую папку.

И полезно собрать **Development Build + Script Debugging** — тогда в логе будет больше деталей, а если краш всё-таки в нашем коде, появится читаемый стек.

---

## Порядок действий, если коротко

1. D3D11 вместо D3D12 + Graphics Jobs off + Compression Disabled.
2. Clean Build в **новую** папку.
3. Не помогло → Scripting Backend = Mono + `link.xml`.
4. Не помогло → собрать **только сцену дрона**, без города.
5. Заработало без города → чинить меши/коллайдеры фона (шаг 3).

Напиши, что получилось на каждом шаге, и заодно: **какой размер у файла `level0`** и **какая сцена стоит первой** в списке сборки. По этим двум ответам сразу станет ясно, битая это сборка или сцена не влезает в лимиты — и добьём точечно.

### USER
я сделал все что написано и все равно вылетает билд в .exe как это исправтиь? вот лог:
Mono path[0] = 'C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/Managed'
Mono config path = 'C:/WorkProject/WarDroneB/TestBuild04/MonoBleedingEdge/etc'
Found 3 interfaces on host : 0) 192.168.0.103 1) 172.24.228.220 2) 10.6.7.1
Player connection [34380]  Target information:

Player connection [34380]  * "[IP] 192.168.0.103 [Port] 55000 [Flags] 2 [Guid] 3915590549 [EditorId] 1337524287 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 0 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [34380]  * "[IP] 172.24.228.220 [Port] 55000 [Flags] 2 [Guid] 3915590549 [EditorId] 1337524287 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 0 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [34380]  * "[IP] 10.6.7.1 [Port] 55000 [Flags] 2 [Guid] 3915590549 [EditorId] 1337524287 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 0 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [34380] Started UDP target info broadcast (1) on [225.0.0.222:54997].

Input System module state changed to: Initialized.
[Physics::Module] Initialized fallback backend.
[Physics::Module] Id: 0xdecafbad
Initialize engine version: 6000.3.28f1 (8f082d2fe476)
[Subsystems] Discovering subsystems at path C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/UnitySubsystems
GfxDevice: creating device client; kGfxThreadingModeThreaded
Direct3D:
    Version:  Direct3D 11.0 [level 11.1]
    Renderer: NVIDIA GeForce RTX 3090 (ID=0x2204)
    Vendor:   NVIDIA
    VRAM:     24326 MB
    Driver:   32.0.15.8108
Begin MonoManager ReloadAssembly
- Loaded All Assemblies, in  0.196 seconds
- Finished resetting the current domain, in  0.004 seconds
[Physics::Module] Selected backend.
[Physics::Module] Name: PhysX
[Physics::Module] Id: 0xf2b8ea05
[Physics::Module] SDK Version: 4.1.2
[Physics::Module] Integration Version: 1.0.0
[Physics::Module] Threading Mode: Multi-Threaded
<RI> Initializing input.

Using Windows.Gaming.Input
<RI> Input initialized.

The file 'C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/level0' is corrupted! Remove it and launch unity again!
[Position out of bounds!]
Crash!!!
SymInit: Symbol-SearchPath: '.;C:\WorkProject\WarDroneB\TestBuild04;C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/Plugins\x86_64;C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/Plugins;C:\WorkProject\WarDroneB\TestBuild04;C:\Windows;C:\Windows\system32;', symOptions: 534, UserName: 'KirikDK'
OS-Version: 10.0.0
C:\WorkProject\WarDroneB\TestBuild04\MyDroneA.exe:MyDroneA.exe (00007FF7F40E0000), size: 679936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 6000.3.28.2093
C:\Windows\SYSTEM32\ntdll.dll:ntdll.dll (00007FF867030000), size: 2064384 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\KERNEL32.DLL:KERNEL32.DLL (00007FF865080000), size: 774144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1889
C:\Windows\System32\KERNELBASE.dll:KERNELBASE.dll (00007FF864860000), size: 2957312 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\WorkProject\WarDroneB\TestBuild04\UnityPlayer.dll:UnityPlayer.dll (00007FFF8AE50000), size: 92422144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 6000.3.28.2093
C:\Windows\System32\USER32.dll:USER32.dll (00007FF8657A0000), size: 1691648 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\win32u.dll:win32u.dll (00007FF864FA0000), size: 139264 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\GDI32.dll:GDI32.dll (00007FF8652C0000), size: 176128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\gdi32full.dll:gdi32full.dll (00007FF864D90000), size: 1110016 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\msvcp_win.dll:msvcp_win.dll (00007FF864730000), size: 643072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.789
C:\Windows\System32\ucrtbase.dll:ucrtbase.dll (00007FF864EA0000), size: 1048576 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.789
C:\Windows\System32\ole32.dll:ole32.dll (00007FF865B80000), size: 1220608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1202
C:\Windows\System32\RPCRT4.dll:RPCRT4.dll (00007FF865410000), size: 1200128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1806
C:\Windows\System32\combase.dll:combase.dll (00007FF865E20000), size: 3489792 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\VERSION.dll:VERSION.dll (00007FF85E580000), size: 40960 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\msvcrt.dll:msvcrt.dll (00007FF865140000), size: 647168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.0.19041.546
C:\Windows\System32\SHLWAPI.dll:SHLWAPI.dll (00007FF865B20000), size: 348160 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\System32\SETUPAPI.dll:SETUPAPI.dll (00007FF866230000), size: 4620288 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\cfgmgr32.dll:cfgmgr32.dll (00007FF864BB0000), size: 319488 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\bcrypt.dll:bcrypt.dll (00007FF864D60000), size: 159744 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1023
C:\Windows\System32\ADVAPI32.dll:ADVAPI32.dll (00007FF866180000), size: 712704 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\sechost.dll:sechost.dll (00007FF865220000), size: 638976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\SHELL32.dll:SHELL32.dll (00007FF866820000), size: 7614464 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\System32\OLEAUT32.dll:OLEAUT32.dll (00007FF866750000), size: 839680 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.985
C:\Windows\System32\IMM32.dll:IMM32.dll (00007FF8651E0000), size: 204800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\OPENGL32.dll:OPENGL32.dll (00007FF8174C0000), size: 1200128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\WINMM.dll:WINMM.dll (00007FF8552A0000), size: 159744 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\CRYPT32.dll:CRYPT32.dll (00007FF864C00000), size: 1400832 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1889
C:\Windows\System32\WS2_32.dll:WS2_32.dll (00007FF865730000), size: 438272 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\IPHLPAPI.DLL:IPHLPAPI.DLL (00007FF863B30000), size: 241664 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\WINHTTP.dll:WINHTTP.dll (00007FF85D2A0000), size: 1097728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\HID.DLL:HID.DLL (00007FF862AB0000), size: 53248 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\d3d11.dll:d3d11.dll (00007FF8627F0000), size: 2502656 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dxgi.dll:dxgi.dll (00007FF862B10000), size: 995328 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\GLU32.dll:GLU32.dll (00007FF818AB0000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\UIAutomationCore.DLL:UIAutomationCore.DLL (00007FF8113E0000), size: 3100672 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.2.19041.1741
C:\Windows\SYSTEM32\Secur32.dll:Secur32.dll (00007FF84A270000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dwmapi.dll:dwmapi.dll (00007FF861040000), size: 192512 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\PROPSYS.dll:PROPSYS.dll (00007FF85E9F0000), size: 1007616 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.0.19041.1741
C:\Windows\SYSTEM32\SSPICLI.DLL:SSPICLI.DLL (00007FF864620000), size: 204800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll:cpcrypt.dll (00007FF864200000), size: 135168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.16728.0
C:\Windows\system32\apphelp.dll:apphelp.dll (00007FF860C70000), size: 593920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\kernel.appcore.dll:kernel.appcore.dll (00007FF862AC0000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\bcryptPrimitives.dll:bcryptPrimitives.dll (00007FF8647D0000), size: 532480 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1415
C:\Windows\system32\uxtheme.dll:uxtheme.dll (00007FF860D70000), size: 647168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\shcore.dll:shcore.dll (00007FF865D70000), size: 708608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\clbcatq.dll:clbcatq.dll (00007FF8666A0000), size: 716800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 2001.12.10941.16384
C:\Windows\system32\wbem\wbemprox.dll:wbemprox.dll (00007FF84B4D0000), size: 69632 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1320
C:\Windows\SYSTEM32\wbemcomn.dll:wbemcomn.dll (00007FF84B0D0000), size: 589824 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\system32\wbem\wbemsvc.dll:wbemsvc.dll (00007FF84A350000), size: 81920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1320
C:\Windows\system32\wbem\fastprox.dll:fastprox.dll (00007FF84A440000), size: 1093632 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\amsi.dll:amsi.dll (00007FF849910000), size: 126976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\WorkProject\WarDroneB\TestBuild04\WinPixEventRuntime.dll:WinPixEventRuntime.dll (00007FF85E830000), size: 57344 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 1.0.2403.8001
C:\Windows\SYSTEM32\MSVCP140.dll:MSVCP140.dll (00007FF847190000), size: 643072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140_1.dll:VCRUNTIME140_1.dll (00007FF847260000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140.dll:VCRUNTIME140.dll (00007FF847230000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\windows.storage.dll:windows.storage.dll (00007FF861510000), size: 7933952 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\Wldp.dll:Wldp.dll (00007FF864160000), size: 196608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\profapi.dll:profapi.dll (00007FF864670000), size: 126976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.844
C:\WorkProject\WarDroneB\TestBuild04\MonoBleedingEdge\EmbedRuntime\mono-2.0-bdwgc.dll:mono-2.0-bdwgc.dll (00007FFF8A400000), size: 10756096 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\system32\mswsock.dll:mswsock.dll (00007FF863E90000), size: 434176 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\NSI.dll:NSI.dll (00007FF865960000), size: 32768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.610
C:\Windows\SYSTEM32\dhcpcsvc6.DLL:dhcpcsvc6.DLL (00007FF85E590000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dhcpcsvc.DLL:dhcpcsvc.DLL (00007FF85E4F0000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\DNSAPI.dll:DNSAPI.dll (00007FF863B70000), size: 827392 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\system32\napinsp.dll:napinsp.dll (00007FF83A9E0000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\pnrpnsp.dll:pnrpnsp.dll (00007FF83A9C0000), size: 110592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\wshbth.dll:wshbth.dll (00007FF83A9A0000), size: 86016 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\NLAapi.dll:NLAapi.dll (00007FF85FA30000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\winrnr.dll:winrnr.dll (00007FF83A980000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Program Files\Bonjour\mdnsNSP.dll:mdnsNSP.dll (00000000632E0000), size: 155648 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 3.0.0.10
C:\Windows\System32\MSCTF.dll:MSCTF.dll (00007FF8652F0000), size: 1134592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll:nvldumdx.dll (00007FF85AFD0000), size: 802816 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\msasn1.dll:msasn1.dll (00007FF864320000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\cryptnet.dll:cryptnet.dll (00007FF85E540000), size: 200704 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.906
C:\Windows\SYSTEM32\cryptbase.dll:cryptbase.dll (00007FF8640D0000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\drvstore.dll:drvstore.dll (00007FF85DEF0000), size: 1343488 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1949
C:\Windows\SYSTEM32\devobj.dll:devobj.dll (00007FF8644E0000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\wintrust.dll:wintrust.dll (000002B35A0A0000), size: 434176 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\imagehlp.dll:imagehlp.dll (00007FF865940000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1415
C:\Windows\SYSTEM32\CRYPTSP.dll:CRYPTSP.dll (00007FF8640B0000), size: 98304 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll:cpadvai.dll (00007FF864090000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.16639.0
C:\Program Files\Crypto Pro\CSP\cpcsp.dll:cpcsp.dll (00007FF8634A0000), size: 32768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.14960.0
C:\Program Files\Crypto Pro\CSP\cpcspi.dll:cpcspi.dll (00007FF863280000), size: 2187264 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.18680.0
C:\Program Files\Crypto Pro\CSP\cpsuprt.dll:cpsuprt.dll (00007FF8631E0000), size: 634880 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.15962.0
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll:nvgpucomp64.dll (00007FF8558C0000), size: 64565248 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll:NvMemMapStoragex.dll (00007FF8552D0000), size: 909312 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\SYSTEM32\POWRPROF.dll:POWRPROF.dll (00007FF863C50000), size: 307200 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\UMPDC.dll:UMPDC.dll (00007FF863B10000), size: 73728 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll:nvwgf2umx.dll (00007FF84DF90000), size: 92487680 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\system32\nvspcap64.dll:nvspcap64.dll (00007FF81A2E0000), size: 1339392 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 11.0.5.420
C:\Windows\SYSTEM32\ntmarta.dll:ntmarta.dll (00007FF863860000), size: 208896 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dxcore.dll:dxcore.dll (00007FF85B520000), size: 241664 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll:nvppex.dll (00007FFFF4690000), size: 1798144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\System32\MMDevApi.dll:MMDevApi.dll (00007FF85D150000), size: 544768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1503
C:\Windows\SYSTEM32\AUDIOSES.DLL:AUDIOSES.DLL (00007FF8220A0000), size: 1581056 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\resourcepolicyclient.dll:resourcepolicyclient.dll (00007FF860F40000), size: 81920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.Input.dll:Windows.Gaming.Input.dll (00007FF85C990000), size: 864256 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\SYSTEM32\inputhost.dll:inputhost.dll (00007FF85B170000), size: 1384448 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1741
C:\Windows\System32\CoreMessaging.dll:CoreMessaging.dll (00007FF860A40000), size: 991232 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\CoreUIComponents.dll:CoreUIComponents.dll (00007FF860360000), size: 3530752 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\wintypes.dll:wintypes.dll (00007FF85FBA0000), size: 1392640 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\twinapi.appcore.dll:twinapi.appcore.dll (00007FF85DC80000), size: 2097152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\Windows.Gaming.UI.GameBar.dll:Windows.Gaming.UI.GameBar.dll (00007FFF963A0000), size: 110592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\System32\WININET.dll:WININET.dll (00007FF83A250000), size: 5083136 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 11.0.19041.2193
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll:cpwinet.dll (00007FF8388E0000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.15621.0
C:\Windows\System32\appresolver.dll:appresolver.dll (00007FF8204B0000), size: 589824 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\Bcp47Langs.dll:Bcp47Langs.dll (00007FF85B660000), size: 372736 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\System32\USERENV.dll:USERENV.dll (00007FF8645F0000), size: 188416 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.572
C:\Windows\System32\SLC.dll:SLC.dll (00007FF862D90000), size: 167936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1682
C:\Windows\System32\sppc.dll:sppc.dll (00007FF862D60000), size: 151552 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1682
C:\Windows\SYSTEM32\xinput1_3.dll:xinput1_3.dll (0000000000400000), size: 122880 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 9.18.944.0
C:\Windows\SYSTEM32\dcomp.dll:dcomp.dll (00007FF860170000), size: 1978368 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\textinputframework.dll:textinputframework.dll (00007FF85B2D0000), size: 1019904 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dbghelp.dll:dbghelp.dll (00007FF862600000), size: 1982464 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.867

========== OUTPUTTING STACK TRACE ==================

0x00007FFF8D0B573D (UnityPlayer) UnityMain
0x00007FFF8D0B4B59 (UnityPlayer) UnityMain
SymInit: Symbol-SearchPath: '.;C:\WorkProject\WarDroneB\TestBuild04;C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/Plugins\x86_64;C:/WorkProject/WarDroneB/TestBuild04/MyDroneA_Data/Plugins;C:\WorkProject\WarDroneB\TestBuild04;C:\Windows;C:\Windows\system32;', symOptions: 534, UserName: 'KirikDK'
OS-Version: 10.0.0
C:\WorkProject\WarDroneB\TestBuild04\MyDroneA.exe:MyDroneA.exe (00007FF7F40E0000), size: 679936 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild04\MyDroneA.exe', fileVersion: 6000.3.28.2093
C:\Windows\SYSTEM32\ntdll.dll:ntdll.dll (00007FF867030000), size: 2064384 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\ntdll.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\KERNEL32.DLL:KERNEL32.DLL (00007FF865080000), size: 774144 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\KERNEL32.DLL', fileVersion: 10.0.19041.1889
C:\Windows\System32\KERNELBASE.dll:KERNELBASE.dll (00007FF864860000), size: 2957312 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\KERNELBASE.dll', fileVersion: 10.0.19041.2193
C:\WorkProject\WarDroneB\TestBuild04\UnityPlayer.dll:UnityPlayer.dll (00007FFF8AE50000), size: 92422144 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild04\UnityPlayer.dll', fileVersion: 6000.3.28.2093
C:\Windows\System32\USER32.dll:USER32.dll (00007FF8657A0000), size: 1691648 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\USER32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\win32u.dll:win32u.dll (00007FF864FA0000), size: 139264 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\win32u.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\GDI32.dll:GDI32.dll (00007FF8652C0000), size: 176128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\GDI32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\gdi32full.dll:gdi32full.dll (00007FF864D90000), size: 1110016 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\gdi32full.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\msvcp_win.dll:msvcp_win.dll (00007FF864730000), size: 643072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\msvcp_win.dll', fileVersion: 10.0.19041.789
C:\Windows\System32\ucrtbase.dll:ucrtbase.dll (00007FF864EA0000), size: 1048576 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ucrtbase.dll', fileVersion: 10.0.19041.789
C:\Windows\System32\ole32.dll:ole32.dll (00007FF865B80000), size: 1220608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ole32.dll', fileVersion: 10.0.19041.1202
C:\Windows\System32\RPCRT4.dll:RPCRT4.dll (00007FF865410000), size: 1200128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\RPCRT4.dll', fileVersion: 10.0.19041.1806
C:\Windows\System32\combase.dll:combase.dll (00007FF865E20000), size: 3489792 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\combase.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\VERSION.dll:VERSION.dll (00007FF85E580000), size: 40960 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VERSION.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\msvcrt.dll:msvcrt.dll (00007FF865140000), size: 647168 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\msvcrt.dll', fileVersion: 7.0.19041.546
C:\Windows\System32\SHLWAPI.dll:SHLWAPI.dll (00007FF865B20000), size: 348160 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SHLWAPI.dll', fileVersion: 10.0.19041.2075
C:\Windows\System32\SETUPAPI.dll:SETUPAPI.dll (00007FF866230000), size: 4620288 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SETUPAPI.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\cfgmgr32.dll:cfgmgr32.dll (00007FF864BB0000), size: 319488 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\cfgmgr32.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\bcrypt.dll:bcrypt.dll (00007FF864D60000), size: 159744 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\bcrypt.dll', fileVersion: 10.0.19041.1023
C:\Windows\System32\ADVAPI32.dll:ADVAPI32.dll (00007FF866180000), size: 712704 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ADVAPI32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\sechost.dll:sechost.dll (00007FF865220000), size: 638976 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\sechost.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\SHELL32.dll:SHELL32.dll (00007FF866820000), size: 7614464 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SHELL32.dll', fileVersion: 10.0.19041.2075
C:\Windows\System32\OLEAUT32.dll:OLEAUT32.dll (00007FF866750000), size: 839680 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\OLEAUT32.dll', fileVersion: 10.0.19041.985
C:\Windows\System32\IMM32.dll:IMM32.dll (00007FF8651E0000), size: 204800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\IMM32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\OPENGL32.dll:OPENGL32.dll (00007FF8174C0000), size: 1200128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\OPENGL32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\WINMM.dll:WINMM.dll (00007FF8552A0000), size: 159744 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\WINMM.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\CRYPT32.dll:CRYPT32.dll (00007FF864C00000), size: 1400832 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CRYPT32.dll', fileVersion: 10.0.19041.1889
C:\Windows\System32\WS2_32.dll:WS2_32.dll (00007FF865730000), size: 438272 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\WS2_32.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\IPHLPAPI.DLL:IPHLPAPI.DLL (00007FF863B30000), size: 241664 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\IPHLPAPI.DLL', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\WINHTTP.dll:WINHTTP.dll (00007FF85D2A0000), size: 1097728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\WINHTTP.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\HID.DLL:HID.DLL (00007FF862AB0000), size: 53248 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\HID.DLL', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\d3d11.dll:d3d11.dll (00007FF8627F0000), size: 2502656 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\d3d11.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dxgi.dll:dxgi.dll (00007FF862B10000), size: 995328 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxgi.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\GLU32.dll:GLU32.dll (00007FF818AB0000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\GLU32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\UIAutomationCore.DLL:UIAutomationCore.DLL (00007FF8113E0000), size: 3100672 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\UIAutomationCore.DLL', fileVersion: 7.2.19041.1741
C:\Windows\SYSTEM32\Secur32.dll:Secur32.dll (00007FF84A270000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Secur32.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dwmapi.dll:dwmapi.dll (00007FF861040000), size: 192512 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dwmapi.dll', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\PROPSYS.dll:PROPSYS.dll (00007FF85E9F0000), size: 1007616 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\PROPSYS.dll', fileVersion: 7.0.19041.1741
C:\Windows\SYSTEM32\SSPICLI.DLL:SSPICLI.DLL (00007FF864620000), size: 204800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\SSPICLI.DLL', fileVersion: 10.0.19041.2130
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll:cpcrypt.dll (00007FF864200000), size: 135168 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll', fileVersion: 5.0.16728.0
C:\Windows\system32\apphelp.dll:apphelp.dll (00007FF860C70000), size: 593920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\apphelp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\kernel.appcore.dll:kernel.appcore.dll (00007FF862AC0000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\kernel.appcore.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\bcryptPrimitives.dll:bcryptPrimitives.dll (00007FF8647D0000), size: 532480 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\bcryptPrimitives.dll', fileVersion: 10.0.19041.1415
C:\Windows\system32\uxtheme.dll:uxtheme.dll (00007FF860D70000), size: 647168 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\uxtheme.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\shcore.dll:shcore.dll (00007FF865D70000), size: 708608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\shcore.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\clbcatq.dll:clbcatq.dll (00007FF8666A0000), size: 716800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\clbcatq.dll', fileVersion: 2001.12.10941.16384
C:\Windows\system32\wbem\wbemprox.dll:wbemprox.dll (00007FF84B4D0000), size: 69632 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\wbemprox.dll', fileVersion: 10.0.19041.1320
C:\Windows\SYSTEM32\wbemcomn.dll:wbemcomn.dll (00007FF84B0D0000), size: 589824 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\wbemcomn.dll', fileVersion: 10.0.19041.1566
C:\Windows\system32\wbem\wbemsvc.dll:wbemsvc.dll (00007FF84A350000), size: 81920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\wbemsvc.dll', fileVersion: 10.0.19041.1320
C:\Windows\system32\wbem\fastprox.dll:fastprox.dll (00007FF84A440000), size: 1093632 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\fastprox.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\amsi.dll:amsi.dll (00007FF849910000), size: 126976 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\amsi.dll', fileVersion: 10.0.19041.2075
C:\WorkProject\WarDroneB\TestBuild04\WinPixEventRuntime.dll:WinPixEventRuntime.dll (00007FF85E830000), size: 57344 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild04\WinPixEventRuntime.dll', fileVersion: 1.0.2403.8001
C:\Windows\SYSTEM32\MSVCP140.dll:MSVCP140.dll (00007FF847190000), size: 643072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\MSVCP140.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140_1.dll:VCRUNTIME140_1.dll (00007FF847260000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VCRUNTIME140_1.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140.dll:VCRUNTIME140.dll (00007FF847230000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VCRUNTIME140.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\windows.storage.dll:windows.storage.dll (00007FF861510000), size: 7933952 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\windows.storage.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\Wldp.dll:Wldp.dll (00007FF864160000), size: 196608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Wldp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\profapi.dll:profapi.dll (00007FF864670000), size: 126976 (result: 0), SymType: '-nosymbols-', PDB: 'C:\Windows\SYSTEM32\profapi.dll', fileVersion: 10.0.19041.844
C:\WorkProject\WarDroneB\TestBuild04\MonoBleedingEdge\EmbedRuntime\mono-2.0-bdwgc.dll:mono-2.0-bdwgc.dll (00007FFF8A400000), size: 10756096 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild04\MonoBleedingEdge\EmbedRuntime\mono-2.0-bdwgc.dll'
C:\Windows\system32\mswsock.dll:mswsock.dll (00007FF863E90000), size: 434176 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\mswsock.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\NSI.dll:NSI.dll (00007FF865960000), size: 32768 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\NSI.dll', fileVersion: 10.0.19041.610
C:\Windows\SYSTEM32\dhcpcsvc6.DLL:dhcpcsvc6.DLL (00007FF85E590000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dhcpcsvc6.DLL', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dhcpcsvc.DLL:dhcpcsvc.DLL (00007FF85E4F0000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dhcpcsvc.DLL', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\DNSAPI.dll:DNSAPI.dll (00007FF863B70000), size: 827392 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\DNSAPI.dll', fileVersion: 10.0.19041.1865
C:\Windows\system32\napinsp.dll:napinsp.dll (00007FF83A9E0000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\napinsp.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\pnrpnsp.dll:pnrpnsp.dll (00007FF83A9C0000), size: 110592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\pnrpnsp.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\wshbth.dll:wshbth.dll (00007FF83A9A0000), size: 86016 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wshbth.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\NLAapi.dll:NLAapi.dll (00007FF85FA30000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\NLAapi.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\winrnr.dll:winrnr.dll (00007FF83A980000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\winrnr.dll', fileVersion: 10.0.19041.546
C:\Program Files\Bonjour\mdnsNSP.dll:mdnsNSP.dll (00000000632E0000), size: 155648 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Bonjour\mdnsNSP.dll', fileVersion: 3.0.0.10
C:\Windows\System32\MSCTF.dll:MSCTF.dll (00007FF8652F0000), size: 1134592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\MSCTF.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll:nvldumdx.dll (00007FF85AFD0000), size: 802816 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\msasn1.dll:msasn1.dll (00007FF864320000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\msasn1.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\cryptnet.dll:cryptnet.dll (00007FF85E540000), size: 200704 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\cryptnet.dll', fileVersion: 10.0.19041.906
C:\Windows\SYSTEM32\cryptbase.dll:cryptbase.dll (00007FF8640D0000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\cryptbase.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\drvstore.dll:drvstore.dll (00007FF85DEF0000), size: 1343488 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\drvstore.dll', fileVersion: 10.0.19041.1949
C:\Windows\SYSTEM32\devobj.dll:devobj.dll (00007FF8644E0000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\devobj.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\wintrust.dll:wintrust.dll (000002B35A0A0000), size: 434176 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\wintrust.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\imagehlp.dll:imagehlp.dll (00007FF865940000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\imagehlp.dll', fileVersion: 10.0.19041.1415
C:\Windows\SYSTEM32\CRYPTSP.dll:CRYPTSP.dll (00007FF8640B0000), size: 98304 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\CRYPTSP.dll', fileVersion: 10.0.19041.546
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll:cpadvai.dll (00007FF864090000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll', fileVersion: 5.0.16639.0
C:\Program Files\Crypto Pro\CSP\cpcsp.dll:cpcsp.dll (00007FF8634A0000), size: 32768 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpcsp.dll', fileVersion: 5.0.14960.0
C:\Program Files\Crypto Pro\CSP\cpcspi.dll:cpcspi.dll (00007FF863280000), size: 2187264 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpcspi.dll', fileVersion: 5.0.18680.0
C:\Program Files\Crypto Pro\CSP\cpsuprt.dll:cpsuprt.dll (00007FF8631E0000), size: 634880 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpsuprt.dll', fileVersion: 5.0.15962.0
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll:nvgpucomp64.dll (00007FF8558C0000), size: 64565248 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll:NvMemMapStoragex.dll (00007FF8552D0000), size: 909312 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll'
C:\Windows\SYSTEM32\POWRPROF.dll:POWRPROF.dll (00007FF863C50000), size: 307200 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\POWRPROF.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\UMPDC.dll:UMPDC.dll (00007FF863B10000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\UMPDC.dll'
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll:nvwgf2umx.dll (00007FF84DF90000), size: 92487680 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll', fileVersion: 32.0.15.8108
C:\Windows\system32\nvspcap64.dll:nvspcap64.dll (00007FF81A2E0000), size: 1339392 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\nvspcap64.dll', fileVersion: 11.0.5.420
C:\Windows\SYSTEM32\ntmarta.dll:ntmarta.dll (00007FF863860000), size: 208896 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\ntmarta.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dxcore.dll:dxcore.dll (00007FF85B520000), size: 241664 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxcore.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll:nvppex.dll (00007FFFF4690000), size: 1798144 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll', fileVersion: 32.0.15.8108
C:\Windows\System32\MMDevApi.dll:MMDevApi.dll (00007FF85D150000), size: 544768 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\MMDevApi.dll', fileVersion: 10.0.19041.1503
C:\Windows\SYSTEM32\AUDIOSES.DLL:AUDIOSES.DLL (00007FF8220A0000), size: 1581056 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\AUDIOSES.DLL', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\resourcepolicyclient.dll:resourcepolicyclient.dll (00007FF860F40000), size: 81920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\resourcepolicyclient.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.Input.dll:Windows.Gaming.Input.dll (00007FF85C990000), size: 864256 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Windows.Gaming.Input.dll', fileVersion: 10.0.19041.1566
C:\Windows\SYSTEM32\inputhost.dll:inputhost.dll (00007FF85B170000), size: 1384448 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\inputhost.dll', fileVersion: 10.0.19041.1741
C:\Windows\System32\CoreMessaging.dll:CoreMessaging.dll (00007FF860A40000), size: 991232 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CoreMessaging.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\CoreUIComponents.dll:CoreUIComponents.dll (00007FF860360000), size: 3530752 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CoreUIComponents.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\wintypes.dll:wintypes.dll (00007FF85FBA0000), size: 1392640 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\wintypes.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\twinapi.appcore.dll:twinapi.appcore.dll (00007FF85DC80000), size: 2097152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\twinapi.appcore.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\Windows.Gaming.UI.GameBar.dll:Windows.Gaming.UI.GameBar.dll (00007FFF963A0000), size: 110592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Windows.Gaming.UI.GameBar.dll', fileVersion: 10.0.19041.746
C:\Windows\System32\WININET.dll:WININET.dll (00007FF83A250000), size: 5083136 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\WININET.dll', fileVersion: 11.0.19041.2193
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll:cpwinet.dll (00007FF8388E0000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll', fileVersion: 5.0.15621.0
C:\Windows\System32\appresolver.dll:appresolver.dll (00007FF8204B0000), size: 589824 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\appresolver.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\Bcp47Langs.dll:Bcp47Langs.dll (00007FF85B660000), size: 372736 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Bcp47Langs.dll', fileVersion: 10.0.19041.1566
C:\Windows\System32\USERENV.dll:USERENV.dll (00007FF8645F0000), size: 188416 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\USERENV.dll', fileVersion: 10.0.19041.572
C:\Windows\System32\SLC.dll:SLC.dll (00007FF862D90000), size: 167936 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SLC.dll', fileVersion: 10.0.19041.1682
C:\Windows\System32\sppc.dll:sppc.dll (00007FF862D60000), size: 151552 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\sppc.dll', fileVersion: 10.0.19041.1682
C:\Windows\SYSTEM32\xinput1_3.dll:xinput1_3.dll (0000000000400000), size: 122880 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\xinput1_3.dll', fileVersion: 9.18.944.0
C:\Windows\SYSTEM32\dcomp.dll:dcomp.dll (00007FF860170000), size: 1978368 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dcomp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\textinputframework.dll:textinputframework.dll (00007FF85B2D0000), size: 1019904 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\textinputframework.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dbghelp.dll:dbghelp.dll (00007FF862600000), size: 1982464 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dbghelp.dll', fileVersion: 10.0.19041.867
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BADA6EF)
0x00007FFF8BADA6EF (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BADD615)
0x00007FFF8BADD615 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8B2D515A)
0x00007FFF8B2D515A (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF4BED7)
0x00007FFF8BF4BED7 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF5F37E)
0x00007FFF8BF5F37E (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF3D558)
0x00007FFF8BF3D558 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF51B7F)
0x00007FFF8BF51B7F (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF51C12)
0x00007FFF8BF51C12 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF73304)
0x00007FFF8BF73304 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF76007)
0x00007FFF8BF76007 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF11356)
0x00007FFF8BF11356 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BFA2D42)
0x00007FFF8BFA2D42 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BFA0976)
0x00007FFF8BFA0976 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BF9D5A8)
0x00007FFF8BF9D5A8 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BA3AEE3)
0x00007FFF8BA3AEE3 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BA35E06)
0x00007FFF8BA35E06 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BA363E1)
0x00007FFF8BA363E1 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BA3645A)
0x00007FFF8BA3645A (UnityPlayer) (function-name not available)
0x00007FFF8CD76B9F (UnityPlayer) UnityMain
0x00007FF865097034 (KERNEL32) BaseThreadInitThunk
0x00007FF8670826A1 (ntdll) RtlUserThreadStart

========== END OF STACKTRACE ===========

A crash has been intercepted by the crash handler. For call stack and other details, see the latest crash report generated in:
 * C:/Users/KirikDK/AppData/Local/Temp/WarDrone/MyDroneA/Crashes


### ASSISTANT
Лог сузил круг очень сильно. Смотри, что **исключено**:

- `Mono path[0]` + `mono-2.0-bdwgc.dll` → сборка на **Mono**, стриппинг IL2CPP ни при чём.
- `Direct3D 11.0` + `kGfxThreadingModeThreaded` → **D3D11, Graphics Jobs выключены**, видеоподсистема ни при чём.
- Ошибка возникает **сразу после `<RI> Input initialized.`**, до загрузки геометрии и до `Awake` любого нашего скрипта.

Остаётся одно: **движок не может прочитать файл сцены `level0`**. Это данные, а не код. Причём раз ошибка одинаковая на Mono/IL2CPP и D3D11/D3D12 — файл либо физически битый, либо в сцене есть объект, который ломает сериализацию при билде.

Дальше нужно **локализовать** это, а не перебирать настройки наугад.

---

## Шаг 1. Два замера (30 секунд, дай мне эти цифры)

1. Размер файла `C:\WorkProject\WarDroneB\TestBuild04\MyDroneA_Data\level0` — **в байтах/КБ**.
   - 0 КБ или подозрительно мало (десятки КБ при тяжёлой сцене) → файл **обрезан при записи**.
   - Сотни МБ → сцена упирается в лимиты.
2. Открой **Console ▸ ⋮ ▸ Open Editor Log** сразу после билда и найди поиском: `level0`, `error`, `Exception`, `serializ`. Часто настоящая причина пишется именно там, в логе редактора во время сборки, а не в логе плеера.

---

## Шаг 2. Лестница изоляции (это точно найдёт виновника)

Делай по порядку, каждый раз — **Clean Build в новую папку**.

### Тест A — пустая сцена
1. `File ▸ New Scene` (Empty), добавь только Camera и Directional Light, сохрани как `Assets/TestEmpty.unity`.
2. В Build Profiles оставь **только эту сцену**.
3. Build → запуск.

- **Падает так же** → проблема в окружении (Unity/антивирус/диск), переходи к шагу 3.
- **Запустилось** → проблема в содержимом сцены, переходи к тесту B.

### Тест B — чистая сцена дрона с нуля
1. `File ▸ New Scene` (Basic), сохрани как `Assets/DroneClean.unity`.
2. Создай пустой GameObject `Simulator`, повесь `SimBootstrap`.
3. `Tools ▸ Drone Sim ▸ Build Demo Scene` — наш генератор соберёт всё заново.
4. Ctrl+S, в Build Profiles только эта сцена, Build → запуск.

- **Запустилось** → старая сцена повреждена/содержит проблемный ассет. Город (`extrudeFront`, `FonBuildings`) — главный подозреваемый, переходи к шагу 4.
- **Падает** → проблема в наших объектах; удаляй по половине (сначала HUD + SimTelemetryPanel, потом DroneBreakup, потом город) и повторяй, пока не найдёшь компонент.

### Тест C — старая сцена без города
Открой свою рабочую сцену, `File ▸ Save As` → `Assets/DroneNoCity.unity`, удали весь фон/город/террейн, оставь дрон + землю + камеру + HUD. Build → запуск.

---

## Шаг 3. Окружение — если падает даже пустая сцена

В твоём логе в процесс внедрены **CryptoPro** (`cpcrypt.dll`, `cpadvai.dll`, `cpwinet.dll`) и **NVIDIA capture** (`nvspcap64.dll`). CryptoPro ставит глобальный хук через `AppInit_DLLs` и лезет в файловые операции — это реальный кандидат на порчу файла при записи билда.

1. **Отключи внедрение AppInit_DLLs** (нужен админ):
   `Win+R` → `regedit` →
   `HKEY_LOCAL_MACHINE\SOFTWARE\Microsoft\Windows NT\CurrentVersion\Windows`
   параметр **`LoadAppInit_DLLs` = 0**. Перезагрузка. (После теста можно вернуть 1.)
2. Отключи оверлей NVIDIA (GeForce Experience / NVIDIA App ▸ Overlay off).
3. Собери **на другой диск/папку**, например `D:\Builds\Test05` — исключит проблемы с правами и антивирусом на `C:\WorkProject`.
4. Проверь свободное место на диске (билд тяжёлой сцены может требовать десятки ГБ временных файлов).
5. Запусти exe **от имени администратора** один раз.
6. Если ничего — **другая версия Unity**: у тебя `6000.3.28f1`. Поставь рядом **6000.0 LTS** через Hub и собери тот же проект в ней. Если в LTS билд работает — это регрессия 6.3, и это уже конкретный баг-репорт.

---

## Шаг 4. Ремонт сцены — если виноват контент

1. **Пересохрани сцену под новым именем** (`File ▸ Save As`) — это перезаписывает YAML и лечит подпорченный файл сцены.
2. **Проверь missing-скрипты и огромные коллайдеры** — добавь в `Assets/Editor/` этот инструмент:

```csharp
#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class SceneHealthCheck
{
    [MenuItem("Tools/Drone Sim/Check Scene Health")]
    public static void Check()
    {
        int missing = 0, objects = 0, bigColliders = 0;
        for (int s = 0; s < SceneManager.sceneCount; s++)
        {
            var scene = SceneManager.GetSceneAt(s);
            foreach (var root in scene.GetRootGameObjects())
                Scan(root.transform, ref missing, ref objects, ref bigColliders);
        }
        Debug.Log($"[SceneCheck] объектов: {objects}, missing-скриптов: {missing}, гигантских MeshCollider: {bigColliders}");
    }

    static void Scan(Transform t, ref int missing, ref int objects, ref int big)
    {
        objects++;
        var comps = t.GetComponents<Component>();
        foreach (var c in comps)
            if (c == null) { missing++; Debug.LogWarning("[Missing script] " + Path(t), t.gameObject); }

        var mc = t.GetComponent<MeshCollider>();
        if (mc != null && mc.sharedMesh != null)
        {
            Vector3 size = Vector3.Scale(mc.sharedMesh.bounds.size, t.lossyScale);
            if (Mathf.Max(size.x, Mathf.Max(size.y, size.z)) > 500f)
            {
                big++;
                Debug.LogWarning($"[Огромный MeshCollider] {Path(t)}  размер {size}  вершин {mc.sharedMesh.vertexCount}", t.gameObject);
            }
        }
        foreach (Transform c in t) Scan(c, ref missing, ref objects, ref big);
    }

    static string Path(Transform t)
    {
        string p = t.name;
        while (t.parent != null) { t = t.parent; p = t.name + "/" + p; }
        return p;
    }
}
#endif
```

Запусти `Tools ▸ Drone Sim ▸ Check Scene Health` и пришли вывод.

3. Для фона (`extrudeFront`, `FonBuildings`): **удали MeshCollider**, у мешей в импорте выключи **Read/Write Enabled**, поставь **Mesh Compression = Medium**, у текстур **Max Size 2048**. Затем `Assets ▸ Reimport` на папку с городом.
4. Если сцена содержит **Terrain** с потерянным TerrainData — удали или переназначь данные.

---

## Что мне прислать, чтобы добить точно

1. **Размер `level0`** в байтах.
2. Результат **теста A** (пустая сцена) и **теста B** (чистая сцена дрона).
3. Строки из **Editor.log** сразу после билда, где встречается `level0` / `error` / `Exception`.
4. Вывод **Check Scene Health**.

По этим четырём пунктам проблема определяется однозначно: битая запись файла (окружение) или конкретный объект в сцене. Гадать дальше по логу плеера бессмысленно — он показывает только следствие («не могу прочитать сцену»), а причина пишется на этапе сборки.

### USER
Провел тесты по инструции результаты.
Тест A — пустая сцена с камерой: бидится и запускается прекрасно все работает
Тест B — чистая сцена дрона с нуля добавил галочки  Development Build + Script Debugging и билд падает , вот получившийся лог:
Mono path[0] = 'C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/Managed'
Mono config path = 'C:/WorkProject/WarDroneB/TestBuild07/MonoBleedingEdge/etc'
Starting managed debugger on port 56967
Using monoOptions --debugger-agent=transport=dt_socket,embedding=1,server=y,suspend=n,address=0.0.0.0:56967
Found 3 interfaces on host : 0) 192.168.0.103 1) 172.24.228.220 2) 10.6.7.1
Player connection [29204]  Target information:

Player connection [29204]  * "[IP] 192.168.0.103 [Port] 55000 [Flags] 2 [Guid] 3787070967 [EditorId] 1337524287 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 1 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [29204]  * "[IP] 172.24.228.220 [Port] 55000 [Flags] 2 [Guid] 3787070967 [EditorId] 1337524287 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 1 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [29204]  * "[IP] 10.6.7.1 [Port] 55000 [Flags] 2 [Guid] 3787070967 [EditorId] 1337524287 [Version] 1048832 [Id] WindowsPlayer(2,Home-PC) [Debug] 1 [PackageName] WindowsPlayer [ProjectName] MyDroneA" 

Player connection [29204] Started UDP target info broadcast (1) on [225.0.0.222:54997].

Input System module state changed to: Initialized.
[Physics::Module] Initialized fallback backend.
[Physics::Module] Id: 0xdecafbad
Initialize engine version: 6000.3.28f1 (8f082d2fe476)
[Subsystems] Discovering subsystems at path C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/UnitySubsystems
GfxDevice: creating device client; kGfxThreadingModeThreaded
Direct3D:
    Version:  Direct3D 11.0 [level 11.1]
    Renderer: NVIDIA GeForce RTX 3090 (ID=0x2204)
    Vendor:   NVIDIA
    VRAM:     24326 MB
    Driver:   32.0.15.8108
Begin MonoManager ReloadAssembly
- Loaded All Assemblies, in  0.206 seconds
- Finished resetting the current domain, in  0.005 seconds
[Physics::Module] Selected backend.
[Physics::Module] Name: PhysX
[Physics::Module] Id: 0xf2b8ea05
[Physics::Module] SDK Version: 4.1.2
[Physics::Module] Integration Version: 1.0.0
[Physics::Module] Threading Mode: Multi-Threaded
<RI> Initializing input.

Using Windows.Gaming.Input
<RI> Input initialized.

The file 'C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/level0' is corrupted! Remove it and launch unity again!
[Position out of bounds!]
Crash!!!
SymInit: Symbol-SearchPath: '.;C:\WorkProject\WarDroneB\TestBuild07;C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/Plugins\x86_64;C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/Plugins;C:\WorkProject\WarDroneB\TestBuild07;C:\Windows;C:\Windows\system32;', symOptions: 534, UserName: 'KirikDK'
OS-Version: 10.0.0
C:\WorkProject\WarDroneB\TestBuild07\MyDroneA.exe:MyDroneA.exe (00007FF70A200000), size: 679936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 6000.3.28.2093
C:\Windows\SYSTEM32\ntdll.dll:ntdll.dll (00007FF867030000), size: 2064384 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\KERNEL32.DLL:KERNEL32.DLL (00007FF865080000), size: 774144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1889
C:\Windows\System32\KERNELBASE.dll:KERNELBASE.dll (00007FF864860000), size: 2957312 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\WorkProject\WarDroneB\TestBuild07\UnityPlayer.dll:UnityPlayer.dll (00007FFF8BB50000), size: 92422144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 6000.3.28.2093
C:\Windows\System32\USER32.dll:USER32.dll (00007FF8657A0000), size: 1691648 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\win32u.dll:win32u.dll (00007FF864FA0000), size: 139264 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\GDI32.dll:GDI32.dll (00007FF8652C0000), size: 176128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\gdi32full.dll:gdi32full.dll (00007FF864D90000), size: 1110016 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\msvcp_win.dll:msvcp_win.dll (00007FF864730000), size: 643072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.789
C:\Windows\System32\ucrtbase.dll:ucrtbase.dll (00007FF864EA0000), size: 1048576 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.789
C:\Windows\System32\ole32.dll:ole32.dll (00007FF865B80000), size: 1220608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1202
C:\Windows\System32\RPCRT4.dll:RPCRT4.dll (00007FF865410000), size: 1200128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1806
C:\Windows\System32\combase.dll:combase.dll (00007FF865E20000), size: 3489792 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\VERSION.dll:VERSION.dll (00007FF85E580000), size: 40960 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\SHLWAPI.dll:SHLWAPI.dll (00007FF865B20000), size: 348160 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\System32\msvcrt.dll:msvcrt.dll (00007FF865140000), size: 647168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.0.19041.546
C:\Windows\System32\SETUPAPI.dll:SETUPAPI.dll (00007FF866230000), size: 4620288 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\cfgmgr32.dll:cfgmgr32.dll (00007FF864BB0000), size: 319488 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\bcrypt.dll:bcrypt.dll (00007FF864D60000), size: 159744 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1023
C:\Windows\System32\ADVAPI32.dll:ADVAPI32.dll (00007FF866180000), size: 712704 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\sechost.dll:sechost.dll (00007FF865220000), size: 638976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\SHELL32.dll:SHELL32.dll (00007FF866820000), size: 7614464 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\System32\OLEAUT32.dll:OLEAUT32.dll (00007FF866750000), size: 839680 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.985
C:\Windows\System32\IMM32.dll:IMM32.dll (00007FF8651E0000), size: 204800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\OPENGL32.dll:OPENGL32.dll (00007FF8174C0000), size: 1200128 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\WINMM.dll:WINMM.dll (00007FF8552A0000), size: 159744 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\CRYPT32.dll:CRYPT32.dll (00007FF864C00000), size: 1400832 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1889
C:\Windows\System32\WS2_32.dll:WS2_32.dll (00007FF865730000), size: 438272 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\IPHLPAPI.DLL:IPHLPAPI.DLL (00007FF863B30000), size: 241664 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\HID.DLL:HID.DLL (00007FF862AB0000), size: 53248 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\WINHTTP.dll:WINHTTP.dll (00007FF85D2A0000), size: 1097728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\d3d11.dll:d3d11.dll (00007FF8627F0000), size: 2502656 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dxgi.dll:dxgi.dll (00007FF862B10000), size: 995328 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\UIAutomationCore.DLL:UIAutomationCore.DLL (00007FF8113E0000), size: 3100672 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.2.19041.1741
C:\Windows\SYSTEM32\GLU32.dll:GLU32.dll (00007FF818AB0000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\Secur32.dll:Secur32.dll (00007FF84A270000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dwmapi.dll:dwmapi.dll (00007FF861040000), size: 192512 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\PROPSYS.dll:PROPSYS.dll (00007FF85E9F0000), size: 1007616 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 7.0.19041.1741
C:\Windows\SYSTEM32\SSPICLI.DLL:SSPICLI.DLL (00007FF864620000), size: 204800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll:cpcrypt.dll (00007FF864200000), size: 135168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.16728.0
C:\Windows\system32\apphelp.dll:apphelp.dll (00007FF860C70000), size: 593920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\kernel.appcore.dll:kernel.appcore.dll (00007FF862AC0000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\bcryptPrimitives.dll:bcryptPrimitives.dll (00007FF8647D0000), size: 532480 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1415
C:\Windows\system32\uxtheme.dll:uxtheme.dll (00007FF860D70000), size: 647168 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\shcore.dll:shcore.dll (00007FF865D70000), size: 708608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\clbcatq.dll:clbcatq.dll (00007FF8666A0000), size: 716800 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 2001.12.10941.16384
C:\Windows\system32\wbem\wbemprox.dll:wbemprox.dll (00007FF84B4D0000), size: 69632 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1320
C:\Windows\SYSTEM32\wbemcomn.dll:wbemcomn.dll (00007FF84B0D0000), size: 589824 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\system32\wbem\wbemsvc.dll:wbemsvc.dll (00007FF84A350000), size: 81920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1320
C:\Windows\system32\wbem\fastprox.dll:fastprox.dll (00007FF84A440000), size: 1093632 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\amsi.dll:amsi.dll (00007FF849910000), size: 126976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\WorkProject\WarDroneB\TestBuild07\WinPixEventRuntime.dll:WinPixEventRuntime.dll (00007FF848B60000), size: 57344 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 1.0.2403.8001
C:\Windows\SYSTEM32\MSVCP140.dll:MSVCP140.dll (00007FF847190000), size: 643072 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140_1.dll:VCRUNTIME140_1.dll (00007FF847260000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140.dll:VCRUNTIME140.dll (00007FF847230000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\windows.storage.dll:windows.storage.dll (00007FF861510000), size: 7933952 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\Wldp.dll:Wldp.dll (00007FF864160000), size: 196608 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\profapi.dll:profapi.dll (00007FF864670000), size: 126976 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.844
C:\WorkProject\WarDroneB\TestBuild07\MonoBleedingEdge\EmbedRuntime\mono-2.0-bdwgc.dll:mono-2.0-bdwgc.dll (00007FFF8B100000), size: 10756096 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\system32\mswsock.dll:mswsock.dll (00007FF863E90000), size: 434176 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\NSI.dll:NSI.dll (00007FF865960000), size: 32768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.610
C:\Windows\SYSTEM32\dhcpcsvc6.DLL:dhcpcsvc6.DLL (00007FF85E590000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dhcpcsvc.DLL:dhcpcsvc.DLL (00007FF85E4F0000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\DNSAPI.dll:DNSAPI.dll (00007FF863B70000), size: 827392 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\system32\napinsp.dll:napinsp.dll (00007FF83A9E0000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\pnrpnsp.dll:pnrpnsp.dll (00007FF83A9C0000), size: 110592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\wshbth.dll:wshbth.dll (00007FF83A9A0000), size: 86016 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\system32\NLAapi.dll:NLAapi.dll (00007FF85FA30000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\winrnr.dll:winrnr.dll (00007FF83A980000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Program Files\Bonjour\mdnsNSP.dll:mdnsNSP.dll (00000000632E0000), size: 155648 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 3.0.0.10
C:\Windows\System32\MSCTF.dll:MSCTF.dll (00007FF8652F0000), size: 1134592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll:nvldumdx.dll (00007FF85AFD0000), size: 802816 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\msasn1.dll:msasn1.dll (00007FF864320000), size: 73728 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\cryptnet.dll:cryptnet.dll (00007FF85E540000), size: 200704 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.906
C:\Windows\SYSTEM32\cryptbase.dll:cryptbase.dll (00007FF8640D0000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\drvstore.dll:drvstore.dll (00007FF85DEF0000), size: 1343488 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1949
C:\Windows\SYSTEM32\devobj.dll:devobj.dll (00007FF8644E0000), size: 180224 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\wintrust.dll:wintrust.dll (000001F74CE90000), size: 434176 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\imagehlp.dll:imagehlp.dll (00007FF865940000), size: 118784 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1415
C:\Windows\SYSTEM32\CRYPTSP.dll:CRYPTSP.dll (00007FF8640B0000), size: 98304 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll:cpadvai.dll (00007FF864090000), size: 94208 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.16639.0
C:\Program Files\Crypto Pro\CSP\cpcsp.dll:cpcsp.dll (00007FF8634A0000), size: 32768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.14960.0
C:\Program Files\Crypto Pro\CSP\cpcspi.dll:cpcspi.dll (00007FF863280000), size: 2187264 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.18680.0
C:\Program Files\Crypto Pro\CSP\cpsuprt.dll:cpsuprt.dll (00007FF8631E0000), size: 634880 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.15962.0
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll:nvgpucomp64.dll (00007FF8558C0000), size: 64565248 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll:NvMemMapStoragex.dll (00007FF8552D0000), size: 909312 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\SYSTEM32\POWRPROF.dll:POWRPROF.dll (00007FF863C50000), size: 307200 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\UMPDC.dll:UMPDC.dll (00007FF863B10000), size: 73728 (result: 0), SymType: '-deferred-', PDB: ''
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll:nvwgf2umx.dll (00007FF84DF90000), size: 92487680 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\system32\nvspcap64.dll:nvspcap64.dll (00007FF81A2E0000), size: 1339392 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 11.0.5.420
C:\Windows\SYSTEM32\ntmarta.dll:ntmarta.dll (00007FF863860000), size: 208896 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dxcore.dll:dxcore.dll (00007FF85B520000), size: 241664 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll:nvppex.dll (00007FFFF4690000), size: 1798144 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 32.0.15.8108
C:\Windows\System32\MMDevApi.dll:MMDevApi.dll (00007FF85D150000), size: 544768 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1503
C:\Windows\SYSTEM32\AUDIOSES.DLL:AUDIOSES.DLL (00007FF8220A0000), size: 1581056 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\resourcepolicyclient.dll:resourcepolicyclient.dll (00007FF860F40000), size: 81920 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.Input.dll:Windows.Gaming.Input.dll (00007FF85C990000), size: 864256 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\SYSTEM32\inputhost.dll:inputhost.dll (00007FF85B170000), size: 1384448 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1741
C:\Windows\SYSTEM32\wintypes.dll:wintypes.dll (00007FF85FBA0000), size: 1392640 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2130
C:\Windows\System32\CoreMessaging.dll:CoreMessaging.dll (00007FF860A40000), size: 991232 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2193
C:\Windows\System32\CoreUIComponents.dll:CoreUIComponents.dll (00007FF860360000), size: 3530752 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.546
C:\Windows\System32\twinapi.appcore.dll:twinapi.appcore.dll (00007FF85DC80000), size: 2097152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1865
C:\Windows\System32\Windows.Gaming.UI.GameBar.dll:Windows.Gaming.UI.GameBar.dll (00007FFF963A0000), size: 110592 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.746
C:\Windows\System32\WININET.dll:WININET.dll (00007FF83A250000), size: 5083136 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 11.0.19041.2193
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll:cpwinet.dll (00007FF8388E0000), size: 49152 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 5.0.15621.0
C:\Windows\System32\appresolver.dll:appresolver.dll (00007FF8204B0000), size: 589824 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1620
C:\Windows\System32\SLC.dll:SLC.dll (00007FF862D90000), size: 167936 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1682
C:\Windows\System32\Bcp47Langs.dll:Bcp47Langs.dll (00007FF85B660000), size: 372736 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1566
C:\Windows\System32\USERENV.dll:USERENV.dll (00007FF8645F0000), size: 188416 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.572
C:\Windows\System32\sppc.dll:sppc.dll (00007FF862D60000), size: 151552 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.1682
C:\Windows\SYSTEM32\xinput1_3.dll:xinput1_3.dll (0000000000400000), size: 122880 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 9.18.944.0
C:\Windows\SYSTEM32\dcomp.dll:dcomp.dll (00007FF860170000), size: 1978368 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\textinputframework.dll:textinputframework.dll (00007FF85B2D0000), size: 1019904 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dbghelp.dll:dbghelp.dll (00007FF862600000), size: 1982464 (result: 0), SymType: '-deferred-', PDB: '', fileVersion: 10.0.19041.867

========== OUTPUTTING STACK TRACE ==================

0x00007FFF8DDB573D (UnityPlayer) UnityMain
0x00007FFF8DDB4B59 (UnityPlayer) UnityMain
SymInit: Symbol-SearchPath: '.;C:\WorkProject\WarDroneB\TestBuild07;C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/Plugins\x86_64;C:/WorkProject/WarDroneB/TestBuild07/MyDroneA_Data/Plugins;C:\WorkProject\WarDroneB\TestBuild07;C:\Windows;C:\Windows\system32;', symOptions: 534, UserName: 'KirikDK'
OS-Version: 10.0.0
C:\WorkProject\WarDroneB\TestBuild07\MyDroneA.exe:MyDroneA.exe (00007FF70A200000), size: 679936 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild07\MyDroneA.exe', fileVersion: 6000.3.28.2093
C:\Windows\SYSTEM32\ntdll.dll:ntdll.dll (00007FF867030000), size: 2064384 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\ntdll.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\KERNEL32.DLL:KERNEL32.DLL (00007FF865080000), size: 774144 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\KERNEL32.DLL', fileVersion: 10.0.19041.1889
C:\Windows\System32\KERNELBASE.dll:KERNELBASE.dll (00007FF864860000), size: 2957312 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\KERNELBASE.dll', fileVersion: 10.0.19041.2193
C:\WorkProject\WarDroneB\TestBuild07\UnityPlayer.dll:UnityPlayer.dll (00007FFF8BB50000), size: 92422144 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild07\UnityPlayer.dll', fileVersion: 6000.3.28.2093
C:\Windows\System32\USER32.dll:USER32.dll (00007FF8657A0000), size: 1691648 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\USER32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\win32u.dll:win32u.dll (00007FF864FA0000), size: 139264 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\win32u.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\GDI32.dll:GDI32.dll (00007FF8652C0000), size: 176128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\GDI32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\gdi32full.dll:gdi32full.dll (00007FF864D90000), size: 1110016 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\gdi32full.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\msvcp_win.dll:msvcp_win.dll (00007FF864730000), size: 643072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\msvcp_win.dll', fileVersion: 10.0.19041.789
C:\Windows\System32\ucrtbase.dll:ucrtbase.dll (00007FF864EA0000), size: 1048576 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ucrtbase.dll', fileVersion: 10.0.19041.789
C:\Windows\System32\ole32.dll:ole32.dll (00007FF865B80000), size: 1220608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ole32.dll', fileVersion: 10.0.19041.1202
C:\Windows\System32\RPCRT4.dll:RPCRT4.dll (00007FF865410000), size: 1200128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\RPCRT4.dll', fileVersion: 10.0.19041.1806
C:\Windows\System32\combase.dll:combase.dll (00007FF865E20000), size: 3489792 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\combase.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\VERSION.dll:VERSION.dll (00007FF85E580000), size: 40960 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VERSION.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\SHLWAPI.dll:SHLWAPI.dll (00007FF865B20000), size: 348160 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SHLWAPI.dll', fileVersion: 10.0.19041.2075
C:\Windows\System32\msvcrt.dll:msvcrt.dll (00007FF865140000), size: 647168 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\msvcrt.dll', fileVersion: 7.0.19041.546
C:\Windows\System32\SETUPAPI.dll:SETUPAPI.dll (00007FF866230000), size: 4620288 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SETUPAPI.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\cfgmgr32.dll:cfgmgr32.dll (00007FF864BB0000), size: 319488 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\cfgmgr32.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\bcrypt.dll:bcrypt.dll (00007FF864D60000), size: 159744 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\bcrypt.dll', fileVersion: 10.0.19041.1023
C:\Windows\System32\ADVAPI32.dll:ADVAPI32.dll (00007FF866180000), size: 712704 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\ADVAPI32.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\sechost.dll:sechost.dll (00007FF865220000), size: 638976 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\sechost.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\SHELL32.dll:SHELL32.dll (00007FF866820000), size: 7614464 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SHELL32.dll', fileVersion: 10.0.19041.2075
C:\Windows\System32\OLEAUT32.dll:OLEAUT32.dll (00007FF866750000), size: 839680 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\OLEAUT32.dll', fileVersion: 10.0.19041.985
C:\Windows\System32\IMM32.dll:IMM32.dll (00007FF8651E0000), size: 204800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\IMM32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\OPENGL32.dll:OPENGL32.dll (00007FF8174C0000), size: 1200128 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\OPENGL32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\WINMM.dll:WINMM.dll (00007FF8552A0000), size: 159744 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\WINMM.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\CRYPT32.dll:CRYPT32.dll (00007FF864C00000), size: 1400832 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CRYPT32.dll', fileVersion: 10.0.19041.1889
C:\Windows\System32\WS2_32.dll:WS2_32.dll (00007FF865730000), size: 438272 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\WS2_32.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\IPHLPAPI.DLL:IPHLPAPI.DLL (00007FF863B30000), size: 241664 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\IPHLPAPI.DLL', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\HID.DLL:HID.DLL (00007FF862AB0000), size: 53248 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\HID.DLL', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\WINHTTP.dll:WINHTTP.dll (00007FF85D2A0000), size: 1097728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\WINHTTP.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\d3d11.dll:d3d11.dll (00007FF8627F0000), size: 2502656 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\d3d11.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dxgi.dll:dxgi.dll (00007FF862B10000), size: 995328 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxgi.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\UIAutomationCore.DLL:UIAutomationCore.DLL (00007FF8113E0000), size: 3100672 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\UIAutomationCore.DLL', fileVersion: 7.2.19041.1741
C:\Windows\SYSTEM32\GLU32.dll:GLU32.dll (00007FF818AB0000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\GLU32.dll', fileVersion: 10.0.19041.2193
C:\Windows\SYSTEM32\Secur32.dll:Secur32.dll (00007FF84A270000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Secur32.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dwmapi.dll:dwmapi.dll (00007FF861040000), size: 192512 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dwmapi.dll', fileVersion: 10.0.19041.746
C:\Windows\SYSTEM32\PROPSYS.dll:PROPSYS.dll (00007FF85E9F0000), size: 1007616 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\PROPSYS.dll', fileVersion: 7.0.19041.1741
C:\Windows\SYSTEM32\SSPICLI.DLL:SSPICLI.DLL (00007FF864620000), size: 204800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\SSPICLI.DLL', fileVersion: 10.0.19041.2130
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll:cpcrypt.dll (00007FF864200000), size: 135168 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpcrypt.dll', fileVersion: 5.0.16728.0
C:\Windows\system32\apphelp.dll:apphelp.dll (00007FF860C70000), size: 593920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\apphelp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\kernel.appcore.dll:kernel.appcore.dll (00007FF862AC0000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\kernel.appcore.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\bcryptPrimitives.dll:bcryptPrimitives.dll (00007FF8647D0000), size: 532480 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\bcryptPrimitives.dll', fileVersion: 10.0.19041.1415
C:\Windows\system32\uxtheme.dll:uxtheme.dll (00007FF860D70000), size: 647168 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\uxtheme.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\shcore.dll:shcore.dll (00007FF865D70000), size: 708608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\shcore.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\clbcatq.dll:clbcatq.dll (00007FF8666A0000), size: 716800 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\clbcatq.dll', fileVersion: 2001.12.10941.16384
C:\Windows\system32\wbem\wbemprox.dll:wbemprox.dll (00007FF84B4D0000), size: 69632 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\wbemprox.dll', fileVersion: 10.0.19041.1320
C:\Windows\SYSTEM32\wbemcomn.dll:wbemcomn.dll (00007FF84B0D0000), size: 589824 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\wbemcomn.dll', fileVersion: 10.0.19041.1566
C:\Windows\system32\wbem\wbemsvc.dll:wbemsvc.dll (00007FF84A350000), size: 81920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\wbemsvc.dll', fileVersion: 10.0.19041.1320
C:\Windows\system32\wbem\fastprox.dll:fastprox.dll (00007FF84A440000), size: 1093632 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wbem\fastprox.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\amsi.dll:amsi.dll (00007FF849910000), size: 126976 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\amsi.dll', fileVersion: 10.0.19041.2075
C:\WorkProject\WarDroneB\TestBuild07\WinPixEventRuntime.dll:WinPixEventRuntime.dll (00007FF848B60000), size: 57344 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild07\WinPixEventRuntime.dll', fileVersion: 1.0.2403.8001
C:\Windows\SYSTEM32\MSVCP140.dll:MSVCP140.dll (00007FF847190000), size: 643072 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\MSVCP140.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140_1.dll:VCRUNTIME140_1.dll (00007FF847260000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VCRUNTIME140_1.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\VCRUNTIME140.dll:VCRUNTIME140.dll (00007FF847230000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\VCRUNTIME140.dll', fileVersion: 14.51.36247.0
C:\Windows\SYSTEM32\windows.storage.dll:windows.storage.dll (00007FF861510000), size: 7933952 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\windows.storage.dll', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\Wldp.dll:Wldp.dll (00007FF864160000), size: 196608 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\Wldp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\profapi.dll:profapi.dll (00007FF864670000), size: 126976 (result: 0), SymType: '-nosymbols-', PDB: 'C:\Windows\SYSTEM32\profapi.dll', fileVersion: 10.0.19041.844
C:\WorkProject\WarDroneB\TestBuild07\MonoBleedingEdge\EmbedRuntime\mono-2.0-bdwgc.dll:mono-2.0-bdwgc.dll (00007FFF8B100000), size: 10756096 (result: 0), SymType: '-exported-', PDB: 'C:\WorkProject\WarDroneB\TestBuild07\MonoBleedingEdge\EmbedRuntime\mono-2.0-bdwgc.dll'
C:\Windows\system32\mswsock.dll:mswsock.dll (00007FF863E90000), size: 434176 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\mswsock.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\NSI.dll:NSI.dll (00007FF865960000), size: 32768 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\NSI.dll', fileVersion: 10.0.19041.610
C:\Windows\SYSTEM32\dhcpcsvc6.DLL:dhcpcsvc6.DLL (00007FF85E590000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dhcpcsvc6.DLL', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\dhcpcsvc.DLL:dhcpcsvc.DLL (00007FF85E4F0000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dhcpcsvc.DLL', fileVersion: 10.0.19041.2130
C:\Windows\SYSTEM32\DNSAPI.dll:DNSAPI.dll (00007FF863B70000), size: 827392 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\DNSAPI.dll', fileVersion: 10.0.19041.1865
C:\Windows\system32\napinsp.dll:napinsp.dll (00007FF83A9E0000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\napinsp.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\pnrpnsp.dll:pnrpnsp.dll (00007FF83A9C0000), size: 110592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\pnrpnsp.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\wshbth.dll:wshbth.dll (00007FF83A9A0000), size: 86016 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\wshbth.dll', fileVersion: 10.0.19041.546
C:\Windows\system32\NLAapi.dll:NLAapi.dll (00007FF85FA30000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\NLAapi.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\winrnr.dll:winrnr.dll (00007FF83A980000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\winrnr.dll', fileVersion: 10.0.19041.546
C:\Program Files\Bonjour\mdnsNSP.dll:mdnsNSP.dll (00000000632E0000), size: 155648 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Bonjour\mdnsNSP.dll', fileVersion: 3.0.0.10
C:\Windows\System32\MSCTF.dll:MSCTF.dll (00007FF8652F0000), size: 1134592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\MSCTF.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll:nvldumdx.dll (00007FF85AFD0000), size: 802816 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvldumdx.dll', fileVersion: 32.0.15.8108
C:\Windows\SYSTEM32\msasn1.dll:msasn1.dll (00007FF864320000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\msasn1.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\cryptnet.dll:cryptnet.dll (00007FF85E540000), size: 200704 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\cryptnet.dll', fileVersion: 10.0.19041.906
C:\Windows\SYSTEM32\cryptbase.dll:cryptbase.dll (00007FF8640D0000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\cryptbase.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\drvstore.dll:drvstore.dll (00007FF85DEF0000), size: 1343488 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\drvstore.dll', fileVersion: 10.0.19041.1949
C:\Windows\SYSTEM32\devobj.dll:devobj.dll (00007FF8644E0000), size: 180224 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\devobj.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\wintrust.dll:wintrust.dll (000001F74CE90000), size: 434176 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\wintrust.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\imagehlp.dll:imagehlp.dll (00007FF865940000), size: 118784 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\imagehlp.dll', fileVersion: 10.0.19041.1415
C:\Windows\SYSTEM32\CRYPTSP.dll:CRYPTSP.dll (00007FF8640B0000), size: 98304 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\CRYPTSP.dll', fileVersion: 10.0.19041.546
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll:cpadvai.dll (00007FF864090000), size: 94208 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpadvai.dll', fileVersion: 5.0.16639.0
C:\Program Files\Crypto Pro\CSP\cpcsp.dll:cpcsp.dll (00007FF8634A0000), size: 32768 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpcsp.dll', fileVersion: 5.0.14960.0
C:\Program Files\Crypto Pro\CSP\cpcspi.dll:cpcspi.dll (00007FF863280000), size: 2187264 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpcspi.dll', fileVersion: 5.0.18680.0
C:\Program Files\Crypto Pro\CSP\cpsuprt.dll:cpsuprt.dll (00007FF8631E0000), size: 634880 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Crypto Pro\CSP\cpsuprt.dll', fileVersion: 5.0.15962.0
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll:nvgpucomp64.dll (00007FF8558C0000), size: 64565248 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvgpucomp64.dll', fileVersion: 32.0.15.8108
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll:NvMemMapStoragex.dll (00007FF8552D0000), size: 909312 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\NvMemMapStoragex.dll'
C:\Windows\SYSTEM32\POWRPROF.dll:POWRPROF.dll (00007FF863C50000), size: 307200 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\POWRPROF.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\UMPDC.dll:UMPDC.dll (00007FF863B10000), size: 73728 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\UMPDC.dll'
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll:nvwgf2umx.dll (00007FF84DF90000), size: 92487680 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvwgf2umx.dll', fileVersion: 32.0.15.8108
C:\Windows\system32\nvspcap64.dll:nvspcap64.dll (00007FF81A2E0000), size: 1339392 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\system32\nvspcap64.dll', fileVersion: 11.0.5.420
C:\Windows\SYSTEM32\ntmarta.dll:ntmarta.dll (00007FF863860000), size: 208896 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\ntmarta.dll', fileVersion: 10.0.19041.546
C:\Windows\SYSTEM32\dxcore.dll:dxcore.dll (00007FF85B520000), size: 241664 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dxcore.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll:nvppex.dll (00007FFFF4690000), size: 1798144 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\DriverStore\FileRepository\nv_dispi.inf_amd64_7621da800da0efae\nvppex.dll', fileVersion: 32.0.15.8108
C:\Windows\System32\MMDevApi.dll:MMDevApi.dll (00007FF85D150000), size: 544768 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\MMDevApi.dll', fileVersion: 10.0.19041.1503
C:\Windows\SYSTEM32\AUDIOSES.DLL:AUDIOSES.DLL (00007FF8220A0000), size: 1581056 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\AUDIOSES.DLL', fileVersion: 10.0.19041.1865
C:\Windows\SYSTEM32\resourcepolicyclient.dll:resourcepolicyclient.dll (00007FF860F40000), size: 81920 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\resourcepolicyclient.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\Windows.Gaming.Input.dll:Windows.Gaming.Input.dll (00007FF85C990000), size: 864256 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Windows.Gaming.Input.dll', fileVersion: 10.0.19041.1566
C:\Windows\SYSTEM32\inputhost.dll:inputhost.dll (00007FF85B170000), size: 1384448 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\inputhost.dll', fileVersion: 10.0.19041.1741
C:\Windows\SYSTEM32\wintypes.dll:wintypes.dll (00007FF85FBA0000), size: 1392640 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\wintypes.dll', fileVersion: 10.0.19041.2130
C:\Windows\System32\CoreMessaging.dll:CoreMessaging.dll (00007FF860A40000), size: 991232 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CoreMessaging.dll', fileVersion: 10.0.19041.2193
C:\Windows\System32\CoreUIComponents.dll:CoreUIComponents.dll (00007FF860360000), size: 3530752 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\CoreUIComponents.dll', fileVersion: 10.0.19041.546
C:\Windows\System32\twinapi.appcore.dll:twinapi.appcore.dll (00007FF85DC80000), size: 2097152 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\twinapi.appcore.dll', fileVersion: 10.0.19041.1865
C:\Windows\System32\Windows.Gaming.UI.GameBar.dll:Windows.Gaming.UI.GameBar.dll (00007FFF963A0000), size: 110592 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Windows.Gaming.UI.GameBar.dll', fileVersion: 10.0.19041.746
C:\Windows\System32\WININET.dll:WININET.dll (00007FF83A250000), size: 5083136 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\WININET.dll', fileVersion: 11.0.19041.2193
C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll:cpwinet.dll (00007FF8388E0000), size: 49152 (result: 0), SymType: '-exported-', PDB: 'C:\Program Files\Common Files\Crypto Pro\AppCompat\cpwinet.dll', fileVersion: 5.0.15621.0
C:\Windows\System32\appresolver.dll:appresolver.dll (00007FF8204B0000), size: 589824 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\appresolver.dll', fileVersion: 10.0.19041.1620
C:\Windows\System32\SLC.dll:SLC.dll (00007FF862D90000), size: 167936 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\SLC.dll', fileVersion: 10.0.19041.1682
C:\Windows\System32\Bcp47Langs.dll:Bcp47Langs.dll (00007FF85B660000), size: 372736 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\Bcp47Langs.dll', fileVersion: 10.0.19041.1566
C:\Windows\System32\USERENV.dll:USERENV.dll (00007FF8645F0000), size: 188416 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\USERENV.dll', fileVersion: 10.0.19041.572
C:\Windows\System32\sppc.dll:sppc.dll (00007FF862D60000), size: 151552 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\System32\sppc.dll', fileVersion: 10.0.19041.1682
C:\Windows\SYSTEM32\xinput1_3.dll:xinput1_3.dll (0000000000400000), size: 122880 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\xinput1_3.dll', fileVersion: 9.18.944.0
C:\Windows\SYSTEM32\dcomp.dll:dcomp.dll (00007FF860170000), size: 1978368 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dcomp.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\textinputframework.dll:textinputframework.dll (00007FF85B2D0000), size: 1019904 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\textinputframework.dll', fileVersion: 10.0.19041.2075
C:\Windows\SYSTEM32\dbghelp.dll:dbghelp.dll (00007FF862600000), size: 1982464 (result: 0), SymType: '-exported-', PDB: 'C:\Windows\SYSTEM32\dbghelp.dll', fileVersion: 10.0.19041.867
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8C7DA6EF)
0x00007FFF8C7DA6EF (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8C7DD615)
0x00007FFF8C7DD615 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8BFD515A)
0x00007FFF8BFD515A (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC4BED7)
0x00007FFF8CC4BED7 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC5F37E)
0x00007FFF8CC5F37E (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC3D558)
0x00007FFF8CC3D558 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC51B7F)
0x00007FFF8CC51B7F (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC51C12)
0x00007FFF8CC51C12 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC73304)
0x00007FFF8CC73304 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC76007)
0x00007FFF8CC76007 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC11356)
0x00007FFF8CC11356 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CCA2D42)
0x00007FFF8CCA2D42 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CCA0976)
0x00007FFF8CCA0976 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8CC9D5A8)
0x00007FFF8CC9D5A8 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8C73AEE3)
0x00007FFF8C73AEE3 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8C735E06)
0x00007FFF8C735E06 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8C7363E1)
0x00007FFF8C7363E1 (UnityPlayer) (function-name not available)
  ERROR: SymGetSymFromAddr64, GetLastError: 'Попытка обращения к неверному адресу.' (Address: 00007FFF8C73645A)
0x00007FFF8C73645A (UnityPlayer) (function-name not available)
0x00007FFF8DA76B9F (UnityPlayer) UnityMain
0x00007FF865097034 (KERNEL32) BaseThreadInitThunk
0x00007FF8670826A1 (ntdll) RtlUserThreadStart

========== END OF STACKTRACE ===========

A crash has been intercepted by the crash handler. For call stack and other details, see the latest crash report generated in:
 * C:/Users/KirikDK/AppData/Local/Temp/WarDrone/MyDroneA/Crashes





### ASSISTANT
Отличные тесты — они закрыли почти всё. Раз **пустая сцена билдится и запускается**, а **чистая сцена дрона падает с той же ошибкой**, то:

- окружение (антивирус, диск, CryptoPro, D3D, IL2CPP) — **ни при чём**;
- город — **ни при чём**;
- виновник — **что-то из нашего набора компонентов**, что ломает сериализацию сцены при билде.

И у меня есть конкретный подозреваемый.

---

## Главный подозреваемый: `DronePresentation.cs`

В этом файле лежат **три MonoBehaviour сразу**:

```csharp
public class ChaseCamera : MonoBehaviour { ... }
public class PropVisual   : MonoBehaviour { ... }
public class MotorSound   : MonoBehaviour { ... }
```

**Имя файла не совпадает ни с одним из классов.** Это нарушение базового правила Unity: класс MonoBehaviour должен лежать в файле с точно таким же именем. В редакторе Unity разруливает типы рефлексией, поэтому всё работает — а при билде ссылка `m_Script` на такой класс пишется некорректно, и плеер при чтении сцены вылетает с `Position out of bounds!`.

Это моя ошибка при компоновке кода — я сложил три класса в один файл ради компактности. Разносим.

### 1. Создай `Assets/Scripts/ChaseCamera.cs`

```csharp
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
```

### 2. Создай `Assets/Scripts/PropVisual.cs`

```csharp
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
```

### 3. Создай `Assets/Scripts/MotorSound.cs`

```csharp
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
```

### 4. Удали `Assets/Scripts/DronePresentation.cs` (через окно Project, вместе с .meta)

### 5. Пересоздай тестовую сцену

Старые ссылки на скрипты в уже сохранённой сцене после разделения отвалятся, поэтому:
новая пустая сцена → `Simulator` + `SimBootstrap` → `Tools ▸ Drone Sim ▸ Build Demo Scene` → Ctrl+S → Clean Build в новую папку.

---

## Заодно — мелкая страховка в `UdpLink.cs`

Публичное поле-делегат Unity сериализовать не умеет; пометим его явно, чтобы исключить любые сюрпризы:

```csharp
    [System.NonSerialized] public System.Func<string> TelemetryJsonProvider;
```

(замени строку `public Func<string> TelemetryJsonProvider;`)

---

## Если после этого всё ещё падает — бисекция за 2–3 билда

Сцена маленькая, делить быстро. Каждый раз Clean Build в новую папку.

**B1.** В чистой сцене дрона удали целиком объект **Quadcopter** (останутся Environment, SimCamera, HUD, Simulator). Build.
- **Запустилось** → виновник среди компонентов дрона (`PropVisual`, `MotorSound`, `QuadPhysics`, `FlightController`, `UdpLink`, `BatteryModel`, `DroneInputManager`, `DroneBreakup`).
- **Падает** → виновник среди `ChaseCamera` / `SimHud` / `SimTelemetryPanel` / `SimBootstrap` / `WindSystem`.

**B2.** Дальше удаляй по одному объекту/компоненту из оставшейся половины и повторяй. Обычно хватает 2–3 итераций.

---

## И самое важное — лог редактора

Я прошу его третий раз, и он реально решит вопрос за минуту: **сразу после билда** открой `Console ▸ ⋮ ▸ Open Editor Log` и поищи:

```
script
missing
serializ
Exception
level0
```

Если проблема в скриптах, там будет что-то вроде *«The referenced script … is missing»* или ошибка сериализации с именем конкретного компонента — и мы попадём точно в цель без перебора.

Пришли: результат после разделения файлов + эти строки из Editor.log. Дальше добьём за один шаг.

### USER
спасибо все заработало билд собрался! мне очень нравится вид окна с графиками телеметрии! А можно ли сделать его вдвое меньше и дизайн приблизить к другим окнам по стилю оно собой перекрывает большую часть пространства и добавить крестик закрытия окна как на обычных окнах windows?

### ASSISTANT
Готово — окно стало компактным (520×372 вместо 1010×660), полностью в стиле остальных панелей HUD, с крестиком, сворачиванием, перетаскиванием за заголовок и живым ресайзом за угол.

## Полная замена `Assets/Scripts/SimTelemetryPanel.cs`

```csharp
using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

[DefaultExecutionOrder(110)]
public class SimTelemetryPanel : MonoBehaviour
{
    [Header("Окно")]
    [Tooltip("F7 — показать/скрыть. Крестик в заголовке закрывает, «–» сворачивает, двойной клик по заголовку тоже сворачивает")]
    public bool show = true;
    public Rect windowRect = new Rect(1380, 620, 520, 372);
    public bool collapsed;

    [Header("Запись")]
    [Tooltip("Частота выборки телеметрии, Гц")] [Range(5, 60)] public int sampleRateHz = 20;
    [Tooltip("Длина окна графиков, с")] [Range(10, 120)] public int windowSeconds = 30;
    public bool paused;
    [Tooltip("Окно скрыто или свёрнуто — полностью остановить запись, опрос событий и отрисовку")]
    public bool pauseWhenHidden = true;

    [Header("Графики")]
    [Tooltip("Частота перерисовки графиков, Гц")] [Range(2, 30)] public int plotHz = 10;
    [Range(28, 120)] public int graphHeight = 42;
    [Tooltip("Ширина графиков подстраивается под ширину окна")] public bool autoFitWidth = true;
    [Range(160, 1024)] public int graphWidth = 320;
    public bool autoScale = true;

    // ---------- метрики ----------
    const float Pad = 7, TitleH = 22, TabsH = 20, ToolH = 18, HeadH = 18, ValH = 14, Gap = 6, ScrollW = 8;
    const float MinW = 340, MinH = 176, CardH = 104;

    // ---------- каналы ----------
    const int ALT = 0, VS = 1, P = 2, Q = 3, R = 4, WX = 5, WZ = 6, GE = 7,
              ROLL = 8, PITCH = 9, VBAT = 10, AMPS = 11, RPM0 = 12, DUTY0 = 16, CHANS = 20;

    class Chan { public int id; public string name; public Color color; public float current; public string text = "0.00"; }
    class Group
    {
        public string title; public bool open; public Chan[] chans;
        public Texture2D tex; public Color32[] px;
        public float min, max; public bool scaled; public string rangeText = "";
    }
    class RotorCard
    {
        public string title = "", status = "", rpm = "", l1 = "", l2 = "", l3 = "", l4 = "";
        public Color statusColor; public float duty;
    }
    struct Evt { public string time; public string text; public Color c; }

    float[][] buf; int cap, head, count;
    readonly List<Evt> events = new List<Evt>();
    readonly RotorCard[] cards = { new RotorCard(), new RotorCard(), new RotorCard(), new RotorCard() };
    List<Group> groups;
    int tab; float sampleTimer, plotTimer, savedHeight = 372;
    Vector2 scrollTraces, scrollRotors, scrollLog;
    string infoText = "", summary1 = "", summary2 = "";
    float plotMs; bool resizing; int gripId = -1;
    static readonly System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

    FlightController fc; QuadPhysics ph; BatteryModel bat; WindSystem wind; DroneBreakup breakup;
    System.Action crashHandler; bool subscribed;

    // ---------- палитра (общая с HUD) ----------
    static readonly Color Bg = new Color(0.02f, 0.04f, 0.07f, 0.94f);
    static readonly Color TitleBg = new Color(0.04f, 0.075f, 0.11f, 0.98f);
    static readonly Color Surface = new Color(0.055f, 0.085f, 0.125f, 0.95f);
    static readonly Color SurfaceHi = new Color(0.10f, 0.15f, 0.21f, 0.98f);
    static readonly Color Track = new Color(1f, 1f, 1f, 0.07f);
    static readonly Color Border = new Color(0.25f, 0.9f, 1f, 0.22f);
    static readonly Color GraphBg = new Color(0.015f, 0.03f, 0.05f, 1f);
    static readonly Color Acc = new Color(0.25f, 0.9f, 1f);
    static readonly Color Ok = new Color(0.3f, 1f, 0.45f);
    static readonly Color Warn = new Color(1f, 0.75f, 0.2f);
    static readonly Color Err = new Color(1f, 0.3f, 0.3f);
    static readonly Color Dim = new Color(0.72f, 0.78f, 0.84f);
    static readonly Color Cp = new Color(1f, 0.35f, 0.7f);
    static readonly Color Cb = new Color(0.4f, 0.6f, 1f);
    static readonly Color Cv = new Color(0.7f, 0.5f, 1f);
    static readonly Color Cy = new Color(1f, 0.85f, 0.2f);
    static readonly Color Co = new Color(1f, 0.6f, 0.2f);

    // ---------- стили ----------
    bool styled;
    Texture2D white;
    readonly Dictionary<int, Texture2D> texCache = new Dictionary<int, Texture2D>();
    readonly List<Texture2D> owned = new List<Texture2D>();
    GUIStyle stWindow, stTabOn, stTabOff, stChipOn, stChipOff, stHeader, stClose, stMin;
    GUIStyle stTitle, stTiny, stSmall, stBig, stTinyR, stTinyC, stSmallBold;
    GUIStyle stVBar, stVThumb, stHBar, stHThumb;

    bool lastArmed, lastCrashed, lastFs, lastLow, lastCrit, lastVrs, lastBroken;
    FlightMode lastMode;

    void Start()
    {
        fc = FindObjectOfType<FlightController>();
        ph = FindObjectOfType<QuadPhysics>();
        bat = FindObjectOfType<BatteryModel>();
        wind = FindObjectOfType<WindSystem>();
        breakup = FindObjectOfType<DroneBreakup>();
        enabled = fc != null && ph != null;
        if (!enabled) return;

        savedHeight = windowRect.height;
        lastMode = fc.Mode;
        crashHandler = () => Log("CRASH — удар", Err);
        ph.OnCrash += crashHandler;
        subscribed = true;

        groups = new List<Group>
        {
            new Group { title = "altitude (m) · climb rate (m/s)", open = true, chans = new[]
            {
                new Chan { id = ALT, name = "alt", color = Acc },
                new Chan { id = VS,  name = "vs",  color = Cb }
            }},
            new Group { title = "body rates p/q/r (°/s)", open = true, chans = new[]
            {
                new Chan { id = P, name = "p", color = Cp },
                new Chan { id = Q, name = "q", color = Ok },
                new Chan { id = R, name = "r", color = Cb }
            }},
            new Group { title = "wind X/Z (m/s) · ground effect", open = false, chans = new[]
            {
                new Chan { id = WX, name = "wx", color = Cv },
                new Chan { id = WZ, name = "wz", color = new Color(0.5f, 0.4f, 0.9f) },
                new Chan { id = GE, name = "GE", color = Cy }
            }},
            new Group { title = "rotor rpm (об/мин)", open = true, chans = new[]
            {
                new Chan { id = RPM0 + 0, name = "M1", color = Acc },
                new Chan { id = RPM0 + 1, name = "M2", color = Ok },
                new Chan { id = RPM0 + 2, name = "M3", color = Cp },
                new Chan { id = RPM0 + 3, name = "M4", color = Co }
            }},
            new Group { title = "motor duty (%)", open = false, chans = new[]
            {
                new Chan { id = DUTY0 + 0, name = "M1", color = Acc },
                new Chan { id = DUTY0 + 1, name = "M2", color = Ok },
                new Chan { id = DUTY0 + 2, name = "M3", color = Cp },
                new Chan { id = DUTY0 + 3, name = "M4", color = Co }
            }},
            new Group { title = "battery V / A", open = false, chans = new[]
            {
                new Chan { id = VBAT, name = "V", color = Ok },
                new Chan { id = AMPS, name = "A", color = Warn }
            }},
            new Group { title = "attitude roll/pitch (°)", open = false, chans = new[]
            {
                new Chan { id = ROLL,  name = "roll",  color = Acc },
                new Chan { id = PITCH, name = "pitch", color = Cp }
            }},
        };
        EnsureBuffer();
        Log("панель телеметрии запущена", Dim);
    }

    void Update()
    {
        bool key = false;
#if ENABLE_INPUT_SYSTEM
        var kb = Keyboard.current; if (kb != null) key = kb.f7Key.wasPressedThisFrame;
#elif ENABLE_LEGACY_INPUT_MANAGER
        key = Input.GetKeyDown(KeyCode.F7);
#endif
        if (key) show = !show;

        if (fc == null || ph == null || groups == null) return;
        bool idle = !show || collapsed;
        if (idle && pauseWhenHidden) return;

        EnsureBuffer();
        PollEvents();

        sampleTimer -= Time.unscaledDeltaTime;
        if (sampleTimer <= 0f)
        {
            sampleTimer = 1f / Mathf.Max(1, sampleRateHz);
            if (!paused) Push();
        }

        plotTimer -= Time.unscaledDeltaTime;
        if (plotTimer <= 0f)
        {
            plotTimer = 1f / Mathf.Max(1, plotHz);
            if (!idle && tab == 0)
            {
                sw.Restart();
                int gw = GraphW(), gh = Mathf.Max(20, graphHeight);
                foreach (var g in groups) if (g.open) Plot(g, gw, gh);
                sw.Stop();
                plotMs = Mathf.Lerp(plotMs, (float)sw.Elapsed.TotalMilliseconds, 0.3f);
            }
            if (!idle && tab == 1) UpdateCards();
            infoText = $"{count}/{cap} · {sampleRateHz}Гц · plot {plotMs:0.0}ms";
        }
    }

    // ---------- буфер ----------
    void EnsureBuffer()
    {
        int want = Mathf.Clamp(sampleRateHz * windowSeconds, 16, 4000);
        if (buf != null && cap == want) return;
        cap = want;
        buf = new float[CHANS][];
        for (int i = 0; i < CHANS; i++) buf[i] = new float[cap];
        head = 0; count = 0;
        if (groups != null) foreach (var g in groups) g.scaled = false;
    }

    void Push()
    {
        buf[ALT][head] = fc.AltAgl;
        buf[VS][head] = fc.Vs;
        buf[ROLL][head] = fc.AttRoll;
        buf[PITCH][head] = fc.AttPitchUp;
        buf[VBAT][head] = bat != null ? bat.Voltage : 0f;
        buf[AMPS][head] = bat != null ? bat.Current : 0f;
        buf[GE][head] = ph.GroundEffectFactor;
        buf[WX][head] = wind != null ? wind.Current.x : 0f;
        buf[WZ][head] = wind != null ? wind.Current.z : 0f;

        Vector3 w = ph.Rb != null
            ? ph.transform.InverseTransformDirection(ph.Rb.angularVelocity) * Mathf.Rad2Deg
            : Vector3.zero;
        buf[P][head] = -w.z; buf[Q][head] = w.x; buf[R][head] = w.y;

        for (int i = 0; i < 4; i++)
        {
            buf[RPM0 + i][head] = ph.motors[i].rpm;
            buf[DUTY0 + i][head] = ph.motors[i].cmd * 100f;
        }
        head++; if (head >= cap) head = 0;
        if (count < cap) count++;
    }

    // ---------- графики ----------
    int GraphW()
    {
        float avail = windowRect.width - Pad * 2 - ScrollW;
        int w = autoFitWidth ? (int)avail : graphWidth;
        w = Mathf.Clamp((w / 8) * 8, 96, 1024);
        return w;
    }

    void EnsureTex(Group g, int w, int h)
    {
        if (g.tex != null && g.tex.width == w && g.tex.height == h) return;
        if (g.tex != null) Destroy(g.tex);
        g.tex = new Texture2D(w, h, TextureFormat.RGBA32, false)
        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp, hideFlags = HideFlags.HideAndDontSave };
        g.px = new Color32[w * h];
    }

    void Plot(Group g, int W, int H)
    {
        EnsureTex(g, W, H);
        var px = g.px;
        Color32 bg = new Color32(4, 8, 13, 255);
        for (int i = 0; i < px.Length; i++) px[i] = bg;
        Color32 grid = new Color32(20, 30, 40, 255);
        for (int gy = 1; gy < 4; gy++) { int y = H * gy / 4; for (int x = 0; x < W; x++) px[y * W + x] = grid; }
        for (int gx = 1; gx < 6; gx++) { int x = W * gx / 6; for (int y = 0; y < H; y++) px[y * W + x] = grid; }

        if (count < 2) { g.tex.SetPixels32(px); g.tex.Apply(false); return; }
        int start = head - count; if (start < 0) start += cap;

        float tmin = float.MaxValue, tmax = float.MinValue;
        for (int c = 0; c < g.chans.Length; c++)
        {
            var arr = buf[g.chans[c].id];
            int idx = start;
            for (int i = 0; i < count; i++)
            {
                float v = arr[idx];
                if (v < tmin) tmin = v;
                if (v > tmax) tmax = v;
                idx++; if (idx >= cap) idx = 0;
            }
        }
        if (tmin > tmax) { tmin = 0; tmax = 1; }
        float pad = (tmax - tmin) * 0.15f + 0.001f;
        tmin -= pad; tmax += pad;
        if (autoScale)
        {
            if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
            else { g.min = Mathf.Lerp(g.min, tmin, 0.25f); g.max = Mathf.Lerp(g.max, tmax, 0.25f); }
        }
        else if (!g.scaled) { g.min = tmin; g.max = tmax; g.scaled = true; }
        g.rangeText = $"{g.min:0.0} … {g.max:0.0}";

        float range = Mathf.Max(1e-4f, g.max - g.min);
        int denom = Mathf.Max(1, cap - 1);
        for (int c = 0; c < g.chans.Length; c++)
        {
            var ch = g.chans[c];
            var arr = buf[ch.id];
            Color32 col = ch.color;
            int idx = start, px0 = 0, py0 = 0;
            float last = 0f;
            for (int i = 0; i < count; i++)
            {
                float v = arr[idx];
                int x = i * (W - 1) / denom;
                int y = Mathf.Clamp((int)((v - g.min) / range * (H - 1) + 0.5f), 0, H - 1);
                if (i > 0) Line(px, W, H, px0, py0, x, y, col);
                px0 = x; py0 = y; last = v;
                idx++; if (idx >= cap) idx = 0;
            }
            ch.current = last;
            ch.text = last.ToString("0.00");
        }
        g.tex.SetPixels32(px);
        g.tex.Apply(false);
    }

    static void Line(Color32[] px, int W, int H, int x0, int y0, int x1, int y1, Color32 c)
    {
        int dx = Mathf.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (x0 >= 0 && x0 < W && y0 >= 0 && y0 < H) px[y0 * W + x0] = c;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    // ---------- карточки ----------
    void UpdateCards()
    {
        for (int i = 0; i < 4; i++)
        {
            var m = ph.motors[i];
            var cd = cards[i];
            cd.title = "M" + (i + 1) + " · " + (m.spinDir > 0 ? "CW" : "CCW");
            if (m.cmd > 0.97f) { cd.status = "SATURATED"; cd.statusColor = Warn; }
            else if (m.vrs) { cd.status = "VRS"; cd.statusColor = Warn; }
            else if (m.rpm < 100f) { cd.status = "OFF"; cd.statusColor = Dim; }
            else { cd.status = "RUN"; cd.statusColor = Ok; }
            cd.rpm = m.rpm.ToString("N0");
            cd.duty = m.cmd;

            float eff = ph.gramsPerWatt * (1.15f - 0.5f * m.cmd);
            float power = (m.thrustN / 0.00981f) / Mathf.Max(1f, eff);
            float amps = bat != null ? power / Mathf.Max(1f, bat.Voltage) : 0f;
            cd.l1 = $"T {m.thrustN:0.00} N    Q {m.torqueNm:0.0000}";
            cd.l2 = $"I {amps:0.0} A     DUTY {m.cmd * 100f:0}%";
            cd.l3 = $"IGE {Ige(m):0.000}   VRS {(m.vrs ? "1.00" : "0.00")}";
            cd.l4 = $"h {m.groundH:0.00} m   ω {m.rpm * 0.10472f:0} rad/s";
        }
        summary1 = $"Тяга {ph.TotalThrustN:0.0} Н ({ph.TotalThrustN / 0.00981f:0} г)   Мощность {ph.PowerW:0} Вт";
        summary2 = bat != null
            ? $"Батарея {bat.Voltage:0.00} В  {bat.Current:0.0} А  {bat.Soc * 100f:0}%   GE x{ph.GroundEffectFactor:0.000}   AS {ph.Airspeed:0.0} м/с"
            : $"GE x{ph.GroundEffectFactor:0.000}   AS {ph.Airspeed:0.0} м/с";
    }

    float Ige(MotorUnit m)
    {
        if (!ph.groundEffect) return 1f;
        float R = ph.propRadius;
        if (m.groundH >= 4f * R) return 1f;
        float h = Mathf.Max(m.groundH, R * 0.35f);
        float r = R / (4f * h);
        return Mathf.Clamp(1f / (1f - r * r), 1f, ph.groundEffectMax);
    }

    // ---------- журнал ----------
    void Log(string text, Color c)
    {
        events.Add(new Evt { time = Time.time.ToString("0.0") + "s", text = text, c = c });
        while (events.Count > 300) events.RemoveAt(0);
    }

    void PollEvents()
    {
        if (fc.Armed != lastArmed) { lastArmed = fc.Armed; Log(fc.Armed ? "ARM" : "DISARM", fc.Armed ? Ok : Dim); }
        if (fc.Mode != lastMode) { lastMode = fc.Mode; Log("MODE → " + fc.Mode, Acc); }
        if (ph.Crashed != lastCrashed) { lastCrashed = ph.Crashed; Log(ph.Crashed ? "CRASHED" : "REPAIRED / RESET", ph.Crashed ? Err : Ok); }
        if (fc.Failsafe != lastFs) { lastFs = fc.Failsafe; Log(fc.Failsafe ? "FAILSAFE ON" : "FAILSAFE OFF", fc.Failsafe ? Err : Ok); }
        if (bat != null)
        {
            if (bat.IsLow != lastLow) { lastLow = bat.IsLow; if (bat.IsLow) Log("LOW BATTERY", Warn); }
            if (bat.IsCritical != lastCrit) { lastCrit = bat.IsCritical; if (bat.IsCritical) Log("BATTERY CRITICAL", Err); }
        }
        if (ph.VrsActive != lastVrs) { lastVrs = ph.VrsActive; if (ph.VrsActive) Log("VORTEX RING STATE", Warn); }
        if (breakup != null && breakup.IsBroken != lastBroken)
        { lastBroken = breakup.IsBroken; Log(lastBroken ? "BREAKUP — дрон разобран" : "REASSEMBLED", lastBroken ? Err : Ok); }
    }

    // ---------- стили ----------
    Texture2D Solid(Color c)
    {
        int key = ((int)(c.r * 255f) << 24) | ((int)(c.g * 255f) << 16) | ((int)(c.b * 255f) << 8) | (int)(c.a * 255f);
        if (texCache.TryGetValue(key, out var t) && t != null) return t;
        t = new Texture2D(1, 1, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave };
        t.SetPixel(0, 0, c); t.Apply(false);
        texCache[key] = t; owned.Add(t);
        return t;
    }

    GUIStyle Btn(Color n, Color h, Color a, Color textN, Color textH, int size, TextAnchor anchor, int padL)
    {
        var s = new GUIStyle
        {
            fontSize = size,
            fontStyle = FontStyle.Bold,
            alignment = anchor,
            padding = new RectOffset(padL, padL, 0, 0),
            margin = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0)
        };
        s.normal.background = Solid(n); s.normal.textColor = textN;
        s.hover.background = Solid(h); s.hover.textColor = textH;
        s.active.background = Solid(a); s.active.textColor = textH;
        s.focused.background = Solid(n); s.focused.textColor = textN;
        return s;
    }

    GUIStyle Lbl(int size, TextAnchor anchor, FontStyle fs)
    {
        var s = new GUIStyle
        {
            fontSize = size,
            alignment = anchor,
            fontStyle = fs,
            clipping = TextClipping.Clip,
            padding = new RectOffset(0, 0, 0, 0),
            margin = new RectOffset(0, 0, 0, 0)
        };
        s.normal.textColor = Color.white;
        return s;
    }

    void EnsureStyles()
    {
        if (styled) return;
        styled = true;

        white = new Texture2D(1, 1, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave };
        white.SetPixel(0, 0, Color.white); white.Apply(false);
        owned.Add(white);

        stWindow = new GUIStyle { padding = new RectOffset(0, 0, 0, 0), margin = new RectOffset(0, 0, 0, 0) };
        stWindow.normal.background = null;

        Color tabOn = new Color(0.10f, 0.28f, 0.34f, 0.98f);
        Color tabOnHi = new Color(0.14f, 0.36f, 0.44f, 1f);
        stTabOn = Btn(tabOn, tabOnHi, tabOnHi, Acc, Color.white, 10, TextAnchor.MiddleCenter, 4);
        stTabOff = Btn(Surface, SurfaceHi, SurfaceHi, Dim, Color.white, 10, TextAnchor.MiddleCenter, 4);
        stChipOn = Btn(tabOn, tabOnHi, tabOnHi, Acc, Color.white, 9, TextAnchor.MiddleCenter, 3);
        stChipOff = Btn(Surface, SurfaceHi, SurfaceHi, Dim, Color.white, 9, TextAnchor.MiddleCenter, 3);
        stHeader = Btn(new Color(0.05f, 0.085f, 0.125f, 0.96f), SurfaceHi, SurfaceHi,
                       Acc, Color.white, 10, TextAnchor.MiddleLeft, 8);
        stClose = Btn(new Color(0, 0, 0, 0), new Color(0.85f, 0.20f, 0.20f, 0.95f), new Color(1f, 0.32f, 0.32f, 1f),
                      Dim, Color.white, 14, TextAnchor.MiddleCenter, 0);
        stMin = Btn(new Color(0, 0, 0, 0), new Color(0.14f, 0.36f, 0.44f, 0.95f), new Color(0.18f, 0.46f, 0.56f, 1f),
                    Dim, Color.white, 13, TextAnchor.MiddleCenter, 0);

        stTitle = Lbl(12, TextAnchor.MiddleLeft, FontStyle.Bold);
        stTiny = Lbl(9, TextAnchor.MiddleLeft, FontStyle.Normal);
        stTinyR = Lbl(9, TextAnchor.MiddleRight, FontStyle.Normal);
        stTinyC = Lbl(9, TextAnchor.MiddleCenter, FontStyle.Bold);
        stSmall = Lbl(10, TextAnchor.MiddleLeft, FontStyle.Normal);
        stSmallBold = Lbl(10, TextAnchor.MiddleLeft, FontStyle.Bold);
        stBig = Lbl(22, TextAnchor.MiddleLeft, FontStyle.Bold);

        stVThumb = new GUIStyle
        {
            fixedWidth = ScrollW,
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0),
            stretchHeight = true
        };
        stVThumb.normal.background = Solid(new Color(Acc.r, Acc.g, Acc.b, 0.42f));
        stVThumb.hover.background = Solid(new Color(Acc.r, Acc.g, Acc.b, 0.72f));
        stVThumb.active.background = Solid(Acc);

        stVBar = new GUIStyle
        {
            fixedWidth = ScrollW,
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0),
            stretchHeight = true,
            thumb = stVThumb
        };
        stVBar.normal.background = Solid(new Color(1f, 1f, 1f, 0.05f));
        stVBar.hover.background = Solid(new Color(1f, 1f, 1f, 0.08f));

        stHThumb = new GUIStyle
        {
            fixedHeight = ScrollW,
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0),
            stretchWidth = true
        };
        stHThumb.normal.background = Solid(new Color(Acc.r, Acc.g, Acc.b, 0.42f));
        stHThumb.hover.background = Solid(new Color(Acc.r, Acc.g, Acc.b, 0.72f));
        stHThumb.active.background = Solid(Acc);

        stHBar = new GUIStyle
        {
            fixedHeight = ScrollW,
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0),
            stretchWidth = true,
            thumb = stHThumb
        };
        stHBar.normal.background = Solid(new Color(1f, 1f, 1f, 0.05f));
    }

    // ---------- примитивы отрисовки ----------
    void Fill(Rect r, Color c)
    {
        if (r.width <= 0 || r.height <= 0) return;
        var old = GUI.color; GUI.color = c;
        GUI.DrawTexture(r, white);
        GUI.color = old;
    }
    void Txt(Rect r, string s, GUIStyle st, Color c)
    {
        var old = GUI.color; GUI.color = c;
        GUI.Label(r, s, st);
        GUI.color = old;
    }
    void Frame(Rect r, Color c)
    {
        Fill(new Rect(r.x, r.y, r.width, 1), c);
        Fill(new Rect(r.x, r.yMax - 1, r.width, 1), c);
        Fill(new Rect(r.x, r.y, 1, r.height), c);
        Fill(new Rect(r.xMax - 1, r.y, 1, r.height), c);
    }
    void Bar(Rect r, float v, Color c)
    {
        Fill(r, Track);
        Fill(new Rect(r.x, r.y, r.width * Mathf.Clamp01(v), r.height), c);
    }

    // ---------- GUI ----------
    void OnGUI()
    {
        if (!show || fc == null || groups == null) return;
        EnsureStyles();

        var saved = GUI.matrix;
        var savedColor = GUI.color;
        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        float W = Screen.width / scale, H = 1080f;

        windowRect.width = Mathf.Clamp(windowRect.width, MinW, W);
        windowRect.height = collapsed ? TitleH : Mathf.Clamp(windowRect.height, MinH, H);
        windowRect.x = Mathf.Clamp(windowRect.x, 0, Mathf.Max(0, W - windowRect.width));
        windowRect.y = Mathf.Clamp(windowRect.y, 0, Mathf.Max(0, H - windowRect.height));

        try { windowRect = GUI.Window(77, windowRect, DrawWindow, GUIContent.none, stWindow); }
        catch (System.Exception e) { show = false; UnityEngine.Debug.LogError("[Telemetry] " + e); }

        GUI.color = savedColor;
        GUI.matrix = saved;
    }

    void ToggleCollapse()
    {
        if (!collapsed) { savedHeight = windowRect.height; windowRect.height = TitleH; collapsed = true; }
        else { windowRect.height = Mathf.Max(MinH, savedHeight); collapsed = false; }
    }

    void DrawWindow(int id)
    {
        float w = windowRect.width, h = windowRect.height;
        Fill(new Rect(0, 0, w, h), Bg);

        // ---- заголовок ----
        Rect tr = new Rect(0, 0, w, TitleH);
        Fill(tr, TitleBg);
        Fill(new Rect(0, 0, w, 1), new Color(Acc.r, Acc.g, Acc.b, 0.75f));
        Fill(new Rect(0, TitleH - 1, w, 1), new Color(Acc.r, Acc.g, Acc.b, 0.20f));
        Fill(new Rect(Pad, TitleH * 0.5f - 3, 6, 6), paused ? Warn : Ok);
        Txt(new Rect(Pad + 12, 0, 150, TitleH), "ТЕЛЕМЕТРИЯ", stTitle, Acc);
        Txt(new Rect(Pad + 104, 0, 60, TitleH), "[F7]", stTiny, Dim);

        Rect bClose = new Rect(w - Pad - 16, 4, 16, TitleH - 8);
        Rect bMin = new Rect(bClose.x - 19, 4, 16, TitleH - 8);
        if (GUI.Button(bMin, collapsed ? "+" : "–", stMin)) ToggleCollapse();
        if (GUI.Button(bClose, "×", stClose)) { show = false; return; }

        var ev = Event.current;
        Rect dragArea = new Rect(0, 0, Mathf.Max(0, bMin.x - 4), TitleH);
        if (ev.type == EventType.MouseDown && ev.button == 0 && ev.clickCount == 2 && dragArea.Contains(ev.mousePosition))
        { ToggleCollapse(); ev.Use(); return; }

        if (!collapsed)
        {
            float cx = Pad, cw = w - Pad * 2;

            // ---- вкладки ----
            float ty = TitleH + 2;
            string[] tabs = { "LIVE TRACES", "ROTOR BENCH", "EVENT LOG" };
            float tw = (cw - 8) / 3f;
            for (int i = 0; i < 3; i++)
                if (GUI.Button(new Rect(cx + i * (tw + 4), ty, tw, TabsH - 4), tabs[i], tab == i ? stTabOn : stTabOff)) tab = i;

            // ---- панель инструментов ----
            float by = ty + TabsH;
            float bx = cx;
            int[] secs = { 10, 30, 60 };
            foreach (var s in secs)
            {
                if (GUI.Button(new Rect(bx, by, 32, ToolH - 4), s + "с", windowSeconds == s ? stChipOn : stChipOff)) windowSeconds = s;
                bx += 34;
            }
            bx += 4;
            if (GUI.Button(new Rect(bx, by, 74, ToolH - 4), "AUTOSCALE", autoScale ? stChipOn : stChipOff)) autoScale = !autoScale;
            bx += 78;
            if (GUI.Button(new Rect(bx, by, 52, ToolH - 4), paused ? "● REC" : "PAUSE", paused ? stChipOn : stChipOff)) paused = !paused;
            bx += 56;
            if (GUI.Button(new Rect(bx, by, 48, ToolH - 4), "CLEAR", stChipOff))
            { head = 0; count = 0; events.Clear(); foreach (var g in groups) g.scaled = false; }
            bx += 54;
            float infoW = Mathf.Max(0, cx + cw - bx);
            if (infoW > 40) Txt(new Rect(bx, by, infoW, ToolH - 4), infoText, stTinyR, Dim);

            // ---- содержимое ----
            float top = by + ToolH + 2;
            Rect view = new Rect(cx, top, cw, Mathf.Max(20, h - top - Pad));
            Fill(view, new Color(0.015f, 0.03f, 0.05f, 0.55f));
            if (tab == 0) DrawTraces(view);
            else if (tab == 1) DrawRotors(view);
            else DrawLog(view);

            // ---- уголок ресайза ----
            Rect grip = new Rect(w - 13, h - 13, 13, 13);
            for (int i = 0; i < 3; i++)
                Fill(new Rect(grip.x + 3 + i * 3, grip.yMax - 4 - i * 3, 2 + i * 0, 2), new Color(Acc.r, Acc.g, Acc.b, 0.55f));
            for (int i = 0; i < 3; i++)
                Fill(new Rect(grip.xMax - 4 - i * 3, grip.y + 4 + i * 3, 2, 2), new Color(Acc.r, Acc.g, Acc.b, 0.30f));

            if (gripId < 0) gripId = GUIUtility.GetControlID(FocusType.Passive);
            else GUIUtility.GetControlID(FocusType.Passive);

            if (ev.type == EventType.MouseDown && ev.button == 0 && grip.Contains(ev.mousePosition))
            { resizing = true; GUIUtility.hotControl = gripId; ev.Use(); }
            else if (ev.type == EventType.MouseDrag && resizing && GUIUtility.hotControl == gripId)
            {
                windowRect.width = Mathf.Max(MinW, windowRect.width + ev.delta.x);
                windowRect.height = Mathf.Max(MinH, windowRect.height + ev.delta.y);
                ev.Use();
            }
            else if (ev.type == EventType.MouseUp && resizing && GUIUtility.hotControl == gripId)
            { resizing = false; GUIUtility.hotControl = 0; ev.Use(); }
        }

        Frame(new Rect(0, 0, w, h), Border);
        GUI.DragWindow(dragArea);
    }

    void DrawTraces(Rect view)
    {
        float cw = view.width - ScrollW;
        float total = 0;
        foreach (var g in groups) { total += HeadH; if (g.open) total += ValH + graphHeight + Gap; }
        Rect content = new Rect(0, 0, cw, Mathf.Max(total, view.height));
        scrollTraces = GUI.BeginScrollView(view, scrollTraces, content, false, true, stHBar, stVBar, GUIStyle.none);

        float y = 0;
        int gw = GraphW(), gh = Mathf.Max(20, graphHeight);
        foreach (var g in groups)
        {
            Rect hr = new Rect(0, y, cw, HeadH - 3);
            if (GUI.Button(hr, (g.open ? "▼  " : "►  ") + g.title, stHeader)) { g.open = !g.open; g.scaled = false; }
            Fill(new Rect(hr.x, hr.y, 2, hr.height), new Color(Acc.r, Acc.g, Acc.b, 0.8f));
            y += HeadH;
            if (!g.open) continue;

            float vw = Mathf.Min(94f, (cw - 96f) / Mathf.Max(1, g.chans.Length));
            for (int i = 0; i < g.chans.Length; i++)
                Txt(new Rect(2 + i * vw, y, vw - 3, ValH), g.chans[i].text + " " + g.chans[i].name, stTiny, g.chans[i].color);
            Txt(new Rect(cw - 94, y, 92, ValH), g.rangeText, stTinyR, Dim);
            y += ValH;

            Rect gr = new Rect(0, y, Mathf.Min(gw, cw), gh);
            Fill(gr, GraphBg);
            if (g.tex != null) GUI.DrawTexture(gr, g.tex);
            Frame(gr, new Color(Acc.r, Acc.g, Acc.b, 0.16f));
            y += gh + Gap;
        }
        GUI.EndScrollView();
    }

    void DrawRotors(Rect view)
    {
        float cw = view.width - ScrollW;
        int cols = Mathf.Clamp((int)(cw / 196f), 1, 4);
        int rows = Mathf.CeilToInt(4f / cols);
        float cardW = (cw - (cols - 1) * Gap) / cols;
        float total = rows * (CardH + Gap) + 34;
        Rect content = new Rect(0, 0, cw, Mathf.Max(total, view.height));
        scrollRotors = GUI.BeginScrollView(view, scrollRotors, content, false, true, stHBar, stVBar, GUIStyle.none);

        for (int i = 0; i < 4; i++)
        {
            int col = i % cols, row = i / cols;
            DrawCard(new Rect(col * (cardW + Gap), row * (CardH + Gap), cardW, CardH), cards[i]);
        }
        float sy = rows * (CardH + Gap) + 2;
        Txt(new Rect(2, sy, cw - 4, 14), summary1, stSmall, Acc);
        Txt(new Rect(2, sy + 15, cw - 4, 14), summary2, stSmall, Dim);
        GUI.EndScrollView();
    }

    void DrawCard(Rect r, RotorCard cd)
    {
        Fill(r, Surface);
        Fill(new Rect(r.x, r.y, r.width, 1), new Color(Acc.r, Acc.g, Acc.b, 0.5f));
        Frame(r, new Color(Acc.r, Acc.g, Acc.b, 0.12f));

        Txt(new Rect(r.x + 6, r.y + 4, r.width - 78, 13), cd.title, stSmallBold, Dim);
        Rect chip = new Rect(r.xMax - 70, r.y + 4, 64, 13);
        Fill(chip, new Color(cd.statusColor.r, cd.statusColor.g, cd.statusColor.b, 0.16f));
        Txt(chip, cd.status, stTinyC, cd.statusColor);

        Txt(new Rect(r.x + 6, r.y + 17, r.width - 46, 26), cd.rpm, stBig, Acc);
        Txt(new Rect(r.xMax - 40, r.y + 30, 34, 12), "RPM", stTiny, Dim);

        Bar(new Rect(r.x + 6, r.y + 46, r.width - 12, 4), cd.duty, cd.duty > 0.97f ? Warn : Ok);

        Txt(new Rect(r.x + 6, r.y + 54, r.width - 12, 12), cd.l1, stTiny, Dim);
        Txt(new Rect(r.x + 6, r.y + 66, r.width - 12, 12), cd.l2, stTiny, Dim);
        Txt(new Rect(r.x + 6, r.y + 78, r.width - 12, 12), cd.l3, stTiny, Dim);
        Txt(new Rect(r.x + 6, r.y + 90, r.width - 12, 12), cd.l4, stTiny, new Color(Dim.r, Dim.g, Dim.b, 0.7f));
    }

    void DrawLog(Rect view)
    {
        float cw = view.width - ScrollW;
        const float rowH = 15;
        Rect content = new Rect(0, 0, cw, Mathf.Max(events.Count * rowH, view.height));
        scrollLog = GUI.BeginScrollView(view, scrollLog, content, false, true, stHBar, stVBar, GUIStyle.none);
        for (int i = 0; i < events.Count; i++)
        {
            var e = events[events.Count - 1 - i];
            Rect r = new Rect(0, i * rowH, cw, rowH - 1);
            if ((i & 1) == 0) Fill(r, new Color(1f, 1f, 1f, 0.022f));
            Txt(new Rect(4, r.y, 46, r.height), e.time, stTiny, Dim);
            Fill(new Rect(52, r.y + 3, 2, r.height - 6), new Color(e.c.r, e.c.g, e.c.b, 0.7f));
            Txt(new Rect(60, r.y, cw - 64, r.height), e.text, stSmall, e.c);
        }
        GUI.EndScrollView();
    }

    void OnDestroy()
    {
        if (subscribed && ph != null && crashHandler != null) ph.OnCrash -= crashHandler;
        subscribed = false;
        if (groups != null) foreach (var g in groups) if (g.tex != null) Destroy(g.tex);
        foreach (var t in owned) if (t != null) Destroy(t);
        owned.Clear(); texCache.Clear();
    }
}
```

**Управление окном:** `F7` — показать/скрыть · `×` — закрыть · `–` / двойной клик по заголовку — свернуть в заголовок · тянуть за заголовок — переместить · тянуть за правый нижний угол — изменить размер (графики и карточки моторов перестраиваются под новую ширину автоматически).

### USER
я заменил скрипт SimTelemetryPanel.cs но возникли такие ошибки помоги их исправить, вот сами ошибки: 
Assets\Scripts\SimTelemetryPanel.cs(492,13): error CS0117: 'GUIStyle' does not contain a definition for 'thumb'

Assets\Scripts\SimTelemetryPanel.cs(516,13): error CS0117: 'GUIStyle' does not contain a definition for 'thumb'

Assets\Scripts\SimTelemetryPanel.cs(673,28): error CS1501: No overload for method 'BeginScrollView' takes 8 arguments

Assets\Scripts\SimTelemetryPanel.cs(708,28): error CS1501: No overload for method 'BeginScrollView' takes 8 arguments

Assets\Scripts\SimTelemetryPanel.cs(748,25): error CS1501: No overload for method 'BeginScrollView' takes 8 arguments



### ASSISTANT
Обе ошибки — мои промахи в API:

- **`GUIStyle.thumb` не существует.** Стиль бегунка скроллбара берётся не из стиля дорожки, а из `GUISkin.verticalScrollbarThumb` / `horizontalScrollbarThumb`.
- **`GUI.BeginScrollView`** (в отличие от `GUILayout.BeginScrollView`) имеет максимум **7 параметров** — версии с `background` у него нет.

Ниже 6 точечных правок.

---

## 1. Поле для копии скина

Найди строку `bool styled;` и замени на:

```csharp
    bool styled;
    GUISkin skinCopy;
```

## 2. Блок `stVBar` в `EnsureStyles()`

Найди блок от `stVBar = new GUIStyle` до двух строк `.background` и замени целиком на:

```csharp
        stVBar = new GUIStyle
        {
            fixedWidth = ScrollW,
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0),
            stretchHeight = true
        };
        stVBar.normal.background = Solid(new Color(1f, 1f, 1f, 0.05f));
        stVBar.hover.background = Solid(new Color(1f, 1f, 1f, 0.08f));
```

## 3. Блок `stHBar` (конец `EnsureStyles()`)

Найди блок от `stHBar = new GUIStyle` до конца метода и замени целиком на:

```csharp
        stHBar = new GUIStyle
        {
            fixedHeight = ScrollW,
            margin = new RectOffset(0, 0, 0, 0),
            padding = new RectOffset(0, 0, 0, 0),
            border = new RectOffset(0, 0, 0, 0),
            stretchWidth = true
        };
        stHBar.normal.background = Solid(new Color(1f, 1f, 1f, 0.05f));

        // собственный скин: бегунки скроллбаров Unity берёт именно отсюда
        skinCopy = Instantiate(GUI.skin);
        skinCopy.hideFlags = HideFlags.HideAndDontSave;
        skinCopy.verticalScrollbar = stVBar;
        skinCopy.verticalScrollbarThumb = stVThumb;
        skinCopy.horizontalScrollbar = stHBar;
        skinCopy.horizontalScrollbarThumb = stHThumb;
    }
```

## 4. Метод `OnGUI()` целиком

```csharp
    void OnGUI()
    {
        if (!show || fc == null || groups == null) return;
        EnsureStyles();

        var saved = GUI.matrix;
        var savedColor = GUI.color;
        var oldSkin = GUI.skin;

        float scale = Screen.height / 1080f;
        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(scale, scale, 1));
        float W = Screen.width / scale, H = 1080f;

        windowRect.width = Mathf.Clamp(windowRect.width, MinW, W);
        windowRect.height = collapsed ? TitleH : Mathf.Clamp(windowRect.height, MinH, H);
        windowRect.x = Mathf.Clamp(windowRect.x, 0, Mathf.Max(0, W - windowRect.width));
        windowRect.y = Mathf.Clamp(windowRect.y, 0, Mathf.Max(0, H - windowRect.height));

        if (skinCopy != null) GUI.skin = skinCopy;
        try { windowRect = GUI.Window(77, windowRect, DrawWindow, GUIContent.none, stWindow); }
        catch (System.Exception e) { show = false; UnityEngine.Debug.LogError("[Telemetry] " + e); }
        finally
        {
            GUI.skin = oldSkin;
            GUI.color = savedColor;
            GUI.matrix = saved;
        }
    }
```

## 5. Три вызова `BeginScrollView` — убрать последний аргумент

В `DrawTraces`:

```csharp
        scrollTraces = GUI.BeginScrollView(view, scrollTraces, content, false, true, stHBar, stVBar);
```

В `DrawRotors`:

```csharp
        scrollRotors = GUI.BeginScrollView(view, scrollRotors, content, false, true, stHBar, stVBar);
```

В `DrawLog`:

```csharp
        scrollLog = GUI.BeginScrollView(view, scrollLog, content, false, true, stHBar, stVBar);
```

## 6. `OnDestroy()` — освободить скин

```csharp
    void OnDestroy()
    {
        if (subscribed && ph != null && crashHandler != null) ph.OnCrash -= crashHandler;
        subscribed = false;
        if (groups != null) foreach (var g in groups) if (g.tex != null) Destroy(g.tex);
        foreach (var t in owned) if (t != null) Destroy(t);
        owned.Clear(); texCache.Clear();
        if (skinCopy != null) Destroy(skinCopy);
    }
```

---

Скин подменяется только на время отрисовки этого окна и возвращается в `finally`, поэтому остальной HUD (`SimHud`) остаётся со своим оформлением, а бегунки телеметрии будут cyan-овые в общей палитре. После этих правок всё скомпилируется.

### USER
все отлично работает но авиагоризонт отображается, странно можно ли это поправить чтобы было похоже на настоящий авиагоризонт в симуляторах

### ASSISTANT
## Новый файл `Assets/Scripts/AttitudeIndicator.cs`

```csharp
using UnityEngine;

// Круговой авиагоризонт: рендерится в текстуру с собственным круговым клиппингом,
// поэтому небо/земля/шкалы никогда не выходят за прибор при любом крене.
public sealed class AttitudeIndicator : System.IDisposable
{
    [Header("Внешний вид")]
    public int Size = 320;
    public float UpdateHz = 40f;
    public float PixelsPerDegree = 3.5f;
    public int MajorStep = 10;
    public int MinorStep = 5;
    public bool SlipBall = true;
    public bool GlassSheen = true;
    public bool RollScale = true;
    public Color SkyTop = new Color(0.055f, 0.30f, 0.62f);
    public Color SkyHorizon = new Color(0.33f, 0.63f, 0.90f);
    public Color GroundHorizon = new Color(0.55f, 0.40f, 0.21f);
    public Color GroundBottom = new Color(0.20f, 0.13f, 0.06f);
    public Color Ink = new Color(0.96f, 0.98f, 1f);
    public Color Symbol = new Color(1f, 0.84f, 0.08f);
    public Color Accent = new Color(0.25f, 0.9f, 1f);

    Texture2D tex; Color32[] px; int size;
    readonly Color32[] sky = new Color32[256], gnd = new Color32[256];
    readonly float[] vign = new float[256];
    bool lutBuilt; Color cST, cSH, cGH, cGB;
    float lastT = -1f, lastRoll = 999f, lastPitch = 999f, lastSlip = 999f;
    float c, R, R2, Ro2, sinR, cosR;

    static readonly byte[][] Glyph;
    static AttitudeIndicator()
    {
        string[] src =
        {
            "01110,10001,10011,10101,11001,10001,01110",
            "00100,01100,00100,00100,00100,00100,01110",
            "01110,10001,00001,00010,00100,01000,11111",
            "11111,00010,00100,00010,00001,10001,01110",
            "00010,00110,01010,10010,11111,00010,00010",
            "11111,10000,11110,00001,00001,10001,01110",
            "00110,01000,10000,11110,10001,10001,01110",
            "11111,00001,00010,00100,01000,01000,01000",
            "01110,10001,10001,01110,10001,10001,01110",
            "01110,10001,10001,01111,00001,00010,01100"
        };
        Glyph = new byte[10][];
        for (int i = 0; i < 10; i++)
        {
            var rows = src[i].Split(',');
            Glyph[i] = new byte[7];
            for (int r = 0; r < 7; r++)
            {
                byte b = 0;
                for (int k = 0; k < 5; k++) if (rows[r][k] == '1') b |= (byte)(1 << (4 - k));
                Glyph[i][r] = b;
            }
        }
    }

    public void Draw(Rect rect, float roll, float pitch, float slip)
    {
        if (Event.current == null || Event.current.type == EventType.Repaint) Ensure(roll, pitch, slip);
        if (tex == null) return;
        var old = GUI.color; GUI.color = Color.white;
        GUI.DrawTexture(rect, tex, ScaleMode.StretchToFill, true);
        GUI.color = old;
    }

    void Alloc()
    {
        size = Mathf.Clamp(Size, 128, 640);
        if (tex != null) Object.Destroy(tex);
        tex = new Texture2D(size, size, TextureFormat.RGBA32, false)
        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp, hideFlags = HideFlags.HideAndDontSave };
        px = new Color32[size * size];
        lastT = -1f;
    }

    void BuildLut()
    {
        for (int i = 0; i < 256; i++)
        {
            float t = i / 255f;
            sky[i] = Color.Lerp(SkyHorizon, SkyTop, Mathf.Pow(t, 0.75f));
            gnd[i] = Color.Lerp(GroundHorizon, GroundBottom, Mathf.Pow(t, 0.75f));
            vign[i] = 1f - 0.34f * Mathf.Pow(t, 1.6f);
        }
        cST = SkyTop; cSH = SkyHorizon; cGH = GroundHorizon; cGB = GroundBottom;
        lutBuilt = true;
    }

    void Ensure(float roll, float pitch, float slip)
    {
        if (tex == null || size != Mathf.Clamp(Size, 128, 640)) Alloc();
        if (!lutBuilt || cST != SkyTop || cSH != SkyHorizon || cGH != GroundHorizon || cGB != GroundBottom) BuildLut();

        float now = Time.realtimeSinceStartup;
        float interval = 1f / Mathf.Max(5f, UpdateHz);
        bool changed = Mathf.Abs(roll - lastRoll) > 0.02f || Mathf.Abs(pitch - lastPitch) > 0.02f || Mathf.Abs(slip - lastSlip) > 0.01f;
        if (lastT >= 0f && now - lastT < interval) return;
        if (lastT >= 0f && !changed && now - lastT < 0.4f) return;
        lastT = now; lastRoll = roll; lastPitch = pitch; lastSlip = slip;

        c = (size - 1) * 0.5f;
        float Ro = size * 0.5f - 0.5f;
        R = Ro - Mathf.Max(3f, size * 0.013f);
        R2 = R * R; Ro2 = Ro * Ro;
        sinR = Mathf.Sin(roll * Mathf.Deg2Rad); cosR = Mathf.Cos(roll * Mathf.Deg2Rad);
        float pOff = pitch * PixelsPerDegree;
        float gradK = 255f / (R * 1.1f), invR2 = 1f / R2, invR = 1f / R;

        Color32 clear = new Color32(0, 0, 0, 0);
        Color32 hor = Ink;
        Color32 bezel = new Color(0.055f, 0.085f, 0.12f);
        Color32 ring = new Color(Accent.r * 0.55f, Accent.g * 0.55f, Accent.b * 0.55f);
        float band = (R + 1.3f) * (R + 1.3f);

        for (int y = 0; y < size; y++)
        {
            float dy = c - y, dy2 = dy * dy;
            float v = c * sinR + dy * cosR + pOff;
            float dx = -c, dx2 = c * c;
            int row = y * size;
            for (int x = 0; x < size; x++)
            {
                float r2 = dx2 + dy2;
                if (r2 > Ro2) px[row + x] = clear;
                else if (r2 > R2)
                {
                    float shade = 0.70f + 0.45f * (dy / Ro);
                    Color32 bz = r2 < band ? ring : bezel;
                    px[row + x] = new Color32((byte)(bz.r * shade), (byte)(bz.g * shade), (byte)(bz.b * shade), 255);
                }
                else
                {
                    float av = v < 0 ? -v : v;
                    int vi = (int)(av * gradK); if (vi > 255) vi = 255;
                    Color32 col = v >= 0 ? sky[vi] : gnd[vi];
                    if (v > -1.15f && v < 1.15f) col = hor;
                    float sh = vign[(int)(r2 * invR2 * 255f)];
                    if (GlassSheen)
                    {
                        float s = (-dx * 0.45f + dy * 0.85f) * invR;
                        if (s > 0.35f) sh += (s - 0.35f) * 0.24f;
                    }
                    float rr = col.r * sh, gg = col.g * sh, bb = col.b * sh;
                    px[row + x] = new Color32((byte)(rr > 255 ? 255 : rr), (byte)(gg > 255 ? 255 : gg), (byte)(bb > 255 ? 255 : bb), 255);
                }
                v -= sinR; dx2 += 2f * dx + 1f; dx += 1f;
            }
        }

        DrawLadder(pitch);
        if (RollScale) DrawRollScale(roll, slip);
        DrawSymbol();

        tex.SetPixels32(px);
        tex.Apply(false);
    }

    // ---------- растр ----------
    void Blend(int x, int y, Color32 col, float a)
    {
        int i = y * size + x; var d = px[i];
        px[i] = new Color32((byte)(d.r + (col.r - d.r) * a), (byte)(d.g + (col.g - d.g) * a), (byte)(d.b + (col.b - d.b) * a), 255);
    }

    void Stamp(float ix, float iy, float rad, Color32 col, float a)
    {
        float px0 = c + ix, py0 = c - iy;
        int x0 = Mathf.FloorToInt(px0 - rad - 1), x1 = Mathf.CeilToInt(px0 + rad + 1);
        int y0 = Mathf.FloorToInt(py0 - rad - 1), y1 = Mathf.CeilToInt(py0 + rad + 1);
        for (int y = y0; y <= y1; y++)
        {
            if (y < 0 || y >= size) continue;
            for (int x = x0; x <= x1; x++)
            {
                if (x < 0 || x >= size) continue;
                float dx = x - c, dy = c - y;
                if (dx * dx + dy * dy > R2) continue;
                float ex = x - px0, ey = y - py0;
                float cov = rad + 0.5f - Mathf.Sqrt(ex * ex + ey * ey);
                if (cov <= 0f) continue;
                Blend(x, y, col, (cov > 1f ? 1f : cov) * a);
            }
        }
    }

    void SegI(float x0, float y0, float x1, float y1, float w, Color32 col, bool dash)
    {
        float dx = x1 - x0, dy = y1 - y0;
        float len = Mathf.Sqrt(dx * dx + dy * dy);
        int n = Mathf.Max(1, Mathf.CeilToInt(len * 1.5f));
        float rad = w * 0.5f;
        for (int i = 0; i <= n; i++)
        {
            if (dash && (i / 3) % 2 == 1) continue;
            float t = (float)i / n;
            Stamp(x0 + dx * t, y0 + dy * t, rad, col, 1f);
        }
    }

    void SegA(float u0, float v0, float u1, float v1, float w, Color32 col, bool dash)
    {
        SegI(u0 * cosR - v0 * sinR, u0 * sinR + v0 * cosR,
             u1 * cosR - v1 * sinR, u1 * sinR + v1 * cosR, w, col, dash);
    }

    void TriI(float ax, float ay, float bx, float by, float cx2, float cy2, Color32 col)
    {
        float pax = c + ax, pay = c - ay, pbx = c + bx, pby = c - by, pcx = c + cx2, pcy = c - cy2;
        int minX = Mathf.Max(0, Mathf.FloorToInt(Mathf.Min(pax, Mathf.Min(pbx, pcx))));
        int maxX = Mathf.Min(size - 1, Mathf.CeilToInt(Mathf.Max(pax, Mathf.Max(pbx, pcx))));
        int minY = Mathf.Max(0, Mathf.FloorToInt(Mathf.Min(pay, Mathf.Min(pby, pcy))));
        int maxY = Mathf.Min(size - 1, Mathf.CeilToInt(Mathf.Max(pay, Mathf.Max(pby, pcy))));
        float den = (pby - pcy) * (pax - pcx) + (pcx - pbx) * (pay - pcy);
        if (Mathf.Abs(den) < 1e-4f) return;
        for (int y = minY; y <= maxY; y++)
            for (int x = minX; x <= maxX; x++)
            {
                float dx = x - c, dy = c - y;
                if (dx * dx + dy * dy > R2) continue;
                float l1 = ((pby - pcy) * (x - pcx) + (pcx - pbx) * (y - pcy)) / den;
                float l2 = ((pcy - pay) * (x - pcx) + (pax - pcx) * (y - pcy)) / den;
                float l3 = 1f - l1 - l2;
                if (l1 < -0.06f || l2 < -0.06f || l3 < -0.06f) continue;
                float a = Mathf.Clamp01(Mathf.Min(l1, Mathf.Min(l2, l3)) * 6f + 0.5f);
                Blend(x, y, col, a);
            }
    }

    void TextA(string s, float u, float v, float sc, Color32 col, bool rightAlign)
    {
        float w = s.Length * 6f * sc - sc;
        float u0 = rightAlign ? u - w : u;
        float vTop = v + 3.5f * sc;
        for (int gi = 0; gi < s.Length; gi++)
        {
            char ch = s[gi];
            if (ch >= '0' && ch <= '9')
            {
                var g = Glyph[ch - '0'];
                for (int r = 0; r < 7; r++)
                {
                    byte bits = g[r];
                    if (bits == 0) continue;
                    for (int k = 0; k < 5; k++)
                    {
                        if ((bits & (1 << (4 - k))) == 0) continue;
                        float gu = u0 + k * sc, gv = vTop - r * sc;
                        Stamp(gu * cosR - gv * sinR, gu * sinR + gv * cosR, sc * 0.56f, col, 1f);
                    }
                }
            }
            u0 += 6f * sc;
        }
    }

    // ---------- элементы прибора ----------
    void DrawLadder(float pitch)
    {
        Color32 ink = Ink;
        float ppd = PixelsPerDegree;
        int minor = Mathf.Max(1, MinorStep), major = Mathf.Max(minor, MajorStep);
        for (int p = -90; p <= 90; p += minor)
        {
            if (p == 0) continue;
            float v = (p - pitch) * ppd;
            if (v > R + 30f || v < -R - 30f) continue;
            bool isMajor = p % major == 0;
            float hw = isMajor ? R * 0.27f : R * 0.12f;
            SegA(-hw, v, hw, v, isMajor ? 2.2f : 1.5f, ink, p < 0);
            if (!isMajor) continue;
            float cap = p > 0 ? -6f : 6f;
            SegA(-hw, v, -hw, v + cap, 2f, ink, false);
            SegA(hw, v, hw, v + cap, 2f, ink, false);
            if (Mathf.Abs(v) < R - 6f)
            {
                string t = Mathf.Abs(p).ToString();
                TextA(t, -hw - 6f, v, 2f, ink, true);
                TextA(t, hw + 6f, v, 2f, ink, false);
            }
        }
    }

    void DrawRollScale(float roll, float slip)
    {
        Color32 ink = Ink;
        int[] angs = { 10, -10, 20, -20, 30, -30, 45, -45, 60, -60 };
        foreach (int a in angs)
        {
            float rad = a * Mathf.Deg2Rad;
            float sx = Mathf.Sin(rad), cy = Mathf.Cos(rad);
            bool big = Mathf.Abs(a) == 30 || Mathf.Abs(a) == 60;
            float len = big ? 13f : (Mathf.Abs(a) == 45 ? 11f : 7f);
            SegI(sx * (R - 2f), cy * (R - 2f), sx * (R - 2f - len), cy * (R - 2f - len),
                 Mathf.Abs(a) == 45 ? 3.2f : 2f, ink, false);
        }

        TriI(0f, R - 15f, -6.5f, R - 2.5f, 6.5f, R - 2.5f, ink);

        float pr = roll * Mathf.Deg2Rad;
        float sx2 = Mathf.Sin(pr), cy2 = Mathf.Cos(pr);
        float tx = cy2, ty = -sx2;
        float rTip = R - 16f, rBase = R - 26f;
        TriI(sx2 * rTip, cy2 * rTip,
             sx2 * rBase + tx * 6.5f, cy2 * rBase + ty * 6.5f,
             sx2 * rBase - tx * 6.5f, cy2 * rBase - ty * 6.5f, Symbol);

        if (!SlipBall) return;
        float rb = R - 33f;
        SegI(sx2 * (rb + 4f) + tx * 5.5f, cy2 * (rb + 4f) + ty * 5.5f,
             sx2 * (rb - 4f) + tx * 5.5f, cy2 * (rb - 4f) + ty * 5.5f, 1.8f, ink, false);
        SegI(sx2 * (rb + 4f) - tx * 5.5f, cy2 * (rb + 4f) - ty * 5.5f,
             sx2 * (rb - 4f) - tx * 5.5f, cy2 * (rb - 4f) - ty * 5.5f, 1.8f, ink, false);
        float off = Mathf.Clamp(slip, -1f, 1f) * 9f;
        Stamp(sx2 * rb + tx * off, cy2 * rb + ty * off, 3.4f, Symbol, 1f);
    }

    void DrawSymbol()
    {
        Color32 shadow = new Color32(12, 16, 20, 255);
        Color32 sym = Symbol;
        float w = R * 0.36f, inner = R * 0.13f;
        SegI(-w, 0, -inner, 0, 5.5f, shadow, false);
        SegI(-inner, 0, -inner, -7f, 5.5f, shadow, false);
        SegI(inner, 0, w, 0, 5.5f, shadow, false);
        SegI(inner, 0, inner, -7f, 5.5f, shadow, false);
        SegI(-w, 0, -inner, 0, 3.4f, sym, false);
        SegI(-inner, 0, -inner, -7f, 3.4f, sym, false);
        SegI(inner, 0, w, 0, 3.4f, sym, false);
        SegI(inner, 0, inner, -7f, 3.4f, sym, false);
        Stamp(0, 0, 3.6f, shadow, 1f);
        Stamp(0, 0, 2.4f, sym, 1f);
    }

    public void Dispose()
    {
        if (tex != null) Object.Destroy(tex);
        tex = null; px = null;
    }
}
```

---

## Патчи `SimHud.cs`

### 1. Поля — добавь сразу после `public ChaseCamera cam;`

```csharp
    [Header("Авиагоризонт (ADI)")]
    [Tooltip("Разрешение текстуры прибора")] [Range(160, 512)] public int adiResolution = 320;
    [Tooltip("Частота перерисовки прибора, Гц")] [Range(10, 60)] public int adiUpdateHz = 40;
    [Tooltip("Пикселей на градус тангажа")] [Range(2f, 6f)] public float adiPixelsPerDegree = 3.5f;
    public bool adiSlipBall = true;
    public bool adiGlass = true;
    public bool adiRollScale = true;
    public Color adiSkyTop = new Color(0.055f, 0.30f, 0.62f);
    public Color adiSkyHorizon = new Color(0.33f, 0.63f, 0.90f);
    public Color adiGroundHorizon = new Color(0.55f, 0.40f, 0.21f);
    public Color adiGroundBottom = new Color(0.20f, 0.13f, 0.06f);
    public Color adiSymbol = new Color(1f, 0.84f, 0.08f);

    AttitudeIndicator adi;
    Vector3 prevVel; bool velInit; float slipSmooth;
```

### 2. Метод `Start()` целиком

```csharp
    void Start()
    {
        if (fc == null) fc = FindObjectOfType<FlightController>();
        if (ph == null) ph = FindObjectOfType<QuadPhysics>();
        if (bat == null) bat = FindObjectOfType<BatteryModel>();
        if (wind == null) wind = FindObjectOfType<WindSystem>();
        if (inp == null) inp = FindObjectOfType<DroneInputManager>();
        if (udp == null) udp = FindObjectOfType<UdpLink>();
        if (cam == null) cam = FindObjectOfType<ChaseCamera>();

        white = new Texture2D(1, 1); white.SetPixel(0, 0, Color.white); white.Apply();
        adi = new AttitudeIndicator();
        enabled = fc != null;
    }
```

### 3. Метод `Update()` целиком

```csharp
    void Update()
    {
        fps = Mathf.Lerp(fps, 1f / Mathf.Max(0.0001f, Time.unscaledDeltaTime), 0.05f);
        if (Hotkeys.Down(SimKey.Help)) wHelp = !wHelp;
        if (Hotkeys.Down(SimKey.Calib)) { wCal = !wCal; if (wCal && inp != null && inp.Wizard == CalWizardState.Off) inp.BeginCal(); }
        if (Hotkeys.Down(SimKey.Env)) wEnv = !wEnv;
        if (Hotkeys.Down(SimKey.Inputs)) wInp = !wInp;
        if (Hotkeys.Down(SimKey.Tune)) wTune = !wTune;
        if (Hotkeys.Down(SimKey.HudToggle)) showHud = !showHud;

        // скольжение для шарика указателя скольжения (боковая перегрузка)
        if (ph != null && ph.Rb != null)
        {
            Vector3 v = ph.Rb.velocity;
            float dt = Time.deltaTime;
            if (velInit && dt > 0.0001f)
            {
                Vector3 acc = (v - prevVel) / dt - Physics.gravity;
                float lat = Vector3.Dot(acc, ph.transform.right) / Mathf.Max(1f, Mathf.Abs(Physics.gravity.y));
                slipSmooth = Mathf.Lerp(slipSmooth, Mathf.Clamp(lat, -1f, 1f), 1f - Mathf.Exp(-6f * dt));
            }
            prevVel = v; velInit = true;
        }
    }
```

### 4. Метод `DrawADI(Rect r)` целиком + новый `DrawCompassTape`

```csharp
    void DrawADI(Rect r)
    {
        Panel(new Rect(r.x - 4, r.y - 44, r.width + 8, r.height + 66), null);
        DrawCompassTape(new Rect(r.x + 4, r.y - 38, r.width - 8, 26));

        if (adi == null) adi = new AttitudeIndicator();
        adi.Size = adiResolution;
        adi.UpdateHz = adiUpdateHz;
        adi.PixelsPerDegree = adiPixelsPerDegree;
        adi.SlipBall = adiSlipBall;
        adi.GlassSheen = adiGlass;
        adi.RollScale = adiRollScale;
        adi.SkyTop = adiSkyTop;
        adi.SkyHorizon = adiSkyHorizon;
        adi.GroundHorizon = adiGroundHorizon;
        adi.GroundBottom = adiGroundBottom;
        adi.Symbol = adiSymbol;
        adi.Accent = Cacc;
        adi.Draw(r, fc.AttRoll, fc.AttPitchUp, slipSmooth);

        // цифровые读出 в свободных углах квадратной подложки
        Rect lr = new Rect(r.x + 4, r.yMax - 26, 84, 20);
        Rect rr = new Rect(r.xMax - 88, r.yMax - 26, 84, 20);
        Fill(lr, new Color(0f, 0f, 0f, 0.45f));
        Fill(rr, new Color(0f, 0f, 0f, 0.45f));
        Txt(lr, $"R {fc.AttRoll:+0;-0;0}°", 13, Color.white, TextAnchor.MiddleCenter, true);
        Txt(rr, $"P {fc.AttPitchUp:+0;-0;0}°", 13, Color.white, TextAnchor.MiddleCenter, true);

        // дом
        float hy = r.yMax + 4;
        Txt(new Rect(r.x, hy, r.width, 18), $"HOME {fc.HomeDist:0} m", 13, Cacc, TextAnchor.MiddleCenter, true);
        Vector2 piv = new Vector2(r.x + r.width * 0.5f + 58, hy + 9);
        var m = GUI.matrix;
        GUIUtility.RotateAroundPivot(fc.HomeBearing - fc.Heading, piv);
        Fill(new Rect(piv.x - 1.5f, piv.y - 7, 3, 9), Cacc);
        Fill(new Rect(piv.x - 5, piv.y - 3, 10, 3), Cacc);
        GUI.matrix = m;
    }

    void DrawCompassTape(Rect cr)
    {
        Fill(cr, new Color(0f, 0f, 0f, 0.42f));
        Fill(new Rect(cr.x, cr.y, cr.width, 1), new Color(Cacc.r, Cacc.g, Cacc.b, 0.35f));
        GUI.BeginGroup(cr);
        float hdg = fc.Heading;
        int baseDeg = Mathf.RoundToInt(hdg / 15f) * 15;
        for (int d = -75; d <= 75; d += 15)
        {
            int deg = baseDeg + d;
            float x = cr.width * 0.5f + (deg - hdg) * 3.1f;
            if (x < 8 || x > cr.width - 8) continue;
            int mark = ((deg % 360) + 360) % 360;
            bool cardinal = mark % 90 == 0;
            Fill(new Rect(x - 1, cr.height - 8, 2, cardinal ? 8 : 5), cardinal ? Cacc : Cdim);
            string s = mark == 0 ? "N" : mark == 90 ? "E" : mark == 180 ? "S" : mark == 270 ? "W" : mark.ToString();
            Txt(new Rect(x - 20, 1, 40, 15), s, cardinal ? 13 : 11, cardinal ? Cacc : Cdim, TextAnchor.MiddleCenter, cardinal);
        }
        Fill(new Rect(cr.width * 0.5f - 1, cr.height - 11, 2, 11), Cacc);
        GUI.EndGroup();

        Rect box = new Rect(cr.x + cr.width * 0.5f - 26, cr.y - 17, 52, 17);
        Fill(box, Cbg);
        Frame(box, new Color(Cacc.r, Cacc.g, Cacc.b, 0.5f));
        Txt(box, $"{Mathf.Repeat(hdg, 360f):000}°", 12, Cacc, TextAnchor.MiddleCenter, true);
    }

    void Frame(Rect r, Color c)
    {
        Fill(new Rect(r.x, r.y, r.width, 1), c);
        Fill(new Rect(r.x, r.yMax - 1, r.width, 1), c);
        Fill(new Rect(r.x, r.y, 1, r.height), c);
        Fill(new Rect(r.xMax - 1, r.y, 1, r.height), c);
    }
```

### 5. Метод `DrawTapes(Rect adi)` целиком

```csharp
    void DrawTapes(Rect a)
    {
        // ---- скорость слева ----
        Rect sp = new Rect(a.x - 58, a.y + 14, 52, a.height - 28);
        Panel(sp, null);
        Txt(new Rect(sp.x, sp.y + 3, sp.width, 15), "GS", 11, Cdim, TextAnchor.MiddleCenter, true);
        Rect sg = new Rect(sp.x, sp.y + 19, sp.width, sp.height - 22);
        GUI.BeginGroup(sg);
        float gs = fc.Gs;
        for (int v = Mathf.FloorToInt(gs) - 10; v <= gs + 10; v++)
        {
            if (v < 0 || v % 2 != 0) continue;
            float y = sg.height * 0.5f - (v - gs) * 13f;
            if (y < 4 || y > sg.height - 4) continue;
            Fill(new Rect(sg.width - 9, y, 7, 1), Cdim);
            Txt(new Rect(0, y - 8, sg.width - 12, 16), v.ToString(), 10, Cdim, TextAnchor.MiddleRight);
        }
        GUI.EndGroup();
        Rect sv = new Rect(sp.x - 2, sp.y + sp.height * 0.5f - 11, sp.width + 4, 22);
        Fill(sv, Cbg); Frame(sv, new Color(Cacc.r, Cacc.g, Cacc.b, 0.55f));
        Txt(sv, gs.ToString("0.0"), 14, Cacc, TextAnchor.MiddleCenter, true);
        Txt(new Rect(sp.x - 4, sp.yMax + 4, sp.width + 24, 16), $"AS {ph.Airspeed:0.0}", 11, Cdim);

        // ---- высота справа ----
        Rect al = new Rect(a.xMax + 6, a.y + 14, 58, a.height - 28);
        Panel(al, null);
        Txt(new Rect(al.x, al.y + 3, al.width, 15), "ALT", 11, Cdim, TextAnchor.MiddleCenter, true);
        Rect ag = new Rect(al.x, al.y + 19, al.width, al.height - 22);
        GUI.BeginGroup(ag);
        float alt = fc.AltAgl;
        for (int v = Mathf.FloorToInt(alt) - 14; v <= alt + 14; v++)
        {
            if (v < 0 || v % 5 != 0) continue;
            float y = ag.height * 0.5f - (v - alt) * 5.5f;
            if (y < 4 || y > ag.height - 4) continue;
            Fill(new Rect(2, y, 7, 1), Cdim);
            Txt(new Rect(11, y - 8, al.width - 12, 16), v.ToString(), 10, Cdim);
        }
        GUI.EndGroup();
        Rect av = new Rect(al.x - 2, al.y + al.height * 0.5f - 11, al.width + 4, 22);
        Fill(av, Cbg); Frame(av, new Color(Cacc.r, Cacc.g, Cacc.b, 0.55f));
        Txt(av, alt.ToString("0.0"), 14, Cacc, TextAnchor.MiddleCenter, true);

        // ---- вертикальная скорость ----
        Rect vb = new Rect(al.xMax + 5, al.y + 10, 9, al.height - 20);
        Fill(vb, new Color(1f, 1f, 1f, 0.06f));
        Fill(new Rect(vb.x, vb.y + vb.height * 0.5f - 1, vb.width, 2), new Color(1f, 1f, 1f, 0.25f));
        float vs = Mathf.Clamp(fc.Vs, -6f, 6f);
        float h = Mathf.Abs(vs) / 6f * (vb.height * 0.5f);
        Color vc = Mathf.Abs(fc.Vs) > 4f ? Cwarn : Cok;
        Fill(vs >= 0 ? new Rect(vb.x + 1, vb.y + vb.height * 0.5f - h, vb.width - 2, h)
                     : new Rect(vb.x + 1, vb.y + vb.height * 0.5f, vb.width - 2, h), vc);
        Txt(new Rect(al.x - 4, al.yMax + 4, al.width + 22, 16), $"VS {fc.Vs:+0.0;-0.0;0.0}", 11, vc);
    }
```

### 6. Добавь метод `OnDestroy()` в класс

```csharp
    void OnDestroy()
    {
        if (adi != null) adi.Dispose();
        if (white != null) Destroy(white);
    }
```

### USER
я все сделал но возникла ошибки помоги исправить:

Assets\Scripts\AttitudeIndicator.cs(74,26): error CS0104: 'Object' is an ambiguous reference between 'UnityEngine.Object' and 'object'

Assets\Scripts\AttitudeIndicator.cs(341,26): error CS0104: 'Object' is an ambiguous reference between 'UnityEngine.Object' and 'object'



### ASSISTANT
Тот же конфликт имён, что и раньше: IDE добавила в файл `using System;`, и `Object` стал неоднозначным (`UnityEngine.Object` vs `System.Object`). В файле два вызова `Object.Destroy(tex)` — в `Alloc()` и в `Dispose()`.

## Исправление (шапка файла)

Приведи начало `Assets/Scripts/AttitudeIndicator.cs` к такому виду:

```csharp
using UnityEngine;
using Object = UnityEngine.Object;   // снимает конфликт UnityEngine.Object / System.Object
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;
```

Alias имеет приоритет над обычными `using`, поэтому даже если `using System;` останется в файле — ошибки уйдут. Больше ничего менять не нужно.

## Альтернатива

В этом файле `using System;` вообще не нужен — `System.IDisposable` уже написан полностью квалифицированно. Так что можно просто **удалить строку `using System;`** из шапки. Но вариант с alias надёжнее: IDE не сломает файл повторным автоимпортом.

## Если хочешь точечно, без правки шапки

Замени обе строки (74 и 341):

```csharp
        if (tex != null) Object.Destroy(tex);
```

на:

```csharp
        if (tex != null) UnityEngine.Object.Destroy(tex);
```

Любой из трёх вариантов закрывает обе ошибки — прибор нарисуется круговым, с корректной шкалой тангажа, шкалой крена и шариком скольжения.

### USER
Ура все отлично работает но можно ли перевернуть цвета в авиагоризонте получилось что небо снизу а земля сверху хотя квадрокоптер просто горизонтально стоит на стартовом столе посмотри скриншет чтобы понять

### ASSISTANT