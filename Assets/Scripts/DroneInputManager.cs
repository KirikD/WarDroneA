using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using UnityEngine.InputSystem.HID;
#endif

public enum JoystickBackend { Windows, InputSystem }
public enum ThrottleMixMode { Priority, Max }
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
    [Tooltip("Windows — прямое чтение winmm: 6 сырых осей X/Y/Z/R/U/V без нормализации Unity (рекомендуется для SM001). InputSystem — пакет Unity.")]
    public JoystickBackend backend = JoystickBackend.Windows;
    public int deviceIndex = 0;
    public CalibrationData cal = new CalibrationData();
    [Tooltip("Печатать список осей/кнопок при подключении устройства в Console")]
    public bool logDeviceOnConnect = true;

    readonly WindowsJoystickReader winJoy = new WindowsJoystickReader();
    bool UseWin => backend == JoystickBackend.Windows && WindowsJoystickReader.Supported;
    static readonly string[] WinAxisLabels = { "X", "Y", "Z", "R", "U", "V", "POV X", "POV Y" };
    [Header("Газ: кто главный")]
    [Tooltip("Priority — газ у источника, который последним двигал ручку газа (клавиатура и джойстик не мешают друг другу). Max — берётся максимальное значение.")]
    public ThrottleMixMode throttleMix = ThrottleMixMode.Priority;

    float lastKbThrT = -1f, lastJoyThrT = -1f, lastUdpThrT = -1f;
    float prevJoyThr, prevUdpThr;
    int thrOwner = 0;
    readonly bool[] prevButtons = new bool[32];
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
    public bool JoyConnected => UseWin ? winJoy.Current != null : joyDevice != null;
    public string DeviceLayout => UseWin ? "winmm (raw)" : (joyDevice != null ? joyDevice.layout : "none");
    public CalWizardState Wizard { get; private set; } = CalWizardState.Off;

    float kbThr; float[] kbAxes = new float[4];
    float lastKbActive, lastJoyActive, lastUdpActive, lastJoyDominant;
    bool udpEverFresh, joyEverActive;
    bool armFlag, disarmFlag, resetFlag, resetFullFlag, modeFlag, camFlag;
    float refreshTimer;
#if ENABLE_INPUT_SYSTEM
    InputDevice joyDevice;
    class AxisEntry { public InputControl ctrl; public int comp; public string label; } // comp: -1 scalar, 0=x, 1=y
    readonly List<AxisEntry> axisEntries = new List<AxisEntry>();
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

        float nowT = Time.realtimeSinceStartup;
        if (Mathf.Abs(JoyCal[2] - prevJoyThr) > 0.01f) { prevJoyThr = JoyCal[2]; lastJoyThrT = nowT; }
        float ut = udp != null ? udp.Control.throttle : -1f;
        if (Mathf.Abs(ut - prevUdpThr) > 0.01f) { prevUdpThr = ut; lastUdpThrT = nowT; }
        PollActionButtons();

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
    // ---------- устройства ----------
    void RefreshDevices()
    {
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
        foreach (var e in stick) axisEntries.Add(e);
        foreach (var e in other) axisEntries.Add(e);
        foreach (var e in extra) axisEntries.Add(e);
        foreach (var e in axisEntries) AxisLabels.Add(e.label);

        if (logDeviceOnConnect) DumpAllControls();
#endif
    }

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

    public void CycleDevice() { deviceIndex++; refreshTimer = 0; }
    public float RawAxis(int i) => i >= 0 && i < AxisValues.Count ? AxisValues[i] : 0f;
    public float RawChannel(int ch)
    {
        int bi = cal.buttonIndex[ch];
        if (bi >= 0) return bi < ButtonValues.Count && ButtonValues[bi] ? 1f : -1f;
        return RawAxis(cal.sourceIndex[ch]);
    }
    public void ToggleBackend()
    {
        backend = backend == JoystickBackend.Windows ? JoystickBackend.InputSystem : JoystickBackend.Windows;
        refreshTimer = 0f;
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
    {        // в AutoMapSticksMode2():
        if (UseWin)
        {
            for (int i = 0; i < 6; i++) { cal.sourceIndex[i] = i; cal.buttonIndex[i] = -1; }
            SaveCal();
            UnityEngine.Debug.Log("[winmm] Каналы = X,Y,Z,R,U,V. Проверь в F4 и переназначь кликом R/P/T/Y.");
            return;
        }
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
        if (Hotkeys.Held(SimKey.ThrUp)) { kbThr += kbThrottleSpeed * fine * dt; lastKbThrT = Time.realtimeSinceStartup; }
        if (Hotkeys.Held(SimKey.ThrDn)) { kbThr -= kbThrottleSpeed * fine * dt; lastKbThrT = Time.realtimeSinceStartup; }
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