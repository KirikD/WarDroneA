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
    [Header("Кнопки действий (индекс кнопки, -1 = не назначено)")]
    public int armButton = -1;
    public int disarmButton = -1;
    public int resetButton = -1;
    public int modeButton = -1;
    public int cameraButton = -1;
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