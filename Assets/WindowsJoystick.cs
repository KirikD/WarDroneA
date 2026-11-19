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