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