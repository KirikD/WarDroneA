using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(100)]
public class SimHud : MonoBehaviour
{
    [Header("Ссылки (заполняются в редакторе, иначе найдутся сами)")]
    public FlightController fc;
    public QuadPhysics ph;
    public BatteryModel bat;
    public WindSystem wind;
    public DroneInputManager inp;
    public UdpLink udp;
    public ChaseCamera cam;

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

    Texture2D white;
    readonly Dictionary<int, GUIStyle> labels = new Dictionary<int, GUIStyle>();
    GUIStyle winStyle;
    bool showHud = true;
    bool wHelp, wCal, wEnv, wInp, wTune;
    Rect rHelp = new Rect(420, 120, 640, 560);
    Rect rCal = new Rect(460, 160, 560, 460);
    Rect rEnv = new Rect(980, 140, 420, 520);
    Rect rInp = new Rect(300, 140, 780, 640);
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
            Vector3 v = ph.Rb.linearVelocity;
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
    void OnDestroy()
    {
        if (adi != null) adi.Dispose();
        if (white != null) Destroy(white);
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
        {
            Txt(new Rect(W / 2 - 360, 770, 720, 24),
                "ARM: Space / кнопка джойстика / стики: газ вниз + YAW вправо (0.7 с)",
                15, Cdim, TextAnchor.MiddleCenter);
            bool ok = string.IsNullOrEmpty(fc.ArmBlockReason);
            Txt(new Rect(W / 2 - 360, 794, 720, 24),
                ok ? "готов к арму" : fc.ArmBlockReason,
                16, ok ? Cok : Cwarn, TextAnchor.MiddleCenter, true);
        }
    }

    // ---------- окна ----------
float Slider(string name, float v, float min, float max)
{
    GUILayout.BeginHorizontal();
    GUILayout.Label(name, GUILayout.Width(170));
    v = GUILayout.HorizontalSlider(v, min, max);   // было GUILayout.Slider — такого метода не существует
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
            GUILayout.Box("", GUILayout.Width(Mathf.Clamp(Mathf.Abs(inp.AxisValues[i]) * 60f, 1f, 60f)), GUILayout.Height(10));
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
        GUILayout.Label($"Джойстик: {(inp.JoyConnected ? "подключен" : "нет")}  layout={inp.DeviceLayout}");
        if (GUILayout.Button("Сменить устройство")) inp.CycleDevice();
        if (GUILayout.Button("Авто-назначить стики (Mode 2)")) inp.AutoMapSticksMode2();
        GUILayout.Space(4);
        GUILayout.Label("Сырые оси: двинь стик → строка меняется → нажми канал справа (R/P/T/Y):");
        int axisCount = Mathf.Min(inp.AxisLabels.Count, inp.AxisValues.Count);
        for (int i = 0; i < axisCount && i < 16; i++)
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
        GUILayout.Label($"Кнопок: {inp.ButtonValues.Count} (двигай стики/тумблеры — смотри, что нажимается):");
        for (int i = 0; i < inp.ButtonValues.Count && i < 32; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label($"[{i}] {(inp.ButtonValues[i] ? "ON" : "off")}", GUILayout.Width(90));
            if (GUILayout.Button("→AUX1", GUILayout.Width(60))) inp.AssignButtonToChannel(i, 4);
            if (GUILayout.Button("→AUX2", GUILayout.Width(60))) inp.AssignButtonToChannel(i, 5);
            GUILayout.EndHorizontal();
            if (GUILayout.Button("ARM", GUILayout.Width(40))) inp.AssignAction("arm", i);
            if (GUILayout.Button("DIS", GUILayout.Width(36))) inp.AssignAction("disarm", i);
            if (GUILayout.Button("RST", GUILayout.Width(36))) inp.AssignAction("reset", i);
            if (GUILayout.Button("MODE", GUILayout.Width(46))) inp.AssignAction("mode", i);
            if (GUILayout.Button("CAM", GUILayout.Width(40))) inp.AssignAction("camera", i);
        }
        if (GUILayout.Button("Показать ВСЕ контролы устройства в Console")) inp.DumpAllControls();
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