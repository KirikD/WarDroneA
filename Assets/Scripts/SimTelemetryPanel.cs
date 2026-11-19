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
    GUISkin skinCopy;
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
            stretchHeight = true
        };
        stVBar.normal.background = Solid(new Color(1f, 1f, 1f, 0.05f));
        stVBar.hover.background = Solid(new Color(1f, 1f, 1f, 0.08f));
       
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
        scrollTraces = GUI.BeginScrollView(view, scrollTraces, content, false, true, stHBar, stVBar);

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
        scrollRotors = GUI.BeginScrollView(view, scrollRotors, content, false, true, stHBar, stVBar);

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
        scrollLog = GUI.BeginScrollView(view, scrollLog, content, false, true, stHBar, stVBar);
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
        if (skinCopy != null) Destroy(skinCopy);
    }
}