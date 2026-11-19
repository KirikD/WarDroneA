using UnityEngine;
using Object = UnityEngine.Object;   // снимает конфликт UnityEngine.Object / System.Object
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;

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
        if (tex != null) UnityEngine.Object.Destroy(tex);
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