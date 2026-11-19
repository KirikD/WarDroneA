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
        var br = d.AddComponent<DroneBreakup>(); br.physics = ph;
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

        // панель телеметрии — на тот же объект, что и HUD (или на отдельный)
        if (UnityEngine.Object.FindObjectOfType<SimTelemetryPanel>() == null)
        {
            var hud = UnityEngine.Object.FindObjectOfType<SimHud>();
            GameObject host = hud != null ? hud.gameObject : new GameObject("Telemetry");
            if (hud == null) ctx.register?.Invoke(host);
            host.AddComponent<SimTelemetryPanel>();
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
        var br = UnityEngine.Object.FindObjectOfType<DroneBreakup>();
        if (br != null) br.physics = ph;
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