using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

// Алиасы: защита от глобальных using проекта (System, System.Net.Mime и т.п.)
using Random = UnityEngine.Random;
using Application = UnityEngine.Application;
using Object = UnityEngine.Object;
using System.Diagnostics;

// ============================================================
//  Квадрокоптер: генерация в редакторе, полёт по точкам,
//  ветер, вращение винтов, разрушение на фрагменты.
//  Упрощённая физика: скорость задаётся напрямую, наклон — визуальный.
// ============================================================
[RequireComponent(typeof(Rigidbody), typeof(BoxCollider))]
public class QuadcopterDrone : MonoBehaviour
{
    [Header("Маршрут (летает по кругу)")]
    public Transform[] waypoints;
    public float flightSpeed = 3f;        // м/с
    public float arriveRadius = 0.4f;     // радиус засчитывания точки

    [Header("Манёвренность")]
    public float acceleration = 2.5f;     // резвость разгона/торможения
    public float yawSpeed = 120f;         // град/с
    public float maxTilt = 25f;           // максимальный наклон корпуса

    [Header("Ветер")]
    public float windStrength = 0.6f;     // сила порывов, м/с
    public float gustFrequency = 0.5f;    // частота порывов
    public Vector3 windDirection = Vector3.right; // постоянный снос

    [Header("Винты")]
    public float propSpinSpeed = 2200f;   // град/с

    [Header("Разрушение")]
    public string groundTag = "Ground";   // тег земли
    public float explosionForce = 4f;     // разлёт обломков
    public float debrisLifetime = 10f;    // через сколько удалить обломки (0 = не удалять)
    public float spawnProtection = 1f;   // сек. игнорирования контактов после старта
    float ignoreCollisionsUntil;
    [SerializeField] List<DronePart> parts = new();
    [SerializeField] List<DroneProp> props = new();
    [SerializeField] Transform visual;

    Rigidbody rb; BoxCollider rootCol;
    int wpIndex; float yaw, spin, seed;
    Vector3 smoothVel, velDamp, windVel, hoverPos;
    bool crashed;
    public bool IsCrashed => crashed;
    // ---------------- Жизненный цикл ----------------
    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rootCol = GetComponent<BoxCollider>();
        rb.isKinematic = false;
        rb.useGravity = false;
        rb.linearVelocity = Vector3.zero;
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        crashed = false;
    }
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rootCol = GetComponent<BoxCollider>();

        // сброс рантайм-состояния
        crashed = false; spin = 0f; wpIndex = 0;
        smoothVel = velDamp = windVel = Vector3.zero;
        ignoreCollisionsUntil = Time.time + spawnProtection;

    // восстанавливаем ссылки из иерархии, а не доверяем сериализованным спискам
    RebuildLinks();
        if (parts.Count == 0 || props.Count != 4) Generate(); // некомплект — пересоздаём начисто

        // принудительно приводим физику в рабочее состояние
        rb.isKinematic = false;
        rb.useGravity = false;
        rb.linearVelocity = Vector3.zero;
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rootCol.enabled = true;
        rootCol.size = new Vector3(1.25f, 0.34f, 1.25f);
        rootCol.center = new Vector3(0, 0.04f, 0);

        foreach (var p in parts)
            if (p) Physics.IgnoreCollision(rootCol, p.GetComponent<BoxCollider>());

        seed = Random.value * 1000f;
        hoverPos = transform.position;

        UnityEngine.Debug.Log($"[Quadcopter] деталей: {parts.Count}, винтов: {props.Count}, " +
                  $"точек: {(waypoints == null ? 0 : waypoints.Length)}, visual: {visual != null}");
        // точки не должны быть детьми дрона: иначе цель убегает вместе с ним
        if (waypoints != null)
            foreach (var w in waypoints)
                if (w != null && w.IsChildOf(transform))
                {
                    
                    w.SetParent(null, true);
                }
    }
    void FixedUpdate()
    {
        if (crashed || visual == null) return;
        float dt = Time.fixedDeltaTime;

        // --- выбор цели ---
        Vector3 target = hoverPos;
        bool hasRoute = waypoints != null && waypoints.Length > 0;
        if (hasRoute)
        {
            int guard = 0;
            while (waypoints[wpIndex] == null && guard++ < waypoints.Length)
                wpIndex = (wpIndex + 1) % waypoints.Length;

            if (waypoints[wpIndex] != null)
            {
                target = waypoints[wpIndex].position;
                if ((target - transform.position).magnitude < arriveRadius)
                {
                    wpIndex = (wpIndex + 1) % waypoints.Length; // по кругу
                    if (waypoints[wpIndex] != null) target = waypoints[wpIndex].position;
                }
            }
        }

        Vector3 to = target - transform.position;
        Vector3 desired = hasRoute
            ? to.normalized * flightSpeed
            : Vector3.ClampMagnitude(to * 2f, flightSpeed); // удержание позиции без маршрута

        smoothVel = Vector3.SmoothDamp(smoothVel, desired, ref velDamp,
            1f / Mathf.Max(0.1f, acceleration), flightSpeed * 2f, dt);

        // --- ветер: шум Перлина + постоянный снос ---
        float t = Time.time * gustFrequency + seed;
        Vector3 gust = new Vector3(
            Mathf.PerlinNoise(t, seed) - 0.5f,
            (Mathf.PerlinNoise(seed, t) - 0.5f) * 0.6f,
            Mathf.PerlinNoise(t + seed, t) - 0.5f) * 2f * windStrength;
        gust += windDirection.normalized * windStrength * 0.3f;
        windVel = Vector3.Lerp(windVel, gust, dt * 2f);

        rb.linearVelocity = smoothVel + windVel;

        // --- рыскание носом по направлению движения ---
        Vector3 hv = new Vector3(smoothVel.x, 0, smoothVel.z);
        if (hv.sqrMagnitude > 0.02f)
        {
            float targetYaw = Mathf.Atan2(hv.x, hv.z) * Mathf.Rad2Deg;
            yaw = Mathf.MoveTowardsAngle(yaw, targetYaw, yawSpeed * dt);
        }
    }

    void Update()
    {
        // --- винты (раскручиваются плавно, при аварии останавливаются) ---
            if (visual == null || crashed) return;
            float dt = Time.deltaTime;
            // ...дальше существующий код наклона без изменений
        
        if (crashed) return;

        // --- визуальный наклон: тангаж вперёд, крен в сторону движения + дрожь от ветра ---
        Vector3 lv = Quaternion.Inverse(Quaternion.Euler(0f, yaw, 0f)) * smoothVel;
        float pitch = Mathf.Clamp(lv.z / Mathf.Max(1f, flightSpeed), -1f, 1f) * maxTilt;
        float roll = -Mathf.Clamp(lv.x / Mathf.Max(1f, flightSpeed), -1f, 1f) * maxTilt;
        float t = Time.time * 1.7f + seed;
        pitch += (Mathf.PerlinNoise(t, seed * 2f) - 0.5f) * windStrength * 10f;
        roll += (Mathf.PerlinNoise(seed * 2f, t) - 0.5f) * windStrength * 10f;

        visual.localRotation = Quaternion.Slerp(visual.localRotation,
            Quaternion.Euler(pitch, yaw, roll), dt * 6f);
    }

    // ---------------- Столкновения и разрушение ----------------

    void OnCollisionEnter(Collision c) => HandleImpact(c.collider);

    public void HandleImpact(Collider other)
    {
        if (crashed || other == null) return;
        if (Time.time < ignoreCollisionsUntil) return; // защита на старте

        var otherDrone = other.GetComponentInParent<QuadcopterDrone>();
        if (otherDrone == null)
        {
            var dp = other.GetComponent<DronePart>();
            if (dp) otherDrone = dp.owner;
        }

        if (otherDrone != null)
        {
            if (otherDrone != this)
            {
                UnityEngine.Debug.Log($"[Quadcopter] авария: столкновение с дроном {otherDrone.name}");
                otherDrone.Crash(); Crash();
            }
            return;
        }
        if (other.gameObject.tag == groundTag)
        {
            UnityEngine.Debug.Log($"[Quadcopter] авария: контакт с землёй {other.name}");
            Crash();
        }
    }
    public void RebuildLinks()
    {
        if (visual == null) visual = transform.Find("Visual");
        if (visual == null)
        {
            visual = new GameObject("Visual").transform;
            visual.SetParent(transform, false);
        }

        // все детали и винты внутри корня, включая неактивные
        var allParts = new List<DronePart>(GetComponentsInChildren<DronePart>(true));
        var allProps = new List<DroneProp>(GetComponentsInChildren<DroneProp>(true));

        // потерянные детали, разбросанные по сцене, но помеченные этим владельцем
        foreach (var p in FindObjectsByType<DronePart>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (p.owner == this && !allParts.Contains(p)) allParts.Add(p);

        // включаем всё и возвращаем потерянные ветки под Visual
        var moved = new HashSet<Transform>();
        foreach (var p in allParts)
        {
            p.owner = this;
            p.gameObject.SetActive(true);
            if (p.transform.IsChildOf(visual)) continue;
            var t = p.transform;
            while (t.parent != null && t.parent != transform && t.parent != visual) t = t.parent;
            if (moved.Add(t)) t.SetParent(visual, true);
        }
        foreach (var pr in allProps)
        {
            pr.owner = this;
            pr.gameObject.SetActive(true);
            if (!pr.transform.IsChildOf(visual)) pr.transform.SetParent(visual, true);
        }

        parts.Clear(); parts.AddRange(allParts);
        props.Clear(); props.AddRange(allProps);

        var rc = GetComponent<BoxCollider>();
        foreach (var p in parts)
            if (p) Physics.IgnoreCollision(rc, p.GetComponent<BoxCollider>());


        UnityEngine.Debug.Log($"[Quadcopter] пересборка: деталей {parts.Count}, винтов {props.Count}");
    }
    public void Crash()
    {
        if (crashed) return;
        crashed = true;
        UnityEngine.Debug.Log("[Quadcopter] разрушение на старте/в полёте!");

        Vector3 v = rb.linearVelocity;
        Vector3 center = transform.position;

        foreach (var p in parts)
        {
            if (!p) continue;
            var tr = p.transform;
            tr.SetParent(null, true); // отсоединяем фрагмент

            var prb = tr.GetComponent<Rigidbody>();
            prb.isKinematic = false;
            prb.useGravity = true;
            prb.constraints = RigidbodyConstraints.None;

            Vector3 dir = ((tr.position - center).normalized + Vector3.up * 0.6f).normalized;
            prb.linearVelocity = v * 0.5f + dir * Random.Range(0.4f, 1f) * explosionForce;
            prb.angularVelocity = Random.insideUnitSphere * 12f;

            if (debrisLifetime > 0f) Destroy(p.gameObject, debrisLifetime);
        }

        rb.linearVelocity = Vector3.zero;
        rb.isKinematic = true;
        rootCol.enabled = false;
        if (debrisLifetime > 0f) Destroy(gameObject, debrisLifetime);
    }

    // ---------------- Генерация модели (как на фото, упрощённо) ----------------

    public void Generate()
    {
        ClearGenerated();
        rb = GetComponent<Rigidbody>();
        rootCol = GetComponent<BoxCollider>();

        visual = new GameObject("Visual").transform;
        visual.SetParent(transform, false);

        var carbon = MakeMat("Carbon", new Color(0.09f, 0.09f, 0.10f));
        var alu = MakeMat("Aluminum", new Color(0.62f, 0.64f, 0.66f));
        var copper = MakeMat("Copper", new Color(0.72f, 0.35f, 0.12f));
        var propM = MakeMat("Prop", new Color(0.13f, 0.13f, 0.14f));
        var yellow = MakeMat("GPS", new Color(0.95f, 0.75f, 0.10f));
        var red = MakeMat("Battery", new Color(0.70f, 0.12f, 0.10f));
        var black = MakeMat("Black", new Color(0.05f, 0.05f, 0.05f));
        var lens = MakeMat("Lens", new Color(0.10f, 0.12f, 0.20f));

        // Корпус — пакет пластин со стойками
        AddPart(visual, "Plate_Lower", PrimitiveType.Cube, new Vector3(0, -0.01f, 0), Vector3.zero, new Vector3(0.14f, 0.02f, 0.56f), carbon, 0.15f);
        AddPart(visual, "Plate_Mid", PrimitiveType.Cube, new Vector3(0, 0.05f, 0), Vector3.zero, new Vector3(0.12f, 0.02f, 0.46f), carbon, 0.10f);
        AddPart(visual, "Plate_Top", PrimitiveType.Cube, new Vector3(0, 0.10f, 0), Vector3.zero, new Vector3(0.10f, 0.02f, 0.34f), carbon, 0.08f);
        for (int i = 0; i < 4; i++)
        {
            float sx = i < 2 ? 1 : -1, sz = (i % 2 == 0) ? 1 : -1;
            AddPart(visual, "Standoff_" + i, PrimitiveType.Cylinder, new Vector3(sx * 0.045f, 0.02f, sz * 0.14f), Vector3.zero, new Vector3(0.012f, 0.06f, 0.012f), alu, 0.01f);
        }
        AddPart(visual, "Battery", PrimitiveType.Cube, new Vector3(0, 0.07f, -0.02f), Vector3.zero, new Vector3(0.07f, 0.035f, 0.18f), red, 0.25f);

        // GPS на мачте + две антенны (жёлтая деталь сверху, как на фото)
        AddPart(visual, "GPS_Mast", PrimitiveType.Cylinder, new Vector3(0, 0.145f, -0.20f), Vector3.zero, new Vector3(0.015f, 0.035f, 0.015f), black, 0.01f);
        AddPart(visual, "GPS_Puck", PrimitiveType.Cube, new Vector3(0, 0.19f, -0.20f), Vector3.zero, new Vector3(0.055f, 0.02f, 0.055f), yellow, 0.02f);
        for (int i = 0; i < 2; i++)
        {
            float sx = i == 0 ? 1 : -1;
            AddPart(visual, "Antenna_" + i, PrimitiveType.Cylinder, new Vector3(sx * 0.10f, 0.18f, -0.22f), new Vector3(15f, 0, 85f * sx), new Vector3(0.006f, 0.10f, 0.006f), black, 0.005f);
        }

        // Камера спереди снизу (нос = +Z)
        AddPart(visual, "Camera", PrimitiveType.Cube, new Vector3(0, -0.045f, 0.26f), Vector3.zero, new Vector3(0.05f, 0.05f, 0.045f), black, 0.03f);
        AddPart(visual, "Lens", PrimitiveType.Cylinder, new Vector3(0, -0.045f, 0.29f), new Vector3(90f, 0, 0), new Vector3(0.03f, 0.01f, 0.03f), lens, 0.01f);

        // Боковые раскосы между лучами + ножки (как на фото)
        for (int i = 0; i < 2; i++)
        {
            float sx = i == 0 ? 1 : -1;
            AddPart(visual, "Brace_" + i, PrimitiveType.Cube, new Vector3(sx * 0.27f, -0.005f, 0), Vector3.zero, new Vector3(0.018f, 0.012f, 0.60f), alu, 0.03f);
            for (int j = 0; j < 2; j++)
            {
                float sz = j == 0 ? 1 : -1;
                AddPart(visual, "Foot_" + i + j, PrimitiveType.Cube, new Vector3(sx * 0.27f, -0.045f, sz * 0.28f), Vector3.zero, new Vector3(0.025f, 0.07f, 0.025f), black, 0.01f);
            }
        }

        // Лучи, моторы, винты (X-конфигурация)
        int[,] quads = { { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 } };
        for (int q = 0; q < 4; q++)
        {
            int sx = quads[q, 0], sz = quads[q, 1];
            float angle = Mathf.Atan2(sx, sz) * Mathf.Rad2Deg;

            AddPart(visual, "Arm_" + q, PrimitiveType.Cube, new Vector3(sx * 0.18f, 0.01f, sz * 0.18f), new Vector3(0, angle, 0), new Vector3(0.05f, 0.035f, 0.5f), carbon, 0.08f);
            AddPart(visual, "Motor_" + q, PrimitiveType.Cylinder, new Vector3(sx * 0.36f, 0.045f, sz * 0.36f), Vector3.zero, new Vector3(0.07f, 0.025f, 0.07f), black, 0.06f);
            AddPart(visual, "MotorRing_" + q, PrimitiveType.Cylinder, new Vector3(sx * 0.36f, 0.03f, sz * 0.36f), Vector3.zero, new Vector3(0.074f, 0.006f, 0.074f), copper, 0.01f);

            // Винт: втулка + 3 лопасти, диагонали крутятся в одну сторону
            var propRoot = new GameObject("Prop_" + q).transform;
            propRoot.SetParent(visual, false);
            propRoot.localPosition = new Vector3(sx * 0.36f, 0.095f, sz * 0.36f);
            var prop = propRoot.gameObject.AddComponent<DroneProp>();
            prop.direction = sx * sz > 0 ? 1 : -1;
            props.Add(prop);
            prop.owner = this;
            AddPart(propRoot, "Hub_" + q, PrimitiveType.Cylinder, Vector3.zero, Vector3.zero, new Vector3(0.024f, 0.0125f, 0.024f), black, 0.01f);
            for (int b = 0; b < 3; b++)
            {
                var holder = new GameObject("BladeHolder_" + b).transform;
                holder.SetParent(propRoot, false);
                holder.localEulerAngles = new Vector3(0, b * 120f, 0);
                AddPart(holder, "Blade_" + q + "_" + b, PrimitiveType.Cube, new Vector3(0.125f, 0, 0), new Vector3(10f * prop.direction, 0, 0), new Vector3(0.23f, 0.005f, 0.032f), propM, 0.01f);
            }
        }

        // Корневой коллайдер ловит землю/другие дроны, детали в полёте кинематические
        rootCol.size = new Vector3(1.25f, 0.34f, 1.25f);
        rootCol.center = new Vector3(0, 0.04f, 0);
        rootCol.enabled = true;

        rb.mass = 1.5f;
        rb.useGravity = false;
        rb.isKinematic = false;
        rb.constraints = RigidbodyConstraints.FreezeRotation;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.linearDamping = 0f;

        foreach (var p in parts)
            Physics.IgnoreCollision(rootCol, p.GetComponent<BoxCollider>());
    }

    public void ClearGenerated()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
            Kill(transform.GetChild(i).gameObject);
        parts.Clear(); props.Clear(); visual = null; crashed = false;
    }

    GameObject AddPart(Transform parent, string name, PrimitiveType type,
        Vector3 localPos, Vector3 euler, Vector3 scale, Material mat, float mass)
    {
        var go = GameObject.CreatePrimitive(type);
        go.name = name;
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localPos;
        go.transform.localEulerAngles = euler;
        go.transform.localScale = scale;

        Kill(go.GetComponent<Collider>()); // у цилиндров капсула — меняем на бокс
        var box = go.AddComponent<BoxCollider>();
        box.size = type == PrimitiveType.Cube ? Vector3.one : new Vector3(1f, 2f, 1f);

        go.GetComponent<MeshRenderer>().sharedMaterial = mat;

        var prb = go.AddComponent<Rigidbody>(); // Rigidbody + BoxCollider на каждой детали
        prb.mass = mass;
        prb.isKinematic = true;   // в полёте деталь едет за корпусом
        prb.useGravity = false;
        prb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

        var part = go.AddComponent<DronePart>();
        part.owner = this;
        parts.Add(part);
        return go;
    }

    Material MakeMat(string name, Color c)
    {
        Shader sh = Shader.Find("Universal Render Pipeline/Lit");
        if (sh == null) sh = Shader.Find("Standard");
        if (sh == null) sh = Shader.Find("Diffuse");

        Material m = null;
#if UNITY_EDITOR
        if (!Application.isPlaying) // в редакторе сохраняем ассеты, чтобы переживали Play Mode
        {
            if (!AssetDatabase.IsValidFolder("Assets/DroneGenerated"))
                AssetDatabase.CreateFolder("Assets", "DroneGenerated");
            if (!AssetDatabase.IsValidFolder("Assets/DroneGenerated/Materials"))
                AssetDatabase.CreateFolder("Assets/DroneGenerated", "Materials");
            string path = $"Assets/DroneGenerated/Materials/{name}.mat";
            m = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (m == null) { m = new Material(sh); AssetDatabase.CreateAsset(m, path); }
        }
#endif
        if (m == null) m = new Material(sh);
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        if (m.HasProperty("_Color")) m.SetColor("_Color", c);
        if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", 0.35f);
        return m;
    }

    static void Kill(UnityEngine.Object o)
    {
        if (o == null) return;
        if (Application.isPlaying) Destroy(o); else DestroyImmediate(o);
    }

    // ---------------- Гизмо маршрута ----------------

    void OnDrawGizmos()
    {
        if (waypoints == null || waypoints.Length == 0) return;
        Gizmos.color = Color.cyan;
        for (int i = 0; i < waypoints.Length; i++)
        {
            var w = waypoints[i];
            if (w == null) continue;
            Gizmos.DrawSphere(w.position, 0.12f);
            var n = waypoints[(i + 1) % waypoints.Length];
            if (n != null) Gizmos.DrawLine(w.position, n.position);
#if UNITY_EDITOR
            Handles.Label(w.position + Vector3.up * 0.2f, "WP " + i);
#endif
        }
    }
}

// Деталь дрона: передаёт столкновения владельцу
public class DronePart : MonoBehaviour
{
    [HideInInspector] public QuadcopterDrone owner;
    void OnCollisionEnter(Collision c) { if (owner != null) owner.HandleImpact(c.collider); }
}

// Маркер винта с направлением вращения
public class DroneProp : MonoBehaviour
{
    public int direction = 1;
    public float spinSpeed = 2200f;      // запасное значение, если владелец потерян
    [HideInInspector] public QuadcopterDrone owner;
    float spin;

    void Update()
    {
        float target = owner != null
            ? (owner.IsCrashed ? 0f : owner.propSpinSpeed)
            : spinSpeed;
        spin = Mathf.MoveTowards(spin, target, 4000f * Time.deltaTime);
        transform.Rotate(0f, spin * direction * Time.deltaTime, 0f, Space.Self);
    }
}

// ---------------- Кнопки в инспекторе ----------------
#if UNITY_EDITOR
[CustomEditor(typeof(QuadcopterDrone))]
public class QuadcopterDroneEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        var drone = (QuadcopterDrone)target;

        EditorGUILayout.Space();
        if (GUILayout.Button("Сгенерировать дрон", GUILayout.Height(32)))
            drone.Generate();
        if (GUILayout.Button("Очистить модель"))
            drone.ClearGenerated();
        if (GUILayout.Button("Добавить путевую точку"))
            AddWaypoint(drone);
       if (GUILayout.Button("Пересобрать ссылки"))
           drone.RebuildLinks();
    }

    static void AddWaypoint(QuadcopterDrone d)
    {
        var folder = GameObject.Find("DroneWaypoints");
        if (folder == null) folder = new GameObject("DroneWaypoints");

        int count = d.waypoints == null ? 0 : d.waypoints.Length;
        var go = new GameObject("WP_" + count);
        Undo.RegisterCreatedObjectUndo(go, "Add Waypoint");
        go.transform.SetParent(folder.transform);

        Vector3 basePos = count > 0 && d.waypoints[count - 1] != null
            ? d.waypoints[count - 1].position
            : d.transform.position + Vector3.up * 1.5f;
        go.transform.position = basePos + Vector3.forward * 2f;

        var so = new SerializedObject(d);
        var arr = so.FindProperty("waypoints");
        arr.arraySize++;
        arr.GetArrayElementAtIndex(arr.arraySize - 1).objectReferenceValue = go.transform;
        so.ApplyModifiedProperties();
    }
}
#endif