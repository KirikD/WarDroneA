using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;   // убирает конфликт с System.Random
using Debug = UnityEngine.Debug;     // убирает конфликт с System.Diagnostics.Debug

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
    [Tooltip("Подброс вверх")][Range(0f, 2f)] public float upwardBias = 0.35f;
    [Tooltip("Случайный разброс направления")][Range(0f, 1f)] public float randomness = 0.35f;
    [Tooltip("Случайное вращение обломков, рад/с")][Range(0f, 60f)] public float spinTorque = 18f;

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
    public PhysicsMaterial debrisMaterial;
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
    PhysicsMaterial runtimeMat;

    void Awake()
    {
        if (physics == null) physics = GetComponent<QuadPhysics>();
        cam = FindObjectOfType<ChaseCamera>();
        sound = GetComponent<MotorSound>();
        if (debrisMaterial == null)
        {
            runtimeMat = new PhysicsMaterial("DebrisPhys")
            {
                bounciness = bounciness,
                dynamicFriction = friction,
                staticFriction = friction,
                bounceCombine = PhysicsMaterialCombine.Maximum,
                frictionCombine = PhysicsMaterialCombine.Average
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
        lastVelocity = physics.Rb.linearVelocity;   // предударная скорость (OnCollisionEnter приходит уже после solver'а)
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
        Vector3 rootVel = lastVelocity.sqrMagnitude > 0.01f ? lastVelocity : (rootRb ? rootRb.linearVelocity : Vector3.zero);
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
            rb.linearDamping = debrisDrag;
            rb.angularDamping = debrisAngularDrag;
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

            rb.linearVelocity = pointVel * velocityInheritance + dir * force;
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