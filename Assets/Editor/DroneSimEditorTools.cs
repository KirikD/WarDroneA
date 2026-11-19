using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;   // убирает конфликт UnityEngine.Object / System.Object
using Debug = UnityEngine.Debug;     // на случай, если IDE потом добавит System.Diagnostics

public static class DroneSimEditorTools
{
    const string MatFolder = "Assets/DroneSim/Materials";
    const string PrefabFolder = "Assets/DroneSim/Prefabs";

    [MenuItem("Tools/Drone Sim/Build Demo Scene %#d")]
    public static void BuildScene()
    {
        var existing = Object.FindObjectOfType<QuadPhysics>();
        if (existing != null)
        {
            if (!EditorUtility.DisplayDialog("Дрон уже есть в сцене",
                "Удалить существующие объекты симулятора и собрать заново?", "Пересобрать", "Отмена")) return;
            ClearSimObjects(false);
        }

        Undo.IncrementCurrentGroup();
        Undo.SetCurrentGroupName("Build Drone Scene");
        var drone = DroneSceneBuilder.BuildAll(EditorContext());
        ApplyPhysicsSettings(200);
        MarkDirty();
        Selection.activeGameObject = drone;
        EditorGUIUtility.PingObject(drone);
        Debug.Log("[DroneSim] Сцена создана в редакторе. Сохраните сцену (Ctrl+S) — при Play она больше не генерируется.");
    }
    [MenuItem("Tools/Drone Sim/Add Breakup To Drone")]
    public static void AddBreakup()
    {
        var ph = Object.FindObjectOfType<QuadPhysics>();
        if (ph == null) { EditorUtility.DisplayDialog("Нет дрона", "В сцене нет Quadcopter.", "OK"); return; }
        var br = ph.GetComponent<DroneBreakup>();
        if (br == null) br = Undo.AddComponent<DroneBreakup>(ph.gameObject);
        br.physics = ph;
        EditorUtility.SetDirty(br);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = ph.gameObject;
    }
    [MenuItem("Tools/Drone Sim/Add Telemetry Panel")]
    public static void AddTelemetryPanel()
    {
        if (Object.FindObjectOfType<SimTelemetryPanel>() != null)
        { EditorUtility.DisplayDialog("Уже есть", "Панель телеметрии уже в сцене.", "OK"); return; }

        var hud = Object.FindObjectOfType<SimHud>();
        GameObject host = hud != null ? hud.gameObject : new GameObject("Telemetry");
        if (hud == null) Undo.RegisterCreatedObjectUndo(host, "Add Telemetry Panel");
        Undo.AddComponent<SimTelemetryPanel>(host);

        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = host;
        Debug.Log("[DroneSim] Панель телеметрии добавлена. F7 — открыть/закрыть.");
    }
    [MenuItem("Tools/Drone Sim/Rebuild Drone Only")]
    public static void RebuildDrone()
    {
        var old = Object.FindObjectOfType<QuadPhysics>();
        Vector3 spawn = old != null ? old.transform.position : new Vector3(0, 0.12f, 0);
        if (old != null) Undo.DestroyObjectImmediate(old.gameObject);

        Undo.IncrementCurrentGroup();
        Undo.SetCurrentGroupName("Rebuild Drone");
        var ctx = EditorContext(); ctx.spawn = spawn;
        var drone = DroneSceneBuilder.BuildDrone(ctx);
        DroneSceneBuilder.BuildCameraAndHud(ctx, drone.transform);
        DroneSceneBuilder.WireReferences();
        MarkDirty();
        Selection.activeGameObject = drone;
    }

    [MenuItem("Tools/Drone Sim/Wire References")]
    public static void Wire()
    {
        DroneSceneBuilder.WireReferences();
        MarkDirty();
        Debug.Log("[DroneSim] Ссылки перепривязаны.");
    }

    [MenuItem("Tools/Drone Sim/Clear Sim Objects")]
    public static void ClearSimMenu() => ClearSimObjects(true);

    public static void ClearSimObjects(bool ask)
    {
        if (ask && !EditorUtility.DisplayDialog("Удалить объекты симулятора",
            "Будут удалены Environment, Quadcopter, SimCamera, HUD. Продолжить?", "Удалить", "Отмена")) return;

        string[] roots = { "Quadcopter", "Environment", "SimCamera", "HUD" };
        foreach (var name in roots)
        {
            var go = GameObject.Find(name);
            if (go != null) Undo.DestroyObjectImmediate(go);
        }
        MarkDirty();
    }

    [MenuItem("Tools/Drone Sim/Apply Physics Settings 200 Hz")]
    public static void ApplyPhysicsMenu() { ApplyPhysicsSettings(200); Debug.Log("[DroneSim] Fixed Timestep = 0.005 (200 Hz)."); }

    [MenuItem("Tools/Drone Sim/Save Drone As Prefab")]
    public static void SavePrefab()
    {
        var ph = Object.FindObjectOfType<QuadPhysics>();
        if (ph == null) { EditorUtility.DisplayDialog("Нет дрона", "В сцене нет Quadcopter.", "OK"); return; }
        if (!AssetDatabase.IsValidFolder("Assets/DroneSim")) AssetDatabase.CreateFolder("Assets", "DroneSim");
        if (!AssetDatabase.IsValidFolder(PrefabFolder)) AssetDatabase.CreateFolder("Assets/DroneSim", "Prefabs");
        PrefabUtility.SaveAsPrefabAssetAndConnect(ph.gameObject, PrefabFolder + "/Quadcopter.prefab", InteractionMode.UserAction);
        Debug.Log("[DroneSim] Префаб сохранён: " + PrefabFolder + "/Quadcopter.prefab");
    }

    // ---------- внутреннее ----------
    static DroneBuildContext EditorContext()
    {
        var boot = Object.FindObjectOfType<SimBootstrap>();
        return new DroneBuildContext
        {
            spawn = boot != null ? boot.spawnPos : new Vector3(0, 0.12f, 0),
            markEnvironmentStatic = true,
            materialFactory = EditorMaterial,
            register = go => Undo.RegisterCreatedObjectUndo(go, "Build Drone Scene")
        };
    }

    static Material EditorMaterial(string name, Color c)
    {
        if (!AssetDatabase.IsValidFolder("Assets/DroneSim")) AssetDatabase.CreateFolder("Assets", "DroneSim");
        if (!AssetDatabase.IsValidFolder(MatFolder)) AssetDatabase.CreateFolder("Assets/DroneSim", "Materials");

        string path = $"{MatFolder}/M_{name}.mat";
        var m = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (m == null)
        {
            Shader sh = Shader.Find("Sprites/Default") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
            m = new Material(sh) { color = c };
            AssetDatabase.CreateAsset(m, path);
            AssetDatabase.SaveAssets();
        }
        else { m.color = c; EditorUtility.SetDirty(m); }
        return m;
    }

    static void ApplyPhysicsSettings(int hz)
    {
        try
        {
            var tm = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TimeManager.asset")[0];
            var so = new SerializedObject(tm);
            so.FindProperty("Fixed Timestep").floatValue = 1f / hz;
            so.FindProperty("Maximum Allowed Timestep").floatValue = 0.1f;
            so.ApplyModifiedProperties();
        }
        catch (System.Exception e) { Debug.LogWarning("[DroneSim] TimeManager: " + e.Message); }

        var boot = Object.FindObjectOfType<SimBootstrap>();
        if (boot != null) { boot.physicsHz = hz; EditorUtility.SetDirty(boot); }
    }

    static void MarkDirty()
    {
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
    }
}

[CustomEditor(typeof(SimBootstrap))]
public class SimBootstrapInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        EditorGUILayout.Space(8);
        EditorGUILayout.LabelField("Создание сцены в редакторе", EditorStyles.boldLabel);

        if (GUILayout.Button("СОЗДАТЬ ДЕМО-СЦЕНУ (статичную)", GUILayout.Height(30)))
            DroneSimEditorTools.BuildScene();
        if (GUILayout.Button("Пересобрать только дрон"))
            DroneSimEditorTools.RebuildDrone();
        if (GUILayout.Button("Привязать ссылки"))
            DroneSimEditorTools.Wire();
        if (GUILayout.Button("Сохранить дрон как префаб"))
            DroneSimEditorTools.SavePrefab();
        if (GUILayout.Button("Удалить объекты симулятора"))
            DroneSimEditorTools.ClearSimMenu();

        EditorGUILayout.HelpBox(
            "Сцена строится в редакторе и сохраняется вместе с проектом. " +
            "autoBuildAtRuntime должен быть ВЫКЛЮЧЕН — тогда при Play ничего не генерируется.",
            MessageType.Info);
    }
}