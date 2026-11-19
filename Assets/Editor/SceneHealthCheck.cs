#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class SceneHealthCheck
{
    [MenuItem("Tools/Drone Sim/Check Scene Health")]
    public static void Check()
    {
        int missing = 0, objects = 0, bigColliders = 0;
        for (int s = 0; s < SceneManager.sceneCount; s++)
        {
            var scene = SceneManager.GetSceneAt(s);
            foreach (var root in scene.GetRootGameObjects())
                Scan(root.transform, ref missing, ref objects, ref bigColliders);
        }
        Debug.Log($"[SceneCheck] объектов: {objects}, missing-скриптов: {missing}, гигантских MeshCollider: {bigColliders}");
    }

    static void Scan(Transform t, ref int missing, ref int objects, ref int big)
    {
        objects++;
        var comps = t.GetComponents<Component>();
        foreach (var c in comps)
            if (c == null) { missing++; Debug.LogWarning("[Missing script] " + Path(t), t.gameObject); }

        var mc = t.GetComponent<MeshCollider>();
        if (mc != null && mc.sharedMesh != null)
        {
            Vector3 size = Vector3.Scale(mc.sharedMesh.bounds.size, t.lossyScale);
            if (Mathf.Max(size.x, Mathf.Max(size.y, size.z)) > 500f)
            {
                big++;
                Debug.LogWarning($"[Огромный MeshCollider] {Path(t)}  размер {size}  вершин {mc.sharedMesh.vertexCount}", t.gameObject);
            }
        }
        foreach (Transform c in t) Scan(c, ref missing, ref objects, ref big);
    }

    static string Path(Transform t)
    {
        string p = t.name;
        while (t.parent != null) { t = t.parent; p = t.name + "/" + p; }
        return p;
    }
}
#endif