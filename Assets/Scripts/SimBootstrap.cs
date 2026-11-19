using UnityEngine;

[DefaultExecutionOrder(-100)]
public class SimBootstrap : MonoBehaviour
{
    [Header("Режим сборки")]
    [Tooltip("ВЫКЛ — сцена создана в редакторе и не трогается при Play (рекомендуется). ВКЛ — старое поведение: генерация при запуске.")]
    public bool autoBuildAtRuntime = false;
    [Tooltip("При авто-сборке: не строить, если дрон уже есть в сцене")]
    public bool buildOnlyIfEmpty = true;
    [Tooltip("Точка спавна для авто-сборки и для кнопки построения сцены")]
    public Vector3 spawnPos = new Vector3(0f, 0.12f, 0f);

    [Header("Настройки времени выполнения")]
    public bool applyFixedTimestep = true;
    [Tooltip("Частота физики, Гц")] [Range(50, 400)] public int physicsHz = 200;
    [Range(30, 240)] public int targetFps = 144;
    [Range(0.02f, 0.3f)] public float maxDeltaTime = 0.1f;

    [Header("Ссылки на объекты сцены")]
    public QuadPhysics drone;
    public DroneInputManager input;
    public ChaseCamera chaseCamera;
    public SimHud hud;

    void Awake()
    {
        if (applyFixedTimestep)
        {
            Time.fixedDeltaTime = 1f / physicsHz;
            Time.maximumDeltaTime = maxDeltaTime;
        }
        Application.targetFrameRate = targetFps;

        bool needBuild = autoBuildAtRuntime &&
                         (!buildOnlyIfEmpty || (drone == null && FindObjectOfType<QuadPhysics>() == null));
        if (needBuild) DroneSceneBuilder.BuildAll(DroneBuildContext.Runtime(spawnPos));

        Wire();

        if (drone == null)
            Debug.LogWarning("[SimBootstrap] Дрон не найден. Постройте сцену: Tools ▸ Drone Sim ▸ Build Demo Scene " +
                             "(или кнопка в инспекторе SimBootstrap).");
    }

    public void Wire()
    {
        if (drone == null)
        {
            var p = FindObjectOfType<QuadPhysics>();
            if (p != null) drone = p;
        }
        if (drone == null) return;

        if (input == null) input = drone.GetComponent<DroneInputManager>();
        if (chaseCamera == null) chaseCamera = FindObjectOfType<ChaseCamera>();
        if (chaseCamera != null)
        {
            chaseCamera.target = drone.transform;
            if (chaseCamera.input == null) chaseCamera.input = input;
        }
        if (hud == null) hud = FindObjectOfType<SimHud>();
    }
}