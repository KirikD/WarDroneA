using UnityEngine;

public class WindSystem : MonoBehaviour
{
    [Header("Базовый ветер")]
    [Tooltip("Скорость, м/с")] [Range(0, 25)] public float baseSpeed = 3f;
    [Tooltip("Направление, КУДА дует, градусы (0 = +Z / север)")] [Range(0, 360)] public float directionDeg = 270f;
    [Tooltip("Медленное блуждание направления, ±град")] [Range(0, 45)] public float dirWander = 12f;

    [Header("Порывы и турбулентность")]
    [Tooltip("Амплитуда порывов, м/с")] [Range(0, 15)] public float gustAmp = 2f;
    [Tooltip("Частота порывов")] [Range(0.02f, 2)] public float gustFreq = 0.25f;
    [Tooltip("Амплитуда мелкой турбулентности, м/с")] [Range(0, 8)] public float turbAmp = 0.8f;
    [Tooltip("Пространственный масштаб турбулентности, м")] [Range(1, 100)] public float turbScale = 25f;
    [Tooltip("Вертикальные порывы, м/с")] [Range(0, 5)] public float verticalAmp = 0.5f;

    [Header("Сдвиг ветра по высоте")]
    [Tooltip("Степень профиля (0 — нет сдвига, 0.14 — типичный атмосферный)")] [Range(0, 0.6f)] public float shearExponent = 0.14f;
    [Tooltip("Высота замера базовой скорости, м")] public float referenceHeight = 10f;

    [Header("Среда")]
    [Tooltip("Плотность воздуха, кг/м³ (влияет на тягу и drag)")] [Range(0.6f, 1.4f)] public float airDensity = 1.225f;
    [Tooltip("Гравитация, м/с²")] [Range(1, 20)] public float gravity = 9.81f;
    public float randomSeed = 17f;

    public Vector3 Current { get; set; } // последний сэмпл у дрона (для HUD)

    void Update()
    {
        if (!Mathf.Approximately(Physics.gravity.y, -gravity))
            Physics.gravity = new Vector3(0, -gravity, 0);
    }

    public Vector3 GetWind(Vector3 pos)
    {
        float t = Time.time;
        float dir = directionDeg + dirWander * (Mathf.PerlinNoise(randomSeed, t * 0.05f) * 2f - 1f);
        float h = Mathf.Max(0.1f, pos.y);
        float shear = Mathf.Pow(h / Mathf.Max(0.5f, referenceHeight), shearExponent);
        shear = Mathf.Clamp(shear, 0.2f, 2.5f);
        float gust = gustAmp * (Mathf.PerlinNoise(randomSeed + 100f, t * gustFreq) * 2f - 1f);
        float speed = Mathf.Max(0, baseSpeed * shear + gust);
        Vector3 horiz = Quaternion.Euler(0, dir, 0) * Vector3.forward * speed;
        Vector3 turb = new Vector3(
            Mathf.PerlinNoise(pos.x / turbScale + randomSeed, t * 0.7f) * 2f - 1f,
            (Mathf.PerlinNoise(pos.z / turbScale + randomSeed + 50f, t * 0.9f) * 2f - 1f) * verticalAmp,
            Mathf.PerlinNoise(pos.y / turbScale + randomSeed + 90f, t * 0.8f) * 2f - 1f) * turbAmp;
        return horiz + turb;
    }

    public void PresetCalm() { baseSpeed = 0; gustAmp = 0; turbAmp = 0.1f; verticalAmp = 0; }
    public void PresetBreeze() { baseSpeed = 4; gustAmp = 2; turbAmp = 0.8f; verticalAmp = 0.4f; }
    public void PresetStrong() { baseSpeed = 10; gustAmp = 5; turbAmp = 2.2f; verticalAmp = 1.2f; }
    public void PresetStorm() { baseSpeed = 16; gustAmp = 8; turbAmp = 3.5f; verticalAmp = 2f; }

    void OnDrawGizmosSelected()
    {
        Vector3 dir = Quaternion.Euler(0, directionDeg, 0) * Vector3.forward;
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(Vector3.up * 2, dir * (1f + baseSpeed * 0.3f));
    }
}