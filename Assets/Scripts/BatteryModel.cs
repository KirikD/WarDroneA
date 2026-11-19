using UnityEngine;

public class BatteryModel : MonoBehaviour
{
    [Header("Пакет LiPo")]
    [Tooltip("Число банок (S)")] [Range(1, 8)] public int cells = 4;
    [Tooltip("Ёмкость, мАч")] public float capacityMah = 1500f;
    [Tooltip("Внутреннее сопротивление одной банки, Ом")] [Range(0.001f, 0.05f)] public float cellResistance = 0.008f;
    [Tooltip("Кривая напряжения банки от SoC (0..1)")]
    public AnimationCurve cellCurve = new AnimationCurve(
        new Keyframe(0, 3.0f), new Keyframe(0.1f, 3.3f), new Keyframe(0.3f, 3.55f),
        new Keyframe(0.6f, 3.75f), new Keyframe(0.9f, 4.0f), new Keyframe(1, 4.2f));
    [Tooltip("Потребление полётного контроллера/FPV, Вт")] public float idlePowerW = 3f;

    public float Voltage { get; private set; }
    public float Current { get; private set; }
    public float Soc { get; private set; } = 1f;
    public float UsedMah { get; private set; }
    public float CellVoltage => Voltage / cells;
    public float FullVoltage => cells * 4.2f;
    public bool IsLow => CellVoltage < 3.5f;
    public bool IsCritical => CellVoltage < 3.3f;
    public bool IsEmpty => CellVoltage < 3.0f;

    void Awake() { Voltage = FullVoltage; }

    public void Step(float dt, float motorPowerW)
    {
        float power = motorPowerW + idlePowerW;
        float ocv = cells * cellCurve.Evaluate(Soc);
        float i = power / Mathf.Max(1f, Voltage);
        float target = Mathf.Max(0f, ocv - i * cellResistance * cells);
        Voltage += (target - Voltage) * Mathf.Clamp01(dt * 12f); // динамика просадки
        Current = i;
        UsedMah += i * dt / 3.6f;
        Soc = Mathf.Clamp01(1f - UsedMah / capacityMah);
    }

    public void ResetState() { Soc = 1; UsedMah = 0; Current = 0; Voltage = FullVoltage; }
}