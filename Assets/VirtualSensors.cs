using UnityEngine;

/// <summary>
/// Симуляция датчиков робота GFS-X: ультразвуковой датчик (конус лучей),
/// ИК-датчики препятствий (короткие одиночные лучи) и ИК-датчик клешни
/// (обнаружение мяча TargetBall).
/// </summary>
public class VirtualSensors : MonoBehaviour
{
    [Header("Точки-якоря датчиков")]
    public Transform centerPoint;     // УЗ-датчик (вперёд)
    public Transform leftIRPoint;     // ИК влево
    public Transform rightIRPoint;    // ИК вправо
    public Transform gripperIRPoint;  // ИК внутри клешни

    [Header("Параметры ультразвукового датчика")]
    public float ultrasonicMaxRange = 2.0f;   // максимальная дальность, м
    public float ultrasonicConeAngle = 30f;   // угол конуса обзора, град
    public int ultrasonicRayCount = 5;        // количество лучей веера

    [Header("Параметры ИК-датчиков стен")]
    public float irWallRange = 0.15f;         // дальность обнаружения стены, м

    [Header("Параметры ИК-датчика клешни")]
    public float gripperIRRange = 0.08f;      // дальность обнаружения мяча, м
    public string targetBallTag = "TargetBall";

    [Header("Слои для рейкастов")]
    public LayerMask obstacleLayerMask = ~0;  // по умолчанию — все слои

    // Публичные результаты (обновляются каждый FixedUpdate)
    [Header("Результаты (только для чтения)")]
    [Range(0f, 1f)] public float ultrasonicDistance01 = 1f; // 0 = вплотную, 1 = чисто
    public int leftIR = 0;    // 1 = стена рядом, 0 = свободно
    public int rightIR = 0;
    public int gripperIR = 0; // 1 = мяч обнаружен в клешне

    private void FixedUpdate()
    {
        ultrasonicDistance01 = ReadUltrasonic();
        leftIR = ReadWallIR(leftIRPoint);
        rightIR = ReadWallIR(rightIRPoint);
        gripperIR = ReadGripperIR();
    }

    /// <summary>
    /// УЗ-датчик: веер лучей в пределах конуса, ищем кратчайшее расстояние
    /// до препятствия (мяч игнорируется — слишком мал для реальной УЗ-локации).
    /// Возвращает нормализованное значение: 0 = вплотную, 1 = чисто.
    /// </summary>
    private float ReadUltrasonic()
    {
        if (centerPoint == null) return 1f;

        float closestDistance = ultrasonicMaxRange;
        float halfAngle = ultrasonicConeAngle * 0.5f;

        for (int i = 0; i < ultrasonicRayCount; i++)
        {
            // Распределяем лучи равномерно по конусу от -halfAngle до +halfAngle
            float t = ultrasonicRayCount > 1 ? (float)i / (ultrasonicRayCount - 1) : 0.5f;
            float angle = Mathf.Lerp(-halfAngle, halfAngle, t);

            Vector3 rayDir = Quaternion.Euler(0f, angle, 0f) * centerPoint.forward;

            if (Physics.Raycast(centerPoint.position, rayDir, out RaycastHit hit, ultrasonicMaxRange, obstacleLayerMask))
            {
                // Игнорируем мяч — он слишком мал для УЗ-локации
                if (hit.collider.CompareTag(targetBallTag))
                    continue;

                if (hit.distance < closestDistance)
                    closestDistance = hit.distance;
            }

            // Отладочная визуализация в редакторе
            Debug.DrawRay(centerPoint.position, rayDir * ultrasonicMaxRange, Color.cyan);
        }

        return Mathf.Clamp01(closestDistance / ultrasonicMaxRange);
    }

    /// <summary>
    /// ИК-датчик стены: одиночный короткий луч. 1 — стена обнаружена, 0 — свободно.
    /// </summary>
    private int ReadWallIR(Transform point)
    {
        if (point == null) return 0;

        bool hitWall = Physics.Raycast(point.position, point.forward, out RaycastHit hit, irWallRange, obstacleLayerMask);
        Debug.DrawRay(point.position, point.forward * irWallRange, hitWall ? Color.red : Color.green);

        // Мяч не считается стеной
        if (hitWall && hit.collider.CompareTag(targetBallTag))
            return 0;

        return hitWall ? 1 : 0;
    }

    /// <summary>
    /// ИК-датчик клешни: направлен внутрь захвата, фиксирует наличие мяча
    /// с тегом TargetBall на близком расстоянии.
    /// </summary>
    private int ReadGripperIR()
    {
        if (gripperIRPoint == null) return 0;

        bool hitBall = Physics.Raycast(gripperIRPoint.position, gripperIRPoint.forward, out RaycastHit hit, gripperIRRange, obstacleLayerMask);
        Debug.DrawRay(gripperIRPoint.position, gripperIRPoint.forward * gripperIRRange, hitBall ? Color.yellow : Color.gray);

        if (hitBall && hit.collider.CompareTag(targetBallTag))
            return 1;

        return 0;
    }
}
