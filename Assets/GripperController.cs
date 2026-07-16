using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Управление клешнёй GFS-X. Реализует "логический" захват мяча вместо
/// физического (твёрдые губки в симуляторах плохо держат круглые объекты):
/// при обнаружении мяча датчиком клешни — мяч "прилипает" к HoldPoint
/// (isKinematic = true, коллайдер выключен, SetParent), при разжатии —
/// физика возвращается.
/// </summary>
public class GripperController : MonoBehaviour
{
    [Header("Ссылки")]
    public VirtualSensors sensors;   // источник значения gripperIR
    public Transform holdPoint;      // точка между губками клешни
    public Transform leftJaw;        // Circle.003
    public Transform rightJaw;       // Circle.004

    [Header("Состояние клешни")]
    public bool isGripperClosed = false; // управляется извне (команда сжать/разжать)

    [Header("Физическое движение губок")]
    [Tooltip("Поворот каждой губки от импортированного раскрытого положения до смыкания.")]
    [Min(0f)] public float closeAngle = 35f;
    [Tooltip("Дополнительный поворот наружу от импортированного положения.")]
    [Min(0f)] public float extraOpenAngle = 15f;
    [Min(1f)] public float jawRotationSpeed = 90f;

    // Текущий удерживаемый мяч (null, если ничего не захвачено)
    private Rigidbody heldBallRb;
    private Collider heldBallCollider;
    private Transform heldBallOriginalParent;
    private Transform leftHinge;
    private Transform rightHinge;
    private bool jawsReady;

    // Центры круглых шарниров в локальной плоскости общего основания Circle.002.
    private static readonly Vector2 LeftHingeXZ = new Vector2(-0.000381f, 0.001094f);
    private static readonly Vector2 RightHingeXZ = new Vector2(0.004971f, -0.003142f);

    private void Awake()
    {
        FindJawsIfNeeded();

        if (leftJaw == null || rightJaw == null)
        {
            Debug.LogError("GripperController: не найдены обе губки Circle.003/Circle.004.", this);
            return;
        }

        leftHinge = CreateHinge("LeftJawHinge", leftJaw, LeftHingeXZ);
        rightHinge = CreateHinge("RightJawHinge", rightJaw, RightHingeXZ);
        jawsReady = true;
    }

    private void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard != null && keyboard.gKey.wasPressedThisFrame)
        {
            SetGripperClosed(!isGripperClosed);
        }
    }

    /// <summary>
    /// Вызывается извне для сжатия/разжатия клешни (например, по команде игрока или ИИ).
    /// </summary>
    public void SetGripperClosed(bool closed)
    {
        isGripperClosed = closed;

        if (!isGripperClosed && heldBallRb != null)
        {
            ReleaseBall();
        }
    }

    private void FixedUpdate()
    {
        AnimateJaws();

        if (sensors == null || holdPoint == null) return;

        // Захватываем во время смыкания, пока мяч ещё находится между губками.
        // Ожидание полного закрытия позволяет коллайдерам вытолкнуть мяч вверх.
        if (isGripperClosed && sensors.gripperIR == 1 && heldBallRb == null)
        {
            TryGrabBall();
        }

        // Если клешня открыта во время удержания — отпускаем мяч
        if (!isGripperClosed && heldBallRb != null)
        {
            ReleaseBall();
        }
    }

    private void AnimateJaws()
    {
        if (!jawsReady) return;

        // Импортированное положение является раскрытым. При закрытии губки
        // симметрично поворачиваются внутрь вокруг реальных круглых шарниров.
        Quaternion leftTarget = Quaternion.AngleAxis(
            isGripperClosed ? closeAngle : -extraOpenAngle,
            Vector3.up);

        Quaternion rightTarget = Quaternion.AngleAxis(
            isGripperClosed ? -closeAngle : extraOpenAngle,
            Vector3.up);

        float maxStep = jawRotationSpeed * Time.fixedDeltaTime;
        leftHinge.localRotation = Quaternion.RotateTowards(leftHinge.localRotation, leftTarget, maxStep);
        rightHinge.localRotation = Quaternion.RotateTowards(rightHinge.localRotation, rightTarget, maxStep);
    }

    private bool AreJawsClosed()
    {
        if (!jawsReady) return false;

        Quaternion leftClosed = Quaternion.AngleAxis(closeAngle, Vector3.up);
        Quaternion rightClosed = Quaternion.AngleAxis(-closeAngle, Vector3.up);
        return Quaternion.Angle(leftHinge.localRotation, leftClosed) < 0.5f
            && Quaternion.Angle(rightHinge.localRotation, rightClosed) < 0.5f;
    }

    private static Transform CreateHinge(string hingeName, Transform jaw, Vector2 hingeXZ)
    {
        Transform commonBase = jaw.parent;
        float hingeY = jaw.localPosition.y;

        GameObject hingeObject = new GameObject(hingeName);
        Transform hinge = hingeObject.transform;
        hinge.SetParent(commonBase, false);
        hinge.localPosition = new Vector3(hingeXZ.x, hingeY, hingeXZ.y);
        hinge.localRotation = Quaternion.identity;
        hinge.localScale = Vector3.one;

        // Сохраняем мировое положение губки, меняя только точку вращения.
        jaw.SetParent(hinge, true);
        return hinge;
    }

    private void FindJawsIfNeeded()
    {
        if (leftJaw != null && rightJaw != null) return;

        foreach (Transform child in transform.root.GetComponentsInChildren<Transform>(true))
        {
            if (child.parent == null || child.parent.name != "Circle.002") continue;

            if (leftJaw == null && child.name == "Circle.003")
                leftJaw = child;
            else if (rightJaw == null && child.name == "Circle.004")
                rightJaw = child;
        }
    }

    /// <summary>
    /// Ищет мяч рядом с датчиком клешни и переводит его в "логически захваченное" состояние.
    /// </summary>
    private void TryGrabBall()
    {
        Collider[] nearby = Physics.OverlapSphere(sensors.gripperIRPoint.position, sensors.gripperIRRange);

        foreach (var col in nearby)
        {
            if (!col.CompareTag(sensors.targetBallTag)) continue;

            Rigidbody ballRb = col.attachedRigidbody;
            if (ballRb == null) continue;

            heldBallRb = ballRb;
            heldBallCollider = col;
            heldBallOriginalParent = ballRb.transform.parent;

            // Отключаем физику мяча и делаем его дочерним объектом HoldPoint
            heldBallRb.isKinematic = true;
            heldBallCollider.enabled = false;

            heldBallRb.transform.SetParent(holdPoint);
            heldBallRb.transform.localPosition = Vector3.zero;
            heldBallRb.transform.localRotation = Quaternion.identity;

            break; // захватываем только один мяч за раз
        }
    }

    /// <summary>
    /// Возвращает мяч в исходное физическое состояние (разжатие клешни).
    /// </summary>
    private void ReleaseBall()
    {
        if (heldBallRb == null) return;

        heldBallRb.transform.SetParent(heldBallOriginalParent);
        heldBallCollider.enabled = true;
        heldBallRb.isKinematic = false;

        heldBallRb = null;
        heldBallCollider = null;
        heldBallOriginalParent = null;
    }
}
