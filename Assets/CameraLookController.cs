using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Поворачивает дочернюю камеру относительно её исходного направления.
/// Стрелки влево/вправо дают обзор 180 градусов, вверх/вниз — 90 градусов.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Camera))]
public sealed class CameraLookController : MonoBehaviour
{
    [Header("Скорость")]
    [Min(1f)] public float rotationSpeed = 90f;

    [Header("Пределы обзора")]
    [Min(0f)] public float horizontalLimit = 90f;
    [Min(0f)] public float verticalLimit = 45f;

    private Quaternion initialLocalRotation;
    private float yaw;
    private float pitch;

    private void Awake()
    {
        initialLocalRotation = transform.localRotation;
    }

    private void LateUpdate()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null) return;

        float horizontal = 0f;
        if (keyboard.leftArrowKey.isPressed) horizontal -= 1f;
        if (keyboard.rightArrowKey.isPressed) horizontal += 1f;

        float vertical = 0f;
        if (keyboard.upArrowKey.isPressed) vertical += 1f;
        if (keyboard.downArrowKey.isPressed) vertical -= 1f;

        yaw = Mathf.Clamp(
            yaw + horizontal * rotationSpeed * Time.deltaTime,
            -horizontalLimit,
            horizontalLimit);

        pitch = Mathf.Clamp(
            pitch - vertical * rotationSpeed * Time.deltaTime,
            -verticalLimit,
            verticalLimit);

        transform.localRotation =
            initialLocalRotation * Quaternion.Euler(pitch, yaw, 0f);
    }
}
