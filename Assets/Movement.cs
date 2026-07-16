using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class TankMovement : MonoBehaviour
{
    public float moveSpeed = 8f;   // Скорость движения вперед
    public float turnSpeed = 100f; // Скорость разворота
    
    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.freezeRotation = true; // Важно: предотвращает опрокидывание
    }

    void FixedUpdate()
    {
        Keyboard kb = Keyboard.current;
        if (kb == null) return;

        // 1. Считываем ввод
        float forwardInput = 0f;
        if (kb.wKey.isPressed) forwardInput += 1f;
        if (kb.sKey.isPressed) forwardInput -= 1f;

        float turnInput = 0f;
        if (kb.dKey.isPressed) turnInput += 1f;
        if (kb.aKey.isPressed) turnInput -= 1f;

        // 2. Вычисляем скорости "гусениц"
        // ForwardInput отвечает за общую тягу
        // TurnInput создает разницу скоростей между сторонами
        float leftTrack = forwardInput + turnInput;
        float rightTrack = forwardInput - turnInput;

        // 3. Движение (среднее арифметическое тяги)
        float moveValue = (leftTrack + rightTrack) / 2f;
        Vector3 moveVelocity = transform.forward * moveValue * moveSpeed * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + moveVelocity);

        // 4. Поворот (разница скоростей тяги)
        float turnValue = (leftTrack - rightTrack) / 2f;
        float rotation = turnValue * turnSpeed * Time.fixedDeltaTime;
        Quaternion turnRotation = Quaternion.Euler(0f, rotation, 0f);
        rb.MoveRotation(rb.rotation * turnRotation);
    }
}