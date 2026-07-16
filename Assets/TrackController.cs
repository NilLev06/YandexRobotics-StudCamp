using UnityEngine;
using UnityEngine.InputSystem; // Подключили новую систему ввода

[RequireComponent(typeof(Rigidbody))]
public class TrackController : MonoBehaviour
{
    [Header("Калибровка движения")]
    public float moveSpeed = 0.37f;
    public float turnSpeed = 120f;
    public float turnK = 0.30f;
    public float maxLinearCmd = 0.25f;
    
    [Header("Настройки моторов (PWM)")]
    public float motorDeadzone = 10f;
    public float minMotorPwm = 35f;
    public float maxPwmStep = 15f;

    [Header("Управление (Inputs)")]
    [Range(-1f, 1f)] public float gas = 0f;
    [Range(-1f, 1f)] public float steer = 0f;

    private Rigidbody rb;
    private float currentLeftPwm = 0f;
    private float currentRightPwm = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        
        // Настройка Rigidbody
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        rb.linearDamping = 9f; 
        rb.angularDamping = 9f;
    }

    void Update()
    {
        // Сбрасываем значения каждый кадр
        gas = 0f;
        steer = 0f;

        // Читаем кнопки через новую систему ввода (Input System Package)
        if (Keyboard.current != null)
        {
            // Стрелки зарезервированы для камеры; робот управляется через WASD.
            if (Keyboard.current.wKey.isPressed) gas = 1f;
            else if (Keyboard.current.sKey.isPressed) gas = -1f;

            if (Keyboard.current.dKey.isPressed) steer = 1f;
            else if (Keyboard.current.aKey.isPressed) steer = -1f;
        }
    }

    void FixedUpdate()
    {
        float clampedGas = Mathf.Clamp(gas, -1f, 1f) * maxLinearCmd;
        float clampedSteer = Mathf.Clamp(steer, -1f, 1f);

        float leftSpeedTarget = clampedGas + (clampedSteer * turnK);
        float rightSpeedTarget = clampedGas - (clampedSteer * turnK);

        float targetLeftPwm = ApplyMotorLogic(leftSpeedTarget * 200f);
        float targetRightPwm = ApplyMotorLogic(rightSpeedTarget * 200f);

        currentLeftPwm = Mathf.MoveTowards(currentLeftPwm, targetLeftPwm, maxPwmStep);
        currentRightPwm = Mathf.MoveTowards(currentRightPwm, targetRightPwm, maxPwmStep);

        float effectiveLeftTarget = currentLeftPwm / 200f;
        float effectiveRightTarget = currentRightPwm / 200f;

        float effectiveGas = (effectiveLeftTarget + effectiveRightTarget) / 2f;
        float effectiveSteer = (effectiveLeftTarget - effectiveRightTarget) / (2f * turnK);

        // --- ДВИЖЕНИЕ ---
        // Поступательное движение оставляем физическим (rb.MovePosition)
        Vector3 movement = transform.forward * effectiveGas * moveSpeed * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + movement);

        // --- ВНЕДРЕННЫЙ КОД ВРАЩЕНИЯ ---
        // Вращение вокруг локальной оси Y на основе эффективного руля и скорости поворота
        transform.Rotate(Vector3.up * (effectiveSteer * turnSpeed) * Time.fixedDeltaTime);
    }

    private float ApplyMotorLogic(float pwm)
    {
        float absPwm = Mathf.Abs(pwm);
        if (absPwm < motorDeadzone) return 0f;
        if (absPwm < minMotorPwm) return Mathf.Sign(pwm) * minMotorPwm;
        return pwm;
    }
}
