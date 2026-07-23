# Обучение GFS-X (FixedArm) — Этап 2 эстафеты

Агент: езда + **pan (S5)**, клешня фикс (S1=−46° / S2=10° / S3=−90° / S4=35–85), tilt ≈ −15°.

## Сценарий эпизода

1. Ровер у **стартового** края (красный куб `RedHomeCube`)  
2. Мяч у **противоположного** края, **вне начального FOV** камеры  
3. **8** коробок фиксированно  
4. Seek → зона мяча → захват → **возврат к красному кубу** с мячом  

## Конфиг

| Параметр | Значение |
|----------|----------|
| Behavior | `GFSX_Brain_Fixed` |
| `max_steps` | **30 000 000** |
| Run id | `fixed_arm_30m_relay_x4` |
| Env | 4 × headless |
| Эпизод | до 6000 steps |

Метрика: `GFSX/Grasp/CatchPercent`.

## Запуск

```bash
conda activate mlagents
cd /home/vladislavdauer/PycharmProjects/YandexRobotics-StudCamp
export DISPLAY=:0
export MONO_THREADS_SUSPEND=preemptive

mlagents-learn config_fixed.yaml \
  --run-id=fixed_arm_30m_relay_x4 \
  --env=Build/GFSX_Simulator \
  --num-envs=4 \
  --no-graphics \
  --force
```

Валидация: **GFS-X → Validate Fixed Arm Now**.
