# Запуск обучения GFS-X (Fixed / Mobile)

Руководство по запуску двух режимов обучения: **фиксированная клешня** и **подвижная рука**.  
Команды рассчитаны на Linux; окружение conda — `mlagents`.

---

## Сцены и behavior

| Режим | Сцена Unity | Behavior name | Continuous actions |
|-------|-------------|---------------|-------------------|
| Фиксированная клешня | `Assets/Scenes/P3_DigitalTwin_FixedArm.unity` | `GFSX_Brain_Fixed` | 3 (газ, руль, камера) |
| Подвижная рука | `Assets/Scenes/P2_DigitalTwin.unity` | `GFSX_Brain_Mobile` | 5 (+ S1, S2; S3 locked) |

Конфиг обучения: [`config.yaml`](config.yaml) — оба behavior уже описаны.

Перед первым запуском (или после смены сцен):

```bash
# Unity batchmode — создаёт/обновляет P3 и настраивает Behavior Parameters
/home/vladislavdauer/Unity/Hub/Editor/6000.5.3f1/Editor/Unity \
  -batchmode -nographics -quit \
  -projectPath /home/vladislavdauer/PycharmProjects/YandexRobotics-StudCamp \
  -executeMethod SetupTrainingScenes.SetupBatch
```

---

## Подготовка окружения

```bash
conda activate mlagents
cd /home/vladislavdauer/PycharmProjects/YandexRobotics-StudCamp
```

TensorBoard (в отдельном терминале):

```bash
conda activate mlagents
cd /home/vladislavdauer/PycharmProjects/YandexRobotics-StudCamp
tensorboard --logdir results --port 6006 --bind_all
```

Откройте в браузере: **http://localhost:6006**

### TensorBoard — что смотреть

| График | Где в UI | Зачем |
|--------|----------|-------|
| `Environment/Cumulative Reward` | Scalars → ваш `run-id` | Растёт ли средняя награда за эпизод |
| `Environment/Episode Length` | Scalars | Уменьшается ли время до захвата мяча |
| `Policy/Learning Rate` | Scalars | Плановое снижение LR (linear schedule) |
| `Losses/Policy Loss`, `Losses/Value Loss` | Scalars | Нет ли взрыва loss |
| `Policy/Entropy` | Scalars | Исследование: слишком быстрое падение → застревание |

Логи пишутся в `results/<run-id>/` (файлы `events.out.tfevents.*`).  
Если запущено несколько обучений (`fixed_arm_*`, `mobile_arm_*`), в TensorBoard можно включить/выключить нужные run в левой панели **Runs**.

> **Совет:** обновление графиков — раз в ~20k шагов (`summary_freq` в `config.yaml`). Обновите страницу или включите автообновление в TensorBoard.

---

## Вариант A — обучение из Unity Editor (с графикой)

1. Откройте нужную сцену (`P2` или `P3`).
2. Убедитесь, что на роботе **Behavior Type = Default** (обучение, не Inference).
3. Запустите обучение:

```bash
# Фиксированная клешня
mlagents-learn config.yaml --run-id=fixed_arm_editor --force

# Подвижная рука
mlagents-learn config.yaml --run-id=mobile_arm_editor --force
```

4. В Unity нажмите **Play**.

> В сцене при старте `ArenaRuntimeBootstrap` создаёт **40 параллельных арен** в одном процессе Unity.  
> Итого: **1 Unity Editor ≈ 40 агентов**.

---

## Вариант B — headless (без графики, Linux build)

### 1. Сборка симулятора

```bash
/home/vladislavdauer/Unity/Hub/Editor/6000.5.3f1/Editor/Unity \
  -batchmode -nographics -quit \
  -projectPath /home/vladislavdauer/PycharmProjects/YandexRobotics-StudCamp \
  -executeMethod BuildSimulator.BuildLinux
```

Бинарник: `Build/GFSX_Simulator`  
(в build попадают обе сцены; активная — та, что была первой в Build Settings при сборке).

Для **mobile** и **fixed** удобнее собирать отдельные билды или менять порядок сцен в **File → Build Settings** перед сборкой.  
Для быстрого старта можно обучать оба режима из Editor (вариант A).

### 2. Запуск без окна

```bash
# 1 процесс билда × 40 агентов внутри ≈ 40 параллельных роботов
mlagents-learn config.yaml \
  --run-id=fixed_arm_headless \
  --env=Build/GFSX_Simulator \
  --num-envs=1 \
  --no-graphics \
  --force
```

```bash
# 4 процесса × 40 агентов ≈ 160 параллельных роботов
mlagents-learn config.yaml \
  --run-id=mobile_arm_headless_x4 \
  --env=Build/GFSX_Simulator \
  --num-envs=4 \
  --no-graphics \
  --force
```

```bash
# 8 процессов — если хватает CPU/RAM (≈ 320 агентов)
mlagents-learn config.yaml \
  --run-id=mobile_arm_headless_x8 \
  --env=Build/GFSX_Simulator \
  --num-envs=8 \
  --no-graphics \
  --force
```

### Сводка по `--num-envs`

| `--num-envs` | Процессов Unity | Агентов (при 40 аренах) | Когда использовать |
|--------------|-----------------|-------------------------|--------------------|
| 1 | 1 | ~40 | Отладка, слабый ПК |
| 2 | 2 | ~80 | Баланс скорость/ресурсы |
| 4 | 4 | ~160 | Рекомендуемый headless |
| 8 | 8 | ~320 | Мощная машина |

Число арен задаётся `MultiArenaManager.arenaCount` (по умолчанию 40 через `ArenaRuntimeBootstrap`).

---

## Продолжение обучения (resume)

```bash
mlagents-learn config.yaml \
  --run-id=fixed_arm_headless \
  --env=Build/GFSX_Simulator \
  --num-envs=4 \
  --no-graphics \
  --resume
```

Без `--force` — продолжит с последнего checkpoint в `results/<run-id>/`.

---

## Экспорт модели

После обучения ONNX лежит в:

```
results/<run-id>/GFSX_Brain_Fixed.onnx
results/<run-id>/GFSX_Brain_Mobile.onnx
```

В Unity: **Behavior Parameters → Model** — перетащите нужный `.onnx`, **Behavior Type = Inference Only**.

---

## Поведение наград (кратко)

- **Рука (MobileArm):** S1/S2 у мяча; S3 locked. Клиренс пола без касания (~3 мм), raycast игнорирует коллайдеры робота (раньше бился в клешню и задирал S1 в небо). Жёсткий потолок `mobileTrainingS1Maximum=0°`. Штрафы high-arm / sky / IR occlusion только в MobileArm.
- **Камера vs корпус:** небольшой штраф за поворот корпуса; бонус за поворот камеры, когда мяч не по центру кадра; дополнительный штраф, если крутят корпусом при смещённом мяче.

---

## Частые проблемы

| Симптом | Решение |
|---------|---------|
| `Unable to start Unity environment` | Проверьте путь `--env`, права на `Build/GFSX_Simulator`, наличие `libgrpc_csharp_ext.x64.so` в `Build/GFSX_Simulator_Data/Managed/` |
| Behavior не учится | Behavior name в сцене должен совпадать с `config.yaml` (`GFSX_Brain_Fixed` / `GFSX_Brain_Mobile`) |
| Мало GPU/CPU | Уменьшите `--num-envs` до 1–2 |
| Нужна только одна арена | В Inspector `MultiArenaManager → Arena Count = 1` или отключите `ArenaRuntimeBootstrap` |
