# GFS-X Fixed model: Unity to real robot

This directory contains the launch tooling for running the supplied
15,008,000-step `GFSX_Brain_Fixed` model on the physical GFS-X robot through
Unity. It does not run a Unity simulation: Unity performs inference and sends
guarded ROS commands to the real robot at `192.168.2.152`.

The project is pinned to:

- Unity `6000.0.79f1`
- ML-Agents `4.0.3`
- AI Inference `2.6.1`
- ROS-TCP Connector `0.7.0-preview` with the local reconnect/latency patch
- Fixed policy SHA-256
  `28919e156d31cd9d26ad22bde3a338b878ab0a9ea3dfde5557148ec0bbace488`
- Ball detector SHA-256
  `0e8baf30da0f988a54dde2a8ea089112bb1e2cd2ccde5155a5368521961242ca`

## Before step 1: safety and prerequisites

Use a clear floor area and keep the robot lifted until the drive-direction
test. Keep a hand ready on the power switch. `F10` or `Backspace` is the
software emergency stop, but it is not a substitute for removing power.

The Mac and robot must be on the same trusted LAN. Disable any VPN route that
captures `192.168.2.152`. ROS-TCP is not authenticated or encrypted; do not
expose ports `10000` or `8080` to the internet.

The existing robot image must already provide:

- Docker and image `ros_noetic_hardware_v2`;
- `/opt/ros/noetic` and `/root/catkin_ws` with `ros_tcp_endpoint`;
- XiaoRGeek hardware modules in `/root/XiaoRGeek`;
- OpenCV for Python 3 on the Pi;
- access to `/dev`, GPIO, I2C/PWM, the ultrasonic sensor, four IR inputs, and
  the USB camera.

Those manufacturer-specific drivers and the prebuilt Docker image are not in
this repository. The launch scripts check the image before starting.

This robot's verified custom wiring is:

| Input | Physical sensor |
| --- | --- |
| IO1 | right obstacle IR |
| IO2 | left obstacle IR |
| IO3 | claw IR |
| IO4 | rear IR |

The bridge maps logical S1-S6 to physical servo sockets `1,2,3,4,7,8`.
S4 is `0°` fully open and `50°` fully closed. S5 is limited to `15–160°`.

## Step 1: verify the checkout on the Mac

From the repository root:

```bash
cd amir-proj
./RealRobot/scripts/verify_local.sh
```

This performs Python and shell syntax checks, runs all 25 bridge safety tests,
and verifies the exact policy, YOLO, scene, and patched transport checksums.

## Step 2: deploy the Pi runtime

Turn on the robot and confirm SSH works:

```bash
ssh pi@192.168.2.152
```

Do not put a password in the repository. Set up an SSH key if repeated password
prompts are inconvenient.

From `amir-proj` on the Mac:

```bash
./RealRobot/scripts/deploy_to_pi.sh
```

The default destination is `/home/pi/gfsx-unity`. Override it only when
needed:

```bash
GFSX_PI_TARGET=pi@192.168.2.152 \
GFSX_PI_DIR=/home/pi/gfsx-unity \
./RealRobot/scripts/deploy_to_pi.sh
```

The deploy script intentionally never overwrites the live servo-state file.

## Step 3: record the current servo-angle estimate

On the Pi:

```bash
cd /home/pi/gfsx-unity
cp -n gfsx_servo_state.example.json gfsx_servo_state.json
nano gfsx_servo_state.json
```

Replace all six values with the best known current physical angles before the
first launch. This file is last-commanded state, not encoder feedback. A wrong
estimate can make the software believe a joint is already at its target.

The launcher refuses malformed values and enforces these ranges:

```text
S1 0..180   S2 0..180   S3 0..180
S4 0..50    S5 15..160  S6 0..180
```

The live state file is ignored by Git.

## Step 4: start and verify the Pi bridge

On the Pi:

```bash
cd /home/pi/gfsx-unity
./start_unity_ros_safe.sh
```

The launcher:

1. starts a read-only camera stream on port `8080`;
2. starts ROS core and ROS-TCP Endpoint on port `10000`;
3. starts the hardware bridge fully disarmed;
4. forces the command namespace to `/gfsx`;
5. refuses startup if GPIO, servo, or ultrasonic initialization fails;
6. refuses success unless the bridge subscribes to all four Unity command
   topics.

Useful checks:

```bash
docker exec gfsx_unity_ros tail -f /tmp/gfsx_bridge.log
curl http://192.168.2.152:8080/healthz
```

The launcher itself verifies subscribers for:

```text
/gfsx/cmd_vel
/gfsx/drive_enable
/gfsx/servo_targets_degrees
/gfsx/servo_enable
```

The Pi independently blocks translating motion when its filtered sensor
snapshot is older than `0.40 s`, when forward sonar clearance is `≤ 0.12 m`,
when either front IR is active, or when reversing with IO4 active. A pure
in-place pivot remains available so the robot can turn away; the normal drive
heartbeat and `0.45 s` command watchdog still apply.

To stop safely:

```bash
cd /home/pi/gfsx-unity
./stop_unity_ros_safe.sh
```

## Step 5: start ball perception on the Mac

Create the environment once:

```bash
cd amir-proj
conda env create -f RealRobot/mac/environment.yml
```

Start detection before pressing Play in Unity:

```bash
./RealRobot/mac/run_real_vision.sh
```

It reads `http://192.168.2.152:8080/`, validates the YOLO hash, detects only
class `0` (ball), and sends timestamped observations to Unity at
`127.0.0.1:5005`. It never publishes motor or servo commands. Press `q` in its
camera window to quit.

## Step 6: prepare the Fixed live scene in Unity

1. Open Unity Hub.
2. Click **Add** or **Open**.
3. Choose the repository's `amir-proj` folder.
4. Open it with Unity `6000.0.79f1`.
5. Wait for package import and script compilation to finish.
6. In the top menu click
   **GFS-X → ML Agents → Create or Repair FIXED Real Robot LIVE Scene**.
7. Then click
   **GFS-X → ML Agents → Validate FIXED Real Robot LIVE Scene**.
8. Run
   **GFS-X → ML Agents → Run FIXED Real Robot NETWORK-SILENT Smoke Test**.

The smoke test temporarily forces `127.0.0.1`, suppresses all physical ROS
publishers, executes the exact ONNX for at least five decisions, and requires
every physical gate to stay disarmed.

After it passes, open:

```text
Assets/GFSXRealRobot/Scenes/P4_RealRobotFixedLive.unity
```

The scene is intentionally a headless physical-I/O Agent host; it does not
need the simulated robot mesh.

## Step 7: start real inference, still disarmed

Confirm the Pi bridge and Mac detector are both running. Keep the tracks
lifted, then click Unity's **Play** triangle.

Every Play session starts disarmed. Do not arm anything until the Game view
HUD shows fresh:

- ROS connection;
- ultrasonic and IO1-IO4 readings;
- six-servo state and both gate acknowledgements;
- motor PWM telemetry;
- vision packets and a fresh policy action.

If any required stream becomes stale, the router blocks arming or disarms the
affected output.

## Step 8: authorize servos and drive

Each gate needs two key presses within the displayed eight-second confirmation
window. Pressing a key once does not authorize motion.

With the arm physically clear:

1. Press `F6`, read the proposed motion, wait for the displayed dwell, then
   press `F6` again. This authorizes fixed
   `S1/S2/S3/S6 = 70/180/90/75°`.
2. Wait until the HUD says the fixed arm is **AT TARGET**.
3. Press `F8` twice to authorize S5 camera pan.
4. Press `F7` twice to authorize S4 claw open/close.
5. With tracks still lifted, press `F9` twice to authorize drive.
6. Verify forward, reverse, left, right, and immediate stop before placing the
   robot on the floor.

Press `F10` or `Backspace` at any time to disarm drive and all servo channels.
Stopping Play also sends disarm messages.

## What is and is not validated

Validated locally:

- exact Fixed ONNX identity and Unity import contract;
- Fixed scene creation/preflight;
- network-silent inference smoke test with physical publishing suppressed;
- 25 Pi bridge logic tests, including the Pi-local stale/obstacle gate;
- absolute servo limits, motor dead-zone handling, watchdogs, and topic names;
- a rear IR input on IO4, in addition to IO1-IO3.

Not yet revalidated on the current network:

- a fresh end-to-end live session after the earlier VPN/routing problem;
- the new repository-provided camera streamer on the physical Pi;
- a successful ball pickup by this Fixed policy.

The Fixed training run recorded zero catches and never observed `holding=1`.
Treat pickup behavior as experimental. Observations 10-13 use applied-PWM dead
reckoning because this robot has no wheel encoders or IMU. Servo telemetry is
also commanded state, not measured joint feedback.

After `F9` is authorized, a fresh policy action can command the tracks
immediately. There is no separate mission-start gate. Yandex Rover/M2M HTTP
activation is not implemented in this package.

## Files deliberately excluded

The repository does not include Unity `Library`, `Temp`, `Logs`, `UserSettings`,
Python caches, run logs, checkpoints, `.pt` files, intermediate/old ONNX
models, legacy or Mobile physical scenes, backups, exports, passwords, or the
live Pi servo-state file.
