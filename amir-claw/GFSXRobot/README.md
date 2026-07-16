# GFS-X claw robot — drag-and-drop Unity package

This archive contains only the configured robot. It does **not** contain a
plane, ground, light, test ball, scene, or project settings.

## Install

1. Use Unity 6 or newer.
2. Unzip the archive.
3. Drag the complete `GFSXRobot` folder into the `Assets` area of your Unity
   Project window. Keep the folder together; do not copy only the prefab.
4. Wait for Unity to finish importing and compiling.
5. Open `Assets/GFSXRobot/Prefabs`.
6. Drag one prefab into the Hierarchy:

   - `GFS-X Robot - Full URP ML-Agents` preserves the real robot Camera,
     URP camera data, and ML-Agents `CameraSensorComponent`. Use this in a
     Unity 6 URP project with the package versions below.
   - `GFS-X Robot - Portable Core` preserves the complete robot, claw,
     colliders, servos, sensors, tracks, HUD, and real Camera, but removes the
     URP- and ML-Agents-specific camera components and uses built-in fallback
     materials. Use it when those packages are not installed.

## Full prefab requirements

- Unity `6000.0.79f1` or compatible Unity 6 release
- Universal Render Pipeline `17.0.4`
- Input System `1.19.0`
- ML-Agents `4.0.3`

The controls also contain a legacy-input fallback, so the Portable Core
prefab compiles and works when the Input System package is absent.

## Before pressing Play

- Add your own floor/terrain with a Collider. No plane is included, so the
  Rigidbody correctly falls under gravity in an empty scene.
- The robot's child `cam` is an enabled real Camera tagged `MainCamera`.
  Disable your scene's old Main Camera if you want the robot view.
- The packaged robot's AudioListener is disabled to avoid duplicate-listener
  warnings. Enable it only if your scene has no other AudioListener.

## Keyboard controls

- `W/S` or Up/Down arrows: forward/reverse
- `A/D` or Left/Right arrows: skid-steer turn
- `1/2`: shoulder servo S1
- `3/4`: elbow servo S2
- `5/6`: wrist-roll servo S3
- `7`: close jaws
- `8`: open jaws
- `0`: reset arm
- `J/L`: sensor tower pan S5
- `K/I`: camera tilt S6
- `U`: reset sensor head

Click the Game view once if the keyboard is controlling another Unity panel.

## Claw and editable limits

Select the robot root and edit `Gfsx Arm Rig Controller` in the Inspector.
The distributed binary claw uses:

- OPEN servo position: `35°`
- CLOSED servo position: `85°`
- open physical jaw-hinge angle: `20°`
- closed physical jaw-hinge angle: `0°`

Both jaw meshes have enabled convex MeshColliders. The package setup helper
adds only the `TargetBall` tag without replacing the recipient project's
TagManager. To test grasping, give an object a Rigidbody, a Collider, and the
`TargetBall` tag, then place it between the jaws.

## ML-Agents note

The Full prefab contains a Camera and `CameraSensorComponent`, but it does not
yet contain an ML-Agents `Agent` or `BehaviorParameters`. Those must be added
when the specific observation space, action space, rewards, and training task
are defined.

## Included / excluded

Included: robot hierarchy, split tracks, claw colliders, arm and sensor-head
rigs, camera mesh and real Camera, virtual sensors, scripts, custom Inspectors,
FBX, generated tread meshes/materials, and only the textures actually used.

Excluded: Ground, TargetBall, Directional Light, scenes, render settings,
ProjectSettings, Packages, ROS/URDF tools, unused ambient-occlusion textures,
the obsolete merged-treads child, and temporary test/export scripts.
