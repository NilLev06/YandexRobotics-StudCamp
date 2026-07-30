# FixedArm catch-fix + camera smooth (+20M resume)

## Current run
`fixed_arm_30m_catchfix_x6` · 30M · 40×6 — catch-weighted rewards.

## After 30M (auto)
Waiter rebuilds with camera fixes and **resumes same run to 50M** (+20M):
- reset tilt **0°** (was −15° down)
- pan/tilt speeds **45/35 °/s** (was 100/80)
- command deadzone **0.15** + light jitter penalty
- downward tilt floor **−12°**

Logs:
- `Logs/fixed_arm_30m_catchfix_x6.log` (first 30M)
- `Logs/wait_then_resume_50m_camfix.log` (waiter)
- `Logs/fixed_arm_30m_catchfix_x6_resume50m.log` (+20M)

```bash
# waiter already queued; manual resume if needed:
mlagents-learn config_fixed.yaml \
  --run-id=fixed_arm_30m_catchfix_x6 \
  --env=Build/GFSX_Simulator --num-envs=6 --no-graphics --resume
```

Watch **`GFSX/Grasp/CatchPercent`**.
