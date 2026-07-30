# FixedArm 20M from best catch weights

## Analysis
| Run | Sustained Catch% | Notes |
|-----|------------------|-------|
| **fixed_arm_30m_relay_x4** | **~65% peak / ~42% end** | 3 act pan-only, tilt −15°, 60°/s |
| fixed_arm_10m_relay_x4 | ~47% | same camera/action |
| fixed_arm_30m_catchfix_x6 | ~42% early | 4 act pan+tilt — incompatible |
| fixed_arm_50m_cam_x6 | ~3–14% | farmed dense reward |

## Recipe
- **Weights:** `--initialize-from=fixed_arm_30m_relay_x4` (ckpt ~30M)
- **Camera:** pan-only, tilt locked −15°, 60°/s (from relay)
- **Search:** ball memory + pan assist (from catchfix line)
- **Rewards:** catch +15 / miss −4 (catch-weighted)
- **Net:** 192/192 matching relay

## Run
`fixed_arm_20m_relayinit_x6` · 20M · 40×6

```bash
conda activate mlagents
export GFSX_ARENA_COUNT=40
mlagents-learn config_fixed.yaml \
  --run-id=fixed_arm_20m_relayinit_x6 \
  --env=Build/GFSX_Simulator \
  --num-envs=6 --no-graphics --force \
  --initialize-from=fixed_arm_30m_relay_x4
```

Watch **`GFSX/Grasp/CatchPercent`**.
