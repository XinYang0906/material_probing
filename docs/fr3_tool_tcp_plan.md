# FR3 Tool TCP Plan

See `docs/fr3_ros2_moveit_architecture.md` for the current end-to-end FR3
probing architecture and entrypoint contract.

FR3 probing now has a configurable `spatula_tip` link attached after
`fr3_hand_tcp` in the robot model. The default transform is an estimate from
the MuJoCo spatula approximation:

```text
parent: fr3_hand_tcp
child:  spatula_tip
xyz:    0 0 0.2366
rpy:    0 0 0
```

This is only a starting value. Replace it with a measured tool calibration
before real contact.

Planned integration steps:

1. Launch FR3 MoveIt with `use_spatula_tip:=true` and calibrated
   `spatula_tip_xyz` / `spatula_tip_rpy`.
2. Confirm `fr3_arm` uses a chain ending at `spatula_tip` in the generated SRDF.
3. Keep `tip_link:=spatula_tip` for probing once the frame is calibrated.
4. Verify TF with `ros2 run tf2_ros tf2_echo fr3_link0 spatula_tip`.
5. Run all probing entries with `tip_link:=spatula_tip` on fake hardware before
   any real contact.
6. Add force/torque feedback and contact abort thresholds before using
   insertion, compression, drag, or lift-detach stages on hardware.

Current inclined insertion status:

- `run_fr3_probe.py` supports the FR3 `inclined_insertion` stage sequence:
  `APPROACH -> TILT -> INSERT -> HOLD -> RETRACT -> UNTILT`.
- The entry commands `spatula_tip` pose goals and limits tilt to 10-15 deg.
- Insert depth still has the shared 0.03 m hard limit.
- This remains a fake-hardware trajectory test until the spatula TCP is
  measured and contact force/torque aborts are in place.
