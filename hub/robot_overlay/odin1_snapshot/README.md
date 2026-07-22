# Odin1 driver deployment snapshot

This directory preserves only the Yunji deployment delta, not a copy of the
vendor repository. The observed upstream is
`https://github.com/manifoldsdk/odin_ros_driver.git`, tag `v0.13.0`, commit
`13aa528b1da581e2168ac858f8b144f0b4438a7a`.

`odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch` is the exact dirty-tree
diff observed at `/home/nyu/odin_ws/src/odin_ros_driver` on 2026-07-22. It:

- boots firmware 0.13.1 in mode 0, starts the requested sensor streams, then
  switches to mode 1 so RGB/DTOF/IMU rates do not remain zero;
- selects `custom_map_mode: 1`;
- enables the ten-second `/odin1/cloud_slam` RViz decay display.

Reconstruction starts from the exact commit and applies the patch once:

```bash
git clone https://github.com/manifoldsdk/odin_ros_driver.git
cd odin_ros_driver
git checkout 13aa528b1da581e2168ac858f8b144f0b4438a7a
git apply --check /path/to/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch
git apply /path/to/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch
```

The device-specific `calib.yaml` is deliberately not duplicated here. Obtain
it from serial `O1-P070100205` and verify SHA-256
`c8cbd48bd8f8b08b8f174f557faf48649ee1101a3dfe0daf82ceae3832d7c23d`
before building. A calibration from a different serial must not be substituted.

Classification: upstream identity and working-tree bytes are observed; the
patch is an exact captured deployment delta. It contains no simulator data,
firmware image, binary SDK, model weight or credential.
