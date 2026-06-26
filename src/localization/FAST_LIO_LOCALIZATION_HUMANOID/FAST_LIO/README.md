# fast_lio_humanoid

This package is the humanoid FAST-LIO variant used by
`open3d_loc_humanoid`. It is renamed from upstream `fast_lio` so it can coexist
with this workspace's default `fast_lio` package.

Use it through the integrated bringup:

```bash
ros2 launch me_nav2_bringup cpu_real_nav.launch.py localization_backend:=humanoid
```

For isolated debugging:

```bash
ros2 launch fast_lio_humanoid mapping.launch.py config_file:=mid360.yaml
```

The default bringup overrides LiDAR/IMU topics and FAST-LIO extrinsics from
`src/me_nav2_bringup/config/vehicle.yaml`.
