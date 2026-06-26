# FAST_LIO_LOCALIZATION_HUMANOID Integration

This directory is a trimmed ROS 2 Humble integration of the humanoid
FAST-LIO + Open3D localization stack.

The upstream media assets, sample map, nested Git metadata, runtime logs, and
large FAST-LIO documentation assets are intentionally omitted from this
workspace copy.

## Packages

- `fast_lio_humanoid`: the upstream humanoid FAST-LIO variant, renamed so it can
  coexist with the workspace default `fast_lio` package.
- `open3d_loc_humanoid`: Open3D global localization against an offline point
  cloud map.

## Build

Build the default CPU stack first:

```bash
./scripts/build_cpu_real.sh
```

Build the optional humanoid localization stack only when Open3D is available:

```bash
Open3D_DIR=/abs/path/to/Open3D ./scripts/build_humanoid_localization.sh
```

The Open3D CMake config path can also be passed through colcon:

```bash
./scripts/build_humanoid_localization.sh --cmake-args -DOpen3D_DIR=/abs/path/to/Open3D
```

## Launch

The default launch behavior is unchanged. Enable the humanoid backend
explicitly:

```bash
ros2 launch me_nav2_bringup cpu_real_nav.launch.py \
  localization_backend:=humanoid \
  prior_pcd_file:=/abs/path/to/map.pcd
```

`open3d_loc_humanoid` consumes the `prior_pcd_file` path through its `path_map`
parameter. Both `.pcd` and `.ply` maps can be used if Open3D supports the file.

## Topics

The humanoid backend keeps the upstream public localization outputs:

- `/localization_3d`
- `/localization_3d_confidence`
- `/localization_3d_delay_ms`
- `map -> odom` TF

Debug point cloud topics are namespaced under `/humanoid_loc/...` to avoid
conflicts with Nav2 `/map` and `/scan`.
