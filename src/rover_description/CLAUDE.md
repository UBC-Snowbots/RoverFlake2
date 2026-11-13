# rover_description

Holds the URDFs for every robot. Add new robots here instead of creating separate `*_urdf` packages.

## Layout

- `urdf/` — one file per robot (`mini_rover.urdf`, `Chassis_jan25.SLDASM.urdf`).
  `Chassis_LiDAR.SLDASM.urdf` and `ee_urdf_try3.urdf` are known broken: their meshes aren't in the repo.
- `meshes/chassis/`, `meshes/mini_rover/`, `meshes/arm/` — robot-specific geometry.
- `meshes/misc/` — sensors (IMU, GPS, LiDAR). Files are prefixed by robot (`mini_rover_imu.STL`)
  because each was exported in its own assembly's link frame, so they can't be shared across robots as-is.
- Mesh paths: `package://rover_description/meshes/<subdir>/<file>.STL`. Mesh filenames are snake_case.
- View any URDF: `ros2 launch rover_description display_chassis.launch.py urdf:=mini_rover.urdf`

## SolidWorks exports

Exports come out as ROS 1 packages named after the assembly (e.g. `UpdatedMiniRoverV8`).
Don't port the package. Move the `.urdf` into `urdf/` and the meshes into the right `meshes/` subdir,
rewrite the `package://<AssemblyName>/meshes/...` paths, and throw away the rest
(catkin CMakeLists/package.xml, `.launch`, CSV, export.log, joint_names yaml).
Exported joint limits are `0`/`0`, which locks the joint, so set real limits.

## Mini rover

- **Wheels:** all six wheels share `wheel_left.STL` / `wheel_right.STL`. They're the same wheel, and the
  exported copies only differed by spin angle. The back-left mesh was missing from the export.
- **Bogies:** plain revolutes with hand-set limits.
- **Rockers and differential:** one degree of freedom, modelled with `<mimic>`:
  - `RockerL_joint` is the free (parent) joint.
  - `RockerR_joint` mimics it with `multiplier="-1"`. This is exact by symmetry.
  - `Differential_joint` mimics it with a placeholder multiplier (`1`) that still needs tuning in RViz.
    It's roughly rocker link arm / differential arm and only accurate at small angles.
  - A rocker is the parent so that the linkage approximation only affects the cosmetic bar, not the
    rockers/wheels that feed TF. Swap the parent to `Differential_joint` (rockers at `±1/k`) only if
    the real sensor ends up on the differential pivot. The parent should be whatever is physically measured.
  - Mimic is one-directional; joints can't mimic each other (a cycle is invalid).
  - Rocker and differential limits are placeholders (±0.35 rad).
- **Who applies mimic:** joint_state_publisher(_gui) and ros2_control (Jazzy) apply it.
  robot_state_publisher does not. A custom `/joint_states` publisher must publish `RockerR = -RockerL`
  itself, and if both rockers are measured, publish one combined value like `(L - R) / 2`.
  Gazebo ignores `<mimic>`. The Unity sim must enforce the coupling itself (constraint or script); not done yet.
- **Open issue:** the exported base frame looks odd. Left wheel joints sit at y ≈ -0.10, but the wheel
  meshes extend toward +y, so check the axes when cleaning up link/joint names (still SolidWorks names).
