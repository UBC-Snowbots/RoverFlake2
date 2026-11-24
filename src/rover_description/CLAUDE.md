# rover_description

Holds the URDFs for every robot. Add new robots here instead of creating separate `*_urdf` packages.

## Layout

- `urdf/` — one file per robot (`mini_rover.urdf`, `chassis.urdf` = big rover).
- `meshes/chassis/`, `meshes/mini_rover/`, `meshes/arm/` — robot-specific geometry.
- `meshes/misc/` — sensors (IMU, GPS, LiDAR). Files are prefixed by robot (`mini_rover_imu.STL`)
  because each was exported in its own assembly's link frame, so they can't be shared across robots as-is.
- Mesh paths: `package://rover_description/meshes/<subdir>/<file>.STL`. Mesh filenames are snake_case.
- View any URDF: `ros2 launch rover_description display_chassis.launch.py urdf:=mini_rover.urdf` (default: `chassis.urdf`)

## SolidWorks exports

Exports come out as ROS 1 packages named after the assembly (e.g. `UpdatedMiniRoverV8`).
Don't port the package. Move the `.urdf` into `urdf/` and the meshes into the right `meshes/` subdir,
rewrite the `package://<AssemblyName>/meshes/...` paths, and throw away the rest
(catkin CMakeLists/package.xml, `.launch`, CSV, export.log, joint_names yaml).
Exported joint limits are `0`/`0`, which locks the joint, so set real limits.

## Mini rover

- **Launch files:**
  - `mini_rover_description.launch.py` runs only robot_state_publisher (`use_sim_time` arg). It publishes
    `/robot_description` and TF. Include it from bringup on the real rover so Nav2 etc. have TF.
    Moving joints only get TF once something publishes `/joint_states`.
  - `display_mini_rover.launch.py` includes the description launch and adds joint_state_publisher_gui (`gui:=false` to skip) and RViz.
    It's for viewing only: the GUI publishes fake joint states, so don't run it alongside the real rover.

- **Wheels:** all six wheels share `wheel_left.STL` / `wheel_right.STL`. They're the same wheel, and the
  exported copies only differed by spin angle. The back-left mesh was missing from the export.
- **Bogies:** plain revolutes with hand-set limits.
- **Rockers and differential:** one degree of freedom, modelled with `<mimic>`:
  - `RockerL_joint` is the free (parent) joint.
  - `RockerR_joint` mimics it with `multiplier="-1"`. This is exact by symmetry.
  - `Differential_joint` (yaw, axis z) mimics it with `multiplier="0.25"`, worked out from the meshes:
    a link rod runs roughly along x from a tab on top of each rocker (0.054 m above the pivot,
    0.0524 m effective lever) to near the end of the bar (arm ~0.21 m; bar half-length 0.225 m).
    0.0524 / 0.21 ≈ 0.25. It's a small-angle linearization, and the sign was checked against both sides.
  - A rocker is the parent so that the linkage approximation only affects the cosmetic bar, not the
    rockers/wheels that feed TF. Swap the parent to `Differential_joint` (rockers at `±1/k`) only if
    the real sensor ends up on the differential pivot. The parent should be whatever is physically measured.
  - Mimic is one-directional; joints can't mimic each other (a cycle is invalid).
  - Rocker and differential limits are placeholders (±0.35 rad).
- **LiDAR (Scanse Sweep V1):** `LiDAR_Link` origin is the *top* of the sensor head. `lidar_scan_link`
  (child, z = -0.0204) is the scan plane at the lens centre: measured from the mesh, and it matches the
  datasheet (beam 33 mm above the base of a 52.8 mm sensor) to within 0.6 mm. The driver's `frame_id`
  should be `lidar_scan_link`. Its +x is assumed to point at the Sweep's 0° (status LED); this hasn't
  been verified on the real mount. The bottom ~9 mm of the mesh is a mount/spacer, not the sensor.
- **Who applies mimic:** joint_state_publisher(_gui) and ros2_control (Jazzy) apply it.
  robot_state_publisher does not. A custom `/joint_states` publisher must publish `RockerR = -RockerL`
  itself, and if both rockers are measured, publish one combined value like `(L - R) / 2`.
  Gazebo ignores `<mimic>`. The Unity sim must enforce the coupling itself (constraint or script); not done yet.
- **Open issue:** the exported base frame looks odd. Left wheel joints sit at y ≈ -0.10, but the wheel
  meshes extend toward +y, so check the axes when cleaning up link/joint names (still SolidWorks names).

## Big rover (`chassis.urdf`)

Same structure and joint names as the mini rover: `RockerL/R_joint` (y, mimic R = -L), `BogieL/R_joint` (y),
`WF/WM/WB{L,R}_joint` (continuous, y), and `Differential_joint` (z, mimics RockerL). The SolidWorks exports
in `rover_cad/Chassis/URDF/` are unusable as-is:
- every joint has a 0/0 limit;
- there are no wheels;
- the bogie joint is coaxial with the rocker pivot;
- the jan25/LiDAR bogie meshes are 16 mm stubs;
- the differential bar is rotated about 77°;
- the Nov-2025 `Chassis.SLDASM` export bundles the whole assembly, several CAD revisions overlaid, into every link mesh.

`chassis.urdf` was built by hand from them:
- **Base frame, sensors, LiDAR, PTZ:** taken from `Chassis_LiDAR.SLDASM` (Feb 2026), which is the jan25 export plus LiDAR/PTZ.
  `base_link` is the SolidWorks origin: front-bottom of the frame, frame spans x -0.6..0, +x forward, +y left.
- **Pivots and axles:** measured from the old export's meshes (circle fits, matching L/R to within 1 mm):
  - rocker pivot (-0.25, ±0.2319, 0.1032);
  - bogie pivot (-0.506, 0.013);
  - axles at x = 0.097 (front, on the rocker), -0.302 and -0.706 (on the bogie), z ≈ -0.095.
- **Visual meshes:** `rocker_*`, `bogie_*`, `differential_bar` and `diff_rod_*` were cut out of a *single* mesh of the old
  export (`Right Bogie.STL`, which holds the whole assembly) by connected component and position. They were then
  snapped to a 0.5 mm grid and re-expressed in the new link frames. Don't merge or dedupe triangles across the
  old export's link meshes: the overlaid revisions share triangles, and splitting them leaves sawtooth fragments
  (the artifacts seen in RViz on the first version).
  They're visual only, close enough to the real parts; re-export from SolidWorks to replace them.
- **Differential coupling:** `multiplier="-0.3"` = -(0.1127 m rod-ball height above the rocker pivot) / (0.375 m bar-ball |y|).
  Rods run along x. It's a small-angle linearization. The sign was checked with joint_state_publisher:
  rocker +0.2 moves the bar end +0.0225 m in x, against an expected +0.0224 m.
- **Rods (`meshes/chassis/diff_rod_{left,right}.STL`):** not in the URDF, because they hang between two moving links. They're for Unity.
  - Each rod covers the tube plus both rod-end sockets. The balls stay on the rocker/bar meshes.
  - Mesh origin = rocker-end ball centre, axes aligned with `base_link` (at zero pose).
  - Ball centres in `base_link`: rocker end (-0.25, ±0.35375, 0.21585), bar end (-0.53573, ±0.375, 0.21747).
  - In Unity: pin the origin to the rocker ball and aim at the bar ball (or use two ball joints).
  - Left is the right rod mirrored in y. The real left rod uses left-hand threads, which you can't see.
- **Placeholders:**
  - wheels: the visual is the mini rover wheel scaled ×1.527 to r = 0.1 m (from `drive_control` `WHEEL_RADIUS_METERS`),
    with its axle stub trimmed off, giving a 0.069 m width. It's centred on the joint at |y| = 0.443, just outside
    the 0.408 m hub face. Collision is a matching cylinder;
  - rocker/bogie/differential limits;
  - rocker/bogie/wheel inertials.
  Get the real wheel size from the 26-27 wheel CAD.
- Same mimic caveats as the mini rover: robot_state_publisher, Gazebo and Unity don't apply mimic.
