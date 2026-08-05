# HMI "Tasks" Section — Design

Date: 2026-08-04. Approved by Aaron in-session.
Working doc — delete before branch is finalized (per repo convention).

## Goal

Add a "Tasks" sidebar section to `rover_hmi_core` with operator tools for CIRC
2026 competition tasks, so nobody does symbol manipulation in their head or
alt-tabs to a PDF mid-run.

Sources: https://circ.cstag.ca/2026/tasks/ and the Vending Machine Technician
Manual rev 1.1 (Snack Run supporting doc).

## Modules (all `sectionName() == "Tasks"`, `defaultVisible() == false`)

### 1. Vending Decoder — `VendingDecoderModule`, hint `main`, no ROS
Three chained stages in one panel (Snack Run, Err2 hardware fault):
- **7-seg clicker** (`SevenSegWidget`, separate file): click the segments seen
  lit on the machine; shows `0b0gfedcba` byte, hex, decimal, ASCII char.
  "→ Grid" pushes the byte into the XOR grid.
- **XOR grid**: 4 columns × N rows (start 3, add/remove) of byte cells
  (binary `0b...`/bare binary/`0x..` hex accepted). Footer: per-column XOR in
  binary + decoded ASCII, plus the combined string large (e.g. "UddU" →
  switch sequence).
- **Free converter**: one input row, live: binary/hex byte groups → text, or
  plain text → binary (for pre-M3c raw-binary selection displays).

### 2. Morse — `MorseModule`, hint `right`
Thin client of the existing `rover_vision` pipeline
(`morse_camera_pub_node.py` → `cam_0/morse_led_brightness` Int32 @30fps;
`morse_decoder_node.py` → `/morse_decoded` String, CIRC T=66.7 ms):
- Live LED dot + brightness bar with 220 threshold mark; StaleMonitor "NO
  SIGNAL" when the pipeline is down.
- Decoded text: current word large; completed words appended to a history log
  (the decoder republishes the growing word and resets at word gaps).
- Manual decode: type `.-` groups (space-separated, `/` = word gap) → text.
- Encode: type text → grouped dots/dashes, large, for keying the vault code.
- `toggleCallback()` drops both subscriptions while hidden.
- Shared code table in `tasks/morse_code.h` (mirrors the Python node's table).

### 3. Task Timers — `TaskTimersModule`, hint `right`, no ROS
- Add/remove named countdown timers (RoverCooked orders): mm:ss spinboxes +
  name; per-row start/pause/reset/remove; green → yellow (≤30 s) → flashing
  red (expired). One 200 ms QTimer drives all rows.
- Cadence helpers (Snack Run): a 5 s credit-tap cycle bar that flashes at each
  cycle start, and a TAP button showing the interval since the previous press
  (green ≤1 s) for percussive-maintenance rhythm.

### 4. Task Reference — `TaskReferenceModule`, hint `bottom`, no ROS
- QComboBox over `config/tasks/reference/*.md`, rendered with
  `QTextBrowser::setMarkdown`, reload button. Content editable without
  recompiling. Seeded with one file per task (error codes Err0–4, bypass 4321,
  key dimensions/rules).
- Package root resolved like `spectro_paths` (env `ROVERFLAKE_ROOT`, then baked
  `ROVER_HMI_CORE_SOURCE_DIR`); read-only, so no ownership adoption needed.

## Plumbing
- New dirs `src/tasks/`, `include/rover_hmi_core/tasks/`, `config/tasks/reference/`.
- `plugins.xml`: four `<class>` entries under `<!-- Tasks -->`.
- `CMakeLists.txt`: sources added to `rover_hmi_modules`, include dir added to
  its PRIVATE include list. No host/tiling changes.
- No Q_OBJECT anywhere (lambdas + paintEvent overrides only).
- Ephemeral state (grid, timers) not persisted; Task Reference saves the
  selected doc via `saveState()`.

## Verification
- `colcon build --packages-select rover_hmi_core` in the rover container.
- Visual pass and morse live-decode (needs vision nodes + blinking LED) are
  Aaron's bench calls; manual `.-` entry and all converters testable on desk.
