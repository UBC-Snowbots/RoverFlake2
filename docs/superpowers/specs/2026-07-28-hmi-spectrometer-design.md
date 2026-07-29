# HMI Spectrometer Integration — Design

Date: 2026-07-28
Branch: `feat/aaronrhim/hmi_spectro` (off `main`)

## Goal

Bring the Science_RD spectrophotometer workflow — which is built on Les Wright's
[PySpectrometer2](https://github.com/leswright1977/PySpectrometer2) conventions — into
`rover_hmi_core` so standard curves can be built and unknown samples measured from the
rover HMI instead of a laptop shell.

## What Science_RD borrows from PySpectrometer2

Science_RD does not import PySpectrometer2; it replicates five conventions so its CSVs and
calibration stay interchangeable with the upstream tool:

| Borrowed | Detail |
|---|---|
| `readcal()` calibration | `caldata.txt` = 2 lines (pixel indices, then wavelengths); degree-2 fit for exactly 3 points, degree-3 for 4+; R² reported |
| CSV contract | `Wavelength,Intensity`; intensities clipped to 0–255 integers |
| Spectrum extraction | Mean of 3 rows around the frame's vertical centre |
| Capture geometry | 800×600 — must match the resolution calibration was performed at |
| Smoothing | Savitzky-Golay (scipy in Science_RD; hand-rolled upstream) |

Not used, and therefore out of scope: `wavelength_to_rgb`, `peakIndexes`,
`generateGraticule`, and the live OpenCV GUI.

**Known gap:** upstream `writecal()` (interactive calibration) has no Science_RD equivalent.
`caldata.txt` must currently be produced by running PySpectrometer2 elsewhere. Closing this
is phase 4, not a silent omission.

## Relationship to the existing HMI "Spectrophotometer"

`ScienceAnalysisModule` already renders a section labelled Spectrophotometer. That is a
**different instrument**: six per-vial absorbance floats in
`ScienceSensorData.spectro_absorbance`, published by the science embedded node. This work
adds the camera + diffraction-grating instrument alongside it. Naming keeps them distinct
("Spectrometer" for the camera instrument) so operators do not confuse the two.

## Architecture

The HMI is C++/Qt; the Science_RD pipeline is Python (numpy/pandas/scipy/matplotlib). The
HMI **shells out to the Python** rather than reimplementing it, matching the intent already
recorded in `src/rover_hmi_core/src/science/TASKS.md` and keeping one source of numerical
truth. A C++ port would duplicate validated science code in a second language and drift.

Components:

- `src/rover_hmi_core/scripts/spectrometer/` — vendored `curve_gen.py`,
  `capture_and_predict.py`, `fake_data_gen.py`. Science_RD is a separate repo that will not
  exist on the rover, so the scripts must live here.
- `python_runner.{h,cpp}` — small reusable `QProcess` wrapper: run a script, stream stdout
  into a log pane, report exit code. Kept separate so the module stays small.
- `spectrometer_module.{h,cpp}` — the `GuiModule`, section `Science`.
- `spectro_paths.{h,cpp}` — resolves the in-repo data directory (see Storage).

## UI

One module, three panels:

1. **Capture** — frame count, Pi/USB camera toggle, device index. Runs
   `capture_and_predict.py`; shows absorbance and predicted concentration.
2. **Build curve** — blank file, `concentration,file` standards, wavelength, unit, assay
   name. Runs `curve_gen.py build`; shows R² and the fitted equation.
3. **Spectrum view** — renders the captured `Wavelength,Intensity` CSV natively in Qt for an
   immediate plot, and displays the matplotlib PNGs (`standard_curve.png`,
   `spectra_overlay.png`) that `curve_gen` already writes.

## Storage

Models, captures and results live under `src/rover_hmi_core/config/spectrometer/` — in-repo,
never `~/.config`. The directory is resolved from `$ROVERFLAKE_ROOT` when set, else from the
build-time source path, mirroring how `LayoutStore` resolves layouts on the `new_hmi` branch.
(`LayoutStore` does not exist on `main`, so this branch carries its own small resolver; if the
branches converge later, the two should be unified.)

## Error handling

Every failure names its remedy rather than surfacing a traceback:

- `caldata.txt` missing → "calibrate first" with the expected path.
- Python interpreter or a required module missing → detected at module start; buttons
  disabled with the reason shown.
- Camera absent or busy → surfaced from the script's stderr.
- Non-zero exit → stderr tail displayed in the log pane.

Long-running captures never block the Qt event loop — `QProcess` runs asynchronously.

## Testing

- `fake_data_gen.py` produces synthetic CSVs, so the whole build/predict pipeline is testable
  with no camera and no rover.
- The existing Science_RD datasets (`Rezasurin/`, `Biurete-V2/`) provide real-data regression
  checks: a rebuilt model should reproduce the R² recorded in their
  `standard_curve_results.csv`.
- Widget construction is smoke-tested headless via `QT_QPA_PLATFORM=offscreen`.

## Container dependency blocker

`matplotlib` is broken in `roverflake2:gpu`: it is built against numpy 1.x while the image
carries numpy 2.2.6, so importing it raises `numpy.core.multiarray failed to import`.
`curve_gen.py` cannot run there until this is fixed. numpy (2.2.6), pandas (2.3.3) and scipy
(1.8.0) import cleanly on their own. Fixing the Python environment is part of phase 1.

## Phasing

Each phase is a single-concern PR:

1. Vendored scripts, Python dependency fix, in-repo path resolver. No UI.
2. Capture + predict UI.
3. Curve building UI and visualisation (native plot + generated PNGs).
4. Optional: on-rover calibration, closing the `writecal` gap.
