# Spectrometer scripts

Vendored from `Science_RD/CODE_Spectrophotometer` so the rover carries them without that
repo. The HMI's Spectrometer module runs these via `QProcess`; they stay plain CLI scripts
and remain runnable by hand.

| File | Purpose |
|---|---|
| `curve_gen.py` | Build a standard curve from known standards (`build`), or predict unknowns (`predict`) |
| `capture_and_predict.py` | Capture a spectrum from the camera, save CSV, predict concentration |
| `fake_data_gen.py` | Synthetic CSVs for testing with no hardware |

## Provenance

The measurement conventions come from Les Wright's
[PySpectrometer2](https://github.com/leswright1977/PySpectrometer2) (GPL-3.0), so CSVs and
calibration stay interchangeable with that tool:

- `caldata.txt` is two lines — pixel indices, then wavelengths. Degree-2 fit for exactly 3
  points, degree-3 for 4 or more (`readcal()`).
- Spectrum CSVs are `Wavelength,Intensity` with intensities clipped to 0–255 integers.
- A spectrum is the mean of 3 rows around the frame's vertical centre.
- Capture is 800×600 and must match the resolution used during calibration.

Upstream's interactive calibration (`writecal()`) has no equivalent here: `caldata.txt` must
be produced by running PySpectrometer2 itself. Capture cannot run until that file exists.

Keep edits in sync with Science_RD rather than diverging quietly — these scripts are the
single source of numerical truth for the assay.

## Dependencies

```bash
pip3 install -r requirements.txt
```

Already installed in the rover container. Camera capture additionally needs `picamera2`
(Pi camera) or `opencv-python` (USB).

## Verification

The pipeline reproduces Science_RD's recorded Rezasurin results exactly — rebuilding that
curve yields R² 0.980833 and slope 1.636312, matching its `yeast_model.json`. Use
`fake_data_gen.py` for a hardware-free smoke test:

```bash
python3 fake_data_gen.py
python3 curve_gen.py build --standards 0,blank.csv 1,sample_1ppm.csv 5,sample_5ppm.csv 10,sample_10ppm.csv --wavelength 525
python3 curve_gen.py predict --sample sample_10ppm.csv --model curve_model.json
```
