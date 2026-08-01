#!/usr/bin/env python3
"""
capture_and_predict.py
======================
Headless script for the Raspberry Pi that:
  1. Captures a spectrum from the Pi camera (replicating PySpectrometer2's pipeline)
  2. Saves it as a CSV in PySpectrometer2 format (Wavelength, Intensity)
  3. Runs it through curve_gen.py's prediction pipeline
  4. Saves a results CSV with the predicted concentration

Requires:
  - curve_gen.py in the same directory
  - A trained model JSON from: python curve_gen.py build ...
  - caldata.txt in the same directory (written by PySpectrometer2 during calibration)
  - picamera2 (Pi camera) OR opencv (USB camera)

Usage
-----
  # Pi camera:
  python capture_and_predict.py --model yeast_model.json --output sample.csv

  # USB camera:
  python capture_and_predict.py --model yeast_model.json --output sample.csv --usb --device 0

  # Override blank on the day:
  python capture_and_predict.py --model yeast_model.json --output sample.csv --blank todays_blank.csv

  # Specify how many frames to average (more = less noise, default 10):
  python capture_and_predict.py --model yeast_model.json --output sample.csv --frames 20
"""

import os
import sys
import json
import argparse
import datetime
import numpy as np
import csv

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

# Row averaging: PySpectrometer2 averages 3 rows around the centre of the frame.
# Increase for noisier cameras.
AVERAGING_ROWS = 3

# Camera capture resolution — must match what PySpectrometer2 was calibrated with.
# PySpectrometer2 uses 800x600 for USB cameras.
CAPTURE_WIDTH  = 800
CAPTURE_HEIGHT = 600

# Calibration file written by PySpectrometer2. Must exist before running this script.
CALDATA_FILE = "caldata.txt"

# ─────────────────────────────────────────────────────────────────────────────
# CALIBRATION
# ─────────────────────────────────────────────────────────────────────────────

def load_calibration(caldata_path: str) -> np.poly1d:
    """
    Read caldata.txt written by PySpectrometer2 and return a polynomial
    that maps pixel index → wavelength (nm).

    PySpectrometer2 writes caldata.txt as exactly TWO lines:
        Line 0: comma-separated pixel numbers  e.g.  581, 230, 631, 487
        Line 1: comma-separated wavelengths    e.g.  375, 513, 546.5, 611.6

    This exactly replicates the readcal() function in specFunctions.py:
      - 3 points  → 2nd order polynomial
      - 4+ points → 3rd order polynomial
    """
    if not os.path.exists(caldata_path):
        raise FileNotFoundError(
            f"Calibration file not found: {caldata_path}\n"
            f"Run PySpectrometer2 and perform calibration first."
        )

    with open(caldata_path, 'r') as f:
        lines = f.readlines()

    if len(lines) < 2:
        raise ValueError(
            f"caldata.txt must have 2 lines (pixels then wavelengths). "
            f"Only {len(lines)} line(s) found. Re-run calibration in PySpectrometer2."
        )

    # Line 0: pixel numbers (integers)
    pixels      = [int(x.strip())   for x in lines[0].strip().split(',')]
    # Line 1: wavelengths (floats)
    wavelengths = [float(x.strip()) for x in lines[1].strip().split(',')]

    if len(pixels) != len(wavelengths):
        raise ValueError(
            f"caldata.txt has {len(pixels)} pixel values but "
            f"{len(wavelengths)} wavelength values. Re-run calibration."
        )
    if len(pixels) < 3:
        raise ValueError(
            f"caldata.txt has only {len(pixels)} calibration point(s). "
            f"Need at least 3. Re-run calibration in PySpectrometer2."
        )

    print(f"  Calibration points loaded: {len(pixels)}")
    for p, w in zip(pixels, wavelengths):
        print(f"    pixel {p} → {w} nm")

    # Fit polynomial — identical degree logic to PySpectrometer2
    degree = 3 if len(pixels) > 3 else 2
    poly   = np.poly1d(np.polyfit(pixels, wavelengths, degree))

    # Report R² so miscalibration is obvious
    predicted = poly(np.array(pixels))
    corr      = np.corrcoef(wavelengths, predicted)[0, 1]
    r2        = corr ** 2
    print(f"  Degree-{degree} polynomial fit, R² = {r2:.6f}")

    return poly


def pixel_to_wavelengths(poly: np.poly1d, width: int) -> np.ndarray:
    """Map every pixel column across the frame width to a wavelength."""
    return poly(np.arange(width))


# ─────────────────────────────────────────────────────────────────────────────
# FRAME CAPTURE
# ─────────────────────────────────────────────────────────────────────────────

def capture_frame_picam(n_frames: int = 10) -> np.ndarray:
    """
    Capture n_frames from the Pi camera and return their mean as a
    grayscale (H x W) array. Requires picamera2.
    """
    try:
        from picamera2 import Picamera2
    except ImportError:
        raise ImportError(
            "picamera2 not found. Install with: sudo apt install python3-picamera2\n"
            "Or use --usb for a USB camera."
        )

    picam = Picamera2()
    config = picam.create_still_configuration(
        main={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()

    import time
    time.sleep(2)  # let the camera warm up and auto-expose

    frames = []
    for _ in range(n_frames):
        frame = picam.capture_array()   # shape: (H, W, 3), uint8
        gray  = frame.mean(axis=2)      # convert to grayscale by averaging RGB channels
        frames.append(gray)
        time.sleep(0.05)

    picam.stop()
    picam.close()
    return np.mean(frames, axis=0)      # shape: (H, W)


def capture_frame_usb(device: int = 0, n_frames: int = 10) -> np.ndarray:
    """
    Capture n_frames from a USB camera and return their mean as a
    grayscale (H x W) array. Requires opencv.
    """
    try:
        import cv2
    except ImportError:
        raise ImportError("opencv not found. Install with: pip install opencv-python")

    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open USB camera at device {device}")

    import time
    time.sleep(1)   # warm up

    frames = []
    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            raise RuntimeError("Failed to capture frame from USB camera")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(float)
        frames.append(gray)
        time.sleep(0.05)

    cap.release()
    return np.mean(frames, axis=0)


# ─────────────────────────────────────────────────────────────────────────────
# SPECTRUM EXTRACTION
# ─────────────────────────────────────────────────────────────────────────────

def extract_spectrum(frame: np.ndarray, avg_rows: int = AVERAGING_ROWS) -> np.ndarray:
    """
    Extract a 1D intensity spectrum from the captured frame by averaging
    AVERAGING_ROWS rows around the vertical centre, replicating PySpectrometer2's
    3-row averaging approach.

    Returns a 1D array of intensity values, one per pixel column.
    """
    h, w    = frame.shape
    centre  = h // 2
    half    = avg_rows // 2
    row_start = max(centre - half, 0)
    row_end   = min(centre + half + 1, h)
    strip   = frame[row_start:row_end, :]   # shape: (avg_rows, W)
    return strip.mean(axis=0)               # shape: (W,)


# ─────────────────────────────────────────────────────────────────────────────
# CSV SAVE
# ─────────────────────────────────────────────────────────────────────────────

def save_spectrum_csv(wavelengths: np.ndarray, intensities: np.ndarray,
                      output_path: str):
    """
    Save spectrum to CSV in PySpectrometer2 format:
        Wavelength,Intensity
        400.1,210
        ...
    Intensities are clipped to 0–255 and rounded to integers to match the
    real instrument output.
    """
    intensities_int = np.clip(intensities, 0, 255).round().astype(int)
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Wavelength", "Intensity"])
        for wl, iv in zip(wavelengths, intensities_int):
            writer.writerow([round(float(wl), 1), int(iv)])
    print(f"  Spectrum saved → {output_path}")


# ─────────────────────────────────────────────────────────────────────────────
# PREDICTION (calls into curve_gen logic directly)
# ─────────────────────────────────────────────────────────────────────────────

def run_prediction(sample_csv: str, model_path: str,
                   blank_override: str | None = None) -> dict:
    """
    Load curve_gen's model and predict concentration from the saved CSV.
    Imports curve_gen functions directly so everything stays in one process.
    """
    # Add the script's directory to path so curve_gen can be imported
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)

    try:
        import curve_gen as cg
    except ImportError:
        raise ImportError(
            "curve_gen.py not found in the same directory as this script."
        )

    model_data = cg.load_model(model_path)
    model_name = model_data["model_name"]
    popt       = model_data["popt"]
    wavelength = model_data["wavelength"]
    unit       = model_data["unit"]
    assay_name = model_data["assay_name"]
    blank_csv  = blank_override if blank_override else model_data["blank_csv"]

    blank_df  = cg.load_spectrum(blank_csv)
    sample_df = cg.load_spectrum(sample_csv)

    i_blank = cg.get_intensity(blank_df,  wavelength)
    i_sample = cg.get_intensity(sample_df, wavelength)
    A        = cg.compute_absorbance(i_blank, i_sample)
    conc     = cg.predict_concentration(A, model_name, popt)

    return {
        "assay":       assay_name,
        "model":       model_name,
        "wavelength":  wavelength,
        "unit":        unit,
        "blank_csv":   blank_csv,
        "sample_csv":  sample_csv,
        "intensity":   round(i_sample, 2),
        "absorbance":  round(A, 5),
        "predicted":   round(conc, 5),
        "r2":          model_data["r2"],
    }


# ─────────────────────────────────────────────────────────────────────────────
# RESULTS SAVE
# ─────────────────────────────────────────────────────────────────────────────

def save_results(result: dict, results_path: str):
    """Append the prediction result to a CSV log file."""
    file_exists = os.path.exists(results_path)
    with open(results_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=result.keys())
        if not file_exists:
            writer.writeheader()
        writer.writerow(result)
    print(f"  Result appended → {results_path}")


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        prog="capture_and_predict.py",
        description="Headless spectrum capture and concentration prediction for Raspberry Pi.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Pi camera, yeast assay:
  python capture_and_predict.py --model yeast_model.json --output sample.csv

  # USB camera:
  python capture_and_predict.py --model yeast_model.json --output sample.csv --usb --device 0

  # Fresh blank taken today:
  python capture_and_predict.py --model yeast_model.json --output sample.csv --blank todays_blank.csv

  # Average 20 frames for less noise:
  python capture_and_predict.py --model yeast_model.json --output sample.csv --frames 20
        """
    )

    parser.add_argument("--model",      required=True,  metavar="FILE",
                        help="Model JSON from 'python curve_gen.py build ...'")
    parser.add_argument("--output",     default=None,   metavar="FILE",
                        help="Filename to save the captured spectrum CSV. "
                             "Default: auto-generated with timestamp.")
    parser.add_argument("--blank",      default=None,   metavar="FILE",
                        help="Override blank CSV stored in model (use a fresh blank).")
    parser.add_argument("--results",    default="predictions_log.csv", metavar="FILE",
                        help="CSV file to append prediction results to. "
                             "Default: predictions_log.csv")
    parser.add_argument("--caldata",    default=CALDATA_FILE, metavar="FILE",
                        help=f"PySpectrometer2 calibration file. Default: {CALDATA_FILE}")
    parser.add_argument("--frames",     type=int, default=10, metavar="N",
                        help="Number of frames to average for noise reduction. Default: 10")
    parser.add_argument("--usb",        action="store_true",
                        help="Use a USB camera instead of the Pi camera.")
    parser.add_argument("--device",     type=int, default=0, metavar="N",
                        help="USB camera device number (e.g. 0 for /dev/video0). Default: 0")

    args = parser.parse_args()

    # Auto-generate output filename if not specified
    if args.output is None:
        timestamp   = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"capture_{timestamp}.csv"

    print(f"\n{'='*55}")
    print(f"  Headless Capture + Predict")
    print(f"  Model    : {args.model}")
    print(f"  Output   : {args.output}")
    print(f"  Camera   : {'USB (device ' + str(args.device) + ')' if args.usb else 'Pi Camera'}")
    print(f"  Frames   : {args.frames}")
    print(f"{'='*55}\n")

    # ── 1. Load calibration ───────────────────────────────────────────────
    print("Loading calibration...")
    poly       = load_calibration(args.caldata)
    wavelengths = pixel_to_wavelengths(poly, CAPTURE_WIDTH)

    # ── 2. Capture frame(s) ────────────────────────────────────────────────
    print(f"Capturing {args.frames} frame(s)...")
    if args.usb:
        frame = capture_frame_usb(device=args.device, n_frames=args.frames)
    else:
        frame = capture_frame_picam(n_frames=args.frames)
    print(f"  Frame shape: {frame.shape}")

    # ── 3. Extract spectrum ────────────────────────────────────────────────
    print("Extracting spectrum...")
    intensities = extract_spectrum(frame)

    # ── 4. Save CSV ────────────────────────────────────────────────────────
    print("Saving spectrum CSV...")
    save_spectrum_csv(wavelengths, intensities, args.output)

    # ── 5. Predict ─────────────────────────────────────────────────────────
    print("Predicting concentration...")
    result = run_prediction(args.output, args.model, blank_override=args.blank)

    print(f"\n  {'─'*40}")
    print(f"  Assay      : {result['assay']}")
    print(f"  Intensity  = {result['intensity']:.2f}")
    print(f"  Absorbance = {result['absorbance']:.5f}")
    print(f"  Predicted  = {result['predicted']:.5f} {result['unit']}")
    print(f"  {'─'*40}\n")

    # ── 6. Save result ─────────────────────────────────────────────────────
    result["timestamp"] = datetime.datetime.now().isoformat()
    save_results(result, args.results)

    print(f"\n{'='*55}")
    print(f"  Done.")
    print(f"{'='*55}\n")


if __name__ == "__main__":
    main()