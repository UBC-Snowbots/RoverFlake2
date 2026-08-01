#!/usr/bin/env python3
"""
curve_gen.py
============
Generate standard curves for life detection assays from PySpectrometer2 CSV data,
and predict concentrations from unknown samples.

TWO MODES
---------

1. BUILD — fit a standard curve from known standards, save the model:

    python curve_gen.py build \\
        --blank blank.csv \\
        --standards 0,blank.csv 1,sample_1ppm.csv 2,sample_2ppm.csv 5,sample_5ppm.csv 10,sample_10ppm.csv 20,sample_20ppm.csv \\
        --wavelength 525 \\
        --unit ppm \\
        --name "Resazurin Assay" \\
        --model-out resazurin_model.json

2. PREDICT — load a saved model and predict concentration from an unknown sample:

    python curve_gen.py predict \\
        --sample unknown.csv \\
        --model resazurin_model.json

Run with --help for full options:

    python curve_gen.py build --help
    python curve_gen.py predict --help
"""

import os
import sys
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")   # non-interactive backend — safe for SSH / headless Pi
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter


# ─────────────────────────────────────────────────────────────────────────────
# DEFAULTS
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_WAVELENGTH    = 525.0
DEFAULT_TOLERANCE     = 1.0
DEFAULT_BLANK         = "blank.csv"
DEFAULT_UNIT          = "ppm"
DEFAULT_ASSAY_NAME    = "Life Detection Assay"
DEFAULT_MODEL_OUT     = "curve_model.json"
DEFAULT_SAVGOL_WINDOW = 11
DEFAULT_SAVGOL_POLY   = 3


# ─────────────────────────────────────────────────────────────────────────────
# SPECTRUM HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def load_spectrum(filepath, smooth=True,
                  window=DEFAULT_SAVGOL_WINDOW,
                  poly=DEFAULT_SAVGOL_POLY):
    """Load a PySpectrometer2 CSV and optionally apply Savitzky-Golay smoothing."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")
    df = pd.read_csv(filepath)
    df.columns = [c.strip() for c in df.columns]
    if "Wavelength" not in df.columns or "Intensity" not in df.columns:
        raise ValueError(
            f"{filepath}: expected columns 'Wavelength' and 'Intensity', "
            f"got {list(df.columns)}"
        )
    df = df.sort_values("Wavelength").reset_index(drop=True)
    if smooth and len(df) >= window:
        df["Intensity"] = savgol_filter(
            df["Intensity"].astype(float), window, poly
        )
    return df


def get_intensity(df, wavelength, tol=DEFAULT_TOLERANCE):
    """Return intensity at the closest wavelength within ±tol nm."""
    mask   = (df["Wavelength"] - wavelength).abs() <= tol
    subset = df[mask]
    if subset.empty:
        raise ValueError(
            f"No wavelength found within ±{tol} nm of {wavelength} nm. "
            f"Available range: {df['Wavelength'].min():.1f}–{df['Wavelength'].max():.1f} nm"
        )
    idx = (subset["Wavelength"] - wavelength).abs().idxmin()
    return float(df.loc[idx, "Intensity"])


def compute_absorbance(i_blank, i_sample):
    """Beer-Lambert: A = log10(I0 / I).  Clamps to avoid log(0)."""
    i_blank  = max(i_blank,  1.0)
    i_sample = max(i_sample, 1.0)
    ratio = i_blank / i_sample
    return np.log10(ratio) if ratio >= 1e-6 else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# CURVE FITTING
# ─────────────────────────────────────────────────────────────────────────────

def linear_model(x, m, b):
    return m * x + b

def quadratic_model(x, a, b, c):
    return a * x**2 + b * x + c


def fit_best_model(concentrations, absorbances):
    """
    Fit linear and quadratic models; return the better one.
    Quadratic accepted only if R² improves >1% AND curvature is physically meaningful.
    Returns: (model_fn, popt, r_squared, model_name)
    """
    x = np.array(concentrations, dtype=float)
    y = np.array(absorbances,    dtype=float)

    def r2(y_true, y_pred):
        ss_res = np.sum((y_true - y_pred) ** 2)
        ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
        return 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    popt_lin, _ = curve_fit(linear_model, x, y, p0=[1, 0])
    r2_lin = r2(y, linear_model(x, *popt_lin))

    r2_quad, popt_quad = None, None
    if len(x) >= 3:
        try:
            popt_quad, _ = curve_fit(quadratic_model, x, y, p0=[0, 1, 0])
            r2_quad = r2(y, quadratic_model(x, *popt_quad))
        except RuntimeError:
            pass

    if r2_quad is not None:
        improvement       = r2_quad - r2_lin
        x_range           = x.max() - x.min()
        quad_contribution = abs(popt_quad[0]) * x_range ** 2
        if improvement > 0.01 and quad_contribution > 0.05:
            return quadratic_model, popt_quad, r2_quad, "Quadratic"

    return linear_model, popt_lin, r2_lin, "Linear"


def predict_concentration(absorbance, model_name, popt):
    """Back-calculate concentration from absorbance using the fitted model."""
    if model_name == "Linear":
        m, b = popt
        return (absorbance - b) / m if m != 0 else float("nan")
    elif model_name == "Quadratic":
        a, b, c = popt
        discriminant = b ** 2 - 4 * a * (c - absorbance)
        if discriminant < 0 or a == 0:
            return float("nan")
        x1 = (-b + np.sqrt(discriminant)) / (2 * a)
        x2 = (-b - np.sqrt(discriminant)) / (2 * a)
        candidates = [v for v in [x1, x2] if v >= 0]
        return min(candidates) if candidates else float("nan")
    return float("nan")


# ─────────────────────────────────────────────────────────────────────────────
# MODEL SAVE / LOAD
# ─────────────────────────────────────────────────────────────────────────────

def save_model(path, model_name, popt, r2, wavelength, blank_csv, unit, assay_name):
    """Serialise the fitted model to a human-readable JSON file."""
    data = {
        "assay_name": assay_name,
        "unit":       unit,
        "wavelength": wavelength,
        "blank_csv":  blank_csv,
        "model_name": model_name,
        "popt":       list(popt),
        "r2":         r2,
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  Model saved → {path}")


def load_model(path):
    """Load a previously saved model JSON. Returns the data dict."""
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Model file not found: {path}\n"
            f"Run 'python standard_curve.py build ...' first to create it."
        )
    with open(path) as f:
        data = json.load(f)
    for key in ("model_name", "popt", "wavelength", "blank_csv", "unit"):
        if key not in data:
            raise ValueError(f"Model file {path} is missing key: '{key}'")
    return data


# ─────────────────────────────────────────────────────────────────────────────
# BUILD MODE
# ─────────────────────────────────────────────────────────────────────────────

def cmd_build(args):
    wavelength = args.wavelength
    blank_csv  = args.blank
    unit       = args.unit
    assay_name = args.name
    model_out  = args.model_out
    smooth     = not args.no_smooth

    print(f"\n{'='*60}")
    print(f"  {assay_name} — Building Standard Curve")
    print(f"  Measurement wavelength : {wavelength} nm")
    print(f"  Blank file             : {blank_csv}")
    print(f"  Model output           : {model_out}")
    print(f"{'='*60}\n")

    # Parse "concentration,filename" pairs
    standards = []
    for entry in args.standards:
        try:
            conc_str, csv_path = entry.split(",", 1)
            standards.append((float(conc_str), csv_path.strip()))
        except ValueError:
            print(f"  [ERROR] Cannot parse: '{entry}'")
            print(f"          Expected format: concentration,filename")
            print(f"          Example: 5,sample_5ppm.csv")
            sys.exit(1)

    # Load blank
    blank_df = load_spectrum(blank_csv, smooth=smooth)
    i_blank  = get_intensity(blank_df, wavelength)
    print(f"Blank → I₀ = {i_blank:.2f} at {wavelength} nm\n")

    # Process each standard
    records = []
    spectra = {"Blank": blank_df}

    for conc, csv_path in standards:
        if not os.path.exists(csv_path):
            print(f"  [WARN] Skipping missing file: {csv_path}")
            continue
        df  = load_spectrum(csv_path, smooth=smooth)
        i_s = get_intensity(df, wavelength)
        A   = compute_absorbance(i_blank, i_s)
        label = f"{conc} {unit}"
        print(f"  {label:15s}  I = {i_s:.2f}   A = {A:.4f}")
        records.append({"Concentration": conc, "Intensity": i_s,
                         "Absorbance": A, "File": csv_path})
        spectra[label] = df

    if len(records) < 2:
        print("\n  [ERROR] Need at least 2 valid standard files to fit a curve.")
        sys.exit(1)

    results_df     = pd.DataFrame(records)
    concentrations = results_df["Concentration"].values
    absorbances    = results_df["Absorbance"].values

    # Fit
    model_fn, popt, r2, model_name = fit_best_model(concentrations, absorbances)

    print(f"\n  Best fit : {model_name}   R² = {r2:.6f}")
    if model_name == "Linear":
        m, b = popt
        print(f"  Equation : A = {m:.6f} × C + {b:.6f}")
        if m > 0:
            print(f"  Detection limit (A ≈ 0.01) : ~{(0.01 - b) / m:.3f} {unit}")
    else:
        a, b, c = popt
        print(f"  Equation : A = {a:.4e}·C² + {b:.6f}·C + {c:.6f}")

    # Save model
    save_model(model_out, model_name, popt, r2,
               wavelength, blank_csv, unit, assay_name)

    # Save results CSV
    results_df["Predicted_Conc"] = results_df["Absorbance"].apply(
        lambda A: predict_concentration(A, model_name, popt)
    )
    results_df.to_csv(args.results_csv, index=False, float_format="%.5f")
    print(f"  Results  → {args.results_csv}")

    # Spectra overlay plot
    fig, ax = plt.subplots(figsize=(10, 5))
    cmap = plt.get_cmap("viridis", len(spectra))
    for i, (label, df) in enumerate(spectra.items()):
        ax.plot(df["Wavelength"], df["Intensity"],
                label=label, color=cmap(i), lw=1.5)
    ax.axvline(wavelength, color="red", linestyle="--", lw=1,
               label=f"λ = {wavelength} nm")
    ax.set_xlabel("Wavelength (nm)")
    ax.set_ylabel("Pixel Intensity (0–255)")
    ax.set_title(f"{assay_name} — Spectra Overlay")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_xlim(blank_df["Wavelength"].min(), blank_df["Wavelength"].max())
    ax.set_ylim(0, 260)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(args.plot_spectra, dpi=150)
    plt.close()
    print(f"  Spectra  → {args.plot_spectra}")

    # Standard curve plot
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(concentrations, absorbances, color="royalblue",
               zorder=5, s=60, label="Measured standards")
    x_fit = np.linspace(0, max(concentrations) * 1.1, 300)
    y_fit = model_fn(x_fit, *popt)
    ax.plot(x_fit, y_fit, color="firebrick", lw=2,
            label=f"{model_name} fit  (R² = {r2:.4f})")
    for row in results_df.itertuples():
        ax.annotate(f"{row.Concentration} {unit}",
                    (row.Concentration, row.Absorbance),
                    textcoords="offset points", xytext=(6, 4), fontsize=8)
    ax.set_xlabel(f"Concentration ({unit})")
    ax.set_ylabel(f"Absorbance (log₁₀ I₀/I)  at {wavelength} nm")
    ax.set_title(f"{assay_name} — Standard Curve")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    plt.tight_layout()
    plt.savefig(args.plot_curve, dpi=150)
    plt.close()
    print(f"  Curve    → {args.plot_curve}")

    print(f"\n{'='*60}")
    print(f"  Done.  To predict an unknown run:")
    print(f"    python curve_gen.py predict \\")
    print(f"        --sample unknown.csv --model {model_out}")
    print(f"{'='*60}\n")


# ─────────────────────────────────────────────────────────────────────────────
# PREDICT MODE
# ─────────────────────────────────────────────────────────────────────────────

def cmd_predict(args):
    model_data = load_model(args.model)

    model_name = model_data["model_name"]
    popt       = model_data["popt"]
    wavelength = model_data["wavelength"]
    unit       = model_data["unit"]
    assay_name = model_data["assay_name"]
    blank_csv  = args.blank if args.blank else model_data["blank_csv"]
    smooth     = not args.no_smooth

    print(f"\n{'='*60}")
    print(f"  {assay_name} — Predicting Unknown Concentrations")
    print(f"  Model      : {model_name}   (R² = {model_data['r2']:.6f})")
    print(f"  Wavelength : {wavelength} nm")
    print(f"  Blank      : {blank_csv}")
    print(f"{'='*60}\n")

    blank_df = load_spectrum(blank_csv, smooth=smooth)
    i_blank  = get_intensity(blank_df, wavelength)

    results = []
    for sample_csv in args.sample:
        if not os.path.exists(sample_csv):
            print(f"  [WARN] Skipping missing file: {sample_csv}")
            continue
        sample_df = load_spectrum(sample_csv, smooth=smooth)
        i_s  = get_intensity(sample_df, wavelength)
        A    = compute_absorbance(i_blank, i_s)
        conc = predict_concentration(A, model_name, popt)
        print(f"  {sample_csv}")
        print(f"    Intensity  = {i_s:.2f}")
        print(f"    Absorbance = {A:.4f}")
        print(f"    Predicted  = {conc:.4f} {unit}\n")
        results.append({"File": sample_csv, "Intensity": i_s,
                         "Absorbance": A, "Predicted_Conc": conc, "Unit": unit})

    if results and args.results_csv:
        pd.DataFrame(results).to_csv(args.results_csv, index=False,
                                      float_format="%.5f")
        print(f"  Results saved → {args.results_csv}")


# ─────────────────────────────────────────────────────────────────────────────
# ARGUMENT PARSER
# ─────────────────────────────────────────────────────────────────────────────

def build_parser():
    parser = argparse.ArgumentParser(
        prog="curve_gen.py",
        description="PySpectrometer2 standard curve builder and predictor.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # ── build ──────────────────────────────────────────────────────────────
    build = subparsers.add_parser(
        "build",
        help="Fit a standard curve from known-concentration CSVs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="""
Fit a standard curve from known standards and save the model to JSON.

Example:
  python curve_gen.py build \\
      --blank blank.csv \\
      --standards 0,blank.csv 1,sample_1ppm.csv 5,sample_5ppm.csv 10,sample_10ppm.csv \\
      --wavelength 525 --unit ppm --name "Resazurin Assay" --model-out resazurin.json
        """
    )
    build.add_argument("--blank",      default=DEFAULT_BLANK,      metavar="FILE",
                        help=f"Blank/reference CSV. Default: {DEFAULT_BLANK}")
    build.add_argument("--standards",  nargs="+", required=True,   metavar="CONC,FILE",
                        help="Space-separated concentration,filename pairs. e.g. 0,blank.csv 5,sample_5ppm.csv")
    build.add_argument("--wavelength", type=float, default=DEFAULT_WAVELENGTH, metavar="NM",
                        help=f"Measurement wavelength in nm. Default: {DEFAULT_WAVELENGTH}")
    build.add_argument("--unit",       default=DEFAULT_UNIT,        metavar="UNIT",
                        help=f"Concentration unit label. Default: {DEFAULT_UNIT}")
    build.add_argument("--name",       default=DEFAULT_ASSAY_NAME,  metavar="NAME",
                        help=f'Assay name for plot titles. Default: "{DEFAULT_ASSAY_NAME}"')
    build.add_argument("--model-out",  default=DEFAULT_MODEL_OUT,   metavar="FILE",
                        help=f"Output path for model JSON. Default: {DEFAULT_MODEL_OUT}")
    build.add_argument("--results-csv",default="standard_curve_results.csv", metavar="FILE",
                        help="Output path for per-standard results CSV.")
    build.add_argument("--plot-curve", default="standard_curve.png",  metavar="FILE",
                        help="Output path for standard curve plot.")
    build.add_argument("--plot-spectra",default="spectra_overlay.png", metavar="FILE",
                        help="Output path for spectra overlay plot.")
    build.add_argument("--no-smooth",  action="store_true",
                        help="Disable Savitzky-Golay smoothing.")

    # ── predict ────────────────────────────────────────────────────────────
    predict = subparsers.add_parser(
        "predict",
        help="Predict concentration of unknown sample(s) using a saved model.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description="""
Predict concentration for one or more unknown CSV files.

Example (single):
  python curve_gen.py predict --sample unknown.csv --model resazurin.json

Example (batch):
  python curve_gen.py predict \\
      --sample unknown1.csv unknown2.csv unknown3.csv \\
      --model resazurin.json --results-csv predictions.csv
        """
    )
    predict.add_argument("--sample",      nargs="+", required=True, metavar="FILE",
                          help="One or more unknown sample CSV files.")
    predict.add_argument("--model",        default=DEFAULT_MODEL_OUT, metavar="FILE",
                          help=f"Saved model JSON from 'build'. Default: {DEFAULT_MODEL_OUT}")
    predict.add_argument("--blank",        default=None,              metavar="FILE",
                          help="Override the blank CSV stored in the model (e.g. fresh blank taken today).")
    predict.add_argument("--results-csv",  default=None,              metavar="FILE",
                          help="Optional path to save predictions as CSV.")
    predict.add_argument("--no-smooth",    action="store_true",
                          help="Disable Savitzky-Golay smoothing.")

    return parser


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = build_parser()
    args   = parser.parse_args()

    if args.command == "build":
        cmd_build(args)
    elif args.command == "predict":
        cmd_predict(args)