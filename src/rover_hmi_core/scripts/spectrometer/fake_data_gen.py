import pandas as pd
import numpy as np

rng = np.random.default_rng(seed=42)  # reproducible

WAVELENGTHS     = np.linspace(400, 700, 640)
ABSORPTION_PEAK = 545
PEAK_WIDTH      = 15
BLANK_BASELINE  = 210
DARK_LEVEL      = 3   # real sensors have a small dark current

# Concentrations → max dip fraction (Beer-Lambert: A ∝ C, so dip ∝ C)
# At 20 ppm we want ~80% absorption (dip_fraction=0.8), so scale linearly
MAX_CONC        = 20
MAX_DIP         = 0.30

STANDARDS = [0, 1, 2, 5, 10, 20]  # ppm

# ── Dark scan ─────────────────────────────────────────────────────────────
dark_intensity = DARK_LEVEL + rng.integers(0, 2, len(WAVELENGTHS))
pd.DataFrame({
    'Wavelength': WAVELENGTHS.round(1),
    'Intensity':  dark_intensity.clip(0, 255).astype(int)
}).to_csv('dark.csv', index=False)

# ── Blank scan (dark-corrected baseline + noise) ───────────────────────────
# Simulate a smooth illumination curve (slightly brighter in the middle)
illumination = BLANK_BASELINE + 20 * np.exp(-((WAVELENGTHS - 550)**2) / (2 * 100**2))
blank_noise  = rng.integers(-2, 3, len(WAVELENGTHS))
blank_raw    = (illumination + blank_noise).clip(0, 255).astype(int)

pd.DataFrame({
    'Wavelength': WAVELENGTHS.round(1),
    'Intensity':  blank_raw
}).to_csv('blank.csv', index=False)

# ── Gaussian absorption profile (shape, not amplitude) ────────────────────
peak_shape = np.exp(-((WAVELENGTHS - ABSORPTION_PEAK)**2) / (2 * PEAK_WIDTH**2))

# ── Standard series ────────────────────────────────────────────────────────
for conc in STANDARDS:
    dip_fraction = (conc / MAX_CONC) * MAX_DIP          # Beer-Lambert scaling
    sample_noise = rng.integers(-2, 3, len(WAVELENGTHS))
    sample_raw   = blank_raw * (1 - dip_fraction * peak_shape) + sample_noise
    
    fname = 'blank.csv' if conc == 0 else f'sample_{conc}ppm.csv'
    pd.DataFrame({
        'Wavelength': WAVELENGTHS.round(1),
        'Intensity':  sample_raw.round().clip(0, 255).astype(int)
    }).to_csv(fname, index=False)
    
    # Verify expected absorbance matches Beer-Lambert prediction
    i0   = float(blank_raw[np.argmin(np.abs(WAVELENGTHS - ABSORPTION_PEAK))])
    i_s  = float(sample_raw[np.argmin(np.abs(WAVELENGTHS - ABSORPTION_PEAK))])
    A_expected = np.log10(i0 / max(i_s, 1))
    print(f"  {conc:>4} ppm  →  A ≈ {A_expected:.4f}")

print("\nFiles written: dark.csv, blank.csv, sample_1ppm.csv … sample_20ppm.csv")