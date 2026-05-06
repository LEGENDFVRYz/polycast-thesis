"""
Standalone FFT spectrum analyser for UWB anchor ranges.

Reads one or more uwb_range_report.csv files, computes the power spectrum
of each anchor's raw signal, and prints the dominant oscillation frequency.
Also saves a 2x2 spectrum plot as uwb_spectrum.png.

Usage:
    python -m background.pipelines.preprocess.uwb.analyze_anchor_spectrum
    python -m background.pipelines.preprocess.uwb.analyze_anchor_spectrum --csv s1.csv s2.csv
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def load_raw(paths: list[str]) -> dict:
    raw = {i: [] for i in range(4)}
    for p in paths:
        with open(p, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                for i in range(4):
                    try:
                        v = float(row[f'raw_{i}'])
                        if v > 0.05:
                            raw[i].append(v)
                    except (KeyError, ValueError):
                        pass
    return {i: np.array(raw[i], dtype=np.float32) for i in range(4)}


def dominant_frequencies(signal: np.ndarray, fs: float,
                          n_top: int = 5) -> list[tuple[float, float]]:
    """Return top-n (frequency_hz, power) pairs above 0.5 Hz."""
    # Remove DC (mean) before FFT
    s = signal - signal.mean()
    N = len(s)
    fft_vals = np.abs(np.fft.rfft(s)) ** 2
    freqs    = np.fft.rfftfreq(N, d=1.0 / fs)

    # Only look above 0.5 Hz (below is just slow pen drift)
    mask = freqs >= 0.5
    fft_masked = fft_vals.copy()
    fft_masked[~mask] = 0.0

    top_idx = np.argsort(fft_masked)[-n_top:][::-1]
    return [(float(freqs[i]), float(fft_masked[i])) for i in top_idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", nargs="+", default=["uwb_range_report.csv"])
    parser.add_argument("--fs",  type=float, default=100.0,
                        help="UWB sample rate Hz")
    args = parser.parse_args()

    print(f"[Spectrum] Loading {len(args.csv)} file(s)...")
    raw = load_raw(args.csv)

    print("\n── Dominant oscillation frequencies (above 0.5 Hz) ──────────")
    results = {}
    for i in range(4):
        sig = raw[i]
        if len(sig) < 256:
            print(f"  A{i}: insufficient data")
            continue
        top = dominant_frequencies(sig, args.fs, n_top=3)
        results[i] = top
        freqs_str = "  |  ".join(f"{f:.2f} Hz (pwr={p:.0f})" for f, p in top)
        print(f"  A{i}: {freqs_str}")

    # ── Plot ──────────────────────────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt

        fig, axs = plt.subplots(2, 2, figsize=(14, 8))
        fig.suptitle("UWB Anchor Range — Power Spectrum (DC removed)", fontsize=14)
        axs_flat = axs.flatten()

        for i in range(4):
            ax = axs_flat[i]
            sig = raw[i]
            if len(sig) < 256:
                ax.set_title(f"A{i} — no data")
                continue

            s = sig - sig.mean()
            N = len(s)
            fft_vals = np.abs(np.fft.rfft(s)) ** 2
            freqs    = np.fft.rfftfreq(N, d=1.0 / args.fs)

            # Only plot 0–20 Hz — pen motion and multipath live here
            mask = (freqs >= 0.5) & (freqs <= 20.0)
            ax.plot(freqs[mask], fft_vals[mask], color='steelblue', linewidth=1.2)

            # Mark top peaks
            if i in results:
                for f, p in results[i][:2]:
                    ax.axvline(f, color='red', linestyle='--', alpha=0.7,
                               label=f'{f:.2f} Hz')

            ax.set_title(f"Anchor A{i}")
            ax.set_xlabel("Frequency (Hz)")
            ax.set_ylabel("Power")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        out = "uwb_spectrum.png"
        plt.savefig(out, dpi=150)
        print(f"\n[Spectrum] Plot saved → {out}")
        plt.show()

    except ImportError:
        print("[Spectrum] matplotlib not available — skipping plot")


if __name__ == "__main__":
    main()
