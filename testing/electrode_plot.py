#!/usr/bin/env python3
"""
Electrode Characterization Visualization
Generates a 4-subplot figure from 24 electrode tests (E-01 through E-24).
All tested at -80dB on Port1 Dev9, day-old resin batch.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from pathlib import Path

# ---------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------
electrodes = list(range(1, 25))
ids = [f"E-{i:02d}" for i in electrodes]

snr   = [27.8, 20.3, 27.4, 14.9, 27.4, 42.8, 18.0, 21.3,
         38.4, 22.9, 23.7, 15.0, 17.0, 43.0, 14.5, 23.4,
         16.1, 27.0, 19.1, 23.3, 17.1, 21.9, 29.4, 21.2]

hz60  = [6.46, 29.00, 84.32, 103.88, 31.76, 0.41, 73.31, 83.14,
         2.00, 11.89, 28.90, 49.93, 32.34, 0.64, 48.03, 9.30,
         44.42, 8.30, 23.07, 12.24, 32.68, 15.13, 6.37, 4.94]

rms   = [101.1, 397.1, 1686.9, 1308.1, 639.6, 38.8, 1420.0, 1579.9,
         43.8, 234.4, 587.8, 831.3, 501.5, 34.9, 601.7, 187.1,
         783.1, 143.2, 376.3, 247.7, 564.9, 240.5, 106.1, 83.2]

grades = ["F", "F", "F", "F", "F", "A*", "F", "F",
          "B", "F", "F", "F", "F", "A",  "F", "F",
          "F", "F", "F", "F", "F", "F",  "F", "F"]

# Fresh resin baseline
baseline_snr  = 45.8
baseline_60hz = 0.48
baseline_rms  = 27.8

# Grade colors
grade_colors = {"A*": "#00e676", "A": "#66bb6a", "B": "#42a5f5", "C": "#ffa726", "F": "#ef5350"}

def bar_color(g):
    return grade_colors.get(g, grade_colors["F"])

bar_cols = [bar_color(g) for g in grades]

# ---------------------------------------------------------------------------
# Figure setup
# ---------------------------------------------------------------------------
plt.style.use("dark_background")
fig, axes = plt.subplots(2, 2, figsize=(16, 12), dpi=100)
fig.suptitle("Electrode Characterization: Day-Old Resin Batch (n=24)",
             fontsize=18, fontweight="bold", color="white", y=0.97)
fig.subplots_adjust(hspace=0.34, wspace=0.28, top=0.91, bottom=0.06, left=0.07, right=0.97)

ax1, ax2, ax3, ax4 = axes.flat

# ---------------------------------------------------------------------------
# 1) Top-left: 60Hz Pickup by Electrode Number
# ---------------------------------------------------------------------------
ax1.bar(electrodes, hz60, color=bar_cols, edgecolor="white", linewidth=0.3, zorder=3)
ax1.axhline(baseline_60hz, color="#00e676", ls="--", lw=1.4, label=f"Fresh baseline ({baseline_60hz} mV)")
ax1.axhline(5.0, color="#ffa726", ls="--", lw=1.2, label="Grade C threshold (5 mV)")
ax1.set_xlabel("Electrode #", fontsize=11)
ax1.set_ylabel("60 Hz Pickup (mV)", fontsize=11)
ax1.set_title("60 Hz Pickup by Electrode Number", fontsize=13, fontweight="bold")
ax1.set_xticks(electrodes)
ax1.set_xticklabels(electrodes, fontsize=8)
ax1.set_xlim(0.25, 24.75)
ax1.legend(loc="upper left", fontsize=8, framealpha=0.7)
ax1.grid(axis="y", alpha=0.2, zorder=0)

# ---------------------------------------------------------------------------
# 2) Top-right: 60Hz Pickup Histogram
# ---------------------------------------------------------------------------
bins = [0, 2, 5, 10, 20, 40, 60, 80, 110]
bin_centers = [(bins[i] + bins[i+1]) / 2 for i in range(len(bins) - 1)]

# Count per grade per bin for stacked histogram
grade_order = ["A*", "A", "B", "F"]
grade_counts = {g: [] for g in grade_order}
for i in range(len(bins) - 1):
    lo, hi = bins[i], bins[i + 1]
    for g in grade_order:
        count = sum(1 for v, gr in zip(hz60, grades) if lo <= v < hi and gr == g)
        grade_counts[g].append(count)

bottoms = np.zeros(len(bins) - 1)
bar_width = [bins[i+1] - bins[i] for i in range(len(bins) - 1)]
for g in grade_order:
    counts = grade_counts[g]
    if sum(counts) == 0:
        continue
    ax2.bar(bin_centers, counts, width=[w * 0.85 for w in bar_width],
            bottom=bottoms, color=grade_colors[g], edgecolor="white",
            linewidth=0.5, label=g, zorder=3)
    bottoms += np.array(counts)

# Annotate total count on each bin
totals = [sum(grade_counts[g][i] for g in grade_order) for i in range(len(bins) - 1)]
for i, total in enumerate(totals):
    if total > 0:
        ax2.text(bin_centers[i], total + 0.15, str(total),
                 ha="center", va="bottom", fontsize=9, fontweight="bold", color="white")

ax2.set_xlabel("60 Hz Pickup (mV)", fontsize=11)
ax2.set_ylabel("Count", fontsize=11)
ax2.set_title("60 Hz Pickup Distribution", fontsize=13, fontweight="bold")
ax2.set_xticks(bins)
ax2.set_xticklabels([str(b) for b in bins], fontsize=8)
ax2.set_ylim(0, max(totals) + 1.5)
ax2.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
ax2.legend(loc="upper right", fontsize=8, framealpha=0.7)
ax2.grid(axis="y", alpha=0.2, zorder=0)

# ---------------------------------------------------------------------------
# 3) Bottom-left: SNR vs 60Hz Scatter
# ---------------------------------------------------------------------------
for g in grade_order:
    gx = [v for v, gr in zip(hz60, grades) if gr == g]
    gy = [v for v, gr in zip(snr, grades) if gr == g]
    if gx:
        ax3.scatter(gx, gy, c=grade_colors[g], s=60, edgecolors="white",
                    linewidth=0.5, label=g, zorder=5)

# Threshold lines
ax3.axhline(43, color="#66bb6a", ls="--", lw=1.0, alpha=0.6, label="A threshold (43 dB)")
ax3.axhline(35, color="#42a5f5", ls="--", lw=1.0, alpha=0.6, label="B threshold (35 dB)")
ax3.axhline(25, color="#ffa726", ls="--", lw=1.0, alpha=0.6, label="C threshold (25 dB)")
ax3.axvline(1.0, color="#66bb6a", ls=":", lw=1.0, alpha=0.5)
ax3.axvline(2.0, color="#42a5f5", ls=":", lw=1.0, alpha=0.5)
ax3.axvline(5.0, color="#ffa726", ls=":", lw=1.0, alpha=0.5)

# Baseline marker
ax3.scatter([baseline_60hz], [baseline_snr], c="white", s=100, marker="*",
            edgecolors="white", linewidth=0.5, zorder=6, label=f"Fresh baseline")

ax3.set_xscale("log")
ax3.set_xlabel("60 Hz Pickup (mV) [log]", fontsize=11)
ax3.set_ylabel("SNR (dB)", fontsize=11)
ax3.set_title("SNR vs 60 Hz Pickup", fontsize=13, fontweight="bold")
ax3.set_xlim(0.2, 200)
ax3.set_ylim(10, 50)

# Quadrant labels
ax3.text(0.35, 46, "A zone", fontsize=8, color="#66bb6a", alpha=0.8, fontweight="bold")
ax3.text(0.35, 37, "B zone", fontsize=8, color="#42a5f5", alpha=0.8, fontweight="bold")
ax3.text(0.35, 27, "C zone", fontsize=8, color="#ffa726", alpha=0.8, fontweight="bold")
ax3.text(0.35, 17, "F zone", fontsize=8, color="#ef5350", alpha=0.8, fontweight="bold")
ax3.text(80, 46, "High noise", fontsize=7, color="gray", alpha=0.6, ha="center")

ax3.legend(loc="upper right", fontsize=7, framealpha=0.7, ncol=1)
ax3.grid(alpha=0.15, zorder=0)
ax3.xaxis.set_major_formatter(ticker.ScalarFormatter())
ax3.xaxis.set_minor_formatter(ticker.NullFormatter())
ax3.set_xticks([0.5, 1, 2, 5, 10, 20, 50, 100])
ax3.get_xaxis().set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:g}"))

# ---------------------------------------------------------------------------
# 4) Bottom-right: SNR by Electrode Number
# ---------------------------------------------------------------------------
ax4.bar(electrodes, snr, color=bar_cols, edgecolor="white", linewidth=0.3, zorder=3)
ax4.axhline(baseline_snr, color="#00e676", ls="--", lw=1.4, label=f"Fresh baseline ({baseline_snr} dB)")
ax4.axhline(43, color="#66bb6a", ls=":", lw=1.2, alpha=0.7, label="A threshold (43 dB)")
ax4.axhline(35, color="#42a5f5", ls=":", lw=1.2, alpha=0.7, label="B threshold (35 dB)")
ax4.axhline(25, color="#ffa726", ls=":", lw=1.2, alpha=0.7, label="C threshold (25 dB)")
ax4.set_xlabel("Electrode #", fontsize=11)
ax4.set_ylabel("SNR (dB)", fontsize=11)
ax4.set_title("SNR by Electrode Number", fontsize=13, fontweight="bold")
ax4.set_xticks(electrodes)
ax4.set_xticklabels(electrodes, fontsize=8)
ax4.set_xlim(0.25, 24.75)
ax4.set_ylim(0, 52)
ax4.legend(loc="upper right", fontsize=8, framealpha=0.7)
ax4.grid(axis="y", alpha=0.2, zorder=0)

# ---------------------------------------------------------------------------
# Summary annotation
# ---------------------------------------------------------------------------
pass_count = sum(1 for g in grades if g in ("A*", "A", "B"))
fail_count = len(grades) - pass_count
summary = (f"Pass: {pass_count}/24  ({pass_count/24*100:.0f}%)  |  "
           f"Fail: {fail_count}/24  ({fail_count/24*100:.0f}%)  |  "
           f"Median 60Hz: {sorted(hz60)[len(hz60)//2]:.1f} mV  |  "
           f"Median SNR: {sorted(snr)[len(snr)//2]:.1f} dB")
fig.text(0.5, 0.015, summary, ha="center", fontsize=10, color="#b0bec5",
         fontstyle="italic")

# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------
out_path = Path(__file__).parent / "electrode_distribution.png"
fig.savefig(out_path, dpi=100, facecolor=fig.get_facecolor(), bbox_inches="tight")
print(f"Saved: {out_path}")
plt.close(fig)
