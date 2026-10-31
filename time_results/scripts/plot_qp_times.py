#!/usr/bin/env python3
"""
Plot QP computation times for the paper.

Usage:
    python3 plot_qp_times.py <file1.npy> [<file2.npy> ...] [--save] [--dir <output_dir>]

Generates:
  1. Box plot of QP solve times per robot (UAV and UGV side-by-side)
  2. Histogram of QP solve times (UAV vs UGV overlaid)
  3. If multiple files are given, a combined box plot across all trials

The script handles both the new per-robot format (list of arrays) and
the old flat-array format.
"""

import argparse
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive backend
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _normalise(data, key):
    """Return a list of 1-D arrays (one per robot) regardless of on-disk format."""
    arr = data[key]
    if isinstance(arr, np.ndarray) and arr.ndim == 1:
        return [arr]
    return [np.asarray(r, dtype=float) for r in arr]


def _load_file(path):
    """Load a single .npy file and return (drone_times, ugv_times) lists of arrays."""
    data = np.load(path, allow_pickle=True).item()
    drone_times = _normalise(data, "drone_qp_times") if "drone_qp_times" in data else []
    ugv_times = _normalise(data, "ugv_qp_times") if "ugv_qp_times" in data else []
    return drone_times, ugv_times


def _to_ms(t):
    return t * 1000.0


# ---------------------------------------------------------------------------
# Plot functions
# ---------------------------------------------------------------------------

def plot_box_per_robot(drone_times, ugv_times, save_path=None):
    """Box plot: one box per robot, UAVs and UGVs grouped."""
    fig, ax = plt.subplots(figsize=(8, 5))

    positions = []
    labels = []
    box_data = []
    colors = []

    n_drones = len(drone_times)
    n_ugvs = len(ugv_times)

    for i, t in enumerate(drone_times):
        if len(t) > 0:
            box_data.append(_to_ms(t))
            positions.append(i + 1)
            labels.append(f"UAV {i+1}")
            colors.append("#2196F3")

    offset = n_drones + 1  # gap between UAV and UGV groups
    for i, t in enumerate(ugv_times):
        if len(t) > 0:
            box_data.append(_to_ms(t))
            positions.append(offset + i + 1)
            labels.append(f"UGV {i+1}")
            colors.append("#FF5722")

    bp = ax.boxplot(box_data, positions=positions, widths=0.6, patch_artist=True,
                    showfliers=True, flierprops=dict(marker="o", markersize=3, alpha=0.5))

    for patch, color in zip(bp["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylabel("QP Solve Time (ms)", fontsize=12)
    ax.set_title("QP Computation Time per Robot", fontsize=13)
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Saved: {save_path}")
    plt.close(fig)


def plot_histogram(drone_times, ugv_times, save_path=None):
    """Overlaid histogram of UAV vs UGV QP times."""
    fig, ax = plt.subplots(figsize=(8, 5))

    drone_flat = np.concatenate([t for t in drone_times if len(t) > 0]) if drone_times else np.array([])
    ugv_flat = np.concatenate([t for t in ugv_times if len(t) > 0]) if ugv_times else np.array([])

    bins = np.linspace(0, max(
        _to_ms(drone_flat).max() if len(drone_flat) > 0 else 1,
        _to_ms(ugv_flat).max() if len(ugv_flat) > 0 else 1,
    ), 50)

    if len(drone_flat) > 0:
        ax.hist(_to_ms(drone_flat), bins=bins, alpha=0.6, label=f"UAV (n={len(drone_flat)})", color="#2196F3")
    if len(ugv_flat) > 0:
        ax.hist(_to_ms(ugv_flat), bins=bins, alpha=0.6, label=f"UGV (n={len(ugv_flat)})", color="#FF5722")

    ax.set_xlabel("QP Solve Time (ms)", fontsize=12)
    ax.set_ylabel("Frequency", fontsize=12)
    ax.set_title("Distribution of QP Computation Times", fontsize=13)
    ax.legend(fontsize=11)
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Saved: {save_path}")
    plt.close(fig)


def plot_combined_box(all_drone_times, all_ugv_times, save_path=None):
    """
    Combined box plot across multiple trial files.
    all_drone_times: 1-D array (concatenated across robots and trials)
    all_ugv_times:   same for UGVs
    """
    fig, ax = plt.subplots(figsize=(6, 5))

    box_data = []
    labels = []
    colors = []

    if len(all_drone_times) > 0:
        box_data.append(_to_ms(all_drone_times))
        labels.append("UAV")
        colors.append("#2196F3")

    if len(all_ugv_times) > 0:
        box_data.append(_to_ms(all_ugv_times))
        labels.append("UGV")
        colors.append("#FF5722")

    if not box_data:
        plt.close(fig)
        return

    bp = ax.boxplot(box_data, positions=range(1, len(box_data) + 1), widths=0.5,
                    patch_artist=True, showfliers=True,
                    flierprops=dict(marker="o", markersize=3, alpha=0.5))

    for patch, color in zip(bp["boxes"], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels, fontsize=12)
    ax.set_ylabel("QP Solve Time (ms)", fontsize=12)
    ax.set_title("QP Computation Time (All Trials)", fontsize=13)
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Saved: {save_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Plot QP computation times")
    parser.add_argument("files", nargs="+", help="One or more qp_times_*.npy files")
    parser.add_argument("--save", action="store_true", help="Save plots as PNG files")
    parser.add_argument("--dir", default=".", help="Output directory for saved plots")
    args = parser.parse_args()

    if args.save:
        os.makedirs(args.dir, exist_ok=True)

    all_drone_flat = []
    all_ugv_flat = []

    for fpath in args.files:
        drone_times, ugv_times = _load_file(fpath)
        basename = os.path.splitext(os.path.basename(fpath))[0]

        print(f"\n=== {fpath} ===")
        for i, t in enumerate(drone_times):
            if len(t) > 0:
                all_drone_flat.append(t)
                print(f"  UAV {i+1}: n={len(t)}, mean={np.mean(t)*1000:.3f} ms, std={np.std(t)*1000:.3f} ms")
        for i, t in enumerate(ugv_times):
            if len(t) > 0:
                all_ugv_flat.append(t)
                print(f"  UGV {i+1}: n={len(t)}, mean={np.mean(t)*1000:.3f} ms, std={np.std(t)*1000:.3f} ms")

        if args.save:
            plot_box_per_robot(
                drone_times, ugv_times,
                save_path=os.path.join(args.dir, f"qp_box_{basename}.png"),
            )
            plot_histogram(
                drone_times, ugv_times,
                save_path=os.path.join(args.dir, f"qp_hist_{basename}.png"),
            )

    # Combined across all files
    if len(args.files) > 1 and args.save:
        combined_drone = np.concatenate(all_drone_flat) if all_drone_flat else np.array([])
        combined_ugv = np.concatenate(all_ugv_flat) if all_ugv_flat else np.array([])
        plot_combined_box(
            combined_drone, combined_ugv,
            save_path=os.path.join(args.dir, "qp_box_all_trials.png"),
        )

    # Print overall summary
    if all_drone_flat:
        drone_all = np.concatenate(all_drone_flat)
        print(f"\n=== Overall UAV ===")
        print(f"  n={len(drone_all)}, mean={np.mean(drone_all)*1000:.3f} ms, "
              f"median={np.median(drone_all)*1000:.3f} ms, std={np.std(drone_all)*1000:.3f} ms")

    if all_ugv_flat:
        ugv_all = np.concatenate(all_ugv_flat)
        print(f"\n=== Overall UGV ===")
        print(f"  n={len(ugv_all)}, mean={np.mean(ugv_all)*1000:.3f} ms, "
              f"median={np.median(ugv_all)*1000:.3f} ms, std={np.std(ugv_all)*1000:.3f} ms")


if __name__ == "__main__":
    main()
