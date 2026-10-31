#!/usr/bin/env python3
"""
plot_scalability.py – QP computation time vs number of UAV-UGV pairs.

Input files must be named:  scalability_N<number>_qp_times_*.npy
They can be passed explicitly or discovered from a directory.

Usage:
    python3 plot_scalability.py scalability_N5_*.npy scalability_N10_*.npy ... [--save] [--dir <out>]
    python3 plot_scalability.py --dir <results_dir> --save
"""

import argparse
import os
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _load(path):
    data = np.load(path, allow_pickle=True).item()
    drone = [np.asarray(t, dtype=float) for t in data.get("drone_qp_times", [])]
    ugv   = [np.asarray(t, dtype=float) for t in data.get("ugv_qp_times",   [])]
    # old flat-array per-robot files
    if not drone and "drone_qp_times" in data:
        arr = np.asarray(data["drone_qp_times"], dtype=float)
        if arr.ndim == 1:
            drone = [arr]
    return drone, ugv


def _stats(arrays):
    """Return (mean_ms, std_ms, median_ms, n) from a list of 1-D arrays."""
    nonempty = [a for a in arrays if len(a) > 0]
    if not nonempty:
        return None
    flat = np.concatenate(nonempty) * 1000.0   # → ms
    return float(np.mean(flat)), float(np.std(flat)), float(np.median(flat)), int(len(flat))


def _n_from_filename(path):
    """Extract N from filenames like scalability_N10_qp_times_*.npy"""
    m = re.search(r'[Nn](\d+)', os.path.basename(path))
    return int(m.group(1)) if m else None


# ──────────────────────────────────────────────────────────────────────────────
# Plots
# ──────────────────────────────────────────────────────────────────────────────

def plot_mean_vs_n(records, save_path=None):
    """
    Line plot: mean ± std QP time (ms) vs number of pairs N.
    records: list of (N, uav_stats, ugv_stats)  where stats = (mean, std, median, n) or None
    """
    records = sorted(records, key=lambda r: r[0])
    Ns = [r[0] for r in records]

    fig, ax = plt.subplots(figsize=(7, 5))

    for label, idx, color, marker in [("UAV", 1, "#2196F3", "o"), ("UGV", 2, "#FF5722", "s")]:
        means = [r[idx][0] if r[idx] else None for r in records]
        stds  = [r[idx][1] if r[idx] else None for r in records]
        valid = [(n, m, s) for n, m, s in zip(Ns, means, stds) if m is not None]
        if not valid:
            continue
        vN, vM, vS = zip(*valid)
        ax.errorbar(vN, vM, yerr=vS, fmt=f"-{marker}", color=color, capsize=4,
                    linewidth=2, markersize=7, label=f"{label} (mean ± std)")

    ax.set_xlabel("Number of UAV-UGV Pairs", fontsize=13)
    ax.set_ylabel("QP Solve Time (ms)", fontsize=13)
    ax.set_title("QP Computation Time vs Number of Pairs", fontsize=14)
    ax.set_xticks(Ns)
    ax.legend(fontsize=11)
    ax.grid(alpha=0.35)
    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Saved: {save_path}")
    plt.close(fig)


def plot_box_vs_n(records, save_path=None):
    """
    Side-by-side box plots grouped by N.
    """
    records = sorted(records, key=lambda r: r[0])
    fig, ax = plt.subplots(figsize=(max(7, len(records)*2), 5))

    positions = []
    box_data  = []
    colors    = []
    tick_pos  = []
    tick_lbl  = []
    gap = 0.4
    width = 0.35
    base = 1

    for N, uav_stats, ugv_stats, uav_arrays, ugv_arrays in records:
        # UAV box
        uav_flat = np.concatenate([a for a in uav_arrays if len(a) > 0]) * 1000.0 \
                   if any(len(a) > 0 for a in uav_arrays) else None
        ugv_flat = np.concatenate([a for a in ugv_arrays if len(a) > 0]) * 1000.0 \
                   if any(len(a) > 0 for a in ugv_arrays) else None

        center = base
        if uav_flat is not None:
            positions.append(center - width/2 - gap/4)
            box_data.append(uav_flat)
            colors.append("#2196F3")
        if ugv_flat is not None:
            positions.append(center + width/2 + gap/4)
            box_data.append(ugv_flat)
            colors.append("#FF5722")

        tick_pos.append(center)
        tick_lbl.append(f"N={N}")
        base += 1 + gap

    if not box_data:
        plt.close(fig)
        return

    bp = ax.boxplot(box_data, positions=positions, widths=width,
                    patch_artist=True, showfliers=False,
                    medianprops=dict(color="black", linewidth=1.5))
    for patch, c in zip(bp["boxes"], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.75)

    # Legend proxy
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(facecolor="#2196F3", alpha=0.75, label="UAV"),
                        Patch(facecolor="#FF5722", alpha=0.75, label="UGV")],
              fontsize=11)

    ax.set_xticks(tick_pos)
    ax.set_xticklabels(tick_lbl, fontsize=12)
    ax.set_ylabel("QP Solve Time (ms)", fontsize=13)
    ax.set_title("QP Computation Time Distribution vs Number of Pairs", fontsize=14)
    ax.grid(axis="y", alpha=0.35)
    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Saved: {save_path}")
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="*", help="scalability_N*_qp_times_*.npy files")
    parser.add_argument("--save",  action="store_true")
    parser.add_argument("--dir",   default=".", help="Output / search directory")
    args = parser.parse_args()

    paths = list(args.files)
    if not paths:
        # auto-discover from --dir
        for f in sorted(os.listdir(args.dir)):
            if f.startswith("scalability_N") and f.endswith(".npy"):
                paths.append(os.path.join(args.dir, f))

    if not paths:
        print("No scalability files found.")
        return

    if args.save:
        os.makedirs(args.dir, exist_ok=True)

    records = []   # (N, uav_stats, ugv_stats, uav_arrays, ugv_arrays)
    for path in paths:
        N = _n_from_filename(path)
        if N is None:
            print(f"WARNING: cannot parse N from {path}, skipping.")
            continue
        drone_arrays, ugv_arrays = _load(path)
        uav_stats = _stats(drone_arrays)
        ugv_stats = _stats(ugv_arrays)

        print(f"\n=== N={N} | {os.path.basename(path)} ===")
        if uav_stats:
            print(f"  UAV: n={uav_stats[3]}, mean={uav_stats[0]:.3f} ms, "
                  f"median={uav_stats[2]:.3f} ms, std={uav_stats[1]:.3f} ms")
        else:
            print("  UAV: no data")
        if ugv_stats:
            print(f"  UGV: n={ugv_stats[3]}, mean={ugv_stats[0]:.3f} ms, "
                  f"median={ugv_stats[2]:.3f} ms, std={ugv_stats[1]:.3f} ms")
        else:
            print("  UGV: no data")

        records.append((N, uav_stats, ugv_stats, drone_arrays, ugv_arrays))

    if not records:
        print("No valid records to plot.")
        return

    if args.save:
        plot_mean_vs_n(
            [(r[0], r[1], r[2]) for r in records],
            save_path=os.path.join(args.dir, "scalability_mean_vs_n.png")
        )
        plot_box_vs_n(
            records,
            save_path=os.path.join(args.dir, "scalability_box_vs_n.png")
        )
    else:
        plot_mean_vs_n([(r[0], r[1], r[2]) for r in records])
        plot_box_vs_n(records)


if __name__ == "__main__":
    main()
