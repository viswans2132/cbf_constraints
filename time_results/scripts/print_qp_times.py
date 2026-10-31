#!/usr/bin/env python3

import numpy as np
import sys


def _normalise(data, key):
    """Return a list of 1-D arrays (one per robot) regardless of on-disk format."""
    arr = data[key]
    if isinstance(arr, np.ndarray) and arr.ndim == 1:
        # Old flat format – treat as a single robot
        return [arr]
    return [np.asarray(r, dtype=float) for r in arr]


def _print_stats(label, times_list):
    """Print per-robot and overall statistics for a list of 1-D timing arrays."""
    print(f"\n--- {label} ---")
    all_times = []
    for i, t in enumerate(times_list):
        if len(t) == 0:
            print(f"  {label} {i+1}: no data")
            continue
        all_times.append(t)
        print(
            f"  {label} {i+1}: "
            f"n={len(t)}, "
            f"mean={np.mean(t)*1000:.3f} ms, "
            f"median={np.median(t)*1000:.3f} ms, "
            f"std={np.std(t)*1000:.3f} ms, "
            f"min={np.min(t)*1000:.3f} ms, "
            f"max={np.max(t)*1000:.3f} ms"
        )

    if all_times:
        flat = np.concatenate(all_times)
        print(
            f"  Overall (all {label}): "
            f"n={len(flat)}, "
            f"mean={np.mean(flat)*1000:.3f} ms, "
            f"median={np.median(flat)*1000:.3f} ms, "
            f"std={np.std(flat)*1000:.3f} ms, "
            f"min={np.min(flat)*1000:.3f} ms, "
            f"max={np.max(flat)*1000:.3f} ms"
        )
    else:
        print("  No data available.")


def main():
    if len(sys.argv) < 2:
        print("Usage: python print_qp_times.py <qp_times_file.npy>")
        return

    file_path = sys.argv[1]

    # Load stored dictionary
    data = np.load(file_path, allow_pickle=True).item()

    print("\n===== QP Timing Statistics =====")
    print(f"File: {file_path}")

    if "drone_qp_times" in data:
        drone_times = _normalise(data, "drone_qp_times")
        _print_stats("UAV", drone_times)

    if "ugv_qp_times" in data:
        ugv_times = _normalise(data, "ugv_qp_times")
        _print_stats("UGV", ugv_times)

    print("================================\n")


if __name__ == "__main__":
    main()