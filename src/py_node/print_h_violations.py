#!/usr/bin/env python3

import numpy as np
import sys


def weighted_violation_percentage(t, h):
    """
    Percentage of active time for which h < 0.
    NaNs are ignored.

    Parameters
    ----------
    t : array, shape (T,)
        Time stamps
    h : array, shape (T,)
        CBF values over time

    Returns
    -------
    percent : float
    """
    t = np.asarray(t, dtype=float)
    h = np.asarray(h, dtype=float)

    if len(t) < 2:
        return np.nan

    valid = ~np.isnan(h)
    if np.sum(valid) < 2:
        return np.nan

    dt = np.diff(t)

    # assign each interval [t_k, t_{k+1}) to h[k]
    valid_intervals = valid[:-1]
    violating_intervals = valid[:-1] & (h[:-1] < -0.1)

    total_time = np.sum(dt[valid_intervals])
    violating_time = np.sum(dt[violating_intervals])

    if total_time <= 0.0:
        return np.nan

    return 100.0 * violating_time / total_time


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 violation_stats.py <h_func_file.npy>")
        return

    file_path = sys.argv[1]

    data = np.load(file_path, allow_pickle=True).item()
    drone_logs = data["drone_h_logs"]
    ugv_logs = data["ugv_h_logs"]

    if len(drone_logs) == 0 or len(ugv_logs) == 0:
        print("Empty h logs.")
        return

    t_d = np.array([entry["t"] for entry in drone_logs], dtype=float)
    t_g = np.array([entry["t"] for entry in ugv_logs], dtype=float)

    no_agents = len(drone_logs[0]["landing"])

    print("\n========== UAV CONSTRAINT VIOLATION PERCENTAGES ==========\n")

    # --------------------------------------------------
    # 1) UAV landing constraint: one per UAV
    # --------------------------------------------------
    landing_h = np.full((len(drone_logs), no_agents), np.nan, dtype=float)
    for k, entry in enumerate(drone_logs):
        landing_h[k, :] = np.asarray(entry["landing"], dtype=float)

    print("UAV landing constraints:")
    for i in range(no_agents):
        pct = weighted_violation_percentage(t_d, landing_h[:, i])
        if np.isnan(pct):
            print(f"  UAV {i+1}: no active data")
        else:
            print(f"  UAV {i+1}: {pct:.2f}%")

    print()

    # --------------------------------------------------
    # 2) UAV boundary constraints: one set per UAV
    # if boundary logs are available
    # --------------------------------------------------
    if "boundary" in drone_logs[0]:
        boundary_names = ["x_max", "x_min", "y_max", "y_min", "z_max"]
        drone_boundary_h = np.full((len(drone_logs), no_agents, 5), np.nan, dtype=float)

        for k, entry in enumerate(drone_logs):
            arr = np.asarray(entry["boundary"], dtype=float)
            if arr.shape == (no_agents, 5):
                drone_boundary_h[k, :, :] = arr

        print("UAV boundary constraints:")
        for i in range(no_agents):
            for c in range(5):
                pct = weighted_violation_percentage(t_d, drone_boundary_h[:, i, c])
                if np.isnan(pct):
                    print(f"  UAV {i+1}, {boundary_names[c]}: no active data")
                else:
                    print(f"  UAV {i+1}, {boundary_names[c]}: {pct:.2f}%")
        print()

    # --------------------------------------------------
    # 3) UAV-UGV air constraints: one per UAV-UGV pair
    # --------------------------------------------------
    air_h = {
        (i, k): np.full(len(drone_logs), np.nan, dtype=float)
        for i in range(no_agents) for k in range(no_agents) if i != k
    }

    for idx, entry in enumerate(drone_logs):
        for item in entry["air"]:
            i = int(item["drone"])
            k = int(item["ugv"])
            air_h[(i, k)][idx] = float(item["h"])

    print("UAV-UGV air constraints:")
    for (i, k), series in air_h.items():
        pct = weighted_violation_percentage(t_d, series)
        if np.isnan(pct):
            print(f"  UAV {i+1} wrt UGV {k+1}: no active data")
        else:
            print(f"  UAV {i+1} wrt UGV {k+1}: {pct:.2f}%")
    print()

    # --------------------------------------------------
    # 4) UAV-UAV pair constraints: one per UAV pair
    # --------------------------------------------------
    drone_pair_h = {
        (i, j): np.full(len(drone_logs), np.nan, dtype=float)
        for i in range(no_agents) for j in range(i + 1, no_agents)
    }

    for idx, entry in enumerate(drone_logs):
        for item in entry["pair"]:
            i = int(item["i"])
            j = int(item["j"])
            drone_pair_h[(i, j)][idx] = float(item["h"])

    print("UAV-UAV pair constraints:")
    for (i, j), series in drone_pair_h.items():
        pct = weighted_violation_percentage(t_d, series)
        if np.isnan(pct):
            print(f"  UAV {i+1}-{j+1}: no active data")
        else:
            print(f"  UAV {i+1}-{j+1}: {pct:.2f}%")
    print()

    print("========== UGV CONSTRAINT VIOLATION PERCENTAGES ==========\n")

    # --------------------------------------------------
    # 5) UGV boundary constraints: one set per UGV
    # --------------------------------------------------
    if "boundary" in ugv_logs[0]:
        boundary_names = ["x_max", "x_min", "y_max", "y_min"]
        ugv_boundary_h = np.full((len(ugv_logs), no_agents, 4), np.nan, dtype=float)

        for k, entry in enumerate(ugv_logs):
            arr = np.asarray(entry["boundary"], dtype=float)
            if arr.shape == (no_agents, 4):
                ugv_boundary_h[k, :, :] = arr

        print("UGV boundary constraints:")
        for i in range(no_agents):
            for c in range(4):
                pct = weighted_violation_percentage(t_g, ugv_boundary_h[:, i, c])
                if np.isnan(pct):
                    print(f"  UGV {i+1}, {boundary_names[c]}: no active data")
                else:
                    print(f"  UGV {i+1}, {boundary_names[c]}: {pct:.2f}%")
        print()

    # --------------------------------------------------
    # 6) UGV-UGV pair constraints: one per UGV pair
    # --------------------------------------------------
    ugv_pair_h = {
        (i, k): np.full(len(ugv_logs), np.nan, dtype=float)
        for i in range(no_agents) for k in range(i + 1, no_agents)
    }

    for idx, entry in enumerate(ugv_logs):
        for item in entry["pair"]:
            i = int(item["i"])
            k = int(item["k"])
            ugv_pair_h[(i, k)][idx] = float(item["h"])

    print("UGV-UGV pair constraints:")
    for (i, k), series in ugv_pair_h.items():
        pct = weighted_violation_percentage(t_g, series)
        if np.isnan(pct):
            print(f"  UGV {i+1}-{k+1}: no active data")
        else:
            print(f"  UGV {i+1}-{k+1}: {pct:.2f}%")
    print()


if __name__ == "__main__":
    main()