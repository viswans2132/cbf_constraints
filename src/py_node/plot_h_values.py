#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import sys


def pair_key(i, j):
    return f"{i+1}-{j+1}"


def air_key(i, k):
    return f"UAV{i+1}-UGV{k+1}"


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_h_funcs.py <h_func_file.npy>")
        return

    file_path = sys.argv[1]

    data = np.load(file_path, allow_pickle=True).item()
    drone_logs = data["drone_h_logs"]
    # ugv_logs = data["ugv_h_logs"]

    if len(drone_logs) == 0:
        print("No drone h logs found.")
        return

    # Time from drone logs
    t = np.array([entry["t"] for entry in drone_logs], dtype=float)
    t = t - t[0]

    no_agents = drone_logs[0]["landing"].shape[0]

    # --------------------------------------------------
    # 1) UAV landing h
    # one curve per UAV
    # --------------------------------------------------
    landing_h = np.zeros((len(drone_logs), no_agents), dtype=float)
    print(drone_logs[0])

    for idx, entry in enumerate(drone_logs):
        landing_h[idx, :] = np.array(entry["landing"], dtype=float)

    plt.figure()
    for i in range(no_agents):
        plt.plot(t, landing_h[:, i], label=f"UAV {i+1}")
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("h")
    plt.title("UAV Landing CBF")
    plt.legend()
    plt.grid(True)

    # --------------------------------------------------
    # 2) UAV-UAV pair h
    # one curve per active pair over time
    # missing values -> NaN
    # --------------------------------------------------
    drone_pair_series = {}
    for i in range(no_agents):
        for j in range(i + 1, no_agents):
            drone_pair_series[pair_key(i, j)] = np.full(len(drone_logs), np.nan, dtype=float)

    for idx, entry in enumerate(drone_logs):
        for pair_entry in entry["pair"]:
            i = int(pair_entry["i"])
            j = int(pair_entry["j"])
            h = float(pair_entry["h"])
            drone_pair_series[pair_key(i, j)][idx] = h

    plt.figure()
    for key, values in drone_pair_series.items():
        plt.plot(t, values, label=key)
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("h")
    plt.title("UAV-UAV Pairwise CBF")
    plt.legend()
    plt.grid(True)

    # --------------------------------------------------
    # 3) UAV-UGV air h
    # one curve per UAV-UGV combination
    # missing values -> NaN
    # --------------------------------------------------
    air_series = {}
    for i in range(no_agents):
        for k in range(no_agents):
            if i == k:
                continue
            air_series[air_key(i, k)] = np.full(len(drone_logs), np.nan, dtype=float)

    for idx, entry in enumerate(drone_logs):
        for air_entry in entry["air"]:
            i = int(air_entry["drone"])
            k = int(air_entry["ugv"])
            h = float(air_entry["h"])
            air_series[air_key(i, k)][idx] = h

    plt.figure()
    for key, values in air_series.items():
        plt.plot(t, values, label=key)
    plt.axhline(0.0, linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("h")
    plt.title("UAV-UGV Air CBF")
    plt.legend()
    plt.grid(True)

    # # --------------------------------------------------
    # # 4) UGV-UGV pair h
    # # one curve per UGV pair
    # # missing values -> NaN
    # # --------------------------------------------------
    # ugv_pair_series = {}
    # for i in range(no_agents):
    #     for k in range(i + 1, no_agents):
    #         ugv_pair_series[pair_key(i, k)] = np.full(len(ugv_logs), np.nan, dtype=float)

    # for idx, entry in enumerate(ugv_logs):
    #     for pair_entry in entry["pair"]:
    #         i = int(pair_entry["i"])
    #         k = int(pair_entry["k"])
    #         h = float(pair_entry["h"])
    #         ugv_pair_series[pair_key(i, k)][idx] = h

    # t_ugv = np.array([entry["t"] for entry in ugv_logs], dtype=float)
    # t_ugv = t_ugv - t_ugv[0]

    # plt.figure()
    # for key, values in ugv_pair_series.items():
    #     plt.plot(t_ugv, values, label=key)
    # plt.axhline(0.0, linestyle="--")
    # plt.xlabel("Time (s)")
    # plt.ylabel("h")
    # plt.title("UGV-UGV Pairwise CBF")
    # plt.legend()
    # plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()