#!/usr/bin/env python3

import numpy as np
import sys


def compute_average(times):
    """Compute overall average from list of lists."""
    flat = [t for robot in times for t in robot]
    if len(flat) == 0:
        return None
    return np.mean(flat)


def compute_robot_averages(times):
    """Compute average per robot."""
    averages = []
    for robot_times in times:
        if len(robot_times) == 0:
            averages.append(None)
        else:
            averages.append(np.mean(robot_times))
    return averages


def main(filename):

    data = np.load(filename, allow_pickle=True).item()

    ugv_task_times = data["ugv_task_times"]
    uav_task_times = data["uav_task_times"]
    uav_return_times = data["uav_return_times"]

    # Per-robot averages
    ugv_robot_avg = compute_robot_averages(ugv_task_times)
    uav_task_robot_avg = compute_robot_averages(uav_task_times)
    uav_return_robot_avg = compute_robot_averages(uav_return_times)

    # Overall averages
    ugv_avg = compute_average(ugv_task_times)
    uav_task_avg = compute_average(uav_task_times)
    uav_return_avg = compute_average(uav_return_times)

    print("\n===== PER ROBOT AVERAGES =====\n")

    for i, val in enumerate(uav_task_robot_avg):
        print(f"UAV {i+1} task avg: {val:.3f} s")

    for i, val in enumerate(uav_return_robot_avg):
        print(f"UAV {i+1} return avg: {val:.3f} s")

    for i, val in enumerate(ugv_robot_avg):
        print(f"UGV {i+1} task avg: {val:.3f} s")

    print("\n===== OVERALL AVERAGES =====\n")

    print(f"Average UAV task time   : {uav_task_avg:.3f} s")
    print(f"Average UAV return time : {uav_return_avg:.3f} s")
    print(f"Average UGV task time   : {ugv_avg:.3f} s")


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("Usage: python analyze_results.py <results_file.npy>")
        sys.exit(1)

    main(sys.argv[1])