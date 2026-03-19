#!/usr/bin/env python3

import numpy as np
import sys

def main():

    if len(sys.argv) < 2:
        print("Usage: python print_qp_times.py <qp_times_file.npy>")
        return

    file_path = sys.argv[1]

    # Load stored dictionary
    data = np.load(file_path, allow_pickle=True).item()

    drone_qp_times = np.array(data["drone_qp_times"])
    # ugv_qp_times = np.array(data["ugv_qp_times"])

    # Compute averages
    avg_drone_qp = np.mean(drone_qp_times)
    # avg_ugv_qp = np.mean(ugv_qp_times)

    print("\n===== QP Timing Statistics =====")
    print(f"Number of iterations: {len(drone_qp_times)}")
    print()
    print(f"Average UAV QP solve time : {avg_drone_qp:.6f} seconds ({avg_drone_qp*1000:.3f} ms)")
    # print(f"Average UGV QP solve time : {avg_ugv_qp:.6f} seconds ({avg_ugv_qp*1000:.3f} ms)")
    print("================================\n")


if __name__ == "__main__":
    main()