#!/usr/bin/env python3
import rosbag
import sys
from collections import defaultdict

if len(sys.argv) < 2:
    print("Usage: python3 print_cpu.py <bagfile>")
    sys.exit(1)

bag_path = sys.argv[1]

# Storage
master_sum = 0.0
master_count = 0

dcf_data = defaultdict(lambda: {"sum": 0.0, "count": 0})
ugv_data = defaultdict(lambda: {"sum": 0.0, "count": 0})

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages():

        # ✅ Skip everything not related to CPU monitor
        if not topic.startswith("cpu_monitor/"):
            continue

        cpu = msg.data

        # MASTER NODES
        if topic.startswith("cpu_monitor/master_"):
            master_sum += cpu
            master_count += 1

        # DCF DRONES
        elif topic.startswith("cpu_monitor/dcf"):
            parts = topic.split('/')
            robot_name = parts[1]  # dcfX

            if "drone_node" in topic or "position_controller_node" in topic:
                dcf_data[robot_name]["sum"] += cpu
                dcf_data[robot_name]["count"] += 1

        # UGV TURTLES
        elif topic.startswith("cpu_monitor/demo_turtle"):
            parts = topic.split('/')
            robot_name = parts[1]  # demo_turtleX

            if "ugv_node" in topic:
                ugv_data[robot_name]["sum"] += cpu
                ugv_data[robot_name]["count"] += 1


# ---- RESULTS ----

print("\n===== MASTER NODES =====")
if master_count > 0:
    print(f"Average CPU (master total): {master_sum / master_count:.3f}")
else:
    print("No master data")

print("\n===== DCF ROBOTS =====")
dcf_tot_sum = 0.0
dcf_tot_count = 0

for robot in sorted(dcf_data.keys()):
    data = dcf_data[robot]
    avg = data["sum"] / data["count"]
    print(f"{robot}: {avg:.3f}")

    dcf_tot_sum += data["sum"]
    dcf_tot_count += data["count"]

if dcf_tot_count > 0:
    print(f"\nAverage CPU (all DCF robots combined): {dcf_tot_sum / dcf_tot_count:.3f}")

print("\n===== UGV ROBOTS =====")
ugv_tot_sum = 0.0
ugv_tot_count = 0

for robot in sorted(ugv_data.keys()):
    data = ugv_data[robot]
    avg = data["sum"] / data["count"]
    print(f"{robot}: {avg:.3f}")

    ugv_tot_sum += data["sum"]
    ugv_tot_count += data["count"]

if ugv_tot_count > 0:
    print(f"\nAverage CPU (all UGV robots combined): {ugv_tot_sum / ugv_tot_count:.3f}")