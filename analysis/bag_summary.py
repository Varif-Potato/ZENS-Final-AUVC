#!/usr/bin/env python3
"""
AUVC Competition Bag Analyzer

Reads a competition ROS 2 bag (MCAP format) and prints a summary of:
  - Bag duration and topic statistics
  - Flash events (lights on / off)
  - Depth range readings
  - AprilTag detections
  - Heading + target angle tracking
  - Thruster command ranges

Usage:
    source /path/to/ros_ws/install/setup.bash
    python3 bag_summary.py <bag_directory>
"""

import sys
import json
from pathlib import Path
from collections import Counter
from datetime import datetime, timezone

import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict

from std_msgs.msg import String, Bool, Int16, Float32, Float64
from sensor_msgs.msg import Image, FluidPressure
from mavros_msgs.msg import ManualControl


MSG_TYPE_MAP = {
    "std_msgs/msg/String": String,
    "std_msgs/msg/Bool": Bool,
    "std_msgs/msg/Int16": Int16,
    "std_msgs/msg/Float32": Float32,
    "std_msgs/msg/Float64": Float64,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/FluidPressure": FluidPressure,
    "mavros_msgs/msg/ManualControl": ManualControl,
}


def ns_to_str(ns: int) -> str:
    dt = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
    return dt.strftime("%H:%M:%S.%f")[:-3]


def format_duration_s(ns: int) -> str:
    total_s = ns / 1e9
    m, s = divmod(int(total_s), 60)
    return f"{m}m {s}s"


def analyze_bag(bag_path: str):
    storage = StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage, converter)

    topic_types = reader.get_all_topics_and_types()

    topic_msg_count: dict[str, int] = Counter()
    topic_msg_info: dict[str, dict] = {}

    flash_events: list[dict] = []
    pressure_readings: list[dict] = []
    apriltag_events: list[dict] = []
    yolo_events: list[dict] = []
    manual_control_samples: list[dict] = []
    heading_samples: list[dict] = []
    target_angle_samples: list[dict] = []
    target_depth_samples: list[dict] = []

    first_stamp: int | None = None
    last_stamp: int | None = None

    while reader.has_next():
        topic_name, serialized, timestamp = reader.read_next()
        if isinstance(timestamp, int) and timestamp > 0:
            if first_stamp is None or timestamp < first_stamp:
                first_stamp = timestamp
            if last_stamp is None or timestamp > last_stamp:
                last_stamp = timestamp

        topic_msg_count[topic_name] += 1

        type_map = {t.name: t.type for t in topic_types}
        msg_type_str = type_map.get(topic_name, "")

        if msg_type_str not in MSG_TYPE_MAP:
            continue

        msg_type = MSG_TYPE_MAP[msg_type_str]
        msg = deserialize_message(serialized, msg_type)

        if topic_name == "/target_lights":
            flash_events.append({
                "stamp": timestamp,
                "time": ns_to_str(timestamp) if timestamp else "N/A",
                "pwm": msg.data,
            })

        elif topic_name == "/pressure":
            pressure_readings.append({
                "stamp": timestamp,
                "time": ns_to_str(timestamp) if timestamp else "N/A",
                "pressure_pa": msg.fluid_pressure,
                "depth_m_approx": (msg.fluid_pressure - 101325) / (1025 * 9.81),
            })

        elif topic_name == "/apriltags/detections":
            try:
                detections = json.loads(msg.data)
                for det in detections:
                    center = det.get("center")
                    dist = (center[0] ** 2 + center[1] ** 2) ** 0.5 if center else None
                    apriltag_events.append({
                        "stamp": timestamp,
                        "time": ns_to_str(timestamp) if timestamp else "N/A",
                        "tag_id": det["id"],
                        "center": center,
                        "distance": round(dist, 3) if dist else None,
                    })
            except (json.JSONDecodeError, KeyError, IndexError):
                pass

        elif topic_name == "/yolo/detections":
            try:
                yolo_results = json.loads(msg.data)
                yolo_events.append({
                    "stamp": timestamp,
                    "time": ns_to_str(timestamp) if timestamp else "N/A",
                    "count": len(yolo_results),
                })
            except json.JSONDecodeError:
                pass

        elif topic_name == "/manual_control":
            manual_control_samples.append({
                "stamp": timestamp,
                "time": ns_to_str(timestamp) if timestamp else "N/A",
                "x": msg.x,
                "y": msg.y,
                "z": msg.z,
                "r": msg.r,
            })

        elif topic_name == "/heading":
            heading_samples.append({
                "stamp": timestamp,
                "time": ns_to_str(timestamp) if timestamp else "N/A",
                "heading": msg.data,
            })

        elif topic_name == "/target_angle":
            target_angle_samples.append({
                "stamp": timestamp,
                "time": ns_to_str(timestamp) if timestamp else "N/A",
                "target_angle": msg.data,
            })

        elif topic_name == "/target_depth":
            target_depth_samples.append({
                "stamp": timestamp,
                "time": ns_to_str(timestamp) if timestamp else "N/A",
                "target_depth": msg.data,
            })

    topic_stats = {
        t: {
            "type": next(
                (tt.type for tt in topic_types if tt.name == t), "unknown"
            ),
            "messages": topic_msg_count[t],
        }
        for t in topic_msg_count
    }

    return {
        "bag_path": bag_path,
        "first_stamp": first_stamp,
        "last_stamp": last_stamp,
        "duration_ns": last_stamp - first_stamp if first_stamp and last_stamp else 0,
        "duration_str": format_duration_s(last_stamp - first_stamp) if first_stamp and last_stamp else "N/A",
        "total_messages": sum(topic_msg_count.values()),
        "topic_count": len(topic_stats),
        "topics": topic_stats,
        "flash_events": flash_events,
        "pressure_readings": pressure_readings,
        "apriltag_events": apriltag_events,
        "yolo_events": yolo_events,
        "manual_control_samples": manual_control_samples,
        "heading_samples": heading_samples,
        "target_angle_samples": target_angle_samples,
        "target_depth_samples": target_depth_samples,
    }


def print_summary(result: dict):
    print("=" * 65)
    print("  AUVC COMPETITION BAG SUMMARY")
    print("=" * 65)

    bag = Path(result["bag_path"])
    print(f"  Bag:          {bag.name}")
    print(f"  Duration:     {result['duration_str']}")
    print(f"  Start:        {ns_to_str(result['first_stamp']) if result['first_stamp'] else 'N/A'}")
    print(f"  End:          {ns_to_str(result['last_stamp']) if result['last_stamp'] else 'N/A'}")
    print(f"  Topics:       {result['topic_count']}")
    print(f"  Total msgs:   {result['total_messages']:,}")
    print()

    # --- Topic summary ---
    print("-" * 65)
    print("  TOPICS")
    print("-" * 65)
    for topic, info in sorted(result["topics"].items()):
        tshort = info["type"].split("/")[-1] if "/" in info["type"] else info["type"]
        print(f"  {topic:40s} {info['messages']:>8,}  ({tshort})")
    print()

    # --- Flash events ---
    print("-" * 65)
    print("  FLASH EVENTS  (lights on = LED trigger)")
    print("-" * 65)
    flashes = result["flash_events"]
    if flashes:
        ons = [e for e in flashes if e["pwm"] > 1500]
        offs = [e for e in flashes if e["pwm"] <= 1500]
        print(f"  Total transitions: {len(flashes)}  (on={len(ons)}, off={len(offs)})")
        print()
        for e in flashes[:30]:
            status = "ON " if e["pwm"] > 1500 else "OFF"
            print(f"    {e['time']}  lights {status} (pwm={e['pwm']})")
        if len(flashes) > 30:
            print(f"    ... ({len(flashes) - 30} more transitions)")
    else:
        print("  (no flash events)")
    print()

    # --- AprilTag detections ---
    print("-" * 65)
    print("  APRILTAG DETECTIONS")
    print("-" * 65)
    tags = result["apriltag_events"]
    if tags:
        tag_ids = Counter(e["tag_id"] for e in tags)
        near_tags = [e for e in tags if e["distance"] is not None and e["distance"] < 1.0]
        print(f"  Total detections: {len(tags)}")
        print(f"  Tag IDs seen:     {dict(tag_ids)}")
        print(f"  Near detections:  {len(near_tags)}  (distance < 1.0)")
        if near_tags:
            print("  Near detections (first 10):")
            for e in near_tags[:10]:
                print(f"    {e['time']}  tag={e['tag_id']}  dist={e['distance']}")
    else:
        print("  (no AprilTag detections)")
    print()

    # --- Depth ---
    print("-" * 65)
    print("  DEPTH (from pressure sensor)")
    print("-" * 65)
    depths = result["pressure_readings"]
    if depths:
        depth_values = [d["depth_m_approx"] for d in depths]
        min_d = min(depth_values)
        max_d = max(depth_values)
        avg_d = sum(depth_values) / len(depth_values)
        print(f"  Samples:  {len(depths)}")
        print(f"  Min:      {min_d:.2f} m")
        print(f"  Max:      {max_d:.2f} m")
        print(f"  Avg:      {avg_d:.2f} m")
    else:
        print("  (no pressure data)")
    print()

    # --- Heading ---
    print("-" * 65)
    print("  HEADING")
    print("-" * 65)
    headings = result["heading_samples"]
    if headings:
        hv = [h["heading"] for h in headings]
        print(f"  Samples: {len(headings)}")
        print(f"  Range:   {min(hv)}° to {max(hv)}°")
    else:
        print("  (no heading data)")
    print()

    # --- Target angle ---
    print("-" * 65)
    print("  TARGET ANGLE")
    print("-" * 65)
    ta = result["target_angle_samples"]
    if ta:
        tv = [t["target_angle"] for t in ta]
        print(f"  Samples: {len(ta)}")
        print(f"  Range:   {min(tv)}° to {max(tv)}°")
    else:
        print("  (no target angle data)")
    print()

    # --- Thruster commands ---
    print("-" * 65)
    print("  THRUSTER COMMANDS (manual_control)")
    print("-" * 65)
    mc = result["manual_control_samples"]
    if mc:
        print(f"  Samples: {len(mc)}")
        for axis in ("x", "y", "z", "r"):
            vals = [m[axis] for m in mc]
            print(f"  {axis}:   min={min(vals):+.2f}  max={max(vals):+.2f}  avg={sum(vals)/len(vals):+.2f}")
    else:
        print("  (no manual control data)")
    print()

    print("=" * 65)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bag_summary.py <bag_directory>")
        sys.exit(1)

    bag_path = sys.argv[1]
    if not Path(bag_path).is_dir():
        print(f"Error: bag directory not found: {bag_path}")
        sys.exit(1)

    rclpy.init()
    try:
        result = analyze_bag(bag_path)
        print_summary(result)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
