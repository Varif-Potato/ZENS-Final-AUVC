import sys
import json
import math
import csv
import os
import base64
from pathlib import Path
from collections import Counter, defaultdict
from datetime import datetime, timezone

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_ordereddict
from cv_bridge import CvBridge

from std_msgs.msg import String, Bool, Int16, Float32, Float64
from sensor_msgs.msg import Image, FluidPressure, BatteryState
from mavros_msgs.msg import ManualControl, OverrideRCIn, State, Altitude
from geometry_msgs.msg import PoseStamped, TwistStamped


MSG_TYPE_MAP = {
    "std_msgs/msg/String": String,
    "std_msgs/msg/Bool": Bool,
    "std_msgs/msg/Int16": Int16,
    "std_msgs/msg/Float32": Float32,
    "std_msgs/msg/Float64": Float64,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/FluidPressure": FluidPressure,
    "sensor_msgs/msg/BatteryState": BatteryState,
    "mavros_msgs/msg/ManualControl": ManualControl,
    "mavros_msgs/msg/OverrideRCIn": OverrideRCIn,
    "mavros_msgs/msg/State": State,
    "mavros_msgs/msg/Altitude": Altitude,
    "geometry_msgs/msg/PoseStamped": PoseStamped,
    "geometry_msgs/msg/TwistStamped": TwistStamped,
}

IMAGE_TOPICS = ["/camera", "/apriltags/annotated"]

_bridge = CvBridge()


def ns_to_str(ns: int) -> str:
    dt = datetime.fromtimestamp(ns / 1e9, tz=timezone.utc)
    return dt.strftime("%H:%M:%S.%f")[:-3]


def format_duration_s(ns: int) -> str:
    total_s = ns / 1e9
    m, s = divmod(int(total_s), 60)
    return f"{m}m {s}s"


def depth_from_pressure(pressure_pa: float) -> float:
    return (pressure_pa - 101325) / (1000.0 * 9.81)


class BagAnalyzer:
    def __init__(self, bag_path: str):
        self.bag_path = Path(bag_path)
        self.topic_types = []
        self.topic_msg_count: Counter = Counter()

        self.numeric_data: dict[str, list] = defaultdict(list)
        self.image_frames: dict[str, list] = defaultdict(list)

        self.flash_events: list[dict] = []
        self.pressure_readings: list[dict] = []
        self.apriltag_events: list[dict] = []
        self.yolo_events: list[dict] = []
        self.manual_control_samples: list[dict] = []
        self.heading_samples: list[dict] = []
        self.target_angle_samples: list[dict] = []
        self.target_depth_samples: list[dict] = []

        self.mavros_states: list[dict] = []
        self.battery_samples: list[dict] = []
        self.pose_samples: list[dict] = []
        self.altitude_samples: list[dict] = []

        self.first_stamp: int | None = None
        self.last_stamp: int | None = None
        self.fps_estimates: dict[str, float] = {}

    def _ns(self, stamp) -> int | None:
        if isinstance(stamp, int) and stamp > 0:
            return stamp
        return None

    # -----------------------------------------------------------------
    #  Bag reading
    # -----------------------------------------------------------------

    def analyze(self):
        storage = StorageOptions(uri=str(self.bag_path), storage_id="mcap")
        converter = ConverterOptions("", "")
        reader = SequentialReader()
        reader.open(storage, converter)

        self.topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in self.topic_types}

        image_timestamps: dict[str, list[int]] = defaultdict(list)
        total = 0

        print(f"  Reading bag: {self.bag_path.name} ...", flush=True)

        while reader.has_next():
            topic_name, serialized, timestamp_signed = reader.read_next()
            total += 1
            stamp = self._ns(timestamp_signed)
            if stamp is not None:
                if self.first_stamp is None or stamp < self.first_stamp:
                    self.first_stamp = stamp
                if self.last_stamp is None or stamp > self.last_stamp:
                    self.last_stamp = stamp

            self.topic_msg_count[topic_name] += 1
            msg_type_str = type_map.get(topic_name, "")

            if msg_type_str not in MSG_TYPE_MAP:
                continue

            msg_type = MSG_TYPE_MAP[msg_type_str]
            msg = deserialize_message(serialized, msg_type)

            if topic_name in IMAGE_TOPICS and isinstance(msg, Image):
                image_timestamps[topic_name].append(stamp)
                try:
                    cv_img = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.image_frames[topic_name].append((stamp, cv_img))
                except Exception:
                    pass
                continue

            if isinstance(msg, String):
                if topic_name == "/apriltags/detections":
                    self._handle_apriltags(msg, stamp)
                elif topic_name == "/yolo/detections":
                    self._handle_yolo(msg, stamp)
                continue

            if topic_name == "/target_lights" and isinstance(msg, Int16):
                self.flash_events.append(dict(stamp=stamp, time=ns_to_str(stamp), pwm=msg.data))
                self.numeric_data[topic_name].append((stamp, dict(pwm=msg.data)))
            elif topic_name == "/pressure" and isinstance(msg, FluidPressure):
                d = depth_from_pressure(msg.fluid_pressure)
                self.pressure_readings.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    pressure_pa=msg.fluid_pressure, depth_m_approx=d,
                ))
                self.numeric_data[topic_name].append((stamp, dict(fluid_pressure=msg.fluid_pressure, depth_m=d)))
            elif topic_name == "/manual_control" and isinstance(msg, ManualControl):
                self.manual_control_samples.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    x=msg.x, y=msg.y, z=msg.z, r=msg.r,
                ))
                self.numeric_data[topic_name].append((stamp, dict(x=msg.x, y=msg.y, z=msg.z, r=msg.r)))
            elif topic_name == "/heading" and isinstance(msg, Int16):
                self.heading_samples.append(dict(stamp=stamp, time=ns_to_str(stamp), heading=msg.data))
                self.numeric_data[topic_name].append((stamp, dict(heading=msg.data)))
            elif topic_name == "/target_angle" and isinstance(msg, Int16):
                self.target_angle_samples.append(dict(stamp=stamp, time=ns_to_str(stamp), target_angle=msg.data))
                self.numeric_data[topic_name].append((stamp, dict(target_angle=msg.data)))
            elif topic_name == "/target_depth" and isinstance(msg, Float32):
                self.target_depth_samples.append(dict(stamp=stamp, time=ns_to_str(stamp), target_depth=msg.data))
                self.numeric_data[topic_name].append((stamp, dict(target_depth=msg.data)))
            elif topic_name == "/mavros/state" and isinstance(msg, State):
                self.mavros_states.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    connected=msg.connected, armed=msg.armed, mode=msg.mode,
                ))
            elif topic_name == "/mavros/battery" and isinstance(msg, BatteryState):
                self.battery_samples.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    voltage=msg.voltage, current=msg.current, percentage=msg.percentage,
                ))
                self.numeric_data[topic_name].append((stamp, dict(
                    voltage=msg.voltage, current=msg.current, percentage=msg.percentage,
                )))
            elif topic_name == "/mavros/local_position/pose" and isinstance(msg, PoseStamped):
                p = msg.pose.position
                self.pose_samples.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    position_x=p.x, position_y=p.y, position_z=p.z,
                ))
                self.numeric_data[topic_name].append((stamp, dict(position_x=p.x, position_y=p.y, position_z=p.z)))
            elif topic_name == "/mavros/altitude" and isinstance(msg, Altitude):
                self.altitude_samples.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    altitude_mono=msg.altitude_monotonic, altitude_amsl=msg.altitude_amsl,
                ))
                self.numeric_data[topic_name].append((stamp, dict(
                    altitude_mono=msg.altitude_monotonic, altitude_amsl=msg.altitude_amsl,
                )))

        for topic, stamps in image_timestamps.items():
            if len(stamps) > 1:
                dt_s = (stamps[-1] - stamps[0]) / 1e9
                self.fps_estimates[topic] = len(stamps) / dt_s if dt_s > 0 else 30.0
            else:
                self.fps_estimates[topic] = 30.0

        print(f"  Done — {self.topic_msg_count.total():,} msgs, {len(self.topic_msg_count)} topics", flush=True)

    def _handle_apriltags(self, msg: String, stamp):
        try:
            detections = json.loads(msg.data)
            for det in detections:
                center = det.get("center")
                dist = det.get("distance")
                self.apriltag_events.append(dict(
                    stamp=stamp, time=ns_to_str(stamp),
                    tag_id=det["id"], center=center,
                    distance=round(dist, 3) if dist else None,
                ))
        except (json.JSONDecodeError, KeyError, IndexError):
            pass

    def _handle_yolo(self, msg: String, stamp):
        try:
            results = json.loads(msg.data)
            self.yolo_events.append(dict(
                stamp=stamp, time=ns_to_str(stamp), count=len(results),
            ))
        except json.JSONDecodeError:
            pass

    # -----------------------------------------------------------------
    #  Helpers
    # -----------------------------------------------------------------

    def _rel_time(self, stamps) -> list[float]:
        t0 = self.first_stamp or 0
        return [(s - t0) / 1e9 for s in stamps]

    def _topic_type(self, name: str) -> str:
        for t in self.topic_types:
            if t.name == name:
                return t.type.split("/")[-1]
        return "unknown"

    # -----------------------------------------------------------------
    #  Text summary
    # -----------------------------------------------------------------

    def print_text_summary(self):
        dur_ns = self.last_stamp - self.first_stamp if self.first_stamp and self.last_stamp else 0

        print("=" * 65)
        print("  AUVC COMPETITION BAG SUMMARY")
        print("=" * 65)
        print(f"  Bag:          {self.bag_path.name}")
        print(f"  Duration:     {format_duration_s(dur_ns)}")
        print(f"  Start:        {ns_to_str(self.first_stamp) if self.first_stamp else 'N/A'}")
        print(f"  End:          {ns_to_str(self.last_stamp) if self.last_stamp else 'N/A'}")
        print(f"  Topics:       {len(self.topic_msg_count)}")
        print(f"  Total msgs:   {self.topic_msg_count.total():,}")
        print()

        print("-" * 65)
        print("  TOPICS")
        print("-" * 65)
        for topic, count in sorted(self.topic_msg_count.items()):
            print(f"  {topic:45s} {count:>8,}  ({self._topic_type(topic)})")
        print()

        self._text_flash()
        self._text_apriltags()
        self._text_yolo()
        self._text_depth()
        self._text_heading()
        self._text_thrusters()
        self._text_mavros()

        print("=" * 65)

    def _text_flash(self):
        print("-" * 65)
        print("  FLASH EVENTS  (lights on = LED trigger)")
        print("-" * 65)
        if self.flash_events:
            ons = sum(1 for e in self.flash_events if e["pwm"] > 1500)
            offs = len(self.flash_events) - ons
            print(f"  Total transitions: {len(self.flash_events)}  (on={ons}, off={offs})")
            print()
            for e in self.flash_events[:30]:
                status = "ON " if e["pwm"] > 1500 else "OFF"
                print(f"    {e['time']}  lights {status} (pwm={e['pwm']})")
            if len(self.flash_events) > 30:
                print(f"    ... ({len(self.flash_events) - 30} more transitions)")
        else:
            print("  (no flash events)")
        print()

    def _text_apriltags(self):
        print("-" * 65)
        print("  APRILTAG DETECTIONS")
        print("-" * 65)
        tags = self.apriltag_events
        if tags:
            tag_ids = Counter(e["tag_id"] for e in tags)
            near_tags = [e for e in tags if e["distance"] is not None and e["distance"] < 1.0]
            print(f"  Total detections: {len(tags)}")
            print(f"  Tag IDs seen:     {dict(tag_ids)}")
            print(f"  Near detections:  {len(near_tags)}  (distance < 1.0)")
            if near_tags:
                print("  Near detections (first 10):")
                for e in near_tags[:10]:
                    print(f"    {e['time']}  tag={e['tag_id']}  dist={e['distance']}m")
        else:
            print("  (no AprilTag detections)")
        print()

    def _text_yolo(self):
        print("-" * 65)
        print("  YOLO DETECTIONS")
        print("-" * 65)
        if self.yolo_events:
            total_obj = sum(e["count"] for e in self.yolo_events)
            print(f"  Detection events: {len(self.yolo_events)}")
            print(f"  Total objects:    {total_obj}")
        else:
            print("  (no YOLO detections)")
        print()

    def _text_depth(self):
        print("-" * 65)
        print("  DEPTH (from pressure sensor)")
        print("-" * 65)
        depths = self.pressure_readings
        if depths:
            vals = [d["depth_m_approx"] for d in depths]
            print(f"  Samples:  {len(depths)}")
            print(f"  Min:      {min(vals):.2f} m")
            print(f"  Max:      {max(vals):.2f} m")
            print(f"  Avg:      {sum(vals)/len(vals):.2f} m")
        else:
            print("  (no pressure data)")
        print()

    def _text_heading(self):
        print("-" * 65)
        print("  HEADING TRACKING")
        print("-" * 65)
        h = self.heading_samples
        if h:
            hv = [e["heading"] for e in h]
            print(f"  Heading samples:  {len(h)}")
            print(f"  Heading range:    {min(hv)} to {max(hv)}")
            if self.target_angle_samples:
                tv = [e["target_angle"] for e in self.target_angle_samples]
                print(f"  Target samples:   {len(self.target_angle_samples)}")
                print(f"  Target range:     {min(tv)} to {max(tv)}")
        else:
            print("  (no heading data)")
        print()

    def _text_thrusters(self):
        print("-" * 65)
        print("  THRUSTER COMMANDS (manual_control)")
        print("-" * 65)
        mc = self.manual_control_samples
        if mc:
            print(f"  Samples: {len(mc)}")
            for axis in ("x", "y", "z", "r"):
                vals = [m[axis] for m in mc]
                print(f"  {axis}:   min={min(vals):+.2f}  max={max(vals):+.2f}  avg={sum(vals)/len(vals):+.2f}")
        else:
            print("  (no manual control data)")
        print()

    def _text_mavros(self):
        if self.mavros_states:
            print("-" * 65)
            print("  MAVROS STATE")
            print("-" * 65)
            armed_count = sum(1 for e in self.mavros_states if e["armed"])
            print(f"  State changes: {len(self.mavros_states)}")
            print(f"  Armed samples:  {armed_count}")
            modes = Counter(e["mode"] for e in self.mavros_states)
            print(f"  Modes seen:     {dict(modes)}")
            print()

        if self.battery_samples:
            print("-" * 65)
            print("  BATTERY")
            print("-" * 65)
            valid = [e for e in self.battery_samples if e["voltage"] > 0]
            if valid:
                voltages = [e["voltage"] for e in valid]
                print(f"  Samples:   {len(valid)}")
                print(f"  Voltage:   {min(voltages):.2f}V - {max(voltages):.2f}V")
                percentages = [e["percentage"] for e in valid if e["percentage"] >= 0]
                if percentages:
                    print(f"  Battery:   {percentages[-1]*100:.0f}% (final)")
            else:
                print("  (no valid battery data)")
            print()

    # -----------------------------------------------------------------
    #  CSV export
    # -----------------------------------------------------------------

    def export_csvs(self, out_dir: Path):
        csv_dir = out_dir / "csvs"
        csv_dir.mkdir(parents=True, exist_ok=True)

        for topic, data_list in self.numeric_data.items():
            if not data_list:
                continue
            _, first = data_list[0]
            fieldnames = ["stamp", "time"] + list(first.keys())
            safe = topic.strip("/").replace("/", "_")
            path = csv_dir / f"{safe}.csv"
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                for s, vals in data_list:
                    row = dict(stamp=s, time=ns_to_str(s), **vals)
                    w.writerow(row)
            print(f"  CSV: {safe}.csv  ({len(data_list)} rows)")

        for name, events in [
            ("apriltag_detections", self.apriltag_events),
            ("yolo_detections", self.yolo_events),
            ("mavros_state", self.mavros_states),
        ]:
            if not events:
                continue
            fieldnames = list(events[0].keys())
            path = csv_dir / f"{name}.csv"
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                w.writerows(events)
            print(f"  CSV: {name}.csv  ({len(events)} rows)")

    # -----------------------------------------------------------------
    #  Video extraction
    # -----------------------------------------------------------------

    def export_videos(self, out_dir: Path):
        video_dir = out_dir / "video"
        video_dir.mkdir(parents=True, exist_ok=True)

        for topic in IMAGE_TOPICS:
            frames = self.image_frames.get(topic, [])
            if not frames:
                print(f"  Video: {topic} — no frames")
                continue
            fps = self.fps_estimates.get(topic, 30.0)
            safe = topic.strip("/").replace("/", "_")
            h, w = frames[0][1].shape[:2]

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(video_dir / f"{safe}.mp4"), fourcc, fps, (w, h))
            for _, frame in frames:
                writer.write(frame)
            writer.release()
            dur = len(frames) / fps
            print(f"  Video: {safe}.mp4  ({len(frames)} frames @ {fps:.1f} FPS, {dur:.1f}s)")

        raw = self.image_frames.get("/camera", [])
        ann = self.image_frames.get("/apriltags/annotated", [])
        if raw and ann:
            self._composite_video(video_dir)

    def _composite_video(self, video_dir: Path):
        raw = self.image_frames["/camera"]
        ann = self.image_frames["/apriltags/annotated"]
        fps = self.fps_estimates.get("/apriltags/annotated", 30.0)
        n = min(len(raw), len(ann))
        if n == 0:
            return

        h, w = raw[0][1].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(video_dir / "composite.mp4"), fourcc, fps, (w * 2, h))

        for i in range(n):
            _, rframe = raw[i]
            _, aframe = ann[i]
            if rframe.shape != aframe.shape:
                aframe = cv2.resize(aframe, (w, h))
            comp = np.hstack([rframe, aframe])
            cv2.putText(comp, "RAW", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(comp, "ANNOTATED", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            ts_str = ns_to_str(raw[i][0]) if raw[i][0] else ""
            cv2.putText(comp, ts_str, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            writer.write(comp)

        writer.release()
        print(f"  Video: composite.mp4  ({n} frames @ {fps:.1f} FPS)")

    # -----------------------------------------------------------------
    #  Plots
    # -----------------------------------------------------------------

    def generate_plots(self, out_dir: Path):
        plot_dir = out_dir / "plots"
        plot_dir.mkdir(parents=True, exist_ok=True)

        any_plot = False
        for method in [
            self._plot_depth, self._plot_heading, self._plot_thrusters,
            self._plot_lights, self._plot_detections, self._plot_battery,
            self._plot_position,
        ]:
            if method(plot_dir):
                any_plot = True

        if not any_plot:
            print("  Plots: no data available")

    def _plot_depth(self, plot_dir):
        if not self.pressure_readings:
            return False
        stamps = [e["stamp"] for e in self.pressure_readings]
        depths = [e["depth_m_approx"] for e in self.pressure_readings]
        t = self._rel_time(stamps)

        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(t, depths, "b-", linewidth=1, label="Depth")

        if self.target_depth_samples:
            td_stamps = [e["stamp"] for e in self.target_depth_samples]
            td_vals = [e["target_depth"] for e in self.target_depth_samples]
            td_t = self._rel_time(td_stamps)
            ax.step(td_t, td_vals, "r--", linewidth=1.5, where="post", label="Target Depth")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Depth (m)")
        ax.set_title("Depth over Time")
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.invert_yaxis()

        fig.tight_layout()
        fig.savefig(plot_dir / "depth.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: depth.png")
        return True

    def _plot_heading(self, plot_dir):
        if not self.heading_samples:
            return False
        stamps = [e["stamp"] for e in self.heading_samples]
        headings = [e["heading"] for e in self.heading_samples]
        t = self._rel_time(stamps)

        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(t, headings, "b-", linewidth=1, label="Heading")

        if self.target_angle_samples:
            ta_stamps = [e["stamp"] for e in self.target_angle_samples]
            ta_vals = [e["target_angle"] for e in self.target_angle_samples]
            ta_t = self._rel_time(ta_stamps)
            ax.plot(ta_t, ta_vals, "r-", linewidth=1, label="Target Angle")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (deg)")
        ax.set_title("Heading Tracking")
        ax.set_ylim(0, 360)
        ax.grid(True, alpha=0.3)
        ax.legend()

        fig.tight_layout()
        fig.savefig(plot_dir / "heading.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: heading.png")
        return True

    def _plot_thrusters(self, plot_dir):
        if not self.manual_control_samples:
            return False
        stamps = [e["stamp"] for e in self.manual_control_samples]
        t = self._rel_time(stamps)

        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        for ax, axis, color in zip(axes, ["x", "z", "r"], ["b", "g", "r"]):
            vals = [e[axis] for e in self.manual_control_samples]
            ax.plot(t, vals, f"{color}-", linewidth=1)
            ax.set_ylabel(f"{axis}")
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color="gray", linestyle="--", linewidth=0.5)

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("Thruster Commands (ManualControl)")
        fig.tight_layout()
        fig.savefig(plot_dir / "thrusters.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: thrusters.png")
        return True

    def _plot_lights(self, plot_dir):
        if not self.flash_events:
            return False
        stamps = [e["stamp"] for e in self.flash_events]
        pwms = [e["pwm"] for e in self.flash_events]
        t = self._rel_time(stamps)

        fig, ax = plt.subplots(figsize=(12, 3))
        ax.step(t, pwms, "y-", linewidth=2, where="post")
        ax.axhline(y=1500, color="gray", linestyle="--", linewidth=0.5, label="Threshold")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("PWM")
        ax.set_title("Light Flasher Activity")
        ax.set_ylim(1000, 2000)
        ax.grid(True, alpha=0.3)
        ax.legend()

        for i in range(len(t) - 1):
            if pwms[i] > 1500:
                ax.axvspan(t[i], t[i + 1], alpha=0.15, color="yellow")

        fig.tight_layout()
        fig.savefig(plot_dir / "lights.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: lights.png")
        return True

    def _plot_detections(self, plot_dir):
        fig, ax = plt.subplots(figsize=(12, 4))

        if self.apriltag_events:
            tag_stamps = [e["stamp"] for e in self.apriltag_events]
            tag_ids = [e["tag_id"] for e in self.apriltag_events]
            ax.scatter(self._rel_time(tag_stamps), tag_ids, c="blue", s=20, alpha=0.6, label="AprilTag", zorder=3)

        if self.yolo_events:
            yo_stamps = [e["stamp"] for e in self.yolo_events]
            yo_counts = [e["count"] for e in self.yolo_events]
            ax.scatter(self._rel_time(yo_stamps), yo_counts, c="red", s=20, alpha=0.6, marker="x", label="YOLO", zorder=3)

        if not self.apriltag_events and not self.yolo_events:
            plt.close(fig)
            return False

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Tag ID / Object Count")
        ax.set_title("Detection Timeline")
        ax.grid(True, alpha=0.3)
        ax.legend()

        fig.tight_layout()
        fig.savefig(plot_dir / "detections.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: detections.png")
        return True

    def _plot_battery(self, plot_dir):
        valid = [e for e in self.battery_samples if e["voltage"] > 0]
        if not valid:
            return False
        stamps = [e["stamp"] for e in valid]
        voltages = [e["voltage"] for e in valid]
        t = self._rel_time(stamps)

        fig, ax = plt.subplots(figsize=(12, 3))
        ax.plot(t, voltages, "g-", linewidth=1)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Voltage (V)")
        ax.set_title("Battery Voltage")
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(plot_dir / "battery.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: battery.png")
        return True

    def _plot_position(self, plot_dir):
        if not self.pose_samples:
            return False
        stamps = [e["stamp"] for e in self.pose_samples]
        t = self._rel_time(stamps)

        fig, axes = plt.subplots(3, 1, figsize=(12, 6), sharex=True)
        for ax, axis, color in zip(axes, ["position_x", "position_y", "position_z"], ["b", "g", "r"]):
            vals = [e[axis] for e in self.pose_samples]
            ax.plot(t, vals, f"{color}-", linewidth=1)
            ax.set_ylabel(axis.split("_")[-1] + " (m)")
            ax.grid(True, alpha=0.3)

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("Local Position (MAVROS)")
        fig.tight_layout()
        fig.savefig(plot_dir / "position.png", dpi=150)
        plt.close(fig)
        print(f"  Plot: position.png")
        return True

    # -----------------------------------------------------------------
    #  HTML report
    # -----------------------------------------------------------------

    def generate_html(self, out_dir: Path):
        dur_ns = self.last_stamp - self.first_stamp if self.first_stamp and self.last_stamp else 0
        plot_dir = out_dir / "plots"

        def img_tag(name: str) -> str:
            p = plot_dir / name
            if p.exists():
                with open(p, "rb") as f:
                    b64 = base64.b64encode(f.read()).decode()
                return f'<img src="data:image/png;base64,{b64}" style="width:100%;max-width:960px;display:block;margin:10px auto;">'
            return ""

        def section(title: str, body: str) -> str:
            return f"<h2>{title}</h2>\n{body}\n"

        def build_table(headers: list[str], rows: list[list], max_rows=100) -> str:
            if not rows:
                return "<p>No data</p>\n"
            h = "".join(f"<th>{x}</th>" for x in headers)
            r = ""
            for row in rows[:max_rows]:
                r += "<tr>" + "".join(f"<td>{v}</td>" for v in row) + "</tr>\n"
            if len(rows) > max_rows:
                r += f'<tr><td colspan="{len(headers)}">... ({len(rows) - max_rows} more rows)</td></tr>\n'
            return f"<table><thead><tr>{h}</tr></thead><tbody>\n{r}</tbody></table>\n"

        content = []

        content.append(f"""
        <div class="stats">
            <div class="stat"><span class="label">Duration</span><span class="value">{format_duration_s(dur_ns)}</span></div>
            <div class="stat"><span class="label">Topics</span><span class="value">{len(self.topic_msg_count)}</span></div>
            <div class="stat"><span class="label">Messages</span><span class="value">{self.topic_msg_count.total():,}</span></div>
            <div class="stat"><span class="label">Start</span><span class="value">{ns_to_str(self.first_stamp) if self.first_stamp else "N/A"}</span></div>
        </div>
        """)

        topic_rows = [[t, f"{c:,}", self._topic_type(t)] for t, c in sorted(self.topic_msg_count.items())]
        content.append(section("Topics", build_table(["Topic", "Messages", "Type"], topic_rows)))

        content.append(section("Depth", img_tag("depth.png") + build_table(
            ["Time", "Depth (m)", "Pressure (Pa)"],
            [(e["time"], f"{e['depth_m_approx']:.2f}", f"{e['pressure_pa']:.0f}") for e in self.pressure_readings[:50]],
        )))

        if self.heading_samples:
            hrows = [[h["time"], str(h["heading"])] for h in self.heading_samples[:50]]
            if self.target_angle_samples:
                for i, row in enumerate(hrows[:50]):
                    if i < len(self.target_angle_samples):
                        row.append(str(self.target_angle_samples[i]["target_angle"]))
                    else:
                        row.append("")
            content.append(section("Heading Tracking", img_tag("heading.png") + build_table(
                ["Time", "Heading (deg)", "Target Angle (deg)"], hrows,
            )))

        content.append(section("Thruster Commands", img_tag("thrusters.png") + build_table(
            ["Time", "X", "Y", "Z", "R"],
            [(e["time"], f"{e['x']:.2f}", f"{e['y']:.2f}", f"{e['z']:.2f}", f"{e['r']:.2f}") for e in self.manual_control_samples[:50]],
        )))

        content.append(section("Light Flasher", img_tag("lights.png") + build_table(
            ["Time", "PWM", "Status"],
            [(e["time"], str(e["pwm"]), "ON" if e["pwm"] > 1500 else "OFF") for e in self.flash_events[:50]],
        )))

        content.append(section("Detection Timeline", img_tag("detections.png")))
        if self.apriltag_events:
            content.append(section("AprilTag Detections", build_table(
                ["Time", "Tag ID", "Distance"],
                [(e["time"], str(e["tag_id"]), f"{e['distance']:.3f}m" if e["distance"] else "N/A") for e in self.apriltag_events[:100]],
            )))

        if self.battery_samples:
            bhead = ["Time", "Voltage", "Current", "Battery %"]
            brows = [(e["time"], f"{e['voltage']:.2f}V", f"{e['current']:.3f}A", f"{e['percentage']*100:.0f}%") for e in self.battery_samples[:30]]
            content.append(section("Battery", img_tag("battery.png") + build_table(bhead, brows)))

        if self.pose_samples:
            content.append(section("Position", img_tag("position.png")))

        video_dir = out_dir / "video"
        video_links = []
        for fname in ["composite.mp4", "camera_raw.mp4", "apriltags_annotated.mp4"]:
            if (video_dir / fname).exists():
                video_links.append(f'<p><a href="../video/{fname}">Play {fname}</a></p>\n')
        if video_links:
            content.append(section("Video", '<div class="videos">\n' + "".join(video_links) + "</div>\n"))

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>AUVC Bag Report - {self.bag_path.name}</title>
<style>
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #0d1117; color: #c9d1d9; padding: 20px; }}
    h1 {{ color: #58a6ff; margin-bottom: 10px; }}
    h2 {{ color: #f0883e; margin: 30px 0 10px; border-bottom: 1px solid #30363d; padding-bottom: 5px; }}
    .stats {{ display: flex; gap: 20px; flex-wrap: wrap; margin: 20px 0; }}
    .stat {{ background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 15px 25px; }}
    .stat .label {{ display: block; font-size: 12px; color: #8b949e; text-transform: uppercase; }}
    .stat .value {{ display: block; font-size: 24px; font-weight: bold; }}
    table {{ width: 100%; border-collapse: collapse; margin: 10px 0; font-size: 13px; }}
    th, td {{ padding: 6px 10px; text-align: left; border-bottom: 1px solid #21262d; }}
    th {{ background: #161b22; color: #8b949e; font-weight: 600; position: sticky; top: 0; }}
    tr:hover {{ background: #1c2128; }}
    .videos p {{ margin: 5px 0; }}
    .videos a {{ color: #58a6ff; text-decoration: none; }}
    .videos a:hover {{ text-decoration: underline; }}
    img {{ border-radius: 4px; border: 1px solid #30363d; }}
    .footer {{ margin-top: 40px; padding-top: 10px; border-top: 1px solid #30363d; color: #8b949e; font-size: 12px; }}
</style>
</head>
<body>
<h1>AUVC Bag Report</h1>
<p>Bag: <strong>{self.bag_path.name}</strong> | {ns_to_str(self.first_stamp) if self.first_stamp else ""}</p>
{"".join(content)}
<div class="footer">Generated by bag_summary.py at {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}</div>
</body>
</html>"""

        html_path = out_dir / "summary.html"
        with open(html_path, "w") as f:
            f.write(html)
        print(f"  HTML: summary.html")

    # -----------------------------------------------------------------
    #  Run all
    # -----------------------------------------------------------------

    def run_all(self, output_dir: Path = None):
        if output_dir is None:
            output_dir = self.bag_path
        output_dir = Path(output_dir)
        os.makedirs(output_dir, exist_ok=True)

        self.analyze()

        print()
        print("-" * 65)
        print("  OUTPUTS")
        print("-" * 65)
        print()

        print("TEXT SUMMARY:")
        print()
        self.print_text_summary()

        print("CSV EXPORT:")
        self.export_csvs(output_dir)
        print()

        print("VIDEO EXTRACTION:")
        self.export_videos(output_dir)
        print()

        print("PLOTS:")
        self.generate_plots(output_dir)
        print()

        print("HTML REPORT:")
        self.generate_html(output_dir)
        print()

        print("Done. Output directory: " + str(output_dir.resolve()))


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 bag_summary.py <bag_directory> [output_directory]")
        print()
        print("  <bag_directory>     Path to the ROS2 MCAP bag directory")
        print("  [output_directory]  Output dir (default: same as bag directory)")
        sys.exit(1)

    bag_path = sys.argv[1]
    if not Path(bag_path).is_dir():
        print(f"Error: bag directory not found: {bag_path}")
        sys.exit(1)

    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    rclpy.init()
    try:
        analyzer = BagAnalyzer(bag_path)
        analyzer.run_all(output_dir)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
