"""Interactive capture controller with episode-based recording (ROS2 Jazzy).

State machine:
  idle -> warmup -> ready -> recording -> ready -> ... -> idle

Commands vary by state (context-sensitive prompt).
"""

import datetime
import json
import os
import platform
import select
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String

STATE_IDLE = "idle"
STATE_WARMUP = "warmup"
STATE_READY = "ready"
STATE_RECORDING = "recording"


class InteractiveRecorder(Node):
    """CLI-driven ros2 bag recording with episode lifecycle."""

    def __init__(self) -> None:
        super().__init__("interactive_capture")

        self.declare_parameter("bag_dir", "outputs")
        self.declare_parameter("base_name", "capture")
        self.declare_parameter("prompt_interval_sec", 0.5)
        self.declare_parameter("topics", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("warmup_duration_s", 15.0)
        self.declare_parameter("episode_topic", "/session/episode")

        self.bag_dir = os.path.abspath(
            self.get_parameter("bag_dir").value
        )
        self.base_name = self.get_parameter("base_name").value
        self.prompt_interval = float(
            self.get_parameter("prompt_interval_sec").value
        )
        self.topics: list[str] = list(
            self.get_parameter("topics").value or []
        )
        self.warmup_duration_s = float(
            self.get_parameter("warmup_duration_s").value
        )
        self.episode_topic: str = self.get_parameter("episode_topic").value

        if not self.topics:
            raise ValueError("'topics' parameter must be a non-empty list")

        if self.episode_topic not in self.topics:
            self.topics.append(self.episode_topic)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.episode_pub = self.create_publisher(
            String, self.episode_topic, qos
        )

        self.state = STATE_IDLE
        self.record_proc: Optional[subprocess.Popen] = None
        self.active_bag_dir: Optional[str] = None
        self.episode_counter = 0
        self.episodes: List[dict] = []
        self.warmup_start_time: Optional[float] = None

    def run(self) -> None:
        os.makedirs(self.bag_dir, exist_ok=True)
        self._print_help()
        self._print_prompt()

        while rclpy.ok():
            if self.state == STATE_WARMUP:
                self._warmup_tick()
                continue

            cmd = self._read_command()
            if cmd is None:
                continue

            self._dispatch(cmd)
            self._print_prompt()

        if self.state != STATE_IDLE:
            self._stop_session(discard_current_episode=True)

    def _dispatch(self, cmd: str) -> None:
        if self.state == STATE_IDLE:
            if cmd == "s":
                self._start_session()
            elif cmd == "l":
                self._list_recordings()
            elif cmd == "r":
                self._delete_last_recording()
            elif cmd == "q":
                self._quit()
            else:
                self._print_help()

        elif self.state == STATE_READY:
            if cmd == "e":
                self._start_episode()
            elif cmd == "c":
                self._stop_session(discard_current_episode=False)
            elif cmd == "q":
                self._quit()
            else:
                print("In READY state. Use: e(episode), c(end session), q(quit)")

        elif self.state == STATE_RECORDING:
            if cmd == "e":
                self._end_episode()
            elif cmd == "c":
                self._stop_session(discard_current_episode=True)
            elif cmd == "q":
                self._quit()
            else:
                print("In RECORDING state. Use: e(end episode), "
                      "c(end session + discard current), q(quit)")

    # ---- state transitions ----

    def _start_session(self) -> None:
        if self.state != STATE_IDLE:
            print("Session already active.")
            return

        self.active_bag_dir = self._recording_dir()
        self.episode_counter = 0
        self.episodes = []

        cmd = [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", self.active_bag_dir,
        ] + self.topics
        print("Starting session recording:")
        print("  {}".format(" ".join(shlex.quote(part) for part in cmd)))

        self.record_proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid
        )
        time.sleep(1.0)

        self._write_session_sidecar()

        self._publish_event("warmup_start")
        self.warmup_start_time = time.time()
        self.state = STATE_WARMUP

        print("")
        print("=" * 60)
        print("IMU WARM-UP: Move the D455 with slow, smooth translations")
        print("in a textured area for {:.0f} seconds.".format(
            self.warmup_duration_s
        ))
        print("DO NOT: hold still, rotate only, or move too fast.")
        print("=" * 60)

    def _warmup_tick(self) -> None:
        elapsed = time.time() - self.warmup_start_time
        remaining = self.warmup_duration_s - elapsed

        if remaining <= 0:
            self._publish_event("warmup_end")
            self.state = STATE_READY
            print("")
            print("=" * 60)
            print("WARM-UP COMPLETE. Device is ready to record.")
            print("Press 'e' to start an episode, 'c' to end session.")
            print("=" * 60)
            self._print_prompt()
            return

        ready, _, _ = select.select([sys.stdin], [], [], 1.0)
        if ready:
            sys.stdin.readline()
            print("  (warm-up cannot be skipped, "
                  "{:.0f}s remaining)".format(remaining))
        else:
            sys.stdout.write(
                "\r  [warm-up] {:.0f}s / {:.0f}s elapsed "
                "— keep moving...  ".format(elapsed, self.warmup_duration_s)
            )
            sys.stdout.flush()

    def _start_episode(self) -> None:
        self.episode_counter += 1
        eid = self.episode_counter
        self._publish_event("episode_start:{}".format(eid))
        self.episodes.append({"id": eid, "status": "recording"})
        self.state = STATE_RECORDING
        print("")
        print("[episode {}] Recording started. "
              "Press 'e' to end episode.".format(eid))

    def _end_episode(self) -> None:
        eid = self.episode_counter
        self._publish_event("episode_end:{}".format(eid))
        for ep in self.episodes:
            if ep["id"] == eid and ep["status"] == "recording":
                ep["status"] = "kept"
        self.state = STATE_READY
        kept = sum(1 for ep in self.episodes if ep["status"] == "kept")
        print("[episode {}] Ended. Total kept: {}. "
              "Press 'e' for next, 'c' to end session.".format(eid, kept))

    def _discard_current_episode(self) -> None:
        if self.state != STATE_RECORDING:
            return
        eid = self.episode_counter
        self._publish_event("episode_discard:{}".format(eid))
        for ep in self.episodes:
            if ep["id"] == eid:
                ep["status"] = "discarded"
        print("[episode {}] Discarded.".format(eid))

    def _stop_session(self, discard_current_episode: bool = False) -> None:
        if self.state == STATE_RECORDING:
            if discard_current_episode:
                self._discard_current_episode()
            else:
                self._end_episode()

        if self.record_proc is None:
            self.state = STATE_IDLE
            return

        print("Stopping session recording...")
        try:
            os.killpg(os.getpgid(self.record_proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            self.record_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("Recorder did not stop in time; terminating.")
            try:
                os.killpg(os.getpgid(self.record_proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            self.record_proc.wait(timeout=5)

        self._update_session_sidecar()

        if self.active_bag_dir and os.path.isdir(self.active_bag_dir):
            kept = sum(1 for ep in self.episodes if ep["status"] == "kept")
            discarded = sum(
                1 for ep in self.episodes if ep["status"] == "discarded"
            )
            print("Session saved: {}".format(self.active_bag_dir))
            print("  Episodes: {} kept, {} discarded".format(kept, discarded))
        else:
            print("Session stopped. No bag directory found.")

        self.record_proc = None
        self.active_bag_dir = None
        self.state = STATE_IDLE

    def _quit(self) -> None:
        if self.state != STATE_IDLE:
            self._stop_session(discard_current_episode=True)
        print("Exiting interactive capture controller.")
        rclpy.shutdown()

    # ---- episode topic ----

    def _publish_event(self, event_str: str) -> None:
        msg = String()
        msg.data = event_str
        self.episode_pub.publish(msg)
        self.get_logger().info("Episode event: %s" % event_str)

    # ---- sidecar ----

    def _sidecar_path(self) -> Optional[str]:
        if self.active_bag_dir is None:
            return None
        return self.active_bag_dir + ".session.json"

    def _write_session_sidecar(self) -> None:
        path = self._sidecar_path()
        if path is None:
            return
        try:
            now_ros = self.get_clock().now()
            sidecar = {
                "ros_time_ns": now_ros.nanoseconds,
                "wall_clock_ns": time.time_ns(),
                "perf_counter_ns": time.perf_counter_ns(),
                "hostname": platform.node(),
                "kernel": platform.release(),
                "topics": self.topics,
                "warmup_duration_s": self.warmup_duration_s,
                "episodes": [],
                "ros_distro": "jazzy",
                "bag_format": "mcap",
                "created_utc": (
                    datetime.datetime.utcnow().isoformat() + "Z"
                ),
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(sidecar, f, indent=2)
            print("Wrote session sidecar: {}".format(path))
        except Exception as exc:
            self.get_logger().warn(
                "Failed to write session sidecar: %s" % exc
            )

    def _update_session_sidecar(self) -> None:
        path = self._sidecar_path()
        if path is None or not os.path.isfile(path):
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                sidecar = json.load(f)
            sidecar["episodes"] = self.episodes
            sidecar["finished_utc"] = (
                datetime.datetime.utcnow().isoformat() + "Z"
            )
            with open(path, "w", encoding="utf-8") as f:
                json.dump(sidecar, f, indent=2)
        except Exception as exc:
            self.get_logger().warn(
                "Failed to update session sidecar: %s" % exc
            )

    # ---- UI helpers ----

    def _print_help(self) -> None:
        print("")
        print("UMI-Dex Interactive Capture — ROS2 Jazzy (episode-based)")
        print("  s : start new session (IMU warm-up + episode recording)")
        print("  e : start/end episode (within a session)")
        print("  c : end session (save bag with all episodes)")
        print("  l : list saved recordings")
        print("  r : delete last saved recording")
        print("  q : quit")
        print("")

    def _print_prompt(self) -> None:
        if self.state == STATE_IDLE:
            hint = "s(session) l(list) r(remove) q(quit)"
        elif self.state == STATE_READY:
            kept = sum(1 for ep in self.episodes if ep["status"] == "kept")
            hint = "e(episode) c(end session) | {} episodes kept".format(kept)
        elif self.state == STATE_RECORDING:
            hint = "e(end episode {}) c(end session+discard)".format(
                self.episode_counter
            )
        else:
            hint = ""
        sys.stdout.write("[{}] {} > ".format(self.state, hint))
        sys.stdout.flush()

    def _read_command(self) -> Optional[str]:
        while rclpy.ok():
            ready, _, _ = select.select(
                [sys.stdin], [], [], self.prompt_interval
            )
            if not ready:
                continue
            line = sys.stdin.readline()
            if line == "":
                return "q"
            return line.strip().lower()
        return None

    def _recording_dir(self) -> str:
        stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        return os.path.join(
            self.bag_dir, "{}_{}".format(self.base_name, stamp)
        )

    def _all_bags(self) -> list[str]:
        """Return bag directories sorted by modification time."""
        if not os.path.isdir(self.bag_dir):
            return []
        candidates = []
        for name in os.listdir(self.bag_dir):
            full = os.path.join(self.bag_dir, name)
            if os.path.isdir(full) and os.path.isfile(
                os.path.join(full, "metadata.yaml")
            ):
                candidates.append(full)
        return sorted(candidates, key=os.path.getmtime)

    def _list_recordings(self) -> None:
        bags = self._all_bags()
        if not bags:
            print("No recordings found.")
            return
        print("Recordings:")
        for bag in bags:
            self._print_bag_summary(bag, indent="  ")

    def _delete_last_recording(self) -> None:
        bags = self._all_bags()
        if not bags:
            print("No finished recording to delete.")
            return
        last = bags[-1]
        shutil.rmtree(last, ignore_errors=True)
        sidecar = last + ".session.json"
        if os.path.isfile(sidecar):
            os.remove(sidecar)
        print("Deleted: {}".format(last))

    def _print_bag_summary(self, bag_dir: str, indent: str = "") -> None:
        name = os.path.basename(bag_dir)
        try:
            result = subprocess.run(
                ["ros2", "bag", "info", bag_dir],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                for line in result.stdout.strip().splitlines():
                    print("{}{}".format(indent, line))
            else:
                print("{}{} (info unavailable)".format(indent, name))
        except Exception:
            print("{}{} (info unavailable)".format(indent, name))


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = InteractiveRecorder()
    except ValueError as exc:
        print("FATAL: {}".format(exc), file=sys.stderr)
        rclpy.try_shutdown()
        return
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
