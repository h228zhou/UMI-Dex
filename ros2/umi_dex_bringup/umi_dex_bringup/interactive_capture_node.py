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
import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

STATE_IDLE = "idle"
STATE_WARMUP = "warmup"
STATE_READY = "ready"
STATE_RECORDING = "recording"

IR1_TOPIC = "/camera/infra1/image_rect_raw"
IR2_TOPIC = "/camera/infra2/image_rect_raw"
IMU_TOPIC = "/camera/imu"


class InteractiveRecorder(Node):
    """CLI-driven ros2 bag recording with episode lifecycle."""

    def __init__(self) -> None:
        super().__init__("interactive_capture")

        self.declare_parameter("bag_dir", "outputs")
        self.declare_parameter("base_name", "capture")
        self.declare_parameter("prompt_interval_sec", 0.5)
        self.declare_parameter("topics", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("warmup_min_wait_s", 8.0)
        self.declare_parameter("warmup_ok_sustain_s", 2.0)
        self.declare_parameter("warmup_timeout_s", 60.0)
        self.declare_parameter("episode_topic", "/session/episode")
        self.declare_parameter("enable_slam_probe", True)
        self.declare_parameter("slam_vocab_path", "config/ORBvoc.txt")
        self.declare_parameter("slam_settings_path", "config/intel_d455.yaml")

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
        self.warmup_min_wait_s = float(
            self.get_parameter("warmup_min_wait_s").value
        )
        self.warmup_ok_sustain_s = float(
            self.get_parameter("warmup_ok_sustain_s").value
        )
        self.warmup_timeout_s = float(
            self.get_parameter("warmup_timeout_s").value
        )
        self.episode_topic: str = self.get_parameter("episode_topic").value
        self.enable_slam_probe: bool = bool(
            self.get_parameter("enable_slam_probe").value
        )
        self.slam_vocab_path: str = self.get_parameter("slam_vocab_path").value
        self.slam_settings_path: str = self.get_parameter(
            "slam_settings_path"
        ).value

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

        self._probe = None  # type: Optional[object]
        self._slam_subs: list = []
        self._likely_ready_since: Optional[float] = None
        self._last_warmup_snapshot: Optional[dict] = None
        self._warmup_incomplete: bool = False

        # Executor spins subscriber callbacks in a background thread so the
        # interactive stdin loop in run() doesn't starve them.
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

    def start_executor(self) -> None:
        if self._executor is not None:
            return
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="rclpy-spin", daemon=True,
        )
        self._spin_thread.start()

    def stop_executor(self) -> None:
        if self._executor is None:
            return
        try:
            self._executor.shutdown()
        except Exception:
            pass
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._executor = None
        self._spin_thread = None

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
        self._last_warmup_snapshot = None
        self._warmup_incomplete = False
        self._likely_ready_since = None

        cmd = [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", self.active_bag_dir,
        ] + self.topics
        print("Starting session recording:")
        print("  {}".format(" ".join(shlex.quote(part) for part in cmd)))

        self.record_proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid, stdin=subprocess.DEVNULL
        )
        time.sleep(1.0)

        self._write_session_sidecar()

        self._start_probe()

        self._publish_event("warmup_start")
        self.warmup_start_time = time.time()
        self.state = STATE_WARMUP

        print("")
        print("=" * 60)
        print("IMU WARM-UP: Move the D455 with slow, smooth translations")
        print("in a textured area until the SLAM probe reports ready")
        print("(min {:.0f}s, max {:.0f}s).".format(
            self.warmup_min_wait_s, self.warmup_timeout_s
        ))
        print("DO NOT: hold still, rotate only, or move too fast.")
        print("=" * 60)

    def _start_probe(self) -> None:
        if not self.enable_slam_probe:
            return
        try:
            from umi_dex.slam.online import WarmupProbe
        except Exception as exc:
            self.get_logger().warn(
                "SLAM probe disabled: orbslam3 import failed: %s" % exc
            )
            self._probe = None
            return

        vocab = os.path.abspath(self.slam_vocab_path)
        settings = os.path.abspath(self.slam_settings_path)
        if not os.path.isfile(vocab) or not os.path.isfile(settings):
            self.get_logger().warn(
                "SLAM probe disabled: vocab or settings missing (%s, %s)"
                % (vocab, settings)
            )
            self._probe = None
            return

        try:
            self._probe = WarmupProbe(vocab, settings)
            self._probe.start()
        except Exception as exc:
            self.get_logger().warn("SLAM probe init failed: %s" % exc)
            self._probe = None
            return

        self._slam_subs = [
            self.create_subscription(
                Image, IR1_TOPIC, self._on_ir1, qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, IR2_TOPIC, self._on_ir2, qos_profile_sensor_data,
            ),
            self.create_subscription(
                Imu, IMU_TOPIC, self._on_imu, qos_profile_sensor_data,
            ),
        ]

    def _stop_probe(self) -> None:
        for sub in self._slam_subs:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
        self._slam_subs = []
        if self._probe is not None:
            try:
                self._last_warmup_snapshot = self._probe.snapshot()
            except Exception:
                pass
            try:
                self._probe.shutdown()
            except Exception:
                pass
            self._probe = None

    def _on_ir1(self, msg: Image) -> None:
        if self._probe is None:
            return
        img = self._image_msg_to_mono(msg)
        if img is None:
            return
        t_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        self._probe.push_ir1(t_ns, img)

    def _on_ir2(self, msg: Image) -> None:
        if self._probe is None:
            return
        img = self._image_msg_to_mono(msg)
        if img is None:
            return
        t_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        self._probe.push_ir2(t_ns, img)

    def _on_imu(self, msg: Imu) -> None:
        if self._probe is None:
            return
        t_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        gv = msg.angular_velocity
        av = msg.linear_acceleration
        self._probe.push_imu(
            t_ns, gv.x, gv.y, gv.z, av.x, av.y, av.z,
        )

    @staticmethod
    def _image_msg_to_mono(msg: Image) -> Optional[np.ndarray]:
        try:
            h, w = int(msg.height), int(msg.width)
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if arr.size < h * w:
                return None
            return arr[: h * w].reshape(h, w)
        except Exception:
            return None

    def _warmup_tick(self) -> None:
        now = time.time()
        elapsed = now - self.warmup_start_time

        snap = self._probe.snapshot() if self._probe is not None else None
        proxy_state = snap["proxy_state"] if snap else None

        if proxy_state == "likely_ready":
            if self._likely_ready_since is None:
                self._likely_ready_since = now
        else:
            self._likely_ready_since = None

        if self._probe is not None:
            gated_ready = (
                elapsed >= self.warmup_min_wait_s
                and self._likely_ready_since is not None
                and (now - self._likely_ready_since) >= self.warmup_ok_sustain_s
            )
            if gated_ready:
                self._finish_warmup(
                    "slam-gated at {:.0f}s".format(elapsed),
                    incomplete=False,
                )
                return
            if elapsed >= self.warmup_timeout_s:
                self._force_continue_prompt(elapsed, snap)
                return
        else:
            if elapsed >= self.warmup_timeout_s:
                self._finish_warmup(
                    "wall-clock {:.0f}s".format(elapsed),
                    incomplete=False,
                )
                return

        self._render_warmup_line(elapsed, snap)

    def _render_warmup_line(self, elapsed: float, snap: Optional[dict]) -> None:
        ready, _, _ = select.select([sys.stdin], [], [], 1.0)
        if ready:
            line = sys.stdin.readline()
            cmd = line.strip().lower() if line else ""
            if cmd == "c":
                self._stop_probe()
                self._stop_session(discard_current_episode=True)
                self._print_prompt()
                return
            if cmd == "q":
                self._stop_probe()
                self._quit()
                return
            print(
                "  (warm-up cannot be skipped; press 'c' to abort session "
                "or 'q' to quit, {:.0f}s elapsed)".format(elapsed)
            )
            return

        if snap is None:
            line = "\r  [warm-up] {:.0f}s/{:.0f}s — keep moving...  ".format(
                elapsed, self.warmup_timeout_s,
            )
        else:
            proxy = snap["proxy_state"]
            frames = snap.get("frames_processed", 0)
            viba = snap.get("viba_stage", 0)
            if proxy in ("loading", "waiting", "error"):
                detail = proxy if proxy != "error" else "error({})".format(
                    snap.get("error") or "?"
                )
                line = (
                    "\r  [warm-up] {:.0f}s/{:.0f}s | slam: {} viba={}/2 "
                    "frames={}  ".format(
                        elapsed, self.warmup_timeout_s, detail, viba, frames,
                    )
                )
            else:
                ok_for = 0.0
                if self._likely_ready_since is not None:
                    ok_for = time.time() - self._likely_ready_since
                label = "READY" if proxy == "likely_ready" else "tracking"
                line = (
                    "\r  [warm-up] {:.0f}s/{:.0f}s | slam: {} viba={}/2 "
                    "mp={} kf={} frames={} ok_for={:.1f}s  ".format(
                        elapsed, self.warmup_timeout_s, label, viba,
                        snap["active_map_points"], snap["num_keyframes"],
                        frames, ok_for,
                    )
                )
        sys.stdout.write(line)
        sys.stdout.flush()

    def _finish_warmup(self, reason: str, incomplete: bool) -> None:
        if self._probe is not None:
            self._last_warmup_snapshot = self._probe.snapshot()
        self._warmup_incomplete = bool(incomplete)
        self._stop_probe()
        self._publish_event("warmup_end")
        self.state = STATE_READY
        print("")
        print("=" * 60)
        print("WARM-UP COMPLETE ({}).".format(reason))
        if incomplete:
            print("NOTE: marked warmup_incomplete=true in session sidecar.")
        print("Press 'e' to start an episode, 'c' to end session.")
        print("=" * 60)
        self._print_prompt()

    def _force_continue_prompt(
        self, elapsed: float, snap: Optional[dict]
    ) -> None:
        print("")
        print("=" * 60)
        print(
            "Warmup timed out at {:.0f}s without slam probe reaching "
            "'likely_ready'.".format(elapsed)
        )
        if snap is not None:
            print(
                "  final: state={} mp={} kf={} time_in_ok={:.1f}s".format(
                    snap.get("proxy_state"),
                    snap.get("active_map_points"),
                    snap.get("num_keyframes"),
                    snap.get("time_in_ok_s", 0.0),
                )
            )
        print("Force-continue?  y = proceed with warmup_incomplete=true")
        print("                 n = abort and discard this session")
        print("=" * 60)

        sys.stdout.write("[warmup?] y/n > ")
        sys.stdout.flush()

        while rclpy.ok():
            ready, _, _ = select.select(
                [sys.stdin], [], [], self.prompt_interval
            )
            if not ready:
                continue
            line = sys.stdin.readline()
            if line == "":
                ans = "n"
            else:
                ans = line.strip().lower()
            if ans == "y":
                self._finish_warmup(
                    "force-continue at {:.0f}s".format(elapsed),
                    incomplete=True,
                )
                return
            if ans == "n":
                self._stop_probe()
                self._stop_session(discard_current_episode=True)
                self._print_prompt()
                return
            sys.stdout.write("[warmup?] y/n > ")
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
        self._stop_probe()
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
                "warmup_min_wait_s": self.warmup_min_wait_s,
                "warmup_ok_sustain_s": self.warmup_ok_sustain_s,
                "warmup_timeout_s": self.warmup_timeout_s,
                "slam_probe_enabled": self.enable_slam_probe,
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
            if self._last_warmup_snapshot is not None:
                sidecar["warmup_slam_snapshot"] = self._last_warmup_snapshot
            sidecar["warmup_incomplete"] = self._warmup_incomplete
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
    node.start_executor()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_executor()
        node.destroy_node()
        rclpy.try_shutdown()
