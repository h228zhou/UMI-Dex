#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""In-process ORB-SLAM3 warmup probe.

Runs a stereo-inertial SLAM pipeline in a background thread during the
interactive-recorder warmup state to give the operator real-time feedback
on tracking quality and to gate the warmup->ready transition.

The probe is advisory: trajectories and map points are NOT persisted. Only
a small snapshot of tracking state / map-point count / keyframe count is
exposed to the UI thread. The host node is expected to shut the probe down
before leaving warmup so SLAM does not compete with the bag writer.

This module reuses the same `orbslam3.System` API and IMU-window logic as
the offline replayer in ``umi_dex.slam.replay``.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np


# Proxy states emitted via snapshot()["proxy_state"].
PROXY_LOADING = "loading"
PROXY_WAITING = "waiting"
PROXY_TRACKING = "tracking"
PROXY_LIKELY_READY = "likely_ready"
PROXY_ERROR = "error"

# ORB-SLAM3 tracking states (from Tracking::eTrackingState).
_TS_OK = 2

# Proxy thresholds for the "likely_ready" classifier (fallback only — used when
# authoritative VIBA-stage detection via stderr scraping is unavailable).
_MIN_MAP_POINTS = 150
_MIN_KEYFRAMES = 2
_MIN_TIME_IN_OK_S = 5.0

# Feed cadence target (seconds between frames pushed to SLAM).
_FEED_INTERVAL_S = 0.1  # ~10 Hz

# Stereo pairing window (nanoseconds).
_STEREO_PAIR_MAX_NS = 50_000_000

# IMU window: copy replay.py's +5ms forward tolerance.
_IMU_WINDOW_FWD_NS = 5_000_000


class WarmupProbe:
    """Runs ORB-SLAM3 stereo-inertial in a background thread for warmup feedback.

    Usage from a ROS node::

        probe = WarmupProbe(vocab_path, settings_path)
        probe.start()
        # subscribe to IR1 / IR2 / IMU topics; in each callback:
        probe.push_ir1(t_ns, img)
        probe.push_ir2(t_ns, img)
        probe.push_imu(t_ns, gx, gy, gz, ax, ay, az)
        # periodically in UI thread:
        snap = probe.snapshot()
        # before recording starts:
        probe.shutdown()
    """

    def __init__(self, vocab_path: Path | str, settings_path: Path | str) -> None:
        self._vocab_path = Path(vocab_path)
        self._settings_path = Path(settings_path)

        self._slam = None
        self._t_base_ns: Optional[int] = None
        self._prev_tl_ns: Optional[int] = None
        self._last_imu_row: Optional[tuple] = None
        self._last_feed_wall_s: float = 0.0
        self._frames_processed: int = 0
        self._t_first_ok_wall_s: Optional[float] = None
        self._t_last_ok_wall_s: Optional[float] = None

        self._ir1_buf: deque[tuple[int, np.ndarray]] = deque(maxlen=4)
        self._ir2_buf: deque[tuple[int, np.ndarray]] = deque(maxlen=8)
        self._imu_buf: deque[tuple[int, float, float, float, float, float, float]] = deque(maxlen=4000)

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Authoritative VIBA stage from scraped ORB-SLAM3 stderr.
        # 0 = not started, 1 = VIBA 1 complete, 2 = VIBA 2 complete (IMU warm).
        self._viba_stage: int = 0
        self._stderr_orig_fd: Optional[int] = None
        self._stderr_pipe_r: Optional[int] = None
        self._stderr_reader: Optional[threading.Thread] = None

        # Snapshot fields (guarded by _lock).
        self._snap: dict = {
            "proxy_state": PROXY_LOADING,
            "tracking_state": -1,
            "active_map_points": 0,
            "num_keyframes": 0,
            "total_map_points": 0,
            "frames_processed": 0,
            "time_in_ok_s": 0.0,
            "viba_stage": 0,
            "error": None,
        }

    # ---- lifecycle ----

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._worker, name="warmup-slam-probe", daemon=True
        )
        self._thread.start()

    def shutdown(self, timeout_s: float = 5.0) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            self._thread = None

    # ---- producers (called from ROS subscriber callbacks) ----

    def push_ir1(self, t_ns: int, img: np.ndarray) -> None:
        with self._lock:
            self._ir1_buf.append((int(t_ns), img))
        self._wake.set()

    def push_ir2(self, t_ns: int, img: np.ndarray) -> None:
        with self._lock:
            self._ir2_buf.append((int(t_ns), img))

    def push_imu(
        self,
        t_ns: int,
        gx: float, gy: float, gz: float,
        ax: float, ay: float, az: float,
    ) -> None:
        with self._lock:
            self._imu_buf.append(
                (int(t_ns), float(gx), float(gy), float(gz),
                 float(ax), float(ay), float(az))
            )

    # ---- consumer (UI thread) ----

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snap)

    # ---- worker ----

    def _start_stderr_capture(self) -> None:
        """Redirect OS fd 2 to a pipe and start a reader thread that parses
        ORB-SLAM3's stderr lines for VIBA events.

        Lines are tee'd back to the saved original fd so the operator still
        sees ORB-SLAM3's output (and any rclpy logs that go to stderr).

        Safe to no-op on failure — in that case VIBA detection falls back to
        the heuristic proxy classifier.
        """
        try:
            sys.stderr.flush()
        except Exception:
            pass

        try:
            orig_fd = os.dup(2)
            r_fd, w_fd = os.pipe()
            os.dup2(w_fd, 2)
            os.close(w_fd)
        except Exception:
            return

        self._stderr_orig_fd = orig_fd
        self._stderr_pipe_r = r_fd
        self._stderr_reader = threading.Thread(
            target=self._stderr_worker,
            name="warmup-slam-stderr",
            daemon=True,
        )
        self._stderr_reader.start()

    def _stop_stderr_capture(self) -> None:
        """Restore the original stderr fd and wind down the reader thread."""
        orig = self._stderr_orig_fd
        r_fd = self._stderr_pipe_r
        if orig is not None:
            try:
                os.dup2(orig, 2)
            except Exception:
                pass
        # Closing fd 2 isn't right; closing orig after dup2'ing back is.
        if orig is not None:
            try:
                os.close(orig)
            except Exception:
                pass
            self._stderr_orig_fd = None
        # After dup2 above, the old write-end of the pipe (which was fd 2) is
        # no longer referenced. Reader will get EOF next read.
        if self._stderr_reader is not None:
            self._stderr_reader.join(timeout=1.0)
            self._stderr_reader = None
        if r_fd is not None:
            try:
                os.close(r_fd)
            except Exception:
                pass
            self._stderr_pipe_r = None

    def _stderr_worker(self) -> None:
        r_fd = self._stderr_pipe_r
        orig_fd = self._stderr_orig_fd
        if r_fd is None or orig_fd is None:
            return

        buf = bytearray()
        while True:
            try:
                chunk = os.read(r_fd, 4096)
            except OSError:
                break
            if not chunk:
                break
            # Tee to the saved original stderr so nothing is lost.
            try:
                os.write(orig_fd, chunk)
            except OSError:
                pass
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl])
                del buf[: nl + 1]
                self._parse_stderr_line(line)

    def _parse_stderr_line(self, line: bytes) -> None:
        try:
            text = line.decode("utf-8", errors="replace")
        except Exception:
            return
        stripped = text.strip()
        # Exact-match the two SLAM milestones we care about. Other lines
        # (start VIBA 1, start VIBA 2, "not enough acceleration", etc.) are
        # available if we ever want richer diagnostics.
        if stripped == "end VIBA 1":
            with self._lock:
                if self._viba_stage < 1:
                    self._viba_stage = 1
                    self._snap["viba_stage"] = 1
        elif stripped == "end VIBA 2":
            with self._lock:
                if self._viba_stage < 2:
                    self._viba_stage = 2
                    self._snap["viba_stage"] = 2

    def _worker(self) -> None:
        try:
            import orbslam3
        except Exception as exc:
            self._set_error(f"orbslam3 import failed: {exc}")
            return

        self._start_stderr_capture()
        try:
            try:
                self._slam = orbslam3.System(
                    str(self._vocab_path),
                    str(self._settings_path),
                    orbslam3.Sensor.IMU_STEREO,
                )
                self._slam.initialize()
            except Exception as exc:
                self._set_error(f"orbslam3 init failed: {exc}")
                return

            with self._lock:
                self._snap["proxy_state"] = PROXY_WAITING

            while not self._stop.is_set():
                if not self._wake.wait(timeout=0.2):
                    continue
                self._wake.clear()

                while not self._stop.is_set():
                    fed = self._process_one()
                    if not fed:
                        break

            try:
                self._slam.shutdown()
            except Exception:
                pass
        finally:
            self._stop_stderr_capture()

    def _process_one(self) -> bool:
        """Attempt to feed a single decimated frame. Returns True if fed."""
        now = time.perf_counter()
        if now - self._last_feed_wall_s < _FEED_INTERVAL_S:
            return False

        # Grab latest IR1, clear the buffer (decimation).
        with self._lock:
            if not self._ir1_buf:
                return False
            tl_ns, iml = self._ir1_buf[-1]
            self._ir1_buf.clear()
            ir2_snap = list(self._ir2_buf)
            imu_snap = list(self._imu_buf)

        if not ir2_snap:
            return False

        # Nearest-match IR2 within 50ms.
        tr_ns, imr = min(ir2_snap, key=lambda p: abs(p[0] - tl_ns))
        if abs(tr_ns - tl_ns) > _STEREO_PAIR_MAX_NS:
            return False

        if self._t_base_ns is None:
            self._t_base_ns = tl_ns
        t_base_s = self._t_base_ns / 1e9
        tl_s = (tl_ns / 1e9) - t_base_s

        # Window IMU samples up to tl_ns (+5ms), skip any strictly before prev_tl_ns
        # to preserve monotonicity (mirrors replay.py:169-182).
        frame_imu: list[tuple[float, float, float, float, float, float, float]] = []
        consumed_up_to_ns: Optional[int] = None
        for row in imu_snap:
            if row[0] > tl_ns + _IMU_WINDOW_FWD_NS:
                break
            consumed_up_to_ns = row[0]
            if self._prev_tl_ns is not None and row[0] < self._prev_tl_ns:
                continue
            ts_rel = (row[0] / 1e9) - t_base_s
            # ORB-SLAM3 expects (ax, ay, az, gx, gy, gz, ts).
            frame_imu.append((row[4], row[5], row[6], row[1], row[2], row[3], ts_rel))

        # Drop consumed IMU rows from the shared buffer.
        if consumed_up_to_ns is not None:
            with self._lock:
                while self._imu_buf and self._imu_buf[0][0] <= consumed_up_to_ns:
                    self._imu_buf.popleft()

        if not frame_imu:
            if self._last_imu_row is not None:
                ax, ay, az, gx, gy, gz, _ = self._last_imu_row
                frame_imu.append((ax, ay, az, gx, gy, gz, tl_s))
            else:
                frame_imu.append((0.0, 9.81, 0.0, 0.0, 0.0, 0.0, tl_s))

        try:
            self._slam.process_stereo_inertial_enhanced(iml, imr, tl_s, frame_imu)
        except Exception as exc:
            self._set_error(f"SLAM step failed: {exc}")
            return False

        self._last_imu_row = frame_imu[-1]
        self._prev_tl_ns = tl_ns
        self._frames_processed += 1
        self._last_feed_wall_s = now

        self._refresh_snapshot(now)
        return True

    def _refresh_snapshot(self, now_wall_s: float) -> None:
        tracking_state = -1
        active_map_points = 0
        num_keyframes = 0
        total_map_points = 0
        map_info_err: Optional[str] = None
        tracked_pts_err: Optional[str] = None

        try:
            mi = self._slam.get_map_info()
            tracking_state = int(mi.tracking_state)
            active_map_points = int(mi.active_map_points)
            num_keyframes = int(mi.num_keyframes)
            total_map_points = int(mi.total_map_points)
        except Exception as exc:
            map_info_err = str(exc)

        # Fallback: if get_map_info() is flaky in this binding version, count
        # tracked map points directly. replay.py has the same try/except around
        # get_map_info, indicating this API can fail in practice.
        if map_info_err is not None:
            try:
                pts = list(self._slam.get_tracked_mappoints())
                active_map_points = len(pts)
            except Exception as exc:
                tracked_pts_err = str(exc)

            # Without get_map_info() we cannot read tracking_state directly.
            # Infer "tracking" heuristically: ≥10 processed frames AND ≥150
            # tracked map points means SLAM is actively mapping — strong proxy
            # for tracking_state == OK.
            if self._frames_processed >= 10 and active_map_points >= _MIN_MAP_POINTS:
                tracking_state = _TS_OK

        is_ok = tracking_state == _TS_OK
        if is_ok:
            if self._t_first_ok_wall_s is None:
                self._t_first_ok_wall_s = now_wall_s
            self._t_last_ok_wall_s = now_wall_s
            time_in_ok_s = now_wall_s - self._t_first_ok_wall_s
        else:
            self._t_first_ok_wall_s = None
            self._t_last_ok_wall_s = None
            time_in_ok_s = 0.0

        if is_ok and time_in_ok_s >= _MIN_TIME_IN_OK_S \
                and active_map_points > _MIN_MAP_POINTS \
                and num_keyframes >= _MIN_KEYFRAMES:
            proxy_state = PROXY_LIKELY_READY
        elif is_ok:
            proxy_state = PROXY_TRACKING
        else:
            proxy_state = PROXY_WAITING

        # In the fallback path num_keyframes is unknown. Don't let that gate
        # likely_ready if the direct-tracked-points path is otherwise green.
        if proxy_state == PROXY_TRACKING and num_keyframes == 0 \
                and map_info_err is not None \
                and is_ok and time_in_ok_s >= _MIN_TIME_IN_OK_S \
                and active_map_points > _MIN_MAP_POINTS:
            proxy_state = PROXY_LIKELY_READY

        # Authoritative override: if stderr scraping caught "end VIBA 2",
        # IMU init is fully complete regardless of what the proxy says.
        if self._viba_stage >= 2:
            proxy_state = PROXY_LIKELY_READY

        with self._lock:
            self._snap.update(
                proxy_state=proxy_state,
                tracking_state=tracking_state,
                active_map_points=active_map_points,
                num_keyframes=num_keyframes,
                total_map_points=total_map_points,
                frames_processed=self._frames_processed,
                time_in_ok_s=time_in_ok_s,
                viba_stage=self._viba_stage,
            )
            if map_info_err is not None:
                self._snap["get_map_info_error"] = map_info_err
            if tracked_pts_err is not None:
                self._snap["get_tracked_mappoints_error"] = tracked_pts_err

    def _set_error(self, msg: str) -> None:
        with self._lock:
            self._snap["proxy_state"] = PROXY_ERROR
            self._snap["error"] = msg
