"""
rail_pose_engine.py
------------------------------------------------------------
Independent rail odometry engine for 0xBF protocol.

Design:
- Independent from radarControl / mmWaveStudio / serial port.
- Input is ONLY: (t_send, raw_packet bytes) where raw_packet is 0xBF frame.
- Two motors run in parallel: motor0 -> X axis, motor1 -> Y axis.
- Overwrite semantics on the same motor: new command immediately truncates the old one.
- Output is a continuous-time piecewise-linear trajectory x(t), y(t) in steps.
- All internal times are relative and start from t=0.0.

Author: (project internal)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import struct
from typing import List, Optional, Tuple, Iterable, Dict


# ============================================================
# Data structures
# ============================================================

@dataclass
class BFCommand:
    """Parsed 0xBF command."""
    t_send_abs: float      # absolute time (monotonic or wall clock), as provided
    t_rel: float           # relative time since session start
    motor_mask: int
    direction_mask: int
    speed_hz: int
    duration_ms: int


@dataclass
class Segment:
    """
    A piecewise-constant velocity segment for one axis (X or Y).
    Position is in steps.
    """
    t0: float              # segment start time (relative)
    t1: float              # segment end time (relative)
    v: float               # velocity (steps/s), signed
    p0: float              # position at t0 (steps)

    def pos_at(self, t: float) -> float:
        """Position at time t within/around this segment (clamped)."""
        if t <= self.t0:
            return self.p0
        if t >= self.t1:
            return self.p0 + self.v * (self.t1 - self.t0)
        return self.p0 + self.v * (t - self.t0)


# ============================================================
# Engine
# ============================================================

class RailPoseEngine:
    """
    Continuous-time rail pose engine driven by 0xBF commands.

    Coordinate convention (Rail frame):
      origin: both motors pressed to stops -> (0,0)
      +x: right
      +y: down

    Motor mapping:
      motor0 (mask bit0) -> X axis
      motor1 (mask bit1) -> Y axis

    Direction mapping (configurable):
      direction bit = 0 -> +axis
      direction bit = 1 -> -axis
    """

    BF_STRUCT = struct.Struct("<BBBii")  # header, motorMask, directionMask, speedHz, durationMs

    def __init__(
        self,
        *,
        step_x_m: float = 0.5 / 320000.0,
        step_y_m: float = 0.5 / 320000.0,
        z_m: float = 0.0,
        # direction bit -> sign mapping
        dir_bit0_sign: int = +1,   # directionBit==0
        dir_bit1_sign: int = -1,   # directionBit==1
    ):
        self.step_x_m = float(step_x_m)
        self.step_y_m = float(step_y_m)
        self.z_m = float(z_m)

        self.dir_bit0_sign = int(dir_bit0_sign)
        self.dir_bit1_sign = int(dir_bit1_sign)

        # session time base
        self._started = False
        self._t0_abs: Optional[float] = None

        # preset / initial position in steps
        self._x0_step = 0.0
        self._y0_step = 0.0

        # segment lists (time-ordered, non-overlapping by construction)
        self._x_segments: List[Segment] = []
        self._y_segments: List[Segment] = []

        # raw parsed commands log (optional but useful for debug/export)
        self._cmd_log: List[BFCommand] = []

    # --------------------------
    # Session control
    # --------------------------

    def start(self, t_start_abs: Optional[float] = None) -> None:
        """
        Start a rail session. If t_start_abs is None, it will be set by the first feed().
        """
        self._started = True
        if t_start_abs is not None:
            self._t0_abs = float(t_start_abs)

    def is_started(self) -> bool:
        return self._started

    def session_start_abs(self) -> Optional[float]:
        return self._t0_abs

    def set_preset_steps(self, x0_step: float, y0_step: float) -> None:
        """
        Set initial offset/preset in steps. This does NOT create motion.
        Should be called before feeding commands (recommended).
        """
        self._x0_step = float(x0_step)
        self._y0_step = float(y0_step)

    # --------------------------
    # Parsing / direction
    # --------------------------

    def _ensure_time_base(self, t_send_abs: float) -> None:
        if not self._started:
            # auto-start on first feed if user didn't call start()
            self._started = True
        if self._t0_abs is None:
            self._t0_abs = float(t_send_abs)

    def _to_rel_time(self, t_send_abs: float) -> float:
        self._ensure_time_base(t_send_abs)
        return float(t_send_abs) - float(self._t0_abs)

    def _parse_bf(self, raw_packet: bytes, t_send_abs: float) -> Optional[BFCommand]:
        if raw_packet is None:
            return None
        if len(raw_packet) != self.BF_STRUCT.size:
            return None
        header, motor_mask, direction_mask, speed_hz, duration_ms = self.BF_STRUCT.unpack(raw_packet)
        if header != 0xBF:
            return None
        if speed_hz < 0 or duration_ms < 0:
            return None

        t_rel = self._to_rel_time(t_send_abs)
        return BFCommand(
            t_send_abs=float(t_send_abs),
            t_rel=float(t_rel),
            motor_mask=int(motor_mask),
            direction_mask=int(direction_mask),
            speed_hz=int(speed_hz),
            duration_ms=int(duration_ms),
        )

    def _dir_sign(self, direction_mask: int, motor_idx: int) -> int:
        bit = (direction_mask >> motor_idx) & 0x1
        return self.dir_bit0_sign if bit == 0 else self.dir_bit1_sign

    # --------------------------
    # Core: overwrite segment logic
    # --------------------------

    @staticmethod
    def _pos_at_with_segments(segments: List[Segment], t: float, p_init: float) -> float:
        """
        Compute position for one axis at time t from the piecewise segments.
        segments are time-ordered, non-overlapping.
        """
        if not segments:
            return p_init

        # Fast path: if t is before first segment
        if t <= segments[0].t0:
            return p_init

        # Walk segments (segments are usually not huge; keep simple & robust)
        p = p_init
        for seg in segments:
            if t < seg.t0:
                break
            if t <= seg.t1:
                return seg.pos_at(t)
            # t beyond this segment, accumulate fully
            p = seg.pos_at(seg.t1)
        return p

    def _truncate_last_if_overlaps(self, segments: List[Segment], t_new: float) -> None:
        """
        Overwrite semantics: if the last segment extends beyond t_new, truncate it at t_new.
        """
        if not segments:
            return
        last = segments[-1]
        if t_new <= last.t0:
            # New command earlier than last start: for simplicity, disallow / ignore by policy.
            # (If you need out-of-order feeds, we can extend later.)
            # Here we just truncate last to zero-length to avoid negative durations.
            last.t1 = last.t0
            return
        if t_new < last.t1:
            last.t1 = t_new  # immediate cut

    def _append_segment(self, segments: List[Segment], t0: float, t1: float, v: float, p0_at_t0: float) -> None:
        """
        Append a new segment, assuming t1 >= t0.
        Zero-length segments are ignored.
        """
        if t1 <= t0:
            return
        segments.append(Segment(t0=float(t0), t1=float(t1), v=float(v), p0=float(p0_at_t0)))

    # --------------------------
    # Public input API
    # --------------------------

    def feed(self, t_send_abs: float, raw_packet: bytes) -> Optional[BFCommand]:
        """
        Feed one (t_send_abs, raw_packet) event.

        Returns parsed BFCommand if it's a valid 0xBF packet, else None.
        """
        cmd = self._parse_bf(raw_packet, t_send_abs)
        if cmd is None:
            return None

        self._cmd_log.append(cmd)

        t0 = cmd.t_rel
        dt = cmd.duration_ms / 1000.0
        t1 = t0 + dt

        # Apply to motor0 -> X axis
        if cmd.motor_mask & 0b00000001:
            sgn = self._dir_sign(cmd.direction_mask, 0)
            v = sgn * float(cmd.speed_hz)

            # overwrite: truncate previous X segment at t0
            self._truncate_last_if_overlaps(self._x_segments, t0)

            # compute x(t0) as p0 for new segment
            x_t0 = self.pose_x_at(t0)
            self._append_segment(self._x_segments, t0, t1, v, x_t0)

        # Apply to motor1 -> Y axis
        if cmd.motor_mask & 0b00000010:
            sgn = self._dir_sign(cmd.direction_mask, 1)
            v = sgn * float(cmd.speed_hz)

            # overwrite: truncate previous Y segment at t0
            self._truncate_last_if_overlaps(self._y_segments, t0)

            # compute y(t0) as p0 for new segment
            y_t0 = self.pose_y_at(t0)
            self._append_segment(self._y_segments, t0, t1, v, y_t0)

        return cmd

    # --------------------------
    # Query API
    # --------------------------

    def pose_x_at(self, t_rel: float) -> float:
        """X position in steps at relative time t_rel."""
        return self._pos_at_with_segments(self._x_segments, float(t_rel), self._x0_step)

    def pose_y_at(self, t_rel: float) -> float:
        """Y position in steps at relative time t_rel."""
        return self._pos_at_with_segments(self._y_segments, float(t_rel), self._y0_step)

    def pose_at(self, t_rel: float) -> Tuple[float, float]:
        """(x_step, y_step) at relative time t_rel."""
        return self.pose_x_at(t_rel), self.pose_y_at(t_rel)

    def sample(self, times_rel: Iterable[float]) -> List[Tuple[float, float]]:
        """Batch sample (x_step, y_step) for a list/iterable of relative times."""
        return [self.pose_at(float(t)) for t in times_rel]

    # --------------------------
    # Export API
    # --------------------------

    def export_quat_csv(
        self,
        out_path: Path,
        *,
        # sampling strategy:
        # - If times_rel is provided, export one row per time.
        # - Else export at command boundaries (t0 and t1 of each command) (useful for debug).
        times_rel: Optional[Iterable[float]] = None,
    ) -> None:
        """
        Export a standard pose CSV for workflow / BP.

        Columns:
          t, x_step, y_step, tx_m, ty_m, tz_m, qx,qy,qz,qw
        """
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Quaternion fixed: no rotation in rail-only model
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        if times_rel is None:
            # default debug sampling at all command start/end times (deduplicated, sorted)
            ts = set()
            for c in self._cmd_log:
                t0 = c.t_rel
                t1 = c.t_rel + c.duration_ms / 1000.0
                ts.add(round(t0, 6))
                ts.add(round(t1, 6))
            times = sorted(ts)
        else:
            times = [float(t) for t in times_rel]

        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "x_step", "y_step", "tx_m", "ty_m", "tz_m", "qx", "qy", "qz", "qw"])
            for t in times:
                x_step, y_step = self.pose_at(t)
                tx_m = x_step * self.step_x_m
                ty_m = y_step * self.step_y_m
                w.writerow([t, x_step, y_step, tx_m, ty_m, self.z_m, qx, qy, qz, qw])

    def export_ffmpeg_debug_fake_log(self, out_path: Path) -> None:
        """
        Export a minimal ffmpeg_debug_fake.log to adapt existing workflow.

        Semantics:
          - Log includes the absolute session start time (t0_abs)
          - Declares a time base starting from 0.0
        """
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        t0 = self._t0_abs
        if t0 is None:
            # If nothing fed yet, still export a stub
            t0 = 0.0

        lines = []
        lines.append("[ffmpeg_debug_fake] rail session time base\n")
        lines.append(f"start:{t0:.6f}\n")
        lines.append("time_origin_rel=0.0\n")
        lines.append("note=this is a fake log to align rail pose timeline with camera workflow\n")

        out_path.write_text("".join(lines), encoding="utf-8")

    # --------------------------
    # Debug helpers
    # --------------------------

    def dump_segments(self) -> Dict[str, List[Tuple[float, float, float, float]]]:
        """
        Return segments for debug:
          {"x": [(t0,t1,v,p0), ...], "y": [...]}
        """
        return {
            "x": [(s.t0, s.t1, s.v, s.p0) for s in self._x_segments],
            "y": [(s.t0, s.t1, s.v, s.p0) for s in self._y_segments],
        }

    def command_log(self) -> List[BFCommand]:
        """Return a copy of parsed BF command log."""
        return list(self._cmd_log)

    # --------------------------
    # Adaptive sampling (non-uniform rail frames)
    # --------------------------

    def _build_sampling_times_adaptive(
        self,
        *,
        min_dt: float,
        max_dt: float,
        t_start: float = 0.0,
        t_end: float | None = None,
    ) -> List[float]:
        """
        Build adaptive sampling times for rail frames.

        Guarantees:
        - All command boundaries are included
        - Adjacent samples satisfy: min_dt <= Δt <= max_dt  (after filtering)
        - Long stationary periods are periodically sampled by max_dt stepping
        """
        if min_dt <= 0 or max_dt <= 0:
            raise ValueError("min_dt and max_dt must be positive")
        if min_dt > max_dt:
            raise ValueError("min_dt must be <= max_dt")

        # Infer t_end from command log if not provided
        if t_end is None:
            t_end = 0.0
            for c in self._cmd_log:
                t_end = max(t_end, c.t_rel + c.duration_ms / 1000.0)

        if t_end < t_start:
            t_end = t_start

        times: set[float] = set()

        # 1) Always include start & end
        times.add(float(t_start))
        times.add(float(t_end))

        # 2) Include all command boundaries
        for c in self._cmd_log:
            times.add(float(c.t_rel))
            times.add(float(c.t_rel + c.duration_ms / 1000.0))

        # 3) Enforce max_dt by global stepping
        t = float(t_start)
        while t < float(t_end):
            times.add(t)
            t += float(max_dt)

        # 4) Sort and enforce min_dt (simple forward filter)
        times_sorted = sorted(times)
        if not times_sorted:
            return [float(t_start)]

        filtered = [times_sorted[0]]
        for tt in times_sorted[1:]:
            if tt - filtered[-1] >= float(min_dt):
                filtered.append(tt)

        # Ensure last point is exactly t_end (helpful for downstream)
        if filtered[-1] != float(t_end):
            if float(t_end) - filtered[-1] >= float(min_dt):
                filtered.append(float(t_end))
            else:
                filtered[-1] = float(t_end)

        return filtered

    # --------------------------
    # Trajectory export (uniform dt)
    # --------------------------

    def export_trajectory_csv(
        self,
        out_path,
        *,
        dt: float = 0.001,
        t_start: float = 0.0,
        t_end: float | None = None,
    ) -> None:
        """
        Export continuous rail trajectory at fixed time step.

        CSV columns:
          t, x_step,y_step, x_m,y_m, vx_step_s,vy_step_s, vx_m_s,vy_m_s
        """
        from pathlib import Path
        import csv
        import math

        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Infer end time from commands if not provided
        if t_end is None:
            t_end = 0.0
            for c in self._cmd_log:
                t_end = max(t_end, c.t_rel + c.duration_ms / 1000.0)

        if dt <= 0:
            raise ValueError("dt must be positive")

        if t_end < t_start:
            t_end = t_start

        n = int(math.floor((t_end - t_start) / dt)) + 1

        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "t",
                "x_step", "y_step",
                "x_m", "y_m",
                "vx_step_s", "vy_step_s",
                "vx_m_s", "vy_m_s",
            ])

            prev_x, prev_y = self.pose_at(t_start)

            for i in range(n):
                t = t_start + i * dt
                x, y = self.pose_at(t)

                vx = (x - prev_x) / dt if i > 0 else 0.0
                vy = (y - prev_y) / dt if i > 0 else 0.0

                w.writerow([
                    f"{t:.6f}",
                    f"{x:.3f}", f"{y:.3f}",
                    f"{x * self.step_x_m:.6f}", f"{y * self.step_y_m:.6f}",
                    f"{vx:.3f}", f"{vy:.3f}",
                    f"{vx * self.step_x_m:.6f}", f"{vy * self.step_y_m:.6f}",
                ])

                prev_x, prev_y = x, y

    # --------------------------
    # Workflow-compatible pose export (frame,time_abs,x,y,z,qx,qy,qz,qw)
    # --------------------------

    def export_pose_csv_workflow(
        self,
        out_path,
        *,
        times_rel: Iterable[float] | None = None,
        min_dt: float | None = None,
        max_dt: float | None = None,
        t_start: float = 0.0,
        t_end: float | None = None,
    ) -> None:
        """
        Export pose CSV compatible with existing video workflow:

          frame,time_abs,x,y,z,qx,qy,qz,qw

        Notes:
        - time_abs here is actually time since session start (t_rel), starting from 0.0,
          kept as 'time_abs' only for pipeline compatibility.
        - If times_rel is provided: export exactly those times (frame = index in provided order).
        - Else if min_dt/max_dt are provided: export adaptive non-uniform samples.
        - Else: export at BF command boundaries.
        """
        from pathlib import Path
        import csv

        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # determine times
        if times_rel is not None:
            times = [float(t) for t in times_rel]
        elif (min_dt is not None) and (max_dt is not None):
            times = self._build_sampling_times_adaptive(
                min_dt=float(min_dt),
                max_dt=float(max_dt),
                t_start=float(t_start),
                t_end=t_end,
            )
        else:
            ts = set()
            for c in self._cmd_log:
                ts.add(float(c.t_rel))
                ts.add(float(c.t_rel + c.duration_ms / 1000.0))
            times = sorted(ts) if ts else [0.0]

        # rail-only model: fixed quaternion
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["frame", "time_abs", "x", "y", "z", "qx", "qy", "qz", "qw"])

            for frame, t in enumerate(times):
                x_step, y_step = self.pose_at(t)
                x = x_step * self.step_x_m
                y = y_step * self.step_y_m
                z = self.z_m

                w.writerow([
                    frame,
                    f"{t:.6f}",
                    f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                    f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}", f"{qw:.6f}",
                ])

    # Backward-compatible wrapper (optional): keep your existing adaptive name
    def export_quat_csv_adaptive(
        self,
        out_path,
        *,
        min_dt: float,
        max_dt: float,
        t_start: float = 0.0,
        t_end: float | None = None,
    ) -> None:
        """
        Backward compatible wrapper.
        """
        self.export_pose_csv_workflow(
            out_path,
            min_dt=min_dt,
            max_dt=max_dt,
            t_start=t_start,
            t_end=t_end,
        )
