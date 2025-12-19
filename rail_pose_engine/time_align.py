"""
time_align.py
------------------------------------------------------------
Time alignment layer between external timestamps and rail poses.

Responsibilities:
- Sample rail poses at given relative timestamps
- Export aligned pose tables for radar / camera workflows
"""

from pathlib import Path
import csv
from typing import Iterable, List, Tuple

from rail_pose_engine import RailPoseEngine


def align_times(
    times_rel: Iterable[float],
    engine: RailPoseEngine,
) -> List[Tuple[float, float, float]]:
    """
    Align a list of relative times to rail poses.

    Returns:
        List of (t_rel, x_step, y_step)
    """
    aligned = []
    for t in times_rel:
        x, y = engine.pose_at(t)
        aligned.append((t, x, y))
    return aligned


def export_aligned_csv(
    out_path: str | Path,
    times_rel: Iterable[float],
    engine: RailPoseEngine,
):
    """
    Export aligned rail poses for given times.

    CSV columns:
        t_rel,
        x_step, y_step,
        x_m, y_m,
        z_m,
        qx, qy, qz, qw
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "t_rel",
            "x_step", "y_step",
            "x_m", "y_m",
            "z_m",
            "qx", "qy", "qz", "qw",
        ])

        for t in times_rel:
            x_step, y_step = engine.pose_at(t)
            w.writerow([
                f"{t:.6f}",
                f"{x_step:.3f}", f"{y_step:.3f}",
                f"{x_step * engine.step_x_m:.6f}",
                f"{y_step * engine.step_y_m:.6f}",
                f"{engine.z_m:.6f}",
                qx, qy, qz, qw,
            ])
