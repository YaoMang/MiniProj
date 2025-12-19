"""
rail_pose_replay.py
------------------------------------------------------------
Replay rail motion from recorded rail_events.csv
"""

from pathlib import Path
import csv
from typing import Iterable, Tuple

from rail_pose_engine import RailPoseEngine


def load_rail_events(path: str | Path) -> list[Tuple[float, bytes]]:
    """
    Load rail_events.csv

    Returns:
        List of (t_send_abs, raw_packet)
    """
    events = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["t_send_abs"])
            raw = bytes.fromhex(row["packet_hex"])
            events.append((t, raw))
    return events


def replay_events(
    events: Iterable[Tuple[float, bytes]],
    engine: RailPoseEngine,
):
    """
    Feed recorded events into a RailPoseEngine.
    """
    for t_send_abs, raw_packet in events:
        engine.feed(t_send_abs, raw_packet)
