"""
rail_event_logger.py
------------------------------------------------------------
Minimal event logger for rail 0xBF commands.

Records ONLY facts:
    - when a command was sent
    - what raw bytes were sent
"""

from pathlib import Path
import csv
import time
from typing import BinaryIO


class RailEventLogger:
    def __init__(self, path: str | Path, *, use_monotonic: bool = True):
        self.path = Path(path)
        self.use_monotonic = use_monotonic
        self._file: BinaryIO | None = None
        self._writer: csv.writer | None = None

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "a", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)

        # write header if file is empty
        if self.path.stat().st_size == 0:
            self._writer.writerow(["t_send_abs", "packet_hex"])
            self._file.flush()

    def close(self):
        if self._file:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None

    def log_packet(self, raw_packet: bytes):
        if self._writer is None:
            raise RuntimeError("RailEventLogger is not opened")

        t = time.monotonic() if self.use_monotonic else time.time()
        hex_str = raw_packet.hex()

        self._writer.writerow([f"{t:.9f}", hex_str])
        self._file.flush()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
