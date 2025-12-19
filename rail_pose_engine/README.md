# rail_pose_engine

Command-driven continuous rail odometry engine for mmWave SAR / BP imaging

---

## 1. Overview

`rail_pose_engine.py` is a fully independent rail pose reconstruction module designed for mmWave SAR / BP imaging systems.

Its purpose is to reconstruct a **continuous 2D rail trajectory** from:

- serial **0xBF motion commands**
- and their **command send timestamps**

and export rail poses in a format compatible with:

- radar processing pipelines
- camera / ffmpeg-based workflows
- SAR Back-Projection (BP) imaging

This module is intentionally **decoupled** from:

- radarControl
- mmWaveStudio
- serial I/O
- radar data

It relies on only one fact:

> At what time, which 0xBF command was sent to the rail.

---

## 2. Design Goals

- Fully independent from radar or capture control code
- Supports parallel rail motion and radar acquisition
- Independent X / Y axis modeling
- Overwrite semantics for same-axis commands
- Continuous-time trajectory model
- Arbitrary-time pose sampling
- Compatible with camera / ffmpeg / radar workflows

---

## 3. Coordinate System

### 3.1 Rail Coordinate Frame

The rail coordinate system is defined as:

- origin (0,0): both motors pressed against mechanical stops  
- +x: right  
- +y: down  

This is an **engineering / image-friendly coordinate system**, not a mathematical right-handed system.

### 3.2 Pose Model

- Translation only (no rotation)
- Quaternion is always identity: (0, 0, 0, 1)
- Z coordinate is constant (fixed radar height)

---

## 4. Time Base

- t = 0.0 is defined as the **rail session start**
- All internal times are **relative times**
- Designed to align with camera / ffmpeg / radar timelines

Time source can be:

- `time.monotonic()` (recommended)
- `time.time()` (acceptable if used consistently)

---

## 5. Input Model

### 5.1 RailCommandEvent (Implicit)

The **only input** to `rail_pose_engine` is:

- `t_send_abs`: absolute command send time
- `raw_packet`: raw serial packet (bytes)

Supported packet format:

- `0xBF | motorMask | directionMask | speedHz | durationMs`

---

## 6. Motion Model

### 6.1 Axis Mapping

- motor0 → X axis
- motor1 → Y axis

### 6.2 Overwrite Semantics

For the same motor:

- At any moment, only **one active command** exists
- A new command **immediately terminates** the previous one
- Motion continues with the new parameters

### 6.3 Continuous Trajectory

Each axis is modeled as piecewise constant velocity integrated over time, resulting in continuous functions:

- x(t), y(t)

---

## 7. Minimal Example

```python
from rail_pose_engine import RailPoseEngine
import time

engine = RailPoseEngine()
engine.set_preset_steps(0, 0)
engine.start(time.monotonic())

t_cmd = time.monotonic()
engine.feed(t_cmd, packet_L_20cm)

t_rel = time.monotonic() - engine.session_start_abs()
x_step, y_step = engine.pose_at(t_rel)

print("Rail position:", x_step, y_step)

engine.export_quat_csv("./PostProc/quat_rail.csv")
engine.export_ffmpeg_debug_fake_log("./PostProc/ffmpeg_debug_fake.log")
```

---

## 8. Public API Reference

### 8.1 Constructor

```python
RailPoseEngine(
    step_x_m=0.5/320000,
    step_y_m=0.5/320000,
    z_m=0.0,
    dir_bit0_sign=+1,
    dir_bit1_sign=-1
)
```

Parameters:

- `step_x_m`, `step_y_m`: physical distance per step (meters)
- `z_m`: fixed Z height for exported poses
- `dir_bit0_sign`, `dir_bit1_sign`: direction bit to axis sign mapping  
  - bit = 0 → dir_bit0_sign  
  - bit = 1 → dir_bit1_sign  

---

### 8.2 start()

```python
start(t_start_abs=None)
```

Starts a rail session. If `t_start_abs` is None, session starts automatically on first `feed()`.

---

### 8.3 set_preset_steps()

```python
set_preset_steps(x0_step, y0_step)
```

Sets initial coordinate offset (in steps). Does not generate motion.

---

### 8.4 feed()

```python
feed(t_send_abs, raw_packet) -> BFCommand | None
```

Feeds a serial event. Returns None if packet is not valid 0xBF. Otherwise updates X/Y motion segments and records the command.

---

### 8.5 pose_at()

```python
pose_at(t_rel) -> (x_step, y_step)
```

Returns rail position at relative time `t_rel` (steps).

---

### 8.6 sample()

```python
sample(times_rel) -> List[(x_step, y_step)]
```

Batch sampling, typically at radar frame timestamps for BP.

---

### 8.7 export_quat_csv()

```python
export_quat_csv(path, times_rel=None)
```

Exports pose CSV. If `times_rel` is None, samples at command boundaries (debug). If provided, samples at given times (recommended for BP).

---

### 8.8 export_ffmpeg_debug_fake_log()

```python
export_ffmpeg_debug_fake_log(path)
```

Generates a minimal fake ffmpeg debug log to align rail timeline with camera workflow.

---

## 9. Typical Usage Patterns

### 9.1 radarControl + rail_pose_engine

- radarControl handles acquisition and triggering
- rail_pose_engine handles rail pose reconstruction
- Connected only via `feed()`

### 9.2 Manual / Random Rail Control

- Manual controller sends BF commands
- Feed all BF events into rail_pose_engine
- Reconstruct poses offline and run BP using real trajectory

### 9.3 Offline Replay / Debug

- Log `(t_send_abs, raw_packet)` during experiments
- Replay logs into rail_pose_engine to reconstruct identical trajectories

---

## 10. Common Pitfalls

1. Direction mirroring: adjust `dir_bit0_sign` / `dir_bit1_sign`
2. Inconsistent time sources: use one of `monotonic()` or `time()`
3. Treating presets as motion: presets are offsets only
4. Using TriggerFrame as pose: sample from continuous trajectory instead

---

## 11. Philosophy

Trajectory is a physical fact, independent of radar or algorithms. Imaging is an interpretation of that fact.

This module exists to ensure you know where the radar actually was, not where you assumed it was.
