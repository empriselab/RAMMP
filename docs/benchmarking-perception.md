# Benchmarking Perception Performance

This guide explains how to measure the runtime of RAMMP's two perception
pipelines — **head perception** and **cup (drink) detection** — and how to
produce a before/after comparison for the performance work done on the
`feature/improve-performance` branch.

> These instructions target an **x86** machine (current deployment). The
> pipeline is intended to move to a Jetson (ARM) later; absolute numbers will
> differ there, but the method below is the same.

---

## 1. What is instrumented

`src/rammp/utils/timing.py` provides a lightweight timer. Hot sections of the
perception code are wrapped in `with timer("<section>"):` blocks, which
accumulate, per section: call count, mean / min / max milliseconds, and total
time.

A summary table is printed:

- automatically when the process exits (via `atexit`),
- whenever cup-detection streaming is stopped, and
- on demand by calling `rammp.utils.timing.print_summary()`.

**Sections measured on the current branch (`feature/improve-performance`):**

| Pipeline | Sections |
|---|---|
| Head | `head/run_head_perception_total`, `head/mediapipe_detect` |
| Cup  | `drink/run_perception_total`, `drink/color_mask`, `drink/clean_mask`, `drink/cluster`, `drink/backproject`, `drink/ransac_plane` |

---

## 2. Reading the summary table

```
==============================================================================
PERCEPTION TIMING SUMMARY
------------------------------------------------------------------------------
section                              n    mean ms    min ms    max ms   total s  ~max Hz
------------------------------------------------------------------------------
drink/run_perception_total         300       4.10      3.6      9.2       1.23     243.9
drink/ransac_plane                 300       2.55      ...
...
==============================================================================
```

- **n** — how many times that section ran.
- **mean / min / max ms** — wall-clock time spent inside the section.
- **total s** — cumulative time across all calls.
- **~max Hz** — `1000 / mean ms`; the ceiling rate if that section were the
  only cost. Sanity-check value, not a measured throughput.

The `*_total` rows are the end-to-end cost of one perception call; the other
rows break that down by stage.

---

## 3. Prerequisites for a real benchmark

The perception code only executes — and therefore the timers only fire — when:

1. **A RealSense camera is running** and publishing the wrist camera topics
   (`/camera/wrist/color/image_raw`, `/camera/wrist/aligned_depth_to_color/image_raw`,
   `/camera/wrist/color/camera_info`). Perception consumes live frames.
2. **The action server runs with `--run_on_robot`.** Without that flag the
   node starts in *simulation* mode and replays cached `.pkl` data — the real
   perception code never runs and no timing is recorded.

   Note: `launch/real.launch.py` currently starts `drink_action_server.py`
   with only `--scene_config`. For benchmarking, either add `--run_on_robot`
   to that node's `arguments` list, or run the server directly (see below).
3. The Python environment is set up per `README.md`.

---

## 4. Running a benchmark (current branch)

With the RealSense camera already publishing:

```bash
# 1. Start the action server on the robot (real perception enabled).
ros2 run drink_actions_test drink_action_server.py --run_on_robot --scene_config wheelchair
```

In a second terminal, exercise the pipelines with the action client:

```bash
# Cup detection: start streaming, let it run ~20-30 s, then stop.
ros2 run drink_actions_test drink_action_client.py perceive_cup true
#   ... wait 20-30 seconds ...
ros2 run drink_actions_test drink_action_client.py perceive_cup false
#   -> the PERCEPTION TIMING SUMMARY prints when streaming stops.

# Head perception: trigger bring_cup_to_mouth, which runs head perception
# in a loop while waiting for the mouth-open gesture.
ros2 run drink_actions_test drink_action_client.py bring_cup_to_mouth
```

To get the full table at any time, stop the server with `Ctrl+C` — the
summary prints on exit.

**Tips**

- Let each pipeline run for a few hundred frames so the mean is stable; ignore
  the first 1-2 samples (model/CUDA warm-up inflates them — `min ms` is a
  better "warm" figure).
- The numbers are CPU/GPU dependent — always record what machine produced them.

---

## 5. Before / after comparison

The performance work is bracketed by two commits on `feature/improve-performance`:

- **"after"** = branch tip (`feature/improve-performance`) — MediaPipe head
  perception + optimized cup detection.
- **"before"** = commit **`9cd9042`** ("Add perception timing instrumentation")
  — the timing harness is present, but the original DECA head perception and
  the unoptimized cup detection are still in place.

Procedure:

```bash
# 1. Benchmark "after" (section 4 above). Save the table.

# 2. Switch to the "before" state.
git checkout 9cd9042

# 3. Benchmark again. Save the table.  (See caveats below re: environment.)

# 4. Return to the branch.
git checkout feature/improve-performance
```

**Section name mapping** (the section names differ between the two states):

| Stage | before (`9cd9042`) | after (branch tip) |
|---|---|---|
| Head, end-to-end | `head/run_head_perception_total` | `head/run_head_perception_total` |
| Head, detection | `head/face_detect` + `head/deca_encode` + `head/deca_decode` | `head/mediapipe_detect` |
| Cup, end-to-end | `drink/run_perception_total` | `drink/run_perception_total` |
| Cup, clustering | `drink/dbscan` | `drink/cluster` |
| Cup, back-projection | `drink/backproject` (per-pixel Python loop) | `drink/backproject` (vectorized) |
| Cup, debug image writes | `drink/debug_imwrite` | *(removed — debug writes off by default)* |

Compare `*_total` rows for the headline number, and the per-stage rows to see
where the time went.

### Caveats

- **Head "before" needs the DECA environment.** Commit `9cd9042` uses the
  DECA/FLAME stack, which requires PyTorch, PyTorch3D, and the FLAME model
  files (see that commit's `README.md`). This is a heavy install.
- **Apples-to-apples head comparison needs calibration on both sides.** The
  "before" state has the old DECA calibration committed and works immediately;
  the "after" state needs a one-time MediaPipe calibration recorded on the
  robot first (`python -m rammp.perception.head_perception.calibrate_head
  --tool drink`).
- **Cup detection before/after is the clean, easy comparison** — it needs no
  special environment or calibration on either side. The debug-image-write
  removal alone (`drink/debug_imwrite`, ~18 ms/frame in a local probe) is a
  large, easily demonstrated win. If the head comparison is too much setup,
  the cup comparison stands on its own.
- **`num_samples` changed (3 → 1).** `PerceptionInterface.perceive_cup_info`
  now runs `run_perception` once per call instead of three times. So compare
  both the per-call `drink/run_perception_total` *and* the number of times it
  runs per `perceive_cup_info` — the end-to-end win is larger than the
  per-`run_perception` row alone shows.

---

## 6. Notes

- A no-hardware micro-benchmark is not representative: the committed
  `src/rammp/perception/drink_perception/{rgb,depth}.png` fixture's cup color
  does not match the current HSV threshold, so `run_perception` exits early
  and never reaches clustering/RANSAC. A real RealSense feed is required for a
  meaningful number.
- To disable instrumentation entirely (e.g. to measure its overhead, or for
  production), call `rammp.utils.timing.enable(False)` early at startup. The
  overhead is small but non-zero.
- `rammp.utils.timing.reset()` clears all accumulated stats — useful to
  separate a warm-up phase from the measured phase within one process.
