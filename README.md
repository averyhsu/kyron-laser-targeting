# Laser Weeding — Targeting Pipeline Architecture

Real-time perception-to-actuation pipeline for a galvo-steered laser weeder. The
core design principle: **the system only ever measures deltas between two
bookmarked moments — it never needs to know where it is in the world.** No map
frame, no odometry in the fire path, no global correctness required.

---

## Hardware

| Role | Part | Notes |
|---|---|---|
| Detection + tracking camera | Allied Vision Alvium 1800 U-240c (IMX392, global shutter) | USB3, hardware trigger + strobe |
| Compute (vision) | Jetson Orin Nano | Runs YOLO-pose + KLT |
| Real-time controller | Teensy 4.1 | Owns trigger, gyro, galvo, interlocks |
| IMU | 1 kHz gyro (SPI) | Rigidly on the optical bench |
| Actuator | Galvo-steered laser | Analog ±10 V per axis via external SPI DAC |

**The optical bench is the load-bearing mechanical decision.** Camera, galvo,
laser, IMU, and LEDs all mount to one rigid plate. Because they share a body, any
chassis jolt rotates all of them together — the pixel-to-galvo-angle mapping is
invariant to chassis motion, so terrain, slip, and bounce cancel as common mode.
Everything else (Jetson, Teensy, drive motors, encoders) mounts to the chassis and
is allowed to move.

---

## Design principles

1. **Deltas, not absolutes.** Every quantity is a difference between two bookmarked
   moments — pixel shift between two frames, rotation between two trigger
   timestamps. Nothing relies on a globally correct position, so unbounded drift
   never matters.
2. **The past is measured; only the last ~15 ms is predicted.** Inference latency is
   hidden by replaying motion that was already observed during the inference window,
   not by predicting across it.
3. **Latency doesn't cost accuracy, only lookahead.** Frames are stamped with
   *exposure time*, not arrival time, so a slow USB transfer or a slow model shifts
   *how far ahead* a weed must still be in range — not whether targeting is correct.
4. **Jetson decides what and where; Teensy decides when.** The fire path never
   routes through Linux.
5. **Rotation gets a live sensor; translation gets a prediction.** Rotation is fast
   and violent (ruts, bounce) so it is measured at 1 kHz. Translation is slow and
   smooth (heavy chassis) so a short linear extrapolation is accurate to a fraction
   of a millimeter.

---

## The two loops

### Slow loop — detection (10 Hz)

Every 10th frame is analyzed by the YOLO-pose detector to locate weeds and their
meristem keypoints. Inference takes ~30–50 ms. Its output is always several frames
stale by the time it lands — which is fine, because the fast loop bridges the gap.

### Fast loop — tracking (100 Hz KLT + 1 kHz gyro)

- **KLT (Jetson, 100 Hz):** each frame is compared to the previous one on arrival;
  the pixel displacement is appended to a ring buffer. Translation, measured.
- **Gyro (Teensy, 1 kHz):** every millisecond the gyro is read and integrated into a
  running attitude total, bookmarked against each frame's trigger timestamp.
  Rotation, measured.

Both are the same ring-buffer pattern — accumulate continuously, look up at fire
time. Neither computes anything in the latency path.

---

## Sequence of operation

**Continuous background rhythm**

1. **Every 10 ms** the Teensy pulses the camera trigger and records
   `(frame_seq, t_exposure)`. The image streams to the Jetson over USB (~15 ms
   later). The delay is harmless because the exposure time is known.
2. **On every frame arrival** the Jetson runs KLT vs. the previous frame and appends
   the pixel shift to its ring buffer.
3. **Every 1 ms** the Teensy reads the gyro, adds to its running rotation total, and
   bookmarks that total against the current frame number at each trigger.

**When a weed is present**

4. **Every 10th frame** the Jetson also runs YOLO-pose. Call the analyzed frame
   **frame X**.
5. **On detector completion** it reports the meristem pixel `(u,v)` *in frame X*
   (now several frames old). The Jetson fast-forwards: sum the stored pixel shifts
   from frame X to the newest frame → weed position in the newest frame. Current
   speed comes from the last few shifts.
6. **Jetson → Teensy:** `{weed_id, pixel_pos, pixel_vel, seq}` (call it **frame Y**),
   refreshed every frame.

**The kill**

7. **Every 1 ms** the Teensy computes the weed's position *now*:

   ```
   aim = pixel_pos
       + pixel_vel * (t_now - t_exposure[Y])          # predicted translation
       + gyro_total(now) - gyro_total(t_exposure[Y])  # measured rotation, lookup
   ```

   Convert pixel → galvo angle via the calibration LUT → DAC → mirrors.
8. **When confident, fire** — and keep running step 7 at 1 kHz *during* the burn so
   the beam tracks the weed through the full 10–100 ms dwell. Frames arriving
   mid-burn keep correcting the track.
9. **Safety underneath everything:** if the Jetson goes quiet, tracking quality
   drops, or the containment skirt lifts, the Teensy kills the beam that same
   millisecond. This path never touches Linux.

---

## Timeline: measured vs. predicted

```
   frame X ───── stored KLT shifts (MEASURED) ───── frame Y ── predicted ──▶ now: FIRE
   (detector      real observations, no guessing    (newest    speed+gyro
    starts)                                           frame)    ~15 ms hop
```

- **X → Y:** covered by chained, measured pixel shifts. Bumps and slips between X and
  Y are directly observed because the weed physically moved in the frames.
- **Y → now:** the only predicted segment, always just the newest ~15 ms.

---

## Error budget

| Source | Magnitude | Notes |
|---|---|---|
| Predicted translation (Y→now) | ~0.1 mm | `½·a·Δt²`, a≈1 m/s², Δt≈15 ms. Heavy chassis → velocity barely changes. |
| Rotation over Δt | ~0 | Measured live at 1 kHz, not predicted. |
| Gyro bias drift | microdegrees / engagement | ~0.01°/s over seconds — never needs re-anchoring within an engagement. |
| KLT chain drift (X→Y) | sub-pixel over 10 frames | Re-anchored by every YOLO detection. |
| Parallax (camera↔galvo baseline) | 1.2 mm @ 20 mm baseline, 30 mm Δheight, 500 mm standoff | Shrink baseline + ground-riding skirt to reduce. |

Spot size is ~2–5 mm, so the predicted-motion error is ~30× inside tolerance.

---

## Why no re-anchoring is needed

The gyro integral drifts, but the system never uses the *total* rotation — only the
*difference* between two bookmarked timestamps, spanning at most the seconds a weed
is in play. Delta correctness, not global correctness. The same logic is why the
whole pipeline avoids a world frame entirely.

---

## Open items

- **Camera bandwidth:** IMX392 mono8 full-frame at 100 Hz ≈ 235 MB/s — near the USB3
  practical ceiling. Use on-sensor ROI/AOI readout (e.g. 1936×400 ≈ 77 MB/s) to
  create headroom. Subsample *inference*, not *acquisition* — every frame still
  arrives; only YOLO runs at 1-in-10.
- **Parallax correction:** prefer a pilot-beam visual servo (fire a low-power spot,
  measure pixel error, correct, then fire) over a depth sensor — it also absorbs
  calibration drift, thermal shift, and mount flex. Monocular height regression (an
  extra YOLO-pose head) is the zero-hardware fallback.
- **Galvo drive:** Teensy 4.x has no DAC — needs an external SPI DAC + op-amp stage
  to ±10 V, with position feedback read back on an ADC.
- **Containment skirt:** required for Class 4 safety and fire containment; also
  enables LED illumination (1 ms exposure kills motion blur), removes sun washout,
  and stabilizes standoff.
- **KLT robustness:** seed features on segmented leaf structure, reject
  outliers against consensus, and refuse to fire if KLT residual or frames-since-
  detection exceed thresholds.