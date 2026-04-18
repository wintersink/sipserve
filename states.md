# SipServe State Machine

## 1. INITIALIZING

Rotate until the Home AprilTag is found, then center on it.

### Phases

1. **Scanning** — Burst-rotates right (`SMOOTH_TURN_SPEED`) in short pulses (`SEARCH_ROTATE_BURST_S` on, `SEARCH_PAUSE_S` off) until the Home tag enters the camera frame.
2. **Centering** — Pulse-and-pause rotation (`CENTER_TURN_SPEED` for `CENTER_PULSE_S`, then pause `CENTER_PAUSE_S`) to place the Home tag within `CENTER_PX_TOL` (30 px) of frame center. Once centered, transitions to AWAITING_ORDERS.

---

## 2. AWAITING_ORDERS

Stationary. Motors stopped. Waits for voice commands to build a delivery queue. 

### Voice Commands

| ID | Action |
|----|--------|
| 2 | Wake word — acknowledge readiness |
| 5–8 | Queue Table 1–4 (no duplicates) |
| 9 | Clear all queued orders |
| 10 | Start delivery (requires non-empty queue) |

On "start delivery" with a non-empty queue, transitions to SEARCHING.

---

## 3. SEARCHING

Locate the AprilTag for the first queued table.

### Phases

1. **Initial 180° turn** — Blocking smooth rotation to face away from the tag the robot was just parked at (Home or previous delivery). Runs once on entry.
2. **Roam** (`ROAM_DURATION` = 10 s) — Reactive obstacle-avoidance driving:
   - Front or both front-sides blocked → reverse (`ROAM_REVERSE_S`), then turn toward the clearer side (`ROAM_TURN_S`)
   - Front-left only → turn right
   - Front-right only → turn left
   - Side only → forward slow
   - Clear → forward full
3. **Scan** (`SCAN_DEGREES` = 960°) — Burst-rotate right (`BURST_TURN_SPEED` for `BURST_ROTATE_S`, pause `BURST_PAUSE_S`) to sweep for tags.

Roam and Scan alternate until the target tag is detected. Every tick checks for the target tag first — if found, transitions to LOCKED_ON.

---

## 4. LOCKED_ON

Drive toward the target tag. Front ultrasonics guard against obstacles.

### Phases

1. **Centering** — Pulse-and-pause rotation to place the tag within `CENTER_PX_TOL` (30 px) of frame center before starting forward motion. If the tag disappears during centering, nudges forward (`CENTER_NUDGE_IN` = 2") up to `CENTER_NUDGE_MAX` (5) times to try to reacquire it.
2. **Approach** — Drives forward with bang-bang steering (pulse-and-pause steer corrections when the tag drifts past `APPROACH_STEER_PX` = 60 px off center). Movement decisions each tick:
   - Tag way off-axis → steer pulse (no forward motion)
   - Obstacle closer than tag with room to reach arrival distance → cautious creep forward with `APPROACH_SAFETY_IN` (1") clearance
   - Obstacle closer than tag without room → halt
   - Unknown obstacle (no tag visible, front close) → halt
   - Sides close (`SIDE_SLOW_IN` = 16") → forward at `SLOW_SPEED`
   - Clear → forward at `FULL_SPEED`

### Arrival

Declared when stereo depth < `ARRIVAL_DIST_IN` (12"), or when the tag is visible and front ultrasonic < 12". Also triggers if the tag drops out of view while centered and front ultrasonic reads < 12" (tag likely left the camera's vertical FOV at close range).

On arrival → DELIVERED. If tag is absent past `LOST_TIMEOUT_S` (1 s) and nudges exhausted → LOST.

---

## 5. LOST

Target tag disappeared mid-approach. Attempts to relocate it.

### Phases

Uses the same **Roam / Scan** cycle as SEARCHING (without the initial 180° turn). If the target tag reappears, transitions back to LOCKED_ON.

---

## 6. DELIVERED

Paused at the destination for `DELIVERY_PAUSE_S` (10 s). Motors stopped.

### After Pause

- If more tables remain in the queue → pop the completed delivery, transition to SEARCHING for the next table.
- If queue is empty → transition to RETURNING.

---

## 7. RETURNING

Navigate back to the Home tag. Same approach logic as LOCKED_ON.

### Phases

1. **Approach** — Identical to LOCKED_ON (centering then drive-toward) but targeting the Home tag.
2. **Search** — If the Home tag is lost past `LOST_TIMEOUT_S` and nudges are exhausted, flips into the Roam/Scan cycle. When the Home tag reappears, resets centering and resumes approach.

On arrival at Home → AWAITING_ORDERS.
