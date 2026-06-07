# Signal Controller — Architecture

How the drone remote-control layer is built. This is the blueprint to implement
from. Design goal: **`control.py` is a dumb, reliable connector; the interesting,
swappable behavior lives in small plug-in scripts.**

The whole system follows the project style: **plain functions + a single config
file**, not deep class hierarchies. State is passed in and out explicitly so you
can trace a signal from sensor to motor without chasing hidden object state.

---

## 1. The big picture

```
   ┌────────────┐     state      ┌─────────────────────────────┐    signal     ┌────────────┐
   │  DRONE     │ ─────────────▶ │          control.py         │ ────────────▶ │  DRONE     │
   │  (sensors) │                │      (the connector)        │               │  (motors)  │
   └────────────┘                │                             │               └────────────┘
                                 │   loop @ fixed rate:        │
                                 │   1. read_state()           │
                                 │   2. behavior.update(state) │◀── active behavior
                                 │   3. clamp + failsafe       │      (hover by default)
                                 │   4. send_signal()          │
                                 └─────────────────────────────┘
```

Two data shapes flow through the loop:

- **`DroneState`** — everything the controller knows *right now* (altitude, drift,
  velocity, heading, battery, and optional AI vision data).
- **`ControlSignal`** — what we want the drone to *do* (velocity setpoints + yaw
  rate).

A **behavior** is a function that maps `DroneState -> ControlSignal`. That's the
entire abstraction. `control.py` doesn't care whether that function hovers, follows
a person, or flies a search pattern.

---

## 2. The control loop (`control.py`)

The connector is a fixed-rate loop. Pseudocode:

```python
def main(behavior_name="hover"):
    link = drone_link.connect(config)        # open telemetry + command channel
    behavior = load_behavior(behavior_name)  # default: hover
    behavior_start(behavior, link.read_state(), config)

    try:
        while True:
            tick_start = time.time()

            state  = link.read_state()                 # 1. sense
            signal = run_behavior(behavior, state)     # 2. decide (falls back to hover on error)
            signal = clamp_signal(signal, config)      # 3. make it safe
            signal = apply_failsafes(signal, state, config)
            link.send_signal(signal)                   # 4. act

            sleep_to_keep_rate(tick_start, config.LOOP_HZ)
    except KeyboardInterrupt:
        link.send_signal(SAFE_STOP)   # hover/stop, then disconnect
    finally:
        link.disconnect()
```

Key properties:

- **Fixed rate** (`config.LOOP_HZ`, e.g. 20–50 Hz). Stable control needs a
  predictable tick.
- **The behavior never talks to the drone directly.** It only returns a signal.
  This is what makes behaviors swappable and safe.
- **`run_behavior` is wrapped in a guard** — if the active behavior raises, the
  loop logs it and substitutes a hover signal for that tick.

---

## 3. Data contracts (`signals_control.py`)

These are the shared shapes. Keep them plain (a dataclass or a simple namedtuple).

### DroneState — the inputs

```python
@dataclass
class DroneState:
    # --- flight state ---
    altitude_m: float          # height above ground
    vx: float                  # body-frame velocity, m/s (forward +)
    vy: float                  # body-frame velocity, m/s (right +)
    vz: float                  # vertical velocity, m/s (up +)
    yaw_deg: float             # heading

    # --- position hold ---
    drift_x: float             # metres off the hold point (forward/back)
    drift_y: float             # metres off the hold point (left/right)

    # --- housekeeping ---
    battery: float             # 0.0 - 1.0
    armed: bool
    link_ok: bool              # is the telemetry link healthy?
    dt: float                  # seconds since the last tick

    # --- optional AI vision (None unless a vision source is feeding it) ---
    vision: "VisionTarget | None" = None
```

### VisionTarget — what the AI sees (used by follow-type behaviors)

```python
@dataclass
class VisionTarget:
    found: bool
    offset_x: float            # px the person is off the frame center (+ = right)
    offset_y: float            # px the person is off the frame center (+ = down)
    distance_cm: float | None  # estimated distance to the person
    confidence: float
```

### ControlSignal — the output

```python
@dataclass
class ControlSignal:
    vx: float = 0.0            # desired forward velocity,  m/s
    vy: float = 0.0            # desired sideways velocity, m/s
    vz: float = 0.0            # desired vertical velocity, m/s (+ = climb)
    yaw_rate: float = 0.0      # desired turn rate, deg/s

# The universal safe command: hold still.
SAFE_STOP = ControlSignal(0.0, 0.0, 0.0, 0.0)
```

> Behaviors emit **velocity setpoints**, not raw motor values. The `drone_link`
> layer is responsible for turning setpoints into whatever the autopilot/SDK
> actually wants (MAVLink, Tello SDK, etc.). This keeps behaviors hardware-agnostic.

### PID helper

Drift correction and altitude hold are closed loops, so `signals_control.py` also
holds one tiny reusable PID function/closure:

```python
def make_pid(kp, ki, kd):
    """Return an update(error, dt) -> correction function with its own memory."""
    state = {"integral": 0.0, "prev_error": 0.0}
    def update(error, dt):
        state["integral"] += error * dt
        derivative = (error - state["prev_error"]) / dt if dt > 0 else 0.0
        state["prev_error"] = error
        return kp * error + ki * state["integral"] + kd * derivative
    return update
```

(Closure instead of a class — keeps the memory local and the call site readable.)

---

## 4. The behavior contract (the plug-in API)

A **behavior** is any `.py` file in `behaviors/` that defines an `update` function.
`start` and `stop` are optional lifecycle hooks.

```python
# behaviors/<name>.py
NAME = "<name>"

def start(state, config):     # OPTIONAL — called once when this behavior becomes active
    ...

def update(state, config):    # REQUIRED — called every control tick
    """Map the current DroneState to a ControlSignal."""
    return ControlSignal(...)

def stop(state, config):      # OPTIONAL — called once when switching away
    ...
```

That's the whole interface. If you can write `update(state, config) -> ControlSignal`,
you can add a new drone behavior without touching `control.py`.

---

## 5. Default behavior: `behaviors/hover.py`

Hover is just a behavior that always wants to be at the hold point and at 6 ft.
It uses two PID loops: one on altitude, one on horizontal drift.

```python
NAME = "hover"

# created once via start(); kept module-local (readable, no globals sprayed around)
_alt_pid   = None
_drift_pid_x = None
_drift_pid_y = None

def start(state, config):
    global _alt_pid, _drift_pid_x, _drift_pid_y
    _alt_pid     = make_pid(*config.ALT_PID)
    _drift_pid_x = make_pid(*config.DRIFT_PID)
    _drift_pid_y = make_pid(*config.DRIFT_PID)

def update(state, config):
    # 1. Altitude hold -> vertical velocity
    alt_error = config.HOVER_ALTITUDE_M - state.altitude_m
    vz = _alt_pid(alt_error, state.dt)

    # 2. Drift correction -> horizontal velocity (drive the error to zero)
    vx = _drift_pid_x(-state.drift_x, state.dt)
    vy = _drift_pid_y(-state.drift_y, state.dt)

    # 3. Hold heading
    return ControlSignal(vx=vx, vy=vy, vz=vz, yaw_rate=0.0)
```

- `HOVER_ALTITUDE_M = 1.8288` (6 ft) lives in `config.py`.
- If the drone drifts forward, `drift_x` is positive, the PID commands a backward
  `vx`, and the drone returns to the hold point. Same idea sideways and vertically.
- Gains are tuned in `config.py` — start gentle.

---

## 6. Example behavior: `behaviors/followhuman.py`

Same contract, different job: keep a detected person centered and at a follow
distance. It reads `state.vision` (fed by the AI vision pipeline) instead of the
hold point.

```python
NAME = "followhuman"

_yaw_pid = None
_dist_pid = None

def start(state, config):
    global _yaw_pid, _dist_pid
    _yaw_pid  = make_pid(*config.FOLLOW_YAW_PID)
    _dist_pid = make_pid(*config.FOLLOW_DIST_PID)

def update(state, config):
    v = state.vision
    if v is None or not v.found:
        # No target -> degrade gracefully to holding position (hover-like).
        return ControlSignal(vz=_hold_altitude(state, config))

    # Turn to keep the person centered: yaw toward the horizontal offset.
    yaw_rate = _yaw_pid(v.offset_x, state.dt)

    # Hold follow distance: too far -> move forward, too close -> back off.
    dist_error = (v.distance_cm or config.FOLLOW_DISTANCE_CM) - config.FOLLOW_DISTANCE_CM
    vx = _dist_pid(dist_error, state.dt)

    vz = _hold_altitude(state, config)   # keep 6 ft while following
    return ControlSignal(vx=vx, vy=0.0, vz=vz, yaw_rate=yaw_rate)
```

This shows the pattern for *any* future behavior: read what you need from `state`,
run it through some PID/logic, return a `ControlSignal`. Note it still holds
altitude — behaviors compose the same shared helpers.

---

## 7. Loading & switching behaviors

`control.py` resolves a behavior by name from the `behaviors/` folder using
`importlib`, validates it has an `update`, and falls back to hover on any problem.

```python
import importlib

def load_behavior(name):
    """Import behaviors/<name>.py. Fall back to hover if it can't be loaded."""
    try:
        module = importlib.import_module(f"behaviors.{name}")
        assert hasattr(module, "update"), f"{name} has no update() function"
        return module
    except Exception as e:
        print(f"[CONTROL] Could not load behavior '{name}': {e}. Falling back to hover.")
        return importlib.import_module("behaviors.hover")
```

Ways to choose the behavior (in order of how the user described it):

1. **At launch** — CLI flag: `python control.py --behavior followhuman`.
2. **Default** — no flag → `hover`.
3. **At runtime (extension point)** — `control.py` can watch for a behavior-switch
   request (a command file, a socket message, or a key press) and call
   `switch_behavior(new_name)`, which runs `stop()` on the old behavior and
   `start()` on the new one between ticks. The loop never blocks.

```python
def switch_behavior(current, new_name, state, config):
    behavior_stop(current, state, config)
    new = load_behavior(new_name)
    behavior_start(new, state, config)
    return new
```

---

## 8. Failsafes

`apply_failsafes()` runs after the behavior, every tick. It can override any
signal. Priority order (highest first):

| Condition | Action |
|-----------|--------|
| Telemetry link lost (`not state.link_ok`) | Command `SAFE_STOP` (hover); after a timeout, descend/land |
| Battery below `config.BATTERY_FLOOR` | Override behavior → controlled descent |
| Behavior raised an exception | Substitute a hover signal for this tick |
| Signal exceeds limits | `clamp_signal()` caps speed/tilt before send |

The invariant: **the drone is never left uncommanded, and the safe state is always
hover-then-land.**

---

## 9. config.py (the tuning surface)

Everything you'd touch to tune behavior, in one file:

```python
# --- Control loop ---
LOOP_HZ = 30                 # control ticks per second

# --- Hover / hold ---
HOVER_ALTITUDE_M = 1.8288    # 6 ft target altitude
ALT_PID   = (1.2, 0.0, 0.4)  # (kp, ki, kd) for altitude hold
DRIFT_PID = (0.8, 0.0, 0.2)  # (kp, ki, kd) for horizontal drift

# --- Follow-human ---
FOLLOW_DISTANCE_CM = 200.0
FOLLOW_YAW_PID  = (0.05, 0.0, 0.01)
FOLLOW_DIST_PID = (0.004, 0.0, 0.001)

# --- Safety limits ---
MAX_HORIZONTAL_SPEED = 1.5   # m/s
MAX_VERTICAL_SPEED   = 0.7   # m/s
MAX_YAW_RATE         = 60.0  # deg/s
BATTERY_FLOOR        = 0.15  # land below 15%

# --- Drone link ---
LINK_BACKEND = "stub"        # "stub" | "mavlink" | "tello" | ...
LINK_TIMEOUT_S = 1.0         # telemetry silence before "link lost"
```

---

## 10. drone_link.py (the hardware seam)

The only file that knows how to actually talk to the drone. Everything above it is
hardware-agnostic. It exposes four functions:

```python
def connect(config):  ...     # open the link, return a handle
def read_state():     ...     # -> DroneState   (sensors -> our shape)
def send_signal(sig): ...     # ControlSignal -> autopilot/SDK commands
def disconnect():     ...
```

Implement these against your stack (MAVLink/DroneKit, Tello SDK, a SITL simulator,
etc.). Start with `LINK_BACKEND = "stub"`, which fakes telemetry and just prints
signals — so you can develop and test the whole control system **with no drone and
no risk**.

---

## 11. Build order (suggested)

1. `signals_control.py` — `DroneState`, `ControlSignal`, `make_pid`.
2. `config.py` — the constants above.
3. `drone_link.py` — the **stub** backend first (fake state, print signals).
4. `behaviors/hover.py` — get a stable simulated hover + drift correction.
5. `control.py` — the loop, behavior loader, clamps, failsafes.
6. `behaviors/followhuman.py` — once a vision source can populate `state.vision`.
7. Swap the stub link for real hardware **last**, on the bench, props off.
