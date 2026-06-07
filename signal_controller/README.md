# Signal Controller — Drone Remote-Control Layer

The control layer that sits between the drone's flight hardware and the AI. It
decides, every fraction of a second, **what the drone should physically do** —
and turns that decision into low-level movement signals.

It is built around one idea:

> **`control.py` is a connector with a built-in default.**
> By itself it just makes the drone **hover in place at 6 ft** and corrects any
> drift. Load a *behavior* script (like `followhuman.py`) and `control.py` hands
> control over to it instead — without changing `control.py` at all.

---

## The core concept

`control.py` is a **host** (think of it as the shell the behaviors plug into):

- It owns the connection to the drone (reads telemetry, sends movement signals).
- It runs a fixed-rate **control loop**.
- Each tick, it asks the **active behavior**: "given the current state, what should I do?" — and the behavior returns a movement signal.
- The **default active behavior is `hover`**. Nothing else needs to be running for the drone to hold a stable 6 ft hover.

Behaviors are just small `.py` files that follow one simple contract. Swapping the
drone's job is as easy as pointing `control.py` at a different behavior file.

```
            ┌──────────────────────── control.py (the connector) ────────────────────────┐
            │                                                                              │
  drone  ──▶│  read state ──▶  ACTIVE BEHAVIOR.update(state) ──▶ signal ──▶ send to drone  │──▶ drone
 telemetry  │                  (hover by default)                                          │   motors
            │                                                                              │
            └──────────────────────────────────────────────────────────────────────────────┘
                                         ▲
                                         │  swap the behavior, not the connector
                          ┌──────────────┼───────────────┐
                       hover.py      followhuman.py     (your next behavior).py
                    (default: hold)  (track a person)
```

---

## Default behavior: Hover & Hold

With no behavior loaded, `control.py` runs `hover`:

- **Target altitude:** 6 ft (≈ 1.83 m) above the ground.
- **Target position:** stay put — zero horizontal movement.
- **Drift correction:** if wind or sensor noise pushes the drone off the hold
  point, the hover behavior measures the error and commands a corrective nudge
  back. This is a closed loop (PID) on altitude and horizontal position, so it
  actively fights drift instead of just coasting.

See [ARCHITECTURE.md](ARCHITECTURE.md) for the exact control math.

---

## Loading a different behavior

A behavior is a script that implements the behavior contract (one `update()`
function — see the architecture doc). To run a behavior instead of hovering:

```bash
# Default — drone hovers at 6 ft and holds position
python control.py

# Load a behavior — control.py uses followhuman.py instead of hovering
python control.py --behavior followhuman
```

`control.py` looks up the named script in the `behaviors/` folder, loads it, and
delegates every control tick to it. If the behavior errors out or is missing,
`control.py` **falls back to hover** so the drone never goes uncommanded.

> `followhuman` is the worked example: instead of holding a fixed point, it reads
> the AI's vision target (where the person is, how far away) and steers the drone
> to keep that person centered and at a set follow distance.

---

## Run

```bash
cd signal_controller
python control.py                      # hover at 6 ft (default)
python control.py --behavior followhuman   # follow a detected person
```

Quit with `Ctrl+C` — `control.py` catches it and commands a safe stop/hover before
disconnecting.

---

## File layout

| File | Role |
|------|------|
| `control.py` | **The connector.** Owns the drone link + control loop. Runs the active behavior (default: hover). |
| `config.py` | All tunable numbers in one place — target altitude, PID gains, speed limits, loop rate, link settings. |
| `drone_link.py` | Thin wrapper over the actual drone comms: `read_state()` and `send_signal()`. Swap this to target a different autopilot/SDK. |
| `signals_control.py` | The core data shapes (`DroneState`, `ControlSignal`) and the PID helper — the "signals" math everything shares. |
| `behaviors/hover.py` | Default behavior: hold 6 ft, cancel drift. |
| `behaviors/followhuman.py` | Pluggable behavior: track a person instead of hovering. |

---

## Status

This is the **design + scaffold** stage. The connector model, the behavior
contract, and the hover logic are specified in [ARCHITECTURE.md](ARCHITECTURE.md).
`drone_link.py` is intentionally a stub — drop in the real autopilot/SDK calls for
your hardware there, and nothing else has to change.

---

## Safety notes

- **Default is always safe:** the fallback for *anything* going wrong (missing
  behavior, behavior crash, lost link) is to revert to `hover`.
- **Signals are clamped:** `control.py` limits max horizontal/vertical speed and
  tilt before sending, so a bad behavior can't command a violent maneuver.
- **Hold the props / test on the bench** with motors disabled until the link layer
  and gains are verified.
