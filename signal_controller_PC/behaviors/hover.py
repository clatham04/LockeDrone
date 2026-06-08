"""hover — hold position in place.

The simplest behavior: command centered sticks every tick, so the drone holds via
its own optical-flow altitude/position hold. signals_control.py adds the trim and
handles takeoff/landing; this just decides the sticks.

(Future behaviors like follow_human read sensor/vision data from `state` and return
adjusted sticks here — same shape.)
"""

CENTER = 128


def controller(state):
    """Return (roll, pitch, throttle, yaw) for this tick. All centered = hover."""
    return CENTER, CENTER, CENTER, CENTER
