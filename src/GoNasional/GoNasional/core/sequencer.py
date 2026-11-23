"""Non-blocking step sequencer. Replaces chains of time.sleep() in callbacks."""

from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class Step:
    """One step of a motion sequence.

    on_enter runs once when the step starts. The step ends when at least
    min_time has passed and `done` (if given) returns True, or when `timeout`
    expires. A step with neither `done` nor `timeout` lasts exactly min_time.
    """

    name: str
    on_enter: Optional[Callable[[], None]] = None
    done: Optional[Callable[[], bool]] = None
    min_time: float = 0.0
    timeout: Optional[float] = None


class Sequencer:
    """Runs a list of Steps from a periodic tick, never sleeping."""

    def __init__(self, on_timeout=None):
        self._steps = []
        self._index = 0
        self._step_start = 0.0
        self._entered = False
        self._on_finish = None
        self._on_timeout = on_timeout
        self.name = None

    @property
    def active(self):
        return self.name is not None

    @property
    def step_name(self):
        if not self.active:
            return None
        return self._steps[self._index].name

    def start(self, name, steps, on_finish=None):
        self.name = name
        self._steps = list(steps)
        self._index = 0
        self._entered = False
        self._on_finish = on_finish
        if not self._steps:
            self._finish()

    def abort(self):
        self.name = None
        self._steps = []
        self._on_finish = None

    def _finish(self):
        on_finish = self._on_finish
        self.name = None
        self._steps = []
        self._on_finish = None
        if on_finish is not None:
            on_finish()

    def tick(self, now):
        """Advance at most one step per call so every step gets its on_enter."""
        if not self.active:
            return
        step = self._steps[self._index]
        if not self._entered:
            self._entered = True
            self._step_start = now
            if step.on_enter is not None:
                running = self.name
                step.on_enter()
                # on_enter may abort or restart the sequence.
                if self.name != running or not self._entered:
                    return
            return

        elapsed = now - self._step_start
        finished = False
        if elapsed >= step.min_time:
            if step.done is not None:
                finished = bool(step.done())
            elif step.timeout is None:
                finished = True
        if not finished and step.timeout is not None and elapsed >= step.timeout:
            finished = True
            if step.done is not None and self._on_timeout is not None:
                self._on_timeout(self.name, step.name)
        if not finished:
            return

        self._index += 1
        self._entered = False
        if self._index >= len(self._steps):
            self._finish()
