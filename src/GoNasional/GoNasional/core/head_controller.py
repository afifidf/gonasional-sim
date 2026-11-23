"""Head scan / track logic working on angles only. No ROS imports."""

from .geometry import clamp


class HeadController:
    """Produces pan/tilt targets.

    Modes:
      track  - follow the ball; falls back to scanning once it is lost
      scan   - sweep the field regardless of the ball (landmark search)
      center - look straight ahead at `center_tilt`

    Tracking is incremental: each new ball observation gives the angular offset
    of the ball from the optical axis, and the command moves by a fraction of
    it. The legacy head PID mapped pixel error straight to an absolute joint
    angle, so the ball could only be centred when the head was at zero.

    Scanning is stop-and-look: the head jumps between pan stops `scan_step`
    apart and holds each for `scan_dwell` s. Frames taken while the head
    swings are blurred and their joint angles lag the image, so landmark
    extraction ignores them; with a continuous sweep that was nearly every
    frame in which a goalpost was in view. scan_dwell = 0 gives the old sweep.
    """

    def __init__(self, pan_limit=1.2, tilt_min=-1.2, tilt_max=0.1,
                 scan_rows=(-0.25, -0.70, -1.10), scan_pan_rate=1.0,
                 scan_tilt_rate=1.5, track_gain=0.5, lost_hold_s=1.0,
                 center_tilt=-0.3, max_step=0.15, scan_step=0.6, scan_dwell=0.5):
        self.pan_limit = pan_limit
        self.tilt_min = tilt_min
        self.tilt_max = tilt_max
        self.scan_rows = tuple(scan_rows)
        self.scan_pan_rate = scan_pan_rate
        self.scan_tilt_rate = scan_tilt_rate
        self.track_gain = track_gain
        self.lost_hold_s = lost_hold_s
        self.center_tilt = center_tilt
        self.max_step = max_step
        self.scan_step = scan_step
        self.scan_dwell = scan_dwell
        self._stop_target = None
        self._dwell_until = None
        self._measured = None

        self.mode = 'track'
        self.pan = 0.0
        self.tilt = center_tilt
        self.scanning = False
        self._scan_dir = 1.0
        self._row = 0
        self._last_obs_stamp = None
        self._last_seen = None
        self._last_tick = None

    def reset(self, pan, tilt):
        """Re-seed the command from the measured joints (after an action page)."""
        self.pan = clamp(pan, -self.pan_limit, self.pan_limit)
        self.tilt = clamp(tilt, self.tilt_min, self.tilt_max)
        self._last_tick = None

    def set_mode(self, mode):
        if mode in ('track', 'scan', 'center'):
            self.mode = mode

    def observe(self, offset_pan, offset_tilt, stamp):
        """Ball offset from the optical axis (rad, left/up positive) at `stamp`."""
        if self._last_obs_stamp is not None and stamp <= self._last_obs_stamp:
            return
        self._last_obs_stamp = stamp
        self._last_seen = stamp
        if self.mode != 'track':
            return
        step_pan = clamp(self.track_gain * offset_pan, -self.max_step, self.max_step)
        step_tilt = clamp(self.track_gain * offset_tilt, -self.max_step, self.max_step)
        self.pan = clamp(self.pan + step_pan, -self.pan_limit, self.pan_limit)
        self.tilt = clamp(self.tilt + step_tilt, self.tilt_min, self.tilt_max)

    def update(self, now, measured=None):
        """Advance time; returns the (pan, tilt) command.

        measured: (pan, tilt) the joints actually reached, if known. Scan stops
        only start their dwell once the head is really there; the servos lag
        the command, and dwelling on the command alone left the head moving.
        """
        self._measured = measured
        dt = 0.0 if self._last_tick is None else clamp(now - self._last_tick, 0.0, 0.2)
        self._last_tick = now

        if self.mode == 'center':
            self.scanning = False
            self.pan = self._approach(self.pan, 0.0, self.scan_pan_rate * dt)
            self.tilt = self._approach(self.tilt, self.center_tilt,
                                       self.scan_tilt_rate * dt)
            return self.pan, self.tilt

        ball_fresh = (self._last_seen is not None
                      and now - self._last_seen <= self.lost_hold_s)
        if self.mode == 'track' and ball_fresh:
            self.scanning = False
            return self.pan, self.tilt

        if not self.scanning:
            # Start the sweep from where the head is, moving away from the
            # nearest limit so the first pass covers the most ground.
            self.scanning = True
            self._scan_dir = -1.0 if self.pan > 0.0 else 1.0
            self._row = min(range(len(self.scan_rows)),
                            key=lambda i: abs(self.scan_rows[i] - self.tilt))
            self._stop_target = None
            self._dwell_until = None

        if self.scan_dwell > 0.0:
            return self._scan_stops(now, dt)

        self.pan += self._scan_dir * self.scan_pan_rate * dt
        if self.pan >= self.pan_limit:
            self.pan = self.pan_limit
            self._scan_dir = -1.0
            self._row = (self._row + 1) % len(self.scan_rows)
        elif self.pan <= -self.pan_limit:
            self.pan = -self.pan_limit
            self._scan_dir = 1.0
            self._row = (self._row + 1) % len(self.scan_rows)
        self.tilt = self._approach(self.tilt, self.scan_rows[self._row],
                                   self.scan_tilt_rate * dt)
        return self.pan, self.tilt

    def _scan_stops(self, now, dt):
        tilt_target = self.scan_rows[self._row]
        if self._stop_target is None:
            self._stop_target = clamp(self.pan, -self.pan_limit, self.pan_limit)
        at_stop = (abs(self.pan - self._stop_target) < 1e-6
                   and abs(self.tilt - tilt_target) < 1e-6)
        if at_stop and self._measured is not None:
            at_stop = (abs(self._measured[0] - self._stop_target) < 0.04
                       and abs(self._measured[1] - tilt_target) < 0.04)
        if at_stop:
            if self._dwell_until is None:
                self._dwell_until = now + self.scan_dwell
            elif now >= self._dwell_until:
                self._dwell_until = None
                nxt = self._stop_target + self._scan_dir * self.scan_step
                if abs(nxt) > self.pan_limit + 1e-6:
                    # End of the row: next row, sweep back the other way.
                    self._scan_dir = -self._scan_dir
                    self._row = (self._row + 1) % len(self.scan_rows)
                    nxt = clamp(nxt, -self.pan_limit, self.pan_limit)
                    if abs(nxt - self._stop_target) < 1e-6:
                        nxt = self._stop_target
                self._stop_target = clamp(nxt, -self.pan_limit, self.pan_limit)
        # Swing fast between stops: the time is better spent standing still.
        self.pan = self._approach(self.pan, self._stop_target, 2.0 * self.scan_pan_rate * dt)
        self.tilt = self._approach(self.tilt, self.scan_rows[self._row],
                                   self.scan_tilt_rate * dt)
        return self.pan, self.tilt

    @staticmethod
    def _approach(value, target, max_delta):
        if value < target:
            return min(target, value + max_delta)
        return max(target, value - max_delta)
