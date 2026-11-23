"""Model-based heading observer: predicts the PRESENT heading and turn rate of a robot whose state arrives late, rarely and noisy.

The problem. The map pose arrives 0.1-0.6 s late, 3-15 times a second, with noise and with freezes of several seconds. The
robot also answers slowly to commands (about a second) and ignores small ones (deadband). A controller that takes the old pose and
guesses the speed from noisy differences of it oscillates (see tools/yaw_real_sim.py).

The idea (the same as a Smith predictor / fixed-lag observer). We know what we commanded, so we SIMULATE the robot:

    command --> deadband --> first-order lag (gain g, time constant tau) --> turn rate w --> integrate --> heading

The simulated heading is available NOW and without noise. The slow measurement is used only to correct the simulation:
when a sample arrives, compare it with what the simulation said at the (delayed) moment the sample describes, and nudge
the simulation by a fraction of the difference (gains k1 for the heading, k2 for the turn rate). During a freeze the
simulation simply keeps running on the commands we sent, which is what the robot really does.

The controller then uses   heading_hat + w_hat * lead   (where the robot will come to rest) instead of the raw measurement.
"""
import math
from collections import deque

from lab3_controllers.common import wrap_angle


class HeadingObserver:
    def __init__(self, tau=0.8, gain=1.0, latency=0.2, k1=0.5, k2=0.5, deadband_w=0.0, history=8.0, angular=True):
        self.tau, self.gain, self.latency = max(tau, 1e-3), gain, latency
        self.k1, self.k2, self.deadband = k1, k2, deadband_w
        self.history = history
        self._wrap = wrap_angle if angular else (lambda a: a)      # a heading wraps around, a straight-line position does not
        self.yaw = None                     # simulated heading now
        self.w = 0.0                        # simulated turn rate now
        self.t = None                       # time of the simulation state
        self._hist = deque()                # (time, heading) of the simulation, to compare with delayed samples

    def reset(self):
        self.yaw, self.w, self.t = None, 0.0, None
        self._hist.clear()

    def advance(self, now, u_sent):
        """Run the model from its last time to `now` with the command that was being sent (held constant)."""
        if self.yaw is None:
            return
        dt = now - self.t
        if dt <= 0.0:
            return
        u = 0.0 if abs(u_sent) <= self.deadband else u_sent                 # the robot drops commands inside its deadband
        target = self.gain * u                                               # the turn rate the command asks for
        decay = math.exp(-dt / self.tau)
        self.yaw = self._wrap(self.yaw + target * dt + (self.w - target) * self.tau * (1.0 - decay))
        self.w = target + (self.w - target) * decay
        self.t = now
        self._hist.append((now, self.yaw))
        while self._hist and now - self._hist[0][0] > self.history:
            self._hist.popleft()

    def _yaw_at(self, t):
        """Simulated heading at an earlier time (linear interpolation in the history), or None if too old."""
        h = self._hist
        if len(h) < 2 or t < h[0][0]:
            return None
        prev = h[0]
        for cur in h:
            if cur[0] >= t:
                span = cur[0] - prev[0]
                f = 0.0 if span <= 0.0 else (t - prev[0]) / span
                return self._wrap(prev[1] + f * self._wrap(cur[1] - prev[1]))
            prev = cur
        return h[-1][1]

    def correct(self, now, yaw_meas):
        """A new sample arrived at `now`; it describes the heading at now - latency. Nudge the simulation."""
        if self.yaw is None:
            self.yaw, self.w, self.t = yaw_meas, 0.0, now
            self._hist.append((now, yaw_meas))
            return
        t_m = now - self.latency
        ref = self._yaw_at(t_m)
        if ref is None:                                                       # sample older than the history: use the oldest
            ref = self._hist[0][1] if self._hist else self.yaw
        e = self._wrap(yaw_meas - ref)
        self.yaw = self._wrap(self.yaw + self.k1 * e)
        self.w += self.k2 * e
        # keep the history consistent: the correction also applies to everything after the sample's moment
        self._hist = deque((t, self._wrap(y + self.k1 * e) if t >= t_m else y) for t, y in self._hist)

    def predicted_rest(self, lead):
        """Where the heading will come to rest if the command stopped now: heading + turn rate * lead."""
        return self._wrap(self.yaw + self.w * lead)


class PositionObserver(HeadingObserver):
    """The same observer for a straight-line position: x and the speed (m, m/s), no wrap-around. `deadband_w` is the robot's
    forward-speed deadband (m/s), `tau` / `gain` the forward speed response."""

    def __init__(self, tau=0.6, gain=1.0, latency=0.2, k1=0.5, k2=0.5, deadband_v=0.0, history=8.0):
        super().__init__(tau=tau, gain=gain, latency=latency, k1=k1, k2=k2, deadband_w=deadband_v, history=history, angular=False)

    @property
    def x(self):
        return self.yaw

    @property
    def v(self):
        return self.w
