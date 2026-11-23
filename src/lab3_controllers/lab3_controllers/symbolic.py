"""Symbolic (abstraction-based) control of the differential-drive robot.

Pipeline (same idea as SCOTS / abstraction-based control):
  1. SYMBOLIC MODEL: the state (x, y, theta) is partitioned into cells (a finite set Q), the input (v, w) is a
     small finite set U held for one sampling time tau, and the transition relation T(q, u) is the SET OF ALL
     cells the robot can reach from ANY point of cell q under input u after tau seconds.
  2. TRANSITION TABLE: T is computed with a sound over-approximation. For the exact unicycle flow
        x' = x + (v/w)(sin(th + w tau) - sin th),  y' = y - (v/w)(cos(th + w tau) - cos th),  th' = th + w tau
     the image of a point differs from the image of the cell centre by at most
        |dx'| <= hx + v tau h_th,   |dy'| <= hy + v tau h_th,   |dth'| <= h_th
     (the derivative of x', y' with respect to theta is bounded by v tau). Every cell that touches that box is a
     successor, so the table is guaranteed to contain all possible successors (see test_symbolic.py).
     Successors outside the modelled domain are all mapped to the single cell OUT (unsafe).
  3. CONTROLLER SYNTHESIS: reach-avoid on the nondeterministic table. rank(q) is the smallest number of steps
     within which a goal cell is reached whatever successor happens; policy(q) is an input achieving it. The
     controller is a lookup: measure state -> cell -> input.
"""

import math

import numpy as np

INF = np.iinfo(np.int32).max // 2


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Abstraction:
    """Cells of size (dx, dx, 2pi/n_th) over a box, restricted to a `safe` mask; inputs U; sampling time tau."""

    def __init__(self, x_range, y_range, dx, n_th, inputs, tau, safe_fn=None):
        self.x0, self.y0 = x_range[0], y_range[0]
        self.nx = int(round((x_range[1] - x_range[0]) / dx))
        self.ny = int(round((y_range[1] - y_range[0]) / dx))
        self.dx = dx
        self.n_th = n_th
        self.dth = 2.0 * math.pi / n_th
        self.inputs = np.array(inputs, float)          # shape (M, 2): v, w
        self.tau = tau
        # all cells of the box; the domain keeps the safe ones, in a compact numbering 0..N-1 (N = OUT)
        ix, iy, ith = np.meshgrid(np.arange(self.nx), np.arange(self.ny), np.arange(n_th), indexing='ij')
        ix, iy, ith = ix.ravel(), iy.ravel(), ith.ravel()
        cx, cy, cth = self.centre(ix, iy, ith)
        safe = np.ones(len(ix), bool) if safe_fn is None else safe_fn(cx, cy, cth)
        self.cells = np.stack([ix[safe], iy[safe], ith[safe]], axis=1)       # (N, 3) integer cell coordinates
        self.N = len(self.cells)
        self.OUT = self.N
        self.index = -np.ones((self.nx, self.ny, n_th), np.int64)
        self.index[self.cells[:, 0], self.cells[:, 1], self.cells[:, 2]] = np.arange(self.N)
        self.indptr = None
        self.succ = None

    # ---- geometry ------------------------------------------------------------------------------------
    def centre(self, ix, iy, ith):
        return (self.x0 + (np.asarray(ix) + 0.5) * self.dx, self.y0 + (np.asarray(iy) + 0.5) * self.dx,
                -math.pi + (np.asarray(ith) + 0.5) * self.dth)

    def cell_of(self, x, y, th):
        """Integer cell coordinates of a point, or None when outside the box."""
        ix = int(math.floor((x - self.x0) / self.dx))
        iy = int(math.floor((y - self.y0) / self.dx))
        ith = int(math.floor((wrap(th) + math.pi) / self.dth)) % self.n_th
        if not (0 <= ix < self.nx and 0 <= iy < self.ny):
            return None
        return ix, iy, ith

    def state_index(self, x, y, th):
        c = self.cell_of(x, y, th)
        if c is None:
            return self.OUT
        k = self.index[c]
        return self.OUT if k < 0 else int(k)

    # ---- the transition table ------------------------------------------------------------------------
    def build(self):
        """Fill the table: for every (state, input) the sorted unique list of ALL successor states.

        Stored in CSR form: the successors of pair k = state * M + input are succ[indptr[k]:indptr[k+1]].
        All N cells are processed at once (numpy arrays of length N), once per input."""
        N, M = self.N, len(self.inputs)
        cx, cy, cth = self.centre(self.cells[:, 0], self.cells[:, 1], self.cells[:, 2])
        rows, cols = [], []                       # one entry per (pair, successor)
        for m, (v, w) in enumerate(self.inputs):
            # 1. where does the CENTRE of every cell go under this input, and which cell range does the
            #    image of the whole cell cover?
            lo, hi = self._image_range(cx, cy, cth, v, w)
            # 2. list every cell in those ranges (the same loop for all states; invalid ones are masked)
            span = [int((hi[i] - lo[i]).max()) + 1 for i in range(3)]
            for ox in range(span[0]):
                for oy in range(span[1]):
                    for ot in range(span[2]):
                        j = (lo[0] + ox, lo[1] + oy, lo[2] + ot)
                        valid = (j[0] <= hi[0]) & (j[1] <= hi[1]) & (j[2] <= hi[2])
                        rows.append(np.arange(N)[valid] * M + m)
                        cols.append(self._dest(*j)[valid])
        self._store(np.concatenate(rows), np.concatenate(cols))
        return self

    def _image_range(self, cx, cy, cth, v, w):
        """Integer cell ranges (lo, hi), each a tuple (x, y, theta), covered by the image of every cell under
        the input (v, w) held for tau. The image of the centre is exact; the cell's size is added as the
        sound radius r (position) and h_th (heading)."""
        tau, hth = self.tau, self.dth / 2.0
        if abs(w) < 1e-9:                                    # straight line
            nx_ = cx + v * tau * np.cos(cth)
            ny_ = cy + v * tau * np.sin(cth)
        else:                                                # circular arc
            nx_ = cx + (v / w) * (np.sin(cth + w * tau) - np.sin(cth))
            ny_ = cy - (v / w) * (np.cos(cth + w * tau) - np.cos(cth))
        nth = cth + w * tau
        r = self.dx / 2.0 + abs(v) * tau * hth               # sound radius of the image in x and y
        lo = (np.floor((nx_ - r - self.x0) / self.dx).astype(int),
              np.floor((ny_ - r - self.y0) / self.dx).astype(int),
              np.floor((nth - hth + math.pi) / self.dth + 1e-9).astype(int))
        hi = (np.floor((nx_ + r - self.x0) / self.dx).astype(int),
              np.floor((ny_ + r - self.y0) / self.dx).astype(int),
              np.floor((nth + hth + math.pi) / self.dth - 1e-9).astype(int))
        return lo, hi

    def _dest(self, jx, jy, jt):
        """State index of cell (jx, jy, jt) for every source state; OUT if outside the box or the safe set."""
        inside = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
        k = self.index[np.clip(jx, 0, self.nx - 1), np.clip(jy, 0, self.ny - 1), jt % self.n_th]
        dest = np.full(self.N, self.OUT, np.int64)
        dest[inside] = np.where(k[inside] < 0, self.OUT, k[inside])
        return dest

    def _store(self, rows, cols):
        """Sort, remove duplicates (also repeated OUT) and keep the table in CSR form."""
        order = np.lexsort((cols, rows))
        rows, cols = rows[order], cols[order]
        keep = np.ones(len(rows), bool)
        keep[1:] = (rows[1:] != rows[:-1]) | (cols[1:] != cols[:-1])
        rows, cols = rows[keep], cols[keep]
        self.succ = cols.astype(np.int32)
        self.indptr = np.concatenate([[0], np.cumsum(np.bincount(rows, minlength=self.N * len(self.inputs)))])

    def successors(self, q, m):
        k = q * len(self.inputs) + m
        return self.succ[self.indptr[k]:self.indptr[k + 1]]

    # ---- synthesis -----------------------------------------------------------------------------------
    def synthesize(self, goal):
        """goal: boolean array over the N states. Returns (rank, policy): rank[q] = guaranteed number of steps
        to reach the goal while staying in the domain (INF if none), policy[q] = input index (-1 if none)."""
        N, M = self.N, len(self.inputs)
        rank = np.full(N + 1, INF, np.int64)       # index N = OUT, never winning
        rank[:N][goal] = 0
        policy = np.full(N, -1, np.int64)
        starts = self.indptr[:-1]
        for _ in range(100000):
            worst = np.maximum.reduceat(rank[self.succ], starts).reshape(N, M)      # worst successor per (q, u)
            cand = np.where(worst >= INF, INF, worst + 1)
            best = cand.min(axis=1)
            better = best < rank[:N]
            better &= ~goal
            if not better.any():
                break
            rank[:N][better] = best[better]
            policy[better] = cand[better].argmin(axis=1)
        return rank[:N], policy


def circle_setup(radius=0.5, tube=0.08, dx=0.02, n_th=72, v=0.1, tau=0.2, turn_cells=(-3, -2, -1, 0, 1, 2, 3),
                 heading_band=math.radians(70.0), n_goals=12):
    """A circle of the given radius (default 1 m diameter) starting tangentially at (0, 0) heading +x, driven
    counter-clockwise: centre (0, radius). Safe set: within `tube` of the circle (cell centre margin by the
    half diagonal, so the guarantee holds for every point of the cell) and heading within the band of the tangent.
    Goals: n_goals arcs, each a small window of the circle with heading close to the tangent."""
    # turn rates are chosen so that holding one for tau turns the robot by a WHOLE number of heading cells:
    # w = k * (2 pi / n_th) / tau. Then the heading successor is exactly one cell (no spreading of the
    # heading uncertainty), which is what makes guaranteed reachability possible.
    w_set = [k * (2.0 * math.pi / n_th) / tau for k in turn_cells]
    margin = dx * math.sqrt(2.0) / 2.0
    cxy = (0.0, radius)

    def radial(x, y):
        return np.hypot(x - cxy[0], y - cxy[1]) - radius

    def tangent(x, y):                                    # CCW tangent heading at the point
        return np.arctan2(y - cxy[1], x - cxy[0]) + math.pi / 2.0

    def safe_fn(x, y, th):
        err_h = np.abs(((th - tangent(x, y)) + math.pi) % (2 * math.pi) - math.pi)
        return (np.abs(radial(x, y)) <= tube - margin) & (err_h <= heading_band)

    span = radius + tube + 2 * dx
    a = Abstraction((-span, span), (radius - span, radius + span), dx, n_th,
                    [(v, w) for w in w_set], tau, safe_fn)
    ex, ey, eth = a.centre(a.cells[:, 0], a.cells[:, 1], a.cells[:, 2])
    ang = (np.arctan2(ey - cxy[1], ex - cxy[0]) + math.pi / 2.0)         # angle along the lap, 0 at the start
    ang = np.mod(ang, 2.0 * math.pi)
    err_h = np.abs(((eth - tangent(ex, ey)) + math.pi) % (2 * math.pi) - math.pi)
    goals = []
    for k in range(1, n_goals + 1):
        centre_ang = (2.0 * math.pi * k / n_goals) % (2.0 * math.pi)
        d = np.abs(((ang - centre_ang) + math.pi) % (2 * math.pi) - math.pi)
        goals.append((d <= math.radians(4.0)) & (np.abs(radial(ex, ey)) <= 0.03) & (err_h <= math.radians(25.0)))
    return a, goals, dict(centre=cxy, radius=radius, tube=tube, n_goals=n_goals, tangent=tangent, radial=radial)


class Policy:
    """The deployable part of a synthesized controller: the domain (cell list) and, for every goal, the input to
    apply in each cell. Small (a few MB) compared with the transition table, so the robot can load it instantly."""

    def __init__(self, x0, y0, dx, n_th, tau, inputs, cells, policy, goals, meta, rank=None):
        self.x0, self.y0, self.dx, self.n_th, self.tau = x0, y0, dx, n_th, tau
        self.dth = 2.0 * math.pi / n_th
        self.inputs = np.asarray(inputs, float)
        self.cells = np.asarray(cells)
        self.policy = np.asarray(policy, np.int8)           # (n_goals, N), -1 = no winning input
        self.goals = np.asarray(goals, bool)                 # (n_goals, N)
        self.rank = None if rank is None else np.asarray(rank, np.int16)   # guaranteed steps, 32767 = none
        self.meta = meta
        self.N = len(self.cells)
        self.nx = int(self.cells[:, 0].max()) + 1
        self.ny = int(self.cells[:, 1].max()) + 1
        self.index = -np.ones((self.nx, self.ny, n_th), np.int32)
        self.index[self.cells[:, 0], self.cells[:, 1], self.cells[:, 2]] = np.arange(self.N, dtype=np.int32)

    @classmethod
    def from_abstraction(cls, a, policies, goals, meta, ranks=None):
        rk = None if ranks is None else np.minimum(np.array(ranks), 32767).astype(np.int16)
        return cls(a.x0, a.y0, a.dx, a.n_th, a.tau, a.inputs, a.cells.astype(np.int16), np.array(policies),
                   np.array(goals), meta, rk)

    def save(self, path):
        np.savez_compressed(path, x0=self.x0, y0=self.y0, dx=self.dx, n_th=self.n_th, tau=self.tau,
                            inputs=self.inputs, cells=self.cells.astype(np.int16), policy=self.policy,
                            goals=np.packbits(self.goals, axis=1), n_states=self.N,
                            rank=(self.rank if self.rank is not None else np.zeros((0, 0), np.int16)),
                            meta=np.array(sorted(self.meta.items()), dtype=object))

    @classmethod
    def load(cls, path):
        z = np.load(path, allow_pickle=True)
        n = int(z['n_states'])
        goals = np.unpackbits(z['goals'], axis=1)[:, :n].astype(bool)
        meta = {k: v for k, v in z['meta']}
        rank = z['rank'] if 'rank' in z.files and z['rank'].size else None
        return cls(float(z['x0']), float(z['y0']), float(z['dx']), int(z['n_th']), float(z['tau']),
                   z['inputs'], z['cells'].astype(np.int64), z['policy'], goals, meta, rank)

    def state_index(self, x, y, th):
        """Index of the cell containing the state, or -1 when it is outside the modelled (safe) domain."""
        ix = int(math.floor((x - self.x0) / self.dx))
        iy = int(math.floor((y - self.y0) / self.dx))
        if not (0 <= ix < self.nx and 0 <= iy < self.ny):
            return -1
        ith = int(math.floor((wrap(th) + math.pi) / self.dth)) % self.n_th
        return int(self.index[ix, iy, ith])

    def goal_reached(self, k, q):
        return bool(self.goals[k, q])

    def action(self, k, q):
        m = int(self.policy[k, q])
        return None if m < 0 else (float(self.inputs[m, 0]), float(self.inputs[m, 1]))

    def robust_action(self, k, x, y, th, reach=2):
        """OUTSIDE THE FORMAL GUARANTEE. When the measured cell has no winning input (measurement noise puts the
        state next to the boundary of the winning set, or just outside the tube), use the best-ranked winning
        cell within `reach` cells (+-1 heading cell) of the measurement. Returns (v, w) or None."""
        if self.rank is None:
            return None
        ix = int(math.floor((x - self.x0) / self.dx))
        iy = int(math.floor((y - self.y0) / self.dx))
        ith = int(math.floor((wrap(th) + math.pi) / self.dth)) % self.n_th
        best, best_key = None, None
        for dx_ in range(-reach, reach + 1):
            for dy_ in range(-reach, reach + 1):
                jx, jy = ix + dx_, iy + dy_
                if not (0 <= jx < self.nx and 0 <= jy < self.ny):
                    continue
                for dt_ in (-1, 0, 1):
                    q = int(self.index[jx, jy, (ith + dt_) % self.n_th])
                    if q < 0 or self.policy[k, q] < 0:
                        continue
                    key = (int(self.rank[k, q]), abs(dx_) + abs(dy_) + abs(dt_))
                    if best_key is None or key < best_key:
                        best, best_key = q, key
        return None if best is None else self.action(k, best)
