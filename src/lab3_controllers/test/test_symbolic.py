"""Tests of the symbolic model: soundness of the transition table and the guarantee of the synthesized controller."""
import math
import os
import random
import tempfile

import pytest

np = pytest.importorskip('numpy')

from lab3_controllers import symbolic as sy  # noqa: E402


def flow(x, y, th, v, w, tau):
    """Exact unicycle motion under a constant input."""
    if abs(w) < 1e-9:
        return x + v * tau * math.cos(th), y + v * tau * math.sin(th), th
    return (x + (v / w) * (math.sin(th + w * tau) - math.sin(th)),
            y - (v / w) * (math.cos(th + w * tau) - math.cos(th)), th + w * tau)


def small_abstraction(safe_fn=None):
    dth = 2 * math.pi / 24
    tau = 1.0
    inputs = [(0.1, k * dth / tau) for k in (-2, -1, 0, 1, 2)]
    return sy.Abstraction((-0.3, 0.3), (-0.3, 0.3), 0.02, 24, inputs, tau, safe_fn).build()


def test_table_contains_every_possible_successor():
    """Soundness: from ANY point of a cell, the exact next state lies in a listed successor cell (or OUT)."""
    a = small_abstraction()
    rng = random.Random(0)
    for _ in range(4000):
        q = rng.randrange(a.N)
        m = rng.randrange(len(a.inputs))
        ix, iy, ith = a.cells[q]
        x = a.x0 + (ix + rng.random()) * a.dx
        y = a.y0 + (iy + rng.random()) * a.dx
        th = -math.pi + (ith + rng.random()) * a.dth
        v, w = a.inputs[m]
        nx, ny, nth = flow(x, y, th, v, w, a.tau)
        target = a.state_index(nx, ny, nth)
        assert target in set(a.successors(q, m).tolist()), (q, m, target)


def test_table_corners_are_covered_too():
    """Same check at the extreme points (cell corners), where the over-approximation is tightest."""
    a = small_abstraction()
    rng = random.Random(1)
    for _ in range(2000):
        q = rng.randrange(a.N)
        m = rng.randrange(len(a.inputs))
        ix, iy, ith = a.cells[q]
        x = a.x0 + (ix + rng.choice((1e-9, 1 - 1e-9))) * a.dx
        y = a.y0 + (iy + rng.choice((1e-9, 1 - 1e-9))) * a.dx
        th = -math.pi + (ith + rng.choice((1e-9, 1 - 1e-9))) * a.dth
        nx, ny, nth = flow(x, y, th, *a.inputs[m], a.tau)
        assert a.state_index(nx, ny, nth) in set(a.successors(q, m).tolist())


def test_successors_outside_the_domain_are_out():
    a = small_abstraction(lambda x, y, th: np.abs(x) < 0.1)
    assert a.N < 20 * 20 * 24
    q = int(np.argmax(a.cells[:, 0]))                 # a cell at the edge of the safe set
    outs = [a.OUT in a.successors(q, m).tolist() for m in range(len(a.inputs))]
    assert any(outs)


def test_synthesis_is_exact_and_ranks_strictly_decrease():
    a = small_abstraction()
    ex, ey, _ = a.centre(a.cells[:, 0], a.cells[:, 1], a.cells[:, 2])
    goal = (np.hypot(ex - 0.1, ey) < 0.05)
    rank, pol = a.synthesize(goal)
    win = (rank < sy.INF) & ~goal
    assert win.sum() > 0
    for q in np.flatnonzero(win)[:500]:
        succ = a.successors(int(q), int(pol[q]))
        assert a.OUT not in succ.tolist()
        worst = max(int(rank[s]) for s in succ)
        assert rank[q] == worst + 1                    # rank is exactly 1 + worst successor
        assert worst < rank[q]


def test_guarantee_closed_loop_reaches_the_goal_within_rank_steps():
    """From random points of winning cells, the closed loop reaches the goal in at most rank(q) steps."""
    a = small_abstraction()
    ex, ey, _ = a.centre(a.cells[:, 0], a.cells[:, 1], a.cells[:, 2])
    goal = (np.hypot(ex - 0.1, ey) < 0.05)
    rank, pol = a.synthesize(goal)
    pool = np.flatnonzero((rank < sy.INF) & ~goal)
    assert len(pool) > 50
    rng = random.Random(2)
    for _ in range(300):
        q = int(rng.choice(pool))
        bound = int(rank[q])
        ix, iy, ith = a.cells[q]
        x = a.x0 + (ix + rng.random()) * a.dx
        y = a.y0 + (iy + rng.random()) * a.dx
        th = -math.pi + (ith + rng.random()) * a.dth
        steps = 0
        while not goal[q] and steps <= bound:
            v, w = a.inputs[int(pol[q])]
            x, y, th = flow(x, y, th, v, w, a.tau)
            q = a.state_index(x, y, th)
            assert q != a.OUT, 'left the safe domain'
            steps += 1
        assert goal[q] and steps <= bound


def test_policy_save_load_roundtrip():
    a = small_abstraction()
    ex, ey, _ = a.centre(a.cells[:, 0], a.cells[:, 1], a.cells[:, 2])
    goals = [np.hypot(ex - 0.1, ey) < 0.05, np.hypot(ex + 0.1, ey) < 0.05]
    pols = [a.synthesize(g)[1] for g in goals]
    pol = sy.Policy.from_abstraction(a, pols, goals, dict(radius=0.5))
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'p.npz')
        pol.save(path)
        back = sy.Policy.load(path)
    assert back.N == pol.N and (back.policy == pol.policy).all() and (back.goals == pol.goals).all()
    for _ in range(200):
        x, y, th = random.uniform(-0.25, 0.25), random.uniform(-0.25, 0.25), random.uniform(-3, 3)
        assert back.state_index(x, y, th) == pol.state_index(x, y, th)
        q = pol.state_index(x, y, th)
        if q >= 0:
            assert back.action(0, q) == pol.action(0, q)


def test_circle_setup_geometry():
    """The circle starts tangentially at (0, 0): the start state is inside the domain."""
    a, goals, info = sy.circle_setup(dx=0.02, n_th=24, tau=1.0, turn_cells=(-1, 0, 1), n_goals=12)
    assert a.state_index(0.0, 0.0, 0.0) < a.N
    assert info['centre'] == (0.0, 0.5) and abs(info['radius'] - 0.5) < 1e-12     # 1 m diameter
    assert len(goals) == 12 and all(g.any() for g in goals)
