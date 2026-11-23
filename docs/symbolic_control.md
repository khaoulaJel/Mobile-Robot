# Symbolic control

Abstraction-based control of the differential-drive robot (same idea as SCOTS). Code: `lab3_controllers/symbolic.py`, node
`symbolic_controller.py`, generation `tools/generate_symbolic.py`, simulation `tools/symbolic_sim.py`, tests `test/test_symbolic.py`.

## Problem

A circle of 1 m diameter (centre (0, 0.5), radius 0.5 m), starting tangentially at (0, 0), counter-clockwise. Safe set: within 8 cm of the circle and
heading within 45 deg of the tangent. Specification: reach 12 goal arcs one after another without leaving the safe set.

## Symbolic model

- **Model:** kinematic unicycle, state `(x, y, theta)`, input `(v, w)` held constant for `tau = 1 s`.
- **States (cells):** 0.5 cm x 0.5 cm x 5 deg, restricted to the safe set: **345,888** cells. Everything outside is one special cell `OUT`.
- **Inputs:** `v = 0.1 m/s` and 13 turn rates `w = k * (5 deg) / tau`, `k = -6..6`. Each input turns the heading by a whole number of heading cells, so the
  heading successor is exactly one cell and heading uncertainty does not grow.

## Transition table (all possible successor cells)

For every (cell, input) the table lists every cell reachable from **any** point of the cell after `tau`. The exact flow is

```
x' = x + (v/w)(sin(th + w tau) - sin th),   y' = y - (v/w)(cos(th + w tau) - cos th),   th' = th + w tau     (straight line if w = 0)
```

A point of a cell with half-sizes `hx` (position) and `h_th` (heading) differs from the cell centre by at most `hx + v tau h_th` in the image's x and y (the
derivative of `x'`, `y'` with respect to `theta` is bounded by `v tau`) and by `h_th` in heading. Every cell touching that box is listed, so the table is a
sound over-approximation. Size: 4.5 million (cell, input) pairs, 44 million successor entries (mean 9.8 per pair, at most 16).
`tools/generate_symbolic.py` writes a readable excerpt (`results/symbolic/transition_table_excerpt.csv`); `--full-table PATH` stores the complete table.

## Synthesis

Reach-avoid fixed point on the nondeterministic table: goal cells have rank 0; a cell gets rank `k+1` if some input leads **all** successors to cells of rank
at most `k` and none to `OUT`. The rank is the guaranteed number of steps; the input achieving it is the policy. One policy per goal arc is stored
(`config/symbolic_circle.npz`, 1.9 MB: cell list, policy, goal masks, ranks).
Result: from the start and from any cell of the previous goal, each arc is reached within 34 steps (34 s).

## Run time

Every `tau` the node measures the state, finds its cell, advances to the next goal when the current one is reached, applies the policy input and holds it
(zero-order hold). If the state leaves the modelled tube or no winning input exists it stops. With `robust_lookup: true` (default) a measured cell with no
input falls back to the best-ranked neighbouring cell (noise, unmodelled lag); this is **outside the formal guarantee** and is logged.

## Tests

`test/test_symbolic.py`: every sampled point and cell corner lands in a listed successor cell (soundness); synthesis is exact (rank = 1 + worst successor) and
ranks strictly decrease along the policy; from random points of winning cells the closed loop reaches the goal within the promised number of steps without
leaving the safe set; policy save/load round trip; the circle geometry.

## Results and limits

| Condition | Result |
| --- | --- |
| exact kinematic model | lap in 216 s, worst distance from the circle 3.6 cm: the guarantee holds |
| position noise 1 mm / 3 mm | without the fallback 2/10 and 1/10 laps complete; with it 10/10 |
| first-order lag 0.15 s | completes (6.8 cm) |
| lag 0.3 s | completes only with the fallback (8.6 cm, slightly outside the tube) |
| lag 0.6 s, identified plant behind its command filter | fails |
| Gazebo (three runs) | laps of 252-444 s, worst distance 7.59-7.77 cm of the 8 cm tube, 1-6 fallback steps |

Design lesson: with coarse cells no cell can guarantee anything, because the table adds up to one cell of uncertainty per step and the robot cannot steer
that much in one step. Only fine cells (0.5 cm), 5 deg heading cells and a 1 s step gave a guarantee that chains all 12 goals. The guarantee is for the model
it was built on: it needs a robot that follows speed commands within about 0.3 s. Gazebo measures 0.18 s latency and 0.28-0.39 s coasting, which is marginal.
