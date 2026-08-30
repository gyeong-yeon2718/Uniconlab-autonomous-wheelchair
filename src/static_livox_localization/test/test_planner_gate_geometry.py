"""The planner and the raw gate must refuse the same arcs.

A planner that proposes what the gate forbids does not produce a near miss;
it produces a chair that stands still. Refusing does not move it, so the next
cycle proposes the same arc and the cycle after that repeats it. dwa_core has
carried a comment saying so since the 2026-08-23 motorcycle deadlock, and the
number underneath it was still a disc.

Measured cost of the gap, from person_bypass_20260827_235916.bag:
REQUESTED_PATH_COLLISION was returned 139 times, 130 of them inside a single
ten-second window at 00:11:44 in which the follower asked for +0.500 rad/s -
its maximum - on every cycle, the semantic layer was not blocking, and the
chair travelled a mean 0.023 m/s against a planned 0.325.
"""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            name, SCRIPTS / ("%s.py" % name))
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


ms = load("motion_safety")
core = load("dwa_core")


def arc_poses(v, w, horizon_s, steps=200):
    """The same constant-curvature integration both sides of this use."""
    out = []
    for elapsed in np.linspace(0.0, horizon_s, steps + 1):
        yaw = w * elapsed
        if abs(w) < 1e-9:
            x, y = v * elapsed, 0.0
        else:
            x = (v / w) * math.sin(yaw)
            y = (v / w) * (1.0 - math.cos(yaw))
        out.append((x, y, yaw))
    return np.asarray([out], dtype=float)


def test_the_footprint_is_a_rectangle_and_not_the_disc_it_replaced():
    """0.79 m at the corner, 0.65 ahead, 0.45 to the side - and the disc that
    stood in for all three was 0.50."""
    assert ms.footprint_circumscribed_radius() == pytest.approx(
        math.hypot(0.65, 0.45))
    ahead = ms.footprint_exterior_distance(
        np.array([[[0.0, 0.0, 0.0]]]), np.array([[1.00, 0.0]]))
    beside = ms.footprint_exterior_distance(
        np.array([[[0.0, 0.0, 0.0]]]), np.array([[0.0, 1.00]]))
    assert ahead[0] == pytest.approx(1.00 - 0.65)
    assert beside[0] == pytest.approx(1.00 - 0.45)
    # The corner is the part a disc of 0.50 never covered: a point the old
    # test called 0.29 m clear is inside the rectangle the gate vetoes.
    corner = ms.footprint_exterior_distance(
        np.array([[[0.0, 0.0, 0.0]]]), np.array([[0.60, 0.40]]))
    assert corner[0] == 0.0
    assert math.hypot(0.60, 0.40) > 0.50


def test_a_point_inside_the_swept_body_reads_as_zero_clearance():
    poses = arc_poses(0.35, 0.5, 2.2)
    on_the_arc = poses[0, len(poses[0]) // 2, :2]
    assert ms.footprint_exterior_distance(
        poses, np.asarray([on_the_arc]))[0] == 0.0


@pytest.mark.parametrize("w", [-0.5, -0.25, 0.0, 0.25, 0.5])
@pytest.mark.parametrize("lateral", [0.0, 0.2, -0.35, 0.5])
def test_the_planner_never_calls_clear_what_the_gate_calls_a_collision(
        w, lateral):
    """The property the whole change exists for, over the yaw range the
    follower actually samples and the offsets a person stands at.

    Both sides integrate the same arc and pad the same rectangle, so the
    planner's clearance may be conservative against the gate but never
    optimistic - and it is optimism that deadlocks the chair.
    """
    v, horizon_s = 0.35, 2.2
    poses = arc_poses(v, w, horizon_s)
    for forward in np.arange(0.4, 2.6, 0.1):
        point = np.asarray([[forward, lateral]])
        collides = ms.swept_footprint_collision(
            np.repeat(point, 5, axis=0),
            linear_speed_mps=v, angular_speed_rps=w, horizon_s=horizon_s,
            front_m=ms.FOOTPRINT_FRONT_M, rear_m=ms.FOOTPRINT_REAR_M,
            half_width_m=ms.FOOTPRINT_HALF_WIDTH_M,
            margin_m=ms.SWEEP_MARGIN_M)
        clearance = ms.footprint_exterior_distance(poses, point)[0]
        if collides:
            assert clearance == 0.0, (
                "planner scored %.3f m clear where the gate collides "
                "(forward %.2f, lateral %.2f, w %+.2f)"
                % (clearance, forward, lateral, w))


def test_the_old_disc_would_have_called_the_deadlock_arc_clear():
    """The 2026-08-28 00:11 geometry, as the two shapes saw it.

    A person on the centreline 1.05 m ahead, the follower at its maximum yaw.
    The disc measured from the rollout centre and found room; the rectangle
    the gate enforces is already through them. Nothing about the situation
    changed between those two answers except which shape was asked.
    """
    point = np.asarray([[1.05, 0.03]])
    poses = arc_poses(0.35, 0.5, 2.2)
    centres = poses[0, :, :2]
    disc = float(np.linalg.norm(centres - point, axis=1).min())
    rectangle = ms.footprint_exterior_distance(poses, point)[0]

    assert disc >= core.LEGACY_OBSTACLE_DISC_M, (
        "the historical refusal is not reproduced: the disc rejected this too")
    assert rectangle == 0.0
    assert ms.swept_footprint_collision(
        np.repeat(point, 5, axis=0),
        linear_speed_mps=0.35, angular_speed_rps=0.5, horizon_s=2.2,
        front_m=ms.FOOTPRINT_FRONT_M, rear_m=ms.FOOTPRINT_REAR_M,
        half_width_m=ms.FOOTPRINT_HALF_WIDTH_M,
        margin_m=ms.SWEEP_MARGIN_M)


def test_the_side_clearance_the_disc_kept_is_not_quietly_given_up():
    """OBSTACLE_FLOOR_M changed meaning, so it has to be checked against the
    behaviour it replaced rather than against its own old number: 0.45 m of
    rectangle plus the floor must still be the 0.50 m the disc kept."""
    assert (ms.FOOTPRINT_HALF_WIDTH_M + ms.SWEEP_MARGIN_M
            + core.OBSTACLE_FLOOR_M) == pytest.approx(
                core.LEGACY_OBSTACLE_DISC_M)


def clearance(poses, points, floor_m):
    """The staged answer, given the centre distances a planner already has."""
    centres = poses[:, :, :2]
    centre = np.linalg.norm(
        centres[:, :, None, :] - np.asarray(points)[None, None, :, :],
        axis=3).min(axis=2)
    return ms.footprint_clearance(poses, points, centre, floor_m)


@pytest.mark.parametrize("floor_m", [0.0, 0.05, 0.35])
def test_the_staged_clearance_never_overstates_the_exact_one(floor_m):
    """The bounds exist to skip work, not to change verdicts.

    Where a bound decides, the value returned is that bound - a lower one -
    so it may understate room but must never claim more of it than the
    oriented test would, or the planner starts proposing what the gate
    refuses again by a different route.
    """
    rng = np.random.default_rng(11)
    for _ in range(40):
        w = float(rng.uniform(-0.5, 0.5))
        poses = arc_poses(0.35, w, 2.2, steps=40)
        points = rng.uniform(-1.0, 1.0, (12, 2)) + np.array([1.4, 0.0])
        staged = clearance(poses, points, floor_m)[0]
        exact = ms.footprint_exterior_distance(poses, points)[0]
        assert staged <= exact + 1e-9
        # And the verdict itself is identical, which is the point.
        assert (staged >= floor_m) == (exact >= floor_m)


def test_a_dense_scan_is_decided_by_bounds_not_by_the_quadratic_test():
    """test_obstacle_preview budgets the whole plan() call; this pins the
    reason it fits. 20,000 returns on top of the chair leave no candidate
    needing the oriented test - every one of them fails the cheap bound."""
    rng = np.random.default_rng(3)
    points = rng.uniform(-0.4, 0.4, (20000, 2)) + np.array([0.8, 0.0])
    poses = arc_poses(0.35, 0.25, 2.2, steps=65)
    centre = np.full((1, poses.shape[1]), 0.30)
    assert ms.footprint_clearance(poses, points, centre, 0.05)[0] == 0.0


# ------------------------------- the gate's OTHER veto, and the planner's

sg_consts = dict(HALF_WIDTH_M=0.5, CORRIDOR_MIN_RANGE_M=0.35,
                 FORWARD_FOV_HALF_DEG=50.0, FORWARD_CHECK_EXTRA_M=0.6,
                 GEOMETRY_MARGIN_M=0.9, ACCUMULATION_WINDOW_S=1.0,
                 PIPELINE_BUDGET_S=0.2, MIN_BRAKE_DECEL_MPS2=0.5,
                 MIN_YAW_DECEL_RPS2=0.5)


def gate_would_block(points, speed):
    """safety_gate's OBSTACLE test, written out from its own constants."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    envelope = ms.stopping_envelope(
        measured_speed_mps=speed, requested_speed_mps=speed,
        measured_yaw_rate_rps=0.0, requested_yaw_rate_rps=0.0,
        cloud_age_s=0.0,
        accumulation_s=sg_consts["ACCUMULATION_WINDOW_S"],
        pipeline_s=sg_consts["PIPELINE_BUDGET_S"],
        min_linear_decel_mps2=sg_consts["MIN_BRAKE_DECEL_MPS2"],
        min_angular_decel_rps2=sg_consts["MIN_YAW_DECEL_RPS2"],
        geometry_margin_m=sg_consts["GEOMETRY_MARGIN_M"])
    azimuth = np.abs(np.degrees(np.arctan2(pts[:, 1], pts[:, 0])))
    zone = pts[(pts[:, 0] > sg_consts["CORRIDOR_MIN_RANGE_M"]) &
               (pts[:, 0] < envelope.distance_m + sg_consts["FORWARD_CHECK_EXTRA_M"]) &
               (azimuth < sg_consts["FORWARD_FOV_HALF_DEG"]) &
               (np.abs(pts[:, 1]) < sg_consts["HALF_WIDTH_M"])]
    if len(zone) < 5:
        return False
    return float(np.percentile(zone[:, 0], 5)) < envelope.distance_m


@pytest.mark.parametrize("forward", [0.6, 0.9, 1.2, 1.5, 1.8, 2.2, 2.6, 3.2])
@pytest.mark.parametrize("lateral", [0.0, 0.2, -0.35])
def test_the_speed_the_planner_picks_is_one_the_gate_accepts(forward, lateral):
    """The 2026-08-30 finding, as the property that prevents it.

    safety_gate refuses on two independent tests. The planner has cleared the
    swept rectangle since 2026-08-28; it did not model the straight forward
    corridor, whose length grows with the speed being ASKED for. So it asked
    for 0.80 m/s at an obstacle it could have passed at 0.45 and was vetoed -
    OBSTACLE on 5.0 % of that drive's gate samples against the sweep's 1.0 %,
    including one unbroken 21.5 s block with no wheel turn.

    Whatever speed the cap returns, the gate must accept it. A cap of zero
    means no sampled speed clears the corridor, and nothing is claimed.
    """
    points = [(forward, lateral)] * 8
    cap = core.gate_corridor_speed_cap(points)
    if cap > 0.0:
        assert not gate_would_block(points, cap), (
            "planner would ask %.2f m/s at %.1f m; the gate refuses it"
            % (cap, forward))


def test_the_cap_is_the_fastest_the_gate_allows_not_merely_a_safe_one():
    """A cap that always returned the crawl would pass the test above and be
    useless. It has to be the fastest sampled speed the gate accepts."""
    for forward in (1.5, 1.8, 2.2, 2.6, 3.2):
        points = [(forward, 0.0)] * 8
        cap = core.gate_corridor_speed_cap(points)
        faster = [s for s in (0.8, 0.6, 0.45, 0.35, 0.2, 0.1) if s > cap]
        for speed in faster:
            assert gate_would_block(points, speed), (
                "%.2f m/s at %.1f m is allowed but the cap said %.2f"
                % (speed, forward, cap))


def test_an_empty_corridor_does_not_slow_the_chair():
    assert core.gate_corridor_speed_cap([]) == 0.8
    assert core.gate_corridor_speed_cap([(2.0, 1.2)] * 8) == 0.8
    assert core.gate_corridor_speed_cap([(0.2, 0.0)] * 8) == 0.8
