"""Reading the object list, and deciding what to do about it.

This is the only guard left watching for people when the safety policies are
switched off, so its failure directions matter more than its accuracy. Every
one of them points the same way: unreadable is blocked, unjudged is moving,
and nothing that is not positively known to be standing still is ever driven
around.
"""

import importlib.util
import json
import sys
from pathlib import Path

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    """Load a script module by path, with its siblings importable.

    Scoped and undone: leaving the scripts directory on sys.path for the
    rest of the session lets these module names shadow same-named ones
    elsewhere in the repo, which is a test failure somewhere unrelated and
    no clue at all as to why.
    """
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


cg = load("cluster_guard")
ct = load("cluster_tracking")


def summary(objects, status="OK", stamp=100.0):
    return cg.parse_summary(json.dumps(
        {"stamp": stamp, "status": status, "objects": objects}))


def obj(x, y, size=(0.5, 0.5, 1.0), motion=ct.STATIC, label="obstacle"):
    return {"class": label, "x": x, "y": y, "size": list(size),
            "points": 40, "motion": motion}


# ------------------------------------------------------------------ geometry

def test_a_box_whose_corner_reaches_the_corridor_is_a_threat():
    """The reason this uses extents and not centres. A van whose centre sits
    1.3 m to the side - well outside a 0.45 m corridor - still has its near
    flank 0.3 m from the centre line, and a guard comparing centres would
    drive along it."""
    van = obj(4.0, 1.3, size=(4.0, 2.0, 1.8), label="vehicle")
    threat = cg.nearest_threat(summary([van]), 0.45)
    assert threat is not None
    assert threat.distance_m == pytest.approx(2.0)   # near face, not centre


def test_an_object_clear_of_the_corridor_is_not_a_threat():
    assert cg.nearest_threat(summary([obj(4.0, 2.5)]), 0.45) is None


def test_the_nearest_of_several_wins():
    threat = cg.nearest_threat(
        summary([obj(8.0, 0.0), obj(3.0, 0.1), obj(5.0, -0.2)]), 0.45)
    assert threat.distance_m == pytest.approx(2.75)


def test_a_threat_preserves_direct_track_identity_and_producer_stamp():
    tracked = obj(3.0, 0.1, label="person")
    tracked["id"] = 1641

    threat = cg.nearest_threat(summary([tracked], stamp=123.4), 0.45)

    assert threat.track_id == 1641
    assert threat.observed_stamp_s == 123.4
    assert threat.directly_observed


def test_a_person_is_never_modelled_smaller_than_a_person():
    narrow = {
        "class": "person",
        "x": 2.0,
        "y": 0.0,
        "size": [0.18, 0.18, 1.7],
    }

    box = cg.object_box(narrow)

    assert box is not None
    assert box[2] >= 0.35
    assert box[3] >= 0.35


def test_person_size_floor_does_not_inflate_other_objects():
    thing = {
        "class": "obstacle",
        "x": 2.0,
        "y": 0.0,
        "size": [0.18, 0.18, 0.5],
    }

    assert cg.object_box(thing)[2:] == (0.09, 0.09)


def test_a_lateral_shift_moves_the_corridor_it_is_measured_against():
    """What the bypass probe rests on: stepping 0.6 m aside has to change
    which objects are in the way, or every offset looks equally blocked."""
    beside = obj(4.0, 0.9)
    assert cg.nearest_threat(summary([beside]), 0.45) is None
    assert cg.nearest_threat(summary([beside]), 0.45, lateral_shift_m=0.9) \
        is not None


def test_an_object_already_on_top_of_the_chair_is_here_not_behind_it():
    threat = cg.nearest_threat(summary([obj(0.2, 0.0, size=(2.0, 1.0, 1.0))]),
                               0.45)
    assert threat.distance_m == 0.0


# -------------------------------------------------------- failure directions

def test_an_unusable_summary_blocks_and_is_never_parked():
    threat = cg.nearest_threat(summary([], status="NO_CLOUD"), 0.45)
    assert threat.distance_m == cg.BLOCKED
    assert not threat.parked


@pytest.mark.parametrize("broken", [
    {"class": "obstacle", "y": 0.0, "size": [1, 1, 1]},        # no x
    {"class": "obstacle", "x": 3.0, "y": 0.0},                 # no size
    {"class": "obstacle", "x": "near", "y": 0.0, "size": [1, 1, 1]},
    {"class": "obstacle", "x": float("nan"), "y": 0.0, "size": [1, 1, 1]},
])
def test_a_malformed_object_blocks_rather_than_being_skipped(broken):
    """Skipping it means not seeing an obstacle. It also must not come out
    parked - a manoeuvre around something whose position did not parse is
    the one outcome a producer bug must not be able to cause."""
    threat = cg.nearest_threat(summary([broken]), 0.45)
    assert threat.distance_m == cg.BLOCKED
    assert not threat.parked


def test_an_unrecognised_motion_value_is_treated_as_moving():
    threat = cg.nearest_threat(summary([obj(4.0, 0.0, motion="parked?")]), 0.45)
    assert threat.motion == ct.MOVING


def test_an_object_with_no_motion_field_is_unknown_and_not_parked():
    bare = {"class": "obstacle", "x": 4.0, "y": 0.0, "size": [1, 1, 1]}
    threat = cg.nearest_threat(summary([bare]), 0.45)
    assert threat.motion == ct.UNKNOWN
    assert not threat.parked


@pytest.mark.parametrize("payload", [
    "", "not json", "[]", '{"objects": []}', '{"stamp": "now", "objects": []}',
    '{"stamp": 1.0}'])
def test_an_unparseable_summary_raises_rather_than_reading_as_empty(payload):
    with pytest.raises(ValueError):
        cg.parse_summary(payload)


def test_a_producer_that_never_spoke_is_stale_not_clear():
    assert cg.is_stale(None, 100.0)


def test_a_producer_that_went_quiet_is_stale():
    assert not cg.is_stale(100.0, 100.5)
    assert cg.is_stale(100.0, 100.0 + cg.STALE_S + 0.1)


def test_the_accumulation_window_matches_the_producer():
    """The consumer sizes its stopping envelope with this. If the producer's
    window grows and this does not, the chair brakes for where an object was
    rather than where it is."""
    producer = (SCRIPTS / "obstacle_clusters.py").read_text(encoding="utf-8")
    for line in producer.splitlines():
        if line.startswith("WINDOW_S"):
            assert float(line.split("=")[1].strip()) == cg.ACCUMULATION_S
            return
    raise AssertionError("obstacle_clusters.py no longer defines WINDOW_S")


# ------------------------------------------------------------ what to do next

def threat(distance, motion):
    return cg.Threat(distance, motion)


def decide(threat_in, blocking=True, blocked_for_s=0.0,
           stationary_bypass_ready=False):
    return cg.avoidance_decision(
        threat_in, blocking, blocked_for_s, 5.0, 3.0,
        stationary_bypass_ready=stationary_bypass_ready)


def person(distance, motion):
    return cg.Threat(distance, motion, cg.PERSON_LABEL)


# ------------------------------------------------- what actually branches

@pytest.mark.parametrize("motion", [ct.MOVING])
def test_moving_is_the_thing_that_is_waited_out(motion):
    """The one rule that separates cases, and the only one that should.

    Something that is going to step somewhere is not to be stepped around,
    whatever it is; the arc goes into where it is about to be. Neither the
    class label nor the evidence window can reach past this.
    """
    for build in (threat, person):
        assert decide(build(2.0, motion)) == cg.WAIT
        assert decide(build(2.0, motion), blocking=False) == cg.CLEAR
        assert decide(build(1.0, motion), blocked_for_s=30.0) == cg.WAIT
        assert decide(build(1.0, motion), blocked_for_s=30.0,
                      stationary_bypass_ready=True) == cg.WAIT


def test_the_class_label_no_longer_changes_the_decision():
    """The 2026-08-28 finding, as a property.

    131 of the 587 tracks seen for 20 frames or more changed class label at
    least once; one stationary body 2.61 m ahead alternated person/obstacle
    56 times. A policy branching on that label does not run one rule for
    people and another for objects - it alternates between them on the same
    body several times a second. Whatever the answer is, it has to be the
    same answer at both ends of the flicker.
    """
    for motion in (ct.STATIC, ct.MOVING, ct.UNKNOWN):
        for blocking in (True, False):
            for ready in (True, False):
                for blocked_for_s in (0.0, 30.0):
                    assert decide(
                        threat(2.0, motion), blocking, blocked_for_s,
                        stationary_bypass_ready=ready) == decide(
                        person(2.0, motion), blocking, blocked_for_s,
                        stationary_bypass_ready=ready)


# ------------------------------------------ standing in the way, anything

def test_a_confirmed_stationary_blocker_is_gone_round():
    """Person or object, once the evidence window has closed on it."""
    for build in (threat, person):
        assert decide(build(4.0, ct.STATIC), blocking=False,
                      stationary_bypass_ready=True) == cg.GO_ROUND
        assert decide(build(1.0, ct.STATIC), blocked_for_s=30.0,
                      stationary_bypass_ready=True) == cg.GO_ROUND


def test_an_unconfirmed_stationary_blocker_is_closed_on_not_passed():
    """APPROACH is the run-up, not permission.

    Before 2026-08-28 a parked OBJECT was gone round on the tracker's 1.5 s
    CONFIRM_S from 8 m, and a parked PERSON was stopped for. Both now serve
    one window, and both keep closing while it runs - which is what puts the
    commitment at about 4 m rather than at 8 m on thin evidence or at 2 m
    after standing still.
    """
    for build in (threat, person):
        assert decide(build(4.0, ct.STATIC), blocking=False) == cg.APPROACH
        assert decide(build(4.0, ct.STATIC), blocking=True) == cg.WAIT
        assert decide(build(4.0, ct.STATIC)) != cg.GO_ROUND


def test_a_parked_thing_still_far_off_is_left_alone():
    for build in (threat, person):
        assert decide(build(9.0, ct.STATIC), blocking=False) == cg.CLEAR


def test_an_untrackable_return_that_has_not_moved_for_3s_is_gone_around():
    """The raw scan has no identity, so no same-track window can ever be
    built for it and standing there is all the evidence it can offer. The
    pre-existing fallback, unchanged - and still refused to anything the
    tracker says is moving."""
    for build in (threat, person):
        assert decide(build(1.0, ct.UNKNOWN), blocked_for_s=4.0) == cg.GO_ROUND
        assert decide(build(1.0, ct.UNKNOWN), blocked_for_s=0.0) == cg.WAIT


def test_an_unjudged_return_is_not_approached_on_no_evidence():
    """UNKNOWN is not STATIC. It reaches the time rule or it waits; it never
    gets the run-up, which is for something the tracker has actually
    watched stand still."""
    for build in (threat, person):
        assert decide(build(4.0, ct.UNKNOWN), blocking=False) != cg.APPROACH


def test_a_person_who_leaves_the_corridor_clears_it():
    """Outside the corridor there is no threat; distance is not absence."""
    assert decide(None, blocking=False) == cg.CLEAR


def test_the_label_still_costs_the_object_nothing_once_confirmed():
    """There used to be an exclusion here - the parked motorcycle was gone
    round from 8 m while the person at the same range was not. There is no
    exclusion left to test, because there is no label branch: what the
    object gives up is 1.5 s of evidence against the full window, and what
    it gains is that it stops being a different code path from the person
    standing in the same place."""
    assert decide(threat(4.0, ct.STATIC), blocking=False,
                  stationary_bypass_ready=True) == cg.GO_ROUND
    assert decide(threat(4.0, ct.STATIC), blocking=False) == cg.APPROACH


@pytest.mark.parametrize("label", ["Person", " person ", "PERSON"])
def test_the_label_is_matched_the_way_producers_actually_write_it(label):
    assert cg.Threat(2.0, ct.STATIC, label).is_person


@pytest.mark.parametrize("label", ["", "obstacle", "vehicle", "personnel"])
def test_nothing_else_is_quietly_treated_as_a_person(label):
    """is_person no longer gates the decision, but it still picks the berth,
    and a label the producer did not write must not widen or narrow it by
    accident."""
    assert not cg.Threat(2.0, ct.STATIC, label).is_person


def test_a_clear_corridor_is_clear():
    assert decide(None, blocking=False) == cg.CLEAR


def test_the_chair_resumes_by_the_threat_going_away_not_by_a_timer():
    """A pedestrian crossing: blocked while they are in the corridor, clear
    the moment they are not. Nothing has to remember they were there."""
    assert decide(threat(1.5, ct.MOVING)) == cg.WAIT
    assert decide(None, blocking=False) == cg.CLEAR
