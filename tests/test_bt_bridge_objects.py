"""What survives the Bluetooth link when the pavement is busy.

/perception/objects_summary is written for the follower and the black box:
forty clusters, each carrying up to sixty-four numbers of lateral profile,
five times a second. That is a quarter of a megabyte per second, and the
phone is on an RFCOMM serial link at 2 Hz. The reduction between the two is
therefore not a nicety, and it has two properties the view depends on:

* what a crowded scene drops is the FAR half. Dropping the near half would
  hide the thing the chair is about to drive into, while still showing a
  view full of objects -- worse than showing nothing.
* a parked object publishes no path. Its centroid still slides by
  centimetres as more of it comes into view, and forwarding that would draw
  a scribble under every bollard, which reads as a crowd walking.

The paths themselves are made here rather than at the producer. This branch
drives, and obstacle_clusters.py is one of the files it drives with; the app
does not get to change it. So the bridge remembers where each object has
been -- in the map frame, through the pose, so the chair's own motion
cancels -- and the driving stack is left exactly as it was found.
"""

import importlib.util
import json
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts" / "ros1_bluetooth_bridge.py"


def load():
    """The bridge imports its ROS bindings behind a try/except, so this is
    the deployed file itself, not a copy of its arithmetic."""
    spec = importlib.util.spec_from_file_location("bt_bridge", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load()


def walking_trail(x, y, samples=16, step=0.3):
    """A path that arrives at (x, y), oldest sample first."""
    return [[round(x - step * k, 2), round(y - step * k, 2)]
            for k in range(samples - 1, -1, -1)]


def cluster(index, x, y, moving=True, band="inside", trail=True):
    """One object shaped like obstacle_clusters.py publishes it.

    ``trail`` exists because a producer MAY publish one -- the app branch's
    does -- and the bridge has to prefer it when it is there. On this branch
    nothing does, which is what every reconstruction test below is about.
    """
    obj = {
        "class": "person" if moving else "obstacle",
        "raw_class": "person",
        "band_relation": band,
        "band_inside_fraction": 1.0,
        "x": x,
        "y": y,
        "size": [0.62, 0.55, 1.71],
        "profile": [[0.1, 0.2]] * 64,
        "points": 312,
        "id": index,
        "motion": "moving" if moving else "static",
        "speed_mps": 1.24 if moving else 0.0,
        "age_s": 3.0,
    }
    if trail:
        obj["trail"] = (walking_trail(x, y) if moving
                        else [[x + 0.01 * (k % 3), y - 0.01 * (k % 2)]
                              for k in range(16)])
    return obj


def make_link(state=None):
    """A RosLink with only what the objects callback touches.

    ``trails`` is part of that: the reconstruction is the bridge's own
    working memory, and a link without it is not the object the field runs.
    """
    link = bridge.RosLink.__new__(bridge.RosLink)
    link.state = state if state is not None else bridge.BridgeState()
    link.trails = bridge.TrailMemory()
    return link


def summary(objects, status="OK", frame="lidar"):
    blob = {
        "stamp": 1.0,
        "status": status,
        "band_status": "OK",
        "bloom_filtered": 0,
        "counts": {"person": len(objects)},
        "objects": objects,
    }
    if frame is not None:
        blob["frame"] = frame
    return types.SimpleNamespace(data=json.dumps(blob))


def frame_for(objects, status="OK", frame="lidar"):
    """One /perception/objects_summary through the bridge and out again."""
    link = make_link()
    link._objects_cb(summary(objects, status, frame))
    return link.state.snapshot(True, 5.0, True)


def test_a_crowded_pavement_keeps_the_near_objects():
    objects = [cluster(i, 2.0 + i * 1.3, 0.4) for i in range(40)]
    frame = frame_for(list(reversed(objects)))    # far ones offered first

    assert len(frame["objects"]) == bridge.OBJECT_MAX_COUNT
    assert [o["id"] for o in frame["objects"]] == list(range(10)), \
        "the link dropped the near objects and kept the far ones"


def test_a_parked_object_publishes_no_path():
    frame = frame_for([cluster(1, 3.0, 0.2, moving=False)])

    assert "trail" not in frame["objects"][0], \
        "centroid jitter went out as a path"


def test_a_walkers_path_still_ends_on_the_walker():
    frame = frame_for([cluster(1, 3.0, 0.2)])
    drawn = frame["objects"][0]

    assert len(drawn["trail"]) <= bridge.OBJECT_TRAIL_MAX_POINTS + 1
    assert drawn["trail"][-1] == [drawn["x"], drawn["y"]], \
        "thinning pulled the path off the object it belongs to"


def test_the_frame_says_which_axes_the_objects_are_on():
    """pose_x/pose_y are map frame and these are not. Nothing else in the
    frame distinguishes them, and drawn on the wrong axes an object beside
    the chair lands a hundred metres away."""
    frame = frame_for([cluster(1, 3.0, 0.2)])

    assert frame["objects_frame"] == "lidar"


def test_the_frame_is_the_producers_word_and_not_this_bridges_guess():
    """Two nodes publish this topic and they do not agree: obstacle_clusters
    says "lidar", hybrid_object_fusion says "chair_centre", and those origins
    are about half a metre apart."""
    frame = frame_for([cluster(1, 3.0, 0.2)], frame="chair_centre")

    assert frame["objects_frame"] == "chair_centre"


def test_a_producer_that_names_no_frame_is_not_given_one():
    """Unknown axes must read as unknown. A default here is a guess that
    looks exactly like a measurement."""
    frame = frame_for([cluster(1, 3.0, 0.2)], frame=None)

    assert frame["objects_frame"] is None
    assert "objects_frame" in frame["unavailable"]


def test_the_link_budget_is_not_spent_on_objects():
    """Worst case: ten objects, every one of them walking. The producer's
    own message for the same scene is ~48 kB."""
    objects = [cluster(i, 2.0 + i * 1.3, 0.4) for i in range(40)]
    frame = frame_for(objects)
    frame["type"] = "telemetry"

    size = len(json.dumps(frame, ensure_ascii=False).encode("utf-8"))
    assert size < 6000, "a telemetry frame grew to %d bytes" % size


def test_a_perception_node_that_says_nothing_useful_sends_no_objects():
    frame = frame_for([], status="NO_CLOUD")

    assert frame["objects"] == []
    assert frame["objects_status"] == "NO_CLOUD"


def test_geometry_the_producer_did_not_publish_is_not_invented():
    """A malformed object must not become a box at the origin."""
    frame = frame_for([{"id": 4, "class": "person", "motion": "moving"}])

    assert frame["objects"] == []


def paving(index, x, y, band="outside", height=0.10, points=6):
    """A speck of ground: 0.21 x 0.26 x 0.10 m from six returns.

    Not invented. This is the shape a quarter of every live message was on
    2026-08-27, and drawing it is how the view fills with boxes that mean
    nothing.
    """
    return {
        "class": "outside_band", "raw_class": "obstacle",
        "band_relation": band, "band_inside_fraction": 0.0,
        "x": x, "y": y, "size": [0.21, 0.26, height],
        "points": points, "id": index, "motion": "static",
        "speed_mps": 0.0, "age_s": 3.0, "trail": [],
    }


def test_the_ground_is_not_drawn_as_objects():
    frame = frame_for([paving(1, 6.0, 3.0), paving(2, 6.1, -3.0),
                       cluster(3, 3.0, 0.2)])

    assert [o["id"] for o in frame["objects"]] == [3]
    assert frame["objects_hidden"] == 2


def test_a_speck_in_the_corridor_is_never_hidden():
    """The filter exists to clear the view, not to decide what matters. What
    the chair could hit is not the display's call to suppress."""
    frame = frame_for([paving(1, 2.0, 0.0, band="inside")])

    assert [o["id"] for o in frame["objects"]] == [1]
    assert frame["objects_hidden"] == 0


def test_ground_cannot_crowd_a_real_object_off_the_list():
    """Ten specks are nearer than the person behind them, and the cut is
    nearest-first: filtering has to happen before it, or the one object worth
    seeing is the one that does not fit."""
    objects = [paving(i, 1.0 + i * 0.1, 2.0) for i in range(12)]
    objects.append(cluster(99, 6.0, 0.1))
    frame = frame_for(objects)

    assert 99 in [o["id"] for o in frame["objects"]]


def test_an_object_of_unknown_height_is_kept():
    """Missing evidence is not evidence. A producer that publishes no height
    has not said the thing is flat."""
    speck = paving(1, 6.0, 3.0, points=400)
    speck["size"] = [0.21, 0.26]              # footprint only, no height
    frame = frame_for([speck])

    assert [o["id"] for o in frame["objects"]] == [1]


def test_what_is_left_out_is_counted():
    """A trimmed view that does not say it is trimmed reads as a quiet
    pavement, which is the one thing it must never imply."""
    objects = [cluster(i, 2.0 + i * 0.8, 0.3) for i in range(14)]
    frame = frame_for(objects)

    assert len(frame["objects"]) == bridge.OBJECT_MAX_COUNT
    assert frame["objects_hidden"] == 4


@pytest.mark.parametrize("field", ["id", "class", "band_relation", "x", "y",
                                   "size", "motion", "speed_mps"])
def test_the_view_gets_every_field_it_draws_with(field):
    frame = frame_for([cluster(1, 3.0, 0.2)])

    assert field in frame["objects"][0], "the link stopped forwarding %s" % field


# --------------------------------------------------------------------------
# Paths the bridge makes for itself.
#
# The producer on this branch publishes none, and it is not going to be
# edited to: it is a driving file. Everything below is therefore what the
# app actually receives in the field.
# --------------------------------------------------------------------------

def place(link, pose, objects, now):
    """One frame at a stated pose and moment, without waiting for either."""
    with link.state.lock:
        link.state.pose_x, link.state.pose_y, link.state.pose_yaw_deg = pose
        link.state.pose_stamp = now
    original = bridge.time.time
    bridge.time.time = lambda: now
    try:
        link._objects_cb(summary(objects))
        # Snapshot inside the same frozen moment: the freshness TTL is
        # measured against the wall clock, and a reading stamped at 1000.0
        # read back in 2026 is correctly reported as long gone.
        return link.state.snapshot(True, 5.0, True)
    finally:
        bridge.time.time = original


def test_a_walker_gets_a_path_the_producer_never_sent():
    """The whole point: paths without touching obstacle_clusters.py."""
    link = make_link()
    frame = None
    for step in range(8):
        # Chair stationary at the origin, someone crossing left to right.
        frame = place(link, (0.0, 0.0, 0.0),
                      [cluster(11, 4.0, 1.4 - step * 0.35, trail=False)],
                      1000.0 + step * 0.2)

    drawn = frame["objects"][0]
    assert "trail" in drawn, "the bridge kept no history for a moving object"
    assert drawn["trail"][-1] == [drawn["x"], drawn["y"]], \
        "the path does not end on the object it belongs to"
    assert len(drawn["trail"]) >= 3


def test_a_moving_chair_does_not_draw_paths_under_parked_objects():
    """The failure this design exists to avoid.

    Objects arrive chair-relative, so a bollard's x slides towards the chair
    every frame it drives. Remembered as it arrives, that draws the chair's
    own journey under everything on the pavement, and a viewer reads a
    parked bollard as something walking straight at them.
    """
    link = make_link()
    frame = None
    for step in range(8):
        # Chair driving forward at 0.5 m/s; the object is parked in the map,
        # so it closes at exactly the same rate in the chair's own frame.
        travelled = step * 0.1
        frame = place(link, (travelled, 0.0, 0.0),
                      [dict(cluster(12, 8.0 - travelled, -1.2, trail=False),
                            motion="unknown")],
                      2000.0 + step * 0.2)

    assert "trail" not in frame["objects"][0], \
        "the chair's own motion was drawn as the object's"


def test_a_path_measured_about_a_stale_pose_is_not_drawn():
    """A localization drop-out must show plain dots, not bent paths. The
    pose is the transform at both ends; without a fresh one there is no
    frame to state a path in, and a wrong path outlives the outage."""
    link = make_link()
    for step in range(6):
        place(link, (0.0, 0.0, 0.0),
              [cluster(11, 4.0, 1.2 - step * 0.3, trail=False)],
              3000.0 + step * 0.2)

    now = 3001.2
    with link.state.lock:
        link.state.pose_stamp = now - 3.0          # localizer went quiet
    original = bridge.time.time
    bridge.time.time = lambda: now
    try:
        link._objects_cb(summary([cluster(11, 4.0, -0.6, trail=False)]))
        frame = link.state.snapshot(True, 5.0, True)
    finally:
        bridge.time.time = original

    assert "trail" not in frame["objects"][0]


def test_a_reused_id_does_not_join_two_objects_with_a_line():
    """Ids are track ids where the producer has tracks and list indices
    where it does not, and an index goes to whatever sorts into that slot
    next. Two objects a pavement apart must not be drawn as one that
    teleported."""
    link = make_link()
    for step in range(6):
        place(link, (0.0, 0.0, 0.0),
              [cluster(3, 4.0, 1.0 - step * 0.3, trail=False)],
              4000.0 + step * 0.2)
    frame = place(link, (0.0, 0.0, 0.0),
                  [cluster(3, 4.2, -6.0, trail=False)], 4001.2)

    drawn = frame["objects"][0]
    assert "trail" not in drawn, "one id, two objects, and a line between them"


def test_the_producers_own_path_wins_when_it_publishes_one():
    """hybrid_object_fusion.py and the app branch's producer both do. Theirs
    has the tracker's full history and a frame stated at the source, so it
    is better than anything reconstructed here."""
    link = make_link()
    frame = place(link, (0.0, 0.0, 0.0), [cluster(11, 3.0, 0.2)], 5000.0)

    assert frame["objects"][0]["trail"][-1] == [3.0, 0.2]


def test_memory_of_objects_that_have_gone_is_let_go():
    link = make_link()
    for step in range(40):
        place(link, (0.0, 0.0, 0.0),
              [cluster(100 + step, 4.0, 1.0, trail=False)],
              6000.0 + step * 0.2)

    assert len(link.trails._tracks) <= bridge.OBJECT_TRAIL_MAX_TRACKS


# --------------------------------------------------------------------------
# The label the app prints.
# --------------------------------------------------------------------------

def wall(index=15, x=7.5, y=-3.2, size=(9.4, 0.35, 1.6)):
    """A campus wall as classify() sees it: long, thin, and 1.6 m tall --
    which is over the vehicle floor and inside the vehicle height window."""
    return {
        "class": "vehicle", "raw_class": "vehicle",
        "band_relation": "outside", "band_inside_fraction": 0.0,
        "x": x, "y": y, "size": list(size), "points": 1200,
        "id": index, "motion": "static", "speed_mps": 0.0, "age_s": 30.0,
    }


def car(index=16):
    return dict(wall(index, x=6.0, y=2.8, size=(4.5, 1.8, 1.5)))


def test_a_wall_is_not_drawn_as_a_parked_car():
    frame = frame_for([wall()])

    assert frame["objects"][0]["class"] == "obstacle"
    assert frame["objects"][0]["raw_class"] == "vehicle", \
        "the producer's own word was thrown away rather than kept beside it"


def test_a_car_is_still_a_car():
    frame = frame_for([car()])

    assert frame["objects"][0]["class"] == "vehicle"


def test_the_count_line_agrees_with_what_is_drawn():
    """A dashboard that says 차량 1 above a view drawing one obstacle gets
    believed at the wrong moment."""
    frame = frame_for([wall(), car(), cluster(1, 3.0, 0.2)])

    counts = frame["objects_counts"]
    assert counts.get("vehicle") == 1
    assert counts.get("obstacle") == 1


def test_a_person_is_never_relabelled():
    """cluster_guard reads "person" and only "person". Nothing about a
    display may be able to reach that word."""
    frame = frame_for([cluster(1, 3.0, 0.2)])

    assert frame["objects"][0]["class"] == "person"
    assert bridge.display_class({"class": "person",
                                 "size": [9.9, 0.2, 1.6]}) == "person"
