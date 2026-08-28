"""Which obstacle detector the field launcher brings up, and with what.

On 2026-08-27 two settings changed together and neither is visible from the
node graph: the geometric producer stopped subtracting the fixed map, and the
launcher halved every clustering threshold. What the chair reported afterwards
is in the 2026-08-28 blackbox, across the 00:04:02 swap between the two
producers - same route, same sensor, nineteen minutes apart:

    obstacle_clusters.py        mean 1.73 objects/frame, p99 5,  max 7
    hybrid_geometric_objects    mean 5.05 objects/frame, p99 16, max 23

with 16 % of frames after the swap carrying eight or more objects and none
before it doing so. The learned detector is not what did this: 49,666 of the
49,818 objects published after the swap were geometric and 152 were not.

These pin the rollback as a launcher default rather than a deleted feature,
so the 2026-08-27 graph can still be asked for by name.
"""

import re
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
LAUNCHER = REPO / "tools" / "perception_profile.sh"
BRINGUP = REPO / "tools" / "start_hybrid_avoidance.sh"
DRIVE = REPO / "tools" / "go_hybrid.sh"
GEOMETRIC = (REPO / "src" / "static_livox_localization" / "scripts"
             / "hybrid_geometric_objects.py")


def launcher():
    return LAUNCHER.read_text(encoding="utf-8")


def profile_block(name):
    text = launcher()
    start = text.index("%s)" % name)
    return text[start:text.index(";;", start)]


def test_the_default_profile_is_the_detector_the_chair_drove_on():
    text = launcher()
    assert 'PERCEPTION_PROFILE="${PERCEPTION_PROFILE:-legacy_geometric}"' \
        in text
    legacy = profile_block("legacy_geometric")
    assert ': "${START_POINTPILLARS:=false}"' in legacy


def test_the_producer_is_never_blinded_to_what_the_gate_can_veto():
    """safety_gate has no fixed-map subtraction and cannot get one: it works
    on raw returns in a height band with no idea what anything is. Turning
    subtraction on for the cluster producer alone does not remove a mapped
    wall from the chair's world, only from the follower's half of it, and
    the gate goes on sweeping into it.

    Measured on the 2026-08-28 16:11 drive with subtraction on: the follower
    saw two objects, both outside the band; the gate held 9,838 obstacle
    points, zone_points 0, and refused OBSTACLE_SWEEP on 17 of 20 samples.
    The chair reached waypoint 42 and stopped there.

    Neither profile may ship with it on. It stays a parameter for bag replay,
    where nothing is driving.
    """
    for name in ("legacy_geometric", "hybrid_experimental"):
        assert ': "${GEOMETRIC_FIXED_MAP_SUBTRACTION:=false}"' in             profile_block(name), name


def test_the_legacy_profile_restores_the_field_tested_thresholds():
    """1/5/80 is what ran on 2026-08-27 and what splits one object into
    several; 2/8/40 are obstacle_clusters' own numbers."""
    legacy = profile_block("legacy_geometric")
    assert ': "${GEOMETRIC_MIN_CELL_POINTS:=2}"' in legacy
    assert ': "${GEOMETRIC_MIN_CLUSTER_POINTS:=8}"' in legacy
    assert ': "${GEOMETRIC_MAX_CLUSTERS:=40}"' in legacy


def test_the_2026_08_27_graph_is_still_reachable_by_name():
    """Rolled back, not deleted. A bag replay has to be able to reproduce
    the counts above, and the reason the subtraction was turned off - a
    person against a mapped wall - has not stopped being real."""
    experimental = profile_block("hybrid_experimental")
    assert ': "${START_POINTPILLARS:=true}"' in experimental
    assert ': "${GEOMETRIC_MIN_CELL_POINTS:=1}"' in experimental
    assert ': "${GEOMETRIC_MIN_CLUSTER_POINTS:=5}"' in experimental
    assert ': "${GEOMETRIC_MAX_CLUSTERS:=80}"' in experimental


def test_an_unknown_profile_is_refused_rather_than_guessed():
    text = launcher()
    assert "PERCEPTION_PROFILE must be legacy_geometric or " \
        "hybrid_experimental" in text


def test_the_subtraction_choice_reaches_the_node():
    assert '_fixed_map_subtraction:="$GEOMETRIC_FIXED_MAP_SUBTRACTION"' \
        in BRINGUP.read_text(encoding="utf-8")
    assert '_bool_param("fixed_map_subtraction", True)' \
        in GEOMETRIC.read_text(encoding="utf-8")


def test_the_node_keeps_the_map_unless_told_otherwise():
    """The default has to be safe when the launcher is bypassed - rosrun by
    hand, a launch file, a replay harness. Reading it out of the source
    rather than running it, because this node imports rospy."""
    text = GEOMETRIC.read_text(encoding="utf-8")
    default = re.search(
        r'_bool_param\("fixed_map_subtraction",\s*(\w+)\)', text)
    assert default and default.group(1) == "True"
    # KeepAllGeometry must be reachable only from the false branch.
    assert text.count("legacy.FixedMapFilter = KeepAllGeometry") == 1
    branch = text[text.index("if _bool_param"):]
    assert branch.index("else:") < branch.index(
        "legacy.FixedMapFilter = KeepAllGeometry")


def test_the_warning_only_fires_when_the_map_is_actually_ignored():
    """It fired on every start-up including runs that did subtract, which is
    how a warning stops being read."""
    text = GEOMETRIC.read_text(encoding="utf-8")
    assert "if legacy.FixedMapFilter is KeepAllGeometry:" in text
    warning = text.index("fixed-map subtraction is OFF")
    guard = text.index("if legacy.FixedMapFilter is KeepAllGeometry:")
    assert guard < warning


def test_the_bring_up_and_the_drive_read_the_same_profile():
    """They did not, and the chair stopped at waypoint 42 because of it.

    The profile lived in start_hybrid_avoidance.sh alone, so the bring-up
    honoured PERCEPTION_PROFILE=legacy_geometric and started without the
    learned detector - while go_hybrid.sh defaulted START_POINTPILLARS to
    true on its own, demanded /rtx_pointpillars and
    ~/.config/unicon/pointpillars.env, found neither, and printed REFUSING
    TO START. The follower reported DRIVING the whole time; only /wheel_cmd
    said STOP.

    One sourced copy, and the source has to come BEFORE the local default or
    the default wins.
    """
    for path in (BRINGUP, DRIVE):
        text = path.read_text(encoding="utf-8")
        assert '. "$SCRIPT_DIR/perception_profile.sh"' in text, path.name
    drive = DRIVE.read_text(encoding="utf-8")
    assert drive.index('. "$SCRIPT_DIR/perception_profile.sh"') <         drive.index('START_POINTPILLARS="${START_POINTPILLARS:-true}"')


def test_neither_entry_point_keeps_its_own_copy_of_the_profile():
    """A second copy drifts; that is what this whole file is about."""
    for path in (BRINGUP, DRIVE):
        text = path.read_text(encoding="utf-8")
        assert "legacy_geometric)" not in text, (
            "%s re-implements the profile instead of sourcing it" % path.name)
