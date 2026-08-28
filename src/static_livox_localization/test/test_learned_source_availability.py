"""What a learned detector that skips a frame is allowed to do to the chair.

TOO_FEW_POINTS publishes nothing, so a run of sparse scans ages the learned
summary past learned_max_age_s and the fusion reports LEARNED_STALE. Under
REQUIRE_LEARNED that is mode "blocked", which the semantic supervisor reads
as PERCEPTION_UNUSABLE and turns into a stop.

The threshold that decides how often that happens lived only in
wheelchair_deploys/main-b2c36ae-20260827/pointpillars_rtx2060_min350.yaml -
one directory, no commit - so every tree checked out since went back to the
800 in this repo and the stale verdict came back. These pin the value and
the shape of the failure so the next checkout cannot lose it again.
"""

import importlib.util
import json
import re
import sys
from pathlib import Path

import pytest


PACKAGE = Path(__file__).parents[1]
SCRIPTS = PACKAGE / "scripts"
CONFIG = PACKAGE / "config" / "pointpillars_rtx2060.yaml"
NODE = PACKAGE / "src" / "rtx_pointpillars_node.cpp"


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


hp = load("hybrid_perception")

FIELD_MINIMUM_POINTS = 350


def config_minimum_points():
    match = re.search(r"^minimum_points:\s*(\d+)", CONFIG.read_text(
        encoding="utf-8"), re.MULTILINE)
    assert match, "minimum_points missing from the shipped config"
    return int(match.group(1))


def test_the_shipped_threshold_is_the_one_the_field_settled_on():
    assert config_minimum_points() == FIELD_MINIMUM_POINTS


def test_the_node_default_agrees_with_the_shipped_config():
    """They disagreed, and the disagreement is why a value that existed in
    one deploy directory could look like the default everywhere else."""
    text = NODE.read_text(encoding="utf-8")
    declared = re.search(r"int minimum_points_ = (\d+);", text)
    param = re.search(
        r'param\("minimum_points", minimum_points_, (\d+)\)', text)
    assert declared and param
    assert int(declared.group(1)) == FIELD_MINIMUM_POINTS
    assert int(param.group(1)) == FIELD_MINIMUM_POINTS


def summary(stamp, frame="lidar", objects=()):
    return json.dumps({"status": "OK", "stamp": stamp, "frame": frame,
                       "objects": list(objects)})


def fuse(now_s, learned_stamp, require_learned):
    return hp.fuse_summaries(
        summary(now_s), summary(learned_stamp), now_s,
        require_learned=require_learned)


def test_a_stale_learned_source_blocks_only_when_it_is_required():
    """The whole reason the threshold matters. Required, one second of
    skipped inference is a stopped chair; not required, it is geometry
    only - which is what the chair drove on before the detector existed."""
    fresh = fuse(100.0, 100.0, require_learned=True)
    assert fresh["status"] == "OK"
    assert fresh["sources"]["learned"] == "OK"

    blocked = fuse(100.0, 98.0, require_learned=True)
    assert blocked["status"] == "LEARNED_STALE"
    assert blocked["mode"] == "blocked"

    degraded = fuse(100.0, 98.0, require_learned=False)
    assert degraded["status"] == "OK"
    assert degraded["sources"]["learned"] == "STALE"
    assert degraded["mode"] != "blocked"


def test_the_legacy_profile_cannot_be_stopped_by_a_stale_detector():
    """PERCEPTION_PROFILE=legacy_geometric does not start PointPillars, so
    REQUIRE_LEARNED follows it to false and this failure cannot reach the
    wheels at all. Pinned because the two settings are set in different
    places and only their combination is safe."""
    launcher = (PACKAGE.parents[1] / "tools"
                / "start_hybrid_avoidance.sh").read_text(encoding="utf-8")
    legacy = launcher[launcher.index("legacy_geometric)"):]
    legacy = legacy[:legacy.index(";;")]
    assert ': "${START_POINTPILLARS:=false}"' in legacy
    assert 'REQUIRE_LEARNED="$START_POINTPILLARS"' in launcher


@pytest.mark.parametrize("learned_stamp,expected", [
    (99.8, "OK"),
    (99.7, "OK"),
    (99.5, "STAMP_SKEW"),
    (99.2, "STAMP_SKEW"),
    (98.9, "STALE"),
    (98.0, "STALE"),
    (100.5, "STAMP_FUTURE"),
])
def test_the_real_freshness_budget_is_the_skew_not_the_age(
        learned_stamp, expected):
    """learned_max_age_s is 1.0 s, and it is not the binding number.

    maximum_skew_s is 0.40 s and it is measured against the GEOMETRIC
    stamp, so a learned summary more than 0.40 s behind the geometry is
    refused as STAMP_SKEW long before it is old enough to be STALE. The
    detector therefore has 0.40 s of margin, not 1.0 - two skipped frames
    at 5 Hz - and both verdicts block identically under REQUIRE_LEARNED.

    The 2026-08-27 drive shows both: 12 LEARNED_STALE and 6
    LEARNED_STAMP_SKEW out of 9,874 fused summaries, and they start within
    the same second (00:05:54).
    """
    result = fuse(100.0, learned_stamp, require_learned=False)
    assert result["sources"]["learned"] == expected
