#!/usr/bin/env python3
"""Hybrid control-geometry producer without fixed-map subtraction.

The legacy ``obstacle_clusters.py`` deliberately removes returns represented by
the immutable localization map. That is useful for localization exclusions,
but it is the wrong collision contract: a wall, a bench recorded in the map,
or a person standing close to a mapped surface can disappear from the planner
while the independent raw safety gate still sees it and vetoes every command.

This wrapper reuses the field-tested accumulation, rider filtering, clustering,
profiles, tracking, and publishing code, but replaces only the map-membership
filter with an identity filter. The hybrid launcher remaps its dynamic boxes
to a candidate topic; ``localization_exclusion_boxes.py`` republishes only
people and moving/uncertain objects to the localizer, so mapped walls do not get
removed from registration.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
import obstacle_clusters as legacy


def _float_param(name, default):
    value = rospy.get_param("~" + name, default)
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise rospy.ROSInitException("~%s must be a number" % name)
    if not (value == value) or value in (float("inf"), float("-inf")):
        raise rospy.ROSInitException("~%s must be finite" % name)
    return value


def _bool_param(name, default):
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("true", "1", "yes"):
        return True
    if text in ("false", "0", "no"):
        return False
    raise rospy.ROSInitException("~%s must be true or false" % name)


class KeepAllGeometry(object):
    """Drop-in replacement for FixedMapFilter used only in this process."""

    def __init__(self, *_args, **_kwargs):
        pass

    def retain_novel(self, points_lidar, _map_T_lidar):
        return points_lidar


def _positive_int(name, default):
    value = rospy.get_param("~" + name, default)
    if isinstance(value, bool):
        raise rospy.ROSInitException("~%s must be a positive integer" % name)
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise rospy.ROSInitException("~%s must be a positive integer" % name)
    if value <= 0:
        raise rospy.ROSInitException("~%s must be a positive integer" % name)
    return value


def main():
    # Fixed-map subtraction: on by default since 2026-08-28.
    #
    # Turning it off (2026-08-27, "keep mapped geometry visible to the
    # avoidance planner") made every mapped surface - walls, kerbs, posts,
    # the scenery the prior map exists to describe - arrive as a fresh
    # object every scan. Measured inside one drive, on the same route and
    # the same sensor, across the 2026-08-28 00:04:02 producer swap:
    #
    #   subtraction ON  (obstacle_clusters): mean 1.73 objects/frame,
    #                                        p99 5, max 7, never 8 or more
    #   subtraction OFF (this node):         mean 5.05, p99 16, max 23,
    #                                        16 % of frames at 8 or more
    #
    # That is 2.9x the objects from unchanged surroundings, and it is what
    # "obstacles appear where there are none, and one or two become eleven"
    # is. It is not the learned detector: of 49,818 objects published after
    # the swap, 49,666 were geometric and 152 came from PointPillars.
    #
    # The reason it was turned off is still real - a person standing against
    # a mapped wall can be subtracted along with the wall - so this stays a
    # parameter. It is not the default, because a producer that reports the
    # whole world as novel gives the avoidance layer nothing to avoid.
    if _bool_param("fixed_map_subtraction", True):
        rospy.loginfo(
            "hybrid geometry: fixed-map subtraction is ON; mapped surfaces "
            "are not reported as objects")
    else:
        # ObstacleClusters constructs the filter in __init__. Replace the
        # class before construction so the 500+ MB map is not loaded into a
        # second KD-tree merely to be ignored afterwards.
        legacy.FixedMapFilter = KeepAllGeometry
    node = legacy.ObstacleClusters()

    # Defaults preserve the field-tested clustering thresholds. They are ROS
    # parameters so bag replay may evaluate a more sensitive thin-object
    # profile without editing the implementation.
    legacy.MIN_CELL_POINTS = _positive_int(
        "min_cell_points", legacy.MIN_CELL_POINTS)
    legacy.MIN_CLUSTER_POINTS = _positive_int(
        "min_cluster_points", legacy.MIN_CLUSTER_POINTS)
    legacy.MAX_CLUSTERS = _positive_int(
        "max_clusters", legacy.MAX_CLUSTERS)
    # How far to the SIDE the producer may look.
    #
    # obstacle_clusters is forward-only by construction - ROI_X starts at
    # 0.50 m and the FOV cone is 50 degrees - because rear and side returns
    # are usually the rider and the chair frame. That is right for deciding
    # whether to stop for something ahead, and wrong for the moment the
    # chair is drawing level with the thing it is going round.
    #
    # 2026-08-30 17:27:43.86, mid-bypass: the producer went from one tracked
    # person to ZERO objects and stayed there for 19 seconds, status OK the
    # whole time. safety_gate still saw the returns at x 0.05-0.40, y +0.61
    # - inside 0.50 m and at 57-85 degrees of azimuth, so out on both
    # criteria. The follower, handed an empty list, commanded +0.50 rad/s
    # back toward the route and into what it was passing; only the gate's
    # sweep stopped it. That is the "goes, then stops" the operator saw.
    #
    # The rider exclusion box is what keeps the chair out of this, not the
    # ROI: x -1.00..0.55 and y -0.60..+0.20 covers the occupant and frame,
    # and the returns above sit outside it. Widening the ROI therefore adds
    # the object beside the chair without adding the chair.
    legacy.ROI_X = (_float_param("roi_x_min_m", legacy.ROI_X[0]),
                    legacy.ROI_X[1])
    legacy.FORWARD_FOV_HALF_DEG = _float_param(
        "forward_fov_half_deg", legacy.FORWARD_FOV_HALF_DEG)
    rospy.loginfo("hybrid geometry: ROI x>=%.2f m, FOV +/-%.0f deg",
                  legacy.ROI_X[0], legacy.FORWARD_FOV_HALF_DEG)

    if legacy.MIN_CLUSTER_POINTS < legacy.MIN_CELL_POINTS:
        raise rospy.ROSInitException(
            "~min_cluster_points must be >= ~min_cell_points")

    if legacy.FixedMapFilter is KeepAllGeometry:
        rospy.logwarn(
            "hybrid geometry: fixed-map subtraction is OFF for collision and "
            "avoidance; mapped surfaces remain visible "
            "(cell=%d cluster=%d max=%d)",
            legacy.MIN_CELL_POINTS, legacy.MIN_CLUSTER_POINTS,
            legacy.MAX_CLUSTERS)
    else:
        rospy.loginfo(
            "hybrid geometry: cell=%d cluster=%d max=%d",
            legacy.MIN_CELL_POINTS, legacy.MIN_CLUSTER_POINTS,
            legacy.MAX_CLUSTERS)
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
