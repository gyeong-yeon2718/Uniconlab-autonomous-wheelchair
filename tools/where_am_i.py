#!/usr/bin/env python3
"""Why the chair is or is not ready to drive, in one screen.

terrain_guard reports MASK_BOUNDARY whenever a rollout of the commanded
motion leaves the drivable mask, and a chair parked off the route looks
exactly like that from inside the graph: every node healthy, every reading
fresh, and nothing that can ever become ready. On 2026-08-28 that was read
twice as "hybrid profile never became ready".

Being inside the point-cloud MAP is not the same as being inside the mask.
The map covers the site; the mask is the route corridor.
"""

import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "src/static_livox_localization/scripts"))

import rospy  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from route_mask import RouteMask  # noqa: E402

ROUTE = os.environ.get(
    "ROUTE", os.path.join(REPO, "routes",
                          "20260816_route_v9_clearance_waypoints.json"))
MASK = os.environ.get(
    "DRIVABLE_MASK", os.path.join(REPO, "routes", "route_2d_map_v9.yaml"))


def main():
    pose = {}

    def on_pose(message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        pose["xy"] = (p.x, p.y)
        pose["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    rospy.init_node("where_am_i", anonymous=True, disable_signals=True)
    rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped, on_pose)
    for _ in range(60):
        if "xy" in pose:
            break
        time.sleep(0.1)
    if "xy" not in pose:
        print("NO POSE on /fast_lio_icp/pose - the localizer is not running.")
        return 2

    x, y = pose["xy"]
    yaw = pose["yaw"]
    mask = RouteMask(MASK)
    here = np.array([[x, y]])
    inside = bool(np.asarray(mask.contains_many(here))[0])

    with open(ROUTE) as handle:
        data = json.load(handle)
    points = data.get("waypoints") or data.get("points") or data
    route = np.array([[p["x"], p["y"]] if isinstance(p, dict) else p[:2]
                      for p in points], dtype=float)
    gap = np.linalg.norm(route - np.array([x, y]), axis=1)
    nearest = int(gap.argmin())

    ahead = route[min(nearest + 5, len(route) - 1)] - np.array([x, y])
    bearing = math.atan2(ahead[1], ahead[0])
    off_heading = math.degrees(math.atan2(math.sin(bearing - yaw),
                                          math.cos(bearing - yaw)))

    print("pose            x=%.2f  y=%.2f  yaw=%.1f deg" % (x, y, math.degrees(yaw)))
    print("inside mask     %s" % ("yes" if inside else "NO"))
    if hasattr(mask, "clearance_many"):
        print("to mask edge    %.3f m"
              % float(np.asarray(mask.clearance_many(here))[0]))
    print("nearest wp      %d / %d at %.2f m" % (nearest, len(route), gap[nearest]))
    print("heading error   %+.0f deg from the route direction" % off_heading)
    print()

    drivable = []
    for speed, yaw_rate, label in ((0.30, 0.0, "straight"),
                                   (0.15, 0.20, "left"),
                                   (0.15, -0.20, "right")):
        left_at = None
        for step in range(1, 21):
            elapsed = step * 0.1
            heading = yaw + yaw_rate * elapsed
            if abs(yaw_rate) < 1e-9:
                px = x + speed * elapsed * math.cos(yaw)
                py = y + speed * elapsed * math.sin(yaw)
            else:
                px = x + (speed / yaw_rate) * (math.sin(heading) - math.sin(yaw))
                py = y - (speed / yaw_rate) * (math.cos(heading) - math.cos(yaw))
            if not bool(np.asarray(mask.contains_many(np.array([[px, py]])))[0]):
                left_at = elapsed
                break
        drivable.append(left_at is None)
        print("  %-9s %s" % (label, "stays in the mask for 2 s"
                             if left_at is None
                             else "leaves the mask at %.1f s" % left_at))

    print()
    if inside and any(drivable):
        print("READY to bring up: the chair is on the drivable route.")
        return 0
    print("NOT READY: move the chair onto the route corridor first.")
    print("  terrain_guard will hold MASK_BOUNDARY until it is, and the")
    print("  bring-up can only time out waiting for it.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
