#!/usr/bin/env python3
"""RTX DWA follower that conditionally passes a continuously static person.

The stock hybrid follower intentionally waits for every person. This wrapper
changes only that policy transition: after direct same-track STATIC evidence,
DWA may plan around exactly one person at the turn-speed floor. Moving,
unknown, learned-only, too-close, multiple, stale, or geometrically invalid
people remain stop-only.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scipy_ckdtree_compat import install as install_ckdtree_compat
install_ckdtree_compat()

import rospy
from std_msgs.msg import String

import dwa_core
from gpu_dwa_backend import GpuRequiredError, install_gpu_planner

# Install before DwaFollower constructs dwa_core.DwaPlanner. Environment and
# ROS params still choose CuPy or the diagnostic CPU path.
install_gpu_planner(dwa_core)
from cluster_guard import PERSON_BYPASS  # noqa: E402
from dwa_follower import DwaFollower  # noqa: E402
from person_bypass_policy import (  # noqa: E402
    StaticPersonQualifier,
    person_observations,
)
from waypoint_follower import PERSON_BYPASS_CONFIRM_S  # noqa: E402


class PersonBypassDwaFollower(DwaFollower):
    CONTROL_LAW = "dwa"

    def __init__(self):
        super(PersonBypassDwaFollower, self).__init__()
        # Defaults to the follower's own window so the permit and the drive
        # decision cannot be authorized by two different clocks. Overridable
        # for bag replay, never to make the permit the earlier of the two.
        self.person_bypass_confirmation_s = float(rospy.get_param(
            "~person_bypass_confirmation_s", PERSON_BYPASS_CONFIRM_S))
        self.person_bypass_maximum_gap_s = float(rospy.get_param(
            "~person_bypass_maximum_gap_s", 0.35))
        self.person_bypass_position_jump_m = float(rospy.get_param(
            "~person_bypass_position_jump_m", 0.35))
        self.person_bypass_permit_lifetime_s = float(rospy.get_param(
            "~person_bypass_permit_lifetime_s", 0.45))
        self.person_bypass_maximum_forward_m = float(rospy.get_param(
            "~person_bypass_maximum_forward_m", 8.0))
        self.person_bypass_maximum_lateral_m = float(rospy.get_param(
            "~person_bypass_maximum_lateral_m", 1.0))
        self.person_bypass_minimum_near_m = float(rospy.get_param(
            "~person_bypass_minimum_near_m", 0.60))
        self.person_bypass_speed_mps = float(rospy.get_param(
            "~person_bypass_speed_mps", 0.35))
        self.person_bypass_clearance_m = float(rospy.get_param(
            "~person_bypass_clearance_m", 0.80))
        self.qualifier = StaticPersonQualifier(
            confirmation_s=self.person_bypass_confirmation_s,
            maximum_gap_s=self.person_bypass_maximum_gap_s,
            maximum_position_jump_m=self.person_bypass_position_jump_m,
            permit_lifetime_s=self.person_bypass_permit_lifetime_s,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            maximum_lateral_m=self.person_bypass_maximum_lateral_m,
            minimum_near_distance_m=self.person_bypass_minimum_near_m,
            max_speed_mps=self.person_bypass_speed_mps,
            min_clearance_m=self.person_bypass_clearance_m,
        )
        self.permit_pub = rospy.Publisher(
            "/person_bypass/permit", String, queue_size=1, latch=False)
        self._permit_published_this_cycle = False
        rospy.set_param("~person_bypass_capable", True)
        rospy.loginfo(
            "stationary-person bypass: %.1f s same-track STATIC, "
            "v<=%.2f m/s, clearance>=%.2f m",
            self.person_bypass_confirmation_s,
            self.person_bypass_speed_mps,
            self.person_bypass_clearance_m)

    def publish_permit(self, permit):
        self.permit_pub.publish(String(data=permit.to_json()))
        self._permit_published_this_cycle = True

    def inactive_permit(self, now, reason):
        return self.qualifier.inactive(now.to_sec(), reason)

    def observed_person_permit(self, now):
        """Update qualification even while the motion service is paused.

        The base follower returns from its hold ladder before asking
        ``avoidance_for`` when it is paused. If qualification lived only in
        that method, a person already standing in front of the chair would
        make ``go`` impossible forever: the permit needs motion to start and
        semantic preflight needs the permit before motion may start. Reading
        perception here breaks that cycle without sending any command.
        """
        threat = self.corridor_threat(0.0)
        if threat is None or not threat.is_person:
            self.qualifier.reset()
            return self.inactive_permit(now, "NEAREST_THREAT_NOT_PERSON")
        observations = person_observations(
            self.cluster_summary,
            maximum_forward_m=self.person_bypass_maximum_forward_m,
            maximum_lateral_m=self.person_bypass_maximum_lateral_m,
        )
        return self.qualifier.update(
            observations, now.to_sec(), self.tracking_state == "TRACKING")

    def avoidance_for(self, now, threat, blocking):
        """Publish the gate permit; leave the driving decision to the base.

        This class used to make its own decision as well: on an active permit
        it returned GO_ROUND and reassigned ``dwa_core.OBSTACLE_FLOOR_M``, a
        module global, from inside a control cycle. That predates the base
        follower having a PERSON_BYPASS decision of its own. Keeping both left
        two authorities answering one question with two clocks - this node's
        3.0 s qualifier and the follower's PERSON_BYPASS_CONFIRM_S - and a
        clearance that leaked into every later cycle through a global whose
        restore lived in a `finally` in another method.

        One decision now. The base decides whether the chair may pass, using
        geometry it has; this decides whether the two GATES may be asked to
        allow the arc that decision produces, using direct same-track
        evidence. Both must agree before anything moves, and neither can be
        inferred from the other.
        """
        ordinary = super(PersonBypassDwaFollower, self).avoidance_for(
            now, threat, blocking)
        if threat is None or not threat.is_person:
            self.qualifier.reset()
            self.publish_permit(self.inactive_permit(
                now, "NEAREST_THREAT_NOT_PERSON"))
            return ordinary

        permit = self.observed_person_permit(now)
        self.publish_permit(permit)
        if not permit.active or ordinary != PERSON_BYPASS:
            return ordinary

        # The base has authorized the pass and the permit is live, so the raw
        # gate is about to be asked to allow a curved arc round a body that IS
        # in planner geometry. GATE_STALL is the diagnostic for an obstacle
        # that is NOT, and firing it here would stop the cycle before a curved
        # proposal exists - which is the deadlock it was written to report.
        self.gate_reason = ""
        self.gate_blocked_since = None
        self.gate_detail = "static-person trajectory permit"
        return ordinary

    def step(self):
        self._permit_published_this_cycle = False
        now = rospy.Time.now()
        # Publish a continuously refreshed qualification heartbeat before the
        # base hold ladder can return for PAUSED/MANUAL/STARTUP. This does not
        # bypass any guard; it only lets the later preflight distinguish a
        # stable person from a moving or unknown one before enabling motion.
        self.publish_permit(self.observed_person_permit(now))
        try:
            super(PersonBypassDwaFollower, self).step()
        finally:
            if not self._permit_published_this_cycle:
                if self.tracking_state != "TRACKING":
                    self.qualifier.reset()
                self.publish_permit(self.inactive_permit(
                    now, "FOLLOWER_NOT_EVALUATING_PERSON"))


if __name__ == "__main__":
    try:
        PersonBypassDwaFollower().run()
    except GpuRequiredError as error:
        rospy.logfatal("required RTX DWA backend failed: %s", error)
        raise SystemExit(2)
    except rospy.ROSInterruptException:
        pass
