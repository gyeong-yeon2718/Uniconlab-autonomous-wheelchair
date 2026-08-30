#!/usr/bin/env python3
"""Bluetooth SPP (RFCOMM) telemetry/command bridge for the UniconLab wheelchair NUC.

ATTRIBUTION
-----------
The JSON-lines telemetry/command shape this bridge speaks was derived from
``edge-mobility-monitor`` by Park Hyeongjun (박형준),
https://github.com/Geppetto0608/edge-mobility-monitor -- used with the author's
permission, on condition of attribution.  Keep this notice in every copy.
See ``android_wheelchair_ui/NOTICE.md``.

WHICH STACK THIS TALKS TO
-------------------------
The repo contains two different worlds and only one of them runs in the field:

* ``src/wheelchair_safety`` + ``src/wheelchair_interfaces`` -- the WP0 contract
  scaffold (``/safety/state``, ``/cmd_vel_safe``, ``sidewalk``/``road_free_space``,
  ``armed``, ``reason_mask``).  It is *built*, but ``start_wheelchair_localization.sh``
  never launches it.  Do not integrate against it.
* ``livox_static_localization_ws`` -- what actually drives the chair.  This bridge
  targets that, verified by reading the live workspace on the NUC 2026-08-14.

The real command chain is::

    follower (waypoint/mpc/dwa)  ->  /cmd_vel_raw
      safety_gate.py             ->  /cmd_vel_gated
      tip_guard.py               ->  /cmd_vel
      wheel_cmd_tmp.py           ->  /wheel_cmd   (Int16MultiArray)
      uart.py                    ->  UART         -> motor controller

DESIGN RULES
------------
1. The bridge PUBLISHES EXACTLY ONE TOPIC: ``mode_cmd`` (Int16).  Everything else
   is read-only.  ``wheel_cmd_tmp.py`` rejects ``/cmd_vel`` unless the publisher's
   callerid is ``/tip_guard`` and ``/wheel_status`` unless it is ``/uart``; a
   stray publisher there sets ``fault_latched`` and jams the chair into a fault
   stop.  ``mode_cmd`` has no callerid check, which is why it is the safe lever.
2. E-STOP is ``mode_cmd = 77`` (Manual).  ``uart.py`` immediately transmits the
   motor stop frame and then ignores every autonomous ``wheel_cmd``, because
   ``CmdCallback`` only forwards while mode == 65.  It therefore holds even if
   every ROS node above it dies, and it returns the chair to joystick control.
3. E-STOP also PAUSES the follower, because waypoint_follower.py only *holds* on
   MANUAL_MODE and stays enabled -- without the pause, releasing the e-stop would
   drive off immediately.  stop.sh does both for the same reason.
4. Release is ``mode_cmd = 65`` (Auto), which transmits a stop frame first, so
   re-arming cannot lurch.  It needs an explicit confirm flag and does not
   restart the follower -- driving resumes only on a separate command.
5. Truth comes from ``/wheel_status``: ``data[1]`` is the mode the motor
   controller echoes back.  A command is only reported as effective once that
   echo agrees.
6. Telemetry never invents a value.  Fields with no source are ``null`` and named
   in ``unavailable``.

Usage
-----
    python3 scripts/ros1_bluetooth_bridge.py --self-test
    python3 scripts/ros1_bluetooth_bridge.py                    # observer only
    python3 scripts/ros1_bluetooth_bridge.py --allow-commands   # e-stop + drive
"""

import argparse
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

try:
    import rospy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int16, String
    from std_msgs.msg import Int16MultiArray
    from std_srvs.srv import SetBool
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    from diagnostic_msgs.msg import DiagnosticArray
    DIAGNOSTICS_AVAILABLE = True
except ImportError:
    DIAGNOSTICS_AVAILABLE = False

PROTOCOL_VERSION = 3
SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

AUTO_MODE = 65          # 'A' -- autonomous wheel commands accepted
MANUAL_MODE = 77        # 'M' -- joystick; autonomous commands ignored
MODE_LABELS = {AUTO_MODE: "auto", MANUAL_MODE: "manual"}

FOLLOWER_START_SERVICE = "/waypoint_follower/start"
MOTION_EPS = 0.02       # m/s and rad/s below which a Twist counts as zero
# How long a subsystem reading stays believable after its topic goes quiet.
# /wheel_status already had a TTL, which is why the wheel link reported 끊김 the
# moment the stack came down -- while localization went on claiming TRACKING and
# the obstacle line went on claiming a clear band, from nodes that no longer
# existed. Every one of those publishes at 1 Hz or faster, so five seconds of
# silence means the publisher is gone, not slow.
FIELD_TTL_S = 5.0

# The interpreter the operator scripts are run with. A module constant rather
# than a literal so the job tests can point it at a bash that exists on the
# machine running them; on the NUC this is always /bin/bash.
BASH = "/bin/bash"


def log(message):
    sys.stdout.write("[bt_bridge] %s\n" % message)
    sys.stdout.flush()


def _as_float(text):
    try:
        return round(float(text), 4)
    except (TypeError, ValueError):
        return None


def twist_magnitude(twist):
    return max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))


# Above this many points the route is thinned before sending. A field route is
# captured at 0.2 m spacing; at ~1 m it is visually identical on a phone-sized
# top-down view and costs a fifth of the link budget.
ROUTE_MAX_POINTS = 400
# The drivable corridor is sent as two edge polylines, thinned like the route.
# Fewer points than the route: the band is a smooth pair of offsets, and its job
# on a phone-sized view is to show where the room runs out, not every wobble.
BAND_MAX_POINTS = 200

# Per-object geometry for the app's close-in view, forwarded in every
# telemetry frame. obstacle_clusters.py's own summary cannot go on this link:
# one object carries up to 64 numbers of lateral profile, and forty of them
# would be the whole 2 Hz budget spent on a shape nobody can see from above.
# So only what can be drawn is kept, NEAREST FIRST -- what a crowded scene
# drops is the far half, never the object about to be driven into.
OBJECT_MAX_COUNT = 10
# Path samples per object, at phone size. More draws the same line.
OBJECT_TRAIL_MAX_POINTS = 8
# Below this a path is not a path, it is a parked object's centroid twitching
# by centimetres. Drawn, it says "moving" about something that is not, so it
# is dropped and the object shows as the dot it is.
OBJECT_TRAIL_MIN_SPAN_M = 0.25
# How much of an object's past is kept here. The producer's own tracker keeps
# 3.0 s and this is the same window deliberately: two views of one scene that
# disagree about how long ago something was elsewhere is worse than either.
OBJECT_TRAIL_HISTORY_S = 3.0
# A ceiling on remembered objects, so a scene the filter never trims -- a
# crowded crossing, a scan full of kerb -- cannot grow this without bound.
OBJECT_TRAIL_MAX_TRACKS = 64
# Further than this between two sightings of one id and it is not one object
# any more. Ids are the producer's track ids where it has tracks and plain
# list indices where it does not, and an index is handed to whatever cluster
# happens to sort into that slot next. Without this, two objects a pavement
# apart get joined by a line neither of them walked.
OBJECT_TRAIL_JUMP_M = 1.5

# What the app is not shown, and why it is safe not to show it.
#
# A campus scan is mostly ground. Sampled live 2026-08-27, a quarter of every
# message was clusters like 0.21 x 0.26 x 0.10 m from five returns -- a speck
# of kerb, a drain, a paving lip. Drawn, they are a screen full of boxes with
# nothing in it, and the one object that matters is somewhere in the middle
# of them.
#
# THIS IS A DISPLAY FILTER AND NOTHING ELSE. It runs on the copy going out
# over Bluetooth, never on /perception/objects_summary, which safety_gate and
# the followers read -- filtering there would hide obstacles from the guard,
# which is the one thing this must never do.
#
# Anything in the corridor is exempt whatever its evidence. What stops the
# chair is never hidden from the person watching it.
OBJECT_MIN_HEIGHT_M = 0.25
OBJECT_MIN_POINTS = 10

# Where "long" stops meaning "vehicle", for the label the app prints.
#
# classify() has a floor and no ceiling, so length alone carries it: a kerb
# line, a hedge or a campus wall is over 1.5 m across and sits inside the
# 0.9-2.5 m height window, and comes out as a parked car. What separates a
# car from a wall is not size but shape -- a car seen from anywhere is
# roughly 4.5 x 1.8 m, a wall is however long the scan reached by however
# thin it is -- so both are checked here. 6.5 m clears a real car's 4.9 m
# diagonal and stops well short of a building face; 4.0 clears a car seen
# end-on and rejects the 12:1 slabs the pavement is full of. Measured
# against 25 s of live campus clusters 2026-08-27: 300 of 1254 "vehicles".
#
# Display only, and it has to stay that way. The producer's label is what
# cluster_guard reads (PERSON_LABEL), and nothing here can turn anything
# into or out of a person -- only "vehicle" is ever rewritten, and only into
# "obstacle", a name no controller reads at all.
VEHICLE_MAX_FOOTPRINT_M = 6.5
VEHICLE_MAX_ELONGATION = 4.0


def display_class(obj):
    """The producer's label, with the walls taken back out of "vehicle"."""
    label = obj.get("class")
    if label != "vehicle":
        return label
    size = obj.get("size") or []
    if len(size) < 2:
        return label                    # no shape stated: no grounds to argue
    try:
        span_x, span_y = abs(float(size[0])), abs(float(size[1]))
    except (TypeError, ValueError):
        return label
    long_side = max(span_x, span_y)
    short_side = max(min(span_x, span_y), 1e-3)
    if math.hypot(span_x, span_y) > VEHICLE_MAX_FOOTPRINT_M or \
            long_side / short_side > VEHICLE_MAX_ELONGATION:
        return "obstacle"
    return label


def thin_trail(trail):
    """An object's path, reduced to what is worth the bytes.

    Returns ``[]`` for a path that goes nowhere. A parked object still has a
    history -- its centroid slides by a few centimetres as more of it comes
    into view -- and drawing that puts a scribble under every bollard on the
    pavement, which reads as a crowd. The span is measured as the bounding
    box diagonal rather than end to end, so someone who walks out and back
    still counts as having moved.
    """
    points = []
    for point in trail:
        try:
            points.append([round(float(point[0]), 2),
                           round(float(point[1]), 2)])
        except (IndexError, KeyError, TypeError, ValueError):
            return []
    if len(points) < 2:
        return []
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    if math.hypot(max(xs) - min(xs),
                  max(ys) - min(ys)) < OBJECT_TRAIL_MIN_SPAN_M:
        return []
    stride = max(1, (len(points) + OBJECT_TRAIL_MAX_POINTS - 1)
                 // OBJECT_TRAIL_MAX_POINTS)
    thinned = points[::stride]
    # The newest sample is the one that sits on the object itself. Thinning
    # must never be what pulls a path off the box it belongs to -- that is
    # the only thing saying which path is whose.
    if thinned[-1] != points[-1]:
        thinned.append(points[-1])
    return thinned


class TrailMemory:
    """Where each object has been, remembered on this side of the link.

    The app draws paths so the operator can see that the thing in the
    corridor walked into it rather than having been parked there all along.
    Nothing on the wire carries that: /perception/objects_summary states
    where each object is now, and the history behind it lives inside
    obstacle_clusters.py -- a file the chair drives on, which is not being
    edited for a dashboard. So the bridge keeps its own.

    The samples are stored in the MAP frame and handed back out about the
    chair's CURRENT pose. That is the whole design and not an implementation
    detail: objects arrive chair-relative, so a parked bollard's x/y changes
    every frame the chair moves, and a path stored as it arrived would draw
    the chair's own journey under every stationary object on the pavement --
    a viewer reads that as a crowd walking backwards. Going through the pose
    both ways cancels the chair's motion, so anything genuinely parked
    collapses to a point and only real motion draws a line.

    With no pose there is no frame to do that in, and the honest output is
    no path at all. A dot is worth less than a line and much less than a lie.
    """

    def __init__(self, history_s=OBJECT_TRAIL_HISTORY_S,
                 max_tracks=OBJECT_TRAIL_MAX_TRACKS):
        self.history_s = history_s
        self.max_tracks = max_tracks
        self._tracks = {}

    @staticmethod
    def _to_map(pose, x, y):
        px, py, yaw_deg = pose
        yaw = math.radians(yaw_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return (px + x * cos_yaw - y * sin_yaw,
                py + x * sin_yaw + y * cos_yaw)

    @staticmethod
    def _to_chair(pose, map_x, map_y):
        px, py, yaw_deg = pose
        yaw = math.radians(yaw_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        dx, dy = map_x - px, map_y - py
        return (dx * cos_yaw + dy * sin_yaw, -dx * sin_yaw + dy * cos_yaw)

    def observe(self, key, x, y, pose, now):
        """Record one sighting of ``key``, or do nothing without a pose."""
        if pose is None or key is None:
            return
        map_x, map_y = self._to_map(pose, x, y)
        history = self._tracks.get(key)
        if history and math.hypot(map_x - history[-1][1],
                                  map_y - history[-1][2]) > OBJECT_TRAIL_JUMP_M:
            history = None              # this id now belongs to something else
        if history is None:
            history = self._tracks[key] = []
        history.append((now, map_x, map_y))
        cutoff = now - self.history_s
        while history and history[0][0] < cutoff:
            history.pop(0)

    def trail(self, key, pose):
        """``[[x, y], ...]`` about the chair as it is now, oldest first."""
        history = self._tracks.get(key)
        if pose is None or not history or len(history) < 2:
            return []
        return [list(self._to_chair(pose, map_x, map_y))
                for _stamp, map_x, map_y in history]

    def sweep(self, now):
        """Forget what has gone, and cap what has not.

        Objects leave by walking out of range, and ids are reused, so
        without this the memory grows for as long as the bridge runs.
        """
        for key in [k for k, h in list(self._tracks.items())
                    if not h or (now - h[-1][0]) > self.history_s]:
            del self._tracks[key]
        excess = len(self._tracks) - self.max_tracks
        if excess > 0:
            for key in sorted(self._tracks,
                              key=lambda k: self._tracks[k][-1][0])[:excess]:
                del self._tracks[key]


def worth_drawing(obj):
    """Is this a thing, or is it the ground?

    Height first, because that is what actually separates them: a 0.10 m rise
    outside the corridor is paving, and no number of returns makes it an
    object. The point count catches the other tail, the handful of stray
    returns that a box gets drawn around at fifteen metres.

    Missing evidence keeps the object. A producer that does not publish a
    height has not told us it is flat.
    """
    if obj.get("band_relation") in ("inside", "overlap"):
        return True                     # in the way: never hidden
    size = obj.get("size") or []
    if len(size) >= 3:
        try:
            if float(size[2]) < OBJECT_MIN_HEIGHT_M:
                return False
        except (TypeError, ValueError):
            pass
    points = obj.get("points")
    if isinstance(points, int) and points < OBJECT_MIN_POINTS:
        return False
    return True


def drawable_object(obj, x, y, trail=None):
    """One cluster reduced to what a top-down view can draw.

    Names are the producer's own, so there is one vocabulary between
    /perception/objects_summary, this link and the app rather than three --
    the single exception being the vehicle label, which is corrected for
    display and keeps the producer's word beside it in ``raw_class``.
    """
    size = obj.get("size") or []
    record = {
        "id": obj.get("id"),
        "class": display_class(obj),
        "raw_class": obj.get("class"),
        "band_relation": obj.get("band_relation"),
        "x": round(x, 2),
        "y": round(y, 2),
        # Footprint only. Height is not drawable from above, and the view
        # asks nothing that depends on it.
        "size": ([round(float(size[0]), 2), round(float(size[1]), 2)]
                 if len(size) >= 2 else None),
        "motion": obj.get("motion"),
        "speed_mps": obj.get("speed_mps"),
    }
    # The producer's own path when it publishes one -- it has the tracker's
    # full history and a better frame to state it in -- and the bridge's
    # reconstruction only where it does not.
    thinned = thin_trail(obj.get("trail") or trail or [])
    if thinned:
        record["trail"] = thinned
    return record


_BRINGUP_ROUTE_RE = re.compile(r'^\s*ROUTE=\"\$\{ROUTE:-(?P<path>[^}]+)\}\"')


def route_from_bringup_script(script_dir):
    """The route start_wheelchair_localization.sh would launch with.

    Second-best after the live param and far better than a filename pinned here:
    the bring-up script is where the field default is actually chosen, so reading
    it means the app follows a route promotion without this file being touched.
    """
    seen = []
    for base in (script_dir, "~"):
        if not base:
            continue
        path = os.path.join(os.path.expanduser(base),
                            "start_wheelchair_localization.sh")
        if path in seen:
            continue
        seen.append(path)
        try:
            with open(path, "r") as handle:
                for line in handle:
                    match = _BRINGUP_ROUTE_RE.match(line)
                    if match:
                        return os.path.expandvars(match.group("path"))
        except OSError:
            continue
    return None


def resolve_route_path(cli_default, script_dir=None):
    """Ask the follower which route it is actually driving.

    start_wheelchair_localization.sh launches the follower with `_route:="$ROUTE"`,
    and ROUTE is overridable by environment, so any hard-coded filename here is a
    guess. Guessing wrong is worse than showing nothing: the app would draw one
    route while the chair drove another, and the progress marker would be placed
    on a line that is not the line being followed. The launched value lands in the
    private param /waypoint_follower/route, so read that first.

    With the stack down there is no param -- and that is exactly when the app is
    open, because the operator is about to press [로컬 켜기]. So read the bring-up
    script's own ROUTE default next, and only then the CLI default. Skipping that
    step is how the app came to draw the 1897-point 20260814 algorithm route while
    the chair was pinned to the 1917-point v9 clearance route.
    """
    if ROS_AVAILABLE:
        for key in ("/waypoint_follower/route", "/waypoint_follower/_route"):
            try:
                value = rospy.get_param(key)
            except Exception:                                     # noqa: BLE001
                continue
            if value:
                log("route from follower param %s" % key)
                return str(value)
        log("follower route param not found -- reading the bring-up script")
    from_script = route_from_bringup_script(script_dir)
    if from_script:
        log("route from start_wheelchair_localization.sh: %s"
            % os.path.basename(from_script))
        return from_script
    return cli_default


def load_band(route_path):
    """Left and right edges of the drivable corridor, in the map frame.

    The route alone is a centreline, which tells an operator where the chair is
    going but not how much room it has -- and on this route the room is the
    interesting part: the v9 corridor is what the tight corners are measured
    against. The band file sits next to the route with the same stem and carries
    a station every 0.5 m with left_m/right_m clearances, so the edges are just
    the centreline offset by those, perpendicular to the station heading.
    """
    band_path = re.sub(r"_waypoints\.json$", "_safety_band.json", route_path)
    if band_path == route_path:
        return None
    try:
        with open(os.path.expanduser(band_path), "r") as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    stations = data.get("stations") or []
    if len(stations) < 2:
        return None

    left, right = [], []
    for station in stations:
        try:
            x = float(station["x"])
            y = float(station["y"])
            heading = math.radians(float(station["heading_deg"]))
            lm = float(station.get("left_m", 0.0))
            rm = float(station.get("right_m", 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        # Left of travel is heading + 90 degrees.
        left.append([round(x - math.sin(heading) * lm, 2),
                     round(y + math.cos(heading) * lm, 2)])
        right.append([round(x + math.sin(heading) * rm, 2),
                      round(y - math.cos(heading) * rm, 2)])
    if len(left) < 2:
        return None

    stride = max(1, (len(left) + BAND_MAX_POINTS - 1) // BAND_MAX_POINTS)
    def thin(points):
        slim = points[::stride]
        if slim[-1] != points[-1]:
            slim.append(points[-1])
        return slim
    log("band loaded: %d stations (%d sent) from %s"
        % (len(left), len(thin(left)), os.path.basename(band_path)))
    return {"left": thin(left), "right": thin(right),
            "source": os.path.basename(band_path)}


def load_route(path):
    """Read the waypoint JSON the follower is driving, for the app's map view.

    Sent once per connection rather than in every telemetry frame: the field
    default is 1897 waypoints (~32 kB of JSON), which is fine once but would be
    several times the SPP budget at 2 Hz. The route and /fast_lio_icp/pose share
    the ``map`` frame -- the route was captured from that very topic -- so no
    transform is needed to draw them on the same axes.
    """
    if not path:
        return None
    path = os.path.expanduser(path)
    try:
        with open(path, "r") as handle:
            data = json.load(handle)
    except (OSError, ValueError) as exc:
        log("route not loaded (%s): %s" % (path, exc))
        return None
    points = data.get("waypoints") or []
    if not points:
        log("route %s has no waypoints" % path)
        return None

    full = [[round(float(p.get("x", 0.0)), 2), round(float(p.get("y", 0.0)), 2)]
            for p in points]
    stride = max(1, (len(full) + ROUTE_MAX_POINTS - 1) // ROUTE_MAX_POINTS)
    if stride > 1:
        slim = full[::stride]
        # The end of the route is where the chair is going; never let thinning
        # drop it, or the drawn line stops short of the actual destination.
        if slim[-1] != full[-1]:
            slim.append(full[-1])
    else:
        slim = full

    log("route loaded: %d waypoints (%d sent, stride %d) from %s"
        % (len(full), len(slim), stride, os.path.basename(path)))
    band = load_band(path)
    return {
        "type": "route",
        "frame": data.get("frame"),
        "body_frame_profile": data.get("body_frame_profile"),
        "count": len(slim),
        "count_full": len(full),
        "stride": stride,          # app maps wp_index -> drawn index with this
        "points": slim,
        "source": os.path.basename(path),
        "path": path,
        "band_left": None if band is None else band["left"],
        "band_right": None if band is None else band["right"],
        "band_source": None if band is None else band["source"],
    }


class JobRunner:
    """Runs the operator's own shell scripts, from a fixed allowlist.

    The app should press the same buttons the operator presses at the keyboard --
    ``go.sh`` already publishes ``/mode_cmd 65``, calls the follower service, and
    refuses with a written reason when a precondition fails. Reimplementing that
    in Python would mean two copies of the launch policy that drift apart, so the
    bridge shells out to the real scripts and relays their output.

    ALLOWLIST ONLY. Any bonded phone can open this link, so it must never be able
    to run an arbitrary command -- the wire protocol carries a job *name*, and the
    name is looked up in a table fixed at start-up. Nothing from the phone ever
    reaches a shell.
    """

    #  name  ->  (script filename, human label)
    #
    # trial_0727.sh is deliberately NOT here: it brings the stack up with
    # SAFETY_POLICIES=false, and a phone button that starts a guard-suppressed
    # run is not something this link should offer.
    JOBS = {
        "stack": ("start_wheelchair_localization.sh", "로컬라이제이션 스택 기동"),
        "stack_stop": ("stop_stack.sh", "스택 내리기"),
        "drive": ("go.sh", "주행 시작"),
        "halt": ("stop.sh", "주행 정지"),
    }

    def __init__(self, script_dir, enabled, env=None):
        self.script_dir = os.path.expanduser(script_dir)
        self.enabled = enabled
        # start_wheelchair_localization.sh picks its controller from $PROFILE and
        # defaults to pursuit. The field DWA runs are launched as
        # `PROFILE=dwa SAFETY_POLICIES=true start_wheelchair_localization.sh`, so a
        # bridge started from a plain shell would bring up a *different controller*
        # than the one that was last driven -- same button, same script, different
        # robot behaviour. Carry the operator's environment explicitly instead.
        self.env_overlay = dict(env or {})
        self.lock = threading.Lock()
        self.proc = None
        self.name = None
        self.label = None
        self.started_at = None
        self.finished_at = None
        self.exit_code = None
        self.tail = None
        self.log_path = None

    def resolve(self, name):
        entry = self.JOBS.get(name)
        if entry is None:
            return None, "unknown job %r" % (name,)
        path = os.path.join(self.script_dir, entry[0])
        if not os.path.isfile(path):
            return None, "%s not found at %s" % (entry[0], path)
        return path, entry[1]

    def job_env(self):
        env = os.environ.copy()
        env.update(self.env_overlay)
        return env

    def available(self):
        """Which jobs actually exist on this machine, for the UI to grey buttons."""
        out = {}
        for name in self.JOBS:
            path, _ = self.resolve(name)
            out[name] = path is not None
        return out

    def busy(self):
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    # Stopping must never queue behind anything. stop.sh deliberately checks
    # nothing, and a stop that waits for a bring-up to finish is not a stop.
    #
    # stack_stop is here for a second reason: it is the only real abort for a
    # bring-up in progress. job_cancel signals the tracked process group, but the
    # bring-up detaches its nodes with setsid, so the sensors it already started
    # never see that signal. Making the teardown wait for the bring-up it is
    # meant to undo would be the same bug as a stop that queues.
    ALWAYS_ALLOWED = ("halt", "stack_stop")

    def _spawn(self, path, name):
        """Launch detached without touching the tracked slot."""
        try:
            handle = open("/tmp/bt_job_%s.log" % name, "wb")
        except OSError:
            handle = subprocess.DEVNULL
        return subprocess.Popen(
            [BASH, path], cwd=os.path.expanduser("~"), env=self.job_env(),
            stdin=subprocess.DEVNULL, stdout=handle,
            stderr=subprocess.STDOUT, start_new_session=True)

    def start(self, name):
        if not self.enabled:
            return False, ("script execution is disabled on this bridge "
                           "(--allow-scripts off)")
        path, label = self.resolve(name)
        if path is None:
            return False, label                      # label carries the reason

        if name in self.ALWAYS_ALLOWED:
            # Runs even while a bring-up holds the slot, and does not overwrite
            # that job's status -- the operator still needs to see how it ended.
            try:
                self._spawn(path, name)
            except OSError as exc:
                return False, "failed to launch %s: %s" % (path, exc)
            log("job '%s' started (unqueued): %s" % (name, path))
            return True, "%s 실행함 (로그: /tmp/bt_job_%s.log)" % (label, name)

        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return False, "'%s' is still running; wait for it or stop it first" % self.name
            self.log_path = "/tmp/bt_job_%s.log" % name
            try:
                handle = open(self.log_path, "wb")
            except OSError as exc:
                return False, "cannot open %s: %s" % (self.log_path, exc)
            try:
                # start_new_session so a bridge restart does not kill a bring-up
                # that is already half way through starting the sensors.
                self.proc = subprocess.Popen(
                    [BASH, path],
                    cwd=os.path.expanduser("~"), env=self.job_env(),
                    stdin=subprocess.DEVNULL, stdout=handle,
                    stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                handle.close()
                return False, "failed to launch %s: %s" % (path, exc)
            self.name, self.label = name, label
            self.started_at = time.time()
            self.finished_at = None
            self.exit_code = None
            self.tail = None
        log("job '%s' started: %s" % (name, path))
        return True, "%s 시작함 (로그: %s)" % (label, self.log_path)

    def cancel(self):
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                return False, "no job is running"
            name = self.name
            try:
                # The job runs in its own session, so signal the whole group --
                # start_wheelchair_localization.sh spawns roslaunch children and
                # killing only the parent would leave the sensors up.
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except AttributeError:
                # No process groups on this platform (Windows bench runs).
                self.proc.terminate()
            except Exception as exc:                              # noqa: BLE001
                return False, "could not signal '%s': %s" % (name, exc)
        # Honest about the limit: start_wheelchair_localization.sh launches its
        # nodes with setsid/$DETACH, so those grandchildren sit in their own
        # sessions and this signal never reaches them. Cancelling a bring-up
        # stops the script, not necessarily the sensors it already started.
        return True, ("'%s' 에 종료 신호를 보냈습니다. 단, 이미 분리 실행된 "
                      "노드(라이다·FAST-LIO 등)는 살아있을 수 있으니 NUC에서 확인하세요."
                      % name)

    def snapshot(self):
        with self.lock:
            if self.proc is None:
                return {"job_name": None, "job_state": "idle", "job_elapsed_s": None,
                        "job_exit_code": None, "job_tail": None}
            code = self.proc.poll()
            if code is not None and self.finished_at is None:
                self.finished_at = time.time()
                self.exit_code = code
            state = ("running" if code is None
                     else "succeeded" if code == 0 else "failed")
            end = self.finished_at or time.time()
            # Last non-empty line is what the operator would be reading.
            tail = self.tail
            if self.log_path:
                try:
                    with open(self.log_path, "rb") as handle:
                        lines = [l for l in handle.read().decode(
                            "utf-8", errors="replace").splitlines() if l.strip()]
                    if lines:
                        tail = lines[-1][:160]
                except OSError:
                    pass
            self.tail = tail
            return {
                "job_name": self.name,
                "job_label": self.label,
                "job_state": state,
                "job_elapsed_s": round(end - self.started_at, 1),
                # How long ago it FINISHED, so the app can present an old result
                # as history instead of as the current state of the robot. A
                # failed bring-up from ten minutes ago must not keep shouting
                # after the operator has fixed the thing by hand.
                "job_age_s": (None if self.finished_at is None
                              else round(time.time() - self.finished_at, 1)),
                "job_exit_code": self.exit_code,
                "job_tail": tail,
            }


class BridgeState:
    """Everything the phone can see, with provenance for each field."""

    # A speed reading older than this is not evidence about what the chair is
    # doing now. Wheel odometry runs at 100 Hz, so half a second is generous.
    SPEED_TTL_S = 0.5

    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0
        self.started_at = time.time()

        self.drive_mode = None          # 65 / 77, echoed by the motor controller
        # /wheel_status data[7], which base_model republishes as wheel_battery
        # and this bridge briefly believed. It is not a charge level.
        #
        # Sniffed straight off the UART on 2026-08-25 while the chair was being
        # driven on the joystick: 2505 good frames, and data[7] took exactly two
        # values, 88 and 99. Not a slow drift between them -- it sat at 88 and
        # pulsed to 99 for about half a second, three times in twenty seconds.
        # Nothing that reports remaining charge does that. Earlier sessions add
        # 66 and 77, so the set so far is 11 x {6, 7, 8, 9}: a small status or
        # level code, changing on a sub-second timescale.
        #
        # base_model calls it a battery and differences it into a "consumption"
        # (bridge_to_server.py), but nothing there scales it, documents a unit
        # or bounds it -- the name is an assumption, and the data does not
        # support it. Reported as a diagnostic byte only; what it actually means
        # needs the motor controller's manual.
        self.wheel_status_byte7 = None
        self.wheel_status_stamp = None
        self.follower_status = None
        self.robot_fault = None
        # navigation view
        self.pose_x = None
        self.pose_y = None
        self.pose_yaw_deg = None
        self.pose_stamp = None
        self.loc_fitness = None
        self.loc_inlier_ratio = None
        self.loc_reason = None
        self.wp_index = None
        self.wp_total = None
        self.follower_state = None
        # FAST-LIO's /Odometry. Pose only: laserMapping publishes an all-zero
        # twist, verified on the moving chair 2026-08-23, so this is NOT a speed
        # source. Kept for freshness, and as a fallback if wheel odometry dies.
        self.odom_speed_mps = None
        self.odom_yaw_rate = None
        self.odom_stamp = None
        # /odom from base_model's odom_pub.py: encoder speed integrated in the
        # world frame, 100 Hz. This is the one that knows the chair is rolling.
        self.wheel_odom_speed_mps = None
        self.wheel_odom_yaw_rate = None
        self.wheel_odom_stamp = None
        self.cmd_raw = 0.0              # what the follower wants
        self.cmd_gated = 0.0            # what safety_gate allows
        self.cmd_out = 0.0              # what tip_guard sends to the wheels
        self.cmd_raw_stamp = None
        self.tip_guard_status = None
        self.tip_guard_stamp = None
        self.localization_status = None
        self.localization_stamp = None
        self.objects_summary = None
        self.objects_stamp = None
        # obstacle_clusters.py publishes far more than the one line the app used
        # to show. What an operator needs to know is not "2 clusters" but
        # whether anything is in the corridor and how close it is, so the fields
        # that answer that are carried through instead of being flattened away.
        self.objects_status = None
        self.band_status = None
        self.objects_counts = None
        self.objects_in_band = None
        self.objects_nearest_m = None
        self.objects_nearest_in_band_m = None
        self.bloom_filtered = None
        # The nearest few clusters with enough geometry to draw them. A count
        # and a distance tell an operator that something is there; only this
        # tells them what the chair is looking at, and whether the thing in
        # the corridor walked in or was always parked in it.
        self.objects_list = None
        # Which axes objects_list is on, in the producer's own words. Never
        # guessed: two nodes publish this topic and they do not agree.
        self.objects_frame = None
        # How many clusters the display filter and the count cap left out, so
        # the app can say the view is trimmed rather than imply a quiet field.
        self.objects_hidden = None
        # Roll and pitch out of the localizer's own pose. FAST-LIO builds the
        # map gravity-aligned, so tilt in the map frame is the chair's tilt --
        # and nothing else on this stack publishes an angle at all. tip_guard,
        # despite the name, only reports OK/STALE and the speed it is passing.
        self.pose_roll_deg = None
        self.pose_pitch_deg = None
        self.follower_stamp = None
        self.estop_requested_at = None
        self.last_command_detail = None

    def _fresh(self, value, stamp, now):
        """The value, or None once its publisher has gone quiet."""
        if stamp is None or now - stamp > FIELD_TTL_S:
            return None
        return value

    def ground_speed(self, now=None):
        """Best available answer to "is the chair moving", and where it came from.

        Wheel odometry first. /Odometry looks like the natural source and is what
        this bridge used to read, but FAST-LIO leaves its twist at zero -- so the
        speed tile read 0.0 while the chair drove at 0.30 m/s, and the guard that
        refuses an E-STOP release on a rolling chair could never fire. Caller must
        hold the lock.

        :returns: ``(speed_mps, yaw_rate_radps, source)`` where source is
                  ``"wheel"``, ``"lio"`` or ``None`` when nothing is fresh.
        """
        now = time.time() if now is None else now
        if (self.wheel_odom_stamp is not None
                and now - self.wheel_odom_stamp <= self.SPEED_TTL_S):
            return self.wheel_odom_speed_mps, self.wheel_odom_yaw_rate, "wheel"
        if (self.odom_stamp is not None
                and now - self.odom_stamp <= self.SPEED_TTL_S):
            return self.odom_speed_mps, self.odom_yaw_rate, "lio"
        return None, None, None

    def snapshot(self, ros_connected, ttl_s, follower_available):
        with self.lock:
            self.seq += 1
            now = time.time()

            def age(stamp):
                return None if stamp is None else round(now - stamp, 2)

            wheel_age = age(self.wheel_status_stamp)
            wheel_link_ok = wheel_age is not None and wheel_age <= ttl_s
            speed, yaw_rate, speed_source = self.ground_speed(now)
            # A reading whose publisher has gone quiet is not a reading.
            localization = self._fresh(self.localization_status,
                                       self.localization_stamp, now)
            objects = self._fresh(self.objects_summary, self.objects_stamp, now)
            tip_guard = self._fresh(self.tip_guard_status, self.tip_guard_stamp, now)
            follower = self._fresh(self.follower_status, self.follower_stamp, now)
            follower_live = follower is not None

            # safety_gate holds motion by zeroing its output while the follower is
            # still asking for movement. The gate does not publish its reason, so
            # infer the hold rather than patching a field node to expose it.
            raw_fresh = self.cmd_raw_stamp is not None and now - self.cmd_raw_stamp <= ttl_s
            motion_blocked = bool(raw_fresh and self.cmd_raw > MOTION_EPS
                                  and self.cmd_gated <= MOTION_EPS)

            # Manual mode is NOT the same as "the app e-stopped the chair".
            # start_wheelchair_localization.sh finishes with the base in manual and
            # the follower paused -- that is the normal resting state, and calling
            # it E-STOP made the app shout after every clean bring-up. It is also
            # what the joystick failsafe produces. Only claim E-STOP when this
            # bridge actually commanded one and the controller echoed it back.
            in_manual = self.drive_mode == MANUAL_MODE
            estopped = in_manual and self.estop_requested_at is not None
            pending = (self.estop_requested_at is not None
                       and not in_manual
                       and now - self.estop_requested_at < 3.0)

            frame = {
                "protocol_version": PROTOCOL_VERSION,
                "seq": self.seq,
                "timestamp": now,
                "bridge_uptime_s": round(now - self.started_at, 1),
                "ros_connected": ros_connected,

                "speed_mps": speed,
                "yaw_rate_radps": yaw_rate,
                "speed_source": speed_source,
                "commanded_mps": self.cmd_out,

                "drive_mode": MODE_LABELS.get(self.drive_mode),
                "drive_mode_raw": self.drive_mode,
                "estop_engaged": estopped,
                "estop_pending": pending,
                # Manual without an app e-stop: joystick failsafe, or simply not
                # armed yet after bring-up. Different message, same "won't drive".
                "manual_idle": in_manual and self.estop_requested_at is None,
                "motion_blocked": motion_blocked,
                "follower_start_available": follower_available,

                "wheel_link_ok": wheel_link_ok,
                "wheel_status_age_s": wheel_age,
                "odom_age_s": age(self.odom_stamp),
                "tip_guard_status": tip_guard,
                "follower_status": follower,
                "localization_status": localization,
                "localization_tracking": localization == "TRACKING",
                "objects_summary": objects,
                "objects_status": self._fresh(self.objects_status,
                                              self.objects_stamp, now),
                "band_status": self._fresh(self.band_status,
                                           self.objects_stamp, now),
                "objects_counts": self._fresh(self.objects_counts,
                                              self.objects_stamp, now),
                "objects_in_band": self._fresh(self.objects_in_band,
                                               self.objects_stamp, now),
                "objects_nearest_m": self._fresh(self.objects_nearest_m,
                                                 self.objects_stamp, now),
                "objects_nearest_in_band_m": self._fresh(
                    self.objects_nearest_in_band_m, self.objects_stamp, now),
                "bloom_filtered": self._fresh(self.bloom_filtered,
                                              self.objects_stamp, now),
                # The nearest clusters themselves, for the close-in view.
                # Chair-aligned and x-forward, y-left, but WHICH origin is
                # the producer's to say -- obstacle_clusters.py answers
                # "lidar", hybrid_object_fusion.py answers "chair_centre",
                # and they sit about half a metre apart. Either way it is not
                # the map frame pose_x/pose_y use. Null means the producer
                # did not say, which the app must not read as "lidar".
                "objects": self._fresh(self.objects_list,
                                       self.objects_stamp, now),
                "objects_frame": self._fresh(self.objects_frame,
                                             self.objects_stamp, now),
                "objects_hidden": self._fresh(self.objects_hidden,
                                              self.objects_stamp, now),
                "robot_fault": self.robot_fault,

                # Navigation view. Pose is the same /fast_lio_icp/pose the route
                # was captured from, so route and pose share one frame and can be
                # drawn on the same axes without any transform.
                "pose_x": self.pose_x,
                "pose_y": self.pose_y,
                "pose_yaw_deg": self.pose_yaw_deg,
                "pose_roll_deg": self.pose_roll_deg,
                "pose_pitch_deg": self.pose_pitch_deg,
                "pose_age_s": age(self.pose_stamp),
                "loc_fitness": self._fresh(self.loc_fitness,
                                           self.localization_stamp, now),
                "loc_inlier_ratio": self._fresh(self.loc_inlier_ratio,
                                                self.localization_stamp, now),
                "loc_reason": self._fresh(self.loc_reason,
                                          self.localization_stamp, now),
                "wp_index": self.wp_index if follower_live else None,
                "wp_total": self.wp_total if follower_live else None,
                "follower_state": self.follower_state if follower_live else None,
                "last_command_detail": self.last_command_detail,

                # No node on this stack measures charge; see battery note above.
                "battery_percent": None,
                "wheel_status_byte7": self.wheel_status_byte7,
                "step_level": None,
            }
            # go.sh refuses to start unless all of these hold. Mirror it so the
            # app can grey out the drive button for the same reasons.
            frame["ready_to_drive"] = bool(
                wheel_link_ok
                and self.drive_mode == AUTO_MODE
                and localization == "TRACKING"
                and objects is not None
                and not motion_blocked)
            frame["unavailable"] = sorted(k for k, v in frame.items() if v is None)
            # Fail closed: unknown reads as not-driving-safely.
            frame["display_safe_to_drive"] = bool(
                wheel_link_ok and self.drive_mode == AUTO_MODE and not motion_blocked)
            return frame


class RosLink:
    def __init__(self, state, allow_commands, node_name):
        self.state = state
        self.allow_commands = allow_commands
        self.node_name = node_name
        self.connected = False
        self.mode_pub = None
        self._master_node_cache = None
        self._blackbox_stamp = 0.0
        # Where the objects have been. Kept here rather than in BridgeState
        # because it is working memory of one subscription, not a reading the
        # app is ever shown: what crosses the link is the path it produces.
        self.trails = TrailMemory()

    # rosbag record names itself /record_<stamp> and subscribes to every topic
    # it writes, so the master's own tables are the cheapest place to see it --
    # no extra topic, and nothing to keep in step with the bring-up script's
    # record line. Polled rather than watched: it is an XML-RPC round trip and
    # the answer changes about once a session.
    BLACKBOX_POLL_S = 5.0

    # The field stack's own nodes, for answering "is it already up".
    #
    # The job runner cannot answer that: it knows only what the APP started,
    # so a stack brought up at the keyboard -- which is most of them -- left
    # the dashboard saying 대기 중 and offering [로컬 켜기] next to a chair
    # that was already running, on a master where every one of these was
    # registered. Verified on the NUC 2026-08-27.
    #
    # One from each layer that has to be alive for the chair to drive, so a
    # half-started stack cannot read as a whole one: the localizer, the gate,
    # the tip guard, the wheel encoder and the UART.
    STACK_NODES = ("/moving_icp_localizer", "/safety_gate", "/tip_guard",
                   "/wheel_cmd", "/uart")

    def _master_nodes(self):
        """Every node the master knows, cached, or None when it will not say.

        One query answers both the recording check and the stack check; they
        used to be one call each at the same 5 s period, on a link where the
        round trip is the expensive part.
        """
        if not (ROS_AVAILABLE and self.connected):
            return None
        now = time.time()
        if now - self._blackbox_stamp < self.BLACKBOX_POLL_S:
            return self._master_node_cache
        self._blackbox_stamp = now
        try:
            # MasterProxy injects the caller id itself; passing one is a
            # "bad call arity" fault that returns code -1, which the
            # except-less path below reads as "cannot tell" forever.
            code, _, state = rospy.get_master().getSystemState()
            if code != 1:
                return self._master_node_cache
            nodes = set()
            for section in state:          # publishers, subscribers, services
                for _name, owners in section:
                    nodes.update(owners)
            self._master_node_cache = nodes
        except Exception:
            # A master that will not answer is not evidence of anything.
            return self._master_node_cache
        return self._master_node_cache

    def blackbox_recording(self):
        """True, False, or None when the question cannot be answered.

        None rather than False on any failure. "No recording" and "could not
        ask" look identical on a dashboard and mean opposite things: the first
        is worth acting on before a run, the second is worth ignoring.
        """
        nodes = self._master_nodes()
        if nodes is None:
            return None
        return any(n.startswith("/record_") for n in nodes)

    def stack_nodes_up(self):
        """How many of STACK_NODES the master has, or None if it will not say.

        Deliberately a count rather than a verdict. "The stack is up" is one
        word for a thing that comes up in layers and dies in pieces, and 3 of
        5 is the state an operator most needs to see -- it is the one that
        looks like a working chair right up until it is asked to move.
        """
        nodes = self._master_nodes()
        if nodes is None:
            return None
        return sum(1 for name in self.STACK_NODES if name in nodes)

    # ------------------------------------------------------------------ setup
    @staticmethod
    def master_online(timeout=1.5):
        """Is a ROS master actually listening?

        rospy.init_node() blocks indefinitely when the master is absent -- it sits
        in select() retrying, so the bridge never reaches its first log line and
        never registers the SPP profile. That is fatal here, because the intended
        field workflow is: start the bridge, connect the phone, THEN press
        [로컬 켜기] to launch the stack (which is what starts roscore). The bridge
        must therefore come up happily with no master and attach later.
        """
        uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
        try:
            hostport = uri.split("//", 1)[1]
            host, _, port = hostport.partition(":")
            port = int(port.rstrip("/") or 11311)
        except (IndexError, ValueError):
            return False
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.settimeout(timeout)
        try:
            probe.connect((host, port))
            return True
        except OSError:
            return False
        finally:
            probe.close()

    def start(self):
        if not ROS_AVAILABLE:
            log("rospy not importable -- protocol-only mock server.")
            return
        if not self.master_online():
            log("no ROS master yet -- serving without ROS and retrying in "
                "the background (use the app's [로컬 켜기] to start the stack).")
            threading.Thread(target=self._await_master, daemon=True).start()
            return
        self._connect_ros()

    def _await_master(self):
        while not self.connected:
            time.sleep(3.0)
            if self.master_online():
                log("ROS master appeared -- attaching.")
                self._connect_ros()

    def _connect_ros(self):
        try:
            rospy.init_node(self.node_name, anonymous=False, disable_signals=True)
            rospy.Subscriber("/wheel_status", Int16MultiArray, self._wheel_cb, queue_size=5)
            rospy.Subscriber("/Odometry", Odometry, self._odom_cb, queue_size=1)
            rospy.Subscriber("/odom", Odometry, self._wheel_odom_cb, queue_size=1)
            rospy.Subscriber("/cmd_vel_raw", Twist, self._raw_cb, queue_size=1)
            rospy.Subscriber("/cmd_vel_gated", Twist, self._gated_cb, queue_size=1)
            rospy.Subscriber("/cmd_vel", Twist, self._out_cb, queue_size=1)
            rospy.Subscriber("/tip_guard/status", String, self._tip_cb, queue_size=2)
            rospy.Subscriber("/waypoint_follower/status", String,
                             self._follower_status_cb, queue_size=2)
            rospy.Subscriber("/perception/objects_summary", String, self._objects_cb, queue_size=2)
            rospy.Subscriber("/robot_fault", Int16MultiArray, self._fault_cb, queue_size=2)
            rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                             self._pose_cb, queue_size=1)
            if DIAGNOSTICS_AVAILABLE:
                rospy.Subscriber("/fast_lio_icp/localization_diagnostics",
                                 DiagnosticArray, self._diag_cb, queue_size=5)

            if self.allow_commands:
                # The one and only publisher. uart.py does not check callerid here.
                # Absolute name, matching what go.sh publishes.
                self.mode_pub = rospy.Publisher("/mode_cmd", Int16, queue_size=1)
                log("command mode ON -> publishes mode_cmd only")
            else:
                log("observer mode -- no publishers created.")
            self.connected = True
            log("ROS node '%s' up." % self.node_name)
        except Exception as exc:                                  # noqa: BLE001
            log("ROS init failed (%s: %s) -- continuing without ROS."
                % (type(exc).__name__, exc))

    # -------------------------------------------------------------- callbacks
    def _wheel_cb(self, msg):
        with self.state.lock:
            self.state.wheel_status_stamp = time.time()
            if len(msg.data) > 1:
                self.state.drive_mode = int(msg.data[1])
            if len(msg.data) > 7:
                self.state.wheel_status_byte7 = int(msg.data[7])

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw only; the view is top-down so roll/pitch are not wanted
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        with self.state.lock:
            self.state.pose_x = round(p.x, 3)
            self.state.pose_y = round(p.y, 3)
            self.state.pose_yaw_deg = round(math.degrees(math.atan2(siny, cosy)), 1)
            self.state.pose_roll_deg = round(math.degrees(math.atan2(sinr, cosr)), 1)
            self.state.pose_pitch_deg = round(math.degrees(math.asin(sinp)), 1)
            self.state.pose_stamp = time.time()

    # waypoint_follower.py publishes "%s wp=%d/%d v=%.2f%s"; parse it rather than
    # making the app do string surgery. A blocking reason arrives in place of the
    # state word (MANUAL_MODE, BASE_STALE, CLUSTERS_STALE, HOLD:...).
    _STATUS_RE = re.compile(r"^(?P<state>\S+)\s+wp=(?P<i>\d+)/(?P<n>\d+)")

    def _follower_status_cb(self, msg):
        text = msg.data[:120]
        match = self._STATUS_RE.match(text)
        with self.state.lock:
            self.state.follower_status = text
            self.state.follower_stamp = time.time()
            if match:
                self.state.follower_state = match.group("state")
                self.state.wp_index = int(match.group("i"))
                self.state.wp_total = int(match.group("n"))
            else:
                self.state.follower_state = text.split()[0] if text.split() else None

    def _fault_cb(self, msg):
        # fault_check.py order: scan, odom, imu, roll, pitch
        names = ("scan", "odom", "imu", "roll", "pitch")
        active = [n for n, v in zip(names, msg.data) if v]
        with self.state.lock:
            self.state.robot_fault = ",".join(active) if active else "none"

    def _odom_cb(self, msg):
        v = msg.twist.twist
        speed = (v.linear.x ** 2 + v.linear.y ** 2) ** 0.5
        with self.state.lock:
            self.state.odom_speed_mps = round(speed, 3)
            self.state.odom_yaw_rate = round(v.angular.z, 3)
            self.state.odom_stamp = time.time()

    def _wheel_odom_cb(self, msg):
        # odom_pub.py integrates the encoder speed in the world frame, so linear
        # x and y are both populated and the magnitude is the ground speed.
        v = msg.twist.twist
        speed = (v.linear.x ** 2 + v.linear.y ** 2) ** 0.5
        with self.state.lock:
            self.state.wheel_odom_speed_mps = round(speed, 3)
            self.state.wheel_odom_yaw_rate = round(v.angular.z, 3)
            self.state.wheel_odom_stamp = time.time()

    def _raw_cb(self, msg):
        with self.state.lock:
            self.state.cmd_raw = round(twist_magnitude(msg), 3)
            self.state.cmd_raw_stamp = time.time()

    def _gated_cb(self, msg):
        with self.state.lock:
            self.state.cmd_gated = round(twist_magnitude(msg), 3)

    def _out_cb(self, msg):
        with self.state.lock:
            self.state.cmd_out = round(msg.linear.x, 3)

    def _tip_cb(self, msg):
        with self.state.lock:
            self.state.tip_guard_status = msg.data
            self.state.tip_guard_stamp = time.time()

    # How stale the pose may be before it can no longer place an object.
    # /fast_lio_icp/pose arrives at 10 Hz and the objects at 5; half a second
    # of it missing is a localizer in trouble, and a path drawn about a pose
    # from a second ago is a path drawn in the wrong place.
    TRAIL_POSE_TTL_S = 0.5

    def _trail_pose(self, now):
        """The chair's pose as ``(x, y, yaw_deg)``, or None if it is not fit
        to place anything about.

        Trails are stored in the map frame and drawn about where the chair is
        now, so this is the transform at both ends. Returning None where the
        pose is stale is what makes the app show plain dots during a
        localization drop-out rather than paths bent by the error.
        """
        with self.state.lock:
            x, y = self.state.pose_x, self.state.pose_y
            yaw = self.state.pose_yaw_deg
            stamp = self.state.pose_stamp
        if None in (x, y, yaw) or stamp is None:
            return None
        if (now - stamp) > self.TRAIL_POSE_TTL_S:
            return None
        return (float(x), float(y), float(yaw))

    def _objects_cb(self, msg):
        """obstacle_clusters.py publishes a JSON blob; keep what steers a decision.

        The old reduction threw away the part that matters. "클러스터 2" says
        nothing about whether either of them is in the chair's way, and the band
        relation per object is exactly that: objects whose returns fall inside
        the safety band are the ones the follower holds for.
        """
        text = msg.data
        status = band = counts = None
        in_band = nearest = nearest_in_band = bloom = None
        drawable = frame = hidden = None
        now = time.time()
        pose = self._trail_pose(now)
        try:
            blob = json.loads(text)
            status = blob.get("status")
            # The producer's own word for which axes these are on, carried
            # through rather than asserted here. This bridge assumed "lidar",
            # which was true of obstacle_clusters.py and false the moment a
            # second producer appeared: hybrid_object_fusion.py publishes the
            # same shape in "chair_centre", half a metre away. A view that
            # believes a hardcoded frame draws the pavement in the wrong
            # place and has no way to find out.
            frame = blob.get("frame")
            band = blob.get("band_status")
            bloom = blob.get("bloom_filtered")
            counts = {k: v for k, v in (blob.get("counts") or {}).items()
                      if isinstance(v, int) and v}
            objects = blob.get("objects") or []
            in_band = 0
            by_distance = []
            for obj in objects:
                try:
                    x, y = float(obj["x"]), float(obj["y"])
                except (KeyError, TypeError, ValueError):
                    continue
                distance = math.hypot(x, y)
                by_distance.append((distance, x, y, obj))
                if nearest is None or distance < nearest:
                    nearest = distance
                if obj.get("band_relation") in ("inside", "overlap"):
                    in_band += 1
                    if nearest_in_band is None or distance < nearest_in_band:
                        nearest_in_band = distance
                # Every object is remembered, not only the ten that get drawn:
                # a path has to already exist by the time something walks in
                # from the far half of the scene, which is exactly when it
                # matters most.
                if obj.get("motion") != "static":
                    self.trails.observe(obj.get("id"), x, y, pose, now)
            self.trails.sweep(now)
            # Ground first, then nearest, then cut. Filtering before the cut
            # is the point: ten specks of paving would otherwise fill the ten
            # slots and push a person off the end of the list.
            worth = [item for item in by_distance if worth_drawing(item[3])]
            hidden = len(by_distance) - len(worth)
            worth.sort(key=lambda item: item[0])
            drawable = [drawable_object(obj, x, y,
                                        self.trails.trail(obj.get("id"), pose))
                        for _distance, x, y, obj in worth[:OBJECT_MAX_COUNT]]
            # Everything the app is not being shown, counted. A filtered view
            # that does not say it is filtered is a view that lies quietly.
            hidden += max(0, len(worth) - OBJECT_MAX_COUNT)
            # Counted again under the labels the app will print. The producer
            # counts its own classification, so leaving its tally alone would
            # put "차량 3" above a view drawing three walls as obstacles, and
            # a dashboard that contradicts itself gets believed at the wrong
            # moment. Only the display label moves; nothing is added or lost,
            # and the tally covers every object rather than only the ones
            # that could be placed -- a cluster the view cannot draw is still
            # a cluster the perception node is tracking.
            relabelled = {}
            for obj in objects:
                label = display_class(obj)
                relabelled[label] = relabelled.get(label, 0) + 1
            if objects:
                counts = {k: v for k, v in relabelled.items() if v}
            parts = [str(status or "?")]
            if band and band != status:
                parts.append("band %s" % band)
            if in_band:
                parts.append("회랑 %d" % in_band)
            if nearest is not None:
                parts.append("최근접 %.1fm" % nearest)
            text = " · ".join(parts)
        except (ValueError, TypeError, AttributeError):
            pass
        with self.state.lock:
            self.state.objects_summary = text[:80]
            self.state.objects_status = status
            self.state.band_status = band
            self.state.objects_counts = counts
            self.state.objects_in_band = in_band
            self.state.objects_nearest_m = (None if nearest is None
                                            else round(nearest, 1))
            self.state.objects_nearest_in_band_m = (
                None if nearest_in_band is None else round(nearest_in_band, 1))
            self.state.bloom_filtered = bloom
            self.state.objects_list = drawable
            self.state.objects_frame = frame
            self.state.objects_hidden = hidden
            self.state.objects_stamp = time.time()

    def _diag_cb(self, msg):
        try:
            worst = None
            for status in msg.status:
                if worst is None or status.level > worst.level:
                    worst = status
            if worst is not None:
                # go.sh gates the drive on this message being exactly "TRACKING",
                # so store it verbatim rather than decorating it with a level.
                fields = {}
                for item in getattr(worst, "values", []):
                    fields[item.key] = item.value
                with self.state.lock:
                    self.state.localization_status = worst.message.strip()[:80]
                    self.state.localization_stamp = time.time()
                    # The localizer emits a sentinel (1e9) for fitness while ICP
                    # correction is suppressed -- e.g. STATIONARY_CORRECTION_
                    # SUPPRESSED, which is normal for a parked chair. Printing
                    # "1000000000.000" on the dashboard is worse than printing
                    # nothing, so drop it and let `reason` carry the meaning.
                    fitness = _as_float(fields.get("fitness"))
                    inlier = _as_float(fields.get("inlier_ratio"))
                    suppressed = fitness is not None and fitness >= 1e6
                    self.state.loc_fitness = None if suppressed else fitness
                    self.state.loc_inlier_ratio = (
                        None if suppressed or not inlier else inlier)
                    reason = (fields.get("reason") or "").strip()
                    self.state.loc_reason = reason[:60] or None
        except Exception:                                         # noqa: BLE001
            pass

    # --------------------------------------------------------------- commands
    def _publish_mode(self, value):
        if self.mode_pub is None:
            # Two very different causes, and telling them apart matters: one is a
            # flag, the other is "the robot software is not running yet". The old
            # message blamed the flag for both and sent people hunting the wrong
            # thing in the field.
            if not self.allow_commands:
                return False, "이 브릿지는 명령이 꺼져 있습니다 (--allow-commands off)"
            if not self.connected:
                return False, ("ROS 마스터가 아직 없습니다 — [로컬 켜기]로 스택을 "
                               "먼저 기동하세요. 기동되면 자동으로 붙습니다.")
            return False, "mode_cmd 퍼블리셔가 없습니다 (브릿지 내부 오류)"
        self.mode_pub.publish(Int16(data=value))
        return True, "mode_cmd=%d published" % value

    def engage_estop(self, mark_estop=True):
        """mode_cmd=77 first, then pause the follower -- exactly what stop.sh does.

        Pausing matters more than it looks. waypoint_follower.py holds on
        MANUAL_MODE (line ~826) but stays ``enabled``; it never disables itself.
        So mode 77 alone stops the chair, and then mode 65 would let the follower
        resume *the instant the e-stop is released* -- the release itself would
        drive off. stop.sh calls the service for exactly this reason.

        Order is deliberate: publishing the topic is instant and cannot fail, so
        it happens before the service call, and a missing service never blocks
        the stop.

        ``mark_estop`` decides whether the app is told an EMERGENCY happened,
        and it is the only difference between this and an ordinary [주행 정지].
        Both do the same two acts, because stop.sh does the same two acts; only
        the label differs, and the label matters: ``estop_engaged`` greys out
        the drive controls and points the rider at [E-STOP 해제]. Calling an
        ordinary stop an e-stop sent someone looking for an emergency that
        never happened.
        """
        ok, detail = self._publish_mode(MANUAL_MODE)
        if not ok:
            return ok, detail
        if mark_estop:
            with self.state.lock:
                self.state.estop_requested_at = time.time()
        paused, pause_detail = self.set_follower(False)
        return True, ("%s (mode_cmd=77). uart.py가 모터 정지 프레임을 보내고 "
                      "자율 명령을 무시합니다. 팔로워 %s"
                      % ("E-STOP 발동" if mark_estop else "주행 정지",
                         "정지됨" if paused else "정지 실패(%s) — 해제 전에 확인 필요"
                         % pause_detail[:60]))

    def release_estop(self):
        """mode_cmd=65. Arms the base; does not start the drive.

        It used to take a ``resume`` flag and call the follower's start service
        itself, because the operator asked for one button that arms and drives.
        They still get one button -- Session._arm_and_drive is that button --
        but the driving half of it now goes down the same path as [주행 시작],
        through go.sh and every preflight behind it. Starting the follower from
        here meant the app's ordinary way of setting off was also the only way
        that skipped all of them.
        """
        ok, detail = self._publish_mode(AUTO_MODE)
        if not ok:
            return ok, detail
        with self.state.lock:
            self.state.estop_requested_at = None
        return True, ("released (mode_cmd=65). A stop frame is sent first, so "
                      "the chair does not lurch. Driving stays stopped.")

    def await_auto_echo(self, timeout_s=2.0):
        """Block until the motor controller echoes auto on /wheel_status.

        Design rule 5: a command is effective when the echo agrees, not when we
        publish it. uart.py transmits a stop frame on entering auto and only
        then begins accepting wheel_cmd, so a drive started inside that window
        runs with every command discarded -- the follower reports DRIVING, the
        dashboard is green, and the chair does not move. That is the most
        expensive state to diagnose in the field, so it is worth two seconds.
        """
        deadline = time.time() + timeout_s
        while True:
            now = time.time()
            with self.state.lock:
                mode = self.state.drive_mode
                stamp = self.state.wheel_status_stamp
            if stamp is not None and (now - stamp) <= 2.0 and mode == AUTO_MODE:
                return True, "auto echoed by the motor controller"
            if now >= deadline:
                if stamp is None:
                    return False, "/wheel_status has never arrived"
                if (now - stamp) > 2.0:
                    return False, "/wheel_status went silent %.1fs ago" % (now - stamp)
                return False, ("controller still echoing %s"
                               % MODE_LABELS.get(mode, "mode=%s" % mode))
            time.sleep(0.1)

    def set_follower(self, running, ensure_auto=False):
        if not self.allow_commands:
            return False, "이 브릿지는 명령이 꺼져 있습니다 (--allow-commands off)"
        if not ROS_AVAILABLE:
            return False, "rospy를 쓸 수 없습니다"
        if not self.connected:
            return False, ("ROS 마스터가 아직 없습니다 — [로컬 켜기]로 스택을 먼저 "
                           "기동하세요")
        if ensure_auto:
            # go.sh publishes /mode_cmd 65 before calling the service. Without
            # this the follower would start while uart.py is still in manual and
            # discarding every wheel_cmd -- the chair looks armed and does not move.
            self._publish_mode(AUTO_MODE)
            time.sleep(0.3)          # let uart.py transmit its stop frame first
        try:
            rospy.wait_for_service(FOLLOWER_START_SERVICE, timeout=2.0)
            proxy = rospy.ServiceProxy(FOLLOWER_START_SERVICE, SetBool)
            response = proxy(running)
            return bool(response.success), str(response.message)[:160]
        except Exception as exc:                                  # noqa: BLE001
            return False, ("%s unavailable (%s). Only the pursuit profile "
                           "(waypoint_follower.py) offers it; mpc/dwa do not."
                           % (FOLLOWER_START_SERVICE, type(exc).__name__))

    def follower_available(self):
        if not (ROS_AVAILABLE and self.connected):
            return False
        try:
            import rosservice                                     # noqa: PLC0415
            return FOLLOWER_START_SERVICE in rosservice.get_service_list()
        except Exception:                                         # noqa: BLE001
            return False


class Session:
    """One connected phone. A bad command costs one reply, never the link."""

    def __init__(self, sock, state, ros, ttl_s, rate_hz, jobs=None, route=None,
                 route_finder=None):
        self.sock = sock
        self.state = state
        self.ros = ros
        self.jobs = jobs
        self.route = route
        self.route_finder = route_finder
        self.ttl_s = ttl_s
        self.period = 1.0 / rate_hz
        self.stop = threading.Event()
        self.write_lock = threading.Lock()
        self._follower_cache = (0.0, False)

    def send(self, obj):
        line = (json.dumps(obj) + "\n").encode("utf-8")
        try:
            with self.write_lock:
                self.sock.sendall(line)
            return True
        except OSError:
            self.stop.set()
            return False

    def _follower_available(self):
        now = time.time()
        stamp, value = self._follower_cache
        if now - stamp > 5.0:                 # service lookups are not free
            value = self.ros.follower_available()
            self._follower_cache = (now, value)
        return value

    def broadcast(self):
        # Resolve here rather than at start-up. The intended workflow is: connect
        # the app FIRST, then press [로컬 켜기] to launch the stack -- so at process
        # start the follower does not exist yet and its route param is unset.
        # Resolving once at boot would pin whatever the CLI default happens to be
        # and then keep showing it after the real route became known.
        if self.route is None and self.route_finder is not None:
            self.route = self.route_finder()
        # Route first, so the app can draw the map before any pose arrives.
        if self.route is not None:
            self.send(self.route)
        while not self.stop.is_set():
            frame = self.state.snapshot(self.ros.connected, self.ttl_s,
                                        self._follower_available())
            frame["type"] = "telemetry"
            # Whether this run is being recorded. Finding out afterwards that
            # it was not is the one thing that cannot be fixed afterwards.
            frame["blackbox_recording"] = self.ros.blackbox_recording()
            # Whether the robot software is already running, asked of the ROS
            # master rather than of the job runner -- which only ever knew
            # what this app itself launched.
            up = self.ros.stack_nodes_up()
            frame["stack_nodes_up"] = up
            frame["stack_nodes_total"] = len(self.ros.STACK_NODES)
            frame["stack_running"] = None if up is None else \
                up >= len(self.ros.STACK_NODES)
            if self.jobs is not None:
                frame.update(self.jobs.snapshot())
                frame["jobs_available"] = self.jobs.available()
                frame["scripts_enabled"] = self.jobs.enabled
                # Which controller [로컬 켜기] would actually bring up. The app
                # shows it on the button, because "start the stack" means a
                # different robot depending on this one word.
                frame["stack_profile"] = self.jobs.env_overlay.get(
                    "PROFILE", os.environ.get("PROFILE", "pursuit"))
            if not self.send(frame):
                return
            self.stop.wait(self.period)

    def serve(self):
        broadcaster = threading.Thread(target=self.broadcast, daemon=True)
        broadcaster.start()
        buffer = b""
        try:
            while not self.stop.is_set():
                try:
                    chunk = self.sock.recv(4096)
                except BlockingIOError:
                    # Belt and braces if the descriptor is non-blocking anyway:
                    # EAGAIN means "nothing yet", not "the link is gone".
                    time.sleep(0.05)
                    continue
                except OSError as exc:
                    log("link read error: %s" % exc)
                    break
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > 64 * 1024:
                    log("dropping oversized command buffer")
                    buffer = b""
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    text = raw.decode("utf-8", errors="replace").strip()
                    if text:
                        self.send(self.handle(text))
        finally:
            self.stop.set()
            broadcaster.join(timeout=1.0)

    def handle(self, text):
        log("RX %s" % text)
        reply = {"type": "ack", "request": text[:200], "ok": False, "detail": ""}
        try:
            payload = json.loads(text)
        except ValueError:
            reply["detail"] = "not valid JSON"
            return reply
        if not isinstance(payload, dict):
            reply["detail"] = "expected a JSON object"
            return reply

        command = payload.get("command") or payload.get("action")
        reply["command"] = command
        try:
            if command in ("stop", "estop"):
                ok, detail = self.ros.engage_estop()
            elif command in ("estop_release", "release", "rearm", "arm"):
                ok, detail = self._release(payload)
            elif command == "arm_and_drive":
                ok, detail = self._arm_and_drive(payload)
            elif command == "drive_start":
                ok, detail = self._drive(payload, True)
            elif command == "drive_stop":
                ok, detail = self._halt()
            elif command == "stack_start":
                ok, detail = self._stack_start(payload)
            elif command == "stack_stop":
                ok, detail = self._stack_stop(payload)
            elif command == "job_cancel":
                ok, detail = (self.jobs.cancel() if self.jobs is not None
                              else (False, "스크립트 실행이 설정되지 않았습니다"))
            elif command == "route":
                ok, detail = self._resend_route()
            elif command == "ping":
                ok, detail = True, "pong"
            elif command in ("mode", "step"):
                ok, detail = False, (
                    "not applicable to this stack. Drive mode is auto/manual via "
                    "mode_cmd and is controlled by estop/estop_release; there is "
                    "no step level.")
            else:
                ok, detail = False, "unknown command %r" % (command,)
            reply["ok"], reply["detail"] = ok, detail
            with self.state.lock:
                self.state.last_command_detail = ("%s: %s" % (command, detail))[:160]
            return reply
        except Exception as exc:                                  # noqa: BLE001
            reply["detail"] = "%s: %s" % (type(exc).__name__, exc)
            return reply

    def _resend_route(self):
        """Re-resolve and re-send the route frame.

        The route goes out once per connection, ahead of any telemetry, so a
        client that was not listening yet never sees it -- which is exactly what
        happened: the device picker owns the shared Bluetooth callback until the
        dashboard swaps itself in a moment later, and its onLineReceived is
        empty. The map then stayed blank for the whole session while telemetry
        streamed happily, because telemetry repeats and the route did not.

        Re-resolving rather than replaying the cached copy also picks up the real
        route once the stack is up: connect first, press [로컬 켜기], and the
        follower's own param finally exists.
        """
        if self.route_finder is None:
            return False, "경로가 설정되지 않았습니다 (--route \"\")"
        route = self.route_finder()
        if route is None:
            return False, "경로 파일을 읽지 못했습니다"
        self.route = route
        if not self.send(route):
            return False, "경로 전송 실패"
        return True, "경로 %d개 지점 전송 (%s)" % (route.get("count_full", 0),
                                              route.get("source", "?"))

    def _release(self, payload):
        """Two-step: the app must send confirm=true, and the chair must be stopped."""
        if not payload.get("confirm"):
            return False, ("confirmation required: resend with \"confirm\": true "
                           "after the rider has checked it is safe to proceed")
        with self.state.lock:
            moving, _yaw, source = self.state.ground_speed()
            link_age = (None if self.state.wheel_status_stamp is None
                        else time.time() - self.state.wheel_status_stamp)
        if link_age is None or link_age > self.ttl_s:
            return False, "refusing: no fresh /wheel_status, cannot confirm the chair is stopped"
        # Fail closed. "No speed reading" used to sail straight past this check,
        # which is the same as asserting the chair is stopped on no evidence.
        if moving is None:
            return False, ("refusing: no fresh speed reading (/odom, /Odometry) — "
                           "cannot confirm the chair is stopped")
        if moving > 0.05:
            return False, ("refusing: chair is still moving (%.2f m/s, %s odometry)"
                           % (moving, source))
        return self.ros.release_estop()

    def _arm_and_drive(self, payload):
        """[시동 + 주행] / [E-STOP 해제]: arm the base, then drive the same way
        [주행 시작] does.

        This used to publish mode 65 and call the follower's start service
        directly. Same button, and on a stack brought up through tools/hybrid.sh
        a completely different amount of checking underneath it: go_hybrid.sh
        pings eight nodes, verifies the CuPy/RTX DWA backend, verifies
        PointPillars when the perception profile asks for it, runs
        hybrid_preflight.py and person_bypass_preflight.py, and only then hands
        off to go.sh. None of that ran here.

        And this is not the rare path. The app sends arm_and_drive from
        [주행 시작] too whenever the base is resting in manual, which is the
        normal state immediately after bring-up -- so the ordinary way to set
        off was the one way that skipped every preflight.
        """
        ok, detail = self._release(payload)
        if not ok:
            return ok, detail
        armed, echo = self.ros.await_auto_echo()
        if not armed:
            return False, ("자동 모드(mode_cmd=65)는 보냈지만 모터 컨트롤러가 "
                           "확인해주지 않았습니다 — %s. 주행은 시작하지 "
                           "않았습니다." % echo)
        ok, drive_detail = self._drive(payload, True)
        if not ok:
            return False, ("자동 모드 전환 완료 (mode_cmd=65). 주행은 거부됨 — %s"
                           % drive_detail)
        return True, "시동 완료 (mode_cmd=65). %s" % drive_detail

    def _halt(self):
        """Stop must never refuse, so fall back to the direct path when the script
        is unavailable -- mirroring stop.sh, which checks nothing on purpose.

        The fallback used to pause the follower and stop there, while stop.sh
        also publishes mode 77. Same button, two different amounts of stopping,
        chosen by a flag the rider cannot see: with scripts off the base stayed
        in auto, so anything that published a wheel_cmd afterwards still moved
        the chair. The fallback now does both acts, in stop.sh's order -- mode
        first, because publishing a topic is instant and cannot fail, and a
        missing service must never be what delays a stop.
        """
        if self.jobs is not None and self.jobs.enabled:
            ok, detail = self.jobs.start("halt")
            if ok:
                return ok, detail
            fallback_ok, fallback_detail = self.ros.engage_estop(mark_estop=False)
            return fallback_ok, "%s / 직접 정지: %s" % (detail, fallback_detail)
        return self.ros.engage_estop(mark_estop=False)

    def _stack_start(self, payload):
        if self.jobs is None or not self.jobs.enabled:
            return False, "스크립트 실행이 꺼져 있습니다 (--allow-scripts off)"
        if not payload.get("confirm"):
            return False, ("확인 필요: \"confirm\": true 를 함께 보내세요 — "
                           "라이다·IMU·측위 노드가 기동됩니다 (수 분 소요)")
        return self.jobs.start("stack")

    def _stack_stop(self, payload):
        """Undo [로컬 켜기]. Confirmed, because it ends the drive and the sensors."""
        if self.jobs is None or not self.jobs.enabled:
            return False, "스크립트 실행이 꺼져 있습니다 (--allow-scripts off)"
        if not payload.get("confirm"):
            return False, ("확인 필요: \"confirm\": true 를 함께 보내세요 — "
                           "주행을 멈추고 라이다·측위 노드를 모두 내립니다")
        return self.jobs.start("stack_stop")

    def _drive(self, payload, running):
        """Mirror go.sh's refusals rather than starting into a broken precondition."""
        now = time.time()
        with self.state.lock:
            in_manual = self.state.drive_mode == MANUAL_MODE
            app_estop = self.state.estop_requested_at is not None
            # Freshness matters more here than anywhere. A localizer that died
            # leaves its last word behind, and its last word is usually
            # TRACKING -- refusing on the reading rather than on the topic
            # being alive would wave the drive through onto a dead fix.
            localization = self.state._fresh(self.state.localization_status,
                                             self.state.localization_stamp, now)
            tracking = localization == "TRACKING"
            objects = self.state._fresh(self.state.objects_summary,
                                        self.state.objects_stamp, now)
            link_age = (None if self.state.wheel_status_stamp is None
                        else now - self.state.wheel_status_stamp)
        # Both refusals are "the base is in manual", but they are not the same
        # situation and they do not have the same next step. Calling the resting
        # state an E-STOP sent the operator looking for an emergency that never
        # happened -- and the app greys out [E-STOP 해제] unless one is engaged,
        # so the advice pointed at a disabled button. Say which one it is.
        if in_manual and app_estop:
            return False, ("거부: E-STOP이 걸려 있습니다. [E-STOP 해제]로 먼저 "
                           "푼 다음 주행하세요.")
        if in_manual:
            return False, ("거부: 베이스가 수동 모드입니다 (기동 직후의 정상 상태이거나 "
                           "조종간 페일세이프). [자동 모드 전환]으로 시동을 건 뒤 "
                           "주행하세요.")
        if not payload.get("confirm"):
            return False, ("confirmation required: resend with \"confirm\": true -- "
                           "the chair will begin moving")
        if link_age is None or link_age > self.ttl_s:
            return False, "refusing: wheel base is silent (/wheel_status stale)"
        if objects is None:
            return False, ("refusing: object tracking is silent "
                           "(/perception/objects_summary) -- an empty object list "
                           "reads exactly like clear road")
        if not tracking:
            return False, ("refusing: localization is '%s', not TRACKING"
                           % (localization or "silent"))
        # Prefer the operator's own go.sh: it re-checks authoritatively and its
        # refusal text is the wording the team already knows. Reimplementing the
        # launch policy in Python would mean two copies that drift apart.
        if self.jobs is not None and self.jobs.enabled:
            return self.jobs.start("drive")
        return self.ros.set_follower(running, ensure_auto=True)


# ---------------------------------------------------------------- transports

def start_debug_tcp(args, state, ros, jobs=None, route=None, route_finder=None):
    """Serve the same Session on 127.0.0.1 for bench checks. Loopback only."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.debug_tcp))
    listener.listen(2)
    # rospy installs a process-wide default socket timeout, which a socket created
    # after init_node inherits -- so accept() would keep raising socket.timeout.
    listener.settimeout(None)
    log("DEBUG probe on tcp://127.0.0.1:%d (loopback only)" % args.debug_tcp)

    def accept_loop():
        while True:
            try:
                conn, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                # A client that vanishes between SYN and accept() raises
                # ECONNABORTED, which is not the listener dying -- but treating
                # every OSError as fatal returned from this thread, dropped the
                # last reference to `listener`, and took the probe port down for
                # the rest of the bridge's life with nothing in the log. Only
                # give up once the socket really is closed.
                if listener.fileno() < 0:
                    return
                log("debug probe accept failed (%s: %s) -- still listening"
                    % (type(exc).__name__, exc))
                time.sleep(0.1)
                continue
            threading.Thread(
                target=lambda c=conn: (Session(c, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve(),
                                       c.close()),
                daemon=True).start()

    threading.Thread(target=accept_loop, daemon=True).start()


def serve_bluez_profile(args, state, ros, jobs=None, route=None, route_finder=None):
    """Let bluetoothd own the RFCOMM listen and publish the SPP SDP record.

    Android's createRfcommSocketToServiceRecord(SPP_UUID) needs an SDP record; a
    bare AF_BLUETOOTH bind publishes none.  Registering an org.bluez.Profile1
    needs no root and no PyBluez.  Verified on the NUC as user mprp3.
    """
    import dbus                                                   # noqa: PLC0415
    import dbus.mainloop.glib                                      # noqa: PLC0415
    import dbus.service                                            # noqa: PLC0415
    from gi.repository import GLib                                 # noqa: PLC0415

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    path = "/uniconlab/wheelchair/spp"

    class Profile(dbus.service.Object):
        @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
        def Release(self):
            log("BlueZ released the profile.")

        @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
        def NewConnection(self, device, fd, properties):
            raw = fd.take()
            log("phone connected: %s (fd %d)" % (device, raw))
            client = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                   socket.BTPROTO_RFCOMM, fileno=raw)
            # BlueZ hands the descriptor over in non-blocking mode, so recv()
            # returns EAGAIN straight away and the session would tear down the
            # instant the phone connected. Session.serve() expects to block.
            client.setblocking(True)

            def run():
                try:
                    Session(client, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve()
                finally:
                    try:
                        client.close()
                    except OSError:
                        pass
                    log("phone disconnected: %s" % device)

            threading.Thread(target=run, daemon=True).start()

        @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
        def RequestDisconnection(self, device):
            log("disconnect requested by %s" % device)

    Profile(bus, path)
    manager = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"),
                             "org.bluez.ProfileManager1")
    options = {
        "Name": "UniconLab Wheelchair Bridge",
        "Role": "server",
        "Channel": dbus.UInt16(args.channel),
        "RequireAuthentication": dbus.Boolean(True),
        "RequireAuthorization": dbus.Boolean(False),
        "AutoConnect": dbus.Boolean(False),
    }
    try:
        manager.RegisterProfile(path, SPP_UUID.lower(), options)
    except Exception as exc:                                      # noqa: BLE001
        log("RegisterProfile failed (%s) -- falling back to a raw RFCOMM bind." % exc)
        return serve_raw_socket(args, state, ros, jobs, route, route_finder)

    log("SPP profile registered with bluetoothd (SDP record published, channel %d)"
        % args.channel)
    log("serving at %.1f Hz, commands=%s -- Ctrl-C to stop"
        % (args.rate, "on" if args.allow_commands else "OFF"))
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        log("shutting down.")
    finally:
        try:
            manager.UnregisterProfile(path)
        except Exception:                                         # noqa: BLE001
            pass
    return 0


def register_sdp_record(channel):
    """Legacy SDP registration for the raw-socket transport."""
    try:
        import bluetooth                                          # noqa: PLC0415
        bluetooth.advertise_service(
            None, "UniconLab Wheelchair Bridge",
            service_id=SPP_UUID, service_classes=[SPP_UUID, bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE])
        log("SDP record advertised via PyBluez.")
        return True
    except Exception:                                             # noqa: BLE001
        pass
    try:
        subprocess.run(["sdptool", "add", "--channel=%d" % channel, "SP"],
                       check=True, capture_output=True, timeout=10)
        log("SDP record added via sdptool.")
        return True
    except Exception as exc:                                      # noqa: BLE001
        log("WARNING: no SDP record (%s). Prefer --transport bluez, which needs "
            "no root and publishes one properly." % exc)
        return False


def serve_raw_socket(args, state, ros, jobs=None, route=None, route_finder=None):
    if not hasattr(socket, "AF_BLUETOOTH"):
        log("this Python has no AF_BLUETOOTH; run the bridge on the Linux NUC.")
        return 1
    try:
        server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server.bind((args.bdaddr, args.channel))
        server.listen(1)
    except OSError as exc:
        log("RFCOMM bind failed on channel %d: %s: %s" % (args.channel, type(exc).__name__, exc))
        log("on the NUC check: rfkill list bluetooth / systemctl status bluetooth / "
            "another process already on this channel")
        return 1
    register_sdp_record(args.channel)
    log("listening on RFCOMM channel %d (rate %.1f Hz, commands=%s)"
        % (args.channel, args.rate, "on" if args.allow_commands else "OFF"))
    try:
        while True:
            client, info = server.accept()
            log("phone connected: %s" % (info,))
            try:
                Session(client, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve()
            finally:
                try:
                    client.close()
                except OSError:
                    pass
                log("phone disconnected.")
    except KeyboardInterrupt:
        log("shutting down.")
        return 0
    finally:
        server.close()


# ---------------------------------------------------------------- self-test

def self_test(args, state, ros, jobs=None, route=None, route_finder=None):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]

    def accept_once():
        conn, _ = listener.accept()
        Session(conn, state, ros, args.ttl, args.rate, jobs=jobs, route=route, route_finder=route_finder).serve()
        conn.close()

    threading.Thread(target=accept_once, daemon=True).start()
    client = socket.create_connection(("127.0.0.1", port))
    stream = client.makefile("rb")

    def next_of(kind):
        deadline = time.time() + 6.0
        while time.time() < deadline:
            line = stream.readline()
            if not line:
                return None
            obj = json.loads(line.decode())
            if obj.get("type") == kind:
                return obj
        return None

    def send(obj):
        client.sendall((json.dumps(obj) + "\n").encode())

    failures = []

    def check(name, condition, note=""):
        print("  %-56s %s%s" % (name, "PASS" if condition else "FAIL",
                                "" if not note else "  (%s)" % note))
        if not condition:
            failures.append(name)

    print("\n--- protocol self-test ---")
    first = next_of("telemetry")
    check("telemetry frame arrives", first is not None)
    if first is None:
        return 1
    print("  first frame: %s" % json.dumps(first)[:150])
    check("battery reported as null, not a fake number", first["battery_percent"] is None)
    check("no scaffold fields leak in (armed/reason_mask)",
          "armed" not in first and "reason_mask" not in first)
    check("drive_mode exposed", "drive_mode" in first and "estop_engaged" in first)
    check("fail-closed display flag is False without ROS",
          first["display_safe_to_drive"] is False)

    client.sendall(b"not json\n")
    check("garbage line does NOT drop the link", next_of("telemetry") is not None)

    send({"command": "step", "step": "abc"})
    ack = next_of("ack")
    check("malformed legacy command answers, link survives",
          ack is not None and ack["ok"] is False)

    send({"command": "estop_release"})
    ack = next_of("ack")
    check("release without confirm is refused",
          ack is not None and ack["ok"] is False and "confirmation" in ack["detail"])

    send({"command": "estop_release", "confirm": True})
    ack = next_of("ack")
    check("release with confirm still refused when /wheel_status is stale",
          ack is not None and ack["ok"] is False and "wheel_status" in ack["detail"])

    send({"command": "drive_start", "confirm": True})
    ack = next_of("ack")
    check("drive_start refused with commands disabled",
          ack is not None and ack["ok"] is False)

    # The ordering that matters in arm_and_drive: everything that can refuse
    # has to refuse BEFORE mode 65 goes out. Arming and then discovering the
    # drive cannot start leaves the base armed on a chair nobody is driving.
    send({"command": "arm_and_drive", "confirm": True})
    ack = next_of("ack")
    check("arm_and_drive refuses before arming, not after",
          ack is not None and ack["ok"] is False and "wheel_status" in ack["detail"])

    send({"command": "ping"})
    ack = next_of("ack")
    check("ping answers", ack is not None and ack["ok"] is True)

    client.close()
    listener.close()
    print("--- %s ---\n" % ("all checks passed" if not failures
                            else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
    return 1 if failures else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--channel", type=int, default=1)
    # BDADDR_ANY must be spelled out: Python's AF_BLUETOOTH bind rejects "" with
    # "bad bluetooth address" on Linux as well as Windows. The inherited bridge
    # used "" and could never have bound on the NUC either.
    parser.add_argument("--bdaddr", default="00:00:00:00:00:00")
    parser.add_argument("--rate", type=float, default=2.0, help="telemetry Hz (default 2)")
    parser.add_argument("--ttl", type=float, default=1.0,
                        help="seconds before /wheel_status counts as stale (default 1.0)")
    parser.add_argument("--allow-commands", action="store_true",
                        help="permit e-stop, release and drive start/stop")
    parser.add_argument("--allow-scripts", action="store_true",
                        help="run the allowlisted operator scripts "
                             "(start_wheelchair_localization.sh, go.sh, stop.sh). "
                             "Separate from --allow-commands on purpose: publishing a "
                             "topic and spawning processes on the robot are different "
                             "risks, and this one is opt-in.")
    parser.add_argument("--route", default="~/wheelchair_localization_src/routes/"
                                          "20260814_route_algorithm_waypoints.json",
                        help="waypoint JSON sent to the app once per connection "
                             "for the map view; empty string disables it")
    parser.add_argument("--job-env", action="append", default=[], metavar="KEY=VALUE",
                        help="environment for the launched scripts, repeatable. "
                             "The field DWA runs need "
                             "--job-env PROFILE=dwa --job-env SAFETY_POLICIES=true; "
                             "without it the bring-up script defaults to pursuit.")
    parser.add_argument("--script-dir", default="~",
                        help="directory holding the operator scripts (default ~)")
    parser.add_argument("--node-name", default="wheelchair_bt_bridge")
    parser.add_argument("--transport", choices=("auto", "bluez", "socket"), default="auto",
                        help="'bluez' registers an SPP profile so an SDP record exists")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--debug-tcp", type=int, default=0, metavar="PORT",
                        help="also serve the protocol on 127.0.0.1:PORT for bench checks")
    args = parser.parse_args(argv)

    if args.rate <= 0:
        parser.error("--rate must be positive")

    state = BridgeState()
    ros = RosLink(state, args.allow_commands, args.node_name)
    # Scripts spawn processes on the robot, so they need BOTH gates.
    job_env = {}
    for item in args.job_env:
        key, _, value = item.partition("=")
        if not key or not _:
            log("ignoring malformed --job-env %r (expected KEY=VALUE)" % item)
            continue
        job_env[key] = value
    jobs = JobRunner(args.script_dir, bool(args.allow_scripts and args.allow_commands),
                     job_env)
    # Deliberately NOT resolved here -- see Session.broadcast.
    route = None
    route_finder = lambda: load_route(
        resolve_route_path(args.route, args.script_dir))
    if args.allow_scripts and not args.allow_commands:
        log("--allow-scripts ignored: it also requires --allow-commands.")
    if not args.self_test:
        ros.start()
        if jobs.enabled:
            missing = [n for n, ok in jobs.available().items() if not ok]
            log("script execution ON (dir=%s)%s"
                % (jobs.script_dir,
                   "" if not missing else "; MISSING: %s" % ", ".join(sorted(missing))))

    if args.self_test:
        return self_test(args, state, ros, jobs, route, route_finder)

    if args.debug_tcp:
        start_debug_tcp(args, state, ros, jobs, route, route_finder)

    transport = args.transport
    if transport == "auto":
        try:
            import dbus                                           # noqa: F401,PLC0415
            from gi.repository import GLib                        # noqa: F401,PLC0415
            transport = "bluez"
        except ImportError:
            log("python3-dbus/gi unavailable -- raw RFCOMM bind (no SDP record).")
            transport = "socket"
    if transport == "bluez":
        return serve_bluez_profile(args, state, ros, jobs, route, route_finder)
    return serve_raw_socket(args, state, ros, jobs, route, route_finder)


if __name__ == "__main__":
    sys.exit(main())
