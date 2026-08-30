#!/usr/bin/env bash
# Take the whole field stack down. The other half of [로컬 켜기].
#
# WHY THIS EXISTS
# ---------------
# scripts/ros1_bluetooth_bridge.py has offered a "stack_stop" job since the
# bridge was written -- JobRunner.JOBS maps it to stop_stack.sh -- and the
# Android app has had the [스택 내리기] button wired to it. The script itself
# was never in the repo, so resolve() failed, the button was greyed out for
# every field session, and the bridge lost the only real abort it has for a
# bring-up in progress: job_cancel signals the tracked process group, but the
# bring-up detaches its nodes with setsid, so the sensors it already started
# never see that signal.
#
# ORDER
# -----
# Stop the drive first, sweep second. Killing the follower before the base is
# out of auto mode leaves uart.py in auto with nothing publishing wheel_cmd,
# and the chair coasts on the last command until the 0.6 s starvation watchdog
# notices. mode_cmd=77 transmits the motor stop frame immediately and holds it
# even after every ROS node above it is gone, so it goes first.
set -eo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

say() { printf '\n=== %s ===\n' "$1"; }

say "1/3  stopping the drive"
# stop.sh checks nothing on purpose and never fails; the teardown must not be
# able to refuse either, so its exit status is not allowed to end this script.
STOP="${BASE_STOP:-$SCRIPT_DIR/stop.sh}"
[ -f "$STOP" ] || STOP="$SCRIPT_DIR/stop.sh"
bash "$STOP" || echo "  (stop.sh reported an error; continuing -- the sweep below is the real stop)" >&2

say "2/3  sweeping the stack"
# Exactly what the bring-up sweeps, read out of the bring-up. A second hand-kept
# list here would be edited in the same commit as the first one and would
# therefore never catch anything -- and a node missing from a TEARDOWN list is
# worse than one missing from a start-up list, because it keeps its ROS name and
# its publishers with no new run on top of it to make the trouble visible.
BRINGUP="${BASE_START:-$SCRIPT_DIR/start_wheelchair_localization.sh}"
[ -f "$BRINGUP" ] || BRINGUP="$SCRIPT_DIR/start_wheelchair_localization.sh"
[ -f "$BRINGUP" ] || { echo "ERROR: cannot find start_wheelchair_localization.sh to read the sweep list from" >&2; exit 66; }

PATTERN_LINE="$(sed -n 's/^for pattern in \(.*\); do$/\1/p' "$BRINGUP" | tail -n1)"
[ -n "$PATTERN_LINE" ] || { echo "ERROR: no cleanup loop found in $BRINGUP" >&2; exit 66; }
# The [b]racket guards come across verbatim, which is what keeps pkill -f from
# matching this script's own command line.
eval "set -- $PATTERN_LINE"
echo "  $# patterns, from $(basename "$BRINGUP")"
for pattern in "$@"; do
  pkill -f "$pattern" 2>/dev/null || true
done

# uart.py is the one follow-up the derived list cannot supply. It is a
# roslaunch CHILD (base_model wheel.launch, started by the bring-up at step
# 6/7), so the only name the sweep has for it is 'roslaunch' -- and killing
# roslaunch is a request with a deadline, which the bring-up's own fast_lio
# block exists because it is not always met. Named here for the same reason.
#
# Safe only because step 1 already ran: mode_cmd=77 made the motor controller
# transmit its stop frame and latch manual, and the controller holds that
# latch after uart.py is gone. Killing uart.py FIRST would strand the
# controller in whatever mode it was last told, with nothing left able to
# tell it otherwise. That ordering is the whole reason step 1 is step 1.
EXTRA='[u]art\.py'

# Confirm rather than sleep. Same reasoning as the bring-up's own sweep: the
# processes that matter are the ones that go on publishing after the kill, and
# a fixed sleep asserts they are gone rather than checking.
#
# Verified against the same derived list, so a node added to the bring-up is
# added to the teardown and to the teardown's confirmation in one edit.
SURVIVOR_RE="$(printf '%s|' "$@" | sed 's/|$//')|$EXTRA"
for _ in $(seq 1 10); do
  pgrep -f "$SURVIVOR_RE" >/dev/null 2>&1 || break
  sleep 1
done
survivors="$(pgrep -af "$SURVIVOR_RE" 2>/dev/null || true)"
if [ -n "$survivors" ]; then
  echo "  forcing $(echo "$survivors" | wc -l) survivor(s):" >&2
  echo "$survivors" >&2
  pkill -9 -f "$SURVIVOR_RE" 2>/dev/null || true
  sleep 1
fi

say "3/3  taking the master down"
# Last, and only last. Killing roscore first orphans every node above it into a
# state where they cannot even be asked to stop cleanly.
pkill -f '[r]osmaster' 2>/dev/null || true
pkill -f '[r]oscore' 2>/dev/null || true
for _ in $(seq 1 5); do
  pgrep -f '[r]osmaster' >/dev/null 2>&1 || break
  sleep 1
done
pkill -9 -f '[r]osmaster' 2>/dev/null || true

remaining="$(pgrep -af "$SURVIVOR_RE|[r]osmaster" 2>/dev/null || true)"
if [ -n "$remaining" ]; then
  echo "" >&2
  echo "STILL RUNNING after teardown:" >&2
  echo "$remaining" >&2
  echo "The chair is out of auto mode regardless (mode_cmd=77), so it will not" >&2
  echo "drive, but check the NUC before the next [로컬 켜기]." >&2
  exit 1
fi

echo ""
echo "STACK DOWN. Base is in manual -- the joystick works. [로컬 켜기] to bring it back."
