#!/usr/bin/env bash
# The one file that decides what the app's buttons mean.
#
# WHY THIS EXISTS
# ---------------
# The phone reaches the chair through four home scripts -- ~/start_wheelchair_
# localization.sh, ~/go.sh, ~/stop.sh, ~/stop_stack.sh -- because that is the
# allowlist scripts/ros1_bluetooth_bridge.py runs (JobRunner.JOBS). Each of
# those was maintained by hand on the NUC, in no repository, and each one
# separately named a deploy. On 2026-08-28 they disagreed: [로컬 켜기] brought
# up one tree while [주행 시작] drove another, and nothing said so. On
# 2026-08-30 ~/wheelchair_deploys/current moved twice inside one session and a
# deploy directory's NAME turned out not to describe the tree inside it.
#
# So: four wrappers, one resolver. install_operator_entrypoints.sh writes all
# four as the same three lines into this file, and this file resolves the
# active deploy exactly once. They cannot disagree any more, because there is
# nothing left for them to disagree about.
#
# WHAT "THE ACTIVE DEPLOY" MEANS
# ------------------------------
# Not the directory name, and not REVIEWED_COMMIT. catkin installs relay stubs
# that exec the SOURCE file, so what actually runs is whatever
# <deploy>/ws/src/static_livox_localization points at. That symlink is the
# authority, it is what this script follows, and `resolve` prints it so the
# answer is on screen before the wheels are involved.
#
#   bash tools/wheelchair_entry.sh resolve      # what would run, and from where
#   bash tools/wheelchair_entry.sh check        # + are the four wrappers agreed
#   bash tools/wheelchair_entry.sh stack-start  # [로컬 켜기]
#   bash tools/wheelchair_entry.sh stack-stop   # [스택 내리기]
#   bash tools/wheelchair_entry.sh drive-start  # [주행 시작] / [시동 + 주행]
#   bash tools/wheelchair_entry.sh drive-stop   # [주행 정지]
#   bash tools/wheelchair_entry.sh estop        # [E-STOP]
set -eo pipefail

COMMAND="${1:-}"
[ "$#" -gt 0 ] && shift || true

fail() { echo "ERROR: $*" >&2; exit 1; }

# --------------------------------------------------------------- re-entry
# hybrid.sh, start_hybrid_avoidance.sh and go_hybrid.sh all fall back to
# $HOME/<name>.sh when BASE_START/BASE_GO/BASE_STOP are unset. Once the home
# scripts are wrappers, that fallback points back here, and the loop it makes
# ends at the wheels rather than at an error. We export all three below so the
# fallback is never taken -- this guard is the second line, for the case where
# something in the deploy strips the environment.
if [ -n "${WHEELCHAIR_ENTRY_ACTIVE:-}" ]; then
  echo "ERROR: wheelchair_entry.sh re-entered from ${WHEELCHAIR_ENTRY_ACTIVE}." >&2
  echo "       A deploy script fell back to a \$HOME wrapper instead of using" >&2
  echo "       BASE_START/BASE_GO/BASE_STOP. Refusing to recurse." >&2
  exit 70
fi

# ------------------------------------------------------------- resolution
#
# The deploy this file LIVES IN is the answer, not a symlink somebody else can
# move. ~/wheelchair_deploys/current moved twice under one session on
# 2026-08-30 and ended on a different branch's deploy; a chair that follows it
# changes what it runs while nobody is looking. The four home wrappers name an
# absolute path into one deploy, `cat ~/go.sh` shows you which, and switching
# deploys means re-running install_operator_entrypoints.sh -- an act, with a
# record, rather than a symlink drifting.
#
# `current` is still the fallback for running this out of a plain checkout, and
# $WHEELCHAIR_DEPLOY still overrides everything for bisecting.
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)"
DEPLOYS_ROOT="${WHEELCHAIR_DEPLOYS_ROOT:-$HOME/wheelchair_deploys}"
CONFIG="${WHEELCHAIR_ENTRY_CONFIG:-$HOME/.config/unicon/wheelchair_entry.env}"

DEPLOY_ROOT=""
DEPLOY_SOURCE=""
if [ -n "${WHEELCHAIR_DEPLOY:-}" ]; then
  case "$WHEELCHAIR_DEPLOY" in
    /*) DEPLOY_ROOT="$WHEELCHAIR_DEPLOY" ;;
    *)  DEPLOY_ROOT="$DEPLOYS_ROOT/$WHEELCHAIR_DEPLOY" ;;
  esac
  DEPLOY_SOURCE="\$WHEELCHAIR_DEPLOY override"
elif [ -d "$SCRIPT_DIR/../../ws/src" ]; then
  # <deploy>/<tree>/tools/wheelchair_entry.sh -- the normal installed shape.
  DEPLOY_ROOT="$SCRIPT_DIR/../.."
  DEPLOY_SOURCE="the deploy this script lives in"
else
  selector=""
  if [ -f "$CONFIG" ]; then
    # Read the one key we understand rather than sourcing the file: this runs
    # from a Bluetooth-triggered job, and a sourced file is arbitrary code.
    selector="$(sed -n 's/^[[:space:]]*DEPLOY[[:space:]]*=[[:space:]]*//p' \
                "$CONFIG" | tail -n1 | tr -d "\"'")"
  fi
  if [ -n "$selector" ]; then
    case "$selector" in
      /*) DEPLOY_ROOT="$selector" ;;
      *)  DEPLOY_ROOT="$DEPLOYS_ROOT/$selector" ;;
    esac
    DEPLOY_SOURCE="DEPLOY= in $CONFIG"
  else
    DEPLOY_ROOT="$DEPLOYS_ROOT/current"
    DEPLOY_SOURCE="$DEPLOYS_ROOT/current -- UNPINNED, follows whoever ran use_deploy.sh last"
  fi
fi
[ -d "$DEPLOY_ROOT" ] || fail "no such deploy: $DEPLOY_ROOT (selected by $DEPLOY_SOURCE)"
DEPLOY_ROOT="$(CDPATH= cd -- "$DEPLOY_ROOT" && pwd -P)"

# The symlink catkin's relay stubs exec. Everything else -- the directory name,
# REVIEWED_COMMIT, OFFLINE_VERIFIED -- is a label somebody typed.
SOURCE_LINK="$DEPLOY_ROOT/ws/src/static_livox_localization"
[ -e "$SOURCE_LINK" ] || fail "$SOURCE_LINK does not exist; this deploy has no runnable workspace"
SOURCE_PKG="$(CDPATH= cd -- "$SOURCE_LINK" && pwd -P)"
TREE_ROOT="$(CDPATH= cd -- "$SOURCE_PKG/../.." && pwd -P)"
[ -f "$TREE_ROOT/tools/hybrid.sh" ] || \
  fail "$TREE_ROOT does not look like the wheelchair tree (no tools/hybrid.sh)"

# The 2026-08-30 failure in one check: a deploy whose workspace symlink pointed
# into a DIFFERENT deploy entirely, so the directory you selected and the code
# that ran were unrelated. Refuse by default; ALLOW_FOREIGN_SOURCE=1 for the
# deliberate case (bisecting one tree against another deploy's assets).
FOREIGN=""
case "$TREE_ROOT/" in
  "$DEPLOY_ROOT"/*) ;;
  *) FOREIGN="yes" ;;
esac
if [ -n "$FOREIGN" ] && [ "${ALLOW_FOREIGN_SOURCE:-0}" != "1" ]; then
  fail "$(printf '%s\n' \
    "deploy '$DEPLOY_ROOT' runs code from OUTSIDE itself:" \
    "  ws/src/static_livox_localization -> $SOURCE_PKG" \
    "The directory name therefore says nothing about what would drive." \
    "Fix the symlink, or set ALLOW_FOREIGN_SOURCE=1 if this is deliberate.")"
fi

# The sharpest form of the same check, and the one the app is exposed to: this
# file was installed out of one tree, and the workspace symlink says a
# different tree is what catkin would exec. Whoever re-pointed the symlink got
# a chair that runs code nobody installed. Same escape hatch, said out loud.
SELF_TREE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd -P)"
if [ "$SELF_TREE" != "$TREE_ROOT" ]; then
  if [ "${ALLOW_FOREIGN_SOURCE:-0}" != "1" ]; then
    fail "$(printf '%s\n' \
      "this entry point was installed from  $SELF_TREE" \
      "but the workspace would actually run  $TREE_ROOT" \
      "catkin's relay stubs exec the source file, so the second one is what" \
      "drives. Re-run install_operator_entrypoints.sh from the tree you mean," \
      "or set ALLOW_FOREIGN_SOURCE=1 if this really is deliberate.")"
  fi
  echo "WARNING: entry installed from $SELF_TREE, workspace runs $TREE_ROOT" >&2
fi

identity() {
  # Whatever the deploy is willing to say about itself, in the order we trust
  # it. Never invent one: a wrong branch name is worse than no branch name.
  local file
  for file in REVIEWED_COMMIT OFFLINE_VERIFIED; do
    [ -f "$DEPLOY_ROOT/$file" ] && {
      printf '%s:\n' "$file"
      sed 's/^/    /' "$DEPLOY_ROOT/$file"
    }
  done
  if git -C "$TREE_ROOT" rev-parse --git-dir >/dev/null 2>&1; then
    printf 'git: %s @ %s\n' \
      "$(git -C "$TREE_ROOT" branch --show-current 2>/dev/null || echo '(detached)')" \
      "$(git -C "$TREE_ROOT" rev-parse --short HEAD 2>/dev/null || echo '?')"
  else
    printf 'git: not a repository (deploy trees usually are not)\n'
  fi
}

# ------------------------------------------------------------ environment
# Exported for every child. BASE_* are the reason the fallbacks in hybrid.sh /
# start_hybrid_avoidance.sh / go_hybrid.sh never reach a $HOME wrapper.
# LOCALIZATION_WS matters just as much and is easier to miss: it defaults to
# $HOME/livox_static_localization_ws, the OLD single workspace, so a wrapper
# that forgets it sources one workspace's devel/setup.bash while catkin's relay
# stubs exec a different tree's source.
export WHEELCHAIR_ENTRY_ACTIVE="$DEPLOY_ROOT"
export LOCALIZATION_WS="${LOCALIZATION_WS:-$DEPLOY_ROOT/ws}"
export BASE_START="$TREE_ROOT/tools/start_wheelchair_localization.sh"
export BASE_GO="$TREE_ROOT/tools/go.sh"
export BASE_STOP="$TREE_ROOT/tools/stop.sh"

banner() {
  printf '=== wheelchair entry ===\n'
  printf 'deploy:   %s\n' "$DEPLOY_ROOT"
  printf 'selected: %s\n' "$DEPLOY_SOURCE"
  printf 'runs:     %s%s\n' "$TREE_ROOT" \
    "$([ -n "$FOREIGN" ] && echo '   *** OUTSIDE THE DEPLOY ***')"
  printf 'ws:       %s\n' "$LOCALIZATION_WS"
  identity | sed 's/^/  /'
  printf '========================\n'
}

# --------------------------------------------------------------- commands
case "$COMMAND" in
  resolve)
    banner
    printf 'stack-start -> %s start\n' "$TREE_ROOT/tools/hybrid.sh"
    printf 'stack-stop  -> %s\n' "$TREE_ROOT/tools/stop_stack.sh"
    printf 'drive-start -> %s go\n' "$TREE_ROOT/tools/hybrid.sh"
    printf 'drive-stop  -> %s\n' "$BASE_STOP"
    printf 'estop       -> %s\n' "$BASE_STOP"
    ;;

  check)
    banner
    status=0
    # Every app button, checked against THIS file by absolute path. "They all
    # mention wheelchair_entry.sh" is not the same claim as "they all name the
    # same one", and it is the difference that bit on 2026-08-28.
    for name in start_wheelchair_localization.sh go.sh stop.sh stop_stack.sh; do
      wrapper="$HOME/$name"
      if [ ! -f "$wrapper" ]; then
        printf 'MISSING  ~/%-34s the app button that runs it is dead\n' "$name"
        status=1
      elif grep -qF "$SCRIPT_DIR/wheelchair_entry.sh" "$wrapper"; then
        printf 'ok       ~/%s\n' "$name"
      elif grep -q 'wheelchair_entry.sh' "$wrapper"; then
        printf 'DIVERGED ~/%-34s goes through a DIFFERENT entry point:\n' "$name"
        grep -o '[^" ]*wheelchair_entry\.sh' "$wrapper" | sed 's/^/           /'
        status=1
      else
        printf 'STRAY    ~/%-34s does not go through wheelchair_entry.sh at all\n' "$name"
        status=1
      fi
    done
    for f in "$TREE_ROOT/tools/hybrid.sh" "$TREE_ROOT/tools/stop_stack.sh" \
             "$BASE_START" "$BASE_GO" "$BASE_STOP"; do
      [ -f "$f" ] || { printf 'MISSING  %s\n' "$f"; status=1; }
    done
    [ "$status" -eq 0 ] && printf '\nall four app buttons resolve to this one deploy.\n' \
                        || printf '\nRUN: bash tools/install_operator_entrypoints.sh\n'
    exit "$status"
    ;;

  stack-start)
    banner
    exec bash "$TREE_ROOT/tools/hybrid.sh" start "$@"
    ;;

  stack-stop)
    banner
    exec bash "$TREE_ROOT/tools/stop_stack.sh" "$@"
    ;;

  drive-start)
    banner
    exec bash "$TREE_ROOT/tools/hybrid.sh" go "$@"
    ;;

  # Stopping deliberately does NOT go through hybrid.sh. hybrid.sh's own `stop`
  # resolves BASE_STOP:-$HOME/stop.sh, and $HOME/stop.sh is a wrapper back into
  # this file -- a loop whose every iteration is a stop that has not happened
  # yet. Straight to the deploy's stop.sh, which checks nothing on purpose.
  drive-stop|estop)
    exec bash "$BASE_STOP" "$@"
    ;;

  -h|--help|help|"")
    sed -n '2,33p' "$0" | sed 's/^# \{0,1\}//'
    ;;

  *)
    fail "unknown command: $COMMAND (try: resolve, check, stack-start, stack-stop, drive-start, drive-stop, estop)"
    ;;
esac
