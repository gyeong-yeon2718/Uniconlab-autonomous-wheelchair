# Perception profile defaults, sourced by every entry point that acts on them.
#
# This lived in start_hybrid_avoidance.sh alone until 2026-08-28, which meant
# the bring-up honoured the profile and the DRIVE did not: go_hybrid.sh
# defaults START_POINTPILLARS to true on its own, so a stack brought up with
# the detector deliberately off was then refused permission to move because
# /rtx_pointpillars was not running and ~/.config/unicon/pointpillars.env did
# not exist. The chair sat at waypoint 42 with the follower reporting DRIVING
# and go.sh printing REFUSING TO START.
#
# One copy, sourced by both. Every value stays overridable: an explicit
# export still wins, because these are `:=` defaults.

PERCEPTION_PROFILE="${PERCEPTION_PROFILE:-legacy_geometric}"
case "$PERCEPTION_PROFILE" in
  legacy_geometric)
    : "${START_POINTPILLARS:=false}"
    # Subtraction stays OFF. safety_gate has never had it and cannot: it
    # works on raw returns in a height band. Turning it on for the cluster
    # producer alone removes a mapped wall from the FOLLOWER's half of the
    # world while the gate goes on sweeping into it, which on 2026-08-28
    # held the chair at waypoint 42 with OBSTACLE_SWEEP on 17 of 20 samples
    # and zone_points 0. The inflation this profile undoes is the
    # thresholds: 1/5/80 splits one object into several.
    : "${GEOMETRIC_FIXED_MAP_SUBTRACTION:=false}"
    : "${GEOMETRIC_MIN_CELL_POINTS:=2}"
    : "${GEOMETRIC_MIN_CLUSTER_POINTS:=8}"
    : "${GEOMETRIC_MAX_CLUSTERS:=40}"
    ;;
  hybrid_experimental)
    : "${START_POINTPILLARS:=true}"
    : "${GEOMETRIC_FIXED_MAP_SUBTRACTION:=false}"
    : "${GEOMETRIC_MIN_CELL_POINTS:=1}"
    : "${GEOMETRIC_MIN_CLUSTER_POINTS:=5}"
    : "${GEOMETRIC_MAX_CLUSTERS:=80}"
    ;;
  *)
    echo "ERROR: PERCEPTION_PROFILE must be legacy_geometric or hybrid_experimental" >&2
    exit 64
    ;;
esac
export PERCEPTION_PROFILE START_POINTPILLARS GEOMETRIC_FIXED_MAP_SUBTRACTION
export GEOMETRIC_MIN_CELL_POINTS GEOMETRIC_MIN_CLUSTER_POINTS GEOMETRIC_MAX_CLUSTERS
