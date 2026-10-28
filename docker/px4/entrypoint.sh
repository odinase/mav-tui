#!/bin/sh
set -eu
SITL_ROUTER_IP=$(getent ahostsv4 router | awk 'NR == 1 { print $1 }')
test -n "$SITL_ROUTER_IP" || { echo "Cannot resolve router" >&2; exit 1; }
export SITL_ROUTER_IP
export PX4_SIM_MODEL=sihsim_quadx PX4_SIM_SPEED_FACTOR=1
export PX4_HOME_LAT=47.397742 PX4_HOME_LON=8.545594 PX4_HOME_ALT=488
# rcS sets MAV_SYS_ID = instance + 1, AFTER PX4_PARAM_* overrides.
exec px4 -i 1 -d -w /state -s etc/init.d-posix/rcS /opt/px4/etc
