#!/bin/sh
set -eu
router_ip=$(getent ahostsv4 router | awk 'NR == 1 { print $1 }')
test -n "$router_ip" || { echo "Cannot resolve router" >&2; exit 1; }
exec arducopter --model quad --speedup 1 --sysid 1 \
    --home 47.397742,8.545594,488,0 \
    --defaults /opt/sitl/copter.parm,/opt/sitl/defaults.parm \
    --serial0 "udpclient:${router_ip}:14560"
