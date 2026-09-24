#!/usr/bin/env bash
set -euo pipefail
IFACE="${IFACE:-enp0s31f6}"
LAPTOP="192.168.1.12"
A="192.168.1.10"
B="192.168.1.11"

ip -4 -br address show "$IFACE" | grep -q "$LAPTOP/24" || {
  echo "ERROR: laptop must be $LAPTOP/24 on $IFACE" >&2; exit 1;
}
for ip in "$A" "$B"; do
  ping -I "$IFACE" -c 2 -W 1 "$ip" >/dev/null
  nc -z -w 2 "$ip" 10000
 done
sudo ip neigh flush dev "$IFACE" >/dev/null 2>&1 || true
ping -I "$IFACE" -c 1 -W 1 "$A" >/dev/null
ping -I "$IFACE" -c 1 -W 1 "$B" >/dev/null
MAC_A="$(ip neigh show "$A" dev "$IFACE" | awk '{for(i=1;i<=NF;i++) if($i=="lladdr" && i<NF){print $(i+1); exit}}')"
MAC_B="$(ip neigh show "$B" dev "$IFACE" | awk '{for(i=1;i<=NF;i++) if($i=="lladdr" && i<NF){print $(i+1); exit}}')"
test -n "$MAC_A" && test -n "$MAC_B" && test "$MAC_A" != "$MAC_B" || {
  echo "ERROR: missing or identical robot MAC addresses" >&2; exit 1;
}
echo "DUAL NETWORK PASS"
echo "Robot A receiver: $A MAC=$MAC_A"
echo "Robot B giver:    $B MAC=$MAC_B"
