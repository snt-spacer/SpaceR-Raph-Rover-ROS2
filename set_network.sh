#!/usr/bin/env bash
set -euo pipefail

# Must be root
if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo $0"
  exit 1
fi

# Require netplan
if ! command -v netplan >/dev/null 2>&1; then
  echo "Error: netplan not found. Install netplan (Ubuntu uses it by default)."
  exit 1
fi

# Backup existing configs
ts="$(date +%Y%m%d-%H%M%S)"
backup_dir="/etc/netplan/backup-${ts}"
mkdir -p "$backup_dir"
shopt -s nullglob
for f in /etc/netplan/*.yaml /etc/netplan/*.yml; do
  cp -a "$f" "$backup_dir"/
done
echo "Backed up existing Netplan files to: $backup_dir"
rm -f /etc/netplan/99-livox.yaml

# Write the requested config to /etc/netplan/01-netcfg.yaml
cat > /etc/netplan/01-netcfg.yaml <<'YAML'
network:
  version: 2
  ethernets:
    enP8p1s0:
      renderer: networkd
      optional: true
      addresses:
        - 10.10.0.15/24
        - 192.168.1.5/24     # Livox HAP host endpoint (persistent; owned by networkd)
      routes:
        - to: default
          via: 10.10.0.1
        # The bumblebee's gev_nettweak raises this NIC to jumbo MTU (9000) for
        # GigE camera throughput, but the base-controller MCU shares the NIC and
        # speaks micro-ROS/XRCE over UDP - it cannot receive jumbo frames. Cap the
        # path MTU to the MCU at 1500 so the host never sends it a jumbo frame,
        # while the camera still gets jumbo. (MCU is at 10.10.0.211.)
        - to: 10.10.0.211/32
          scope: link
          mtu: 1500
      nameservers:
        addresses: [10.10.0.1]
      link-local: []
YAML

chmod 600 /etc/netplan/01-netcfg.yaml   # netplan warns if world-readable
echo "Wrote /etc/netplan/01-netcfg.yaml"

# Validate and generate backend configs
netplan generate
echo "Netplan configuration generated successfully."

netplan apply
echo "Netplan configuration applied — enP8p1s0 is now 10.10.0.15/24."
