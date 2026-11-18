#!/usr/bin/env bash
# Run on the HOST before starting the bumblebee container.
# Sets kernel parameters required for reliable GigE Vision streaming on Jetson.
# Background: "GigE camera setup" in the README shipped with the Spinnaker SDK.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo $0"
  exit 1
fi

sudo ip link set enP8p1s0 mtu 9000

# --- Receive buffers (Spinnaker README, GigE setup) ---
sysctl -w net.core.rmem_max=33554432
sysctl -w net.core.rmem_default=33554432
sysctl -w net.core.netdev_max_backlog=4096
sysctl -w net.unix.max_dgram_qlen=7384

# --- Disable Reverse Path Filtering (Spinnaker README, GigE setup) ---
# Required so the kernel doesn't drop GigE Vision packets whose source IP
# arrives on an unexpected interface (common with bridge + camera setups).
sysctl -w net.ipv4.conf.all.rp_filter=0
sysctl -w net.ipv4.conf.enP8p1s0.rp_filter=0
sysctl -w net.ipv4.conf.br0.rp_filter=0

# --- Remove conflicting IP from bridge slave ---
# enP8p1s0 is enslaved to br0; it must not carry its own 10.10.0.x address or
# Spinnaker finds the camera on two adapters in the same subnet → error -1023.
if ip addr show enP8p1s0 2>/dev/null | grep -q '10\.10\.0\.5/24'; then
  ip addr del 10.10.0.5/24 dev enP8p1s0
  echo "Removed 10.10.0.5/24 from enP8p1s0."
fi

# --- Jetson performance (Spinnaker README, networking best practices) ---
# Max power mode and clock boost reduce CPU scheduling jitter during streaming.
if command -v nvpmodel &>/dev/null; then
  nvpmodel -m 0 && echo "nvpmodel: max power mode enabled"
fi
if command -v jetson_clocks &>/dev/null; then
  jetson_clocks && echo "jetson_clocks: CPU/GPU clocks maximised"
fi

echo "GigE host tuning complete."

if [[ "${1:-}" == "--persist" ]]; then
  CONF=/etc/sysctl.d/60-gige-camera.conf
  cat > "$CONF" <<'EOF'
net.core.rmem_max=33554432
net.core.rmem_default=33554432
net.core.netdev_max_backlog=4096
net.unix.max_dgram_qlen=7384
net.ipv4.conf.all.rp_filter=0
net.ipv4.conf.enP8p1s0.rp_filter=0
net.ipv4.conf.br0.rp_filter=0
EOF
  echo "Written to $CONF — will apply on next boot."
  echo "Note: nvpmodel/jetson_clocks are not persisted; add them to /etc/rc.local if needed."
fi
