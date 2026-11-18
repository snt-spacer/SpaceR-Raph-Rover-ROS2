# Setup

## Network

The rover carries its own router and Wi-Fi access point (2.4 GHz and 5 GHz). Connect the
operator computer to it. Default addresses on the rover network:

| Device | Address |
|--------|---------|
| Router | `10.10.0.1` |
| Rover computer (Jetson) | `10.10.0.15`, plus `192.168.1.5` as the LiDAR host endpoint |
| Bumblebee X stereo camera (GigE) | `10.10.0.17` (`CAMERA_IP` in the compose files) |
| Base controller (microcontroller) | `10.10.0.211` (`MCU_IP`) |

```bash
ssh -X <user>@10.10.0.15        # -X enables X11 forwarding for RViz/rqt
```

> Credentials are not stored in this repository. Change the router, Wi-Fi and rover-user
> passwords from their factory defaults before using the rover on a shared network.

## Rover computer (once)

1. **Network:** `sudo ./set_network.sh` writes a persistent netplan config for `enP8p1s0`
   (`10.10.0.15/24` and the LiDAR endpoint `192.168.1.5/24`, with a 1500-byte path MTU to
   the microcontroller; see [Networking details](#networking-details)).
2. **GigE camera tuning:** `sudo ./set_bumblebee_rate.sh --persist` raises receive buffers,
   disables reverse-path filtering and sets the Jetson to maximum performance.
3. **Spinnaker SDK:** the Bumblebee X needs Teledyne FLIR's Spinnaker SDK, which is
   proprietary and not included here. Download it and place the `.deb` packages as described
   in [`component-bumblebee_arm64/library/SPINNAKER_SDK.md`](../component-bumblebee_arm64/library/SPINNAKER_SDK.md).
   Building that image accepts the FLIR EULA non-interactively, so read the EULA first.

## Running the stack

The stack is split across two machines. The **rover** (Jetson) runs all the operational
systems and the Zenoh router; the **operator computer** (a laptop on the rover network)
runs the gamepad teleop and the debug CLI / RViz and connects to the rover's router.

| Machine | Compose file | Build & run script |
|---------|--------------|--------------------|
| Rover (Jetson) | `docker-compose.robot.yml` | `./docker_build_robot.sh` |
| Operator PC | `docker-compose.remote.yml` | `./docker_build_remote.sh` |

### On the rover (Jetson)

```bash
./docker_build_robot.sh                              # build base image + all services, up -d
# or, once built:
docker compose -f docker-compose.robot.yml up -d
docker compose -f docker-compose.robot.yml up -d component-livox_lidar   # a single component
```

Services reach the router either through `depends_on: zenoh_router` or an entrypoint
readiness wait, so bring the router up first (it does automatically with the whole-stack
`up`). Configuration is **baked into the images** at build time; after editing a
component's source or config, rebuild and recreate it:

```bash
docker compose -f docker-compose.robot.yml build component-oak_camera
docker compose -f docker-compose.robot.yml up -d --force-recreate component-oak_camera
```

### On the operator computer

Requires the gamepad plugged in (`/dev/input/js0`) and a local X server for RViz. The
containers connect to the rover's router via `config/zenoh_session.json5`
(`mode: client` → `tcp/10.10.0.15:7447`); if the rover's IP differs, edit that file.

```bash
xhost +local:docker                                  # allow the container to use your X server
./docker_build_remote.sh                             # build teleop + debug_cli, up -d
```

Teleop starts automatically. For topics / RViz:

```bash
docker compose -f docker-compose.remote.yml exec debug_cli bash
#   inside:  ros2 topic list      or      rover-rviz
```

> `docker-compose.yml` (and `docker_build.sh`) remain as a single-machine "everything on
> the Jetson" configuration for local testing.

## Networking details

A single NIC (`enP8p1s0`) carries SSH, the Livox LiDAR, the GigE camera, and the
microcontroller. Addresses are assigned **persistently via netplan** (`set_network.sh`
writes `/etc/netplan/01-netcfg.yaml`) — do not add IPs imperatively in entrypoints, they
get flushed by networkd. The GigE camera runs a jumbo MTU; the path MTU to the
microcontroller is capped to 1500 so its XRCE-DDS sessions stay stable.
