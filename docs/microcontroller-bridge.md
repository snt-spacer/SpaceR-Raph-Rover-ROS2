# Microcontroller bridge

The base-controller microcontroller speaks XRCE-DDS. `micro_ros_agent` (eProsima's Micro
XRCE-DDS Agent) bridges it onto native Fast DDS, which Zenoh cannot reach directly: generic
DDS↔Zenoh bridges don't produce the type hash `rmw_zenoh_cpp` expects.

`controller_relay_dds` and `controller_relay_zenoh` (`component-controller_relay/`) close the
gap: two `rclpy` nodes, one on `rmw_fastrtps_cpp` and one on `rmw_zenoh_cpp`, linked over a
local TCP connection. Each republishes with the real message types on its own middleware, so
topics and services work normally on both sides.

## Configuring the relay

Bridged topics are set once via `BRIDGE_TOPICS` (identical in both relay services in
`docker-compose.robot.yml`, and in `docker-compose.yml` for the single-machine setup):

```
BRIDGE_TOPICS="/topic:pkg/msg/Type:up,/dds_name=/zenoh_name:pkg/msg/Type:up"
```

- `up` — microcontroller (DDS) → rest of the rover (Zenoh); `down` — the reverse.
- Optional `dds_name=zenoh_name` renames the topic between sides, so the firmware's names
  appear under `/controller/*` on the rover. With no `=`, both sides use the same name.

The relay subscribes best-effort (compatible with every firmware publisher) and republishes
reliable.

**Currently bridged topics** (all controller telemetry normalised under `/controller/*`):

| Zenoh name | Type | Dir | Notes |
|------------|------|-----|-------|
| `/controller/cmd_ackermann` | `ackermann_msgs/AckermannDrive` | down | teleop drive command |
| `/controller/joint_states` | `sensor_msgs/JointState` | up | wheel encoders + steering servos |
| `/controller/imu/data` | `sensor_msgs/Imu` | up | controller IMU (frame `imu`) |
| `/controller/odom` | `nav_msgs/Odometry` | up | controller wheel odometry |
| `/controller/cmd_led_*`, `/controller/led_strip_state` | `rapha_interfaces/*` | — | LED interface — bridged but not yet drivable (see the LED note in `docker-compose.yml`) |

Services use `BRIDGE_SERVICES`, where `direction` describes which side *calls* (the real
server is always on the opposite side). Currently bridged: `/controller/set_steering_mode`,
`/controller/calibrate_servos`, `/controller/set_leds`, `/controller/clear_leds`.

> Most firmware topics don't appear in `ros2 topic list`. To add one, check it by exact name
> with `ros2 topic hz /name` under `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, then add an entry.
