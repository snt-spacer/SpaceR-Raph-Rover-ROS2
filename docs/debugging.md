# Debug CLI & RViz

`debug_cli` runs on the operator computer (part of `docker-compose.remote.yml`). Open a
shell in it:

```bash
xhost +local:docker                 # allow X11 (once per host session)
docker compose -f docker-compose.remote.yml exec debug_cli bash
```

Inside the container, a ready-made RViz profile (TF, robot model, `/controller/odom`, raw
and voxel-filtered clouds — camera clouds colored RGB, lidar by height — and one image per
camera) is available via a baked-in alias:

```bash
rover-rviz                          # == rviz2 -d /root/rover_debug.rviz
```

The `debug_cli` image includes `rapha_description`, so RViz can load the robot model.

List topics from the same container:

```bash
ros2 topic list
```

`rqt` is also installed for live parameter tuning (**Plugins → Configuration → Dynamic
Reconfigure**).
