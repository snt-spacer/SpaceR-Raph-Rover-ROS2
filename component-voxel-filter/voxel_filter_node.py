#!/usr/bin/env python3
"""Voxel-grid downsample for the rover point clouds.

Subscribes each input cloud and republishes a voxel-downsampled copy on
`<input><output_suffix>` (default `<input>_filtered`). A single `leaf_size`
parameter (metres) sets the voxel edge length and can be changed live:

    ros2 param set /voxel_filter leaf_size 0.08

Downsampling keeps the first point found in each occupied voxel, so ALL point
fields (rgb, intensity, ...) are preserved exactly - not just XYZ.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult

FLOAT32 = 7  # sensor_msgs/PointField datatype


class VoxelFilter(Node):
    def __init__(self):
        super().__init__("voxel_filter")

        self.declare_parameter(
            "input_topics",
            ["/lidar/points", "/loccam/points", "/navcam/points"],
        )
        self.declare_parameter("leaf_size", 0.01)   # metres
        self.declare_parameter("output_suffix", "_filtered")

        self.leaf = float(self.get_parameter("leaf_size").value)
        inputs = list(self.get_parameter("input_topics").value)
        suffix = self.get_parameter("output_suffix").value

        for topic in inputs:
            out = topic + suffix
            pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, p=pub: self.cb(msg, p),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"voxel filter: {topic} -> {out}")

        self.add_on_set_parameters_callback(self.on_param)
        self.get_logger().info(
            f"leaf_size = {self.leaf} m  (live: ros2 param set /voxel_filter leaf_size <m>)"
        )

    def on_param(self, params):
        for p in params:
            if p.name == "leaf_size":
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason="leaf_size must be > 0")
                self.leaf = float(p.value)
                self.get_logger().info(f"leaf_size -> {self.leaf} m")
        return SetParametersResult(successful=True)

    @staticmethod
    def _offset(msg, name):
        for f in msg.fields:
            if f.name == name and f.datatype == FLOAT32:
                return f.offset
        return None

    def cb(self, msg: PointCloud2, pub):
        n = msg.width * msg.height
        if n == 0:
            return
        xo, yo, zo = (self._offset(msg, a) for a in ("x", "y", "z"))
        if xo is None or yo is None or zo is None:
            return  # need float32 x/y/z

        step = msg.point_step
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: n * step].reshape(n, step)

        def f32(off):
            return raw[:, off:off + 4].copy().view(np.float32).ravel()

        x, y, z = f32(xo), f32(yo), f32(zo)
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        if not finite.all():
            raw, x, y, z = raw[finite], x[finite], y[finite], z[finite]
        if x.size == 0:
            return

        leaf = self.leaf
        ijk = np.floor(np.stack([x, y, z], axis=1) / leaf).astype(np.int64)
        ijk -= ijk.min(axis=0)
        dims = ijk.max(axis=0) + 1
        if int(dims[0]) * int(dims[1]) * int(dims[2]) < (1 << 62):
            key = (ijk[:, 0] * dims[1] + ijk[:, 1]) * dims[2] + ijk[:, 2]
            _, first = np.unique(key, return_index=True)
        else:  # extreme extent: fall back to slower exact unique
            _, first = np.unique(ijk, axis=0, return_index=True)

        kept = raw[first]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = int(kept.shape[0])
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = step
        out.row_step = step * out.width
        out.is_dense = True
        out.data = kept.tobytes()
        pub.publish(out)


def main():
    rclpy.init()
    node = VoxelFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
