#!/usr/bin/env python3
"""Ackermann wheel odometry for the Raph rover.

Subscribes /controller/joint_states (wheel encoders + steering servos) and integrates a
nav_msgs/Odometry estimate on /odom, optionally broadcasting odom->base_footprint.

Kinematics (see RaphRover/raph_common):
  - wheel_fl / wheel_fr are the FIXED (non-steered) wheels -> forward speed.
  - wheel_rl / wheel_rr are STEERED by servo_l / servo_r.
  - The two front wheels spin with opposite joint-axis sign (fl axis +y, fr axis
    -y), so a per-side sign makes both read positive when driving forward.
  - Rear-steer bicycle model: dtheta = steering_sign * (dist / wheelbase) * tan(delta)
    with delta the mean steering angle. steering_sign defaults to -1 for this
    rear-steer geometry; flip it if yaw integrates the wrong way on a test turn.

All parameters are ROS params so they can be tuned live with `ros2 param set`.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_angle(a: float) -> float:
    """Wrap to (-pi, pi] - also unwraps a per-step delta of a continuous joint."""
    return math.atan2(math.sin(a), math.cos(a))


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheelbase", 0.378)
        self.declare_parameter("front_left_joint", "wheel_fl_joint")
        self.declare_parameter("front_right_joint", "wheel_fr_joint")
        self.declare_parameter("steer_left_joint", "servo_l_joint")
        self.declare_parameter("steer_right_joint", "servo_r_joint")
        self.declare_parameter("left_wheel_sign", 1.0)
        self.declare_parameter("right_wheel_sign", -1.0)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("max_dt", 0.5)

        g = lambda n: self.get_parameter(n).value
        self.r = float(g("wheel_radius"))
        self.L = float(g("wheelbase"))
        self.j_fl = g("front_left_joint")
        self.j_fr = g("front_right_joint")
        self.j_sl = g("steer_left_joint")
        self.j_sr = g("steer_right_joint")
        self.s_l = float(g("left_wheel_sign"))
        self.s_r = float(g("right_wheel_sign"))
        self.steer_sign = float(g("steering_sign"))
        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.publish_tf = bool(g("publish_tf"))
        self.max_dt = float(g("max_dt"))

        self.x = self.y = self.th = 0.0
        self.last_stamp = None
        self.last_fl = self.last_fr = None

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        # Wheel/servo states come from the base controller, bridged up under the
        # /controller prefix (docker-compose controller_relay BRIDGE_TOPICS).
        self.create_subscription(JointState, "/controller/joint_states", self.cb, qos)

        self.get_logger().info(
            f"wheel_odometry up: r={self.r} L={self.L} steering_sign={self.steer_sign} "
            f"publish_tf={self.publish_tf} -> /odom ({self.odom_frame} -> {self.base_frame})"
        )

    def cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if self.j_fl not in idx or self.j_fr not in idx:
            return  # not the drivetrain joint_states message

        t = Time.from_msg(msg.header.stamp)
        fl = msg.position[idx[self.j_fl]]
        fr = msg.position[idx[self.j_fr]]
        dl = msg.position[idx[self.j_sl]] if self.j_sl in idx else 0.0
        dr = msg.position[idx[self.j_sr]] if self.j_sr in idx else 0.0
        delta = 0.5 * (dl + dr)

        if self.last_stamp is None:
            self.last_stamp, self.last_fl, self.last_fr = t, fl, fr
            return

        dt = (t - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0 or dt > self.max_dt:
            # duplicate/old stamp or a gap too large to trust: re-baseline only
            self.last_stamp, self.last_fl, self.last_fr = t, fl, fr
            return

        d_fl = normalize_angle(fl - self.last_fl) * self.s_l
        d_fr = normalize_angle(fr - self.last_fr) * self.s_r
        dist = self.r * 0.5 * (d_fl + d_fr)
        dtheta = self.steer_sign * (dist / self.L) * math.tan(delta)

        th_mid = self.th + 0.5 * dtheta
        self.x += dist * math.cos(th_mid)
        self.y += dist * math.sin(th_mid)
        self.th = normalize_angle(self.th + dtheta)

        self.last_stamp, self.last_fl, self.last_fr = t, fl, fr
        self.publish(msg.header.stamp, dist / dt, dtheta / dt)

    def publish(self, stamp, v, omega):
        qz = math.sin(self.th * 0.5)
        qw = math.cos(self.th * 0.5)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        odom.pose.covariance[0] = 0.002    # x
        odom.pose.covariance[7] = 0.002    # y
        odom.pose.covariance[35] = 0.01    # yaw
        odom.twist.covariance[0] = 0.002   # vx
        odom.twist.covariance[35] = 0.01   # wz
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometry()
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
