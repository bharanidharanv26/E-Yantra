#!/usr/bin/env python3

"""Task 2B waypoint navigation action server."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from swift_msgs.msg import SwiftMsgs
from whycode_interfaces.msg import MarkerArray
from error_msg.msg import Error
from std_msgs.msg import Header
from waypoint_navigation.action import NavToWaypoint


# PID settings: [x/pitch, y/roll, z/throttle]
KP = [40.0, 40.0, 40.0]
KI = [1.0, 1.0, 5.0]
KD = [100.0, 100.0, 60.0]

SIGN = [1, 1, 1]
TRIM = [0, 0, 8]
MAX_OFFSET = [120, 120, 250]

MID = 1500
RC_MIN = 1000
RC_MAX = 2000

I_ZONE = [1.0, 1.0, 1.0]
I_LIMIT = [40.0, 40.0, 100.0]
D_TAU = 0.15

MARKER_TIMEOUT = 0.3
DRONE_MARKER_ID = 2

START_TOL = 0.30
KEEP_TOL = 0.38
HOLD_TIME = 3.1
GOAL_TIMEOUT = 60.0

YAW_KP = 0.0
YAW_SIGN = 1
YAW_MAX = 100


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PositionController:
    """PID controller for waypoint position."""

    def __init__(self):
        self.Kp = list(KP)
        self.Ki = list(KI)
        self.Kd = list(KD)

        self.target = None
        self.error = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]

        self.prev_pos = None
        self.prev_time = None

        self.yaw = 0.0
        self.yaw_target = None
        self.inside_since = None

    def set_target(self, target):
        self.target = list(target)
        self.integral[0] = 0.0
        self.integral[1] = 0.0
        self.inside_since = None

    def update_state(self, pos, yaw, now):
        self.yaw = yaw

        if self.yaw_target is None:
            self.yaw_target = yaw

        if self.prev_pos is not None:
            dt = now - self.prev_time

            if dt < 0.002:
                return

            if dt < 0.25:
                alpha = dt / (D_TAU + dt)

                for i in range(3):
                    raw = (pos[i] - self.prev_pos[i]) / dt
                    self.velocity[i] += alpha * (
                        raw - self.velocity[i]
                    )
            else:
                self.velocity = [0.0, 0.0, 0.0]

        self.prev_pos = list(pos)
        self.prev_time = now

    def reset_memory(self):
        self.velocity = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
        self.prev_pos = None
        self.prev_time = None
        self.inside_since = None

    def compute_error(self, pos):
        # Z uses the downward-facing camera convention.
        return [
            self.target[0] - pos[0],
            self.target[1] - pos[1],
            pos[2] - self.target[2],
        ]

    def hold_elapsed(self, now):
        if self.inside_since is None:
            return 0.0
        return now - self.inside_since

    def step(self, pos, now, dt):
        self.error = self.compute_error(pos)

        # Derivative terms use measured, filtered velocity.
        d_error = [
            -self.velocity[0],
            -self.velocity[1],
            self.velocity[2],
        ]

        output = [0.0, 0.0, 0.0]

        for i in range(3):
            if self.Ki[i] > 1e-9 and abs(self.error[i]) < I_ZONE[i]:
                limit = I_LIMIT[i] / self.Ki[i]

                trial = clamp(
                    self.integral[i] + self.error[i] * dt,
                    -limit,
                    limit,
                )

                unsaturated = (
                    self.Kp[i] * self.error[i]
                    + self.Kd[i] * d_error[i]
                    + self.Ki[i] * trial
                )

                # Avoid accumulating integral while saturation pushes
                # farther in the direction of the error.
                if not (
                    abs(unsaturated) > MAX_OFFSET[i]
                    and unsaturated * self.error[i] > 0
                ):
                    self.integral[i] = trial

            out = (
                self.Kp[i] * self.error[i]
                + self.Kd[i] * d_error[i]
                + self.Ki[i] * self.integral[i]
            )

            output[i] = clamp(
                out,
                -MAX_OFFSET[i],
                MAX_OFFSET[i],
            )

        worst_error = max(abs(e) for e in self.error)

        if self.inside_since is None:
            if worst_error < START_TOL:
                self.inside_since = now
        elif worst_error > KEEP_TOL:
            self.inside_since = None

        rc = [
            int(
                clamp(
                    MID + TRIM[i] + SIGN[i] * output[i],
                    RC_MIN,
                    RC_MAX,
                )
            )
            for i in range(3)
        ]

        rc_yaw = MID

        if YAW_KP != 0.0 and self.yaw_target is not None:
            yaw_error = wrap_pi(self.yaw_target - self.yaw)
            yaw_output = clamp(
                YAW_KP * yaw_error,
                -YAW_MAX,
                YAW_MAX,
            )

            rc_yaw = int(
                clamp(
                    MID + YAW_SIGN * yaw_output,
                    RC_MIN,
                    RC_MAX,
                )
            )

        return rc[0], rc[1], rc[2], rc_yaw


class PicoServer(Node):

    def __init__(self):
        super().__init__("pico_server")

        self.ctrl = PositionController()

        self.current_state = [0.0, 0.0, 0.0]
        self.orientation = None
        self.last_marker_time = None
        self.last_ctrl_time = None
        self.sample_time = 0.033
        self.active_goal = None

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = MID
        self.cmd.rc_pitch = MID
        self.cmd.rc_yaw = MID
        self.cmd.rc_throttle = MID

        self.pos_error = Error()

        marker_group = MutuallyExclusiveCallbackGroup()
        control_group = MutuallyExclusiveCallbackGroup()
        heartbeat_group = MutuallyExclusiveCallbackGroup()
        action_group = ReentrantCallbackGroup()

        self.command_pub = self.create_publisher(
            SwiftMsgs, "/drone_command", 10
        )
        self.pos_error_pub = self.create_publisher(
            Error, "/pos_error", 10
        )
        self.heartbeat_pub = self.create_publisher(
            Header, "/controller_heartbeat", 10
        )

        self.create_subscription(
            MarkerArray,
            "/whycode_node/markers",
            self.whycode_callback,
            1,
            callback_group=marker_group,
        )

        self.publish_heartbeat()
        self.create_timer(
            0.1,
            self.publish_heartbeat,
            callback_group=heartbeat_group,
        )

        time.sleep(0.5)
        self.arm()

        self.create_timer(
            self.sample_time,
            self.control_loop,
            callback_group=control_group,
        )

        self.action_server = ActionServer(
            self,
            NavToWaypoint,
            "waypoint_navigation",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_group,
        )

        self.get_logger().info(
            'pico_server ready: action "waypoint_navigation"'
        )

    def publish_heartbeat(self):
        heartbeat = Header()
        heartbeat.stamp = self.get_clock().now().to_msg()
        heartbeat.frame_id = "pico_controller_active"
        self.heartbeat_pub.publish(heartbeat)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = MID
        self.cmd.rc_yaw = MID
        self.cmd.rc_pitch = MID
        self.cmd.rc_throttle = MID
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycode_callback(self, msg):
        if not msg.markers:
            return

        marker = msg.markers[0]

        for item in msg.markers:
            if item.id == DRONE_MARKER_ID:
                marker = item
                break

        position = marker.position.position
        orientation = marker.position.orientation
        now = time.monotonic()

        self.current_state = [
            position.x,
            position.y,
            position.z,
        ]
        self.orientation = orientation
        self.last_marker_time = now

        self.ctrl.update_state(
            self.current_state,
            quaternion_to_yaw(orientation),
            now,
        )

    def hold_neutral(self):
        self.cmd.rc_roll = MID
        self.cmd.rc_pitch = MID
        self.cmd.rc_yaw = MID
        self.cmd.rc_throttle = MID
        self.command_pub.publish(self.cmd)

    def control_loop(self):
        now = time.monotonic()

        if (
            self.last_marker_time is None
            or now - self.last_marker_time > MARKER_TIMEOUT
        ):
            self.hold_neutral()
            self.ctrl.reset_memory()
            self.last_ctrl_time = None
            return

        if self.ctrl.target is None:
            self.hold_neutral()
            return

        if self.last_ctrl_time is None:
            dt = self.sample_time
        else:
            dt = clamp(
                now - self.last_ctrl_time,
                0.005,
                0.1,
            )

        self.last_ctrl_time = now

        pitch, roll, throttle, yaw = self.ctrl.step(
            list(self.current_state),
            now,
            dt,
        )

        self.cmd.rc_pitch = pitch
        self.cmd.rc_roll = roll
        self.cmd.rc_throttle = throttle
        self.cmd.rc_yaw = yaw
        self.command_pub.publish(self.cmd)

        self.pos_error.pitch_error = float(self.ctrl.error[0])
        self.pos_error.roll_error = float(self.ctrl.error[1])
        self.pos_error.throttle_error = float(self.ctrl.error[2])
        self.pos_error.yaw_error = 0.0
        self.pos_error_pub.publish(self.pos_error)

    def goal_callback(self, goal_request):
        self.get_logger().info("goal received")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("cancel requested")
        return CancelResponse.ACCEPT

    def make_feedback(self):
        feedback = NavToWaypoint.Feedback()
        pose = PoseStamped()

        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "whycode"

        pose.pose.position.x = float(self.current_state[0])
        pose.pose.position.y = float(self.current_state[1])
        pose.pose.position.z = float(self.current_state[2])

        if self.orientation is not None:
            pose.pose.orientation = self.orientation
        else:
            pose.pose.orientation.w = 1.0

        feedback.current_waypoint = pose
        return feedback

    def execute_callback(self, goal_handle):
        request = goal_handle.request

        target = [
            request.waypoint.position.x,
            request.waypoint.position.y,
            request.waypoint.position.z,
        ]

        self.active_goal = goal_handle
        self.ctrl.set_target(target)

        start = time.monotonic()
        self.get_logger().info(
            "going to x %.2f y %.2f z %.2f" % tuple(target)
        )

        result = NavToWaypoint.Result()
        last_log = 0.0

        while rclpy.ok():
            now = time.monotonic()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.hov_time = -1
                return result

            if self.active_goal is not goal_handle:
                goal_handle.abort()
                result.hov_time = -1
                return result

            goal_handle.publish_feedback(self.make_feedback())

            held = self.ctrl.hold_elapsed(now)
            reached = self.ctrl.inside_since is not None

            if (
                request.hover and held >= HOLD_TIME
            ) or (
                not request.hover and reached
            ):
                goal_handle.succeed()
                result.hov_time = int(round(now - start))
                self.get_logger().info(
                    "waypoint held. settle time %.1f s"
                    % (now - start)
                )
                return result

            if now - start > GOAL_TIMEOUT:
                self.get_logger().error(
                    "timeout: waypoint not reached"
                )
                goal_handle.abort()
                result.hov_time = -1
                return result

            if now - last_log >= 1.0:
                last_log = now
                error = self.ctrl.error

                self.get_logger().info(
                    "t %.0f | err x %.2f y %.2f z %.2f "
                    "| hold %.1f | yaw %.2f"
                    % (
                        now - start,
                        error[0],
                        error[1],
                        error[2],
                        held,
                        self.ctrl.yaw,
                    )
                )

            time.sleep(0.05)

        goal_handle.abort()
        result.hov_time = -1
        return result


def main(args=None):
    rclpy.init(args=args)

    node = PicoServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "KeyboardInterrupt, shutting down."
        )
    finally:
        try:
            node.disarm()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
