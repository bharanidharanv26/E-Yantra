#!/usr/bin/env python3

"""Task 2B waypoint navigation action client."""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints


class PicoClient(Node):

    def __init__(self):
        super().__init__("pico_client")

        self.waypoint_client = self.create_client(
            GetWaypoints, "waypoint_service"
        )

        self.nav_client = ActionClient(
            self, NavToWaypoint, "waypoint_navigation"
        )

        self.last_feedback_log = 0.0
        self.t_start = None

    def request_waypoints(self):
        while rclpy.ok() and not self.waypoint_client.wait_for_service(
            timeout_sec=2.0
        ):
            self.get_logger().info(
                "waiting for the waypoint service ..."
            )

        if not rclpy.ok():
            return None, None

        request = GetWaypoints.Request()
        request.get_waypoints = True

        future = self.waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response is None or not response.waypoints.poses:
            return None, None

        labels = list(response.marker_ids)

        while len(labels) < len(response.waypoints.poses):
            labels.append("wp%d" % (len(labels) + 1))

        return list(response.waypoints.poses), labels

    def feedback_callback(self, feedback_msg):
        now = time.monotonic()

        if now - self.last_feedback_log >= 1.0:
            self.last_feedback_log = now

            position = (
                feedback_msg.feedback.current_waypoint.pose.position
            )

            self.get_logger().info(
                "feedback: x %.2f y %.2f z %.2f"
                % (position.x, position.y, position.z)
            )

    def go_to(self, pose, label):
        while rclpy.ok() and not self.nav_client.wait_for_server(
            timeout_sec=2.0
        ):
            self.get_logger().info(
                "waiting for the action server ..."
            )

        if not rclpy.ok():
            return False

        if self.t_start is None:
            self.t_start = time.monotonic()

        goal = NavToWaypoint.Goal()
        goal.waypoint = pose
        goal.hover = True

        self.get_logger().info(
            "sending %s: x %.2f y %.2f z %.2f"
            % (
                label,
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
        )

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )

        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "%s was rejected by the action server" % label
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        outcome = result_future.result()

        if (
            outcome is None
            or outcome.status != GoalStatus.STATUS_SUCCEEDED
        ):
            status = None if outcome is None else outcome.status

            self.get_logger().error(
                "%s failed (status %s)" % (label, status)
            )
            return False

        self.get_logger().info(
            "%s REACHED and held. settle time %d s "
            "(mission time %.1f s)"
            % (
                label,
                outcome.result.hov_time,
                time.monotonic() - self.t_start,
            )
        )

        return True

    def run(self):
        poses, labels = self.request_waypoints()

        if poses is None:
            self.get_logger().error(
                "the waypoint service returned no waypoints"
            )
            return

        self.get_logger().info(
            "got %d waypoints: %s"
            % (len(poses), ", ".join(labels))
        )

        for pose, label in zip(poses, labels):
            if not self.go_to(pose, label):
                self.get_logger().error(
                    "navigation stopped at %s" % label
                )
                return

        self.get_logger().info(
            "NAVIGATION COMPLETE: all %d waypoints reached in %.1f s"
            % (len(poses), time.monotonic() - self.t_start)
        )


def main(args=None):
    rclpy.init(args=args)
    node = PicoClient()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
