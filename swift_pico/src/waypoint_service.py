#!/usr/bin/env python3

"""Task 2B waypoint service."""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from waypoint_navigation.srv import GetWaypoints


# Required waypoint order: WP1 -> WP2 -> WP3 -> WP4.
WAYPOINTS = [
    ("wp1", -2.41, -2.41, 13.00),
    ("wp2",  2.41,  1.20, 13.00),
    ("wp3",  1.20, -2.41, 13.00),
    ("wp4", -1.20,  1.20, 13.00),
]


class WaypointService(Node):

    def __init__(self):
        super().__init__("waypoint_service")

        self.srv = self.create_service(
            GetWaypoints,
            "waypoint_service",
            self.handle_request,
        )

        self.get_logger().info(
            "waypoint_service ready (%d waypoints)"
            % len(WAYPOINTS)
        )

    def handle_request(self, request, response):
        if not request.get_waypoints:
            response.waypoints = PoseArray()
            response.marker_ids = []
            return response

        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = "whycode"

        labels = []

        for label, x, y, z in WAYPOINTS:
            pose = Pose()

            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0

            poses.poses.append(pose)
            labels.append(label)

        response.waypoints = poses
        response.marker_ids = labels

        self.get_logger().info(
            "sent %d waypoints: %s"
            % (len(labels), ", ".join(labels))
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = WaypointService()

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
