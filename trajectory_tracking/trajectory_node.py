#!/usr/bin/env python3
"""
Enhanced Trajectory Tracking System with Rich Visualization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.interpolate import CubicSpline
import math
from tf_transformations import euler_from_quaternion


class PathSmoother:
    """Smooths discrete waypoints using cubic spline interpolation"""

    def __init__(self, smoothing_factor=100):
        self.smoothing_factor = smoothing_factor

    def smooth_path(self, waypoints):
        if len(waypoints) < 2:
            return waypoints

        waypoints = np.array(waypoints)
        distances = np.cumsum(
            np.sqrt(
                np.sum(
                    np.diff(
                        waypoints,
                        axis=0)**2,
                    axis=1)))
        distances = np.insert(distances, 0, 0)

        cs_x = CubicSpline(distances, waypoints[:, 0], bc_type='natural')
        cs_y = CubicSpline(distances, waypoints[:, 1], bc_type='natural')

        alpha = np.linspace(0, distances[-1], self.smoothing_factor)
        smooth_x = cs_x(alpha)
        smooth_y = cs_y(alpha)

        return np.column_stack((smooth_x, smooth_y))


class TrajectoryGenerator:
    """Generates time-parameterized trajectory with velocity profile"""

    def __init__(self, max_velocity=0.5, max_acceleration=0.3):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

    def generate_trajectory(self, smooth_path):
        trajectory = []
        segments = np.diff(smooth_path, axis=0)
        segment_lengths = np.linalg.norm(segments, axis=1)
        total_length = np.sum(segment_lengths)

        accel_distance = (self.max_velocity**2) / (2 * self.max_acceleration)

        if 2 * accel_distance >= total_length:
            peak_velocity = np.sqrt(self.max_acceleration * total_length)
            accel_distance = total_length / 2
        else:
            peak_velocity = self.max_velocity

        current_distance = 0
        current_time = 0

        for i, point in enumerate(smooth_path):
            if current_distance < accel_distance:
                velocity = np.sqrt(
                    2 * self.max_acceleration * current_distance)
            elif current_distance > (total_length - accel_distance):
                remaining = total_length - current_distance
                velocity = np.sqrt(2 * self.max_acceleration * remaining)
            else:
                velocity = peak_velocity

            if i < len(smooth_path) - 1:
                dx = smooth_path[i + 1, 0] - point[0]
                dy = smooth_path[i + 1, 1] - point[1]
                theta = np.arctan2(dy, dx)
            else:
                theta = trajectory[-1][2] if trajectory else 0

            trajectory.append(
                (point[0], point[1], theta, velocity, current_time))

            if i < len(segment_lengths):
                dt = segment_lengths[i] / max(velocity, 0.01)
                current_time += dt
                current_distance += segment_lengths[i]

        return trajectory


class PurePursuitController:
    """Pure Pursuit controller for trajectory tracking"""

    def __init__(self, lookahead_distance=0.5, linear_velocity=0.3):
        self.lookahead_distance = lookahead_distance
        self.linear_velocity = linear_velocity
        self.goal_tolerance = 0.1

    def find_lookahead_point(self, robot_pose, trajectory, current_idx):
        robot_x, robot_y = robot_pose[0], robot_pose[1]

        for i in range(current_idx, len(trajectory)):
            point = trajectory[i]
            distance = np.sqrt((point[0] - robot_x) **
                               2 + (point[1] - robot_y)**2)

            if distance >= self.lookahead_distance:
                return point, i

        return trajectory[-1], len(trajectory) - 1

    def compute_control(self, robot_pose, trajectory, current_idx):
        if current_idx >= len(trajectory):
            return 0.0, 0.0, current_idx, True

        lookahead_point, new_idx = self.find_lookahead_point(
            robot_pose, trajectory, current_idx
        )

        goal_distance = np.sqrt(
            (trajectory[-1][0] - robot_pose[0])**2 +
            (trajectory[-1][1] - robot_pose[1])**2
        )

        if goal_distance < self.goal_tolerance:
            return 0.0, 0.0, new_idx, True

        dx = lookahead_point[0] - robot_pose[0]
        dy = lookahead_point[1] - robot_pose[1]

        angle_to_goal = np.arctan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - robot_pose[2])

        curvature = 2 * np.sin(angle_diff) / self.lookahead_distance
        angular_vel = curvature * self.linear_velocity
        angular_vel = np.clip(angular_vel, -2.0, 2.0)

        return self.linear_velocity, angular_vel, new_idx, False

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class TrajectoryTrackingNode(Node):
    """Enhanced ROS2 node with rich visualization"""

    def __init__(self):
        super().__init__('trajectory_tracking_node')

        # Initialize components
        self.path_smoother = PathSmoother(smoothing_factor=100)
        self.trajectory_generator = TrajectoryGenerator(max_velocity=0.3)
        self.controller = PurePursuitController(lookahead_distance=0.5)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.robot_path_pub = self.create_publisher(
            Path, '/robot_trajectory', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10)
        self.goal_marker_pub = self.create_publisher(
            Marker, '/goal_marker', 10)
        self.velocity_arrow_pub = self.create_publisher(
            Marker, '/velocity_arrow', 10)
        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/lookahead_point', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # State variables
        self.robot_pose = [0.0, 0.0, 0.0]
        self.trajectory = []
        self.current_idx = 0
        self.goal_reached = False
        self.robot_path_history = []

        # Define waypoints (CUSTOMIZE THESE!)
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 1.0),
            (3.0, 1.0),
            (4.0, 2.0),
            (4.0, 3.0)
        ]

        # Generate trajectory
        self.generate_and_publish_trajectory()

        # Visualization timer
        self.viz_timer = self.create_timer(0.2, self.publish_visualizations)

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🚀 Enhanced Trajectory Tracking Node Started!')
        self.get_logger().info(f'📍 Following {len(self.waypoints)} waypoints')

    def generate_and_publish_trajectory(self):
        """Generate smooth trajectory from waypoints"""
        smooth_path = self.path_smoother.smooth_path(self.waypoints)
        self.trajectory = self.trajectory_generator.generate_trajectory(
            smooth_path)

        self.publish_path(smooth_path)
        self.publish_waypoint_markers()
        self.publish_goal_marker()

        self.get_logger().info(
            f'✅ Generated trajectory with {len(self.trajectory)} points')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.robot_pose[2] = yaw

        # Store path history
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.robot_path_history.append(pose_stamped)

        # Limit history size
        if len(self.robot_path_history) > 1000:
            self.robot_path_history.pop(0)

    def control_loop(self):
        """Main control loop"""
        if self.goal_reached or len(self.trajectory) == 0:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return

        linear_vel, angular_vel, new_idx, reached = self.controller.compute_control(
            self.robot_pose, self.trajectory, self.current_idx)

        self.current_idx = new_idx
        self.goal_reached = reached

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)

        # Log progress
        if self.current_idx % 20 == 0 and not reached:
            progress = (self.current_idx / len(self.trajectory)) * 100
            self.get_logger().info(f'📊 Progress: {progress:.1f}%')

        if reached and not hasattr(self, '_goal_reached_logged'):
            self.get_logger().info('🎯 Goal Reached!')
            self._goal_reached_logged = True

    def publish_visualizations(self):
        """Publish all visualization markers"""
        self.publish_robot_trajectory()
        self.publish_velocity_arrow()
        self.publish_lookahead_point()

    def publish_path(self, smooth_path):
        """Publish planned path"""
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in smooth_path:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_robot_trajectory(self):
        """Publish actual robot trajectory"""
        if len(self.robot_path_history) < 2:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.robot_path_history

        self.robot_path_pub.publish(path_msg)

    def publish_waypoint_markers(self):
        """Publish waypoint markers"""
        marker_array = MarkerArray()

        for i, waypoint in enumerate(self.waypoints):
            # Sphere marker
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'odom'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = waypoint[0]
            text_marker.pose.position.y = waypoint[1]
            text_marker.pose.position.z = 0.3
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"WP{i}"
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def publish_goal_marker(self):
        """Publish goal marker"""
        if not self.waypoints:
            return

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        goal = self.waypoints[-1]
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = 0.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.01

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        self.goal_marker_pub.publish(marker)

    def publish_velocity_arrow(self):
        """Publish velocity command as arrow"""
        if self.goal_reached:
            return

        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'velocity'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1

        marker.scale.x = 0.5  # Arrow length
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8

        self.velocity_arrow_pub.publish(marker)

    def publish_lookahead_point(self):
        """Publish lookahead point marker"""
        if self.goal_reached or self.current_idx >= len(self.trajectory):
            return

        lookahead_point, _ = self.controller.find_lookahead_point(
            self.robot_pose, self.trajectory, self.current_idx
        )

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lookahead'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = lookahead_point[0]
        marker.pose.position.y = lookahead_point[1]
        marker.pose.position.z = 0.1

        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.lookahead_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
