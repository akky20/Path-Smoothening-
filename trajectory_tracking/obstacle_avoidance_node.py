#!/usr/bin/env python3
"""
Obstacle Avoidance Extension for Trajectory Tracking
Implements Dynamic Window Approach (DWA) for local obstacle avoidance
"""

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class ObstacleDetector:
    """Detects obstacles from laser scan data"""

    def __init__(self, safety_distance=0.5, danger_distance=0.3):
        self.safety_distance = safety_distance
        self.danger_distance = danger_distance
        self.obstacles = []

    def process_scan(self, scan_msg):
        """
        Process laser scan and detect obstacles
        Returns: List of obstacles in robot frame [(distance, angle), ...]
        """
        obstacles = []

        angle = scan_msg.angle_min
        for i, distance in enumerate(scan_msg.ranges):
            # Filter invalid readings
            if scan_msg.range_min < distance < scan_msg.range_max:
                # Check if obstacle is within safety distance
                if distance < self.safety_distance:
                    x = distance * np.cos(angle)
                    y = distance * np.sin(angle)
                    obstacles.append((x, y, distance))

            angle += scan_msg.angle_increment

        self.obstacles = obstacles
        return obstacles

    def is_path_clear(self, target_x, target_y):
        """Check if path to target is clear of obstacles"""
        if not self.obstacles:
            return True

        # Check if any obstacle is in the way
        for obs_x, obs_y, dist in self.obstacles:
            # Simple line-of-sight check
            target_angle = np.arctan2(target_y, target_x)
            obs_angle = np.arctan2(obs_y, obs_x)

            angle_diff = abs(self.normalize_angle(target_angle - obs_angle))

            if angle_diff < 0.2 and dist < self.danger_distance:
                return False

        return True

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class DynamicWindowApproach:
    """
    Dynamic Window Approach for local path planning
    Evaluates velocity commands based on obstacles and goals
    """

    def __init__(self, max_speed=0.5, max_yaw_rate=1.0,
                 max_accel=0.5, max_dyaw_rate=1.5,
                 velocity_resolution=0.05, yaw_rate_resolution=0.1,
                 dt=0.1):

        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.max_accel = max_accel
        self.max_dyaw_rate = max_dyaw_rate
        self.velocity_resolution = velocity_resolution
        self.yaw_rate_resolution = yaw_rate_resolution
        self.dt = dt

        # Cost function weights
        self.heading_weight = 0.5
        self.clearance_weight = 0.3
        self.velocity_weight = 0.2

    def calculate_dynamic_window(self, current_v, current_w):
        """
        Calculate dynamic window of feasible velocities
        Returns: (v_min, v_max, w_min, w_max)
        """
        # Kinematic constraints
        v_min = 0
        v_max = self.max_speed
        w_min = -self.max_yaw_rate
        w_max = self.max_yaw_rate

        # Dynamic constraints
        v_min = max(v_min, current_v - self.max_accel * self.dt)
        v_max = min(v_max, current_v + self.max_accel * self.dt)
        w_min = max(w_min, current_w - self.max_dyaw_rate * self.dt)
        w_max = min(w_max, current_w + self.max_dyaw_rate * self.dt)

        return v_min, v_max, w_min, w_max

    def predict_trajectory(self, v, w, robot_pose, prediction_time=2.0):
        """
        Predict robot trajectory for given velocity
        Returns: List of predicted poses [(x, y, theta), ...]
        """
        x, y, theta = robot_pose
        trajectory = [(x, y, theta)]

        steps = int(prediction_time / self.dt)
        for _ in range(steps):
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            trajectory.append((x, y, theta))

        return trajectory

    def calculate_heading_cost(self, trajectory, goal):
        """Cost based on heading to goal (lower is better)"""
        final_pose = trajectory[-1]
        dx = goal[0] - final_pose[0]
        dy = goal[1] - final_pose[1]
        goal_angle = np.arctan2(dy, dx)
        heading_error = abs(self.normalize_angle(goal_angle - final_pose[2]))
        return heading_error

    def calculate_clearance_cost(self, trajectory, obstacles):
        """Cost based on distance to obstacles (lower is better)"""
        if not obstacles:
            return 0

        min_clearance = float('inf')

        for pose in trajectory:
            for obs_x, obs_y, _ in obstacles:
                distance = np.sqrt((pose[0] - obs_x)**2 + (pose[1] - obs_y)**2)
                min_clearance = min(min_clearance, distance)

        # Inverse clearance (higher clearance = lower cost)
        if min_clearance < 0.3:
            return 1000  # Collision penalty

        return 1.0 / min_clearance

    def calculate_velocity_cost(self, v):
        """Cost for low velocity (prefer higher speeds)"""
        return (self.max_speed - v) / self.max_speed

    def select_velocity(
            self,
            robot_pose,
            current_v,
            current_w,
            goal,
            obstacles):
        """
        Select optimal velocity command using DWA
        Returns: (best_v, best_w)
        """
        v_min, v_max, w_min, w_max = self.calculate_dynamic_window(
            current_v, current_w)

        best_v = current_v
        best_w = current_w
        min_cost = float('inf')

        # Sample velocities in dynamic window
        for v in np.arange(v_min, v_max, self.velocity_resolution):
            for w in np.arange(w_min, w_max, self.yaw_rate_resolution):
                # Predict trajectory
                trajectory = self.predict_trajectory(v, w, robot_pose)

                # Calculate costs
                heading_cost = self.calculate_heading_cost(trajectory, goal)
                clearance_cost = self.calculate_clearance_cost(
                    trajectory, obstacles)
                velocity_cost = self.calculate_velocity_cost(v)

                # Total cost
                total_cost = (self.heading_weight * heading_cost +
                              self.clearance_weight * clearance_cost +
                              self.velocity_weight * velocity_cost)

                # Update best velocity
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_v = v
                    best_w = w

        return best_v, best_w

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class ObstacleAvoidanceNode(Node):
    """
    ROS2 Node integrating obstacle avoidance with trajectory tracking
    """

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Initialize components
        self.obstacle_detector = ObstacleDetector(safety_distance=0.8)
        self.dwa = DynamicWindowApproach(max_speed=0.3, max_yaw_rate=1.0)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # State
        self.current_velocity = 0.0
        self.current_yaw_rate = 0.0
        self.robot_pose = [0.0, 0.0, 0.0]
        self.goal = [5.0, 5.0]
        self.obstacles = []

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Node Started!')

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.obstacles = self.obstacle_detector.process_scan(msg)

    def control_loop(self):
        """Main control loop with obstacle avoidance"""
        if not self.obstacles:
            # No obstacles detected, use original controller
            return

        # Use DWA to select velocity
        best_v, best_w = self.dwa.select_velocity(
            self.robot_pose,
            self.current_velocity,
            self.current_yaw_rate,
            self.goal,
            self.obstacles
        )

        # Publish command
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_w
        self.cmd_vel_pub.publish(cmd)

        # Update state
        self.current_velocity = best_v
        self.current_yaw_rate = best_w


class LocalPathReplanner:
    """
    Replans smooth path locally to avoid obstacles
    """

    def __init__(self, replanning_distance=2.0):
        self.replanning_distance = replanning_distance

    def replan_around_obstacle(self, current_pose, goal, obstacle_position):
        """
        Generate alternative waypoints to avoid obstacle
        Returns: List of new waypoints
        """
        # Calculate avoidance waypoints
        cx, cy = current_pose[0], current_pose[1]
        gx, gy = goal[0], goal[1]
        ox, oy = obstacle_position[0], obstacle_position[1]

        # Calculate perpendicular offset direction
        dx = gx - cx
        dy = gy - cy

        # Perpendicular direction
        perp_x = -dy
        perp_y = dx
        perp_norm = np.sqrt(perp_x**2 + perp_y**2)

        if perp_norm > 0:
            perp_x /= perp_norm
            perp_y /= perp_norm

        # Create avoidance waypoints
        offset = 0.5  # meters
        waypoint1 = (cx + 0.3 * dx + offset * perp_x,
                     cy + 0.3 * dy + offset * perp_y)
        waypoint2 = (cx + 0.7 * dx + offset * perp_x,
                     cy + 0.7 * dy + offset * perp_y)

        return [current_pose[:2], waypoint1, waypoint2, (gx, gy)]

    def smooth_replanned_path(self, waypoints):
        """Apply smoothing to replanned path"""
        from scipy.interpolate import CubicSpline

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

        cs_x = CubicSpline(distances, waypoints[:, 0])
        cs_y = CubicSpline(distances, waypoints[:, 1])

        alpha = np.linspace(0, distances[-1], 50)
        smooth_path = np.column_stack((cs_x(alpha), cs_y(alpha)))

        return smooth_path


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
