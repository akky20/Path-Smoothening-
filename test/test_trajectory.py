#!/usr/bin/env python3
import unittest
import numpy as np
from trajectory_tracking.trajectory_node import PathSmoother, TrajectoryGenerator, PurePursuitController


class TestPathSmoother(unittest.TestCase):
    def setUp(self):
        self.smoother = PathSmoother(smoothing_factor=50)

    def test_straight_line(self):
        waypoints = [(0, 0), (1, 0), (2, 0)]
        smooth_path = self.smoother.smooth_path(waypoints)
        self.assertEqual(len(smooth_path), 50)
        self.assertAlmostEqual(smooth_path[0, 1], 0.0, places=2)

    def test_single_waypoint(self):
        waypoints = [(0, 0)]
        smooth_path = self.smoother.smooth_path(waypoints)
        self.assertEqual(len(smooth_path), 1)


class TestTrajectoryGenerator(unittest.TestCase):
    def setUp(self):
        self.generator = TrajectoryGenerator(max_velocity=0.5)

    def test_trajectory_generation(self):
        path = np.array([[0, 0], [1, 0], [2, 0]])
        trajectory = self.generator.generate_trajectory(path)
        self.assertGreater(len(trajectory), 0)
        # Check time increases
        times = [t[4] for t in trajectory]
        self.assertEqual(times, sorted(times))


class TestPurePursuitController(unittest.TestCase):
    def setUp(self):
        self.controller = PurePursuitController(lookahead_distance=0.5)

    def test_goal_reached(self):
        robot_pose = [0, 0, 0]
        trajectory = [(0.05, 0, 0, 0.3, 0)]
        linear_vel, angular_vel, idx, reached = self.controller.compute_control(
            robot_pose, trajectory, 0)
        self.assertTrue(reached)

    def test_angle_normalization(self):
        angle = self.controller.normalize_angle(4 * np.pi)
        self.assertAlmostEqual(angle, 0.0, places=5)
