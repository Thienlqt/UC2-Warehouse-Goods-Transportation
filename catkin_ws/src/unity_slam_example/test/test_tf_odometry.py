import math
import unittest

from unity_slam_example.tf_odometry import planar_velocity


class PlanarVelocityTests(unittest.TestCase):
    def test_forward_motion_when_robot_faces_world_y(self):
        velocity = planar_velocity((1.0, 2.0, 3.0, math.pi / 2),
                                   (1.1, 2.0, 3.02, math.pi / 2))
        self.assertAlmostEqual(velocity[0], 0.2)
        self.assertAlmostEqual(velocity[1], 0.0)
        self.assertAlmostEqual(velocity[2], 0.0)

    def test_yaw_wrap_does_not_create_velocity_spike(self):
        velocity = planar_velocity((1.0, 0.0, 0.0, math.pi - 0.01),
                                   (1.1, 0.0, 0.0, -math.pi + 0.01))
        self.assertAlmostEqual(velocity[2], 0.2)

    def test_duplicate_backward_and_stale_samples_are_rejected(self):
        for stamp in (1.0, 0.0, 1.6):
            self.assertIsNone(planar_velocity((1.0, 0.0, 0.0, 0.0),
                                             (stamp, 1.0, 0.0, 0.0)))

    def test_stationary_and_reverse_motion(self):
        self.assertEqual(planar_velocity((1.0, 0.0, 0.0, 0.0),
                                         (1.1, 0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(planar_velocity((1.0, 0.0, 0.0, 0.0),
                                              (1.1, -0.02, 0.0, 0.0))[0], -0.2)
