import unittest
from omni_drive.velocity_mapping import map_velocity, shape_axis


class TestVelocityMapping(unittest.TestCase):
    def test_deadzone(self):
        self.assertEqual(shape_axis(0.03, 0.05), 0.0)

    def test_forward_sign_correction(self):
        vx, vy, vw = map_velocity(-1.0, 0.0, 0.0, forward_sign=-1.0, lateral_sign=-1.0)
        self.assertAlmostEqual(vx, 1.0)
        self.assertAlmostEqual(vy, 0.0)
        self.assertAlmostEqual(vw, 0.0)

    def test_strafe_sign(self):
        vx, vy, vw = map_velocity(0.0, -1.0, 0.0, forward_sign=-1.0, lateral_sign=-1.0)
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, 1.0)
        self.assertAlmostEqual(vw, 0.0)

    def test_yaw_button_value(self):
        vx, vy, vw = map_velocity(0.0, 0.0, 1.0, forward_sign=-1.0, lateral_sign=-1.0)
        self.assertAlmostEqual(vw, 1.0)

    def test_outputs_clamped(self):
        values = map_velocity(8.0, -8.0, 3.0)
        self.assertTrue(all(-1.0 <= value <= 1.0 for value in values))


if __name__ == "__main__":
    unittest.main()
