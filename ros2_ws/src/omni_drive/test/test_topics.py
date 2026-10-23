from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class TestSimpleTopicNames(unittest.TestCase):
    def test_velocity_topic_is_cmd_vel(self):
        source = (PACKAGE_ROOT / "omni_drive" / "joy_to_velocity.py").read_text()
        config = (PACKAGE_ROOT / "config" / "omni_drive.yaml").read_text()
        self.assertIn('"velocity_topic": "/cmd_vel"', source)
        self.assertIn("velocity_topic: /cmd_vel", config)
        self.assertNotIn("/r2/cmd_vel", source + config)
        self.assertNotIn("/rs/cmd_vel", source + config)

    def test_launch_does_not_push_namespace(self):
        launch = (PACKAGE_ROOT / "launch" / "omni_drive.launch.py").read_text()
        self.assertNotIn("PushRosNamespace", launch)
        self.assertNotIn("GroupAction", launch)


if __name__ == "__main__":
    unittest.main()
