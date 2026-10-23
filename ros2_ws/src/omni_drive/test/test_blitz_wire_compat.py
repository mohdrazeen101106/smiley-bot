"""Assert the upstream Blitz.pack() contract used by the Pico receiver."""
import importlib.util
from pathlib import Path
import struct
import unittest
from types import SimpleNamespace


BLITZ_SOURCE = Path(__file__).resolve().parents[2] / "blitz_drive" / "blitz" / "blitz.py"
_spec = importlib.util.spec_from_file_location("upstream_blitz_module", BLITZ_SOURCE)
_blitz_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_blitz_module)
Blitz = _blitz_module.Blitz


class TestBlitzWireCompatibility(unittest.TestCase):
    def test_velocity_frame_is_upstream_aa_id5_little_endian_fff(self):
        velocity_schema = Blitz(
            topic="cmd_vel",
            msg_id=5,
            struct_fmt="fff",
            fields=["vx", "vy", "vw"],
            ros_msg=SimpleNamespace,
            from_mcu=False,
        )
        message = SimpleNamespace(vx=0.25, vy=-0.5, vw=1.0)
        actual = velocity_schema.pack(message)
        expected = struct.pack("<BBfff", 0xAA, 5, 0.25, -0.5, 1.0)
        self.assertEqual(len(actual), 14)
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
