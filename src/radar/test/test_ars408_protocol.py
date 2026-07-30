import unittest

from radar.ars408_protocol import (
    decode_cluster_general,
    decode_cluster_status,
    decode_object_general,
    decode_object_status,
)


class Ars408ProtocolTest(unittest.TestCase):
    def test_object_status(self):
        status = decode_object_status(bytes.fromhex("0300123030000000"))
        self.assertEqual(status.count, 3)
        self.assertEqual(status.measurement_counter, 0x1230)
        self.assertEqual(status.interface_version, 3)

    def test_object_general_zero_offsets(self):
        # Encodes raw zero for each signed/offset field in Obj_1_General.
        target = decode_object_general(bytes(8))
        self.assertEqual(target.object_id, 0)
        self.assertEqual(target.x_m, -500.0)
        self.assertEqual(target.y_m, -204.6)
        self.assertEqual(target.vx_mps, -128.0)
        self.assertEqual(target.vy_mps, -64.0)
        self.assertEqual(target.rcs_dbm2, -64.0)

    def test_object_general_zero_physical_values(self):
        # Manual Figure 28 (0x60B): x/y/vx/vy/RCS raw values at their offsets.
        target = decode_object_general(bytes.fromhex("2a4e23ff80200680"))
        self.assertEqual(target.object_id, 0x2A)
        self.assertAlmostEqual(target.x_m, 0.0)
        self.assertAlmostEqual(target.y_m, 0.0)
        self.assertAlmostEqual(target.vx_mps, 0.0)
        self.assertAlmostEqual(target.vy_mps, 0.0)
        self.assertAlmostEqual(target.rcs_dbm2, 0.0)
        self.assertEqual(target.dynamic_property, 6)

    def test_live_cluster_status_layout(self):
        status = decode_cluster_status(bytes.fromhex("180c59bd10"))
        self.assertEqual(status.near_count, 24)
        self.assertEqual(status.far_count, 12)
        self.assertEqual(status.count, 36)
        self.assertEqual(status.measurement_counter, 0x59BD)
        self.assertEqual(status.interface_version, 1)

    def test_live_cluster_general_layout(self):
        # Captured 0x701 payload, decoded according to manual Figure 22.
        target = decode_cluster_general(bytes.fromhex("004e79fd80200178"))
        self.assertAlmostEqual(target.x_m, 2.2)
        self.assertAlmostEqual(target.y_m, -0.5)
        self.assertAlmostEqual(target.vx_mps, 0.0)
        self.assertAlmostEqual(target.vy_mps, 0.0)
        self.assertAlmostEqual(target.rcs_dbm2, -4.0)
        self.assertEqual(target.dynamic_property, 1)


if __name__ == "__main__":
    unittest.main()
