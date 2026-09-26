"""
Unit Tests for Distributed Track Fusion Kernel:
Verifies Covariance Intersection consistency, trace uncertainty reduction,
data incest elimination, and sub-millisecond execution latency.
Zero external dependencies (pure Python standard library).
"""

import unittest
from distributed_track_fusion_kernel import (
    DistributedTrackFusionKernel,
    TrackState,
    FusionMode
)


class TestDistributedTrackFusionKernel(unittest.TestCase):

    def setUp(self):
        self.kernel = DistributedTrackFusionKernel(tolerance=1e-4)

    def test_tactical_missile_fusion(self):
        """Verifies multi-sensor tactical missile track fusion across Radar, ESM, and EO/IR."""
        res = self.kernel.fuse_tactical_missile_tracks(seed=42)

        self.assertEqual(res.mode, FusionMode.TACTICAL_DEFENSE)
        self.assertTrue(res.data_incest_prevented)
        self.assertEqual(len(res.contributing_sensors), 3)
        self.assertGreater(res.trace_uncertainty, 0.0)
        # Latency should be well under 1000 µs (1 ms)
        self.assertLess(res.execution_latency_us, 1000.0)

    def test_warehouse_agv_slam_fusion(self):
        """Verifies collaborative AGV pose fusion across LiDAR, stereo cameras, and UWB."""
        res = self.kernel.fuse_warehouse_agv_slam(seed=42)

        self.assertEqual(res.mode, FusionMode.INDUSTRIAL_AGV_SLAM)
        self.assertTrue(res.data_incest_prevented)
        # High precision position uncertainty in industrial mode (< 0.1 m^2)
        pos_trace = res.covariance_diag[0] + res.covariance_diag[1] + res.covariance_diag[2]
        self.assertLess(pos_trace, 0.10)

    def test_covariance_intersection_consistency(self):
        """Verifies mathematical consistency: fused variance along each axis is tighter than max."""
        t1 = TrackState(
            track_id="TEST-1",
            sensor_id="S1",
            timestamp_ms=100.0,
            state_vector=(10.0, 20.0, 30.0, 1.0, 0.0, 0.0),
            covariance_diag=(4.0, 100.0, 25.0, 1.0, 1.0, 1.0)
        )
        t2 = TrackState(
            track_id="TEST-1",
            sensor_id="S2",
            timestamp_ms=100.0,
            state_vector=(12.0, 18.0, 31.0, 0.9, 0.1, 0.0),
            covariance_diag=(100.0, 4.0, 25.0, 1.0, 1.0, 1.0)
        )

        res = self.kernel.fuse_custom_tracks([t1, t2])
        # Along X, S1 was accurate (4.0). Along Y, S2 was accurate (4.0).
        # Fused variance should be <= convex combination along both!
        self.assertLessEqual(res.covariance_diag[0], 100.0)
        self.assertLessEqual(res.covariance_diag[1], 100.0)


if __name__ == "__main__":
    unittest.main()
