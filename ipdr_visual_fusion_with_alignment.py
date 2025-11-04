"""
iPDR Visual Fusion with Coordinate Frame Alignment
==================================================

This is an improved version that properly handles coordinate frame alignment
between iPDR local frame and COLMAP global frame.

Key improvements:
1. Automatic coordinate frame alignment
2. Transforms visual measurements to iPDR local frame
3. Supports multiple alignment strategies

Author: Claude
Date: 2025-10-29
"""

import numpy as np
from typing import Optional
from ipdr_visual_fusion import (
    iPDRVisualFusion,
    VisualFusionConfig,
    EKFState,
    VisualMeasurement
)
from coordinate_alignment import FrameAlignmentManager


class iPDRVisualFusionAligned:
    """
    iPDR Visual Fusion with automatic coordinate frame alignment

    This class wraps the standard iPDRVisualFusion and adds coordinate
    frame alignment to handle the transformation between:
    - iPDR local frame (starts at origin)
    - COLMAP global frame (world coordinates)
    """

    def __init__(self, config: Optional[VisualFusionConfig] = None,
                 alignment_mode: str = 'first_visual'):
        """
        Initialize fusion system with alignment

        Args:
            config: Visual fusion configuration
            alignment_mode: 'first_visual', 'multi_point', or 'pre_calibrated'
        """
        self.fusion = iPDRVisualFusion(config)
        self.aligner = FrameAlignmentManager(mode=alignment_mode)

        # Track alignment status
        self.alignment_initialized = False
        self.measurements_before_alignment = []

    def initialize_state(self, initial_state: EKFState):
        """Initialize EKF state in LOCAL frame"""
        self.fusion.initialize_state(initial_state)

    def predict_step(self, dt: float, acc: np.ndarray, gyro: np.ndarray,
                    step_len: Optional[float] = None):
        """EKF prediction step (same as before)"""
        self.fusion.predict_step(dt, acc, gyro, step_len)

    def add_visual_measurement_global(self, t_v: float, p_global: np.ndarray,
                                     yaw_global: float, e_repr: float, n_inlier: int):
        """
        Add visual measurement in GLOBAL (COLMAP) frame

        This will:
        1. If alignment not ready: collect alignment data
        2. If alignment ready: transform to local frame and add to fusion

        Args:
            t_v: Timestamp
            p_global: Position in COLMAP global frame [x, y, z]
            yaw_global: Yaw in COLMAP global frame (radians)
            e_repr: Reprojection error
            n_inlier: Number of inliers
        """
        # Get current iPDR position in local frame
        p_local_current, ori_current = self.fusion.get_current_pose()
        yaw_local_current = ori_current[0]

        # Process through alignment manager
        p_local, yaw_local = self.aligner.process_visual_measurement(
            p_global, yaw_global,
            p_local_current, yaw_local_current
        )

        if p_local is None:
            # Alignment not ready yet, store for later
            self.measurements_before_alignment.append({
                't_v': t_v,
                'p_global': p_global,
                'yaw_global': yaw_global,
                'e_repr': e_repr,
                'n_inlier': n_inlier
            })
            return False

        # Alignment is ready!
        if not self.alignment_initialized:
            self.alignment_initialized = True
            print(f"✓ Coordinate alignment established!")
            print(f"  {len(self.measurements_before_alignment)} measurements queued for processing")

            # Process any queued measurements (optional: depends on OOSM capability)
            # For now, we'll just start from this point forward

        # Create visual measurement in LOCAL frame
        vis_meas = VisualMeasurement(
            t_v=t_v,
            p_W=p_local,  # Now in local frame
            yaw_W=yaw_local,  # Now in local frame
            e_repr=e_repr,
            n_inlier=n_inlier
        )

        # Add to fusion system
        self.fusion.add_visual_measurement(vis_meas)
        return True

    def process_visual_measurements(self) -> int:
        """Process all pending visual measurements"""
        return self.fusion.process_visual_measurements()

    def get_current_pose_local(self) -> tuple:
        """Get current pose in LOCAL frame"""
        return self.fusion.get_current_pose()

    def get_current_pose_global(self) -> tuple:
        """Get current pose in GLOBAL frame (COLMAP coordinates)"""
        if not self.alignment_initialized:
            raise RuntimeError("Alignment not initialized, cannot provide global pose")

        # Get local pose
        p_local, ori_local = self.fusion.get_current_pose()
        yaw_local = ori_local[0]

        # Transform to global
        p_global = self.aligner.transform.transform_position(p_local)
        yaw_global = self.aligner.transform.transform_orientation(yaw_local)

        ori_global = ori_local.copy()
        ori_global[0] = yaw_global

        return p_global, ori_global

    def get_statistics(self):
        """Get fusion and alignment statistics"""
        stats = self.fusion.get_statistics()
        stats['alignment'] = self.aligner.get_transform_summary()
        stats['queued_before_alignment'] = len(self.measurements_before_alignment)
        return stats


# ============================================================================
# Example Usage
# ============================================================================

def example_with_alignment():
    """Example showing proper coordinate frame handling"""

    print("=" * 70)
    print("iPDR Visual Fusion with Coordinate Alignment")
    print("=" * 70)

    # Create system with alignment
    config = VisualFusionConfig()
    config.max_pos_innovation = 3.0

    system = iPDRVisualFusionAligned(
        config=config,
        alignment_mode='multi_point'  # Use multiple points for robust alignment
    )

    # Initialize in LOCAL frame (iPDR starts at origin)
    initial_x = np.zeros(10)
    initial_x[0:3] = [0, 0, 0]  # Local origin
    initial_x[3:6] = [0, 0, 0]  # Facing east (local coordinate)
    initial_P = np.eye(10) * 0.1

    initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
    system.initialize_state(initial_state)

    print("\n✓ System initialized in LOCAL frame")
    print(f"  Initial position (local): [0, 0, 0]")
    print(f"  Initial orientation: 0° (facing east in local frame)")

    # Simulate walking scenario
    # iPDR thinks it's walking from (0,0) eastward
    # But in COLMAP global frame, it's actually at (100, 50) and walking north

    print("\n" + "=" * 70)
    print("Simulation: Walking 5 meters with visual measurements")
    print("=" * 70)

    # Ground truth transform (unknown to system)
    # Local east = Global north, offset by (100, 50)
    true_R = np.array([
        [0, -1, 0],  # 90° rotation
        [1,  0, 0],
        [0,  0, 1]
    ])
    true_t = np.array([100, 50, 0])

    dt = 0.01  # 100 Hz
    t = 0.0
    step_length = 0.7

    visual_times = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0]  # When visual arrives

    for step in range(500):  # 5 seconds
        t = step * dt

        # IMU update (local frame)
        acc = np.array([0.1, 0.0, 9.81]) + np.random.randn(3) * 0.05
        gyro = np.array([0.0, 0.0, 0.0]) + np.random.randn(3) * 0.01

        system.predict_step(dt, acc, gyro)

        # Visual measurement (global frame COLMAP)
        if t in visual_times or any(abs(t - vt) < dt for vt in visual_times):
            # Get true position in local frame
            p_local_true, ori_local = system.get_current_pose_local()

            # Simulate COLMAP measurement (in global frame)
            # Add noise
            p_global_true = true_R @ p_local_true + true_t
            p_global_meas = p_global_true + np.random.randn(3) * 0.2

            yaw_local_true = ori_local[0]
            yaw_offset = np.arctan2(true_R[1, 0], true_R[0, 0])
            yaw_global_meas = yaw_local_true + yaw_offset + np.random.randn() * 0.02

            # Add visual measurement in GLOBAL frame
            success = system.add_visual_measurement_global(
                t_v=t - 0.1,  # 100ms delay
                p_global=p_global_meas,
                yaw_global=yaw_global_meas,
                e_repr=0.5,
                n_inlier=50
            )

            if success:
                # Process visual update
                num_updates = system.process_visual_measurements()
                if num_updates > 0:
                    print(f"\n✓ Visual update at t={t:.2f}s")

                    # Show both local and global poses
                    p_local, ori_local = system.get_current_pose_local()
                    p_global, ori_global = system.get_current_pose_global()

                    print(f"  Local pose:  pos={p_local}, yaw={np.rad2deg(ori_local[0]):.1f}°")
                    print(f"  Global pose: pos={p_global}, yaw={np.rad2deg(ori_global[0]):.1f}°")

    # Final statistics
    print("\n" + "=" * 70)
    print("Final Results")
    print("=" * 70)

    stats = system.get_statistics()

    print("\n📍 Final Pose (Local Frame):")
    p_local, ori_local = system.get_current_pose_local()
    print(f"   Position: {p_local}")
    print(f"   Orientation: yaw={np.rad2deg(ori_local[0]):.1f}°")

    if system.alignment_initialized:
        print("\n🌍 Final Pose (Global Frame):")
        p_global, ori_global = system.get_current_pose_global()
        print(f"   Position: {p_global}")
        print(f"   Orientation: yaw={np.rad2deg(ori_global[0]):.1f}°")

    print("\n📊 Statistics:")
    print(f"   Visual measurements: {stats['total_visual_meas']}")
    print(f"   Visual updates: {stats['visual_updates']}")
    print(f"   Acceptance rate: {stats['acceptance_rate']*100:.1f}%")
    print(f"   Queued before alignment: {stats['queued_before_alignment']}")

    if stats['alignment']['initialized']:
        print("\n🔄 Coordinate Alignment:")
        print(f"   Mode: {stats['alignment']['mode']}")
        print(f"   Rotation: {stats['alignment']['rotation_yaw_deg']:.2f}°")
        print(f"   Translation: {stats['alignment']['translation']}")
        print(f"   Alignment pairs used: {stats['alignment']['num_alignment_pairs']}")

    print("\n" + "=" * 70)
    print("✅ Example completed!")
    print("=" * 70)


if __name__ == "__main__":
    example_with_alignment()
