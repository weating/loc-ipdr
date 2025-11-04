"""
iPDR-Visual Coordinate Frame Alignment Module
=============================================

This module handles coordinate frame alignment between:
- iPDR local frame (starts at origin, arbitrary orientation)
- COLMAP global frame (fixed world coordinate system)

Three alignment strategies are provided:
1. Initial alignment: Align at first visual measurement
2. Continuous alignment: Online estimation of transformation
3. Pre-calibrated alignment: Use known transformation

Author: Claude
Date: 2025-10-29
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class CoordinateTransform:
    """
    Transformation from iPDR local frame to COLMAP global frame

    T_global = scale * R * T_local + t

    Attributes:
        R: Rotation matrix (3x3) from local to global
        t: Translation vector (3,) from local to global
        scale: Scale factor (usually 1.0 for metric systems)
        is_initialized: Whether transform has been computed
    """
    R: np.ndarray = None  # 3x3 rotation matrix
    t: np.ndarray = None  # 3x1 translation vector
    scale: float = 1.0
    is_initialized: bool = False

    def transform_position(self, p_local: np.ndarray) -> np.ndarray:
        """
        Transform position from local to global frame

        Args:
            p_local: Position in iPDR local frame [x, y, z]

        Returns:
            Position in COLMAP global frame [x, y, z]
        """
        if not self.is_initialized:
            raise RuntimeError("Transform not initialized")

        return self.scale * (self.R @ p_local) + self.t

    def transform_position_inverse(self, p_global: np.ndarray) -> np.ndarray:
        """
        Transform position from global to local frame

        Args:
            p_global: Position in COLMAP global frame [x, y, z]

        Returns:
            Position in iPDR local frame [x, y, z]
        """
        if not self.is_initialized:
            raise RuntimeError("Transform not initialized")

        return self.R.T @ ((p_global - self.t) / self.scale)

    def transform_orientation(self, yaw_local: float) -> float:
        """
        Transform yaw angle from local to global frame

        Args:
            yaw_local: Yaw in local frame (radians)

        Returns:
            Yaw in global frame (radians)
        """
        if not self.is_initialized:
            raise RuntimeError("Transform not initialized")

        # Extract yaw rotation from R matrix
        yaw_offset = np.arctan2(self.R[1, 0], self.R[0, 0])

        return yaw_local + yaw_offset


class FrameAlignmentManager:
    """
    Manages coordinate frame alignment between iPDR and COLMAP

    Supports three alignment modes:
    1. 'first_visual': Align at first visual measurement
    2. 'multi_point': Use multiple measurements for robust alignment
    3. 'pre_calibrated': Use known transformation
    """

    def __init__(self, mode: str = 'first_visual'):
        """
        Initialize frame alignment manager

        Args:
            mode: Alignment mode ('first_visual', 'multi_point', 'pre_calibrated')
        """
        self.mode = mode
        self.transform = CoordinateTransform()

        # For multi-point alignment
        self.alignment_pairs = []  # List of (p_local, p_global, yaw_local, yaw_global)
        self.min_pairs_required = 3  # Minimum pairs for robust estimation

    def set_pre_calibrated_transform(self, R: np.ndarray, t: np.ndarray, scale: float = 1.0):
        """
        Set pre-calibrated transformation

        Args:
            R: Rotation matrix (3x3)
            t: Translation vector (3,)
            scale: Scale factor
        """
        self.transform.R = R.copy()
        self.transform.t = t.copy()
        self.transform.scale = scale
        self.transform.is_initialized = True

    def add_alignment_pair(self, p_local: np.ndarray, p_global: np.ndarray,
                           yaw_local: float, yaw_global: float):
        """
        Add a pair of corresponding positions for alignment

        Args:
            p_local: Position in iPDR local frame
            p_global: Corresponding position in COLMAP global frame
            yaw_local: Yaw in local frame
            yaw_global: Corresponding yaw in global frame
        """
        self.alignment_pairs.append({
            'p_local': p_local.copy(),
            'p_global': p_global.copy(),
            'yaw_local': yaw_local,
            'yaw_global': yaw_global
        })

    def estimate_transform(self) -> bool:
        """
        Estimate transformation from collected alignment pairs

        Returns:
            True if estimation successful
        """
        if len(self.alignment_pairs) < self.min_pairs_required:
            return False

        if self.mode == 'first_visual':
            # Use only first pair
            return self._estimate_from_single_pair(self.alignment_pairs[0])

        elif self.mode == 'multi_point':
            # Use least squares with multiple pairs
            return self._estimate_from_multiple_pairs()

        return False

    def _estimate_from_single_pair(self, pair: dict) -> bool:
        """
        Estimate transform from a single correspondence

        This assumes:
        - No scale difference (scale = 1.0)
        - Rotation only in yaw (2D alignment)
        - Translation to match positions

        Args:
            pair: Dictionary with 'p_local', 'p_global', 'yaw_local', 'yaw_global'

        Returns:
            True if successful
        """
        # Compute yaw difference
        delta_yaw = pair['yaw_global'] - pair['yaw_local']

        # Create 3D rotation matrix (rotation around z-axis)
        cos_yaw = np.cos(delta_yaw)
        sin_yaw = np.sin(delta_yaw)

        self.transform.R = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [0,        0,       1]
        ])

        # Compute translation
        # p_global = R * p_local + t
        # t = p_global - R * p_local
        self.transform.t = pair['p_global'] - self.transform.R @ pair['p_local']

        self.transform.scale = 1.0
        self.transform.is_initialized = True

        return True

    def _estimate_from_multiple_pairs(self) -> bool:
        """
        Estimate transform from multiple correspondences using least squares

        This uses Umeyama's algorithm for similarity transformation estimation

        Returns:
            True if successful
        """
        if len(self.alignment_pairs) < 2:
            return False

        # Extract positions
        P_local = np.array([pair['p_local'] for pair in self.alignment_pairs]).T  # 3 x N
        P_global = np.array([pair['p_global'] for pair in self.alignment_pairs]).T  # 3 x N

        # Compute centroids
        mu_local = np.mean(P_local, axis=1, keepdims=True)
        mu_global = np.mean(P_global, axis=1, keepdims=True)

        # Center the points
        P_local_centered = P_local - mu_local
        P_global_centered = P_global - mu_global

        # Compute covariance matrix
        H = P_local_centered @ P_global_centered.T

        # SVD
        U, S, Vt = np.linalg.svd(H)

        # Compute rotation
        R = Vt.T @ U.T

        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        # Compute scale (optional, usually 1.0 for metric systems)
        var_local = np.sum(P_local_centered ** 2) / P_local_centered.shape[1]
        scale = np.sum(S) / var_local if var_local > 0 else 1.0

        # For metric systems, force scale = 1.0
        scale = 1.0

        # Compute translation
        t = mu_global - scale * R @ mu_local

        self.transform.R = R
        self.transform.t = t.flatten()
        self.transform.scale = scale
        self.transform.is_initialized = True

        return True

    def process_visual_measurement(self, p_global: np.ndarray, yaw_global: float,
                                   p_local_current: np.ndarray, yaw_local_current: float) \
            -> Tuple[Optional[np.ndarray], Optional[float]]:
        """
        Process visual measurement with frame alignment

        Strategy:
        - If transform not initialized: collect alignment pair and try to estimate
        - If transform initialized: transform visual measurement to local frame

        Args:
            p_global: Visual position in global frame
            yaw_global: Visual yaw in global frame
            p_local_current: Current iPDR position in local frame
            yaw_local_current: Current iPDR yaw in local frame

        Returns:
            (p_local, yaw_local): Transformed measurement in local frame,
                                  or (None, None) if alignment not ready
        """
        if not self.transform.is_initialized:
            # Collect alignment pair
            self.add_alignment_pair(p_local_current, p_global,
                                   yaw_local_current, yaw_global)

            # Try to estimate transform
            if self.estimate_transform():
                print(f"✓ Frame alignment initialized with {len(self.alignment_pairs)} pairs")
                print(f"  Rotation (yaw): {np.rad2deg(np.arctan2(self.transform.R[1,0], self.transform.R[0,0])):.2f}°")
                print(f"  Translation: {self.transform.t}")
            else:
                print(f"⏳ Collecting alignment pairs: {len(self.alignment_pairs)}/{self.min_pairs_required}")
                return None, None

        # Transform visual measurement to local frame
        p_local = self.transform.transform_position_inverse(p_global)

        # Transform yaw
        yaw_offset = np.arctan2(self.transform.R[1, 0], self.transform.R[0, 0])
        yaw_local = yaw_global - yaw_offset

        return p_local, yaw_local

    def get_transform_summary(self) -> dict:
        """Get summary of current transformation"""
        if not self.transform.is_initialized:
            return {'initialized': False}

        yaw_deg = np.rad2deg(np.arctan2(self.transform.R[1, 0], self.transform.R[0, 0]))

        return {
            'initialized': True,
            'mode': self.mode,
            'rotation_yaw_deg': yaw_deg,
            'translation': self.transform.t.tolist(),
            'scale': self.transform.scale,
            'num_alignment_pairs': len(self.alignment_pairs)
        }


# ============================================================================
# Example Usage
# ============================================================================

def example_usage():
    """Example of coordinate frame alignment"""

    print("=" * 70)
    print("Coordinate Frame Alignment Example")
    print("=" * 70)

    # Scenario: iPDR starts at (0, 0, 0) facing east (yaw=0)
    #           COLMAP coordinate: actual position is (10, 5, 0) facing north (yaw=π/2)

    # Create alignment manager
    aligner = FrameAlignmentManager(mode='first_visual')

    # Simulate iPDR walking
    ipdr_positions = [
        np.array([0.0, 0.0, 0.0]),  # Start
        np.array([1.0, 0.0, 0.0]),  # Walk 1m east
        np.array([2.0, 0.0, 0.0]),  # Walk 2m east
    ]

    ipdr_yaws = [0.0, 0.0, 0.0]  # Facing east

    # Corresponding COLMAP observations (with transformation)
    # True transform: R = 90° CCW, t = [10, 5, 0]
    colmap_positions = [
        np.array([10.0, 5.0, 0.0]),  # Corresponds to iPDR (0, 0, 0)
        np.array([10.0, 6.0, 0.0]),  # Corresponds to iPDR (1, 0, 0)
        np.array([10.0, 7.0, 0.0]),  # Corresponds to iPDR (2, 0, 0)
    ]

    colmap_yaws = [np.pi/2, np.pi/2, np.pi/2]  # Facing north

    # Process measurements
    print("\nProcessing visual measurements:")
    for i in range(len(ipdr_positions)):
        print(f"\n--- Measurement {i+1} ---")
        print(f"iPDR (local):   pos={ipdr_positions[i]}, yaw={np.rad2deg(ipdr_yaws[i]):.1f}°")
        print(f"COLMAP (global): pos={colmap_positions[i]}, yaw={np.rad2deg(colmap_yaws[i]):.1f}°")

        p_local, yaw_local = aligner.process_visual_measurement(
            colmap_positions[i], colmap_yaws[i],
            ipdr_positions[i], ipdr_yaws[i]
        )

        if p_local is not None:
            print(f"Transformed to local: pos={p_local}, yaw={np.rad2deg(yaw_local):.1f}°")
            print(f"Error: pos={np.linalg.norm(p_local - ipdr_positions[i]):.4f}m, "
                  f"yaw={np.rad2deg(abs(yaw_local - ipdr_yaws[i])):.4f}°")

    # Print transform summary
    print("\n" + "=" * 70)
    print("Transform Summary:")
    print("=" * 70)
    summary = aligner.get_transform_summary()
    for key, value in summary.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    example_usage()
