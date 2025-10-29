"""
iPDR EKF Visual Fusion Module
=============================

This module implements visual positioning (COLMAP-based) global pose optimization
for the iPDR (indoor Pedestrian Dead Reckoning) EKF system.

Features:
- Visual measurement queue with adaptive covariance
- Out-of-Sequence Measurement (OOSM) handling
- Mahalanobis distance gating for outlier rejection
- Innovation clamping for stability
- Position and heading fusion
- State history buffer for delayed measurements

Author: Claude
Date: 2025-10-29
"""

import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict
import warnings


@dataclass
class VisualMeasurement:
    """
    Visual positioning measurement from COLMAP

    Attributes:
        t_v: Timestamp of visual measurement (seconds)
        p_W: Global position [x, y, z] in world frame (meters)
        yaw_W: Heading angle (yaw) in world frame (radians)
        e_repr: Reprojection error from COLMAP
        n_inlier: Number of inlier points in COLMAP
        confidence: Computed confidence score (0-1)
    """
    t_v: float
    p_W: np.ndarray  # shape (3,)
    yaw_W: float
    e_repr: float
    n_inlier: int
    confidence: float = 1.0

    def __post_init__(self):
        """Validate and convert data"""
        self.p_W = np.asarray(self.p_W, dtype=np.float64)
        if self.p_W.shape != (3,):
            raise ValueError(f"p_W must be shape (3,), got {self.p_W.shape}")
        self.yaw_W = float(self.yaw_W)
        self.t_v = float(self.t_v)


@dataclass
class EKFState:
    """
    Extended Kalman Filter state for iPDR with visual fusion

    State vector: [px, py, pz, yaw, pitch, roll, vx, vy, vz, bias_yaw]
    - Position: px, py, pz (meters)
    - Orientation: yaw, pitch, roll (radians)
    - Velocity: vx, vy, vz (m/s)
    - Bias: bias_yaw (radians)
    """
    t: float
    x: np.ndarray  # State vector, shape (10,)
    P: np.ndarray  # Covariance matrix, shape (10, 10)

    # IMU measurements at this state (for OOSM replay)
    acc: Optional[np.ndarray] = None  # shape (3,)
    gyro: Optional[np.ndarray] = None  # shape (3,)
    step_len: Optional[float] = None

    def __post_init__(self):
        """Validate state dimensions"""
        self.x = np.asarray(self.x, dtype=np.float64)
        self.P = np.asarray(self.P, dtype=np.float64)
        if self.x.shape != (10,):
            raise ValueError(f"State vector must be shape (10,), got {self.x.shape}")
        if self.P.shape != (10, 10):
            raise ValueError(f"Covariance must be shape (10, 10), got {self.P.shape}")

    @property
    def position(self) -> np.ndarray:
        """Get position [x, y, z]"""
        return self.x[0:3]

    @property
    def orientation(self) -> np.ndarray:
        """Get orientation [yaw, pitch, roll]"""
        return self.x[3:6]

    @property
    def velocity(self) -> np.ndarray:
        """Get velocity [vx, vy, vz]"""
        return self.x[6:9]

    @property
    def bias_yaw(self) -> float:
        """Get yaw bias"""
        return self.x[9]

    def copy(self):
        """Deep copy of state"""
        return EKFState(
            t=self.t,
            x=self.x.copy(),
            P=self.P.copy(),
            acc=self.acc.copy() if self.acc is not None else None,
            gyro=self.gyro.copy() if self.gyro is not None else None,
            step_len=self.step_len
        )


class VisualFusionConfig:
    """Configuration parameters for visual fusion"""

    def __init__(self):
        # Adaptive covariance weights
        self.k_pos = 0.1  # Position noise scaling factor
        self.k_yaw = 0.05  # Yaw noise scaling factor

        # Mahalanobis distance thresholds (chi-squared distribution)
        self.mahal_threshold_pos = 16.27  # 3 DOF, 99.9% confidence
        self.mahal_threshold_yaw = 10.83  # 1 DOF, 99.9% confidence

        # Innovation clamping limits
        self.max_pos_innovation = 5.0  # meters
        self.max_yaw_innovation = np.deg2rad(15)  # radians (15 degrees)

        # OOSM parameters
        self.buffer_duration = 10.0  # seconds
        self.max_buffer_size = 5000  # maximum number of states to keep
        self.max_delay = 10.0  # maximum acceptable delay (seconds)

        # Visual measurement queue
        self.visual_queue_maxlen = 100

        # Failure handling
        self.min_inliers = 10  # minimum inliers for valid measurement
        self.max_reproj_error = 5.0  # maximum reprojection error

        # Pitch/roll consistency check
        self.pitch_roll_threshold = np.deg2rad(10)  # radians


class iPDRVisualFusion:
    """
    Main class for iPDR EKF with visual positioning fusion
    """

    def __init__(self, config: Optional[VisualFusionConfig] = None):
        """
        Initialize visual fusion module

        Args:
            config: Configuration parameters, uses defaults if None
        """
        self.config = config if config is not None else VisualFusionConfig()

        # Visual measurement queue
        self.visual_queue = deque(maxlen=self.config.visual_queue_maxlen)

        # State history buffer for OOSM
        self.state_buffer = deque(maxlen=self.config.max_buffer_size)

        # Current EKF state
        self.current_state: Optional[EKFState] = None

        # Statistics
        self.stats = {
            'total_visual_meas': 0,
            'accepted_visual_meas': 0,
            'rejected_timeout': 0,
            'rejected_mahalanobis': 0,
            'rejected_quality': 0,
            'visual_updates': 0,
            'oosm_replays': 0
        }

        # Logging
        self.visual_update_log = []

    def initialize_state(self, initial_state: EKFState):
        """
        Initialize EKF state

        Args:
            initial_state: Initial state with position, orientation, etc.
        """
        self.current_state = initial_state.copy()
        self.state_buffer.clear()
        self.state_buffer.append(self.current_state.copy())

    def add_visual_measurement(self, vis_meas: VisualMeasurement):
        """
        Add visual measurement to processing queue

        Args:
            vis_meas: Visual measurement from COLMAP
        """
        self.visual_queue.append(vis_meas)
        self.stats['total_visual_meas'] += 1

    def predict_step(self, dt: float, acc: np.ndarray, gyro: np.ndarray,
                     step_len: Optional[float] = None):
        """
        EKF prediction step using IMU measurements

        Args:
            dt: Time step (seconds)
            acc: Acceleration measurement [ax, ay, az] (m/s^2)
            gyro: Gyroscope measurement [wx, wy, wz] (rad/s)
            step_len: Step length if step detected (meters)
        """
        if self.current_state is None:
            raise RuntimeError("State not initialized. Call initialize_state() first.")

        # Extract current state
        x = self.current_state.x.copy()
        P = self.current_state.P.copy()

        # State: [px, py, pz, yaw, pitch, roll, vx, vy, vz, bias_yaw]
        yaw, pitch, roll = x[3], x[4], x[5]
        vx, vy, vz = x[6], x[7], x[8]

        # Update orientation from gyroscope
        x[3] += gyro[2] * dt  # yaw
        x[4] += gyro[1] * dt  # pitch
        x[5] += gyro[0] * dt  # roll

        # Wrap angles to [-pi, pi]
        x[3] = wrap_angle(x[3])
        x[4] = wrap_angle(x[4])
        x[5] = wrap_angle(x[5])

        # Update velocity from acceleration (in body frame -> world frame)
        acc_world = rotate_vector_to_world(acc, yaw, pitch, roll)
        x[6] += acc_world[0] * dt
        x[7] += acc_world[1] * dt
        x[8] += (acc_world[2] - 9.81) * dt  # Remove gravity

        # Update position from velocity
        x[0] += vx * dt
        x[1] += vy * dt
        x[2] += vz * dt

        # If step detected, use step length for position update
        if step_len is not None:
            x[0] += step_len * np.cos(yaw)
            x[1] += step_len * np.sin(yaw)

        # State transition matrix F (simplified, identity + dt terms)
        F = np.eye(10)
        F[0, 6] = dt  # px += vx * dt
        F[1, 7] = dt  # py += vy * dt
        F[2, 8] = dt  # pz += vz * dt

        # Process noise Q
        Q = np.eye(10) * 0.01  # Base process noise
        Q[0:3, 0:3] *= 0.1  # Position noise
        Q[3:6, 3:6] *= 0.001  # Orientation noise
        Q[6:9, 6:9] *= 0.5  # Velocity noise
        Q[9, 9] *= 0.0001  # Bias noise

        # Covariance prediction
        P = F @ P @ F.T + Q

        # Update state
        self.current_state.t += dt
        self.current_state.x = x
        self.current_state.P = P
        self.current_state.acc = acc.copy()
        self.current_state.gyro = gyro.copy()
        self.current_state.step_len = step_len

        # Add to history buffer
        self.state_buffer.append(self.current_state.copy())

        # Remove old states beyond buffer duration
        self._trim_state_buffer()

    def _trim_state_buffer(self):
        """Remove states older than buffer_duration from history"""
        if len(self.state_buffer) == 0:
            return

        current_time = self.current_state.t
        while (len(self.state_buffer) > 1 and
               current_time - self.state_buffer[0].t > self.config.buffer_duration):
            self.state_buffer.popleft()

    def process_visual_measurements(self) -> int:
        """
        Process all pending visual measurements in queue

        Returns:
            Number of successful visual updates
        """
        updates = 0

        while len(self.visual_queue) > 0:
            vis_meas = self.visual_queue.popleft()

            if self._process_single_visual_measurement(vis_meas):
                updates += 1

        return updates

    def _process_single_visual_measurement(self, vis_meas: VisualMeasurement) -> bool:
        """
        Process a single visual measurement

        Args:
            vis_meas: Visual measurement to process

        Returns:
            True if successfully updated, False if rejected
        """
        # Quality check
        if not self._check_visual_quality(vis_meas):
            self.stats['rejected_quality'] += 1
            return False

        # Find corresponding historical state
        hist_state = self._find_closest_state(vis_meas.t_v)

        if hist_state is None:
            self.stats['rejected_timeout'] += 1
            return False

        # Compute adaptive covariance
        R_pos, R_yaw = self._adaptive_covariance(vis_meas)

        # Determine update strategy (position only, yaw only, or both)
        yaw_only = self._check_pitch_roll_consistency(hist_state, vis_meas)

        # Perform visual update on historical state
        updated_state = hist_state.copy()

        if yaw_only:
            # Only update yaw
            if not self._visual_yaw_update(updated_state, vis_meas, R_yaw):
                self.stats['rejected_mahalanobis'] += 1
                return False
        else:
            # Update position and yaw
            if not self._visual_position_update(updated_state, vis_meas, R_pos):
                self.stats['rejected_mahalanobis'] += 1
                return False

            if not self._visual_yaw_update(updated_state, vis_meas, R_yaw):
                self.stats['rejected_mahalanobis'] += 1
                return False

        # OOSM replay: propagate updated state to current time
        self._oosm_replay(vis_meas.t_v, updated_state)

        # Statistics and logging
        self.stats['accepted_visual_meas'] += 1
        self.stats['visual_updates'] += 1
        self._log_visual_update(vis_meas, hist_state, updated_state)

        return True

    def _check_visual_quality(self, vis_meas: VisualMeasurement) -> bool:
        """
        Check if visual measurement meets quality criteria

        Args:
            vis_meas: Visual measurement

        Returns:
            True if quality is acceptable
        """
        if vis_meas.n_inlier < self.config.min_inliers:
            return False

        if vis_meas.e_repr > self.config.max_reproj_error:
            return False

        return True

    def _find_closest_state(self, t_v: float) -> Optional[EKFState]:
        """
        Find state in buffer closest to visual measurement time

        Args:
            t_v: Visual measurement timestamp

        Returns:
            Closest state or None if not found or too old
        """
        if len(self.state_buffer) == 0:
            return None

        # Check if within time window
        current_time = self.current_state.t
        if current_time - t_v > self.config.max_delay:
            return None

        # Binary search for closest time
        min_dt = float('inf')
        closest_state = None

        for state in self.state_buffer:
            dt = abs(state.t - t_v)
            if dt < min_dt:
                min_dt = dt
                closest_state = state

        return closest_state.copy() if closest_state is not None else None

    def _adaptive_covariance(self, vis_meas: VisualMeasurement) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute adaptive measurement covariance from visual quality metrics

        Formula: σ = k * e_repr / sqrt(n_inlier)

        Args:
            vis_meas: Visual measurement with quality metrics

        Returns:
            R_pos: Position measurement covariance (3x3)
            R_yaw: Yaw measurement covariance (1x1)
        """
        # Compute noise standard deviations
        sigma_pos = self.config.k_pos * vis_meas.e_repr / np.sqrt(max(vis_meas.n_inlier, 1))
        sigma_yaw = self.config.k_yaw * vis_meas.e_repr / np.sqrt(max(vis_meas.n_inlier, 1))

        # Minimum noise floor
        sigma_pos = max(sigma_pos, 0.01)  # 1 cm minimum
        sigma_yaw = max(sigma_yaw, np.deg2rad(0.5))  # 0.5 degree minimum

        # Build covariance matrices
        R_pos = np.diag([sigma_pos**2] * 3)
        R_yaw = np.array([[sigma_yaw**2]])

        return R_pos, R_yaw

    def _check_pitch_roll_consistency(self, state: EKFState,
                                      vis_meas: VisualMeasurement) -> bool:
        """
        Check if pitch/roll from visual is consistent with EKF
        If inconsistent, only update yaw

        Args:
            state: Current EKF state
            vis_meas: Visual measurement

        Returns:
            True if should only update yaw (inconsistent pitch/roll)
        """
        # For now, assume visual doesn't provide pitch/roll
        # In real implementation, compare if available
        return False  # Default to full update

    def _visual_position_update(self, state: EKFState, vis_meas: VisualMeasurement,
                                R_pos: np.ndarray) -> bool:
        """
        EKF update for position measurement

        Args:
            state: State to update (modified in-place)
            vis_meas: Visual position measurement
            R_pos: Measurement noise covariance

        Returns:
            True if update successful (passed Mahalanobis test)
        """
        # Measurement model: z = H * x + v
        # We observe position directly
        H = np.zeros((3, 10))
        H[0, 0] = 1.0  # px
        H[1, 1] = 1.0  # py
        H[2, 2] = 1.0  # pz

        # Innovation
        z = vis_meas.p_W
        h = state.position
        y = z - h

        # Clamp innovation
        y = clamp_innovation(y, self.config.max_pos_innovation)

        # Innovation covariance
        S = H @ state.P @ H.T + R_pos

        # Mahalanobis distance test
        try:
            S_inv = np.linalg.inv(S)
            mahal_dist = y.T @ S_inv @ y

            if mahal_dist > self.config.mahal_threshold_pos:
                return False  # Reject outlier
        except np.linalg.LinAlgError:
            warnings.warn("Singular innovation covariance in position update")
            return False

        # Kalman gain
        K = state.P @ H.T @ S_inv

        # State update
        state.x = state.x + K @ y

        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(10) - K @ H
        state.P = I_KH @ state.P @ I_KH.T + K @ R_pos @ K.T

        return True

    def _visual_yaw_update(self, state: EKFState, vis_meas: VisualMeasurement,
                          R_yaw: np.ndarray) -> bool:
        """
        EKF update for yaw (heading) measurement

        Args:
            state: State to update (modified in-place)
            vis_meas: Visual yaw measurement
            R_yaw: Measurement noise covariance

        Returns:
            True if update successful (passed Mahalanobis test)
        """
        # Measurement model: z = H * x + v
        # We observe yaw directly
        H = np.zeros((1, 10))
        H[0, 3] = 1.0  # yaw

        # Innovation with angle wrapping
        z = np.array([vis_meas.yaw_W])
        h = np.array([state.x[3]])
        y = wrap_angle(z - h)

        # Clamp innovation
        y = clamp_innovation(y, self.config.max_yaw_innovation)

        # Innovation covariance
        S = H @ state.P @ H.T + R_yaw

        # Mahalanobis distance test
        try:
            S_inv = np.linalg.inv(S)
            mahal_dist = y.T @ S_inv @ y

            if mahal_dist > self.config.mahal_threshold_yaw:
                return False  # Reject outlier
        except np.linalg.LinAlgError:
            warnings.warn("Singular innovation covariance in yaw update")
            return False

        # Kalman gain
        K = state.P @ H.T @ S_inv

        # State update
        state.x = state.x + K @ y.flatten()

        # Wrap yaw angle
        state.x[3] = wrap_angle(state.x[3])

        # Covariance update (Joseph form)
        I_KH = np.eye(10) - K @ H
        state.P = I_KH @ state.P @ I_KH.T + K @ R_yaw @ K.T

        # Reset yaw bias after visual update
        state.x[9] = 0.0

        return True

    def _oosm_replay(self, t_v: float, updated_state: EKFState):
        """
        Out-of-Sequence Measurement replay
        Propagate updated state from t_v to current time

        Args:
            t_v: Visual measurement time
            updated_state: State after visual update at t_v
        """
        # Find all states from t_v to current time
        replay_states = [s for s in self.state_buffer if s.t > t_v]

        if len(replay_states) == 0:
            # Visual measurement is at current time, just update
            self.current_state = updated_state
            return

        # Start from updated state
        state = updated_state.copy()

        # Replay IMU measurements
        for next_state in replay_states:
            dt = next_state.t - state.t

            if next_state.acc is not None and next_state.gyro is not None:
                # Perform prediction step
                self._replay_predict_step(state, next_state, dt)

        # Replace current state with replayed state
        self.current_state = state
        self.stats['oosm_replays'] += 1

    def _replay_predict_step(self, state: EKFState, next_state: EKFState, dt: float):
        """
        Replay a single prediction step during OOSM

        Args:
            state: Current state (modified in-place)
            next_state: Next state with IMU measurements
            dt: Time step
        """
        acc = next_state.acc
        gyro = next_state.gyro
        step_len = next_state.step_len

        # Extract current state
        x = state.x
        P = state.P

        # State: [px, py, pz, yaw, pitch, roll, vx, vy, vz, bias_yaw]
        yaw, pitch, roll = x[3], x[4], x[5]
        vx, vy, vz = x[6], x[7], x[8]

        # Update orientation
        x[3] += gyro[2] * dt  # yaw
        x[4] += gyro[1] * dt  # pitch
        x[5] += gyro[0] * dt  # roll

        # Wrap angles
        x[3] = wrap_angle(x[3])
        x[4] = wrap_angle(x[4])
        x[5] = wrap_angle(x[5])

        # Update velocity
        acc_world = rotate_vector_to_world(acc, yaw, pitch, roll)
        x[6] += acc_world[0] * dt
        x[7] += acc_world[1] * dt
        x[8] += (acc_world[2] - 9.81) * dt

        # Update position
        x[0] += vx * dt
        x[1] += vy * dt
        x[2] += vz * dt

        # Step length correction if available
        if step_len is not None:
            x[0] += step_len * np.cos(yaw)
            x[1] += step_len * np.sin(yaw)

        # State transition matrix
        F = np.eye(10)
        F[0, 6] = dt
        F[1, 7] = dt
        F[2, 8] = dt

        # Process noise
        Q = np.eye(10) * 0.01
        Q[0:3, 0:3] *= 0.1
        Q[3:6, 3:6] *= 0.001
        Q[6:9, 6:9] *= 0.5
        Q[9, 9] *= 0.0001

        # Covariance update
        P = F @ P @ F.T + Q

        # Update state
        state.t = next_state.t
        state.x = x
        state.P = P

    def _log_visual_update(self, vis_meas: VisualMeasurement,
                          before_state: EKFState, after_state: EKFState):
        """
        Log visual update details for analysis

        Args:
            vis_meas: Visual measurement
            before_state: State before update
            after_state: State after update
        """
        log_entry = {
            't_v': vis_meas.t_v,
            'pos_before': before_state.position.copy(),
            'pos_after': after_state.position.copy(),
            'pos_meas': vis_meas.p_W.copy(),
            'yaw_before': before_state.x[3],
            'yaw_after': after_state.x[3],
            'yaw_meas': vis_meas.yaw_W,
            'e_repr': vis_meas.e_repr,
            'n_inlier': vis_meas.n_inlier
        }

        self.visual_update_log.append(log_entry)

    def get_statistics(self) -> Dict:
        """Get fusion statistics"""
        stats = self.stats.copy()
        if stats['total_visual_meas'] > 0:
            stats['acceptance_rate'] = stats['accepted_visual_meas'] / stats['total_visual_meas']
        else:
            stats['acceptance_rate'] = 0.0
        return stats

    def get_current_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current position and orientation

        Returns:
            position: [x, y, z] in meters
            orientation: [yaw, pitch, roll] in radians
        """
        if self.current_state is None:
            raise RuntimeError("State not initialized")

        return self.current_state.position.copy(), self.current_state.orientation.copy()


# ============================================================================
# Utility Functions
# ============================================================================

def wrap_angle(angle: np.ndarray) -> np.ndarray:
    """
    Wrap angle to (-pi, pi] range

    Args:
        angle: Angle(s) in radians (scalar or array)

    Returns:
        Wrapped angle(s)
    """
    wrapped = (angle + np.pi) % (2 * np.pi) - np.pi
    # Handle the edge case where wrapped is exactly -π, convert to π
    wrapped = np.where(np.isclose(wrapped, -np.pi), np.pi, wrapped)
    return wrapped


def clamp_innovation(innovation: np.ndarray, max_value: float) -> np.ndarray:
    """
    Clamp innovation magnitude to prevent large jumps

    Args:
        innovation: Innovation vector
        max_value: Maximum allowed magnitude

    Returns:
        Clamped innovation
    """
    norm = np.linalg.norm(innovation)
    if norm > max_value:
        return innovation * (max_value / norm)
    return innovation


def rotate_vector_to_world(vec: np.ndarray, yaw: float, pitch: float, roll: float) -> np.ndarray:
    """
    Rotate vector from body frame to world frame using ZYX Euler angles

    Args:
        vec: Vector in body frame [x, y, z]
        yaw: Yaw angle (radians)
        pitch: Pitch angle (radians)
        roll: Roll angle (radians)

    Returns:
        Vector in world frame
    """
    # Rotation matrix from body to world (ZYX convention)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])

    return R @ vec


# ============================================================================
# Example Usage
# ============================================================================

if __name__ == "__main__":
    # Example: Initialize and use visual fusion

    # Create configuration
    config = VisualFusionConfig()
    config.max_pos_innovation = 3.0  # More conservative
    config.max_yaw_innovation = np.deg2rad(10)

    # Create fusion system
    fusion = iPDRVisualFusion(config)

    # Initialize state
    initial_x = np.zeros(10)
    initial_x[0:3] = [0, 0, 0]  # Initial position
    initial_x[3:6] = [0, 0, 0]  # Initial orientation
    initial_P = np.eye(10) * 0.1
    initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)

    fusion.initialize_state(initial_state)

    # Simulate IMU measurements
    dt = 0.01
    for i in range(100):
        acc = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.1])
        fusion.predict_step(dt, acc, gyro)

    # Add visual measurement
    vis_meas = VisualMeasurement(
        t_v=0.5,
        p_W=np.array([1.0, 0.5, 0.0]),
        yaw_W=0.05,
        e_repr=0.5,
        n_inlier=50
    )
    fusion.add_visual_measurement(vis_meas)

    # Process visual measurements
    updates = fusion.process_visual_measurements()

    print(f"Visual updates performed: {updates}")
    print(f"Statistics: {fusion.get_statistics()}")

    pos, ori = fusion.get_current_pose()
    print(f"Current position: {pos}")
    print(f"Current orientation (deg): {np.rad2deg(ori)}")
