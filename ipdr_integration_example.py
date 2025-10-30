"""
iPDR Visual Fusion Integration Example
======================================

This script demonstrates how to integrate the visual fusion module
with the existing iPDR EKF system for real-time operation.

The integration follows this pattern:
1. Initialize iPDR EKF and visual fusion
2. Main loop:
   - Process IMU data (EKF prediction)
   - Process step detection (ZLDU optimization)
   - Process visual measurements (visual fusion)
   - Output fused pose

Author: Claude
Date: 2025-10-29
"""

import numpy as np
import pandas as pd
from pathlib import Path
from typing import Optional, Dict, List
import matplotlib.pyplot as plt
from ipdr_visual_fusion import (
    iPDRVisualFusion,
    VisualFusionConfig,
    EKFState,
    VisualMeasurement
)


class iPDRIntegratedSystem:
    """
    Integrated iPDR system with visual fusion

    This class wraps the visual fusion module and provides
    interfaces compatible with the existing iOS iPDR implementation.
    """

    def __init__(self, config: Optional[VisualFusionConfig] = None):
        """
        Initialize integrated system

        Args:
            config: Visual fusion configuration
        """
        self.visual_fusion = iPDRVisualFusion(config)

        # iPDR-specific state
        self.step_count = 0
        self.total_distance = 0.0
        self.last_step_time = 0.0

        # Output trajectory
        self.trajectory = []

        # Logging
        self.imu_log = []
        self.step_log = []
        self.visual_log = []

    def initialize(self, initial_position: np.ndarray = np.zeros(3),
                   initial_orientation: np.ndarray = np.zeros(3)):
        """
        Initialize system with starting pose

        Args:
            initial_position: [x, y, z] in meters
            initial_orientation: [yaw, pitch, roll] in radians
        """
        # Initialize state vector
        initial_x = np.zeros(10)
        initial_x[0:3] = initial_position
        initial_x[3:6] = initial_orientation

        # Initial covariance (high uncertainty)
        initial_P = np.eye(10)
        initial_P[0:3, 0:3] *= 10.0  # Position uncertainty: 10m
        initial_P[3:6, 3:6] *= (np.pi/4)**2  # Orientation: 45 degrees
        initial_P[6:9, 6:9] *= 1.0  # Velocity: 1 m/s
        initial_P[9, 9] *= (np.pi/180)**2  # Bias: 1 degree

        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
        self.visual_fusion.initialize_state(initial_state)

        # Log initial position
        self._log_trajectory()

    def update_imu(self, timestamp: float, acc: np.ndarray, gyro: np.ndarray):
        """
        Update with IMU measurement (called at high frequency, e.g., 100 Hz)

        Args:
            timestamp: Time in seconds
            acc: Acceleration [ax, ay, az] in m/s^2
            gyro: Angular velocity [wx, wy, wz] in rad/s
        """
        if self.visual_fusion.current_state is None:
            raise RuntimeError("System not initialized. Call initialize() first.")

        # Compute dt
        dt = timestamp - self.visual_fusion.current_state.t
        if dt <= 0:
            return  # Skip if timestamp is not increasing

        # EKF prediction step
        self.visual_fusion.predict_step(dt, acc, gyro, step_len=None)

        # Log IMU data
        self.imu_log.append({
            't': timestamp,
            'acc': acc.copy(),
            'gyro': gyro.copy()
        })

        # Log trajectory (can be downsampled)
        if len(self.trajectory) == 0 or timestamp - self.trajectory[-1]['t'] > 0.1:
            self._log_trajectory()

    def update_step(self, timestamp: float, step_length: float):
        """
        Update when step is detected (low frequency, e.g., 1-2 Hz)

        This is called by the step detection module when a step is detected.
        In the iOS implementation, this corresponds to `didTakeStepWithLength:`.

        Args:
            timestamp: Time in seconds
            step_length: Detected step length in meters
        """
        if self.visual_fusion.current_state is None:
            return

        self.step_count += 1
        self.total_distance += step_length
        self.last_step_time = timestamp

        # Update with step length
        # Note: The visual fusion module will use step length in prediction
        # Here we log it and can trigger ZLDU optimization

        self.step_log.append({
            't': timestamp,
            'step_length': step_length,
            'step_count': self.step_count,
            'total_distance': self.total_distance
        })

        # Note: ZLDU optimization can be called here if straight walk is detected
        # This is implemented in the iOS iPDR system but simplified here

    def update_visual(self, vis_meas: VisualMeasurement):
        """
        Add visual positioning measurement (very low frequency, e.g., 0.1-1 Hz)

        This is called when COLMAP returns a pose estimate.

        Args:
            vis_meas: Visual measurement from COLMAP
        """
        # Add to visual queue
        self.visual_fusion.add_visual_measurement(vis_meas)

        # Process all pending visual measurements
        num_updates = self.visual_fusion.process_visual_measurements()

        if num_updates > 0:
            # Log trajectory after visual update
            self._log_trajectory()

            self.visual_log.append({
                't': vis_meas.t_v,
                'num_updates': num_updates,
                'pos_meas': vis_meas.p_W.copy(),
                'yaw_meas': vis_meas.yaw_W
            })

    def _log_trajectory(self):
        """Log current pose to trajectory"""
        if self.visual_fusion.current_state is None:
            return

        state = self.visual_fusion.current_state
        self.trajectory.append({
            't': state.t,
            'pos': state.position.copy(),
            'ori': state.orientation.copy(),
            'P_pos': np.diag(state.P[0:3, 0:3]).copy(),  # Position variance
            'P_ori': np.diag(state.P[3:6, 3:6]).copy()   # Orientation variance
        })

    def get_current_pose(self) -> Dict:
        """
        Get current pose estimate

        Returns:
            Dictionary with position, orientation, and statistics
        """
        if self.visual_fusion.current_state is None:
            raise RuntimeError("System not initialized")

        pos, ori = self.visual_fusion.get_current_pose()
        state = self.visual_fusion.current_state

        return {
            'timestamp': state.t,
            'position': pos,
            'orientation': ori,
            'position_std': np.sqrt(np.diag(state.P[0:3, 0:3])),
            'orientation_std': np.sqrt(np.diag(state.P[3:6, 3:6])),
            'step_count': self.step_count,
            'total_distance': self.total_distance
        }

    def get_statistics(self) -> Dict:
        """Get system statistics"""
        stats = self.visual_fusion.get_statistics()
        stats['step_count'] = self.step_count
        stats['total_distance'] = self.total_distance
        return stats

    def get_trajectory(self) -> pd.DataFrame:
        """
        Get trajectory as DataFrame

        Returns:
            DataFrame with columns: t, x, y, z, yaw, pitch, roll, std_x, std_y, std_z
        """
        if len(self.trajectory) == 0:
            return pd.DataFrame()

        data = {
            't': [entry['t'] for entry in self.trajectory],
            'x': [entry['pos'][0] for entry in self.trajectory],
            'y': [entry['pos'][1] for entry in self.trajectory],
            'z': [entry['pos'][2] for entry in self.trajectory],
            'yaw': [entry['ori'][0] for entry in self.trajectory],
            'pitch': [entry['ori'][1] for entry in self.trajectory],
            'roll': [entry['ori'][2] for entry in self.trajectory],
            'std_x': [np.sqrt(entry['P_pos'][0]) for entry in self.trajectory],
            'std_y': [np.sqrt(entry['P_pos'][1]) for entry in self.trajectory],
            'std_z': [np.sqrt(entry['P_pos'][2]) for entry in self.trajectory],
        }

        return pd.DataFrame(data)


# ============================================================================
# Example Main Loop
# ============================================================================

def main_loop_example():
    """
    Example of main loop integration

    This demonstrates the pattern for integrating visual fusion
    into an existing iPDR system.
    """
    print("=" * 70)
    print("iPDR Visual Fusion Integration Example")
    print("=" * 70)

    # 1. Initialize system
    config = VisualFusionConfig()
    config.max_pos_innovation = 3.0
    config.max_yaw_innovation = np.deg2rad(10)

    system = iPDRIntegratedSystem(config)
    system.initialize(
        initial_position=np.array([0.0, 0.0, 0.0]),
        initial_orientation=np.array([0.0, 0.0, 0.0])
    )

    print("\n✓ System initialized")

    # 2. Simulate sensor data
    dt_imu = 0.01  # 100 Hz IMU
    dt_step = 0.5  # ~2 Hz step detection
    dt_visual = 1.0  # 1 Hz visual positioning

    t = 0.0
    t_end = 10.0

    step_length = 0.7  # meters
    last_step_time = 0.0
    last_visual_time = 0.0

    print(f"\n⏳ Running simulation for {t_end} seconds...")
    print(f"   - IMU rate: {1/dt_imu:.0f} Hz")
    print(f"   - Step rate: {1/dt_step:.1f} Hz")
    print(f"   - Visual rate: {1/dt_visual:.1f} Hz")

    # Simulate walking forward with small rotation
    while t < t_end:
        # IMU update (high frequency)
        acc = np.array([0.1, 0.0, 9.81]) + np.random.randn(3) * 0.1
        gyro = np.array([0.0, 0.0, 0.05]) + np.random.randn(3) * 0.01

        system.update_imu(t, acc, gyro)

        # Step detection (low frequency)
        if t - last_step_time >= dt_step:
            system.update_step(t, step_length)
            last_step_time = t

        # Visual positioning (very low frequency)
        if t - last_visual_time >= dt_visual and t > 0:
            # Simulate COLMAP measurement with some noise
            pos, ori = system.visual_fusion.get_current_pose()

            # Add noise to simulate visual measurement
            pos_meas = pos + np.random.randn(3) * 0.2
            yaw_meas = ori[0] + np.random.randn() * np.deg2rad(2)

            vis_meas = VisualMeasurement(
                t_v=t - 0.1,  # Simulate 100ms delay
                p_W=pos_meas,
                yaw_W=yaw_meas,
                e_repr=0.5,
                n_inlier=50
            )

            system.update_visual(vis_meas)
            last_visual_time = t

        t += dt_imu

    # 3. Get results
    print("\n" + "=" * 70)
    print("Results")
    print("=" * 70)

    final_pose = system.get_current_pose()
    stats = system.get_statistics()

    print("\n📍 Final Pose:")
    print(f"   Position: ({final_pose['position'][0]:.2f}, "
          f"{final_pose['position'][1]:.2f}, {final_pose['position'][2]:.2f}) m")
    print(f"   Orientation: (yaw={np.rad2deg(final_pose['orientation'][0]):.1f}°, "
          f"pitch={np.rad2deg(final_pose['orientation'][1]):.1f}°, "
          f"roll={np.rad2deg(final_pose['orientation'][2]):.1f}°)")
    print(f"   Position Std: ({final_pose['position_std'][0]:.2f}, "
          f"{final_pose['position_std'][1]:.2f}, {final_pose['position_std'][2]:.2f}) m")

    print(f"\n📊 Statistics:")
    print(f"   Steps: {stats['step_count']}")
    print(f"   Distance: {stats['total_distance']:.2f} m")
    print(f"   Visual measurements: {stats['total_visual_meas']}")
    print(f"   Visual updates: {stats['visual_updates']}")
    print(f"   Acceptance rate: {stats['acceptance_rate']*100:.1f}%")
    print(f"   OOSM replays: {stats['oosm_replays']}")

    # 4. Visualize trajectory
    trajectory_df = system.get_trajectory()

    if len(trajectory_df) > 0:
        print("\n📈 Generating visualization...")
        visualize_results(system, trajectory_df)
        print("   ✓ Plots saved to 'ipdr_visual_fusion_results.png'")

    print("\n" + "=" * 70)
    print("✅ Example completed successfully!")
    print("=" * 70)

    return system


def visualize_results(system: iPDRIntegratedSystem, trajectory_df: pd.DataFrame):
    """
    Visualize results with multiple subplots

    Args:
        system: Integrated system
        trajectory_df: Trajectory DataFrame
    """
    fig = plt.figure(figsize=(14, 10))

    # 1. 2D Trajectory (top view)
    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(trajectory_df['x'], trajectory_df['y'], 'b-', linewidth=2, label='Fused Trajectory')

    # Plot visual measurements
    if len(system.visual_log) > 0:
        vis_x = [v['pos_meas'][0] for v in system.visual_log]
        vis_y = [v['pos_meas'][1] for v in system.visual_log]
        ax1.scatter(vis_x, vis_y, c='red', s=100, marker='*',
                   label='Visual Measurements', zorder=5)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('2D Trajectory (Top View)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # 2. Position vs Time
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(trajectory_df['t'], trajectory_df['x'], label='X')
    ax2.plot(trajectory_df['t'], trajectory_df['y'], label='Y')
    ax2.plot(trajectory_df['t'], trajectory_df['z'], label='Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. Orientation vs Time
    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(trajectory_df['t'], np.rad2deg(trajectory_df['yaw']), label='Yaw')
    ax3.plot(trajectory_df['t'], np.rad2deg(trajectory_df['pitch']), label='Pitch')
    ax3.plot(trajectory_df['t'], np.rad2deg(trajectory_df['roll']), label='Roll')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (deg)')
    ax3.set_title('Orientation vs Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Position Uncertainty
    ax4 = plt.subplot(2, 3, 4)
    ax4.plot(trajectory_df['t'], trajectory_df['std_x'], label='σ_x')
    ax4.plot(trajectory_df['t'], trajectory_df['std_y'], label='σ_y')
    ax4.plot(trajectory_df['t'], trajectory_df['std_z'], label='σ_z')

    # Mark visual updates
    if len(system.visual_log) > 0:
        vis_t = [v['t'] for v in system.visual_log]
        for t in vis_t:
            ax4.axvline(t, color='red', alpha=0.3, linestyle='--')

    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Position Std Dev (m)')
    ax4.set_title('Position Uncertainty')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 5. 3D Trajectory
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    ax5.plot(trajectory_df['x'], trajectory_df['y'], trajectory_df['z'], 'b-', linewidth=2)
    ax5.set_xlabel('X (m)')
    ax5.set_ylabel('Y (m)')
    ax5.set_zlabel('Z (m)')
    ax5.set_title('3D Trajectory')

    # 6. Statistics Summary
    ax6 = plt.subplot(2, 3, 6)
    ax6.axis('off')

    stats = system.get_statistics()
    stats_text = f"""
    System Statistics
    ──────────────────────────────
    Steps Taken: {stats['step_count']}
    Total Distance: {stats['total_distance']:.2f} m

    Visual Measurements: {stats['total_visual_meas']}
    Visual Updates: {stats['visual_updates']}
    Acceptance Rate: {stats['acceptance_rate']*100:.1f}%

    Rejected (Quality): {stats['rejected_quality']}
    Rejected (Timeout): {stats['rejected_timeout']}
    Rejected (Mahalanobis): {stats['rejected_mahalanobis']}

    OOSM Replays: {stats['oosm_replays']}
    """

    ax6.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
            verticalalignment='center')

    plt.tight_layout()
    plt.savefig('ipdr_visual_fusion_results.png', dpi=150, bbox_inches='tight')
    plt.close()


# ============================================================================
# Run Example
# ============================================================================

if __name__ == "__main__":
    system = main_loop_example()
