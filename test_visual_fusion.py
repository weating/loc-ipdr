"""
iPDR Visual Fusion Test Suite
==============================

Comprehensive tests for the visual fusion module including:
- Unit tests for individual components
- Integration tests for full system
- Performance tests for real-time operation
- Robustness tests for edge cases

Author: Claude
Date: 2025-10-29
"""

import numpy as np
import time
from typing import List, Dict
import matplotlib.pyplot as plt
from ipdr_visual_fusion import (
    iPDRVisualFusion,
    VisualFusionConfig,
    EKFState,
    VisualMeasurement,
    wrap_angle,
    clamp_innovation,
    rotate_vector_to_world
)


class TestResults:
    """Container for test results"""

    def __init__(self):
        self.tests_passed = 0
        self.tests_failed = 0
        self.test_details = []

    def record_test(self, name: str, passed: bool, details: str = ""):
        """Record test result"""
        if passed:
            self.tests_passed += 1
            status = "✓ PASS"
        else:
            self.tests_failed += 1
            status = "✗ FAIL"

        self.test_details.append({
            'name': name,
            'passed': passed,
            'details': details,
            'status': status
        })

    def print_summary(self):
        """Print test summary"""
        print("\n" + "=" * 70)
        print("Test Summary")
        print("=" * 70)

        for test in self.test_details:
            print(f"{test['status']}: {test['name']}")
            if test['details']:
                print(f"      {test['details']}")

        print("\n" + "-" * 70)
        total = self.tests_passed + self.tests_failed
        success_rate = (self.tests_passed / total * 100) if total > 0 else 0
        print(f"Total: {total} tests, {self.tests_passed} passed, "
              f"{self.tests_failed} failed ({success_rate:.1f}% success)")
        print("=" * 70)


# ============================================================================
# Unit Tests
# ============================================================================

def test_wrap_angle(results: TestResults):
    """Test angle wrapping function"""
    # Test cases - wraps to (-π, π] range
    test_cases = [
        (0.0, 0.0),
        (np.pi, np.pi),
        (-np.pi, np.pi),  # -π wraps to π in (-π, π] convention
        (2*np.pi, 0.0),
        (3*np.pi, np.pi),
        (-2*np.pi, 0.0),
        (np.pi + 0.1, -np.pi + 0.1),  # Just past π wraps to -π side
        (1.5*np.pi, -0.5*np.pi),
    ]

    all_passed = True
    failed_case = None
    for input_angle, expected in test_cases:
        result = wrap_angle(np.array([input_angle]))[0]
        if not np.isclose(result, expected, atol=1e-9):
            all_passed = False
            failed_case = f"Input: {input_angle:.4f}, Expected: {expected:.4f}, Got: {result:.4f}"
            break

    results.record_test(
        "Angle Wrapping",
        all_passed,
        f"Wraps angles to (-π, π] range" + (f" - {failed_case}" if failed_case else "")
    )


def test_clamp_innovation(results: TestResults):
    """Test innovation clamping"""
    # Test case 1: No clamping needed
    innovation1 = np.array([1.0, 2.0, 1.5])
    max_value1 = 3.0
    result1 = clamp_innovation(innovation1, max_value1)
    test1 = np.allclose(result1, innovation1)

    # Test case 2: Clamping needed
    innovation2 = np.array([3.0, 4.0, 0.0])
    max_value2 = 2.0
    result2 = clamp_innovation(innovation2, max_value2)
    test2 = np.isclose(np.linalg.norm(result2), max_value2)

    results.record_test(
        "Innovation Clamping",
        test1 and test2,
        "Clamps large innovations while preserving direction"
    )


def test_rotation_matrix(results: TestResults):
    """Test body-to-world rotation"""
    # Test identity rotation
    vec = np.array([1.0, 0.0, 0.0])
    rotated = rotate_vector_to_world(vec, 0, 0, 0)
    test1 = np.allclose(rotated, vec)

    # Test 90 degree yaw rotation
    vec2 = np.array([1.0, 0.0, 0.0])
    rotated2 = rotate_vector_to_world(vec2, np.pi/2, 0, 0)
    expected2 = np.array([0.0, 1.0, 0.0])
    test2 = np.allclose(rotated2, expected2, atol=1e-10)

    results.record_test(
        "Body-to-World Rotation",
        test1 and test2,
        "Correctly rotates vectors from body to world frame"
    )


def test_visual_measurement(results: TestResults):
    """Test visual measurement data structure"""
    try:
        # Valid measurement
        vis_meas = VisualMeasurement(
            t_v=1.0,
            p_W=np.array([1.0, 2.0, 3.0]),
            yaw_W=0.5,
            e_repr=0.3,
            n_inlier=50
        )
        test1 = True
    except Exception:
        test1 = False

    try:
        # Invalid measurement (wrong shape)
        vis_meas_bad = VisualMeasurement(
            t_v=1.0,
            p_W=np.array([1.0, 2.0]),  # Wrong shape
            yaw_W=0.5,
            e_repr=0.3,
            n_inlier=50
        )
        test2 = False  # Should have raised exception
    except ValueError:
        test2 = True  # Correctly raised exception

    results.record_test(
        "Visual Measurement Validation",
        test1 and test2,
        "Validates measurement structure and dimensions"
    )


def test_ekf_state(results: TestResults):
    """Test EKF state data structure"""
    try:
        # Valid state
        x = np.zeros(10)
        P = np.eye(10)
        state = EKFState(t=0.0, x=x, P=P)

        # Test properties
        test1 = state.position.shape == (3,)
        test2 = state.orientation.shape == (3,)
        test3 = state.velocity.shape == (3,)
        test4 = isinstance(state.bias_yaw, float)

        # Test copy
        state_copy = state.copy()
        test5 = not np.shares_memory(state.x, state_copy.x)

        all_passed = test1 and test2 and test3 and test4 and test5
    except Exception:
        all_passed = False

    results.record_test(
        "EKF State Structure",
        all_passed,
        "State vector and covariance properly structured"
    )


# ============================================================================
# Integration Tests
# ============================================================================

def test_initialization(results: TestResults):
    """Test system initialization"""
    try:
        config = VisualFusionConfig()
        fusion = iPDRVisualFusion(config)

        # Initialize state
        initial_x = np.zeros(10)
        initial_P = np.eye(10)
        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)

        fusion.initialize_state(initial_state)

        # Check initialization
        test1 = fusion.current_state is not None
        test2 = len(fusion.state_buffer) == 1
        test3 = fusion.stats['total_visual_meas'] == 0

        passed = test1 and test2 and test3
    except Exception as e:
        passed = False
        details = f"Exception: {e}"

    results.record_test(
        "System Initialization",
        passed,
        "Correctly initializes state and buffers"
    )


def test_prediction_step(results: TestResults):
    """Test EKF prediction step"""
    try:
        fusion = iPDRVisualFusion()

        # Initialize
        initial_x = np.zeros(10)
        initial_P = np.eye(10) * 0.1
        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
        fusion.initialize_state(initial_state)

        # Run prediction
        acc = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.1])
        dt = 0.01

        fusion.predict_step(dt, acc, gyro)

        # Check state updated
        test1 = fusion.current_state.t == dt
        test2 = len(fusion.state_buffer) == 2
        test3 = fusion.current_state.acc is not None

        # Check yaw changed due to gyro
        test4 = abs(fusion.current_state.x[3] - 0.001) < 1e-6

        passed = test1 and test2 and test3 and test4
    except Exception as e:
        passed = False

    results.record_test(
        "EKF Prediction Step",
        passed,
        "Prediction updates state and covariance"
    )


def test_visual_queue(results: TestResults):
    """Test visual measurement queue"""
    try:
        fusion = iPDRVisualFusion()

        # Add measurements
        for i in range(5):
            vis_meas = VisualMeasurement(
                t_v=float(i),
                p_W=np.array([0.0, 0.0, 0.0]),
                yaw_W=0.0,
                e_repr=0.5,
                n_inlier=50
            )
            fusion.add_visual_measurement(vis_meas)

        test1 = len(fusion.visual_queue) == 5
        test2 = fusion.stats['total_visual_meas'] == 5

        passed = test1 and test2
    except Exception:
        passed = False

    results.record_test(
        "Visual Measurement Queue",
        passed,
        "Correctly queues visual measurements"
    )


def test_adaptive_covariance(results: TestResults):
    """Test adaptive covariance computation"""
    try:
        fusion = iPDRVisualFusion()

        # Good quality measurement
        vis_meas_good = VisualMeasurement(
            t_v=0.0,
            p_W=np.zeros(3),
            yaw_W=0.0,
            e_repr=0.1,  # Low error
            n_inlier=100  # Many inliers
        )
        R_pos_good, R_yaw_good = fusion._adaptive_covariance(vis_meas_good)

        # Poor quality measurement
        vis_meas_poor = VisualMeasurement(
            t_v=0.0,
            p_W=np.zeros(3),
            yaw_W=0.0,
            e_repr=2.0,  # High error
            n_inlier=10  # Few inliers
        )
        R_pos_poor, R_yaw_poor = fusion._adaptive_covariance(vis_meas_poor)

        # Poor quality should have higher noise
        test1 = np.trace(R_pos_poor) > np.trace(R_pos_good)
        test2 = R_yaw_poor[0, 0] > R_yaw_good[0, 0]

        passed = test1 and test2
    except Exception:
        passed = False

    results.record_test(
        "Adaptive Covariance",
        passed,
        "Adjusts noise based on visual quality"
    )


def test_quality_check(results: TestResults):
    """Test visual quality checking"""
    try:
        config = VisualFusionConfig()
        config.min_inliers = 20
        config.max_reproj_error = 1.0

        fusion = iPDRVisualFusion(config)

        # Good quality
        vis_good = VisualMeasurement(
            t_v=0.0, p_W=np.zeros(3), yaw_W=0.0,
            e_repr=0.5, n_inlier=50
        )
        test1 = fusion._check_visual_quality(vis_good)

        # Too few inliers
        vis_bad1 = VisualMeasurement(
            t_v=0.0, p_W=np.zeros(3), yaw_W=0.0,
            e_repr=0.5, n_inlier=10
        )
        test2 = not fusion._check_visual_quality(vis_bad1)

        # Too high error
        vis_bad2 = VisualMeasurement(
            t_v=0.0, p_W=np.zeros(3), yaw_W=0.0,
            e_repr=2.0, n_inlier=50
        )
        test3 = not fusion._check_visual_quality(vis_bad2)

        passed = test1 and test2 and test3
    except Exception:
        passed = False

    results.record_test(
        "Visual Quality Check",
        passed,
        "Rejects poor quality measurements"
    )


def test_state_buffer_trimming(results: TestResults):
    """Test state buffer automatic trimming"""
    try:
        config = VisualFusionConfig()
        config.buffer_duration = 1.0  # 1 second buffer

        fusion = iPDRVisualFusion(config)

        # Initialize
        initial_x = np.zeros(10)
        initial_P = np.eye(10)
        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
        fusion.initialize_state(initial_state)

        # Add many predictions
        acc = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.0])

        for i in range(200):  # 2 seconds at 100 Hz
            fusion.predict_step(0.01, acc, gyro)

        # Buffer should only contain ~1 second of data
        time_span = fusion.current_state.t - fusion.state_buffer[0].t
        test1 = time_span <= config.buffer_duration * 1.1  # Allow 10% margin

        passed = test1
    except Exception as e:
        passed = False

    results.record_test(
        "State Buffer Trimming",
        passed,
        "Automatically removes old states"
    )


# ============================================================================
# End-to-End Tests
# ============================================================================

def test_visual_update_e2e(results: TestResults):
    """End-to-end test of visual update"""
    try:
        fusion = iPDRVisualFusion()

        # Initialize at origin
        initial_x = np.zeros(10)
        initial_P = np.eye(10)
        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
        fusion.initialize_state(initial_state)

        # Predict forward
        acc = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.0])

        for i in range(50):  # 0.5 seconds
            fusion.predict_step(0.01, acc, gyro)

        pos_before, _ = fusion.get_current_pose()

        # Add visual measurement with offset
        vis_meas = VisualMeasurement(
            t_v=0.25,  # Middle of trajectory
            p_W=np.array([1.0, 1.0, 0.0]),  # Offset from origin
            yaw_W=0.0,
            e_repr=0.3,
            n_inlier=50
        )

        fusion.add_visual_measurement(vis_meas)
        num_updates = fusion.process_visual_measurements()

        pos_after, _ = fusion.get_current_pose()

        # Position should have changed
        test1 = num_updates == 1
        test2 = not np.allclose(pos_before, pos_after)
        test3 = fusion.stats['visual_updates'] == 1
        test4 = fusion.stats['oosm_replays'] == 1

        passed = test1 and test2 and test3 and test4
    except Exception as e:
        passed = False
        details = f"Exception: {e}"

    results.record_test(
        "Visual Update End-to-End",
        passed,
        "Full visual update with OOSM replay"
    )


def test_outlier_rejection(results: TestResults):
    """Test Mahalanobis distance outlier rejection"""
    try:
        config = VisualFusionConfig()
        config.mahal_threshold_pos = 10.0  # Tight threshold

        fusion = iPDRVisualFusion(config)

        # Initialize
        initial_x = np.zeros(10)
        initial_P = np.eye(10) * 0.01  # Low uncertainty
        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
        fusion.initialize_state(initial_state)

        # Predict a bit
        for i in range(10):
            fusion.predict_step(0.01, np.array([0, 0, 9.81]), np.zeros(3))

        # Add outlier measurement (very far from current position)
        vis_outlier = VisualMeasurement(
            t_v=0.05,
            p_W=np.array([100.0, 100.0, 0.0]),  # Far away
            yaw_W=0.0,
            e_repr=0.1,
            n_inlier=50
        )

        fusion.add_visual_measurement(vis_outlier)
        num_updates = fusion.process_visual_measurements()

        # Should be rejected
        test1 = num_updates == 0
        test2 = fusion.stats['rejected_mahalanobis'] >= 1

        passed = test1 and test2
    except Exception:
        passed = False

    results.record_test(
        "Outlier Rejection",
        passed,
        "Rejects outliers via Mahalanobis gating"
    )


# ============================================================================
# Performance Tests
# ============================================================================

def test_performance(results: TestResults):
    """Test real-time performance"""
    try:
        fusion = iPDRVisualFusion()

        # Initialize
        initial_x = np.zeros(10)
        initial_P = np.eye(10)
        initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
        fusion.initialize_state(initial_state)

        # Time prediction steps
        acc = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.1])

        start_time = time.time()
        num_predictions = 1000

        for i in range(num_predictions):
            fusion.predict_step(0.01, acc, gyro)

        pred_time = time.time() - start_time
        avg_pred_time = pred_time / num_predictions

        # Time visual updates
        vis_meas = VisualMeasurement(
            t_v=0.5, p_W=np.array([1, 1, 0]),
            yaw_W=0.1, e_repr=0.5, n_inlier=50
        )

        start_time = time.time()
        num_visual = 10

        for i in range(num_visual):
            vis_meas.t_v = 0.5 + i * 0.1
            fusion.add_visual_measurement(vis_meas)
            fusion.process_visual_measurements()

        visual_time = time.time() - start_time
        avg_visual_time = visual_time / num_visual

        # Check performance
        # Prediction should be < 1ms (100 Hz requirement)
        test1 = avg_pred_time < 0.001

        # Visual update should be < 100ms (visual runs at 0.1-1 Hz, so 100ms is acceptable)
        # Even with OOSM replay, this leaves plenty of headroom
        test2 = avg_visual_time < 0.1

        details = f"Pred: {avg_pred_time*1000:.3f}ms, Visual: {avg_visual_time*1000:.3f}ms"
        passed = test1 and test2

    except Exception:
        passed = False
        details = "Performance test failed"

    results.record_test(
        "Real-time Performance",
        passed,
        details
    )


# ============================================================================
# Run All Tests
# ============================================================================

def run_all_tests():
    """Run complete test suite"""
    print("=" * 70)
    print("iPDR Visual Fusion Test Suite")
    print("=" * 70)

    results = TestResults()

    # Unit tests
    print("\n▶ Running Unit Tests...")
    test_wrap_angle(results)
    test_clamp_innovation(results)
    test_rotation_matrix(results)
    test_visual_measurement(results)
    test_ekf_state(results)

    # Integration tests
    print("\n▶ Running Integration Tests...")
    test_initialization(results)
    test_prediction_step(results)
    test_visual_queue(results)
    test_adaptive_covariance(results)
    test_quality_check(results)
    test_state_buffer_trimming(results)

    # End-to-end tests
    print("\n▶ Running End-to-End Tests...")
    test_visual_update_e2e(results)
    test_outlier_rejection(results)

    # Performance tests
    print("\n▶ Running Performance Tests...")
    test_performance(results)

    # Print summary
    results.print_summary()

    return results


# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    results = run_all_tests()

    # Exit with error code if any tests failed
    if results.tests_failed > 0:
        exit(1)
    else:
        exit(0)
