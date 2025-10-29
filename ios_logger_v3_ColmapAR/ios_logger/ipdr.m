//
//  ipdr.m
//  ios_logger
//
//  Created by jerryzh on 2025/7/16.
//  Copyright © 2025 Mac. All rights reserved.
//

#import "ipdr.h"

// 定义常量
static const int kPostureSwitchWindowSize = 10; // 用于姿态切换检测的窗口大小
static const int kZlduWindowSize = 4;          // 用于直线检测的窗口大小

// 私有接口/类扩展，声明所有内部使用的辅助方法
@interface iPDRHeadingEstimator() {
    // 实例变量
    NSTimeInterval _lastTimestamp;
    simd_quatf _ekf_q;
    matrix_float4x4 _ekf_P;
    double _heading_kf_x;
    double _heading_kf_P;
    NSMutableArray<NSNumber *> *_postureOmegaBuffer;
    NSMutableArray<NSNumber *> *_zlduHeadingBuffer;
    NSMutableArray<NSNumber *> *_zlduStepLengthBuffer;
    
    // ZLDU Kalman Filter state
    double _zldu_step_length;
    double _zldu_heading;
    matrix_float2x2 _zldu_P;  // 2x2 covariance matrix for [step_length, heading]

}

// --- 内部方法声明 ---
- (void)resetState;
- (void)runEKFWithOmega:(simd_float3)omega accel:(simd_float3)accel magnet:(simd_float3)magnet dt:(float)dt;
- (matrix_float4x3)getJacobianForVec:(simd_float3)v;
- (double)calculateGyroHeadingChange:(simd_float3)omega pitch:(double)pitch roll:(double)roll dt:(double)dt;
- (void)addToPostureBuffer:(NSNumber *)omegaMag;
- (void)updateHeadingWithKF:(double)deltaYaw noisyEkfYaw:(double)noisyEkfYaw;
- (void)addToZLDUBuffer:(NSNumber *)heading stepLength:(double)stepLength;
- (simd_float3)quaternionToEulerAngles:(simd_quatf)q;
- (BOOL)detectStraightWalk;
- (void)performZLDUOptimization;

@end


@implementation iPDRHeadingEstimator

// 公共初始化方法
- (instancetype)init {
    self = [super init];
    if (self) {
        // 初始化可调参数 (基于论文中的实验值)
        _ekfGyroNoise = 0.001f;
        _ekfAccelNoise = 0.1f;
        _ekfMagNoise = 0.5f;
        _headingKfProcessNoise = 0.001;
        _headingKfMeasurementNoise = 0.1;
        _quasiStaticThreshold = 0.5f;    // 论文中λ=0.5
        _postureSwitchThreshold = 0.1f;  // 论文中γ=0.1
        _zlduStraightWalkThreshold = 5.0f; // 论文中χ=5

        // 初始化状态
        [self resetState];
    }
    return self;
}

// 核心更新方法
- (void)updateWithDeviceMotion:(CMDeviceMotion *)motion rawMagnetometer:(nullable CMMagnetometerData *)rawMagnetometerData {
    if (_lastTimestamp < 0) {
        _lastTimestamp = motion.timestamp;
        _heading_kf_x = motion.heading; // 使用CoreMotion的初始航向作为起点（度）
        return;
    }
    
    double dt = motion.timestamp - _lastTimestamp;
    if (dt <= 0) return; // 防止时间戳异常
    _lastTimestamp = motion.timestamp;
    
    CMRotationRate gyro = motion.rotationRate;
    CMAcceleration gravity = motion.gravity;
    CMCalibratedMagneticField mag = motion.magneticField;

    // --- 步骤 1: EKF 航向预测 (论文 3.1节) ---
    simd_float3 omega_vec = {(float)gyro.x, (float)gyro.y, (float)gyro.z};
    simd_float3 g_vec = {(float)gravity.x, (float)gravity.y, (float)gravity.z};
    simd_float3 m_vec = {(float)mag.field.x, (float)mag.field.y, (float)mag.field.z};
    
    [self runEKFWithOmega:omega_vec accel:g_vec magnet:m_vec dt:dt];
    
    simd_float3 eulerAngles = [self quaternionToEulerAngles:_ekf_q];
    double noisyYawFromEKF = eulerAngles.z;
    
    // --- 步骤 2: 精确计算航向变化量 Δyaw (论文 3.2, 3.3节) ---
    double deltaYaw = [self calculateGyroHeadingChange:omega_vec pitch:eulerAngles.y roll:eulerAngles.x dt:dt];
    
    // --- 步骤 3: KF 航向更新 (论文 3.4节) ---
    [self updateHeadingWithKF:deltaYaw noisyEkfYaw:noisyYawFromEKF];
    
    // 更新最终输出
    _attitudeQuaternion = _ekf_q; // EKF提供完整的姿态
    _pitch = eulerAngles.y;
    _roll = eulerAngles.x;
    _heading = _heading_kf_x; // 使用KF优化后的航向
}

// ZLDU 优化入口
- (void)didTakeStepWithLength:(double)stepLength {
    // 论文 3.5节: ZLDU 优化
    [self addToZLDUBuffer:@(_heading) stepLength:stepLength];
    
    if ([self detectStraightWalk]) {
        [self performZLDUOptimization];
    }
}

#pragma mark - Internal Helper Implementations

// 重置所有状态变量
- (void)resetState {
    _lastTimestamp = -1.0;

    _ekf_q = simd_quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    _ekf_P = matrix_identity_float4x4;
    _ekf_P.columns[0][0] = 1000; _ekf_P.columns[1][1] = 1000;
    _ekf_P.columns[2][2] = 1000; _ekf_P.columns[3][3] = 1000;

    _heading_kf_x = 0.0;
    _heading_kf_P = 1.0;
    
    _attitudeQuaternion = _ekf_q;
    
    _postureOmegaBuffer = [NSMutableArray arrayWithCapacity:kPostureSwitchWindowSize];
    _zlduHeadingBuffer = [NSMutableArray arrayWithCapacity:kZlduWindowSize];
    _zlduStepLengthBuffer = [NSMutableArray arrayWithCapacity:kZlduWindowSize];
    
    // 初始化ZLDU Kalman Filter状态
    _zldu_step_length = 0.7; // 默认步长
    _zldu_heading = 0.0;
    _zldu_P.columns[0] = (simd_float2){0.85f, 0.0f};
    _zldu_P.columns[1] = (simd_float2){0.0f, 0.85f};

}

// 论文核心算法：扩展卡尔曼滤波 (保持原有实现)
- (void)runEKFWithOmega:(simd_float3)omega accel:(simd_float3)accel magnet:(simd_float3)magnet dt:(float)dt {

    // 1. PREDICTION STEP
    simd_float4 q_vec = _ekf_q.vector;
    simd_float4 q_dot = {
        0.5f * (q_vec.w * omega.x + q_vec.y * omega.z - q_vec.z * omega.y),
        0.5f * (q_vec.w * omega.y - q_vec.x * omega.z + q_vec.z * omega.x),
        0.5f * (q_vec.w * omega.z + q_vec.x * omega.y - q_vec.y * omega.x),
       -0.5f * (q_vec.x * omega.x + q_vec.y * omega.y + q_vec.z * omega.z)
    };
    simd_float4 pred_q_vec = q_vec + q_dot * dt;
    _ekf_q = simd_normalize(simd_quaternion(pred_q_vec));

    matrix_float4x4 F = {
        .columns[0] = { 0.0f,  omega.z, -omega.y, omega.x},
        .columns[1] = {-omega.z,  0.0f,  omega.x, omega.y},
        .columns[2] = { omega.y, -omega.x,  0.0f, omega.z},
        .columns[3] = {-omega.x, -omega.y, -omega.z, 0.0f}
    };
    float scalar = 0.5f * dt;
    matrix_float4x4 F_scaled = {
        .columns[0] = F.columns[0] * scalar, .columns[1] = F.columns[1] * scalar,
        .columns[2] = F.columns[2] * scalar, .columns[3] = F.columns[3] * scalar
    };
    matrix_float4x4 A = matrix_identity_float4x4;
    A.columns[0] += F_scaled.columns[0]; A.columns[1] += F_scaled.columns[1];
    A.columns[2] += F_scaled.columns[2]; A.columns[3] += F_scaled.columns[3];

    matrix_float4x4 Q = {
        .columns[0] = {_ekfGyroNoise, 0.0f, 0.0f, 0.0f}, .columns[1] = {0.0f, _ekfGyroNoise, 0.0f, 0.0f},
        .columns[2] = {0.0f, 0.0f, _ekfGyroNoise, 0.0f}, .columns[3] = {0.0f, 0.0f, 0.0f, _ekfGyroNoise}
    };
    matrix_float4x4 temp_P = matrix_multiply(matrix_multiply(A, _ekf_P), matrix_transpose(A));
    temp_P.columns[0] += Q.columns[0]; temp_P.columns[1] += Q.columns[1];
    temp_P.columns[2] += Q.columns[2]; temp_P.columns[3] += Q.columns[3];
    _ekf_P = temp_P;

    // 2. UPDATE STEP (Accelerometer)
    if (simd_length_squared(accel) > 0.01) {
        simd_float3 z_a = simd_normalize(accel);
        simd_float3 h_a = simd_act(_ekf_q, (simd_float3){0.0f, 0.0f, -1.0f});
        simd_float3 y_a = z_a - h_a;

        matrix_float4x3 H_a = [self getJacobianForVec:(simd_float3){0.0f, 0.0f, -1.0f}];
        matrix_float3x3 R_a = {
            .columns[0] = {_ekfAccelNoise, 0.0f, 0.0f}, .columns[1] = {0.0f, _ekfAccelNoise, 0.0f},
            .columns[2] = {0.0f, 0.0f, _ekfAccelNoise}
        };

        matrix_float3x3 S_a = matrix_multiply(H_a, matrix_multiply(_ekf_P, matrix_transpose(H_a)));
        S_a.columns[0] += R_a.columns[0]; S_a.columns[1] += R_a.columns[1];
        S_a.columns[2] += R_a.columns[2];
        
        matrix_float3x4 K_a = matrix_multiply(matrix_multiply(_ekf_P, matrix_transpose(H_a)), simd_inverse(S_a));

        _ekf_q.vector += matrix_multiply(K_a, y_a);
        _ekf_q = simd_normalize(_ekf_q);
        
        matrix_float4x4 K_H_a = matrix_multiply(K_a, H_a);
        matrix_float4x4 I_minus_KH_a = matrix_identity_float4x4;
        I_minus_KH_a.columns[0] -= K_H_a.columns[0]; I_minus_KH_a.columns[1] -= K_H_a.columns[1];
        I_minus_KH_a.columns[2] -= K_H_a.columns[2]; I_minus_KH_a.columns[3] -= K_H_a.columns[3];
        _ekf_P = matrix_multiply(I_minus_KH_a, _ekf_P);
    }

    // 3. UPDATE STEP (Magnetometer)
    if (simd_length_squared(magnet) > 0.01) {
        simd_float3 z_m = simd_normalize(magnet);
        simd_float3 h_m = simd_act(_ekf_q, (simd_float3){0.0f, 1.0f, 0.0f});
        simd_float3 y_m = z_m - h_m;

        matrix_float4x3 H_m = [self getJacobianForVec:(simd_float3){0.0f, 1.0f, 0.0f}];
        matrix_float3x3 R_m = {
            .columns[0] = {_ekfMagNoise, 0.0f, 0.0f},
            .columns[1] = {0.0f, _ekfMagNoise, 0.0f},
            .columns[2] = {0.0f, 0.0f, _ekfMagNoise}
        };

        matrix_float3x3 S_m = matrix_multiply(H_m, matrix_multiply(_ekf_P, matrix_transpose(H_m)));
        S_m.columns[0] += R_m.columns[0];
        S_m.columns[1] += R_m.columns[1];
        S_m.columns[2] += R_m.columns[2];

        matrix_float3x4 K_m = matrix_multiply(matrix_multiply(_ekf_P, matrix_transpose(H_m)), simd_inverse(S_m));

        _ekf_q.vector += matrix_multiply(K_m, y_m);
        _ekf_q = simd_normalize(_ekf_q);

        matrix_float4x4 K_H_m = matrix_multiply(K_m, H_m);
        matrix_float4x4 I_minus_KH_m = matrix_identity_float4x4;
        I_minus_KH_m.columns[0] -= K_H_m.columns[0];
        I_minus_KH_m.columns[1] -= K_H_m.columns[1];
        I_minus_KH_m.columns[2] -= K_H_m.columns[2];
        I_minus_KH_m.columns[3] -= K_H_m.columns[3];
        _ekf_P = matrix_multiply(I_minus_KH_m, _ekf_P);
    }
}

// 计算雅可比矩阵 (保持原有实现)
- (matrix_float4x3)getJacobianForVec:(simd_float3)v {
    float qx = _ekf_q.vector.x, qy = _ekf_q.vector.y, qz = _ekf_q.vector.z, qw = _ekf_q.vector.w;
    float vx = v.x, vy = v.y, vz = v.z;

    matrix_float4x3 H;
    H.columns[0] = (simd_float3){2.0f*(qy*vy + qz*vz), 2.0f*(qw*vz - 2.0f*qx*vy + qy*vx), 2.0f*(-qw*vy - 2.0f*qx*vz + qz*vx)};
    H.columns[1] = (simd_float3){2.0f*(-qw*vz - 2.0f*qy*vx + qx*vy), 2.0f*(qx*vx + qz*vz), 2.0f*(qw*vx - 2.0f*qy*vz + qz*vy)};
    H.columns[2] = (simd_float3){2.0f*(qw*vy - 2.0f*qz*vx + qx*vz), 2.0f*(-qw*vx - 2.0f*qz*vy + qy*vz), 2.0f*(qx*vx + qy*vy)};
    H.columns[3] = (simd_float3){2.0f*(-qy*vz + qz*vy), 2.0f*(qx*vz - qz*vx), 2.0f*(-qx*vy + qy*vx)};

    return H;
}

// 修正：基于论文公式13-15的航向角变化量计算
// 修正：基于论文公式13-15的航向角变化量计算
- (double)calculateGyroHeadingChange:(simd_float3)omega pitch:(double)pitch roll:(double)roll dt:(double)dt
{
    simd_float3 mutable_omega = omega;

    // 步骤 2.1: 准静态约束 (论文 3.2节) - 使用论文中的公式10 (固定 σω=0.001)
    float omega_mag = simd_length(mutable_omega);

    [self addToPostureBuffer:@(omega_mag)];
    if (_postureOmegaBuffer.count == kPostureSwitchWindowSize) {
        double sum = 0.0;
        for (NSNumber *omegaVal in _postureOmegaBuffer) {
            double normalizedOmega = [omegaVal doubleValue] / 0.001; // σω假设为0.001
            sum += (normalizedOmega * normalizedOmega);
        }
        double avgNormalizedSquare = sum / kPostureSwitchWindowSize;

        if (avgNormalizedSquare < _quasiStaticThreshold) {
            mutable_omega = simd_make_float3(0, 0, 0);
        }
    }

    // 步骤 2.2: 姿态切换检测 (论文 3.3节) - 基于公式(16)
    if (_postureOmegaBuffer.count == kPostureSwitchWindowSize) {
        float maxOmega = [[_postureOmegaBuffer valueForKeyPath:@"@max.self"] floatValue];
        float minOmega = [[_postureOmegaBuffer valueForKeyPath:@"@min.self"] floatValue];

        if ((maxOmega - minOmega) > _postureSwitchThreshold) {
            return 0.0; // 发生姿态切换，忽略此次航向变化
        }
    }

    // 步骤 2.3: 基于论文公式(13-15)的坐标变换（R^T·ω 的 Z 分量）
    double cosr = cos(roll);
    double sinr = sin(roll);
    double cosp = cos(pitch);
    double sinp = sin(pitch);

    double omega_pedestrian_z = sinr * mutable_omega.x +
                                (-cosr * sinp) * mutable_omega.y +
                                (cosr * cosp) * mutable_omega.z;

    return omega_pedestrian_z * dt;
}

// 论文核心算法：航向更新卡尔曼滤波 (保持原有实现)
// 论文核心算法：航向更新卡尔曼滤波 (保持原有实现)
- (void)updateHeadingWithKF:(double)deltaYaw noisyEkfYaw:(double)noisyEkfYaw
{
    // 1. 预测
    double x_pred = _heading_kf_x + deltaYaw;
    double p_pred = _heading_kf_P + _headingKfProcessNoise;

    // 2. 更新
    double y = noisyEkfYaw - x_pred;
    double s = p_pred + _headingKfMeasurementNoise;
    double k = p_pred / s;

    _heading_kf_x = x_pred + k * y;
    _heading_kf_P = (1 - k) * p_pred;
}

// 更新姿态切换检测的缓冲区
- (void)addToPostureBuffer:(NSNumber *)omegaMag {
    [_postureOmegaBuffer addObject:omegaMag];
    if (_postureOmegaBuffer.count > kPostureSwitchWindowSize) {
        [_postureOmegaBuffer removeObjectAtIndex:0];
    }
}

// 更新ZLDU缓冲区
- (void)addToZLDUBuffer:(NSNumber *)heading stepLength:(double)stepLength {
    [_zlduHeadingBuffer addObject:heading];
    [_zlduStepLengthBuffer addObject:@(stepLength)];
    
    if (_zlduHeadingBuffer.count > kZlduWindowSize) {
        [_zlduHeadingBuffer removeObjectAtIndex:0];
        [_zlduStepLengthBuffer removeObjectAtIndex:0];
    }
}

// 论文3.5节：直线行走检测 - 基于公式(20)
- (BOOL)detectStraightWalk {
    if (_zlduHeadingBuffer.count < kZlduWindowSize) {
        return NO;
    }

    // 计算平均航向角
    double meanHeading = [[_zlduHeadingBuffer valueForKeyPath:@"@avg.self"] doubleValue];
    
    // 计算方差
    double variance = 0.0;
    for (NSNumber *heading in _zlduHeadingBuffer) {
        double diff = [heading doubleValue] - meanHeading;
        variance += diff * diff;
    }
    variance /= _zlduHeadingBuffer.count;

    return variance < _zlduStraightWalkThreshold;
}

// 论文3.5节：完整的ZLDU Kalman Filter实现
// 论文3.5节：完整的ZLDU Kalman Filter实现
- (void)performZLDUOptimization
{
    if (_zlduHeadingBuffer.count < kZlduWindowSize || _zlduStepLengthBuffer.count < kZlduWindowSize) {
        return;
    }
    
    // 状态向量 [step_length, heading] - 论文公式(22)
    double currentStepLength = [[_zlduStepLengthBuffer lastObject] doubleValue];
    double currentHeading = [[_zlduHeadingBuffer lastObject] doubleValue];
    
    // 状态预测 (论文公式25中的A矩阵)
    matrix_float2x2 A;
    A.columns[0] = (simd_float2){1.0f, 0.0f};
    A.columns[1] = (simd_float2){0.0f, 1.0f};
    simd_float2 x_pred = {(float)currentStepLength, (float)currentHeading};
    
    // 协方差预测 (论文公式25中的Q矩阵)
    matrix_float2x2 Q;
    Q.columns[0] = (simd_float2){0.1f, 0.0f};
    Q.columns[1] = (simd_float2){0.0f, 0.1f};
    matrix_float2x2 P_pred = matrix_multiply(matrix_multiply(A, _zldu_P), matrix_transpose(A));
    P_pred.columns[0] += Q.columns[0];
    P_pred.columns[1] += Q.columns[1];
    
    // 构建观测向量 - 简化版（使用当前步长和上一步的航向）
    simd_float2 z;
    if (_zlduHeadingBuffer.count >= 2) {
        double prevHeading = [[_zlduHeadingBuffer objectAtIndex:_zlduHeadingBuffer.count-2] doubleValue];
        z = (simd_float2){(float)currentStepLength, (float)prevHeading};
    } else {
        z = (simd_float2){(float)currentStepLength, (float)currentHeading};
    }
    
    // 观测矩阵H - 论文公式(24)
    matrix_float2x2 H_zldu;
    H_zldu.columns[0] = (simd_float2){cosf((float)currentHeading), sinf((float)currentHeading)};
    H_zldu.columns[1] = (simd_float2){-currentStepLength * sinf((float)currentHeading), currentStepLength * cosf((float)currentHeading)};
    
    // 观测噪声矩阵R - 论文公式(25)
    matrix_float2x2 R;
    R.columns[0] = (simd_float2){0.01f, 0.0f};
    R.columns[1] = (simd_float2){0.0f, 0.01f};
    
    // 计算Kalman增益
    matrix_float2x2 S = matrix_multiply(H_zldu, matrix_multiply(P_pred, matrix_transpose(H_zldu)));
    S.columns[0] += R.columns[0];
    S.columns[1] += R.columns[1];
    
    matrix_float2x2 K = matrix_multiply(matrix_multiply(P_pred, matrix_transpose(H_zldu)), simd_inverse(S));
    
    // 状态更新 - 论文公式(26)
    simd_float2 h_x = matrix_multiply(H_zldu, x_pred);
    simd_float2 innovation = z - h_x;
    simd_float2 x_updated = x_pred + matrix_multiply(K, innovation);
    
    // 协方差更新
    matrix_float2x2 I_KH;
    I_KH.columns[0] = (simd_float2){1.0f, 0.0f};
    I_KH.columns[1] = (simd_float2){0.0f, 1.0f};
    matrix_float2x2 KH = matrix_multiply(K, H_zldu);
    I_KH.columns[0] -= KH.columns[0];
    I_KH.columns[1] -= KH.columns[1];
    _zldu_P = matrix_multiply(I_KH, P_pred);
    
    // 更新状态
    _zldu_step_length = x_updated.x;
    _zldu_heading = x_updated.y;
    
    // 将优化后的航向反馈到主系统（直接覆盖）
    _heading_kf_x = _zldu_heading;
    _heading = _zldu_heading;
}

// 工具函数：四元数转欧拉角 (保持原有实现)
- (simd_float3)quaternionToEulerAngles:(simd_quatf)q {
    simd_float3 angles;
    float qx = q.vector.x, qy = q.vector.y, qz = q.vector.z, qw = q.vector.w;

    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    angles.x = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        angles.y = copysign(M_PI / 2, sinp);
    else
        angles.y = asin(sinp);

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    angles.z = atan2(siny_cosp, cosy_cosp);

    return angles;
}


@end
