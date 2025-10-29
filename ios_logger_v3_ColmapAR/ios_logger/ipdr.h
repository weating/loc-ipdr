//
//  ipdr.h
//  ios_logger
//
//  Created by jerryzh on 2025/7/16.
//  Copyright © 2025 Mac. All rights reserved.
//
#import <Foundation/Foundation.h>
#import <CoreMotion/CoreMotion.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

@interface iPDRHeadingEstimator : NSObject

// --- 可调参数 (Tunable Parameters) ---
// 这些参数的初始值基于论文或经验，您可能需要根据实际效果进行微调

// EKF 噪声协方差
@property (nonatomic, assign) float ekfGyroNoise;       // 陀螺仪过程噪声
@property (nonatomic, assign) float ekfAccelNoise;      // 加速度计测量噪声
@property (nonatomic, assign) float ekfMagNoise;        // 磁力计测量噪声

// 第二阶段KF (航向更新) 噪声协方差
@property (nonatomic, assign) float headingKfProcessNoise;
@property (nonatomic, assign) float headingKfMeasurementNoise;

// 约束阈值
@property (nonatomic, assign) float quasiStaticThreshold;     // 准静态检测阈值 (rad/s)
@property (nonatomic,assign) float postureSwitchThreshold;    // 姿态切换检测阈值 (rad/s)
@property (nonatomic, assign) float zlduStraightWalkThreshold; // ZLDU直线行走检测阈值 (rad^2)
@property (nonatomic, assign) float quasiStaticVarianceThreshold; // 新的方差阈值
@property (nonatomic, assign) float headingKfHighMeasurementNoise; // 磁场差时的R值


// --- 算法输出 (Algorithm Outputs) ---
@property (nonatomic, readonly) double heading; // 最终输出的航向角 (弧度)
@property (nonatomic, readonly) double pitch;   // 俯仰角 (弧度)
@property (nonatomic, readonly) double roll;    // 翻滚角 (弧度)
@property (nonatomic, readonly) simd_quatf attitudeQuaternion; // 完整的姿态四元数


// --- 公共方法 (Public Methods) ---
- (instancetype)init;

/**
 * @brief 核心更新函数。每次获取到新的传感器数据时调用。
 * @param motion CMDeviceMotion对象，提供了同步的陀螺仪、加速度计和校准后的磁场数据。
 * @param rawMagnetometerData 可选的原始磁力计数据，如果motion.magneticField.accuracy太低，可使用此数据。
 */
- (void)updateWithDeviceMotion:(CMDeviceMotion *)motion rawMagnetometer:(nullable CMMagnetometerData *)rawMagnetometerData;

/**
 * @brief （可选）当检测到一步时调用，用于触发ZLDU优化。
 * @param stepLength 估计的步长（米）。
 */
- (void)didTakeStepWithLength:(double)stepLength;


@end

NS_ASSUME_NONNULL_END
