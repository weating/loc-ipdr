//
//  VisualHeadingFusion.mm
//  iPDR Visual Heading Fusion Module
//
//  Created by Claude (Anthropic)
//  Objective-C++ implementation
//

#import "VisualHeadingFusion.h"
#include <cmath>
#include <deque>

// 辅助函数：角度归一化到 [-π, π]
static inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 辅助函数：角度差（考虑周期性）
static inline double angleDifference(double a, double b) {
    return normalizeAngle(a - b);
}

#pragma mark - Visual Measurement Implementation

@implementation VisualMeasurement

- (instancetype)initWithTimestamp:(NSTimeInterval)timestamp
                         position:(simd_float3)position
                          heading:(double)heading
                          quality:(double)quality {
    self = [super init];
    if (self) {
        _timestamp = timestamp;
        _position = position;
        _heading = normalizeAngle(heading);
        _quality = quality;
    }
    return self;
}

@end


#pragma mark - Heading Fusion Config Implementation

@implementation HeadingFusionConfig

+ (instancetype)defaultConfig {
    HeadingFusionConfig *config = [[HeadingFusionConfig alloc] init];
    config.baseVisualNoise = 0.05;          // ~2.9度
    config.qualityFactor = 2.0;
    config.mahalanobisThreshold = 10.83;    // 1自由度，99.9%
    config.maxInnovation = M_PI / 12.0;     // 15度
    config.bufferDuration = 5.0;            // 5秒缓存
    config.maxBufferSize = 500;
    config.minAlignmentSamples = 3;
    config.autoAlign = YES;
    return config;
}

@end


#pragma mark - Heading State Implementation

@implementation HeadingState
@end


#pragma mark - Visual Heading Fusion Implementation

@interface VisualHeadingFusion () {
    std::deque<HeadingState *> _stateBuffer;
    std::deque<VisualMeasurement *> _alignmentSamples;
}

@property (nonatomic, strong) HeadingFusionConfig *config;
@property (nonatomic, assign) double currentHeading;
@property (nonatomic, assign) double currentVariance;
@property (nonatomic, assign) BOOL isAligned;
@property (nonatomic, assign) double headingOffset;
@property (nonatomic, assign) NSInteger alignmentSamples;
@property (nonatomic, assign) NSTimeInterval currentTime;

// 统计
@property (nonatomic, assign) NSUInteger totalMeasurements;
@property (nonatomic, assign) NSUInteger acceptedMeasurements;
@property (nonatomic, assign) NSUInteger rejectedMahalanobis;
@property (nonatomic, assign) NSUInteger rejectedTimeout;

@end

@implementation VisualHeadingFusion

#pragma mark - Initialization

- (instancetype)init {
    return [self initWithConfig:[HeadingFusionConfig defaultConfig]];
}

- (instancetype)initWithConfig:(HeadingFusionConfig *)config {
    self = [super init];
    if (self) {
        _config = config;
        _currentHeading = 0.0;
        _currentVariance = M_PI * M_PI;  // 高初始不确定度
        _isAligned = NO;
        _headingOffset = 0.0;
        _alignmentSamples = 0;
        _currentTime = 0.0;

        _totalMeasurements = 0;
        _acceptedMeasurements = 0;
        _rejectedMahalanobis = 0;
        _rejectedTimeout = 0;

        _stateBuffer.clear();
        _alignmentSamples.clear();
    }
    return self;
}

#pragma mark - Public Methods

- (void)setInitialHeading:(double)heading variance:(double)variance {
    _currentHeading = normalizeAngle(heading);
    _currentVariance = variance;
    _currentTime = 0.0;

    // 保存初始状态
    [self _saveCurrentState:0.0];
}

- (void)predictWithDt:(double)dt
             gyroRate:(double)gyroRate
         processNoise:(double)processNoise {

    if (processNoise <= 0) {
        // 自动计算过程噪声（基于陀螺仪噪声）
        processNoise = 0.001 * dt;  // 弧度^2
    }

    // 预测步骤（1D卡尔曼滤波）
    // x_pred = x_prev + gyroRate * dt
    // P_pred = P_prev + Q

    _currentHeading = normalizeAngle(_currentHeading + gyroRate * dt);
    _currentVariance += processNoise;
    _currentTime += dt;

    // 保存到历史缓存
    [self _saveCurrentState:gyroRate];

    // 清理旧状态
    [self _trimStateBuffer];
}

- (BOOL)addVisualMeasurement:(VisualMeasurement *)measurement {
    _totalMeasurements++;

    // 检查是否超时
    NSTimeInterval delay = _currentTime - measurement.timestamp;
    if (delay > _config.bufferDuration || delay < 0) {
        _rejectedTimeout++;
        return NO;
    }

    // 如果未对齐，收集对齐样本
    if (!_isAligned && _config.autoAlign) {
        return [self _processAlignmentSample:measurement];
    }

    // 已对齐，进行融合
    return [self _processVisualUpdate:measurement];
}

- (double)getFusedHeading {
    return normalizeAngle(_currentHeading);
}

- (double)getHeadingUncertainty {
    return sqrt(_currentVariance);
}

- (void)reset {
    _currentHeading = 0.0;
    _currentVariance = M_PI * M_PI;
    _isAligned = NO;
    _headingOffset = 0.0;
    _alignmentSamples = 0;
    _currentTime = 0.0;

    _totalMeasurements = 0;
    _acceptedMeasurements = 0;
    _rejectedMahalanobis = 0;
    _rejectedTimeout = 0;

    _stateBuffer.clear();
    _alignmentSamples.clear();
}

- (NSDictionary *)getStatistics {
    double acceptanceRate = _totalMeasurements > 0 ?
        (double)_acceptedMeasurements / _totalMeasurements : 0.0;

    return @{
        @"total_measurements": @(_totalMeasurements),
        @"accepted_measurements": @(_acceptedMeasurements),
        @"rejected_mahalanobis": @(_rejectedMahalanobis),
        @"rejected_timeout": @(_rejectedTimeout),
        @"acceptance_rate": @(acceptanceRate),
        @"is_aligned": @(_isAligned),
        @"heading_offset_deg": @(_headingOffset * 180.0 / M_PI),
        @"current_uncertainty_deg": @([self getHeadingUncertainty] * 180.0 / M_PI)
    };
}

#pragma mark - Private Methods - Alignment

- (BOOL)_processAlignmentSample:(VisualMeasurement *)measurement {
    // 添加到对齐样本
    _alignmentSamples.push_back(measurement);
    _alignmentSamples++;

    NSLog(@"⏳ Collecting alignment samples: %ld/%ld",
          (long)_alignmentSamples,
          (long)_config.minAlignmentSamples);

    // 检查是否有足够样本
    if (_alignmentSamples < _config.minAlignmentSamples) {
        return NO;
    }

    // 估计航向偏移（使用循环平均）
    [self _estimateHeadingOffset];

    _isAligned = YES;

    NSLog(@"✓ Heading alignment established!");
    NSLog(@"  Offset: %.2f°", _headingOffset * 180.0 / M_PI);
    NSLog(@"  Samples: %ld", (long)_alignmentSamples);

    return YES;
}

- (void)_estimateHeadingOffset {
    // 使用循环平均估计航向偏移
    // offset = mean(visual_heading - ipdr_heading)

    double sumSin = 0.0;
    double sumCos = 0.0;

    for (VisualMeasurement *meas : _alignmentSamples) {
        // 找到最接近的历史状态
        HeadingState *state = [self _findClosestState:meas.timestamp];
        if (!state) continue;

        // 计算偏移
        double offset = angleDifference(meas.heading, state.heading);

        // 循环平均（使用复数表示）
        sumSin += sin(offset);
        sumCos += cos(offset);
    }

    _headingOffset = atan2(sumSin, sumCos);
}

#pragma mark - Private Methods - Visual Update

- (BOOL)_processVisualUpdate:(VisualMeasurement *)measurement {
    // 1. 找到对应的历史状态
    HeadingState *historicalState = [self _findClosestState:measurement.timestamp];
    if (!historicalState) {
        _rejectedTimeout++;
        return NO;
    }

    // 2. 将视觉测量转换到iPDR坐标系
    double visualHeadingLocal = normalizeAngle(measurement.heading - _headingOffset);

    // 3. 计算自适应测量噪声
    double R = [self _computeMeasurementNoise:measurement.quality];

    // 4. 创建更新后的状态副本
    double updatedHeading = historicalState.heading;
    double updatedVariance = historicalState.variance;

    // 5. 卡尔曼更新
    BOOL accepted = [self _kalmanUpdate:&updatedHeading
                               variance:&updatedVariance
                            measurement:visualHeadingLocal
                          measurementR:R];

    if (!accepted) {
        _rejectedMahalanobis++;
        return NO;
    }

    // 6. OOSM回放：从historical时刻传播到当前
    [self _replayFromState:historicalState
            updatedHeading:updatedHeading
          updatedVariance:updatedVariance];

    _acceptedMeasurements++;
    return YES;
}

- (BOOL)_kalmanUpdate:(double *)heading
             variance:(double *)variance
          measurement:(double)z
         measurementR:(double)R {

    // 1D卡尔曼更新
    double x = *heading;
    double P = *variance;

    // 创新（innovation）
    double y = angleDifference(z, x);

    // 创新限幅
    if (fabs(y) > _config.maxInnovation) {
        double sign = y > 0 ? 1.0 : -1.0;
        y = sign * _config.maxInnovation;
    }

    // 创新协方差
    double S = P + R;

    // Mahalanobis距离检验
    double mahalanobis = (y * y) / S;
    if (mahalanobis > _config.mahalanobisThreshold) {
        return NO;  // 拒绝异常值
    }

    // 卡尔曼增益
    double K = P / S;

    // 状态更新
    *heading = normalizeAngle(x + K * y);
    *variance = (1.0 - K) * P;  // 简化形式

    return YES;
}

- (void)_replayFromState:(HeadingState *)historicalState
          updatedHeading:(double)updatedHeading
        updatedVariance:(double)updatedVariance {

    // 从历史时刻传播到当前时刻
    double heading = updatedHeading;
    double variance = updatedVariance;
    NSTimeInterval timestamp = historicalState.timestamp;

    // 找到所有需要回放的状态
    for (HeadingState *state : _stateBuffer) {
        if (state.timestamp <= historicalState.timestamp) {
            continue;
        }

        // 预测步骤
        double dt = state.timestamp - timestamp;
        heading = normalizeAngle(heading + state.gyroRate * dt);
        variance += 0.001 * dt;  // 过程噪声

        timestamp = state.timestamp;
    }

    // 更新当前状态
    _currentHeading = heading;
    _currentVariance = variance;
}

#pragma mark - Private Methods - Utilities

- (double)_computeMeasurementNoise:(double)quality {
    // 基于质量的自适应噪声
    // R = baseNoise * (qualityFactor / quality)

    double qualityWeight = _config.qualityFactor / fmax(quality, 0.1);
    double R = _config.baseVisualNoise * qualityWeight;

    // 最小噪声下限
    R = fmax(R, 0.01);  // ~0.57度

    return R * R;  // 方差
}

- (void)_saveCurrentState:(double)gyroRate {
    HeadingState *state = [[HeadingState alloc] init];
    state.timestamp = _currentTime;
    state.heading = _currentHeading;
    state.variance = _currentVariance;
    state.gyroRate = gyroRate;

    _stateBuffer.push_back(state);

    // 限制缓存大小
    if (_stateBuffer.size() > (size_t)_config.maxBufferSize) {
        _stateBuffer.pop_front();
    }
}

- (void)_trimStateBuffer {
    // 移除超出时间窗口的状态
    while (!_stateBuffer.empty()) {
        HeadingState *state = _stateBuffer.front();
        if (_currentTime - state.timestamp > _config.bufferDuration) {
            _stateBuffer.pop_front();
        } else {
            break;
        }
    }
}

- (nullable HeadingState *)_findClosestState:(NSTimeInterval)targetTime {
    if (_stateBuffer.empty()) {
        return nil;
    }

    HeadingState *closest = nil;
    NSTimeInterval minDiff = INFINITY;

    for (HeadingState *state : _stateBuffer) {
        NSTimeInterval diff = fabs(state.timestamp - targetTime);
        if (diff < minDiff) {
            minDiff = diff;
            closest = state;
        }
    }

    // 检查是否在可接受范围内
    if (minDiff > 0.1) {  // 100ms容差
        return nil;
    }

    return closest;
}

@end
