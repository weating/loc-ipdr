//
//  VisualHeadingFusion.h
//  iPDR Visual Heading Fusion Module
//
//  专注于航向角估计的视觉融合模块
//  轻量级实现，易于集成到现有iPDR系统
//

#import <Foundation/Foundation.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

#pragma mark - Visual Measurement

/**
 * 视觉定位测量（来自后端接口）
 */
@interface VisualMeasurement : NSObject

@property (nonatomic, assign) NSTimeInterval timestamp;      // 测量时间戳
@property (nonatomic, assign) simd_float3 position;          // 全局位置 [x, y, z]
@property (nonatomic, assign) double heading;                // 全局航向角（弧度，-π to π）
@property (nonatomic, assign) double quality;                // 质量评分（0-1，可选）

- (instancetype)initWithTimestamp:(NSTimeInterval)timestamp
                         position:(simd_float3)position
                          heading:(double)heading
                          quality:(double)quality;

@end


#pragma mark - Heading Fusion Configuration

/**
 * 航向融合配置参数
 */
@interface HeadingFusionConfig : NSObject

// 自适应权重
@property (nonatomic, assign) double baseVisualNoise;        // 基础视觉噪声（弧度，默认 0.05 = 2.9°）
@property (nonatomic, assign) double qualityFactor;          // 质量因子（默认 2.0）

// Mahalanobis门限
@property (nonatomic, assign) double mahalanobisThreshold;   // 默认 10.83 (1自由度，99.9%置信度)

// 创新限幅
@property (nonatomic, assign) double maxInnovation;          // 最大创新（弧度，默认 15° = 0.26 rad）

// 历史缓存
@property (nonatomic, assign) NSTimeInterval bufferDuration; // 缓存时长（秒，默认 5.0）
@property (nonatomic, assign) NSInteger maxBufferSize;       // 最大缓存数量（默认 500）

// 对齐参数
@property (nonatomic, assign) NSInteger minAlignmentSamples; // 最小对齐样本数（默认 3）
@property (nonatomic, assign) BOOL autoAlign;                // 自动对齐（默认 YES）

+ (instancetype)defaultConfig;

@end


#pragma mark - Heading State (内部使用)

/**
 * 航向状态（用于历史缓存）
 */
@interface HeadingState : NSObject

@property (nonatomic, assign) NSTimeInterval timestamp;
@property (nonatomic, assign) double heading;                // 航向角（弧度）
@property (nonatomic, assign) double variance;               // 方差
@property (nonatomic, assign) double gyroRate;               // 陀螺仪角速度（用于回放）

@end


#pragma mark - Visual Heading Fusion

/**
 * 视觉航向融合主类
 *
 * 功能：
 * 1. 融合iPDR航向和视觉定位航向
 * 2. 自动估计坐标系航向偏移
 * 3. 处理延迟测量（OOSM）
 * 4. Mahalanobis异常检测
 * 5. 自适应权重
 */
@interface VisualHeadingFusion : NSObject

@property (nonatomic, strong, readonly) HeadingFusionConfig *config;

// 当前状态
@property (nonatomic, assign, readonly) double currentHeading;        // 当前航向（弧度）
@property (nonatomic, assign, readonly) double currentVariance;       // 当前方差
@property (nonatomic, assign, readonly) BOOL isAligned;               // 是否已对齐

// 对齐状态
@property (nonatomic, assign, readonly) double headingOffset;         // 航向偏移（弧度）
@property (nonatomic, assign, readonly) NSInteger alignmentSamples;   // 对齐样本数

// 统计信息
@property (nonatomic, assign, readonly) NSUInteger totalMeasurements;
@property (nonatomic, assign, readonly) NSUInteger acceptedMeasurements;
@property (nonatomic, assign, readonly) NSUInteger rejectedMahalanobis;
@property (nonatomic, assign, readonly) NSUInteger rejectedTimeout;

/**
 * 初始化
 */
- (instancetype)initWithConfig:(HeadingFusionConfig *)config;
- (instancetype)init; // 使用默认配置

/**
 * 设置初始航向
 */
- (void)setInitialHeading:(double)heading variance:(double)variance;

/**
 * 预测步骤（基于陀螺仪）
 * @param dt 时间增量（秒）
 * @param gyroRate 陀螺仪航向角速度（弧度/秒）
 * @param processNoise 过程噪声（弧度^2，可选，默认自动）
 */
- (void)predictWithDt:(double)dt
             gyroRate:(double)gyroRate
         processNoise:(double)processNoise;

/**
 * 添加视觉测量
 * @param measurement 视觉测量
 * @return 是否接受该测量
 */
- (BOOL)addVisualMeasurement:(VisualMeasurement *)measurement;

/**
 * 获取融合后的航向（主接口）
 * @return 航向角（弧度，-π to π）
 */
- (double)getFusedHeading;

/**
 * 获取航向不确定度
 * @return 标准差（弧度）
 */
- (double)getHeadingUncertainty;

/**
 * 重置系统
 */
- (void)reset;

/**
 * 获取统计信息
 */
- (NSDictionary *)getStatistics;

@end

NS_ASSUME_NONNULL_END
