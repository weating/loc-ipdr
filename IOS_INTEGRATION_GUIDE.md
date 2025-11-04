# iOS视觉航向融合集成指南

## 🎯 目标

将视觉定位后端接口融合到现有iPDR系统中，**精确估计设备航向角**。

---

## 📦 新增文件

已添加到你的项目：
```
ios_logger_v3_ColmapAR/ios_logger/
├── VisualHeadingFusion.h      // 接口定义
└── VisualHeadingFusion.mm     // Objective-C++实现
```

---

## 🔧 集成步骤

### Step 1: 添加文件到Xcode项目

1. 打开Xcode项目
2. 右键 `ios_logger` 文件夹 → **Add Files to "ios_logger"**
3. 选择 `VisualHeadingFusion.h` 和 `VisualHeadingFusion.mm`
4. 确保 **Copy items if needed** 和 **Add to targets** 被勾选

---

### Step 2: 在ViewController中集成

#### 2.1 在ViewController.h中添加属性

```objective-c
// ViewController.h

#import <UIKit/UIKit.h>
#import "VisualHeadingFusion.h"  // 新增

@interface ViewController : UIViewController

// ... 现有属性 ...

// 新增：视觉航向融合
@property (nonatomic, strong) VisualHeadingFusion *visualHeadingFusion;  // 新增

@end
```

#### 2.2 在viewDidLoad中初始化

```objective-c
// ViewController.mm

- (void)viewDidLoad {
    [super viewDidLoad];

    // ... 现有初始化代码 ...

    // 初始化视觉航向融合（新增）
    HeadingFusionConfig *config = [HeadingFusionConfig defaultConfig];
    config.maxInnovation = M_PI / 12.0;  // 15度最大修正
    config.minAlignmentSamples = 3;      // 3个样本对齐

    self.visualHeadingFusion = [[VisualHeadingFusion alloc] initWithConfig:config];

    // 设置初始航向（使用iPDR的初始航向）
    [self.visualHeadingFusion setInitialHeading:0.0 variance:0.1];

    NSLog(@"✓ Visual heading fusion initialized");
}
```

---

### Step 3: 在iPDR更新循环中添加预测

找到你的IMU处理函数（类似`outputDeviceMotion`），添加预测步骤：

```objective-c
// ViewController.mm
// 在你的IMU更新函数中

- (void)outputDeviceMotion:(CMDeviceMotion *)motion {
    // ... 现有的iPDR更新代码 ...

    // 获取iPDR的航向变化
    double dt = /* 时间间隔，例如 0.01 秒 */;
    double gyroYaw = motion.rotationRate.z;  // Z轴角速度

    // 视觉融合预测步骤（新增）
    [self.visualHeadingFusion predictWithDt:dt
                                   gyroRate:gyroYaw
                               processNoise:0.0];  // 0表示自动

    // ... 其余代码 ...
}
```

**更具体的集成（基于你的现有代码）：**

```objective-c
// 在 outputDeviceMotionData: 方法中（约第773行）

- (void)outputDeviceMotionData:(CMDeviceMotion *)motion {
    NSTimeInterval currentTime = motion.timestamp;
    static NSTimeInterval lastTime = 0;

    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }

    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // 现有的iPDR更新
    CMRotationRate gyro = motion.rotationRate;
    CMAcceleration userAccel = motion.userAcceleration;
    CMAcceleration gravity = motion.gravity;

    dispatch_async(pdrQueue, ^{
        [pdrEstimator updateWithDeviceMotion:motion
                             rawMagnetometer:nil];

        // 新增：视觉融合预测
        double gyroYaw = gyro.z;  // Z轴是航向轴
        [self.visualHeadingFusion predictWithDt:dt
                                       gyroRate:gyroYaw
                                   processNoise:0.0];
    });
}
```

---

### Step 4: 对接视觉定位后端接口

当从后端接口收到视觉定位结果时，调用视觉融合：

```objective-c
// ViewController.mm

/**
 * 处理视觉定位后端返回的结果
 *
 * @param timestamp 测量时间戳
 * @param position 全局位置 (x, y, z)
 * @param heading 全局航向角（弧度）
 * @param quality 质量评分（0-1，可选）
 */
- (void)onVisualLocalizationResult:(NSTimeInterval)timestamp
                          position:(simd_float3)position
                           heading:(double)heading
                           quality:(double)quality {

    // 创建视觉测量
    VisualMeasurement *measurement = [[VisualMeasurement alloc]
                                      initWithTimestamp:timestamp
                                      position:position
                                      heading:heading
                                      quality:quality];

    // 添加到融合系统
    dispatch_async(pdrQueue, ^{
        BOOL accepted = [self.visualHeadingFusion addVisualMeasurement:measurement];

        if (accepted) {
            // 获取融合后的航向
            double fusedHeading = [self.visualHeadingFusion getFusedHeading];
            double uncertainty = [self.visualHeadingFusion getHeadingUncertainty];

            NSLog(@"✓ Visual update accepted");
            NSLog(@"  Fused heading: %.2f° ± %.2f°",
                  fusedHeading * 180.0 / M_PI,
                  uncertainty * 180.0 / M_PI);

            // 可选：更新iPDR的航向（如果需要反馈）
            // [self updateIPDRHeading:fusedHeading];
        } else {
            NSLog(@"⚠️ Visual update rejected");
        }
    });
}
```

---

### Step 5: 获取融合后的航向

在需要使用航向的地方，获取融合结果：

```objective-c
// 获取当前融合航向
- (double)getCurrentHeading {
    return [self.visualHeadingFusion getFusedHeading];
}

// 获取航向不确定度
- (double)getHeadingUncertainty {
    return [self.visualHeadingFusion getHeadingUncertainty];
}

// 获取统计信息
- (void)printFusionStatistics {
    NSDictionary *stats = [self.visualHeadingFusion getStatistics];
    NSLog(@"Fusion Statistics:");
    NSLog(@"  Total measurements: %@", stats[@"total_measurements"]);
    NSLog(@"  Acceptance rate: %.1f%%", [stats[@"acceptance_rate"] doubleValue] * 100);
    NSLog(@"  Heading offset: %.2f°", [stats[@"heading_offset_deg"] doubleValue]);
    NSLog(@"  Uncertainty: %.2f°", [stats[@"current_uncertainty_deg"] doubleValue]);
}
```

---

## 🌐 后端接口集成示例

### 如果使用HTTP请求

```objective-c
// ViewController.mm

- (void)requestVisualLocalization {
    // 构建请求URL
    NSString *urlString = @"http://your-backend.com/api/localize";
    NSURL *url = [NSURL URLWithString:urlString];

    NSMutableURLRequest *request = [NSMutableURLRequest requestWithURL:url];
    [request setHTTPMethod:@"POST"];

    // 准备请求数据（例如：当前图像）
    // ... 上传图像等 ...

    NSURLSessionDataTask *task = [[NSURLSession sharedSession]
        dataTaskWithRequest:request
        completionHandler:^(NSData *data, NSURLResponse *response, NSError *error) {
            if (error) {
                NSLog(@"Visual localization request failed: %@", error);
                return;
            }

            // 解析JSON响应
            NSDictionary *json = [NSJSONSerialization JSONObjectWithData:data
                                                                options:0
                                                                  error:nil];

            // 提取结果
            NSTimeInterval timestamp = [[json objectForKey:@"timestamp"] doubleValue];

            NSArray *posArray = [json objectForKey:@"position"];
            simd_float3 position = simd_make_float3(
                [[posArray objectAtIndex:0] floatValue],
                [[posArray objectAtIndex:1] floatValue],
                [[posArray objectAtIndex:2] floatValue]
            );

            double heading = [[json objectForKey:@"heading"] doubleValue];
            double quality = [[json objectForKey:@"quality"] doubleValue];

            // 调用融合
            [self onVisualLocalizationResult:timestamp
                                    position:position
                                     heading:heading
                                     quality:quality];
        }];

    [task resume];
}

// 定期请求（例如每1秒）
- (void)startVisualLocalization {
    [NSTimer scheduledTimerWithTimeInterval:1.0
                                     target:self
                                   selector:@selector(requestVisualLocalization)
                                   userInfo:nil
                                    repeats:YES];
}
```

### 后端API响应示例

```json
{
  "timestamp": 1234567890.123,
  "position": [10.5, 20.3, 0.0],
  "heading": 1.57,
  "quality": 0.85,
  "status": "success"
}
```

---

## ⚙️ 参数调优

### 基础配置（适合大多数场景）

```objective-c
HeadingFusionConfig *config = [HeadingFusionConfig defaultConfig];
// 默认值：
// - baseVisualNoise = 0.05 rad (~2.9°)
// - maxInnovation = 0.26 rad (15°)
// - mahalanobisThreshold = 10.83
// - minAlignmentSamples = 3
```

### 如果视觉质量好（室外、光照良好）

```objective-c
config.baseVisualNoise = 0.03;      // 更小噪声 (~1.7°)
config.maxInnovation = M_PI / 18.0; // 10度限制
```

### 如果视觉质量一般（室内、复杂环境）

```objective-c
config.baseVisualNoise = 0.08;      // 更大噪声 (~4.6°)
config.maxInnovation = M_PI / 9.0;  // 20度限制
config.minAlignmentSamples = 5;     // 更多对齐样本
```

### 如果陀螺仪漂移大

```objective-c
config.baseVisualNoise = 0.04;      // 更信任视觉
// 在predictWithDt中使用更大的processNoise:
[fusion predictWithDt:dt gyroRate:gyro processNoise:0.002 * dt];
```

---

## 🧪 测试与验证

### 1. 打印调试信息

```objective-c
// 在onVisualLocalizationResult中添加
- (void)onVisualLocalizationResult:(NSTimeInterval)timestamp
                          position:(simd_float3)position
                           heading:(double)heading
                           quality:(double)quality {

    VisualMeasurement *measurement = [[VisualMeasurement alloc]
                                      initWithTimestamp:timestamp
                                      position:position
                                      heading:heading
                                      quality:quality];

    // 获取当前iPDR航向（对比）
    double ipdrHeading = [pdrEstimator heading];

    dispatch_async(pdrQueue, ^{
        BOOL accepted = [self.visualHeadingFusion addVisualMeasurement:measurement];

        if (accepted) {
            double fusedHeading = [self.visualHeadingFusion getFusedHeading];

            NSLog(@"📍 Visual Fusion Update:");
            NSLog(@"  iPDR heading:   %.2f°", ipdrHeading * 180.0 / M_PI);
            NSLog(@"  Visual heading: %.2f°", heading * 180.0 / M_PI);
            NSLog(@"  Fused heading:  %.2f°", fusedHeading * 180.0 / M_PI);
            NSLog(@"  Correction:     %.2f°", (fusedHeading - ipdrHeading) * 180.0 / M_PI);
        }
    });
}
```

### 2. 记录到文件

```objective-c
// 在现有的日志函数中添加
- (void)logFusionData {
    double fusedHeading = [self.visualHeadingFusion getFusedHeading];
    double uncertainty = [self.visualHeadingFusion getHeadingUncertainty];

    NSString *logStr = [NSString stringWithFormat:@"%.6f,%.6f,%.6f\n",
                       [[NSDate date] timeIntervalSince1970],
                       fusedHeading,
                       uncertainty];

    // 写入FusionHeading.txt
    [self writeToFile:logStr filename:@"FusionHeading.txt"];
}
```

### 3. 可视化对比

在UI中显示两个航向：
```objective-c
// 更新UI（主线程）
dispatch_async(dispatch_get_main_queue(), ^{
    double ipdrHeading = [pdrEstimator heading];
    double fusedHeading = [self.visualHeadingFusion getFusedHeading];

    self.ipdrHeadingLabel.text = [NSString stringWithFormat:@"iPDR: %.1f°",
                                  ipdrHeading * 180.0 / M_PI];
    self.fusedHeadingLabel.text = [NSString stringWithFormat:@"Fused: %.1f°",
                                   fusedHeading * 180.0 / M_PI];
});
```

---

## 📊 工作流程

```
1. 应用启动
   ↓
2. 初始化 VisualHeadingFusion
   ↓
3. IMU循环 (100 Hz)
   → predictWithDt(dt, gyroRate, ...)
   ↓
4. 视觉定位返回 (0.5-1 Hz)
   → onVisualLocalizationResult(...)
   → addVisualMeasurement(...)
   ↓
   前3次：收集对齐样本
   ⏳ Collecting alignment samples: 1/3
   ⏳ Collecting alignment samples: 2/3
   ✓ Heading alignment established! Offset: 45.3°
   ↓
   后续：视觉融合更新
   ✓ Visual update accepted
   → 卡尔曼滤波融合
   → OOSM回放（如果有延迟）
   → 输出融合航向
   ↓
5. 获取结果
   → getFusedHeading()
   → 用于导航、AR等
```

---

## ⚠️ 常见问题

### Q1: 融合后航向跳变？

**原因：** 对齐偏移估计不准确

**解决：**
```objective-c
// 增加对齐样本数
config.minAlignmentSamples = 5;  // 从3增加到5

// 减小创新限幅
config.maxInnovation = M_PI / 18.0;  // 从15°减到10°
```

### Q2: 视觉更新被拒绝（Mahalanobis）？

**原因：** iPDR漂移太大 或 视觉噪声太大

**解决：**
```objective-c
// 方案1：放宽门限
config.mahalanobisThreshold = 15.0;  // 从10.83增加

// 方案2：增大视觉噪声（降低视觉权重）
config.baseVisualNoise = 0.08;  // 从0.05增加
```

### Q3: 对齐一直停在收集样本？

**原因：** 视觉测量延迟太大，历史缓存找不到

**解决：**
```objective-c
// 检查时间戳是否正确
NSLog(@"Current time: %.3f", fusion.currentTime);
NSLog(@"Visual timestamp: %.3f", measurement.timestamp);

// 增加缓存时长
config.bufferDuration = 10.0;  // 从5秒增加到10秒
```

### Q4: 如何判断融合是否工作正常？

**检查点：**
```objective-c
// 1. 查看统计信息
NSDictionary *stats = [fusion getStatistics];
NSLog(@"Acceptance rate: %.1f%%", [stats[@"acceptance_rate"] doubleValue] * 100);
// 应该 > 50%

// 2. 查看不确定度
double uncertainty = [fusion getHeadingUncertainty];
NSLog(@"Uncertainty: %.2f°", uncertainty * 180.0 / M_PI);
// 应该逐渐减小到 < 5°

// 3. 查看对齐状态
BOOL aligned = [stats[@"is_aligned"] boolValue];
NSLog(@"Is aligned: %@", aligned ? @"YES" : @"NO");
// 应该是YES
```

---

## 🎓 最佳实践

### 1. 启动流程

```objective-c
// 应用启动时
- (void)viewDidLoad {
    // ... 初始化其他组件 ...

    // 最后初始化视觉融合
    [self setupVisualHeadingFusion];

    // 等待1-2秒让iPDR稳定
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 2 * NSEC_PER_SEC),
                   dispatch_get_main_queue(), ^{
        [self startVisualLocalization];
    });
}
```

### 2. 线程安全

```objective-c
// 所有融合操作在pdrQueue中进行
dispatch_async(pdrQueue, ^{
    [self.visualHeadingFusion predictWithDt:dt gyroRate:gyro processNoise:0.0];
    [self.visualHeadingFusion addVisualMeasurement:measurement];
});

// 读取结果可以在任意线程（只读操作）
double heading = [self.visualHeadingFusion getFusedHeading];
```

### 3. 错误处理

```objective-c
- (void)onVisualLocalizationResult:(NSTimeInterval)timestamp
                          position:(simd_float3)position
                           heading:(double)heading
                           quality:(double)quality {

    // 验证数据
    if (isnan(heading) || isinf(heading)) {
        NSLog(@"⚠️ Invalid heading from visual backend");
        return;
    }

    if (quality < 0.3) {
        NSLog(@"⚠️ Low quality visual measurement: %.2f", quality);
        // 仍然可以尝试添加，系统会自动增大噪声
    }

    // ... 正常处理 ...
}
```

---

## 📝 完整示例代码

```objective-c
// ViewController.mm 完整集成示例

#import "ViewController.h"
#import "VisualHeadingFusion.h"

@interface ViewController ()
@property (nonatomic, strong) VisualHeadingFusion *visualFusion;
@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];

    // 初始化视觉融合
    HeadingFusionConfig *config = [HeadingFusionConfig defaultConfig];
    self.visualFusion = [[VisualHeadingFusion alloc] initWithConfig:config];
    [self.visualFusion setInitialHeading:0.0 variance:0.1];

    // 2秒后开始视觉定位
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 2 * NSEC_PER_SEC),
                   dispatch_get_main_queue(), ^{
        [self startVisualLocalization];
    });
}

// IMU更新（100 Hz）
- (void)outputDeviceMotion:(CMDeviceMotion *)motion {
    static NSTimeInterval lastTime = 0;
    NSTimeInterval currentTime = motion.timestamp;

    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }

    double dt = currentTime - lastTime;
    lastTime = currentTime;

    double gyroYaw = motion.rotationRate.z;

    dispatch_async(pdrQueue, ^{
        // 现有iPDR更新
        [pdrEstimator updateWithDeviceMotion:motion rawMagnetometer:nil];

        // 视觉融合预测
        [self.visualFusion predictWithDt:dt gyroRate:gyroYaw processNoise:0.0];
    });
}

// 视觉定位请求（1 Hz）
- (void)startVisualLocalization {
    [NSTimer scheduledTimerWithTimeInterval:1.0
                                     target:self
                                   selector:@selector(requestVisualLocalization)
                                   userInfo:nil
                                    repeats:YES];
}

- (void)requestVisualLocalization {
    // 调用后端API（示例）
    NSURL *url = [NSURL URLWithString:@"http://your-backend/api/localize"];
    NSURLRequest *request = [NSURLRequest requestWithURL:url];

    [[[NSURLSession sharedSession] dataTaskWithRequest:request
        completionHandler:^(NSData *data, NSURLResponse *response, NSError *error) {
            if (!error && data) {
                NSDictionary *json = [NSJSONSerialization JSONObjectWithData:data
                                                                    options:0
                                                                      error:nil];
                [self handleVisualResult:json];
            }
        }] resume];
}

- (void)handleVisualResult:(NSDictionary *)json {
    NSTimeInterval timestamp = [[json objectForKey:@"timestamp"] doubleValue];
    NSArray *pos = [json objectForKey:@"position"];
    double heading = [[json objectForKey:@"heading"] doubleValue];
    double quality = [[json objectForKey:@"quality"] doubleValue];

    simd_float3 position = simd_make_float3(
        [[pos objectAtIndex:0] floatValue],
        [[pos objectAtIndex:1] floatValue],
        [[pos objectAtIndex:2] floatValue]
    );

    VisualMeasurement *meas = [[VisualMeasurement alloc]
                               initWithTimestamp:timestamp
                               position:position
                               heading:heading
                               quality:quality];

    dispatch_async(pdrQueue, ^{
        BOOL accepted = [self.visualFusion addVisualMeasurement:meas];

        if (accepted) {
            double fusedHeading = [self.visualFusion getFusedHeading];
            NSLog(@"✓ Fused heading: %.2f°", fusedHeading * 180.0 / M_PI);
        }
    });
}

// 获取当前航向（供其他模块使用）
- (double)getCurrentHeading {
    return [self.visualFusion getFusedHeading];
}

@end
```

---

## ✅ 集成检查清单

- [ ] 添加 `VisualHeadingFusion.h` 和 `.mm` 到Xcode项目
- [ ] 在`ViewController.h`中添加属性
- [ ] 在`viewDidLoad`中初始化融合系统
- [ ] 在IMU循环中添加`predictWithDt`调用
- [ ] 实现后端接口调用
- [ ] 实现`onVisualLocalizationResult`处理函数
- [ ] 添加日志输出验证工作
- [ ] 测试对齐过程（前3次测量）
- [ ] 测试融合效果（对比iPDR vs 融合航向）
- [ ] 调优参数（根据实际场景）

---

**版本:** 1.0
**日期:** 2025-10-29
**作者:** Claude (Anthropic)
**支持:** 针对iOS iPDR航向估计优化
