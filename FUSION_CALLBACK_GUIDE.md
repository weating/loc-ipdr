# 视觉位姿融合回调集成指南

## 🎯 核心理解

**你们已有：**
- ✅ 后端请求代码（发送图像、接收位姿）
- ✅ iPDR主循环（IMU更新、步检测）

**需要实现：**
- 📌 在后端返回回调中，调用融合模块
- 📌 OOSM自动处理延迟

---

## 📐 工作流程

```
┌─────────────────────────────────────────────────────────┐
│          你们已有的代码（不需要改）                      │
├─────────────────────────────────────────────────────────┤
│  1. 主循环：IMU → iPDR更新 (100 Hz)                     │
│  2. 后端请求：发送图像 (1 Hz)                           │
│  3. 后端回调：收到位姿结果                              │
└─────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│          需要添加的融合代码（3个调用点）                │
├─────────────────────────────────────────────────────────┤
│  A. 初始化：创建 VisualHeadingFusion 对象              │
│  B. IMU循环：调用 predictWithDt                         │
│  C. 后端回调：调用 addVisualMeasurement                │
│     ↓                                                    │
│  内部自动处理：                                          │
│  - 找历史状态（在t_visual时刻的iPDR状态）              │
│  - 卡尔曼更新                                           │
│  - OOSM回放（从t_visual传播到t_now）                   │
│  - 输出融合航向                                         │
└─────────────────────────────────────────────────────────┘
```

---

## 🔧 具体集成（3个步骤）

### Step A: 初始化（在 viewDidLoad）

```objective-c
// ViewController.mm

- (void)viewDidLoad {
    [super viewDidLoad];

    // ========== 你们已有的代码 ==========
    // ... 初始化 pdrEstimator ...
    // ... 初始化后端请求模块 ...

    // ========== 新增：初始化视觉融合 ==========
    self.visualFusion = [[VisualHeadingFusion alloc] init];
    [self.visualFusion setInitialHeading:0.0 variance:0.1];

    NSLog(@"✓ Visual heading fusion initialized");
}
```

---

### Step B: IMU循环中调用预测（在 IMU回调函数中）

找到你们处理IMU数据的函数，类似这样：

```objective-c
// ViewController.mm
// 你们已有的IMU处理函数（可能叫 outputDeviceMotion 或类似名字）

- (void)outputDeviceMotion:(CMDeviceMotion *)motion {

    // ========== 你们已有的代码 ==========
    static NSTimeInterval lastTime = 0;
    NSTimeInterval currentTime = motion.timestamp;

    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }

    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // 你们已有的iPDR更新
    dispatch_async(pdrQueue, ^{
        [pdrEstimator updateWithDeviceMotion:motion rawMagnetometer:nil];

        // ========== 新增：视觉融合预测 ==========
        double gyroYaw = motion.rotationRate.z;  // Z轴角速度
        [self.visualFusion predictWithDt:dt
                                gyroRate:gyroYaw
                            processNoise:0.0];  // 0表示自动计算
    });
}
```

**关键点：**
- 在你们现有的IMU处理dispatch块中添加一行
- 使用相同的`pdrQueue`保证线程安全
- `dt`和`gyroYaw`从现有代码获取

---

### Step C: 后端回调中调用融合（最关键）

找到你们处理后端返回结果的回调函数，添加融合调用：

```objective-c
// ViewController.mm
// 你们已有的后端回调函数（可能在某个网络请求完成处理中）

/**
 * 这是你们已有的函数（名字可能不同）
 * 当后端返回视觉定位结果时被调用
 */
- (void)onBackendVisualizationResultReceived:(NSDictionary *)result {

    // ========== 你们已有的代码：解析后端返回 ==========
    // 假设后端返回格式类似：
    // {
    //   "timestamp": 1234567890.123,
    //   "position": {"x": 10.5, "y": 20.3, "z": 0.0},
    //   "heading": 1.57,
    //   "confidence": 0.85
    // }

    NSTimeInterval timestamp = [[result objectForKey:@"timestamp"] doubleValue];

    NSDictionary *pos = [result objectForKey:@"position"];
    double x = [[pos objectForKey:@"x"] doubleValue];
    double y = [[pos objectForKey:@"y"] doubleValue];
    double z = [[pos objectForKey:@"z"] doubleValue];

    double heading = [[result objectForKey:@"heading"] doubleValue];
    double confidence = [[result objectForKey:@"confidence"] doubleValue];

    // ========== 新增：调用视觉融合 ==========
    // 创建视觉测量对象
    VisualMeasurement *measurement = [[VisualMeasurement alloc]
        initWithTimestamp:timestamp
        position:simd_make_float3(x, y, z)
        heading:heading
        quality:confidence];

    // 在PDR队列中处理融合（保证线程安全）
    dispatch_async(pdrQueue, ^{
        // 添加视觉测量（内部自动处理OOSM）
        BOOL accepted = [self.visualFusion addVisualMeasurement:measurement];

        if (accepted) {
            // 融合成功
            double fusedHeading = [self.visualFusion getFusedHeading];
            double uncertainty = [self.visualFusion getHeadingUncertainty];

            NSLog(@"✓ Visual fusion update:");
            NSLog(@"  iPDR heading:   %.2f°", [pdrEstimator heading] * 180.0 / M_PI);
            NSLog(@"  Visual heading: %.2f°", heading * 180.0 / M_PI);
            NSLog(@"  Fused heading:  %.2f°", fusedHeading * 180.0 / M_PI);
            NSLog(@"  Uncertainty:    %.2f°", uncertainty * 180.0 / M_PI);

            // ========== 可选：更新iPDR的航向（反馈校正） ==========
            // 如果你们想用融合后的航向替换iPDR的航向
            // [self updateIPDRHeadingWithFused:fusedHeading];

        } else {
            // 融合失败（可能是延迟太大、质量太差、异常值等）
            NSLog(@"⚠️ Visual measurement rejected");
        }
    });

    // ========== 你们已有的其他处理 ==========
    // ... 可能有的UI更新、日志记录等 ...
}
```

---

## 🔍 OOSM是如何自动工作的？

**你不需要手动处理OOSM！** 它在`addVisualMeasurement`内部自动完成：

```
当你调用：
[visualFusion addVisualMeasurement:measurement]

内部自动执行（你看不到，但在后台发生）：

1. 检查时间戳
   if (t_now - t_visual > 5s) → 拒绝（超时）

2. 查找历史状态
   在state_buffer中找到最接近t_visual的状态
   → 找到了 state_at_t_visual = {heading: 10°, variance: 0.1}

3. 在历史时刻进行卡尔曼更新
   visual_heading_local = visual_heading_global - heading_offset
   updated_heading = kalman_update(state_at_t_visual, visual_heading_local)

4. OOSM回放
   从 t_visual 到 t_now：
   for each saved_state in buffer:
       if saved_state.time > t_visual:
           updated_heading += saved_state.gyro_rate * dt

5. 替换当前状态
   current_heading = updated_heading

6. 返回 YES（成功）
```

**关键理解：**
- 📌 历史状态已经在`predictWithDt`时自动保存了
- 📌 OOSM回放在`addVisualMeasurement`内部自动完成
- 📌 你只需要提供视觉测量的时间戳和数据

---

## 📊 时间线示例

```
时间轴：
t=0.0s  t=1.0s  t=1.1s  t=1.2s  ... t=2.0s
  │       │       │       │           │
  │       │       │       │           │ 当前时刻
  │       │       │       │           │
  │       │       │       └─── IMU更新（100 Hz循环）
  │       │       │            predictWithDt(0.01, gyro)
  │       │       │            → 保存到buffer
  │       │       │
  │       │       └─── 后端回调到达！
  │       │            timestamp = 1.0s (拍照时刻)
  │       │            heading = 45°
  │       │
  │       │            调用：addVisualMeasurement(t=1.0, heading=45°)
  │       │
  │       │            内部自动：
  │       │            1. 找到buffer中t=1.0s的状态
  │       │            2. 用45°更新那个状态
  │       │            3. 从1.0s回放到2.0s
  │       │            4. 更新当前状态
  │       │
  │       └─── 拍照时刻（发送给后端）
  │            但结果延迟100ms后才返回
  │
  └─── 程序启动
```

---

## 🎓 常见问题

### Q1: 如果后端回调在另一个线程怎么办？

**A:** 没关系！在回调中用`dispatch_async(pdrQueue, ^{...})`包裹即可

```objective-c
// 后端回调可能在任意线程
- (void)onBackendResult:(NSDictionary *)result {
    // ... 解析数据 ...

    // 确保在PDR队列中处理
    dispatch_async(pdrQueue, ^{
        [self.visualFusion addVisualMeasurement:measurement];
    });
}
```

### Q2: 如何判断融合是否工作正常？

**A:** 打印统计信息

```objective-c
// 定期（例如每5秒）打印统计
- (void)printFusionStats {
    NSDictionary *stats = [self.visualFusion getStatistics];

    NSLog(@"Fusion Statistics:");
    NSLog(@"  Total measurements: %@", stats[@"total_measurements"]);
    NSLog(@"  Acceptance rate: %.1f%%",
          [stats[@"acceptance_rate"] doubleValue] * 100);
    NSLog(@"  Is aligned: %@", stats[@"is_aligned"]);
    NSLog(@"  Heading offset: %.2f°",
          [stats[@"heading_offset_deg"] doubleValue]);
}
```

**正常情况：**
- Acceptance rate > 50%
- Is aligned: YES
- Heading offset: 稳定值（例如90°）

### Q3: 后端时间戳格式不对怎么办？

**A:** 转换为相对时间

```objective-c
// 如果后端返回的是Unix时间戳（秒）
double backend_timestamp = 1234567890.123;

// 转换为相对于程序启动的时间
static NSTimeInterval app_start_time = 0;
if (app_start_time == 0) {
    app_start_time = [[NSDate date] timeIntervalSince1970];
}

double relative_timestamp = backend_timestamp - app_start_time;

// 使用relative_timestamp创建measurement
```

### Q4: 如何使用融合后的航向？

**A:** 有两种方式

**方式1：仅用于显示/记录**
```objective-c
double fusedHeading = [self.visualFusion getFusedHeading];
// 显示在UI上、记录到日志等
```

**方式2：替换iPDR航向（反馈校正）**
```objective-c
if (accepted) {
    double fusedHeading = [self.visualFusion getFusedHeading];

    // 更新iPDR的内部状态（如果iPDR支持）
    // 注意：需要检查你们的iPDR实现是否允许外部设置航向
    [pdrEstimator setHeading:fusedHeading];
}
```

---

## 📝 完整示例（伪代码）

```objective-c
@implementation ViewController

// ========== 初始化 ==========
- (void)viewDidLoad {
    [super viewDidLoad];

    // 已有：初始化iPDR
    self.pdrEstimator = [[iPDRHeadingEstimator alloc] init];

    // 已有：初始化后端请求
    [self setupBackendClient];

    // 新增：初始化视觉融合
    self.visualFusion = [[VisualHeadingFusion alloc] init];
    [self.visualFusion setInitialHeading:0.0 variance:0.1];
}

// ========== IMU循环 ==========
- (void)handleIMUData:(CMDeviceMotion *)motion {
    static NSTimeInterval lastTime = 0;
    double dt = motion.timestamp - lastTime;
    lastTime = motion.timestamp;

    dispatch_async(pdrQueue, ^{
        // 已有：iPDR更新
        [self.pdrEstimator updateWithDeviceMotion:motion
                                  rawMagnetometer:nil];

        // 新增：融合预测
        [self.visualFusion predictWithDt:dt
                                gyroRate:motion.rotationRate.z
                            processNoise:0.0];
    });
}

// ========== 后端回调 ==========
- (void)onBackendLocalizationResult:(NSDictionary *)result {
    // 已有：解析后端数据
    NSTimeInterval timestamp = [[result objectForKey:@"timestamp"] doubleValue];
    double heading = [[result objectForKey:@"heading"] doubleValue];
    double quality = [[result objectForKey:@"quality"] doubleValue];

    // 新增：调用融合
    VisualMeasurement *meas = [[VisualMeasurement alloc]
        initWithTimestamp:timestamp
        position:simd_make_float3(0, 0, 0)  // 位置不重要（仅航向）
        heading:heading
        quality:quality];

    dispatch_async(pdrQueue, ^{
        BOOL success = [self.visualFusion addVisualMeasurement:meas];
        if (success) {
            double fused = [self.visualFusion getFusedHeading];
            NSLog(@"Fused heading: %.2f°", fused * 180.0 / M_PI);
        }
    });
}

@end
```

---

## ✅ 检查清单

集成完成后，检查这些：

- [ ] `viewDidLoad`中初始化了`visualFusion`
- [ ] IMU循环中调用了`predictWithDt`
- [ ] 后端回调中调用了`addVisualMeasurement`
- [ ] 所有调用都在`pdrQueue`中执行（线程安全）
- [ ] 能看到对齐日志："✓ Heading alignment established"
- [ ] 能看到融合日志："✓ Visual fusion update"
- [ ] Acceptance rate > 50%

---

## 🎯 总结

**你需要添加的代码只有3处：**

1. **初始化**（1行）
   ```objective-c
   self.visualFusion = [[VisualHeadingFusion alloc] init];
   ```

2. **IMU循环**（1行）
   ```objective-c
   [self.visualFusion predictWithDt:dt gyroRate:gyro processNoise:0.0];
   ```

3. **后端回调**（5-10行）
   ```objective-c
   VisualMeasurement *meas = [[VisualMeasurement alloc] init...];
   [self.visualFusion addVisualMeasurement:meas];
   double fused = [self.visualFusion getFusedHeading];
   ```

**OOSM完全自动处理，你不需要管！**

---

**版本:** 1.0
**日期:** 2025-10-29
**重点:** 回调集成，OOSM自动处理
