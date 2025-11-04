# iPDR-COLMAP 坐标系对齐完全指南

## 🎯 核心问题

**问题：** iPDR和COLMAP使用不同的坐标系

| 坐标系 | 原点 | 方向 | 尺度 |
|--------|------|------|------|
| **iPDR局部坐标系** | (0, 0, 0) 任意起点 | 任意初始朝向 | 米（步长估计） |
| **COLMAP全局坐标系** | 固定世界原点 | 固定世界方向 | 米/任意单位 |

**如果不对齐，直接融合会导致：**
- ❌ 位置完全错位
- ❌ 朝向不匹配
- ❌ 尺度不一致

---

## 🔧 三种对齐方法

### 方法1：首次视觉对齐（First Visual Alignment）

**原理：** 使用第一个视觉观测建立坐标系关系

```
当收到第一个COLMAP测量时：
1. 记录 iPDR位置 p_local 和 COLMAP位置 p_global
2. 记录 iPDR朝向 yaw_local 和 COLMAP朝向 yaw_global
3. 计算变换：
   - 旋转: R = Rot_z(yaw_global - yaw_local)
   - 平移: t = p_global - R * p_local
```

**代码示例：**

```python
from ipdr_visual_fusion_with_alignment import iPDRVisualFusionAligned
from ipdr_visual_fusion import VisualFusionConfig, EKFState
import numpy as np

# 创建系统，使用首次对齐
config = VisualFusionConfig()
system = iPDRVisualFusionAligned(
    config=config,
    alignment_mode='first_visual'  # 首次对齐模式
)

# 初始化（iPDR局部坐标系）
initial_state = EKFState(
    t=0.0,
    x=np.zeros(10),
    P=np.eye(10) * 0.1
)
system.initialize_state(initial_state)

# 主循环
while running:
    # IMU更新
    system.predict_step(dt, acc, gyro)

    # 当收到COLMAP测量（全局坐标系）
    if colmap_pose_available:
        system.add_visual_measurement_global(
            t_v=colmap_time,
            p_global=colmap_position,  # [x_g, y_g, z_g]
            yaw_global=colmap_yaw,      # 全局朝向
            e_repr=reproj_error,
            n_inlier=num_inliers
        )
        system.process_visual_measurements()
```

**优点：**
- ✅ 简单快速，立即可用
- ✅ 只需一次观测

**缺点：**
- ⚠️ 依赖第一次测量质量
- ⚠️ 如果首次测量有误差，会传播到后续

**适用场景：**
- 第一次COLMAP测量质量很好
- 快速原型开发
- 对精度要求不高

---

### 方法2：多点鲁棒对齐（Multi-Point Robust Alignment）

**原理：** 收集多个对应点，使用最小二乘估计变换

```
收集N个对应点对：
{(p_local_1, p_global_1), ..., (p_local_N, p_global_N)}

使用Umeyama算法求解：
1. 计算质心
2. SVD分解
3. 估计最佳旋转R和平移t
```

**数学公式：**

```
给定对应点集：
P_local = [p1_l, p2_l, ..., pN_l]  (3xN)
P_global = [p1_g, p2_g, ..., pN_g] (3xN)

求解：p_global = s * R * p_local + t

步骤：
1. 计算质心：
   μ_l = mean(P_local)
   μ_g = mean(P_global)

2. 中心化：
   P_l' = P_local - μ_l
   P_g' = P_global - μ_g

3. 协方差矩阵：
   H = P_l' * P_g'^T

4. SVD分解：
   [U, S, V^T] = SVD(H)

5. 旋转矩阵：
   R = V * U^T
   如果 det(R) < 0: 反转最后一列

6. 尺度（可选）：
   s = trace(S) / var(P_l')

7. 平移：
   t = μ_g - s * R * μ_l
```

**代码示例：**

```python
# 创建系统，使用多点对齐
system = iPDRVisualFusionAligned(
    config=config,
    alignment_mode='multi_point'  # 多点对齐模式
)

# 系统会自动收集前3个对应点
# 然后估计最佳变换

# 使用方式与方法1相同
system.initialize_state(initial_state)

while running:
    system.predict_step(dt, acc, gyro)

    if colmap_pose_available:
        # 前几次测量会用于对齐
        system.add_visual_measurement_global(
            t_v=colmap_time,
            p_global=colmap_position,
            yaw_global=colmap_yaw,
            e_repr=reproj_error,
            n_inlier=num_inliers
        )
        system.process_visual_measurements()

# 系统会输出：
# ⏳ Collecting alignment pairs: 1/3
# ⏳ Collecting alignment pairs: 2/3
# ✓ Frame alignment initialized with 3 pairs
```

**优点：**
- ✅ 鲁棒性强，能平均多个测量的误差
- ✅ 对单次测量误差不敏感
- ✅ 可以估计尺度（如果需要）

**缺点：**
- ⚠️ 需要等待多次测量（默认3次）
- ⚠️ 启动延迟

**适用场景：**
- **推荐用于生产环境**
- COLMAP测量可能有噪声
- 需要高精度对齐

---

### 方法3：预标定对齐（Pre-Calibrated Alignment）

**原理：** 离线标定坐标系关系，直接使用已知变换

```
离线标定流程：
1. 在同一场景采集iPDR和COLMAP数据
2. 手动标注对应点或使用标定工具
3. 估计变换参数 (R, t, s)
4. 保存标定结果
5. 运行时直接加载使用
```

**代码示例：**

```python
# 创建系统
system = iPDRVisualFusionAligned(
    config=config,
    alignment_mode='pre_calibrated'
)

# 设置已知的变换（从标定获得）
R_calibrated = np.array([
    [0, -1, 0],   # 90度旋转
    [1,  0, 0],
    [0,  0, 1]
])

t_calibrated = np.array([100.0, 50.0, 0.0])  # 平移
scale_calibrated = 1.0  # 尺度

system.aligner.set_pre_calibrated_transform(
    R=R_calibrated,
    t=t_calibrated,
    scale=scale_calibrated
)

# 之后正常使用，无需等待对齐
system.initialize_state(initial_state)

while running:
    system.predict_step(dt, acc, gyro)

    if colmap_pose_available:
        # 直接使用，无延迟
        system.add_visual_measurement_global(...)
        system.process_visual_measurements()
```

**离线标定工具示例：**

```python
from coordinate_alignment import FrameAlignmentManager

# 收集对应点（手动标注或自动匹配）
aligner = FrameAlignmentManager(mode='multi_point')

# 添加多个对应点
correspondence_pairs = [
    # (iPDR位置, COLMAP位置, iPDR朝向, COLMAP朝向)
    (np.array([0, 0, 0]), np.array([100, 50, 0]), 0.0, np.pi/2),
    (np.array([1, 0, 0]), np.array([100, 51, 0]), 0.0, np.pi/2),
    (np.array([2, 0, 0]), np.array([100, 52, 0]), 0.0, np.pi/2),
    # ... 更多点
]

for p_local, p_global, yaw_local, yaw_global in correspondence_pairs:
    aligner.add_alignment_pair(p_local, p_global, yaw_local, yaw_global)

# 估计变换
aligner.estimate_transform()

# 保存结果
import json
calibration = {
    'R': aligner.transform.R.tolist(),
    't': aligner.transform.t.tolist(),
    'scale': aligner.transform.scale
}

with open('calibration.json', 'w') as f:
    json.dump(calibration, f, indent=2)

print("Calibration saved!")
```

**优点：**
- ✅ 零延迟，立即可用
- ✅ 最高精度（充分标定的情况下）
- ✅ 可重复使用，适合固定环境

**缺点：**
- ⚠️ 需要离线标定步骤
- ⚠️ 如果环境改变需要重新标定
- ⚠️ iPDR起点必须相对固定

**适用场景：**
- 固定环境（如仓库、博物馆）
- 对实时性要求高
- 有充分时间做离线标定

---

## 🔄 坐标变换数学

### 正向变换（局部→全局）

```python
def transform_local_to_global(p_local, R, t, scale=1.0):
    """
    将iPDR局部坐标转换为COLMAP全局坐标

    p_global = scale * R * p_local + t
    """
    return scale * (R @ p_local) + t

def transform_yaw_local_to_global(yaw_local, R):
    """
    将iPDR局部朝向转换为COLMAP全局朝向

    yaw_global = yaw_local + yaw_offset
    """
    yaw_offset = np.arctan2(R[1, 0], R[0, 0])
    return yaw_local + yaw_offset
```

### 逆向变换（全局→局部）

```python
def transform_global_to_local(p_global, R, t, scale=1.0):
    """
    将COLMAP全局坐标转换为iPDR局部坐标

    p_local = R^T * (p_global - t) / scale
    """
    return R.T @ ((p_global - t) / scale)

def transform_yaw_global_to_local(yaw_global, R):
    """
    将COLMAP全局朝向转换为iPDR局部朝向

    yaw_local = yaw_global - yaw_offset
    """
    yaw_offset = np.arctan2(R[1, 0], R[0, 0])
    return yaw_global - yaw_offset
```

---

## 📊 三种方法对比

| 特性 | 首次对齐 | 多点对齐 | 预标定 |
|------|---------|---------|--------|
| **实现复杂度** | 简单 | 中等 | 中等 |
| **启动延迟** | 1次测量 | 3次测量 | 无 |
| **鲁棒性** | 低 | 高 | 最高 |
| **精度** | 依赖首次 | 高 | 最高 |
| **适应性** | 高 | 高 | 低 |
| **离线工作** | 否 | 否 | 是 |

**推荐选择：**
- 🏆 **生产环境**：多点对齐（默认推荐）
- 🚀 **快速原型**：首次对齐
- 🎯 **固定环境**：预标定

---

## 🛠️ 完整使用流程

### Step 1: 选择对齐模式

```python
from ipdr_visual_fusion_with_alignment import iPDRVisualFusionAligned
from ipdr_visual_fusion import VisualFusionConfig

config = VisualFusionConfig()
config.max_pos_innovation = 3.0
config.max_yaw_innovation = np.deg2rad(10)

# 选择一种模式
system = iPDRVisualFusionAligned(
    config=config,
    alignment_mode='multi_point'  # 'first_visual', 'multi_point', 'pre_calibrated'
)
```

### Step 2: 初始化（局部坐标系）

```python
import numpy as np
from ipdr_visual_fusion import EKFState

# iPDR总是从局部原点开始
initial_x = np.zeros(10)
initial_x[0:3] = [0, 0, 0]      # 局部原点
initial_x[3] = 0.0              # 初始朝向（局部坐标系中的东向）
initial_P = np.eye(10) * 0.1

initial_state = EKFState(t=0.0, x=initial_x, P=initial_P)
system.initialize_state(initial_state)
```

### Step 3: 主循环

```python
while running:
    # 1. IMU更新（高频，100 Hz）
    system.predict_step(
        dt=0.01,
        acc=imu_acceleration,  # [ax, ay, az] in body frame
        gyro=imu_gyroscope      # [wx, wy, wz] in body frame
    )

    # 2. 步检测（低频，1-2 Hz）
    if step_detected:
        # 注意：步长是在predict_step中使用的
        pass

    # 3. 视觉更新（很低频，0.1-1 Hz）
    if colmap_pose_available:
        # COLMAP返回的是全局坐标
        success = system.add_visual_measurement_global(
            t_v=colmap_timestamp,
            p_global=colmap_position,    # [x_g, y_g, z_g] in world frame
            yaw_global=colmap_heading,   # yaw in world frame (radians)
            e_repr=colmap_reproj_error,
            n_inlier=colmap_num_inliers
        )

        if success:
            # 对齐已建立，处理视觉更新
            num_updates = system.process_visual_measurements()

            if num_updates > 0:
                print(f"✓ Visual update applied")
```

### Step 4: 获取结果

```python
# 获取局部坐标系中的位姿
p_local, ori_local = system.get_current_pose_local()
print(f"Local position: {p_local}")
print(f"Local yaw: {np.rad2deg(ori_local[0]):.1f}°")

# 获取全局坐标系中的位姿（需要对齐已建立）
if system.alignment_initialized:
    p_global, ori_global = system.get_current_pose_global()
    print(f"Global position: {p_global}")
    print(f"Global yaw: {np.rad2deg(ori_global[0]):.1f}°")

# 获取统计信息
stats = system.get_statistics()
print(f"Visual updates: {stats['visual_updates']}")
print(f"Alignment: {stats['alignment']}")
```

---

## 🧪 测试与验证

### 运行测试脚本

```bash
# 1. 坐标对齐示例
python coordinate_alignment.py

# 2. 完整融合示例（带对齐）
python ipdr_visual_fusion_with_alignment.py

# 3. 原始测试（不含对齐）
python test_visual_fusion.py
```

### 验证对齐是否正确

```python
# 检查变换参数是否合理
stats = system.get_statistics()
alignment = stats['alignment']

if alignment['initialized']:
    print(f"Rotation: {alignment['rotation_yaw_deg']:.2f}°")
    print(f"Translation: {alignment['translation']}")
    print(f"Scale: {alignment['scale']}")

    # 合理性检查
    assert abs(alignment['scale'] - 1.0) < 0.1, "Scale should be close to 1.0"
    assert len(alignment['translation']) == 3, "Translation should be 3D"
```

---

## ⚠️ 常见问题与解决方案

### 问题1: 对齐失败

**症状：**
```
⏳ Collecting alignment pairs: 1/3
⏳ Collecting alignment pairs: 2/3
⏳ Collecting alignment pairs: 3/3
⏳ Collecting alignment pairs: 4/3  # 永远停在这里
```

**原因：**
- COLMAP测量质量太差
- iPDR漂移太大
- 对应点不够分散

**解决：**
```python
# 降低质量要求
config.min_inliers = 5  # 从10降到5
config.max_reproj_error = 10.0  # 从5.0增加到10.0

# 减少所需对齐点数
system.aligner.min_pairs_required = 2  # 从3降到2
```

### 问题2: 对齐后位置仍然跳变

**症状：**
- 对齐成功，但视觉更新后位置突变

**原因：**
- 对齐时刻iPDR已经漂移较大
- 变换估计不准确

**解决：**
```python
# 方案1: 尽早建立对齐（前几秒内）
# 在系统启动后立即获取视觉测量

# 方案2: 使用多点对齐
system = iPDRVisualFusionAligned(
    config=config,
    alignment_mode='multi_point'  # 更鲁棒
)

# 方案3: 增加对齐点数
system.aligner.min_pairs_required = 5  # 更多点
```

### 问题3: 尺度不一致

**症状：**
- iPDR认为走了1米，COLMAP显示0.8米

**原因：**
- 步长估计不准确
- COLMAP尺度不是米制

**解决：**
```python
# 方案1: 标定步长
# 在已知距离上行走，调整步长系数

# 方案2: 估计尺度
# 使用多点对齐时启用尺度估计
# (当前实现中强制scale=1.0，可修改)

# 在 coordinate_alignment.py 中修改：
# line ~235: scale = 1.0  # 改为 scale = np.sum(S) / var_local
```

---

## 📝 总结

### 关键要点

1. **坐标对齐是必须的**
   - iPDR局部坐标 ≠ COLMAP全局坐标
   - 必须在融合前建立变换关系

2. **三种对齐方法各有优劣**
   - 首次对齐：快速但不够鲁棒
   - 多点对齐：推荐用于生产环境
   - 预标定：最高精度，适合固定环境

3. **对齐在系统启动时进行**
   - 前几秒收集对应点
   - 估计变换参数
   - 后续所有测量使用此变换

4. **变换包含三个部分**
   - 旋转 R (3×3)
   - 平移 t (3×1)
   - 尺度 s (标量，通常=1.0)

### 推荐配置

```python
# 生产环境推荐配置
config = VisualFusionConfig()
config.max_pos_innovation = 3.0
config.max_yaw_innovation = np.deg2rad(10)
config.min_inliers = 10
config.max_reproj_error = 5.0

system = iPDRVisualFusionAligned(
    config=config,
    alignment_mode='multi_point'  # 多点鲁棒对齐
)

system.aligner.min_pairs_required = 3  # 至少3个点
```

---

## 🔗 相关文件

- **`coordinate_alignment.py`**: 坐标对齐核心算法
- **`ipdr_visual_fusion_with_alignment.py`**: 带对齐的融合系统
- **`ipdr_visual_fusion.py`**: 原始融合模块（假设已对齐）
- **`VISUAL_FUSION_README.md`**: 完整文档

---

**版本:** 1.0
**日期:** 2025-10-29
**作者:** Claude
