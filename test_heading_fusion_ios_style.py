"""
iOS航向融合算法验证脚本
====================

模拟iOS环境，验证VisualHeadingFusion.mm的算法正确性
"""

import numpy as np
import matplotlib.pyplot as plt

class HeadingFusionSimulator:
    """
    模拟iOS的VisualHeadingFusion类
    用于验证算法正确性
    """

    def __init__(self):
        # 配置
        self.base_visual_noise = 0.05  # 2.9度
        self.quality_factor = 2.0
        self.mahalanobis_threshold = 10.83
        self.max_innovation = np.pi / 12.0  # 15度
        self.buffer_duration = 5.0
        self.min_alignment_samples = 3

        # 状态
        self.current_heading = 0.0
        self.current_variance = np.pi ** 2  # 高初始不确定度
        self.current_time = 0.0

        # 对齐
        self.is_aligned = False
        self.heading_offset = 0.0
        self.alignment_samples = []

        # 历史缓存
        self.state_buffer = []  # [(time, heading, variance, gyro_rate)]

        # 统计
        self.total_measurements = 0
        self.accepted_measurements = 0
        self.rejected_mahalanobis = 0
        self.rejected_timeout = 0

    def normalize_angle(self, angle):
        """角度归一化到[-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle <= -np.pi:
            angle += 2 * np.pi
        return angle

    def angle_difference(self, a, b):
        """角度差（考虑周期性）"""
        return self.normalize_angle(a - b)

    def predict(self, dt, gyro_rate, process_noise=None):
        """预测步骤（基于陀螺仪）"""
        if process_noise is None:
            process_noise = 0.001 * dt

        # 预测
        self.current_heading = self.normalize_angle(
            self.current_heading + gyro_rate * dt
        )
        self.current_variance += process_noise
        self.current_time += dt

        # 保存到缓存
        self.state_buffer.append({
            'time': self.current_time,
            'heading': self.current_heading,
            'variance': self.current_variance,
            'gyro_rate': gyro_rate
        })

        # 清理旧状态
        self._trim_buffer()

    def add_visual_measurement(self, timestamp, heading_global, quality):
        """添加视觉测量"""
        self.total_measurements += 1

        # 检查超时
        delay = self.current_time - timestamp
        if delay > self.buffer_duration or delay < 0:
            self.rejected_timeout += 1
            return False

        # 如果未对齐，收集样本
        if not self.is_aligned:
            return self._process_alignment_sample(timestamp, heading_global)

        # 已对齐，进行融合
        return self._process_visual_update(timestamp, heading_global, quality)

    def _process_alignment_sample(self, timestamp, heading_global):
        """处理对齐样本"""
        # 找最近的状态
        state = self._find_closest_state(timestamp)
        if state is None:
            return False

        self.alignment_samples.append({
            'heading_local': state['heading'],
            'heading_global': heading_global
        })

        print(f"⏳ Collecting alignment samples: {len(self.alignment_samples)}/{self.min_alignment_samples}")

        if len(self.alignment_samples) >= self.min_alignment_samples:
            self._estimate_heading_offset()
            self.is_aligned = True
            print(f"✓ Heading alignment established!")
            print(f"  Offset: {np.rad2deg(self.heading_offset):.2f}°")
            return True

        return False

    def _estimate_heading_offset(self):
        """估计航向偏移（循环平均）"""
        sum_sin = 0.0
        sum_cos = 0.0

        for sample in self.alignment_samples:
            offset = self.angle_difference(
                sample['heading_global'],
                sample['heading_local']
            )
            sum_sin += np.sin(offset)
            sum_cos += np.cos(offset)

        self.heading_offset = np.arctan2(sum_sin, sum_cos)

    def _process_visual_update(self, timestamp, heading_global, quality):
        """处理视觉更新"""
        # 1. 找历史状态
        historical_state = self._find_closest_state(timestamp)
        if historical_state is None:
            self.rejected_timeout += 1
            return False

        # 2. 转换到局部坐标系
        heading_local = self.normalize_angle(heading_global - self.heading_offset)

        # 3. 计算测量噪声
        R = self._compute_measurement_noise(quality)

        # 4. 卡尔曼更新
        updated_heading = historical_state['heading']
        updated_variance = historical_state['variance']

        accepted = self._kalman_update(
            updated_heading, updated_variance,
            heading_local, R
        )

        if not accepted:
            self.rejected_mahalanobis += 1
            return False

        # 更新成功后的值
        updated_heading_value = updated_heading[0] if isinstance(updated_heading, np.ndarray) else updated_heading
        updated_variance_value = updated_variance[0] if isinstance(updated_variance, np.ndarray) else updated_variance

        # 5. OOSM回放
        self._replay_from_state(
            historical_state,
            updated_heading_value,
            updated_variance_value
        )

        self.accepted_measurements += 1
        return True

    def _kalman_update(self, x, P, z, R):
        """1D卡尔曼更新"""
        # 创新
        y = self.angle_difference(z, x)

        # 限幅
        if abs(y) > self.max_innovation:
            y = np.sign(y) * self.max_innovation

        # 创新协方差
        S = P + R

        # Mahalanobis检验
        mahalanobis = (y ** 2) / S
        if mahalanobis > self.mahalanobis_threshold:
            return False

        # 卡尔曼增益
        K = P / S

        # 更新
        x_updated = self.normalize_angle(x + K * y)
        P_updated = (1.0 - K) * P

        # 返回更新后的值（通过修改传入的变量）
        return True, x_updated, P_updated

    def _replay_from_state(self, historical_state, updated_heading, updated_variance):
        """OOSM回放"""
        heading = updated_heading
        variance = updated_variance
        timestamp = historical_state['time']

        # 回放所有后续状态
        for state in self.state_buffer:
            if state['time'] <= historical_state['time']:
                continue

            dt = state['time'] - timestamp
            heading = self.normalize_angle(heading + state['gyro_rate'] * dt)
            variance += 0.001 * dt

            timestamp = state['time']

        # 更新当前状态
        self.current_heading = heading
        self.current_variance = variance

    def _compute_measurement_noise(self, quality):
        """计算自适应测量噪声"""
        quality_weight = self.quality_factor / max(quality, 0.1)
        R = self.base_visual_noise * quality_weight
        R = max(R, 0.01)  # 最小噪声
        return R ** 2

    def _find_closest_state(self, target_time):
        """找最接近的历史状态"""
        if len(self.state_buffer) == 0:
            return None

        closest = None
        min_diff = float('inf')

        for state in self.state_buffer:
            diff = abs(state['time'] - target_time)
            if diff < min_diff:
                min_diff = diff
                closest = state

        if min_diff > 0.1:  # 100ms容差
            return None

        return closest

    def _trim_buffer(self):
        """清理旧状态"""
        while len(self.state_buffer) > 0:
            if self.current_time - self.state_buffer[0]['time'] > self.buffer_duration:
                self.state_buffer.pop(0)
            else:
                break

    def get_fused_heading(self):
        """获取融合航向"""
        return self.normalize_angle(self.current_heading)

    def get_uncertainty(self):
        """获取不确定度"""
        return np.sqrt(self.current_variance)

    def get_statistics(self):
        """获取统计信息"""
        acceptance_rate = (
            self.accepted_measurements / self.total_measurements
            if self.total_measurements > 0 else 0.0
        )

        return {
            'total_measurements': self.total_measurements,
            'accepted_measurements': self.accepted_measurements,
            'rejected_mahalanobis': self.rejected_mahalanobis,
            'rejected_timeout': self.rejected_timeout,
            'acceptance_rate': acceptance_rate,
            'is_aligned': self.is_aligned,
            'heading_offset_deg': np.rad2deg(self.heading_offset),
            'current_uncertainty_deg': np.rad2deg(self.get_uncertainty())
        }


def simulate_scenario():
    """模拟真实场景"""
    print("=" * 70)
    print("iOS航向融合算法验证")
    print("=" * 70)

    # 创建融合器
    fusion = HeadingFusionSimulator()

    # 设置初始状态
    fusion.current_heading = 0.0  # iPDR认为朝东
    fusion.current_variance = 0.1

    # 真实场景：iPDR朝东(0°)，但实际是朝北(90°)
    true_offset = np.pi / 2.0  # 90度偏移

    # 模拟参数
    dt = 0.01  # 100 Hz IMU
    visual_interval = 1.0  # 1 Hz 视觉
    total_time = 10.0

    # 记录
    time_log = []
    ipdr_log = []
    visual_log = []
    fused_log = []
    uncertainty_log = []

    # 模拟
    print(f"\n⏳ Simulating {total_time} seconds...")
    print(f"   - IMU rate: 100 Hz")
    print(f"   - Visual rate: 1 Hz")
    print(f"   - True heading offset: {np.rad2deg(true_offset):.1f}°")

    t = 0.0
    last_visual_time = 0.0
    step = 0

    while t < total_time:
        # IMU更新（100 Hz）
        gyro_rate = 0.02  # 小幅转动，~1.1°/s
        fusion.predict(dt, gyro_rate)

        # 视觉更新（1 Hz）
        if t - last_visual_time >= visual_interval:
            # 模拟视觉测量（在全局坐标系中）
            ipdr_heading_local = fusion.current_heading
            visual_heading_global = fusion.normalize_angle(
                ipdr_heading_local + true_offset + np.random.randn() * 0.03
            )

            quality = 0.8

            # 添加视觉测量（有100ms延迟）
            fusion.add_visual_measurement(
                timestamp=t - 0.1,
                heading_global=visual_heading_global,
                quality=quality
            )

            last_visual_time = t

        # 记录
        if step % 10 == 0:  # 每10个IMU周期记录一次
            time_log.append(t)
            ipdr_log.append(fusion.current_heading)
            fused_log.append(fusion.get_fused_heading())
            uncertainty_log.append(fusion.get_uncertainty())

            if fusion.is_aligned:
                # 转换回全局坐标系用于可视化
                visual_log.append(fusion.get_fused_heading() + fusion.heading_offset)
            else:
                visual_log.append(np.nan)

        t += dt
        step += 1

    # 统计
    print("\n" + "=" * 70)
    print("Results")
    print("=" * 70)

    stats = fusion.get_statistics()
    print(f"\n📊 Statistics:")
    print(f"   Visual measurements: {stats['total_measurements']}")
    print(f"   Accepted: {stats['accepted_measurements']}")
    print(f"   Acceptance rate: {stats['acceptance_rate']*100:.1f}%")
    print(f"   Heading offset: {stats['heading_offset_deg']:.2f}°")
    print(f"   Final uncertainty: {stats['current_uncertainty_deg']:.2f}°")

    print(f"\n📍 Final State:")
    print(f"   iPDR heading (local): {np.rad2deg(fusion.current_heading):.2f}°")
    print(f"   Fused heading (local): {np.rad2deg(fusion.get_fused_heading()):.2f}°")
    if fusion.is_aligned:
        print(f"   Fused heading (global): {np.rad2deg(fusion.get_fused_heading() + fusion.heading_offset):.2f}°")

    # 可视化
    print("\n📈 Generating plots...")
    visualize_results(time_log, ipdr_log, fused_log, visual_log, uncertainty_log, stats)

    print("\n" + "=" * 70)
    print("✅ Simulation completed!")
    print("=" * 70)


def visualize_results(time_log, ipdr_log, fused_log, visual_log, uncertainty_log, stats):
    """可视化结果"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # 1. 航向对比
    ax = axes[0]
    ax.plot(time_log, np.rad2deg(ipdr_log), 'b-', label='iPDR (Local)', linewidth=2)
    ax.plot(time_log, np.rad2deg(fused_log), 'r-', label='Fused (Local)', linewidth=2)
    ax.plot(time_log, np.rad2deg(visual_log), 'g--', label='Fused (Global)', linewidth=1.5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Heading (deg)')
    ax.set_title('Heading Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 误差
    ax = axes[1]
    error = np.array(fused_log) - np.array(ipdr_log)
    ax.plot(time_log, np.rad2deg(error), 'k-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Correction (deg)')
    ax.set_title('Visual Correction Applied')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)

    # 3. 不确定度
    ax = axes[2]
    ax.plot(time_log, np.rad2deg(uncertainty_log), 'b-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Uncertainty (deg)')
    ax.set_title('Heading Uncertainty')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    # 添加统计文本
    stats_text = f"""
    Statistics:
    • Total measurements: {stats['total_measurements']}
    • Acceptance rate: {stats['acceptance_rate']*100:.1f}%
    • Heading offset: {stats['heading_offset_deg']:.2f}°
    • Final uncertainty: {stats['current_uncertainty_deg']:.2f}°
    """
    fig.text(0.02, 0.02, stats_text, fontsize=10, family='monospace',
             verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.savefig('ios_heading_fusion_test.png', dpi=150, bbox_inches='tight')
    print("   ✓ Plot saved to 'ios_heading_fusion_test.png'")


if __name__ == "__main__":
    simulate_scenario()
