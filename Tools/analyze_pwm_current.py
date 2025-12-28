#!/usr/bin/env python3
"""
FOC开环控制电压和电流分析
基于PWM占空比和电机参数计算实际电压和预期电流
"""

import warnings
warnings.filterwarnings('ignore')  # 忽略所有警告

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# 配置matplotlib支持中文显示
try:
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK TC', 'WenQuanYi Zen Hei', 'SimHei', 'Arial']
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
except:
    pass  # 如果字体设置失败，使用默认字体（会有方框但不影响功能）

class FOCCurrentAnalyzer:
    """FOC电流分析器"""
    
    def __init__(self, csv_file, motor_params=None):
        """
        初始化分析器
        Args:
            csv_file: CSV文件路径
            motor_params: 电机参数字典
        """
        self.csv_file = Path(csv_file)
        self.df = None
        
        # 默认电机参数（从用户提供的配置）
        self.params = motor_params or {
            'Vbus': 12.0,                    # 总线电压 [V]
            'pole_pairs': 7,                 # 极对数
            'phase_resistance': 0.14,        # 相电阻 [Ω]
            'phase_inductance': 0.000025,    # 相电感 [H]
            'torque_constant': 0.013,        # 转矩常数 [Nm/A]
            'current_lim': 10.0,             # 电流限制 [A]
            'lockin_current': 1.0,           # Lockin电流 [A]
            'lockin_vel': 2.0,               # Lockin速度 [rad/s]
            'lockin_ramp_time': 0.4,         # Ramp时间 [s]
        }
        
    def load_data(self):
        """加载CSV数据"""
        try:
            self.df = pd.read_csv(self.csv_file)
            print(f"✓ 成功加载数据: {len(self.df)} 条记录")
            
            # 转换PWM列为数值
            for col in ['debug_PWM_A', 'debug_PWM_B', 'debug_PWM_C']:
                if col in self.df.columns:
                    self.df[col + '_numeric'] = pd.to_numeric(self.df[col], errors='coerce')
            
            return True
        except Exception as e:
            print(f"✗ 加载数据失败: {e}")
            return False
    
    def calculate_clarke_transform(self):
        """计算Clarke变换（ABC -> αβ）"""
        print("\n=== Clarke变换分析 ===")
        
        # 获取三相PWM（占空比）
        pwm_a = self.df['debug_PWM_A_numeric'].values
        pwm_b = self.df['debug_PWM_B_numeric'].values
        pwm_c = self.df['debug_PWM_C_numeric'].values
        
        # Clarke变换系数（功率守恒）
        # Vα = (2/3) * [Va - 0.5*Vb - 0.5*Vc]
        # Vβ = (2/3) * [√3/2*Vb - √3/2*Vc]
        sqrt3 = np.sqrt(3)
        
        # 首先将占空比转换为相对于中点的电压
        # PWM占空比0.5对应0V相电压
        Va = (pwm_a - 0.5) * self.params['Vbus']
        Vb = (pwm_b - 0.5) * self.params['Vbus']
        Vc = (pwm_c - 0.5) * self.params['Vbus']
        
        # Clarke变换
        V_alpha = (2.0/3.0) * (Va - 0.5*Vb - 0.5*Vc)
        V_beta = (2.0/3.0) * (sqrt3/2.0*Vb - sqrt3/2.0*Vc)
        
        # 计算电压幅值
        V_magnitude = np.sqrt(V_alpha**2 + V_beta**2)
        
        self.df['V_alpha'] = V_alpha
        self.df['V_beta'] = V_beta
        self.df['V_magnitude'] = V_magnitude
        
        # 统计有效数据
        valid_mask = ~np.isnan(V_magnitude)
        valid_V = V_magnitude[valid_mask]
        
        if len(valid_V) > 0:
            print(f"电压幅值统计（基于Clarke变换）:")
            print(f"  最小值: {valid_V.min():.4f} V")
            print(f"  最大值: {valid_V.max():.4f} V")
            print(f"  平均值: {valid_V.mean():.4f} V")
            print(f"  标准差: {valid_V.std():.4f} V")
            
            return valid_V.mean()
        return 0
    
    def estimate_current(self, V_avg):
        """估算稳态电流"""
        print("\n=== 电流估算 ===")
        
        R = self.params['phase_resistance']
        L = self.params['phase_inductance']
        vel = self.params['lockin_vel']
        
        print(f"\n电机参数:")
        print(f"  相电阻 R = {R} Ω")
        print(f"  相电感 L = {L*1000:.3f} mH")
        print(f"  角速度 ω ≈ {vel} rad/s")
        
        print(f"\n施加电压:")
        print(f"  总线电压 Vbus = {self.params['Vbus']} V")
        print(f"  平均电压幅值 V = {V_avg:.4f} V")
        
        # 方法1: 纯电阻估算（忽略电感和反电动势）
        I_resistive = V_avg / R
        print(f"\n方法1 - 纯电阻模型 (I = V/R):")
        print(f"  估算电流: {I_resistive:.3f} A")
        if I_resistive > self.params['current_lim']:
            print(f"  ⚠ 超过电流限制 {self.params['current_lim']} A!")
        
        # 方法2: 考虑电感（ω·L项）
        Z_inductance = vel * L  # 感抗
        Z_total = np.sqrt(R**2 + Z_inductance**2)
        I_impedance = V_avg / Z_total
        print(f"\n方法2 - R-L阻抗模型 (Z = √(R² + (ωL)²)):")
        print(f"  感抗 ωL = {Z_inductance:.6f} Ω")
        print(f"  总阻抗 Z = {Z_total:.4f} Ω")
        print(f"  估算电流: {I_impedance:.3f} A")
        
        # 方法3: 从目标电流反推需要的电压
        I_target = self.params['lockin_current']
        V_required = I_target * R  # 稳态只需克服电阻
        V_with_inductance = I_target * np.sqrt(R**2 + Z_inductance**2)
        
        print(f"\n方法3 - 从目标电流反推:")
        print(f"  目标电流: {I_target} A")
        print(f"  理论需要电压 (仅R): {V_required:.4f} V")
        print(f"  理论需要电压 (R+ωL): {V_with_inductance:.4f} V")
        print(f"  实际施加电压: {V_avg:.4f} V")
        print(f"  电压余量: {V_avg - V_required:.4f} V")
        
        # 安全性分析
        print(f"\n=== 安全性分析 ===")
        worst_case_current = self.params['Vbus'] / R  # 最坏情况：满占空比
        print(f"最坏情况电流 (100%占空比): {worst_case_current:.3f} A")
        
        if worst_case_current > self.params['current_lim']:
            print(f"⚠ 警告: 最坏情况下会超过电流限制!")
            safety_margin = (self.params['current_lim'] - I_resistive) / self.params['current_lim'] * 100
            print(f"当前安全裕度: {safety_margin:.1f}%")
        else:
            print(f"✓ 在电流限制范围内")
        
        return I_resistive
    
    def analyze_pwm_voltage_relationship(self):
        """分析PWM占空比与电压的关系"""
        print("\n=== PWM占空比分析 ===")
        
        # 获取有效PWM数据
        pwm_a = self.df['debug_PWM_A_numeric'].dropna()
        pwm_b = self.df['debug_PWM_B_numeric'].dropna()
        pwm_c = self.df['debug_PWM_C_numeric'].dropna()
        
        print(f"\n占空比分布:")
        for name, pwm_data in [('PWM_A', pwm_a), ('PWM_B', pwm_b), ('PWM_C', pwm_c)]:
            if len(pwm_data) > 0:
                print(f"\n{name}:")
                print(f"  范围: [{pwm_data.min():.4f}, {pwm_data.max():.4f}]")
                print(f"  平均: {pwm_data.mean():.4f}")
                
                # 计算对应的相电压（相对于中点）
                V_phase = (pwm_data.mean() - 0.5) * self.params['Vbus']
                print(f"  平均相电压（相对中点）: {V_phase:.4f} V")
        
        # 三相和分析
        pwm_sum = (self.df['debug_PWM_A_numeric'] + 
                   self.df['debug_PWM_B_numeric'] + 
                   self.df['debug_PWM_C_numeric']).dropna()
        
        if len(pwm_sum) > 0:
            print(f"\n三相占空比和:")
            print(f"  平均值: {pwm_sum.mean():.4f}")
            print(f"  理论值（SVPWM）: 1.5")
            print(f"  偏差: {abs(pwm_sum.mean() - 1.5):.4f}")
            
            if abs(pwm_sum.mean() - 1.5) < 0.1:
                print(f"  ✓ 符合SVPWM特性")
            else:
                print(f"  ⚠ 偏离SVPWM理论值")
    
    def plot_analysis(self):
        """绘制分析图表"""
        print("\n=== 生成可视化 ===")
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        time = self.df['Time']
        
        # 子图1: 三相PWM占空比
        axes[0].plot(time, self.df['debug_PWM_A_numeric'], 'r-', 
                    label='PWM_A', linewidth=1, alpha=0.7)
        axes[0].plot(time, self.df['debug_PWM_B_numeric'], 'g-', 
                    label='PWM_B', linewidth=1, alpha=0.7)
        axes[0].plot(time, self.df['debug_PWM_C_numeric'], 'b-', 
                    label='PWM_C', linewidth=1, alpha=0.7)
        axes[0].axhline(y=0.5, color='k', linestyle='--', alpha=0.3, label='Midpoint (0.5)')
        axes[0].set_ylabel('PWM Duty Cycle')
        axes[0].set_title('Three-Phase PWM Duty Cycle')
        axes[0].legend(loc='upper right')
        axes[0].grid(True, alpha=0.3)
        axes[0].set_ylim(-0.05, 1.05)
        
        # 子图2: αβ电压
        axes[1].plot(time, self.df['V_alpha'], 'r-', label='V_alpha', linewidth=1, alpha=0.7)
        axes[1].plot(time, self.df['V_beta'], 'b-', label='V_beta', linewidth=1, alpha=0.7)
        axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
        axes[1].set_ylabel('Voltage (V)')
        axes[1].set_title('Clarke Transform: Alpha-Beta Voltage')
        axes[1].legend(loc='upper right')
        axes[1].grid(True, alpha=0.3)
        
        # 子图3: 电压幅值
        axes[2].plot(time, self.df['V_magnitude'], 'purple', linewidth=1.5, alpha=0.8)
        
        # 添加参考线
        V_target = self.params['lockin_current'] * self.params['phase_resistance']
        axes[2].axhline(y=V_target, color='green', linestyle='--', 
                       label=f'Target Voltage (I*R = {V_target:.3f}V)', linewidth=2)
        axes[2].axhline(y=self.params['Vbus'], color='red', linestyle='--', 
                       label=f'Bus Voltage ({self.params["Vbus"]}V)', linewidth=2, alpha=0.5)
        
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Voltage Magnitude (V)')
        axes[2].set_title('Voltage Magnitude |V| = sqrt(V_alpha^2 + V_beta^2)')
        axes[2].legend(loc='upper right')
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 保存图表
        output_file = self.csv_file.parent / f"{self.csv_file.stem}_current_analysis.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✓ 图表已保存: {output_file}")
        
        # 不显示交互式窗口，避免字体警告
        # plt.show()
    
    def run_analysis(self):
        """运行完整分析"""
        print("="*60)
        print("FOC开环控制电压与电流分析")
        print(f"文件: {self.csv_file}")
        print("="*60)
        
        if not self.load_data():
            return False
        
        # PWM占空比分析
        self.analyze_pwm_voltage_relationship()
        
        # Clarke变换计算电压
        V_avg = self.calculate_clarke_transform()
        
        # 电流估算
        self.estimate_current(V_avg)
        
        # 绘图
        try:
            self.plot_analysis()
        except Exception as e:
            print(f"⚠ 绘图失败: {e}")
        
        print("\n" + "="*60)
        print("分析完成")
        print("="*60)
        
        return True


def main():
    """主函数"""
    # 默认CSV文件路径
    csv_file = "Debug/1.csv"
    
    # 如果命令行提供了文件路径，使用命令行参数
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    # 电机参数（从用户提供的配置）
    motor_params = {
        'Vbus': 12.0,                    # 总线电压 [V]
        'pole_pairs': 7,                 # 极对数
        'phase_resistance': 0.14,        # 相电阻 [Ω]
        'phase_inductance': 0.000025,    # 相电感 [H] (25μH)
        'torque_constant': 0.013,        # 转矩常数 [Nm/A]
        'current_lim': 10.0,             # 电流限制 [A]
        'lockin_current': 1.0,           # Lockin目标电流 [A]
        'lockin_vel': 2.0,               # Lockin速度 [rad/s]
        'lockin_ramp_time': 0.4,         # Ramp时间 [s]
    }
    
    # 创建分析器并运行
    analyzer = FOCCurrentAnalyzer(csv_file, motor_params)
    analyzer.run_analysis()


if __name__ == "__main__":
    main()
