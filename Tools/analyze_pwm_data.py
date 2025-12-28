#!/usr/bin/env python3
"""
FOC开环控制PWM占空比数据分析脚本
分析PWM_A, PWM_B, PWM_C三相数据的有效性
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

class PWMDataAnalyzer:
    """PWM数据分析器"""
    
    def __init__(self, csv_file):
        """
        初始化分析器
        Args:
            csv_file: CSV文件路径
        """
        self.csv_file = Path(csv_file)
        self.df = None
        self.issues = []
        
    def load_data(self):
        """加载CSV数据"""
        try:
            self.df = pd.read_csv(self.csv_file)
            print(f"✓ 成功加载数据: {len(self.df)} 条记录")
            print(f"  列名: {list(self.df.columns)}")
            return True
        except Exception as e:
            print(f"✗ 加载数据失败: {e}")
            return False
    
    def check_pwm_range(self):
        """检查PWM占空比是否在有效范围[0, 1]内"""
        print("\n=== 检查PWM范围 ===")
        pwm_cols = ['debug_PWM_A', 'debug_PWM_B', 'debug_PWM_C']
        
        for col in pwm_cols:
            if col not in self.df.columns:
                self.issues.append(f"缺少列: {col}")
                continue
            
            # 转换为数值类型，将'nan'字符串转换为NaN
            numeric_data = pd.to_numeric(self.df[col], errors='coerce')
            
            # 统计有效值
            valid_data = numeric_data.dropna()
            nan_count = numeric_data.isna().sum()
            total_count = len(numeric_data)
            
            print(f"\n{col}:")
            print(f"  总样本数: {total_count}")
            print(f"  有效样本: {len(valid_data)} ({len(valid_data)/total_count*100:.1f}%)")
            print(f"  NaN样本: {nan_count} ({nan_count/total_count*100:.1f}%)")
            
            if len(valid_data) > 0:
                # 检查范围
                min_val = valid_data.min()
                max_val = valid_data.max()
                mean_val = valid_data.mean()
                std_val = valid_data.std()
                
                print(f"  最小值: {min_val:.6f}")
                print(f"  最大值: {max_val:.6f}")
                print(f"  平均值: {mean_val:.6f}")
                print(f"  标准差: {std_val:.6f}")
                
                # 检查是否超出范围
                out_of_range = valid_data[(valid_data < 0) | (valid_data > 1)]
                if len(out_of_range) > 0:
                    issue = f"{col}: {len(out_of_range)} 个样本超出范围[0,1]"
                    self.issues.append(issue)
                    print(f"  ⚠ {issue}")
                else:
                    print(f"  ✓ 所有有效值均在[0, 1]范围内")
    
    def check_three_phase_balance(self):
        """检查三相PWM的平衡性"""
        print("\n=== 检查三相平衡 ===")
        
        pwm_a = pd.to_numeric(self.df['debug_PWM_A'], errors='coerce')
        pwm_b = pd.to_numeric(self.df['debug_PWM_B'], errors='coerce')
        pwm_c = pd.to_numeric(self.df['debug_PWM_C'], errors='coerce')
        
        # 找到三相都有效的数据点
        valid_mask = ~(pwm_a.isna() | pwm_b.isna() | pwm_c.isna())
        valid_count = valid_mask.sum()
        
        print(f"三相同时有效的样本数: {valid_count}")
        
        if valid_count > 0:
            # 计算三相和（对于SVPWM，理想情况下和应该接近常数）
            pwm_sum = pwm_a[valid_mask] + pwm_b[valid_mask] + pwm_c[valid_mask]
            print(f"三相和统计:")
            print(f"  最小值: {pwm_sum.min():.6f}")
            print(f"  最大值: {pwm_sum.max():.6f}")
            print(f"  平均值: {pwm_sum.mean():.6f}")
            print(f"  标准差: {pwm_sum.std():.6f}")
    
    def check_nan_pattern(self):
        """检查NaN值的分布模式"""
        print("\n=== 检查NaN模式 ===")
        
        pwm_cols = ['debug_PWM_A', 'debug_PWM_B', 'debug_PWM_C']
        
        # 转换所有PWM列为数值
        for col in pwm_cols:
            self.df[col + '_numeric'] = pd.to_numeric(self.df[col], errors='coerce')
        
        # 找到NaN出现的行
        nan_mask = (self.df['debug_PWM_A_numeric'].isna() | 
                    self.df['debug_PWM_B_numeric'].isna() | 
                    self.df['debug_PWM_C_numeric'].isna())
        
        nan_indices = self.df[nan_mask].index.tolist()
        
        if len(nan_indices) > 0:
            print(f"包含NaN的行数: {len(nan_indices)}")
            print(f"NaN首次出现: Index {nan_indices[0]}")
            print(f"NaN最后出现: Index {nan_indices[-1]}")
            
            # 检查是否连续
            if len(nan_indices) > 1:
                consecutive = all(nan_indices[i] + 1 == nan_indices[i+1] 
                                for i in range(len(nan_indices)-1))
                if consecutive:
                    print("  ✓ NaN值连续出现")
                else:
                    print("  ⚠ NaN值非连续出现（可能存在间歇性问题）")
        else:
            print("✓ 无NaN值")
    
    def check_time_consistency(self):
        """检查时间戳的一致性"""
        print("\n=== 检查时间序列 ===")
        
        if 'Time' not in self.df.columns:
            self.issues.append("缺少Time列")
            return
        
        time_data = self.df['Time']
        
        # 计算时间间隔
        time_diff = time_data.diff()
        
        print(f"时间跨度: {time_data.min():.6f}s ~ {time_data.max():.6f}s")
        print(f"总时长: {time_data.max() - time_data.min():.6f}s")
        print(f"平均采样间隔: {time_diff.mean():.6f}s")
        print(f"采样间隔标准差: {time_diff.std():.6f}s")
        print(f"采样频率约: {1/time_diff.mean():.1f} Hz")
        
        # 检查是否有时间倒退
        backward_time = time_diff[time_diff < 0]
        if len(backward_time) > 0:
            issue = f"发现 {len(backward_time)} 处时间倒退"
            self.issues.append(issue)
            print(f"  ⚠ {issue}")
    
    def analyze_pwm_transitions(self):
        """分析PWM变化趋势"""
        print("\n=== 分析PWM变化趋势 ===")
        
        pwm_cols = ['debug_PWM_A', 'debug_PWM_B', 'debug_PWM_C']
        
        for col in pwm_cols:
            numeric_col = col + '_numeric'
            if numeric_col in self.df.columns:
                valid_data = self.df[numeric_col].dropna()
                
                if len(valid_data) > 1:
                    # 计算变化率
                    diff = valid_data.diff()
                    max_jump = diff.abs().max()
                    
                    print(f"\n{col}:")
                    print(f"  最大单步变化: {max_jump:.6f}")
                    
                    # 检查是否有异常跳变
                    if max_jump > 0.5:
                        print(f"  ⚠ 存在大幅度跳变（>0.5）")
    
    def plot_data(self):
        """绘制PWM数据图表"""
        print("\n=== 生成数据可视化 ===")
        
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        # 准备数据
        time = self.df['Time']
        pwm_a = pd.to_numeric(self.df['debug_PWM_A'], errors='coerce')
        pwm_b = pd.to_numeric(self.df['debug_PWM_B'], errors='coerce')
        pwm_c = pd.to_numeric(self.df['debug_PWM_C'], errors='coerce')
        
        # 子图1: 三相PWM
        axes[0].plot(time, pwm_a, 'r-', label='PWM_A', linewidth=1.5, alpha=0.7)
        axes[0].plot(time, pwm_b, 'g-', label='PWM_B', linewidth=1.5, alpha=0.7)
        axes[0].plot(time, pwm_c, 'b-', label='PWM_C', linewidth=1.5, alpha=0.7)
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('PWM Duty Cycle')
        axes[0].set_title('Three-Phase PWM Duty Cycle')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_ylim(-0.1, 1.1)
        
        # 子图2: 三相和
        pwm_sum = pwm_a + pwm_b + pwm_c
        axes[1].plot(time, pwm_sum, 'k-', linewidth=1.5)
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Sum of PWM (A+B+C)')
        axes[1].set_title('Sum of Three-Phase PWM')
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 保存图表
        output_file = self.csv_file.parent / f"{self.csv_file.stem}_analysis.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✓ 图表已保存: {output_file}")
        
        plt.show()
    
    def generate_report(self):
        """生成分析报告"""
        print("\n" + "="*50)
        print("分析报告总结")
        print("="*50)
        
        if len(self.issues) == 0:
            print("✓ 未发现明显问题，数据基本正常")
        else:
            print(f"⚠ 发现 {len(self.issues)} 个问题:")
            for i, issue in enumerate(self.issues, 1):
                print(f"  {i}. {issue}")
        
        print("="*50)
    
    def run_full_analysis(self):
        """运行完整分析流程"""
        print("="*50)
        print("FOC开环控制PWM数据分析")
        print(f"文件: {self.csv_file}")
        print("="*50)
        
        if not self.load_data():
            return False
        
        # 执行各项检查
        self.check_time_consistency()
        self.check_pwm_range()
        self.check_three_phase_balance()
        self.check_nan_pattern()
        self.analyze_pwm_transitions()
        
        # 生成可视化
        try:
            self.plot_data()
        except Exception as e:
            print(f"⚠ 绘图失败: {e}")
        
        # 生成报告
        self.generate_report()
        
        return True


def main():
    """主函数"""
    # 默认CSV文件路径
    csv_file = "Debug/1.csv"
    
    # 如果命令行提供了文件路径，使用命令行参数
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    
    # 创建分析器并运行
    analyzer = PWMDataAnalyzer(csv_file)
    analyzer.run_full_analysis()


if __name__ == "__main__":
    main()
