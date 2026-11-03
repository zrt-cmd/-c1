#!/usr/bin/env python3
"""
机械臂可达工作空间扫描工具

复用 arm_controller.py 的 IKSolver 和关节限位，扫描 x-z 平面在不同 pitch 下的可达区域。
输出 ASCII 表格（直观）和 CSV 文件（供选点用）。

用法：
    python3 scan_workspace.py

输出：
    - 控制台：4 张表格（pitch=0°, 30°, 60°, 90°）
    - workspace.csv：所有可达点详细数据
"""

import math
import sys
import csv
from pathlib import Path


# 复用 arm_controller.py 的限位定义
JOINT_LIMITS_RAD = {
    'arm_joint1': (-1.047, 1.047),      # ±60°
    'arm_joint2': (-0.628, 1.466),       # -36° ~ +84°
    'arm_joint3': (-1.047, 1.047),       # ±60°
    'arm_joint4': (-1.466, 1.466),       # ±84°
    'arm_joint5': (-0.838, 0.838),       # ±48°
}


class IKSolver:
    """Analytical IK solver (复用 arm_controller.py L34-86)"""

    def __init__(self, d1, a2, a3, a4):
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4

    def solve(self, x, y, z, pitch=0.0):
        """
        Args:
            x, y, z: target position in base_link frame (meters)
            pitch: end-effector pitch angle (rad), 0=horizontal, >0=down
        Returns:
            List of solutions [[j1,j2,j3,j4,j5]] in URDF rad, or None
        """
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        pz = z - self.d1

        wr = r - self.a4 * math.cos(pitch)
        wz = pz - self.a4 * math.sin(pitch)

        D = wr**2 + wz**2
        max_reach = self.a2 + self.a3
        if math.sqrt(D) > max_reach * 0.98:
            return None
        if math.sqrt(D) < abs(self.a2 - self.a3) * 1.05:
            return None

        cos_theta3 = (D - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
        if abs(cos_theta3) > 1.0:
            return None

        theta3 = math.acos(cos_theta3)
        theta2 = math.atan2(wz, wr) - math.atan2(
            self.a3 * math.sin(theta3), self.a2 + self.a3 * math.cos(theta3))
        theta4 = pitch - (theta2 + theta3)

        theta2_urdf = theta2 + math.pi / 2
        theta3_urdf = theta3
        theta4_urdf = theta4

        return [[theta1, theta2_urdf, theta3_urdf, theta4_urdf, 0.0]]


def check_limits(joints):
    """检查关节角是否在安全限位内"""
    names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
    for i, name in enumerate(names):
        lo, hi = JOINT_LIMITS_RAD[name]
        if joints[i] < lo or joints[i] > hi:
            return False
    return True


def scan_workspace():
    """扫描可达工作空间"""
    # IK 参数（与 arm_controller.py 保持一致）
    ik = IKSolver(d1=0.1005, a2=0.12614, a3=0.12614, a4=0.151)

    # 扫描范围
    x_range = [round(x, 3) for x in [i * 0.02 for i in range(5, 16)]]  # 0.10~0.30, 步长0.02
    z_range = [round(z, 3) for z in [i * 0.02 for i in range(0, 16)]]  # 0.00~0.30, 步长0.02
    pitch_values = [0, math.pi/6, math.pi/3, math.pi/2]  # 0°, 30°, 60°, 90°
    y = 0.0  # 先做 x-z 平面

    results = []

    print("开始扫描可达工作空间...\n")
    print(f"扫描范围：x ∈ [{x_range[0]:.2f}, {x_range[-1]:.2f}], z ∈ [{z_range[0]:.2f}, {z_range[-1]:.2f}]")
    print(f"pitch 角度：{[f'{math.degrees(p):.0f}°' for p in pitch_values]}\n")

    for pitch in pitch_values:
        pitch_deg = math.degrees(pitch)
        print(f"{'='*80}")
        print(f"Pitch = {pitch_deg:.0f}° (末端{'水平' if pitch==0 else '向下倾斜' if pitch<math.pi/2 else '竖直向下'})")
        print(f"{'='*80}\n")

        # 构建 ASCII 表格
        table = {}
        for z in z_range:
            table[z] = {}
            for x in x_range:
                solutions = ik.solve(x, y, z, pitch)
                if solutions and check_limits(solutions[0]):
                    table[z][x] = '✓'
                    results.append({
                        'x': x, 'y': y, 'z': z, 'pitch': pitch, 'pitch_deg': pitch_deg,
                        'j1': solutions[0][0], 'j2': solutions[0][1], 'j3': solutions[0][2],
                        'j4': solutions[0][3], 'j5': solutions[0][4]
                    })
                else:
                    table[z][x] = '✗'

        # 打印表头
        print("    z\\x ", end="")
        for x in x_range:
            print(f"{x:>5.2f}", end=" ")
        print()
        print("    " + "-" * (6 * len(x_range) + 5))

        # 打印表格（z 从高到低）
        for z in reversed(z_range):
            print(f"  {z:>5.2f} |", end="")
            for x in x_range:
                symbol = table[z][x]
                print(f"  {symbol}  ", end="")
            print()
        print()

    # 保存 CSV
    csv_path = Path(__file__).parent / 'workspace.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['x', 'y', 'z', 'pitch_deg', 'j1', 'j2', 'j3', 'j4', 'j5'])
        writer.writeheader()
        for row in results:
            writer.writerow({
                'x': row['x'], 'y': row['y'], 'z': row['z'], 'pitch_deg': row['pitch_deg'],
                'j1': f"{row['j1']:.4f}", 'j2': f"{row['j2']:.4f}", 'j3': f"{row['j3']:.4f}",
                'j4': f"{row['j4']:.4f}", 'j5': f"{row['j5']:.4f}"
            })

    print(f"{'='*80}")
    print(f"扫描完成！共找到 {len(results)} 个可达点")
    print(f"详细数据已保存至：{csv_path}")
    print(f"{'='*80}\n")

    # 统计
    print("统计信息：")
    for pitch in pitch_values:
        count = sum(1 for r in results if r['pitch'] == pitch)
        print(f"  pitch={math.degrees(pitch):>3.0f}°: {count:>3} 个可达点")


if __name__ == '__main__':
    scan_workspace()
