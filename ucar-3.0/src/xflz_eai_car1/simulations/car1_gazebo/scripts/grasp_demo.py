#!/usr/bin/env python3
"""
固定位置抓取演示节点
使用解析法IK求解，控制机械臂完成 pick-and-place 流程。

坐标系说明：
- 简化FK模型基于 base_footprint 坐标系
- 零位时机械臂竖直向上
- 角度从竖直方向计量（不是从水平方向）
- end effector pitch: 0=向上, π/2=水平向前, π=向下(抓取用)
"""
import math
import threading
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener


class GraspDemo(Node):
    def __init__(self):
        super().__init__('grasp_demo')

        # 简化FK模型参数（根据手动参考角度校准）
        # 手动角度 [0, -1.05, -1.05, -1.2, 0] 能抓取到 (0.25, 0, 0.01)
        # 反推得到校准后的基座偏移
        self.d_h = 0.00381    # 校准值（原 0.05681 - 0.053）
        self.d_v = 0.10097    # 校准值（原 0.15997 - 0.059）
        # 连杆长度
        self.L2 = 0.12614     # 大臂
        self.L3 = 0.12614     # 小臂
        self.L4 = 0.074395    # 腕部到 joint5

        # 夹爪中心相对于 arm_link5 的局部坐标偏移
        # arm_joint6: xyz=(0.003, 0, 0.007) from arm_link5
        # 夹爪内指关节中心: xyz=(0.039, 0, 0.003) from arm_link6
        # 合计: (0.042, 0, 0.01) from arm_link5
        self.gripper_center_local = (0.042, 0.0, 0.01)

        # arm_joint5 的 rpy 旋转矩阵（固定值）
        # rpy=(-π/2, 0, -π/2) → R5_rpy
        # arm_joint2 的 rpy 旋转矩阵
        # rpy=(-π/2, -π/2, 0) → T2_rpy
        # 这些用于计算夹爪在世界坐标系中的实际偏移

        # 关节限位
        self.joint_min = -2.356
        self.joint_max = 2.356

        # 目标参数
        # 立方体 spawn 在 world (0.25, 0, 0.02)，4cm cube 中心在 z=0.02
        # 抓取目标 = cube 中心
        self.cube_world = (0.25, 0.0, 0.02)       # cube 中心位置
        self.place_world = (0.20, 0.10, 0.02)     # 放置位置
        self.pre_grasp_height = 0.05              # pre-grasp 高于抓取点的距离
        self.grasp_pitch_deg = 189.0              # 抓取时末端倾斜角（189°=略过垂直向下，与手动参考一致）

        # 夹爪参数
        self.gripper_open = 0.8
        self.gripper_close = 0.0

        # 机器人 spawn 在 z=0.1，但会在重力下落到地面
        # base_footprint 最终在 world z≈0（地面接触点）
        # 实际偏移通过 TF 获取，这里用近似值
        self.robot_spawn_z = 0.0

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.get_logger().info('抓取演示节点已启动，等待 TF...')

        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 用独立线程执行抓取序列
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        """在独立线程中执行抓取序列"""
        time.sleep(5.0)  # 等待控制器就绪和机器人落地

        # 通过 TF 获取 arm_base_link 的实际位置来验证坐标系
        self._verify_tf()

        time.sleep(1.0)
        self.execute_grasp_sequence()

    def _verify_tf(self):
        """通过 TF 验证坐标系"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_footprint', 'arm_base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            self.get_logger().info(
                f'TF验证: arm_base_link in base_footprint = ({x:.4f}, {y:.4f}, {z:.4f})')
            self.get_logger().info(
                f'  预期值: (0.0466, 0, 0.057), d_v应为 z+0.0435+0.0595={z+0.0435+0.059467:.4f}')
        except Exception as e:
            self.get_logger().warn(f'TF 查询失败: {e}')

    # ========== 正运动学（验证用） ==========

    def compute_fk(self, t1, t2, t3, t4):
        """简化FK：返回末端在 base_footprint 中的 (x, y, z)"""
        L2, L3, L4 = self.L2, self.L3, self.L4
        # 各段从竖直方向的累积角度
        # phi_i = -(sum of joint angles up to i)
        r = L2 * math.sin(-t2) + L3 * math.sin(-(t2+t3)) + L4 * math.sin(-(t2+t3+t4))
        z_arm = L2 * math.cos(-t2) + L3 * math.cos(-(t2+t3)) + L4 * math.cos(-(t2+t3+t4))

        x = (self.d_h + r) * math.cos(-t1)
        y = (self.d_h + r) * math.sin(-t1)
        z = self.d_v + z_arm
        return (x, y, z)

    # ========== 逆运动学 ==========

    def _gripper_offset_in_plane(self, pitch):
        """计算夹爪中心相对于 joint5 在臂平面内的偏移 (dr, dz)

        由于 arm_joint5 的 rpy 旋转，夹爪中心不在 joint5 正前方，
        而是有一个依赖于 pitch 角度的偏移。

        通过旋转矩阵链计算：
        R4_plane @ R5_rpy @ gripper_center_local → (dx_world, dz_world)
        """
        # T2_rpy 将臂平面建立起来，之后 Rz(pitch) 是累积旋转
        # R4 in base_footprint (t1=0): T2_rpy @ Rz(pitch)
        # R5 = R4 @ R5_rpy
        # offset_world = R5 @ gripper_center_local
        #
        # 简化计算（预计算 T2_rpy 和 R5_rpy 的乘积效果）：
        # T2_rpy = Ry(-π/2) @ Rx(-π/2), R5_rpy = Rz(-π/2) @ Rx(-π/2)
        # 经过矩阵运算，偏移在臂平面 (x, z) 中为：
        gx, gy, gz = self.gripper_center_local
        sp, cp = math.sin(pitch), math.cos(pitch)
        # 从完整旋转链推导的解析公式：
        # offset_r = gx * cos(pitch) + gz * sin(pitch)  [水平方向]
        # offset_z = -gx * sin(pitch) + gz * cos(pitch) [垂直方向]
        # 但实际上需要考虑 R5_rpy 的效果，直接用数值验证的结果：
        # pitch=135° → offset=(0.037, 0.023)
        # 这等价于：offset_r = gx*sp + gz*cp, offset_z = gx*cp - gz*sp
        # 验证: 0.042*sin(135°) + 0.01*cos(135°) = 0.0297-0.0071 = 0.0226 ≠ 0.037
        # 不对，需要用精确公式

        # 精确计算（使用三角函数展开旋转矩阵链的结果）
        # 从数值验证得到的规律：
        # offset_r ≈ gx * sin(pitch) - gz * cos(pitch)  (不完全匹配)
        # 直接用查表/插值更可靠，但这里用精确的矩阵公式：
        # R4 @ R5_rpy @ [gx, 0, gz]^T 在 (x,z) 平面的投影
        # 经过完整推导（T2_rpy @ Rz(pitch) @ R5_rpy）：
        # 最终 x 分量 = gz*sp + gx*cp  (验证: 0.01*0.707+0.042*0.707=0.0368 ✓)
        # 最终 z 分量 = gz*cp - gx*sp  (验证: 0.01*0.707-0.042*(-0.707)... 不对)
        #
        # 从数值结果反推：
        # pitch=135(sp=0.707,cp=-0.707): dr=0.037, dz=0.023
        # pitch=110(sp=0.940,cp=-0.342): dr=0.024, dz=0.036
        # pitch=150(sp=0.500,cp=-0.866): dr=0.041, dz=0.012
        #
        # dr = gx*cp + gz*sp 不对... 让我直接用数值方法

        # 使用精确的旋转矩阵（内联计算，避免 numpy 依赖）
        # T2_rpy = Ry(-π/2) @ Rx(-π/2):
        #   [[0, 0, -1], [0, 1, 0], [1, 0, 0]] @ [[1,0,0],[0,0,1],[0,-1,0]]
        #   = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]  -- 不对，让我重算
        # Ry(-π/2) = [[0,0,-1],[0,1,0],[1,0,0]]
        # Rx(-π/2) = [[1,0,0],[0,0,1],[0,-1,0]]
        # T2_rpy = Rz(0)@Ry(-π/2)@Rx(-π/2) = Ry(-π/2)@Rx(-π/2)
        #        = [[0,0,-1],[0,1,0],[1,0,0]] @ [[1,0,0],[0,0,1],[0,-1,0]]
        #        = [[0,1,0],[0,0,1],[1,0,0]]
        # Rz(pitch) = [[cp,-sp,0],[sp,cp,0],[0,0,1]]
        # T2_rpy @ Rz(pitch) = [[sp,cp,0],[0,0,1],[cp,-sp,0]]  -- 不对
        # 让我仔细算：
        # [[0,1,0],[0,0,1],[1,0,0]] @ [[cp,-sp,0],[sp,cp,0],[0,0,1]]
        # row0: [0*cp+1*sp+0*0, 0*(-sp)+1*cp+0*0, 0+0+0] = [sp, cp, 0]
        # row1: [0+0+0, 0+0+0, 0+0+1] = [0, 0, 1]
        # row2: [cp+0+0, -sp+0+0, 0+0+0] = [cp, -sp, 0]
        # So R4 = [[sp,cp,0],[0,0,1],[cp,-sp,0]]

        # R5_rpy = Rz(-π/2) @ Rx(-π/2)
        # Rz(-π/2) = [[0,1,0],[-1,0,0],[0,0,1]]
        # Rx(-π/2) = [[1,0,0],[0,0,1],[0,-1,0]]
        # R5_rpy = [[0,1,0],[-1,0,0],[0,0,1]] @ [[1,0,0],[0,0,1],[0,-1,0]]
        #        = [[0,0,1],[-1,0,0],[0,-1,0]]

        # R5 = R4 @ R5_rpy = [[sp,cp,0],[0,0,1],[cp,-sp,0]] @ [[0,0,1],[-1,0,0],[0,-1,0]]
        # row0: [sp*0+cp*(-1)+0, sp*0+cp*0+0*(-1), sp*1+cp*0+0*0] = [-cp, 0, sp]
        # row1: [0+0+0, 0+0-1, 0+0+0] = [0, -1, 0]
        # row2: [cp*0+(-sp)*(-1)+0, 0+0+0, cp*1+(-sp)*0+0] = [sp, 0, cp]
        # R5 = [[-cp, 0, sp], [0, -1, 0], [sp, 0, cp]]

        # offset = R5 @ [gx, 0, gz] = [-cp*gx + sp*gz, 0, sp*gx + cp*gz]
        dr = -cp * gx + sp * gz   # x component in base_footprint
        dz = sp * gx + cp * gz    # z component in base_footprint
        return (dr, dz)

    def compute_ik(self, target_bf, pitch_deg=None):
        """
        解析法IK求解
        target_bf: (x, y, z) 夹爪抓取中心的目标位置，base_footprint 坐标系
        pitch_deg: 末端倾斜角（从竖直方向计，135=向下倾斜45°）
        返回: [j1, j2, j3, j4, j5] 或 None
        """
        if pitch_deg is None:
            pitch_deg = self.grasp_pitch_deg

        x, y, z = target_bf
        pitch = math.radians(pitch_deg)

        # 计算夹爪中心相对于 joint5 的偏移
        grip_dr, grip_dz = self._gripper_offset_in_plane(pitch)

        # Joint1: 底座旋转（轴方向为 -Z，所以取负）
        theta1 = -math.atan2(y, x)

        # 投影到垂直平面
        # IK 求解 joint5 位置 = 目标位置 - 夹爪偏移
        r_target = math.sqrt(x**2 + y**2)
        r_j5 = r_target - grip_dr  # joint5 的水平距离
        z_j5 = z - grip_dz         # joint5 的高度

        r = r_j5 - self.d_h
        z_arm = z_j5 - self.d_v

        # 腕部位置（减去 L4 在 pitch 方向的贡献）
        r_w = r - self.L4 * math.sin(pitch)
        z_w = z_arm - self.L4 * math.cos(pitch)

        dist = math.sqrt(r_w**2 + z_w**2)
        max_reach = self.L2 + self.L3

        if dist > max_reach * 0.99:
            self.get_logger().error(
                f'目标超出范围: dist={dist:.4f}, max={max_reach:.4f}')
            return None
        if dist < abs(self.L2 - self.L3) * 1.1:
            self.get_logger().error(f'目标太近: dist={dist:.4f}')
            return None

        # 余弦定理求肘关节角
        cos_q3 = (dist**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3_local = math.acos(cos_q3)  # 肘上解（肘下解超出关节限位）

        # 求 phi2（link2 从竖直方向的角度）
        alpha = math.atan2(r_w, z_w)
        beta = math.atan2(
            self.L3 * math.sin(q3_local),
            self.L2 + self.L3 * math.cos(q3_local))
        phi2 = alpha - beta

        # 转换为关节角
        theta2 = -phi2
        theta3 = -q3_local
        theta4 = -pitch - theta2 - theta3

        # 检查关节限位
        for name, val in [('j2', theta2), ('j3', theta3), ('j4', theta4)]:
            if val < self.joint_min or val > self.joint_max:
                self.get_logger().error(
                    f'关节 {name} 超限: {math.degrees(val):.1f}° '
                    f'(限位: [{math.degrees(self.joint_min):.0f}°, {math.degrees(self.joint_max):.0f}°])')
                return None

        theta5 = 0.0
        joints = [theta1, theta2, theta3, theta4, theta5]

        # FK 验证：joint5 位置 + 夹爪偏移 = 抓取点
        fk_j5 = self.compute_fk(theta1, theta2, theta3, theta4)
        # 抓取点 = joint5 + gripper offset (在水平面旋转 theta1)
        fk_grip_x = fk_j5[0] + grip_dr * math.cos(-theta1)
        fk_grip_y = fk_j5[1] + grip_dr * math.sin(-theta1)
        fk_grip_z = fk_j5[2] + grip_dz
        fk_grip = (fk_grip_x, fk_grip_y, fk_grip_z)
        error = math.sqrt(sum((a-b)**2 for a, b in zip(fk_grip, target_bf)))
        self.get_logger().info(
            f'IK解: [{", ".join(f"{j:.3f}" for j in joints)}], '
            f'FK验证误差: {error:.5f}m, pitch={pitch_deg:.0f}°, '
            f'j5=({fk_j5[0]:.3f},{fk_j5[2]:.3f}), grip=({fk_grip_x:.3f},{fk_grip_z:.3f})')

        if error > 0.01:
            self.get_logger().warn(f'IK误差较大: {error:.4f}m')

        return joints

    # ========== 运动控制 ==========

    def send_arm_command(self, positions, duration_sec=2.0):
        """发送机械臂关节命令"""
        msg = JointTrajectory()
        msg.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5'
        ]
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        msg.points = [point]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'  机械臂: {[f"{p:.3f}" for p in positions]}')

    def send_gripper_command(self, position, duration_sec=1.0):
        """发送夹爪命令"""
        msg = JointTrajectory()
        msg.joint_names = ['r_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        msg.points = [point]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'  夹爪: {position:.3f}')

    # ========== 坐标转换 ==========

    def world_to_base_footprint(self, world_pos):
        """世界坐标 → base_footprint 坐标"""
        return (
            world_pos[0],
            world_pos[1],
            world_pos[2] - self.robot_spawn_z
        )

    # ========== 抓取序列 ==========

    def execute_grasp_sequence(self):
        """执行完整的 pick-and-place 序列"""
        self.get_logger().info('===== 开始抓取演示 =====')
        self.get_logger().info(
            f'  物体位置(world): {self.cube_world}')
        self.get_logger().info(
            f'  放置位置(world): {self.place_world}')
        self.get_logger().info(
            f'  抓取倾角: {self.grasp_pitch_deg}° (从竖直, 135°=向下倾斜45°)')

        # Step 1: 回到 home
        self.get_logger().info('[Step 1] Home 位置')
        self.send_arm_command([0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(3.0)

        # Step 2: 张开夹爪
        self.get_logger().info('[Step 2] 张开夹爪')
        self.send_gripper_command(self.gripper_open)
        time.sleep(1.5)

        # Step 3: pre-grasp（物体上方）
        pre_grasp_world = (
            self.cube_world[0],
            self.cube_world[1],
            self.cube_world[2] + self.pre_grasp_height
        )
        pre_grasp_bf = self.world_to_base_footprint(pre_grasp_world)
        self.get_logger().info(
            f'[Step 3] Pre-grasp: world={pre_grasp_world}, bf={pre_grasp_bf}')

        # pre-grasp 用较大的 pitch（接近垂直向下），方便接近
        joints = self.compute_ik(pre_grasp_bf, pitch_deg=170)
        if joints is None:
            self.get_logger().error('pre-grasp IK 失败')
            return
        self.send_arm_command(joints)
        time.sleep(3.0)

        # Step 4: 下降到抓取位置（夹爪偏移已在 IK 中补偿）
        grasp_bf = self.world_to_base_footprint(self.cube_world)
        self.get_logger().info(
            f'[Step 4] 抓取位置: world={self.cube_world}, bf={grasp_bf}')

        joints = self.compute_ik(grasp_bf)
        if joints is None:
            self.get_logger().error('grasp IK 失败')
            return
        self.send_arm_command(joints)
        time.sleep(3.0)

        # Step 5: 闭合夹爪
        self.get_logger().info('[Step 5] 闭合夹爪')
        self.send_gripper_command(self.gripper_close)
        time.sleep(2.0)

        # Step 6: 提升
        lift_world = (
            self.cube_world[0],
            self.cube_world[1],
            self.cube_world[2] + self.pre_grasp_height
        )
        lift_bf = self.world_to_base_footprint(lift_world)
        self.get_logger().info(f'[Step 6] 提升')

        joints = self.compute_ik(lift_bf, pitch_deg=170)
        if joints is None:
            self.get_logger().error('lift IK 失败')
            return
        self.send_arm_command(joints)
        time.sleep(3.0)

        # Step 7: 移动到放置位置上方
        place_above_world = (
            self.place_world[0],
            self.place_world[1],
            self.place_world[2] + self.pre_grasp_height
        )
        place_bf = self.world_to_base_footprint(place_above_world)
        self.get_logger().info(f'[Step 7] 放置位置上方: bf={place_bf}')

        joints = self.compute_ik(place_bf, pitch_deg=170)
        if joints is None:
            self.get_logger().error('place-above IK 失败')
            return
        self.send_arm_command(joints)
        time.sleep(3.0)

        # Step 8: 下降到放置高度
        place_bf = self.world_to_base_footprint(self.place_world)
        self.get_logger().info(f'[Step 8] 放置高度: bf={place_bf}')

        joints = self.compute_ik(place_bf)
        if joints is None:
            self.get_logger().error('place IK 失败')
            return
        self.send_arm_command(joints)
        time.sleep(3.0)

        # Step 9: 释放
        self.get_logger().info('[Step 9] 释放物体')
        self.send_gripper_command(self.gripper_open)
        time.sleep(2.0)

        # Step 10: 回到 home
        self.get_logger().info('[Step 10] 回到 Home')
        self.send_arm_command([0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(3.0)

        self.get_logger().info('===== 抓取演示完成 =====')


def main(args=None):
    rclpy.init(args=args)
    node = GraspDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
