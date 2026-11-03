// ============================================================
// 机械臂运动学标定参数 (Robot Calibration Parameters)
// ============================================================
// 所有几何参数集中在此，便于实物装配后修正。
// 测量方法：让机械臂置于零位，用千分尺/激光测距仪测量各关键点。
// 修改后调用 saveCalibration() 持久化到 localStorage。
// ============================================================

export const ROBOT_CALIBRATION = {
  // ----- 基座到 J1 (绕 Z 轴的 yaw 关节) -----
  // J1 旋转轴在 base_footprint 坐标系下的位置
  J1_origin: {
    x: 0.05662368,   // base_link(0.046624) + arm_base_link(0.01)
    y: 0.0,
    z: 0.129500      // base_joint(0.029) + arm_base(0.057) + arm_link1(0.0435)
  },

  // ----- J1 到 J2 (J1 局部系下的 J2 起点) -----
  J2_offset: {
    x: 0.000186,
    y: 0.025,        // +25mm Y 偏移（与 J5 的 -25mm 对消）
    z: 0.059467
  },

  // ----- 连杆 1 (J2 → J3，在 J2 局部系) -----
  link1: {
    x: -0.0009,      // X 偏移（装配偏差导致连杆不完全沿 Z）
    y: 0.000249,     // Y 偏移（忽略）
    z: 0.126141      // 主长度（沿 Z 方向分量）
  },

  // ----- 连杆 2 (J3 → J4，在 J3 局部系) -----
  link2: {
    x: -0.001266,
    y: 0.000249,
    z: 0.126138
  },

  // ----- 末端工具 (J4 → TCP，经过 J5) -----
  // 合成了 J4→J5(0.000919, -0.025283, 0.074395) + J5→TCP(0.000697, 0, 0.076247)
  // 并应用了 J5 的 RPY(0, 0, -π) 固定旋转
  toolFromJ4: {
    x: 0.000222,     // 0.000919 + R_z(-π)·0.000697 ≈ 0.000222
    y: -0.025283,    // J5 的 Y 偏移
    z: 0.150642      // 0.074395 + 0.076247
  },

  // ----- 关节限位 (URDF 硬限位) -----
  jointLimits: {
    q1: { min: -Math.PI, max: Math.PI },
    q2: { min: -Math.PI, max: Math.PI },
    q3: { min: -Math.PI, max: Math.PI },
    q4: { min: -Math.PI, max: Math.PI },
    q5: { min: -Math.PI, max: Math.PI }
  },

  // ----- 求解器参数 -----
  solver: {
    reachTolerance: 1e-5,      // 可达性数值容差 (10μm)
    yPlaneTolerance: 0.01      // Y 平面约束容差 (10mm)
  }
};

/**
 * 从标定参数计算派生几何量
 * @param {Object} cal - ROBOT_CALIBRATION
 * @returns {Object} 派生几何量（连杆长度、偏角、工作空间）
 */
export function deriveGeometry(cal) {
  const link1 = cal.link1;
  const link2 = cal.link2;
  const tool = cal.toolFromJ4;

  const L1 = Math.hypot(link1.x, link1.z);
  const L2 = Math.hypot(link2.x, link2.z);
  const L_TCP = Math.hypot(tool.x, tool.z);

  // TCP 在 J1 局部系的固定 Y 偏移
  // 由于 J2/J3/J4 都绕 Y 轴旋转，Y 分量不受这些旋转影响，直接累加
  // = J2_offset.y + link1.y + link2.y + toolFromJ4.y
  const Y_TCP = cal.J2_offset.y + link1.y + link2.y + tool.y;

  return {
    // 连杆有效长度（X-Z 平面投影）
    L1: L1,
    L2: L2,
    L_TCP: L_TCP,

    // 连杆方向角（相对父系 +Z 轴的偏角，用于精确几何算法）
    PHI1: Math.atan2(link1.x, link1.z),
    PHI2: Math.atan2(link2.x, link2.z),
    PHI_TCP: Math.atan2(tool.x, tool.z),

    // TCP 在 J1 系的固定 Y 偏移（用于 q1 补偿，消除残留误差）
    Y_TCP: Y_TCP,

    // 工作空间
    maxReach: L1 + L2,
    minReach: Math.abs(L1 - L2)
  };
}

// 默认派生几何量（页面加载时计算）
export const GEOM = deriveGeometry(ROBOT_CALIBRATION);
