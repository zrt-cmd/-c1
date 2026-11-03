// ============================================================
// 解析法逆运动学（Analytical IK）
// ============================================================
// 基于几何闭合解的 IK 求解器，无需迭代，速度快（~0.05ms）
// 支持三种模式：固定 Pitch、Auto Pitch、Ignore Pitch
// ============================================================

/**
 * 解析法 IK（固定 Pitch 模式）
 * @param {Object} targetPos - 目标 TCP 位置 {x, y, z}（米）
 * @param {number} targetPitchRad - 目标 Pitch 角度（弧度）
 * @param {Object} config - 机械臂标定参数（ROBOT_CALIBRATION）
 * @param {Object} geom - 派生几何量（GEOM）
 * @returns {Object} IK 解 {q1, q2, q3, q4, q5, branch, error}
 */
export function solveAnalyticalIK(targetPos, targetPitchRad = 0, config, geom) {
  const cal = config;
  const g = geom;

  // === Step 1: 平移到 J1 原点 ===
  const dx_world = targetPos.x - cal.J1_origin.x;
  const dy_world = targetPos.y - cal.J1_origin.y;
  const dz_world = targetPos.z - cal.J1_origin.z;

  // === Step 2: 计算 q1 (base yaw) ===
  // TCP 在 J1 局部系有固定 Y 偏移 Y_TCP（约 0.215mm），不能直接 atan2(dy,dx)。
  // 几何关系：目标水平距离 r = √(dx²+dy²)，TCP 偏移后需命中目标，
  // 故 q1 = atan2(dy,dx) - asin(Y_TCP / r)
  const r_horiz = Math.hypot(dx_world, dy_world);
  let q1 = Math.atan2(dy_world, dx_world);
  if (r_horiz > 1e-9) {
    const asinArg = Math.max(-1, Math.min(1, g.Y_TCP / r_horiz));
    q1 -= Math.asin(asinArg);
  }

  // === Step 3: 旋转到 J1 坐标系 ===
  const c1 = Math.cos(q1), s1 = Math.sin(q1);
  const x_j1 = c1 * dx_world + s1 * dy_world;
  const y_j1 = -s1 * dx_world + c1 * dy_world;
  const z_j1 = dz_world;

  // === Step 4: 检查 Y 坐标约束 ===
  // q1 已补偿 Y_TCP，理想下 y_j1 ≈ Y_TCP，检查残差是否在容差内
  if (Math.abs(y_j1 - g.Y_TCP) > cal.solver.yPlaneTolerance) {
    return {
      q1, q2: 0, q3: 0, q4: 0, q5: 0,
      error: `Target Y residual=${((y_j1-g.Y_TCP)*1000).toFixed(1)}mm in J1 frame, exceeds workspace constraint (tolerance ±${(cal.solver.yPlaneTolerance*1000).toFixed(0)}mm)`
    };
  }

  // === Step 5: 在 X-Z 平面求解 2R IK ===
  // pitch 定义为 J4→TCP 偏移向量在 J1 系的实际指向角。
  // 偏移向量长度 = L_TCP，整体绕 Y 轴旋转 pitch，X 分量已含在 sin(pitch) 中。
  const toolX = geom.L_TCP * Math.sin(targetPitchRad);
  const toolZ = geom.L_TCP * Math.cos(targetPitchRad);

  const x_j4 = x_j1 - toolX;
  const z_j4 = z_j1 - toolZ;

  const x_rel = x_j4 - cal.J2_offset.x;
  const z_rel = z_j4 - cal.J2_offset.z;

  const distSq = x_rel * x_rel + z_rel * z_rel;
  const dist = Math.sqrt(distSq);

  // === Step 6: 可达性检查 ===
  if (dist > g.maxReach + cal.solver.reachTolerance) {
    return {
      q1, q2: 0, q3: 0, q4: 0, q5: 0,
      error: `Out of reach: target distance ${dist.toFixed(4)}m > max reach ${g.maxReach.toFixed(4)}m`
    };
  }
  if (dist < g.minReach - cal.solver.reachTolerance) {
    return {
      q1, q2: 0, q3: 0, q4: 0, q5: 0,
      error: `Too close: target distance ${dist.toFixed(4)}m < min reach ${g.minReach.toFixed(4)}m`
    };
  }

  // === Step 7: 余弦定理求连杆间相对转角 ===
  let cosQ3rel = (distSq - g.L1*g.L1 - g.L2*g.L2) / (2*g.L1*g.L2);
  cosQ3rel = Math.max(-1, Math.min(1, cosQ3rel));
  const q3rel_mag = Math.acos(cosQ3rel);

  // === Step 8: 2R 标准公式 ===
  const alpha = Math.atan2(x_rel, z_rel);

  // === Step 9: 枚举两个 elbow 分支（含 PHI 偏角补偿）===
  const candidates = [];

  for (const sign of [+1, -1]) {
    const q3_rel = sign * q3rel_mag;
    const beta = Math.atan2(g.L2 * Math.sin(q3_rel), g.L1 + g.L2 * Math.cos(q3_rel));

    // 连杆绝对方向角
    const theta1 = alpha - beta;
    const theta2 = theta1 + q3_rel;

    // 关节角 = 连杆方向角 - 连杆自身偏角（PHI 补偿）
    const q2 = theta1 - g.PHI1;
    const q3 = q3_rel + g.PHI1 - g.PHI2;
    const q4 = targetPitchRad - theta2 + g.PHI2 - g.PHI_TCP;

    // 检查关节限位
    const lim = cal.jointLimits;
    if (q2 >= lim.q2.min && q2 <= lim.q2.max &&
        q3 >= lim.q3.min && q3 <= lim.q3.max &&
        q4 >= lim.q4.min && q4 <= lim.q4.max) {
      candidates.push({
        q1, q2, q3, q4, q5: 0,
        branch: sign > 0 ? 'elbow-up' : 'elbow-down',
        error: null
      });
    }
  }

  if (candidates.length === 0) {
    return {
      q1, q2: 0, q3: 0, q4: 0, q5: 0,
      error: 'Both elbow configurations exceed joint limits'
    };
  }

  // 优先返回 elbow-up 配置
  candidates.sort((a, b) => Math.abs(b.q3) - Math.abs(a.q3));

  return candidates[0];
}

/**
 * 解析法 IK（Auto Pitch 模式）
 * 当用户指定的 pitch 不可达时，自动搜索最接近的可行 pitch
 * @param {Object} targetPos - 目标 TCP 位置 {x, y, z}
 * @param {number} preferredPitchRad - 期望 Pitch 角度（弧度）
 * @param {Object} config - 标定参数
 * @param {Object} geom - 派生几何量
 * @returns {Object} IK 解 {q1, q2, q3, q4, q5, actualPitch, autoPitch, pitchDeviation, error}
 */
export function solveAnalyticalIKAutoPitch(targetPos, preferredPitchRad = 0, config, geom) {
  // 先尝试用户指定的 pitch
  const direct = solveAnalyticalIK(targetPos, preferredPitchRad, config, geom);
  if (!direct.error) {
    return { ...direct, actualPitch: preferredPitchRad, autoPitch: false, pitchDeviation: 0, error: null };
  }

  // 扫描所有 pitch 找到最接近 preferredPitch 的可行解
  let bestPitch = null, bestDist = Infinity, bestSol = null;
  for (let pDeg = -180; pDeg <= 180; pDeg += 1) {
    const pRad = pDeg * Math.PI / 180;
    const sol = solveAnalyticalIK(targetPos, pRad, config, geom);
    if (!sol.error) {
      const d = Math.abs(pRad - preferredPitchRad);
      if (d < bestDist) {
        bestDist = d;
        bestPitch = pRad;
        bestSol = sol;
      }
    }
  }

  if (bestSol) {
    return {
      ...bestSol,
      actualPitch: bestPitch,
      autoPitch: true,
      pitchDeviation: bestDist,
      error: null
    };
  }

  return { q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, error: '目标点超出工作空间' };
}

/**
 * 解析法 IK（Ignore Pitch 模式）
 * 扫描所有 pitch 和 elbow 分支，选择 FK 误差最小且肘部最高的配置
 *
 * @param {Object} targetPos - 目标 TCP 位置 {x, y, z}
 * @param {Object} config - 标定参数
 * @param {Object} geom - 派生几何量
 * @param {Function} fkCallback - FK 回调函数 (jointAngles) => {x, y, z} 或 null
 *   jointAngles = {q1, q2, q3, q4, q5}
 * @returns {Object} IK 解 {q1, q2, q3, q4, q5, actualPitch, tcpError, elbowZ, branch, error}
 */
export function solveAnalyticalIKIgnorePitch(targetPos, config, geom, fkCallback) {
  const cal = config;
  const g = geom;
  const inLimits = v => v >= cal.jointLimits.q2.min && v <= cal.jointLimits.q2.max;

  // === Step 1-3: 坐标变换到 J1 系 ===
  const dx_world = targetPos.x - cal.J1_origin.x;
  const dy_world = targetPos.y - cal.J1_origin.y;
  const dz_world = targetPos.z - cal.J1_origin.z;

  // q1 补偿 TCP 的固定 Y 偏移（与 solveAnalyticalIK 一致）
  const r_horiz = Math.hypot(dx_world, dy_world);
  let q1Base = Math.atan2(dy_world, dx_world);
  if (r_horiz > 1e-9) {
    const asinArg = Math.max(-1, Math.min(1, g.Y_TCP / r_horiz));
    q1Base -= Math.asin(asinArg);
  }

  const c1 = Math.cos(q1Base), s1 = Math.sin(q1Base);
  const x_j1 = c1 * dx_world + s1 * dy_world;
  const y_j1 = -s1 * dx_world + c1 * dy_world;
  const z_j1 = dz_world;

  // === Y 约束检查 ===
  // q1 已补偿 Y_TCP 偏移，理想情况下 y_j1 ≈ Y_TCP，残差应在容差内
  if (Math.abs(y_j1 - g.Y_TCP) > cal.solver.yPlaneTolerance) {
    return {
      q1: 0, q2: 0, q3: 0, q4: 0, q5: 0,
      error: `Target Y residual=${((y_j1-g.Y_TCP)*1000).toFixed(1)}mm in J1 frame exceeds workspace`
    };
  }

  let bestErr = Infinity, bestPitch = null, bestSol = null, bestElbowZ = -Infinity;

  for (let pDeg = -180; pDeg <= 180; pDeg += 1) {
    const pRad = pDeg * Math.PI / 180;
    const toolX = g.L_TCP * Math.sin(pRad);
    const toolZ = g.L_TCP * Math.cos(pRad);
    const x_j4 = x_j1 - toolX;
    const z_j4 = z_j1 - toolZ;
    const xRel = x_j4 - cal.J2_offset.x;
    const zRel = z_j4 - cal.J2_offset.z;
    const distSq = xRel*xRel + zRel*zRel;
    const cosQ3 = (distSq - g.L1*g.L1 - g.L2*g.L2) / (2*g.L1*g.L2);
    if (Math.abs(cosQ3) > 1) continue;
    const q3Mag = Math.acos(cosQ3);
    const alpha = Math.atan2(xRel, zRel);

    for (const sgn of [+1, -1]) {
      const q3_rel = sgn * q3Mag;
      const beta = Math.atan2(g.L2*Math.sin(q3_rel), g.L1 + g.L2*Math.cos(q3_rel));

      const theta1 = alpha - beta;
      const theta2 = theta1 + q3_rel;

      const q2 = theta1 - g.PHI1;
      const q3 = q3_rel + g.PHI1 - g.PHI2;
      const q4 = pRad - theta2 + g.PHI2 - g.PHI_TCP;
      if (!inLimits(q2) || !inLimits(q3) || !inLimits(q4)) continue;

      // 调用 FK 回调验证
      const tcpActual = fkCallback({ q1: q1Base, q2, q3, q4, q5: 0 });
      if (!tcpActual) continue;

      const err = Math.sqrt(
        Math.pow(tcpActual.x - targetPos.x, 2) +
        Math.pow(tcpActual.y - targetPos.y, 2) +
        Math.pow(tcpActual.z - targetPos.z, 2)
      );

      const elbowZ = cal.J1_origin.z + cal.J2_offset.z + g.L1 * Math.cos(q2);
      const score = err - 0.5 * Math.max(0, elbowZ);
      const bestScore = bestErr - 0.5 * Math.max(0, bestElbowZ);

      if (score < bestScore - 1e-6) {
        bestErr = err;
        bestPitch = pRad;
        bestElbowZ = elbowZ;
        bestSol = { q1: q1Base, q2, q3, q4, q5: 0, branch: sgn > 0 ? 'A' : 'B' };
      }
    }
  }

  if (bestSol) {
    return {
      ...bestSol,
      actualPitch: bestPitch,
      autoPitch: true,
      pitchDeviation: 0,
      tcpError: bestErr,
      elbowZ: bestElbowZ,
      error: null
    };
  }

  return { q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, error: '目标点超出工作空间' };
}
