// ============================================================
// 数值法逆运动学（Numerical IK - Damped Least Squares）
// ============================================================
// 基于 Jacobian 迭代的 IK 求解器，适用于任意机械臂拓扑
// 时间复杂度：O(n×迭代次数)，典型耗时 10-50ms
// ============================================================

import { invert3x3 } from './utils.js';

/**
 * 数值法 IK（DLS - Damped Least Squares）
 *
 * @param {Object} targetPos - 目标 TCP 位置 {x, y, z}（米）
 * @param {Object} options - 求解参数
 * @param {number} options.maxIter - 最大迭代次数（默认 200）
 * @param {number} options.tolerance - 收敛阈值（米，默认 0.001）
 * @param {number} options.lambda - 阻尼系数（默认 0.5）
 * @param {number} options.stepSize - 步长系数（默认 0.5）
 * @param {number} options.delta - Jacobian 数值微分步长（默认 0.0001）
 * @param {Function} fkCallback - FK 回调函数 (jointAngles) => {x, y, z} 或 null
 *   jointAngles = {q1, q2, q3, q4, q5}
 * @param {Function} getJointsCallback - 获取当前关节角回调 () => {q1, q2, q3, q4, q5}
 * @param {Function} setJointsCallback - 设置关节角回调 (jointAngles) => void
 * @returns {Object} 求解结果 {success, iterations, finalError, joints}
 */
export function solveIK(
  targetPos,
  options = {},
  fkCallback,
  getJointsCallback,
  setJointsCallback
) {
  const {
    maxIter = 200,
    tolerance = 0.001,
    lambda = 0.5,
    stepSize = 0.5,
    delta = 0.0001
  } = options;

  const armJoints = ['q1', 'q2', 'q3', 'q4', 'q5'];

  for (let iter = 0; iter < maxIter; iter++) {
    // 获取当前 TCP 位置
    const currentJoints = getJointsCallback();
    const tcpPose = fkCallback(currentJoints);
    if (!tcpPose) return { success: false, iterations: iter, finalError: Infinity, joints: currentJoints };

    // 计算误差
    const error = {
      x: targetPos.x - tcpPose.x,
      y: targetPos.y - tcpPose.y,
      z: targetPos.z - tcpPose.z
    };
    const errorNorm = Math.sqrt(error.x*error.x + error.y*error.y + error.z*error.z);

    if (errorNorm < tolerance) {
      return { success: true, iterations: iter, finalError: errorNorm, joints: currentJoints };
    }

    // 数值 Jacobian (5x3)
    const jacobian = [];
    for (const jName of armJoints) {
      const origVal = currentJoints[jName];

      // 扰动关节
      const perturbedJoints = { ...currentJoints, [jName]: origVal + delta };
      setJointsCallback(perturbedJoints);

      const tcpPlus = fkCallback(perturbedJoints);
      if (!tcpPlus) continue;

      const dx = (tcpPlus.x - tcpPose.x) / delta;
      const dy = (tcpPlus.y - tcpPose.y) / delta;
      const dz = (tcpPlus.z - tcpPose.z) / delta;

      jacobian.push([dx, dy, dz]);

      // 恢复原值
      setJointsCallback({ ...perturbedJoints, [jName]: origVal });
    }

    // Jacobian 转置 J^T (3x5)
    const jt = [
      [jacobian[0][0], jacobian[1][0], jacobian[2][0], jacobian[3][0], jacobian[4][0]],
      [jacobian[0][1], jacobian[1][1], jacobian[2][1], jacobian[3][1], jacobian[4][1]],
      [jacobian[0][2], jacobian[1][2], jacobian[2][2], jacobian[3][2], jacobian[4][2]]
    ];

    // J * J^T + λ²I (3x3)
    const jjt = [];
    for (let i = 0; i < 3; i++) {
      jjt[i] = [];
      for (let j = 0; j < 3; j++) {
        let sum = 0;
        for (let k = 0; k < 5; k++) sum += jacobian[k][i] * jacobian[k][j];
        if (i === j) sum += lambda * lambda;  // 阻尼项
        jjt[i][j] = sum;
      }
    }

    // 求逆 (J * J^T + λ²I)^-1
    const invJJT = invert3x3(jjt);
    if (!invJJT) continue;  // 奇异矩阵，跳过本次迭代

    // 计算关节增量 Δq = J^T * (J*J^T + λ²I)^-1 * error
    const eVec = [error.x, error.y, error.z];
    const temp = [];
    for (let i = 0; i < 3; i++) {
      temp[i] = 0;
      for (let j = 0; j < 3; j++) temp[i] += invJJT[i][j] * eVec[j];
    }

    const dq = {};
    for (let i = 0; i < 5; i++) {
      let delta_q = 0;
      for (let j = 0; j < 3; j++) delta_q += jt[j][i] * temp[j];
      const jName = armJoints[i];
      dq[jName] = currentJoints[jName] + delta_q * stepSize;
    }

    // 更新关节角
    setJointsCallback(dq);
  }

  // 未收敛
  const finalJoints = getJointsCallback();
  const finalTCP = fkCallback(finalJoints);
  const finalError = finalTCP ? Math.sqrt(
    Math.pow(finalTCP.x - targetPos.x, 2) +
    Math.pow(finalTCP.y - targetPos.y, 2) +
    Math.pow(finalTCP.z - targetPos.z, 2)
  ) : Infinity;

  return { success: false, iterations: maxIter, finalError, joints: finalJoints };
}
