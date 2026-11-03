// ============================================================
// 运动学辅助函数
// ============================================================

/**
 * 3x3 矩阵求逆（高斯-约当消元法）
 * @param {number[][]} m - 3x3 矩阵
 * @returns {number[][]|null} - 逆矩阵，或奇异矩阵时返回 null
 */
export function invert3x3(m) {
  // 增广矩阵 [A|I]
  const aug = [
    [m[0][0], m[0][1], m[0][2], 1, 0, 0],
    [m[1][0], m[1][1], m[1][2], 0, 1, 0],
    [m[2][0], m[2][1], m[2][2], 0, 0, 1]
  ];

  // 高斯消元（化为上三角）
  for (let i = 0; i < 3; i++) {
    // 找主元
    let maxRow = i;
    for (let k = i + 1; k < 3; k++) {
      if (Math.abs(aug[k][i]) > Math.abs(aug[maxRow][i])) maxRow = k;
    }
    [aug[i], aug[maxRow]] = [aug[maxRow], aug[i]];

    // 奇异检查
    if (Math.abs(aug[i][i]) < 1e-10) return null;

    // 归一化主元行
    const pivot = aug[i][i];
    for (let j = 0; j < 6; j++) aug[i][j] /= pivot;

    // 消元
    for (let k = 0; k < 3; k++) {
      if (k !== i) {
        const factor = aug[k][i];
        for (let j = 0; j < 6; j++) aug[k][j] -= factor * aug[i][j];
      }
    }
  }

  // 提取逆矩阵
  return [
    [aug[0][3], aug[0][4], aug[0][5]],
    [aug[1][3], aug[1][4], aug[1][5]],
    [aug[2][3], aug[2][4], aug[2][5]]
  ];
}
