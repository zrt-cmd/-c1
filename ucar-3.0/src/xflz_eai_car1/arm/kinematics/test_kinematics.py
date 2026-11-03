from kinematics.config import Point3D
from kinematics.ik_analytical import solve_analytical_ik_auto_pitch
from kinematics.fk import compute_fk


def test_ik_fk_round_trip():
    """IK 求解目标点 → FK 正解应回到同一位置（闭环验证）。"""
    target = Point3D(0.2, 0.0, 0.2)
    solution = solve_analytical_ik_auto_pitch(target)
    assert not solution.error, f"IK 失败: {solution.error}"

    q = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]
    joint_angles = {f'arm_joint{i+1}': q[i] for i in range(5)}
    pos, _ = compute_fk(joint_angles)

    assert abs(pos.x - target.x) < 1e-3
    assert abs(pos.y - target.y) < 1e-3
    assert abs(pos.z - target.z) < 1e-3


if __name__ == '__main__':
    target = Point3D(0.2, 0.0, 0.2)
    solution = solve_analytical_ik_auto_pitch(target)
    if solution.error:
        print(f"IK 失败: {solution.error}")
    else:
        q = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]
        print(f"IK 解: {q}")
        joint_angles = {f'arm_joint{i+1}': q[i] for i in range(5)}
        pos, T = compute_fk(joint_angles)
        print(f"FK 末端位置: ({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f})")
