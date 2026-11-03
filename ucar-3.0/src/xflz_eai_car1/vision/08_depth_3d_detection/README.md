# 实验 08 — 基于深度信息的 3D 物体检测（深度查看器）

> **内容说明**：按 `test.md` 关联的 `depth_viewer_guide.md` 移植规格实现——**深度相机双栏显示（左 RGB | 右 Depth 伪彩）+ 点击读深度（mm/m + 相机坐标反投影）**。规格明确「无检测」（visual_grasp 最小子集）。vendor 参考课用 MediaPipe Objectron（mediapipe 无 aarch64 wheel，弃用）。

## 模块（docs/build/ 各含构建文档）

```
01 config    深度尺度/无效值/伪彩/显示缩放 + 相机内参 + 数据源（image|ros）+ 服务
02 depth     深度加载（uint16 PNG → mm）、归一化、伪彩、点击读深度 + 反投影
03 view      双栏合成（RGB | Depth）+ 状态栏 + 点击坐标还原/标注
11|04 app       文件模式（本地）/ ROS 模式（板上 rclpy，严禁 remap）+ MJPEG + 窗口点击
```

## 运行

```bash
# 本地单测（rknn23 / 板，24 用例）
pytest course_examples/08_depth_3d_detection/tests/ -v

# 文件模式（config 填 rgb_path/depth_path；depth 为 uint16 PNG）
python src/app.py --config config/depth3d_config.yaml --display   # 窗口点击读深度
python src/app.py --config ...                                     # MJPEG: :8080/depth.mjpg

# 板上 ROS 模式（需先起相机；驱动原生话题名不可 remap）
ros2 launch xflz_eai_car1/examples/camera/HP60C/... （或 car1_peripherals depth_camera.launch.py）
python src/app.py --config config/depth3d_config.yaml --ros        # config source.mode: ros
```

## 验证要点（规格 depth_viewer_guide.md 清单）

- [ ] 双栏图：左 RGB、右 Depth 伪彩（jet），状态栏文字可见
- [ ] 点击深度图任一点 → 终端 `Click (u, v): z = XXXX mm (X.XXX m)`；有内参时附加 `xyz=(x, y, z) m`
- [ ] 点击物体 vs 背景数值差异明显；无效区（0）提示 invalid
- [ ] 点击坐标还原正确（窗口有缩放，÷ display_scale）
- [ ] 板上：`ros2 topic hz .../rgb0/image` ≈ 25Hz；深度单位 mm（数值勿 ×1000 误解）
- [ ] 全部勾选 → 关闭实验 08，进入实验 09（人脸关键点，参考已扒 面部检测课）

## 已知坑（来自 guide §5）

- 首帧黑帧（`rgb.mean() < 5` 跳过，已内置）；深度 0 = 无效；话题严禁 remap
- 内参默认 = `ascam.yaml`（HP60C RGB 标定）；驱动配置加密，深度内参以实测微调
