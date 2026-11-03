# 实验 05 — 多边形轮廓应用（课程名：自主巡线）

> **内容说明**：本实验名义为「自主巡线」，**实际内容 = 亚博课程「OpenCV应用——多边形轮廓」**（`_references/ascam_course/05_巡线_多边形轮廓.md`）：不规则轮廓 → 拟合规则几何形状（凸包/多边形）并标注顶点数。名称可后续更改。
> **设计依据**：用户指示（2026-08-05）——不参考 `car1_app/line_following.py`，纯 OpenCV 多边形轮廓方案。

## 模块（docs/build/ 各含构建文档）

```
01 config    二值化（binary/adaptive/反相/形态学）+ 轮廓面积 + 多边形拟合（epsilon/凸包）
02 pipeline  二值化 → 轮廓提取 → convexHull/approxPolyDP → 「n边形」标注（vendor convex_hull 语义）
03 app       图像/视频/相机 CLI + MJPEG 流 + 每帧打印顶点数/面积
```

## 运行

```bash
# 本地单测（rknn23 / 板，21 用例）
pytest course_examples/05_line_following/tests/ -v

# 板上（ros2_venv）
cd course_examples/05_line_following

# 相机实时（板上标准：HP60C 深度相机走 ascamera 话题）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --ros                     # MJPEG: http://<板IP>:8080/polygon.mjpg
#    --camera N 仅作无 ROS 的 UVC 回退

# 视频/单图
python src/app.py --video test.mp4
python src/app.py --image x.jpg --save out.jpg
```

## 调参说明

- `invert`：暗物亮底=true（默认）；亮物暗底=false
- `threshold_mode`：光照不均时换 adaptive
- `approx_epsilon`：越小顶点越多（拟合更精细）
- `use_hull`：true = 凸包填平凹槽（vendor 语义）；false = 保留凹形细节

## 板上确认清单（验收门）

- [ ] `verify_runtime.py` 全 PASS
- [ ] `--ros` 相机可取流（ascamera 驱动先起；ROS_DOMAIN_ID 一致）
- [ ] 对纸盒/水瓶等物体：轮廓正确拟合成多边形，标注「n边形」正确
- [ ] MJPEG 流 `http://<板IP>:8080/polygon.mjpg` 实时可见
- [ ] 阈值/epsilon 按实际场景微调，误检可控
- [ ] 全部勾选 → 关闭实验 05，进入实验 06（语义分割背景分割）
