# 实验 03 — HSV 颜色过滤识别

对应课程（vendor `03_lab_颜色识别_RGB颜色过滤` / `03_颜色追踪_HSV`）：**BGR→HSV 阈值掩膜 → 形态学开/闭 → 圆形轮廓标识 → 颜色名标注**；可视化对齐车端 `visual_grasp`：**左 RGB 栏（绿轮廓+黄框+红质心十字+颜色名）| 右 HSV Mask 栏（白匹配区）**，双栏推流。注意课程目录名 "lab" = 实验，不是 Lab 色彩空间。

## 模块（docs/build/ 各含构建文档）

```
01 config    颜色 HSV 阈值（默认红/蓝/绿；红色 h2 环形第二段）/ 形态学核 / 轮廓过滤 / 相机参数
02 preprocess bgr_to_hsv → range_mask(h2) → morph_open/close → largest_circle → detect_colors + filter_view
03 app       图像/视频/相机 CLI + MJPEG 流（双栏：左原图|右过滤图） + --tune 调参（H/S/V 滑块）
```

## 运行

```bash
# 本地单测（WSL rknn23 / 板，24 用例）
pytest course_examples/03_lab_color_recognition/tests/ -v

# 板上（ros2_venv）
cd course_examples/03_lab_color_recognition
# 相机（板上标准：HP60C 深度相机走 ascamera 话题；opencv 直连读不了——未编 libv4l）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --ros                     # 相机 + MJPEG: http://<板IP>:8080/lab_color.mjpg
python src/app.py --camera 0                # 无 ROS 的 UVC 回退（索引以实测为准）
python src/app.py --video test.mp4        # 视频
python src/app.py --image x.jpg --save out.jpg

# 调参（H/S/V 滑块，需显示器；板上也支持——tune 走 --ros 话题取流）
python src/app.py --tune 红色 --ros          # 6 滑块实时调 H/S/V；Ctrl+C 退出并打印当前阈值
#   滑块窗口：lab_tune；双栏窗口：lab_view（左原图 | 右处理后）；推流 :8080/lab_color.mjpg 同双栏
```

## 阈值调节说明

- 调参模式显示的 H/S/V 范围直接写入 `config/lab_config.yaml` 对应颜色即生效
- OpenCV HSV 约定：H∈0-179、S/V∈0-255；红色在 0/180 交界 → `h2` 第二段（车端 visual_grasp 同款）
- 形态学：`open_kernel` 去噪点、`close_kernel` 填孔洞；核须奇数
- 轮廓过滤：`min_area` 滤噪、`min_radius/max_radius` 限目标尺寸

## 板上确认清单（验收门）

- [ ] `verify_runtime.py` 全 PASS
- [ ] `--ros` 相机实时可用（ascamera 驱动先起；ROS_DOMAIN_ID 一致）；`--camera 0` UVC 回退可取流（失败则试 1/2…或 `--video`）
- [ ] 红/蓝/绿三种目标均能检出：圆形轮廓 + 窗口左下角颜色名正确
- [ ] MJPEG 流 `http://<板IP>:8080/lab_color.mjpg` 实时可见
- [ ] 阈值按实际光照微调（`--tune` 或改 config），误检/漏检可控
- [ ] 全部勾选 → 关闭实验 03，进入实验 04（二维码生成与识别）
