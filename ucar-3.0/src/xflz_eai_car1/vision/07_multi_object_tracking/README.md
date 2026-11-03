# 实验 07 — 多目标跟踪（DeepSORT 车辆/行人跟踪）

> **内容说明**：课程实验 07「DeepSORT 车辆跟踪分析」。vendor 参考 = 工作区 `Deepsort/`（YOLOv8 检测 + deep_sort_pytorch 移植 + ReID `deepsort.rknn`）；参考代码只读，本实验按 deep_sort 标准算法自实现（纯 numpy，零新增依赖：scipy 板载已有）。

## 模型（已随仓库交付：models/）
- `models/yolov8n.rknn`：检测（COCO 80 类，9 输出；用户 2026-08-05 自转，rknn-toolkit2 2.3.x 验证可加载）
- `models/deep_int8_in_8.rknn`：ReID 外观（用户自转 int8，输入 64×128 ImageNet 归一化，输出 128-d）
- `models/coco_classes.json`：类别名（vendor）
- vendor 原版（`Deepsort/rknnModel/`）留作对比；自导出替代流程 = exp01/06 同款
- ReID 缺失/加载失败 → 自动回退 `sort` 模式（纯 IoU）并告警

## 多线程（三处并行）
- **检测池**：RKNNPool N worker（默认 3，`round_robin` 按 tid%3 绑 NPU 核）
- **ReID 池**：ReIDPool 2 worker × 独立 RKNN 实例（实例非线程安全，每线程一个），提交序号回收保序
- **流水线双缓冲**：帧 N 处理（解码/跟踪/绘制）期间帧 N+1 已在检测池推理

## 模块（docs/build/ 各含构建文档）

```
01 config      检测/跟踪器超参/模式(sort|deepsort)/池/相机/MJPEG（schema 校验）
02 preprocess  letterbox(pad 0) + RGB + unletterbox（exp06 同款）
03 detect      yolov8 9 输出解码（DFL + cls_sum×cls 过滤 + 逐类 NMS，nc 通用）
04 tracker     Kalman(8维, std 按 h 缩放) + Hungarian(scipy) + 外观级联 + IoU + 轨迹生命周期
05 reid        外观提取（crop→64×128→ImageNet 归一化→NCHW→128-d；RKNN/Fake 后端）
06 app         池 + 检测 + 跟踪 + ID 标注 + MJPEG + sort 回退
```

## 运行

```bash
# 本地单测（rknn23 / 板，38 用例）
pytest course_examples/07_multi_object_tracking/tests/ -v

# 板上（ros2_venv；模型拷入 models/）
python src/app.py --video test.mp4
python src/app.py --image x.jpg --save out.jpg

# 相机实时（板上标准：HP60C 深度相机走 ascamera 话题，opencv 直连读不了）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --ros                     # 话题取流 + 3 worker 检测池 + DeepSORT + MJPEG: :8080/track.mjpg
#    --camera N 仅作无 ROS 的 UVC 回退（本板 opencv 未编 libv4l，HP60C 直连失败）
```

## 调参说明

- `tracker.max_dist`：外观余弦门限（deepsort）；`max_iou_distance`：IoU 门限
- `tracker.max_age`：丢失轨迹保留帧数（越大越抗遮挡，越小越跟丢即删）；`n_init`：确认所需连续命中
- `tracker.mode`：deepsort（需 reid 模型）/ sort（纯 IoU）
- `pool.workers` / `core_mask`：round_robin = 3 worker 绑 3 核

## 板上确认清单（验收门）

- [ ] `models/yolov8.rknn` + `models/deepsort.rknn` 就位；`app.py --camera 0` 可运行，打印 `mode=deepsort`
- [ ] 单目标移动：ID 稳定不跳；双目标交叉：ID 不互换
- [ ] 目标遮挡数帧后重现：deepsort 模式恢复原 ID（sort 模式换 ID——对照演示）
- [ ] MJPEG `http://<板IP>:8080/track.mjpg` 实时可见
- [ ] 3 worker 检测池帧率 ≥ 单 worker 1.5 倍
- [ ] `--ros` 相机实时可用（ascamera 驱动先起；ROS_DOMAIN_ID 一致；帧率打印正常）
- [ ] 全部勾选 → 关闭实验 07，进入实验 08（基于深度信息的 3D 物体检测）
