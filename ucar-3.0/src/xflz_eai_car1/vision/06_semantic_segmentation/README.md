# 实验 06 — 语义分割背景替换（课程名：语义分割背景分割）

> **内容说明**：课程「清除背景」语义（vendor 用 opencv_apps 运动分割，需静态相机）；本实验用**语义分割人物抠图 + 背景替换**（yolov8n-seg，COCO 80 类），效果更符合课程目标。模块化、standalone、多线程推理池。

## 模型决策（2026-08-05）
选 **yolov8-seg**（对比 yolov5-seg）：
- fork `ultralytics_yolov8-main` 原生支持 Segment rknn 导出（head.py rknn 分支）→ **13 输出**：每层 `[box(64),cls(80),cls_sum(1),mc(32)]`×3 + proto `[1,32,160,160]`，与 rknn_model_zoo `yolov8_seg` 优化模型布局逐位一致（解码对照其 post_process）
- yolov5-seg 需 airockchip/yolov5 fork（工作区无），导出断链
- DFL/NMS 复用 exp01 postprocess 模式；多线程池复用 exp02 设计（板上无 rknnlite → 完整 `RKNN` 实例 + `core_mask tid%3` 绑核）

## 已交付模型（models/ + images/）
- `models/yolov8n-seg.onnx`：rknn_model_zoo 优化版（13 输出，已验证结构与解码契约一致；n=3.2M 参数）
- `models/yolov8n-seg.rknn`：板上转换产物（int8，13 输出；`_calib.txt` 为生成物不入库）
- `models/bus.jpg` + `coco_80_labels_list.txt` + `download_model.sh`：model zoo 测试/出处材料
- `images/`：COCO val2017 20 张量化校准图（板上转换无需网络；也可再换图）
- **注意（2026-08-06 换 n）**：yolov8n-seg 3.2M 参数（原 s=11M 已弃）——RK3576 单核约 20-40 FPS，3 worker 池约 40-60 FPS；验收门 ≥15 FPS 以实测为准

## 模块（docs/build/ 各含构建文档）

```
01 config    模型/阈值/类别过滤/池参数/背景模式/相机/MJPEG（schema 校验）
02 preprocess  letterbox(pad 0) + RGB + unletterbox（exp01 语义）
03 postprocess  DFL 解码 → grid xyxy → cls_sum×cls 过滤 → 逐类 NMS → mc@proto 掩码
04 background  black/solid/image 背景合成（前景掩码外替换）
05 rknnpool    多线程池：N worker × 独立后端 + round_robin tid%3 绑核
06 mjpeg_server 标准库 MJPEG（复制自 exp02，自包含）
07 app        图像/视频/相机 CLI + 池 + MJPEG + 掩码映射回原图
```

## 运行

```bash
# 本地单测（rknn23 / 板，38 用例）
pytest course_examples/06_semantic_segmentation/tests/ -v

# 模型准备（PC，fork 导出 → 板上转换）
# 1) 下载 yolov8n-seg.pt 并导出（与 exp01 相同机制）：
python -c "from ultralytics.cfg import get_cfg, DEFAULT_CFG; from ultralytics.engine.exporter import export; import sys; sys.argv=['export']; export(get_cfg(DEFAULT_CFG, overrides={'model': 'yolov8n-seg.pt', 'format': 'rknn', 'imgsz': 640}))"
#    产物：yolov8n-seg.onnx（13 输出）→ 拷到板
# 2) 板上转换（ros2_venv；校准图任意场景图 200 张即可，无需标签）：
python src/convert_seg.py --onnx models/yolov8n_seg.onnx --output models/yolov8n_seg.rknn \
    --calib-dir <任意图片目录> --calib-count 200 --target rk3576

# 板上推理
python src/app.py --video test.mp4
python src/app.py --image x.jpg --save out.jpg

# 相机实时（板上标准：HP60C 深度相机走 ascamera 话题，opencv 直连读不了）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --ros                     # 3 worker 池 + MJPEG: http://<板IP>:8080/seg.mjpg
#    --camera N 仅作无 ROS 的 UVC 回退（本板 opencv 未编 libv4l，HP60C 直连失败）
```

## 调参说明

- `model.classes`：保留类别（COCO id；`[0]`=person；空=全部）；多人/多物场景改
- `model.mask_thr`：掩码阈值（0.5 默认；掩码碎/杂 → 调高）
- `pool.workers` / `pool.core_mask`：`round_robin` = 3 worker 绑 3 核（多线程加速）；内存紧时 `auto` + workers 2
- `background.mode`：black / solid(填色) / image(自定义背景图，相对项目根)

## 板上确认清单（验收门）

- [ ] `verify_runtime.py` 全 PASS；`models/yolov8n_seg.rknn` 已转换（`list_outputs()` = 13）
- [ ] `--image` 单人照：person 抠图干净、背景黑/替换图正确
- [ ] `--camera 0` 实时抠图 + MJPEG 可见（UVC 回退；板上优先 `--ros`）
- [ ] `--ros` 相机实时：ascamera 驱动先起；帧率打印正常
- [ ] 多线程生效：3 worker 帧率 ≥ 单线程 2 倍（对照 `workers: 1`）
- [ ] 误检/漏检按场景调 `classes`/`mask_thr`
- [ ] 全部勾选 → 关闭实验 06，进入实验 07（DeepSORT 多目标跟踪）
