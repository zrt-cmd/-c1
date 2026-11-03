# 实验 09：人脸关键点特征检测（Face Landmark）

> 对应 vendor 课程《4、面部检测》（`_references/ascam_course/09_面部检测.md`）：原帧 +
> 人脸关键点网格双栏显示 + 实时 FPS。**方案 = mediapipe Blaze 模型转 RKNN，板上
> RKNNLite NPU 推理 468 点网格**（与 vendor 04_FaceMesh 视觉一致；方案演进见
> `docs/build/02_landmark.md`）。

## 技术选型（2026-08-05/06 板上实测与决策）

| 方案 | 结论 |
|---|---|
| mediapipe 1.0.0（官方 aarch64 wheel） | ❌ 装得上但**运行 SIGILL**：wheel 编译开启 LSE（ARMv8.1），RK3576（A53/A72，ARMv8.0）无 LSE，`create_from_options` 即崩（实测 exit 132）；GPU 后端变量无效 |
| dlib 68 点（vendor 06 移植） | ✅ 可跑但仅 68 点、HOG 检测 CPU 慢——**用户决策推翻**（已删） |
| **RKNN Blaze（现行）** | ✅ mediapipe 官方 tflite（`face_detection_short_range` + `face_landmark`）→ rknn → **NPU 468 点**；路径经 `rknn-mediapipe-master`（rk3568）验证，单图 121ms |
| **板上推理后端 = RKNNLite** | ✅ **2026-08-06 用户决策**：板上统一 `from rknnlite.api import RKNNLite`（rknn-toolkit-lite2 2.3.2，RK3576 专用部署 API）；完整版 rknn-toolkit2 仅留在 x86 转换环境；本实验为此**整体删除重建** |

## 交付形态

**ament_python ROS2 包 `car1_face_landmark/`**——拷贝到板 `~/ros2_ws/src/` 后 colcon 编译。内层 python 包 `face_landmark_core/` 承载纯逻辑（config/backend/landmark/view/mjpeg/demo/convert），无 ROS 强依赖、x86 全可测。

```
09_face_landmarks/
├── README.md  requirements.txt  pytest.ini
├── config/face_config.yaml          ← 配置源（包内副本自包含部署；含 backend.core_mask）
├── docs/build/01..06_*.md           ← 模块构建文档（文档先行）
├── car1_face_landmark/              ← ROS2 包（拷贝到板 ros2_ws/src）
│   ├── package.xml  setup.py  setup.cfg  resource/  launch/  config/
│   └── face_landmark_core/
│       ├── config.py  backend.py  landmark.py  view.py  mjpeg_server.py
│       ├── demo_image.py  camera_demo.py   ← 图片模式 / USB 摄像头实时
│       ├── convert_mediapipe.py     ← tflite → rknn（x86 转换环境；CLI 对齐 mediapipe-rknn）
│       └── face_landmark_node.py    ← ROS 节点（vendor 04_FaceMesh 结构）
├── models/                          ← tflite 源 + rknn 产物（**已入库**）
├── tests/                           ← 109 用例（板端 ros2_venv 全绿，fake 注入）
```

## 模型（已转换入库）

`models/` 已含 **fp16 混合量化** rknn（rk3576 target；无校准集不 int8，landmark 回归 fp16 精度更稳；`.rknn` 与 RKNNLite 通用）：

| 模型 | 输入契约 | 输出契约 |
|---|---|---|
| face_detection_short_range.rknn（584KB） | float32 RGB [0,1] NHWC [1,128,128,3] | regressors [1,896,16] + classificators [1,896,1] |
| face_landmark.rknn（1.6MB） | float32 RGB [0,1] NHWC [1,192,192,3] | conv2d_21 [1,1,1,1404] + conv2d_31 [1,1,1,1] |

如需 int8：`--dtype i8 --dataset calib.txt` 重转（校准集为图片路径列表）：
`PYTHONPATH=car1_face_landmark python -m face_landmark_core.convert_mediapipe --all --check`

## 板上部署与运行（用户执行）

```bash
# 0) 板上推理环境：rknnlite 已装（rknn-toolkit-lite2 2.3.2；wheel 备份在
#    course_examples/runtime/board/rknn_toolkit_lite2-2.3.2-cp310-*.whl）
source ~/ros2_venv/bin/activate && python3 -c "from rknnlite.api import RKNNLite; print('ok')"
# 0b) MJPEG 流服务器：优先 mjpeg_streamer 库（用户决策允许安装；不装则自动回退内置实现）。
#     板上 pip 一律走清华镜像（规则，2026-08-06）：dry-run 验证 = mjpeg_streamer 2024.2.8
#     + aiohttp 依赖族，全部 cp310 aarch64 wheel，无冲突
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple mjpeg_streamer
# 3) 板上相机实时（标准：HP60C 深度相机走 ascamera 话题；opencv 直连读不了——
#    ros2_venv 的 opencv 未编 libv4l；ROS_DOMAIN_ID 必须与驱动一致，本板 =2）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py
# ⚠️ PYTHONPATH 必须保留 $PYTHONPATH（覆盖会丢 Humble 路径 → venv 内 PyPI rclpy 被加载 → 崩）
PYTHONPATH=car1_face_landmark:$PYTHONPATH python3 -m face_landmark_core.camera_demo \
  --config config/face_config.yaml --ros --display   # 或浏览器 :8081/face.mjpg（无 --display）
# 3b) 无 ROS 的 UVC 回退（vendor 06 形态；V4L2+YUYV——本板 NOVATEK 相机 MJPG 损坏会出雪花）
PYTHONPATH=car1_face_landmark:$PYTHONPATH python3 -m face_landmark_core.camera_demo \
  --config config/face_config.yaml --camera 25 --display
# 4) Python 程序验证环境（推荐；ROS 节点模式可选，见下）
#    环境：ros2_venv（rknnlite + mjpeg_streamer）；--ros 需先起 ascamera 驱动（话题严禁 remap）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py
#   单图：左栏绿框 + 右栏 468 网格
PYTHONPATH=car1_face_landmark:$PYTHONPATH python3 -m face_landmark_core.demo_image \
  --config config/face_config.yaml --save out.jpg --display
#   相机实时（--ros 标准）：浏览器 http://<板IP>:8081/face.mjpg
PYTHONPATH=car1_face_landmark:$PYTHONPATH python3 -m face_landmark_core.camera_demo \
  --config config/face_config.yaml --ros --display
#   可选 ROS 节点模式（需先编译：cd ~/ros2_ws && colcon build --symlink-install --packages-select car1_face_landmark）
ros2 launch car1_face_landmark face_landmark.launch.py   # MJPEG: http://<板IP>:8081/face.mjpg
```

多核 NPU（可选）：`config/face_config.yaml` → `backend.core_mask`：0=AUTO（默认）、
1=CORE_0、2=CORE_1、4=CORE_2、3=0_1、7=0_1_2（RK3576 三核）。

## 实现（src 逻辑，x86 全可测）

| 模块 | 职责 |
| `config.py` | schema 校验 `load_config()`（双模型路径、backend.core_mask；source.mode = image\|camera\|ros） |
| `backend.py` | `Backend` 协议；**`RKNNLiteBackend`**（懒导入 `rknnlite.api.RKNNLite`，`init_runtime(core_mask=...)`，无 target 参数）；`FakeBackend`（x86 单测） |
| `landmark.py` | `FaceLandmarkerEngine`：BlazeFace 检测（锚解码+NMS）→ ROI 裁剪 → FaceLandmark（468 点+presence）→ 逆映射原图归一化；注入 fake 可测；`draw_mesh`（478 拓扑 124 边） |
| `convert_mediapipe.py` | tflite → rknn（`--platform` / `--dtype fp\|i8`，对齐 mediapipe-rknn 惯例）+ rknn 头输出名契约自检（`--check`） |
| `view.py` / `mjpeg_server.py` | 双栏合成+状态栏 / stdlib MJPEG 流（:8081，避开 08 的 :8080） |
| `demo_image.py` | 图片模式（读图→检测→双栏→保存/显示） |
| `camera_demo.py` | 摄像头实时模式（`--ros` = ascamera 话题取流标准；`--camera N` = V4L2 回退；循环→检测→双栏→FPS→MJPEG→q 退出；cap/engine 可注入） |
| 显示 | 左栏 = 原图 + **人脸检测框**（YOLO 风格：绿框 + `face <score>` 标签）；右栏 = 468 点网格 |
| `face_landmark_node.py` | ROS 节点（订阅原生话题、spin_once+q、MJPEG） |

## 板上验证清单（2026-08-06 用户确认 ✅）

1. ✅ 图片模式：双栏，右栏 **468 点网格**（眼/眉/唇/脸廓黄点绿线，vendor 视觉），`--save` 出图
2. ✅ **相机实时**：`camera_demo --ros --display`（ascamera 驱动先起）对着人脸网格实时贴合；**左栏绿框 + `face 0.9x` 标签**（YOLO 风格）+ 右栏 468 点网格；多人（num_faces=2）各一套框/网格；无脸时无框不崩溃；FPS ≥10（fp16 混合，NPU）
3. ✅ 板上相机实时（ROS）：HP60C 话题流畅；MJPEG `:8081/face.mjpg` 浏览器可看（SSH 无显示器用）
4. ✅ `q` 退出干净；模型契约：`convert_mediapipe.py --check`（regressors/classificators、conv2d_21/conv2d_31）
5. ✅ 模型一致性：`python3 -c "from rknnlite.api import RKNNLite; ..."` 加载两个 .rknn 无报错

## 已知差异（vs vendor 04_FaceMesh）

- 后端 = RKNN Blaze（468 点，mediapipe 模型）→ RKNNLite NPU 推理；vendor = mediapipe solutions（板上不可装）
- 省略 `/mediapipe/points` 发布（板无 yahboomcar_msgs）；新增 MJPEG 流
- ROI 裁剪/锚解码对齐 mediapipe 语义（参数来自 rknn-mediapipe-master 案例，Apache-2.0）
