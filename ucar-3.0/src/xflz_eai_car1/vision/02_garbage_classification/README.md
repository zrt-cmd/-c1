# 实验 02 — 垃圾分类（YOLOv5 式检测 + 20 细类 + 4 大类映射）

基于 `rubbish_2_7.rknn`（20 类垃圾分类模型，**YOLOv5 式检测头**：3 尺度 × 75 通道 = 3 anchors × (4+1+20)），改造 `rknn-python-Multithreading` 火焰检测例程：**多线程 RKNN 推理 + 逐目标分类 + 4 大类映射 + MJPEG 实时流**。

## 模型事实（元数据核实）

| 项 | 值 |
|---|---|
| 输入 | `[1,3,640,640]` float32，mean/std 0/255，**rgb2bgr=False（须喂 RGB）** |
| 输出 | 3 尺度 `[1,75,80,80] / [1,75,40,40] / [1,75,20,20]` |
| 20 细类 | bailuobo … zhuankuai（`config/labels.txt`） |
| 4 大类 | 1=recyclable 2=kitchen_waste 3=hazardous_waste 4=other_waste（`config/category_map.txt`，英文名） |
| anchors | yolov5 默认（`config/anchors.txt`；以导出配置为准，框偏移时调整） |

## 模块（docs/build/ 各含构建文档）

```
01 config      配置/标签/4 大类映射/anchors 加载（路径相对配置文件解析）
02 preprocess  letterbox 640(pad 0) + BGR→RGB
03 postprocess YOLOv5 解码（anchors/obj/逐类 NMS）+ 4 大类映射（参考 func.py 算法）
04 detector    后端注入（rknn/fake）+ 编排（原图坐标输出）
05 rknnpool    多线程推理池（NPU 核轮转，参考 rknnpool.py）
06 mjpeg_server 标准库 MJPEG 流（替代外部 mjpeg_streamer）
07 app         视频/相机 CLI + MJPEG + FPS
```

## 运行

```bash
# 本地单测（WSL rknn23 / 板，62 用例）
pytest course_examples/02_garbage_classification/tests/ -v

# 板上推理（ros2_venv；模型与配置先同步到板，config 里 model.path 改为板载路径）
cd course_examples/02_garbage_classification
python src/app.py --image x.jpg --save out.jpg        # 单图检测
python src/app.py --video test.mp4              # MJPEG: http://<板IP>:8080/garbage_cam.mjpg
python src/app.py --camera 0 --display          # 相机 + 本地窗口
python src/app.py --video t.mp4 --tpes 6 --port 8080

# 相机实时（板上标准：HP60C 深度相机走 ascamera 话题，opencv 直连读不了）
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --ros
#    --camera N 仅作无 ROS 的 UVC 回退（本板 opencv 未编 libv4l，HP60C 直连失败）
```

## 验证边界（用户环境说明）

| 层级 | 位置 | 内容 | 状态 |
|---|---|---|---|
| 纯逻辑单测 | 本地（WSL rknn23，x86） | config/preprocess/postprocess/detector(注入)/pool/server/app —— **从不加载真模型** | ✅ 62/62 |
| rknn 推理集成 | 板上（RK3576，aarch64） | 真模型加载 + NPU 推理 + 相机 + MJPEG + FPS | ⏳ 待你在板上验证（架构差异，PC 无法测） |

## 板上确认清单（验收门）

- [ ] `verify_runtime.py` 全 PASS（rknn-toolkit2 2.3.2）
- [ ] 模型同步：`rubbish_2_7.rknn` → 板；`config/garbage_config.yaml` 的 `model.path` 改为板载路径
- [ ] `pytest tests/` 62 全绿（板）
- [ ] `python src/app.py --video test.mp4`：avg_fps ≥ 15，浏览器 `http://<板IP>:8080/garbage_cam.mjpg` 可见标注流
- [ ] 标注正确：「大类 细类 score」与目标一致（如水瓶 → `recyclable shuiping 0.xx`）
- [ ] `--camera 0` 相机实时可用
- [ ] `--ros` HP60C 话题取流可用（ascamera 驱动先起；ROS_DOMAIN_ID 与驱动一致）
- [ ] 若框偏移 → 调整 `config/anchors.txt`（核对导出配置）；若分数异常 → 切换 `infer.sigmoid: false`

勾选完成后告知我，关闭实验 02，进入实验 03（Lab 色彩空间颜色识别，参照 ascam 资源 + `car1_app` target_color_lab）。
