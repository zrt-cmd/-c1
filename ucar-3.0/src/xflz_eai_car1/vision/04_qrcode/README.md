# 实验 04 — 二维码生成与识别

对应课程内容（test.md）：**生成（版本/纠错/尺寸/保存）→ 实时检测 → 框选识别区域 → 解析打印文本**。

## 模块（docs/build/ 各含构建文档）

```
01 config    生成参数（version 1-40 / 纠错 L/M/Q/H / box_size / border / 颜色）+ 检测/相机
02 qr        生成（qrcode 库，参考 QRcode_Create.py）→ 检测（OpenCV QRCodeDetector，零权重）
03 app       --generate 存图 / --detect 图像·视频·相机实时识别 + MJPEG 流
```

## 依赖

- **唯一新增：`qrcode` 7.4.2 + pypng**（纯 Python，参考代码同款；板 venv 亦缺）
  - 本地已装（rknn23，清华镜像）；**板上安装**：
    `pip install -i https://pypi.tuna.tsinghua.edu.cn/simple qrcode`
- 检测用 OpenCV 内置 `cv2.QRCodeDetector`——**零新增、无需模型权重**（替代参考的 pyzbar，其需系统库 libzbar0）

## 运行

```bash
# 本地单测（rknn23 / 板，19 用例）
pytest course_examples/04_qrcode/tests/ -v

# 生成（课程内容 1）
python src/app.py --generate "https://example.com" --save my_qr.png
# 识别（课程内容 2/3）
python src/app.py --detect --image my_qr.png          # 打印解析文本
python src/app.py --detect --camera 0                 # 实时 + MJPEG: http://<板IP>:8080/qrcode.mjpg
python src/app.py --detect --video test.mp4
# 板上相机标准（HP60C 深度相机走 ascamera 话题）：
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
python src/app.py --detect --ros                      # ROS2 话题取流 + MJPEG
```

## 板上确认清单（验收门）

- [ ] `pip install qrcode`（清华镜像）成功
- [ ] `--generate "你好" --save qr.png` 生成成功，手机/电脑可扫出内容
- [ ] `--detect --image qr.png` 打印解析文本
- [ ] `--camera 0` 或 `--ros` 实时识别手机屏幕/打印的二维码，打印文本、框选正确
- [ ] MJPEG 流 `http://<板IP>:8080/qrcode.mjpg` 可见（--camera 或 --ros）
- [ ] 全部勾选 → 关闭实验 04，进入实验 05（自主巡线）
