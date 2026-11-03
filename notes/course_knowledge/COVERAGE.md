# 覆盖范围与资料缺口

抓取日期 2026-11-02（Asia/Shanghai）。入口为课程首页。递归跟踪同站点 course_generate_agent 路径内的 HTML 链接和 iframe。

- HTML 页面成功 183，包括全部 107 个实验入口。
- 同站点附件成功 208，失败 30。
- 提取正文共 1,256,613 个字符。
- HTML、正文、代码块、章节锚点、来源和 SHA-256 均保留。
- 交互演示的 HTML / JS 已保存。图像已保存，截图内容和动画行为尚未逐项人工验读。
- 外站链接仅登记，不递归扩展到厂商文档、GitHub、商品网站。
- 未被站内链接引用的隐藏页面不属于本次可发现范围。
- 收录正文代表资料可检索。人工核读聚焦课程原理、接口、实操和易错点，未进行 107 个实验的硬件验证。

## 建设中的实验

- [EXP-02 视觉识别与抓取 · 建设中 · EXP007](https://aibittek.github.io/course_generate_agent/courses/cases/exp007-intelligent-factory/exp-02-vision-grasp/index.html)
- [EXP-03 语音交互与大模型 · 建设中 · EXP007](https://aibittek.github.io/course_generate_agent/courses/cases/exp007-intelligent-factory/exp-03-voice-llm/index.html)
- [EXP-04 综合闭环实战 · 建设中 · EXP007](https://aibittek.github.io/course_generate_agent/courses/cases/exp007-intelligent-factory/exp-04-full-loop/index.html)

## 附件访问失败

以下均经重新请求确认返回 HTTP 404。内容无法读取，不能当成已经观看的视频或已有图片。

- [courses/core/exp003-ros2-application/exp-14-rqt/images/case-rqt-topic.png](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-14-rqt/images/case-rqt-topic.png)
- [media/cases/exp007-intelligent-factory/exp-01-slam-nav/videos/demo-slam-mapping.mp4](https://aibittek.github.io/course_generate_agent/media/cases/exp007-intelligent-factory/exp-01-slam-nav/videos/demo-slam-mapping.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Gripper_v2.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Gripper_v2.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint1_v2.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint1_v2.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint2_v2.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint2_v2.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint3_v2.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint3_v2.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint4_v2.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint4_v2.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint5_v2.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/Joint5_v2.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/主臂安装.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/主臂安装.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/从臂安装.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/从臂安装.mp4)
- [media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/设置电机编号.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-02-arm-assembly/videos/设置电机编号.mp4)
- [media/industry/ind01-endtoend-platform/exp-03-arm-calibration/videos/主臂校准讲解版.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-03-arm-calibration/videos/主臂校准讲解版.mp4)
- [media/industry/ind01-endtoend-platform/exp-03-arm-calibration/videos/从臂校准日志版.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-03-arm-calibration/videos/从臂校准日志版.mp4)
- [media/industry/ind01-endtoend-platform/exp-03-arm-calibration/videos/从臂校准讲解版.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-03-arm-calibration/videos/从臂校准讲解版.mp4)
- [media/industry/ind01-endtoend-platform/exp-04-teleop-checkout/videos/单臂遥操作.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-04-teleop-checkout/videos/单臂遥操作.mp4)
- [media/industry/ind01-endtoend-platform/exp-04-teleop-checkout/videos/双臂遥操作.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind01-endtoend-platform/exp-04-teleop-checkout/videos/双臂遥操作.mp4)
- [media/industry/ind02-act-vla-pipeline/exp-01-teleop-recording/videos/单臂遥操作.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind02-act-vla-pipeline/exp-01-teleop-recording/videos/单臂遥操作.mp4)
- [media/industry/ind02-act-vla-pipeline/exp-01-teleop-recording/videos/双臂遥操作.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind02-act-vla-pipeline/exp-01-teleop-recording/videos/双臂遥操作.mp4)
- [media/industry/ind03-material-sorting/exp-01-case-overview/videos/factory-sorting-demo.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind03-material-sorting/exp-01-case-overview/videos/factory-sorting-demo.mp4)
- [media/industry/ind04-home-tidying/exp-01-case-overview/videos/home-tidying-demo.mp4](https://aibittek.github.io/course_generate_agent/media/industry/ind04-home-tidying/exp-01-case-overview/videos/home-tidying-demo.mp4)
- [media/products/b2-box/videos/b2-feature-intro.mp4](https://aibittek.github.io/course_generate_agent/media/products/b2-box/videos/b2-feature-intro.mp4)
- [media/products/c1-car/videos/c1-nav-grasp-demo.mp4](https://aibittek.github.io/course_generate_agent/media/products/c1-car/videos/c1-nav-grasp-demo.mp4)
- [media/products/ldt-01/videos/demo-factory-sort.mp4](https://aibittek.github.io/course_generate_agent/media/products/ldt-01/videos/demo-factory-sort.mp4)
- [media/products/ldt-01/videos/demo-home-tidy.mp4](https://aibittek.github.io/course_generate_agent/media/products/ldt-01/videos/demo-home-tidy.mp4)
- [media/products/unitree-g1/videos/g1-guide-demo.mp4](https://aibittek.github.io/course_generate_agent/media/products/unitree-g1/videos/g1-guide-demo.mp4)
- [media/products/unitree-go2/videos/go2-inspection-demo.mp4](https://aibittek.github.io/course_generate_agent/media/products/unitree-go2/videos/go2-inspection-demo.mp4)
- [media/products/xlerobot/videos/act.mp4](https://aibittek.github.io/course_generate_agent/media/products/xlerobot/videos/act.mp4)
- [media/products/xlerobot/videos/pi05-desk.mp4](https://aibittek.github.io/course_generate_agent/media/products/xlerobot/videos/pi05-desk.mp4)
- [media/products/xlerobot/videos/pi05.mp4](https://aibittek.github.io/course_generate_agent/media/products/xlerobot/videos/pi05.mp4)
- [media/products/xlerobot/videos/smolvla.mp4](https://aibittek.github.io/course_generate_agent/media/products/xlerobot/videos/smolvla.mp4)

## 外部参考链接（未递归收录）

- https://fonts.googleapis.com/css2
- https://www.unitree.com/cn/g1
- https://support.unitree.com/home/zh/G1_developer
- https://aiui-doc.xf-yun.com/project-2/doc-756/index.html
- https://www.unitree.com/cn/go2
- https://support.unitree.com/home/zh/Go2_developer
- https://www.livoxtech.com/cn/mid-360/specs
- https://www.realsenseai.com/products/depth-camera-d435i
- https://xlerobot.readthedocs.io/zh-cn/latest/index.html
- https://github.com/Vector-Wangel/XLeRobot
- https://datasheet.sisoog.com/MQ-5/file/7a657573/datasheet/79701185ea1f27506c945c417b151c44449520e2.pdf
- https://www.lcsc.com/datasheet/C99297.pdf
- https://fonts.gstatic.com
