"""H7 Plus 基础彩色采图测试：在IDE直接运行，无需其他项目文件。"""

import sensor

sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # 彩色画面，不做二值化或识别。
sensor.set_framesize(sensor.QVGA)  # 320×240。
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)  # 等待自动曝光与白平衡适应环境。

while True:
    sensor.snapshot()  # 原始彩色画面直接显示在 IDE 帧缓冲区。
