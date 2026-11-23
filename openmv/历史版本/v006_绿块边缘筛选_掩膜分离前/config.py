"""H7 Plus 近距离台架测试初值（2026-11-23），不是已完成的实物标定。

依据规则：青绿/奶黄泡沫果，正常果至少55×35mm，绿色树体高410mm。
先按相机离地约300mm、朝向果实、距离约400mm、室内白光补光试拍。
像素尺度估计假定画面横向覆盖约500mm；这不是镜头实测视场。
颜色/曝光/像素参数 origin=assumed，roles=calibration/behavior，maturity=candidate。
权威入口为本文件；改参数后停止并重新运行main.py。
"""

# 图像固定为 QVGA 320×240、RGB565；更改分辨率后重调所有像素参数。
HMIRROR = False
VFLIP = False
WARMUP_MS = 2000
AUTO_CAMERA = True  # 台架预览使用自动曝光、增益及白平衡。
# 以下三项仅在 AUTO_CAMERA=False 时生效，需按实际传感器和光照标定。
# 暗：先增加补光，再把曝光调至15000/20000us；亮：降至8000/5000us。
# 曝光越长运动拖影越大；增益建议先在0~12dB内试，过高会放大噪点。
# RGB初值为中性通道增益，设备实际色偏需用白/灰卡重新调，不能视为标准白平衡。
EXPOSURE_US = 10000
GAIN_DB = 6.0
RGB_GAIN_DB = (0.0, 0.0, 0.0)

# LAB: (L_min,L_max,A_min,A_max,B_min,B_max)，L范围0~100，A/B范围-128~127。
# 亮绿和浅青绿漏检时，放宽L上限85→95、A上限-15→-12、B下限10→-8。
# 台架试调值。浅绿与黄阈值可能少量重叠，现有黄果避让会拦截重叠候选。
GREEN_LAB = (25, 95, -60, -12, -8, 65)
YELLOW_LAB = (45, 100, -15, 20, 20, 85)
ROI = (16, 12, 288, 216)  # 绿果检测区：左右各留16px、上下各留12px。
GREEN_ROI_MARGIN_PX = 2  # 绿块外框距ROI四边至少2px才参与选目标；0关闭。
# 黄果始终检测全图，避免ROI边缘外的黄果漏掉邻近避让。
# 假定横向视场500mm，55×35mm约35×22px；兼顾旋转、遮挡及距离变化，
# 使用较宽的尺寸范围，不把规则的±5%公差直接套成像素阈值。
MIN_PIXELS = 100  # 有效色块像素数；提高可减少噪点，也会漏掉小果/远果。
MIN_AREA = 180  # 外接矩形面积(px²)下限，约13×14px。
MAX_AREA = 10000  # 约100×100px上限；近距离果被过滤时增大。
MIN_FILL = 0.40  # pixels/(w*h)；增大过滤稀疏枝条，也会漏遮挡果。
MIN_ASPECT = 0.40  # min(w,h)/max(w,h)，允许长短轴比到2.5。
YELLOW_MIN_PIXELS = 40  # 黄果采用更低门槛，优先保留局部可见的禁区。
YELLOW_MIN_AREA = 60  # px²；小色块过多时逐步提高。
YELLOW_MARGIN_PX = 18  # 图像外框扩展量；按上述假定视场约28mm，非实测安全距。
STABLE_FRAMES = 3  # 连续帧数；增大更稳但响应更慢。
TRACK_DISTANCE_PX = 25  # 相邻帧中心最大位移；过大会串目标，过小会断跟踪。

# 默认只看IDE画面。UART3对应H7 Plus的P4(TX)、P5(RX)。
UART_ENABLED = False
UART_ID = 3
UART_BAUD = 115200  # 8N1，后续须与Mega端一致。
SEND_PERIOD_MS = 100  # 最大约10Hz；检测慢时实际频率随之降低。
DRAW = True
DRAW_REJECTED_GREEN = True  # 紫框标出通过最小色块门槛、但未通过形状筛选的绿色块。
PRINT_PERIOD_MS = 1000  # IDE日志限频，避免逐帧打印影响帧率。
