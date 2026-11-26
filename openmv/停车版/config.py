"""H7 Plus 近距离台架测试初值（2026-11-23），不是已完成的实物标定。

依据规则：青绿/奶黄泡沫果，正常果至少55×35mm，绿色树体高410mm。
先按相机离地约300mm、朝向果实、距离约400mm、室内白光补光试拍。
像素尺度估计假定画面横向覆盖约500mm；这不是镜头实测视场。
颜色/曝光/像素参数 origin=assumed，roles=calibration/behavior，maturity=candidate。
权威入口为本文件；改参数后停止并重新运行main.py。
"""

# 图像固定为 QVGA 320×240、RGB565；更改分辨率后重调所有像素参数。
HMIRROR = True
VFLIP = True
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
# 独立标定枝条颜色；None 先用绿果阈值。补光灯开启后用 lab_probe.py 复测。
TREE_LAB = None
ROI = (0, 0, 320, 240)  # 全图搜索；贴真实画面边缘的候选仍按下方余量过滤。
GREEN_ROI_MARGIN_PX = 2  # 绿块外框距ROI四边至少2px才参与选目标；0关闭。
# 平视和仰视样本中，果梗常将绿果与树体连为一块。开运算去掉细连接。
# 4px半径对应9×9核；远处小果被削掉时减小，设0可关闭以对照。
GREEN_OPEN_RADIUS_PX = 4
# 黄果始终检测全图，避免ROI边缘外的黄果漏掉邻近避让。
# 假定横向视场500mm，55×35mm约35×22px；兼顾旋转、遮挡及距离变化，
# 使用较宽的尺寸范围，不把规则的±5%公差直接套成像素阈值。
MIN_PIXELS = 100  # 有效色块像素数；提高可减少噪点，也会漏掉小果/远果。
MIN_AREA = 180  # 外接矩形面积(px²)下限，约13×14px。
MIN_GREEN_SHORT_SIDE_PX = 15  # 剔除台架录像中的微小枝尖；减小可找更远的果。
MIN_GREEN_LONG_SIDE_PX = 19  # QVGA 下略高于空树截图里约16px的枝尖长边。
MAX_AREA = 10000  # 约100×100px上限；近距离果被过滤时增大。
MIN_FILL = 0.40  # pixels/(w*h)；增大过滤稀疏枝条，也会漏遮挡果。
MIN_ASPECT = 0.40  # min(w,h)/max(w,h)，允许长短轴比到2.5。
YELLOW_MIN_PIXELS = 40  # 黄果采用更低门槛，优先保留局部可见的禁区。
YELLOW_MIN_AREA = 60  # px²；小色块过多时逐步提高。
YELLOW_STRIP_MAX_HEIGHT_PX = 6  # 只过滤高度不超过6px且宽至少为高3倍的水平条；设0关闭。
YELLOW_MARGIN_PX = 18  # 两色块外框最近距离(px)；约28mm的视场估计值。
STABLE_FRAMES = 3  # 连续帧数；增大更稳但响应更慢。
TRACK_DISTANCE_PX = 25  # 相邻帧中心最大位移；过大会串目标，过小会断跟踪。
TRACK_MISSED_FRAMES = 1  # 已确认的果可跨一次漏检重新匹配；漏检帧不输出坐标。

# 台架击打资格初值，单位为 QVGA 像素。果框须完全在判断区内。
STRIKE_REGION = (48, 48, 224, 166)
STRIKE_MIN_SHORT_SIDE_PX = 18  # 增大可拦截更多枝块，也可能错过远处小果。
STRIKE_MIN_LONG_SIDE_PX = 26
# 近排台架初值。按既有工作距估计 22×35px，已放行截图的果框从 1107px² 起。
# 像素数指开运算后绿块的像素。装车固定镜头后，用近远两排实测重调。
STRIKE_MIN_PROJECTED_AREA_PX2 = 750
STRIKE_MIN_VISIBLE_PIXELS = 500
STRIKE_CONFIRM_FRAMES = 4  # 连续合格的当前帧；一次漏检即从头确认。
STRIKE_MAX_STEP_PX = 12  # 相邻帧中心最大位移；过快横移时重新确认。
STRIKE_YELLOW_MARGIN_PX = 24  # 击打判断使用更大的图像平面间距。

# 校准脚本无需关闭此项，它不会发送。UART3 对应 H7 Plus 的 P4(TX)、P5(RX)。
UART_ENABLED = True
UART_ID = 3
UART_BAUD = 115200  # 8N1，后续须与Mega端一致。
SEND_PERIOD_MS = 100  # 最大约10Hz；检测慢时实际频率随之降低。
DRAW = True
DRAW_REJECTED_GREEN = True  # 紫框标出通过最小色块门槛、但未通过形状筛选的绿色块。
PRINT_PERIOD_MS = 1000  # IDE日志限频，避免逐帧打印影响帧率。

# QVGA 首轮初值，不是装车标定。左右相机可各用自己的坐标，先核对高低位。
HARVEST_HIGH_NEAR = (20, 20, 280, 95)
HARVEST_LOW_NEAR = (20, 120, 280, 110)
# 停车版发送 HV2。旧双边版使用 HV1，需装回旧方向或重新标定。
HARVEST_STOP_MODE = True
HARVEST_CENTER_X_PX = 160  # 画面 y 轴中心线是 x=160 的竖线。
HARVEST_CENTER_TOLERANCE_PX = 8
# (短边、长边、外框面积、有效绿像素数的下限)，分别测高/低近果和后排果后填写。
# None 允许识别及减速，但不给该高度的出杆许可。
HARVEST_HIGH_CLOSE_MIN = None
HARVEST_LOW_CLOSE_MIN = None
# 旧双边版的移动触发区。
HARVEST_HIGH_FIRE = (130, 35, 60, 72)
HARVEST_LOW_FIRE = (130, 135, 60, 75)
HARVEST_ENTRY_AREA = 750  # 绿、黄果作为减速信号的外框面积初值 px²
HARVEST_TREE_REGION = (20, 15, 280, 215)
HARVEST_TREE_MIN_AREA = 1800  # 大枝干外框面积 px²
HARVEST_TREE_MIN_PIXELS = 240
HARVEST_TREE_MIN_LONG = 55
HARVEST_TREE_MAX_ASPECT = 0.50
HARVEST_TREE_VERY_LARGE = 10000
