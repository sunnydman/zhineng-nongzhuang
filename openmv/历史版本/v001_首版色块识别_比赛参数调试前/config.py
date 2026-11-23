"""H7 Plus 初版参数。以下数值仅供搭建流程，均需用实物重新标定。"""

# 图像固定为 QVGA 320×240、RGB565；更改分辨率后重调所有像素参数。
HMIRROR = False
VFLIP = False
WARMUP_MS = 2000
# 启动自动收敛后锁定；填入数值可固定曝光(us)、增益(dB)、白平衡RGB增益(dB)。
# None 会使用本次启动收敛值，不保证不同启动间颜色一致。
EXPOSURE_US = None
GAIN_DB = None
RGB_GAIN_DB = None

# LAB: (L_min,L_max,A_min,A_max,B_min,B_max)，L范围0~100，A/B范围-128~127。
# 占位阈值！绿树也可能命中绿果阈值，必须在IDE阈值编辑器中重选。
GREEN_LAB = (20, 85, -65, -12, 5, 65)
YELLOW_LAB = (35, 100, -12, 25, 25, 100)
CALIBRATED = False  # 真实样本验证后才改True；不代表机构可以自动动作。
ROI = None  # None=全图；或(x,y,w,h)，单位像素，必须处于320×240内。
MIN_PIXELS = 80  # 提高可减少噪点，也会漏掉小果/远果。
MIN_AREA = 120  # 外接矩形面积(px²)下限。
MAX_AREA = 18000  # 增大可容纳近果，也可能把整片树体纳入。
MIN_FILL = 0.30  # 色块像素数/外接矩形面积；增大可过滤细枝，也会漏遮挡果。
MIN_ASPECT = 0.35  # min(w,h)/max(w,h)，增大可过滤细长枝条。
YELLOW_MARGIN_PX = 15  # 黄果外框向四周扩展，增大更保守；不是厘米安全距离。
STABLE_FRAMES = 3  # 连续帧数；增大更稳但响应更慢。
TRACK_DISTANCE_PX = 25  # 相邻帧中心最大位移；过大会串目标，过小会断跟踪。

# 默认只看IDE画面。UART3对应H7 Plus的P4(TX)、P5(RX)。
UART_ENABLED = False
UART_ID = 3
UART_BAUD = 115200  # 8N1，后续须与Mega端一致。
SEND_PERIOD_MS = 100  # 最大约10Hz；检测慢时实际频率随之降低。
DRAW = True
PRINT_PERIOD_MS = 1000  # IDE日志限频，避免逐帧打印影响帧率。
