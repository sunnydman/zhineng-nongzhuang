# 台架击打判断

`strike_decision.py` 逐帧判断当前绿果是否符合台架击打条件。OpenMV 在 IDE 画面和串口终端显示状态。青框是像素判断区，初值见 [config.py](../当前版本/config.py)。

| 显示 | 含义 |
| --- | --- |
| `HIT NONE` | 当前帧没有可确认的绿果 |
| `HIT TRACK` | 视觉仍在跟踪确认 |
| `HIT ZONE` | 果框尚未完整进入青框 |
| `HIT SIZE` | 色块尺寸低于击打判断门槛 |
| `HIT YELLOW` | 黄果在避让范围内 |
| `HIT MULTI` | 当前有多个绿块候选 |
| `HIT WAIT` | 条件初步满足，正在连续确认 |
| `HIT ALLOW` | 当前目标已连续四帧通过判断 |

`ALLOW` 是当前帧的判断状态。丢失目标、离开判断区或出现黄果等情况时，立即撤回并清除计数。目标位置或面积跳变后重新确认。判断所用的青框和黄果距离均为图像像素初值，装车后要按相机视角与机构位置重新标定。

## IDE 验证

1. 把 `strike_decision.py` 和 [当前版本](../当前版本)中的 `config.py`、`vision.py`、`protocol.py` 复制到相机盘根目录。
2. 在 IDE 打开当前版本的 `main.py` 临时运行，确认终端显示 `OPENMV v010`，并保持 `UART_ENABLED = False`。
3. 先横移拍空树，再让单绿果缓慢进出青框。空树全程应无 `ALLOW`，完整进入青框并稳定后可以出现 `ALLOW`，离开时应撤回。
4. 再把黄果挂到绿果附近，以 `HIT YELLOW` 核对避让结果。

桌面验证命令为 `python -m unittest discover -s "openmv/击打判断" -v`。
