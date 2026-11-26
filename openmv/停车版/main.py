"""OpenMV 单边停车疏果。在 OpenMV IDE 中运行。"""

import sensor
import time
import config as cfg
from harvest_camera import setup_camera, detect_frame, draw
from protocol import encode_harvest
from harvest_vision import HarvestVision


def main():
    uart = None
    if cfg.UART_ENABLED:
        from pyb import UART
        uart = UART(cfg.UART_ID, cfg.UART_BAUD, bits=8, parity=None, stop=1,
                    timeout=10, timeout_char=2)
    detector = HarvestVision(cfg)
    clock = time.clock()
    seq = 0
    last_send = time.ticks_ms()
    last_print = last_send
    try:
        setup_camera()
        while True:
            clock.tick()
            img = sensor.snapshot()
            greens, yellows, tree = detect_frame(img, detector)
            near, height, aligned, reason, target = detector.update(greens, yellows, tree)
            now = time.ticks_ms()
            if uart is not None and time.ticks_diff(now, last_send) >= cfg.SEND_PERIOD_MS:
                packet = encode_harvest(seq, near, height, aligned, cfg.HARVEST_STOP_MODE)
                if uart.write(packet) != len(packet):
                    raise OSError("UART short write")
                seq = (seq + 1) & 0xFFFF
                last_send = now
            if cfg.DRAW:
                draw(img, greens, yellows, target, near, height, aligned, reason)
            if time.ticks_diff(now, last_print) >= cfg.PRINT_PERIOD_MS:
                print("HV tree=%d near=%d target=%s aligned=%d reason=%s fps=%.1f G=%s Y=%s" % (
                    tree, near, height, aligned, reason, clock.fps(),
                    [blob[2:] for blob in greens], [blob[2:] for blob in yellows]))
                last_print = now
    except Exception as exc:
        if uart is not None:
            try:
                uart.write(encode_harvest(seq, False, "N", False, cfg.HARVEST_STOP_MODE))
            except Exception:
                pass
        print("vision stopped:", exc)
        raise


if __name__ == "__main__":
    main()
