# Strawberry_car K210: YOLOv2 detect + UART 24B frame + LCD display
DEBUG_LCD = True  # True=K210屏幕显示, False=纯串口

# ==== 启动日志（SD卡 boot.log）====
log_lines = []
def L(msg):
    log_lines.append(msg)
    try:
        with open("/sd/boot.log", "w") as f:
            for line in log_lines:
                f.write(line + "\n")
    except:
        pass

L("boot_ok")

import sensor, image, lcd, time
import KPU as kpu
from machine import UART
import gc, sys
from fpioa_manager import fm
L("imports_ok")

input_size = (224, 224)
labels = ['ripe', 'unripe']
anchors = [1.84, 1.81, 0.5, 0.5, 1.09, 1.06, 0.37, 0.38, 0.63, 0.69]  # z3 anchors

FRAME_HEADER = 0xAA
FRAME_FOOTER = 0x55
FRAME_SIZE = 24
TARGET_SLOTS = 5
TARGET_SIZE = 4
L("vars_ok")

class Comm:
    def __init__(self, uart):
        self.uart = uart
        self.last_send_time = 0

    def compress_x(self, x_raw):
        return int(x_raw * 255 / 320)

    def build_frame(self, objects, labels):
        frame = bytearray(FRAME_SIZE)
        frame[0] = FRAME_HEADER
        count = min(len(objects), TARGET_SLOTS)
        frame[1] = count

        target_list = []
        for obj in objects:
            pos = obj.rect()
            target_list.append({
                'class_id': obj.classid(),
                'x': pos[0] + pos[2] // 2,
                'y': pos[1] + pos[3] // 2,
                'area': pos[2] * pos[3],
                'confidence': obj.value()
            })
        target_list.sort(key=lambda t: t['x'])

        for i in range(TARGET_SLOTS):
            off = 2 + i * TARGET_SIZE
            if i < len(target_list):
                t = target_list[i]
                frame[off] = t['class_id']
                frame[off+1] = self.compress_x(t['x'])
                frame[off+2] = min(t['y'], 240)
                frame[off+3] = min(int(t['area']*255/(320*240)), 255)
            else:
                frame[off] = frame[off+1] = frame[off+2] = frame[off+3] = 0x00

        checksum = frame[1]
        for i in range(2, 22):
            checksum ^= frame[i]
        frame[22] = checksum
        frame[23] = FRAME_FOOTER
        return frame

    def send_detect_result(self, objects, labels):
        ct = time.ticks_ms()
        if ct - self.last_send_time >= 10:
            self.uart.write(self.build_frame(objects, labels))
            self.last_send_time = ct

L("comm_def_ok")

# ==== 主流程 ====
try:
    L("main_start")

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(input_size)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.run(1)
    L("sensor_ok")

    for i in range(20):
        sensor.snapshot()
        time.sleep_ms(30)
    L("skipframes_ok")

    lcd.init(type=1)
    lcd.rotation(0)
    lcd.clear(lcd.BLACK)
    L("lcd_init_ok")

    # 启动画面
    img = image.Image(size=(320, 240))
    img.draw_string(50, 100, "StrawberryCar", color=(0,255,0), scale=3)
    img.draw_string(70, 140, "loading model...", color=(255,255,255), scale=2)
    lcd.display(img)
    L("startup_displayed")

    # UART: IO7=TX, IO6=RX
    fm.register(7, fm.fpioa.UART1_TX, force=True)
    fm.register(6, fm.fpioa.UART1_RX, force=True)
    uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=256)
    comm = Comm(uart)
    L("uart_ok")

    L("loading_model")
    task = kpu.load("/sd/model-291870.kmodel")  # z3
    L("model_loaded")
    kpu.init_yolo2(task, 0.3, 0.3, 5, anchors)
    L("yolo_init_ok")

    prev_objects = []
    no_detect_count = 0
    L("entering_loop")

    while True:
        img = sensor.snapshot()
        t = time.ticks_ms()
        objects = kpu.run_yolo2(task, img)
        t = time.ticks_ms() - t

        if objects:
            filtered_objects = []
            for obj in objects:
                if obj.value() > 0.4:
                    filtered_objects.append(obj)
            if filtered_objects:
                no_detect_count = 0
                prev_objects = filtered_objects
            else:
                no_detect_count += 1
        else:
            no_detect_count += 1

        if objects and filtered_objects:
            display_objects = filtered_objects
        elif no_detect_count <= 5 and prev_objects:
            display_objects = prev_objects
        else:
            display_objects = []

        comm.send_detect_result(display_objects, labels)

        if DEBUG_LCD:
            if display_objects:
                for obj in display_objects:
                    pos = obj.rect()
                    img.draw_rectangle(pos, color=(0, 255, 0))
                    img.draw_string(pos[0], pos[1],
                        "%s:%.2f" % (labels[obj.classid()], obj.value()),
                        scale=2, color=(255, 0, 0))
            img.draw_string(0, 200, "t:%dms" % t, scale=2, color=(255, 0, 0))
            lcd.display(img)

except Exception as e:
    L("CRASH:" + str(e)[:200])
    # 尝试在屏幕上显示错误
    try:
        img = image.Image(size=(320, 240))
        img.draw_string(0, 10, str(e)[:80], scale=1, color=(255,0,0))
        lcd.display(img)
    except:
        pass
finally:
    try:
        with open("/sd/boot.log", "w") as f:
            for line in log_lines:
                f.write(line + "\n")
    except:
        pass
    gc.collect()
