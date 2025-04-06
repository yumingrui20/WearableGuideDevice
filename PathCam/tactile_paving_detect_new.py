import sensor, image, time, lcd, gc, cmath, binascii
from maix import KPU
from modules import ybserial

# 初始化串口模块
serial = ybserial()

# 字符串转10进制（例如 "3" -> 0x33 -> 51）
def str_int(data_str):
    bb = binascii.hexlify(data_str)
    bb = str(bb)[2:-1]
    hex_1 = int(bb[0]) * 16
    hex_2 = int(bb[1], 16)
    return hex_1 + hex_2

# 发送数据的协议构造
def send_data(x, y, w, h, msg):
    start = 0x24
    end = 0x23
    length = 5
    class_num = 0x01
    class_group = 0xBB
    fenge = 0x2c
    crc = 0
    data = []

    for val in [x, y, w, h]:
        low = val & 0xFF
        high = val >> 8 & 0xFF
        data.extend([low, fenge, high, fenge])

    if msg is not None:
        msg_val = str_int(msg)
        data.append(msg_val)
        data.append(fenge)

    data_num = len(data)
    length += data_num

    send_buf = [length, class_num, class_group, data_num] + data
    crc = sum(send_buf) % 256
    send_buf.insert(0, start)
    send_buf.append(crc)
    send_buf.append(end)

    serial.send_bytearray(send_buf)
    print("[SEND]:", send_buf)

# 初始化LCD与摄像头
lcd.init()
lcd.clear(lcd.RED)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)
clock = time.clock()

print("ready load model")

labels = ["straight_path", "dot_path", "green_light", "red_light"]
anchor = (1.09, 2.00, 1.56, 3.31, 3.22, 3.81, 6.83, 1.89, 3.11, 7.41)

kpu = KPU()
kpu.load_kmodel("/sd/KPU/tactile_paving_detect/model-15539.kmodel")
kpu.init_yolo2(anchor, anchor_num=(int)(len(anchor)/2), img_w=320, img_h=240, net_w=320, net_h=240, layer_w=10, layer_h=8, threshold=0.6, nms_value=0.3, classes=len(labels))

while True:
    gc.collect()
    clock.tick()
    start_time = time.ticks_ms()

    img = sensor.snapshot()
    kpu.run_with_output(img)
    dect = kpu.regionlayer_yolo2()

    inference_time = time.ticks_ms() - start_time
    fps = clock.fps()

    if len(dect) > 0:
        for l in dect:
            x, y, w, h = l[0], l[1], l[2], l[3]
            class_id = l[4]
            confidence = l[5]
            label = labels[class_id]
            msg_code = str(class_id + 1)  # 对应发送编号 "1~4"

            # 不同标签不同颜色
            if label == "straight_path":
                color = (255, 0, 0)
            elif label == "dot_path":
                color = (0, 255, 0)
            elif label == "green_light":
                color = (0, 255, 255)
            elif label == "red_light":
                color = (255, 255, 0)
            else:
                color = (255, 255, 255)

            # 绘制框与文字
            img.draw_rectangle(x, y, w, h, color=color)
            img.draw_string(x, y, label, color=color, scale=1.5)
            img.draw_string(x, y + 18, "%.2f  %dms" % (confidence, inference_time), color=color, scale=1.5)

            # 发送串口数据
            send_data(x, y, w, h, msg_code)

    img.draw_string(0, 0, "%2.1f fps" % fps, color=(0, 60, 255), scale=2.0)
    lcd.display(img)
