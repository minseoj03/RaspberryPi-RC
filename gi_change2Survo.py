
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raspberry Pi 4 data collection / manual driving program
Hardware:
- MG90S continuous-rotation servos: left BCM GPIO18, right BCM GPIO14
- I2C: MLX90614 (0x5A), MAX30105 (0x57), MPU6050 (0x68), VL53L0X (0x29)
- CSI camera: OV5647 (normally exposed through /dev/video0 when V4L2 is available)

Important:
1) Continuous-rotation servo stop pulses differ by unit. Calibrate SERVO_STOP_US
   with the wheels lifted before placing the car on the floor.
2) Power servos from a suitable external regulated 5V supply, not the Pi 5V pin.
   Connect the external supply GND and Raspberry Pi GND together.
3) VL53L0X is optional in this program. If it has no power / does not appear at
   0x29, software cannot make it appear.
4) This code has not been tested on the user's physical car.
"""

import os
import sys
import time
import math
import pickle
import subprocess

import cv2 as cv
import numpy as np
import RPi.GPIO as GPIO

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# ----------------------------- User hardware settings -----------------------------
I2C_BUS_NUMBER = 1

SERVO_LEFT_GPIO = 18       # BCM numbering
SERVO_RIGHT_GPIO = 14      # BCM numbering
SERVO_FREQUENCY = 50       # Continuous-rotation servo control frequency

# Calibrate these values with the wheels lifted.
# If a wheel turns while the command is 0, adjust that side's STOP value
# in small steps (e.g. 1490, 1495, 1505, 1510).
SERVO_LEFT_STOP_US = 1500
SERVO_RIGHT_STOP_US = 1500

# Increase/decrease these if the servo direction is reversed or too fast.
SERVO_LEFT_REVERSE = False
SERVO_RIGHT_REVERSE = True

SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_MAX_OFFSET_US = 350   # limits command range; tune after safe testing

# Camera. On Raspberry Pi OS Bullseye, CSI camera availability through OpenCV
# depends on the camera/V4L2 configuration.
CAMERA_INDEX = 0
VIDEO_X = 640
VIDEO_Y = 480

# ----------------------------- Original gi.py collection settings -------------------
WIN_GAP_X = 200
WIN_GAP_Y = 20
WIN_XL_ORG = 0
WIN_XR_ORG = 639
WIN_YU_ORG = 246
WIN_YD_ORG = 459
WIN_XL = WIN_XL_ORG
WIN_XR = WIN_XR_ORG
WIN_YU = WIN_YU_ORG
WIN_YD = WIN_YD_ORG

fileNF = "P"
fileId = 0
DIRECTORY_FLAG = False
imageDir = "Exercise Mode"
filePath = ""

defForSpeed = 60
defMotorAngle = 0.15
defRecTime = 500_000_000
recUnitTime = 100_000_000
REC_LOW_FORWARD_SPEED = 40
VARIABLE_MODE = True
recTimeList = [recUnitTime * i for i in range(1, 12)]

recordOn = False
oneShot = False
mouseCtrlState = False
mouseX = 320
mouseY = 420
cursorColor = (0, 255, 255)
mL = 0
mR = 0

# Existing pickle compatibility: [WIN_XL, WIN_XR, WIN_YU, WIN_YD,
#                                  AUTO_LIGHT, MANUAL_LIGHT, fileId]
# The original lamp hardware is not part of the currently confirmed hardware.
# Keep the two fields for compatibility, but do not drive a lamp or read a CdS.
AUTO_LIGHT = True
MANUAL_LIGHT = False

# Colors are BGR (OpenCV)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
CYAN = (255, 255, 0)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)
GRAY = (32, 32, 32)
BLACK = (0, 0, 0)

# ----------------------------- I2C sensor support -----------------------------------
class SensorMonitor:
    ADDRESSES = {
        "VL53L0X": 0x29,
        "MAX30105": 0x57,
        "MLX90614": 0x5A,
        "MPU6050": 0x68,
    }

    def __init__(self):
        self.bus = None
        self.found = {}
        self.values = {}
        self.last_update = 0.0
        if SMBus is None:
            print("[I2C] smbus2 is not installed. Sensor reading is disabled.")
            return
        try:
            self.bus = SMBus(I2C_BUS_NUMBER)
            self.scan()
            self.initialize_sensors()
        except Exception as exc:
            print(f"[I2C] Could not open I2C bus {I2C_BUS_NUMBER}: {exc}")

    def scan(self):
        if self.bus is None:
            return
        for name, address in self.ADDRESSES.items():
            try:
                self.bus.read_byte(address)
                self.found[name] = True
            except Exception:
                self.found[name] = False
        print("[I2C] Detected:", ", ".join(
            f"{name}=0x{addr:02X}" for name, addr in self.ADDRESSES.items()
            if self.found.get(name)
        ) or "no known sensor addresses")
        if not self.found.get("VL53L0X"):
            print("[I2C] VL53L0X (0x29) not detected; check its power and wiring.")

    def write_byte(self, address, register, value):
        if self.bus:
            self.bus.write_byte_data(address, register, value)

    def read_word_be(self, address, register):
        data = self.bus.read_i2c_block_data(address, register, 2)
        return (data[0] << 8) | data[1]

    @staticmethod
    def signed16(value):
        return value - 65536 if value >= 32768 else value

    def initialize_sensors(self):
        # Wake MPU6050.
        if self.found.get("MPU6050"):
            try:
                self.write_byte(0x68, 0x6B, 0x00)
                time.sleep(0.05)
            except Exception as exc:
                print("[MPU6050] Initialization warning:", exc)

        # Basic MAX30105 setup: reset, FIFO clear, SpO2/HR mode and LED currents.
        if self.found.get("MAX30105"):
            try:
                self.write_byte(0x57, 0x09, 0x40)  # reset
                time.sleep(0.1)
                self.write_byte(0x57, 0x04, 0x00)  # FIFO write pointer
                self.write_byte(0x57, 0x05, 0x00)  # overflow counter
                self.write_byte(0x57, 0x06, 0x00)  # FIFO read pointer
                self.write_byte(0x57, 0x08, 0x4F)  # sample averaging / rollover
                self.write_byte(0x57, 0x09, 0x03)  # SpO2 mode
                self.write_byte(0x57, 0x0A, 0x27)  # sample rate / pulse width config
                self.write_byte(0x57, 0x0C, 0x24)  # red LED current
                self.write_byte(0x57, 0x0D, 0x24)  # IR LED current
            except Exception as exc:
                print("[MAX30105] Initialization warning:", exc)

    def update(self):
        if self.bus is None or time.monotonic() - self.last_update < 0.25:
            return
        self.last_update = time.monotonic()

        if self.found.get("MLX90614"):
            try:
                ambient = self.read_word_be(0x5A, 0x06) * 0.02 - 273.15
                object_temp = self.read_word_be(0x5A, 0x07) * 0.02 - 273.15
                self.values["MLX90614"] = (
                    f"Ambient {ambient:.1f}C / Object {object_temp:.1f}C"
                )
            except Exception as exc:
                self.values["MLX90614"] = f"read error: {exc}"

        if self.found.get("MPU6050"):
            try:
                raw = self.bus.read_i2c_block_data(0x68, 0x3B, 14)
                ax = self.signed16((raw[0] << 8) | raw[1]) / 16384.0
                ay = self.signed16((raw[2] << 8) | raw[3]) / 16384.0
                az = self.signed16((raw[4] << 8) | raw[5]) / 16384.0
                gx = self.signed16((raw[8] << 8) | raw[9]) / 131.0
                gy = self.signed16((raw[10] << 8) | raw[11]) / 131.0
                gz = self.signed16((raw[12] << 8) | raw[13]) / 131.0
                self.values["MPU6050"] = (
                    f"A({ax:.2f},{ay:.2f},{az:.2f})g "
                    f"G({gx:.1f},{gy:.1f},{gz:.1f})dps"
                )
            except Exception as exc:
                self.values["MPU6050"] = f"read error: {exc}"

        if self.found.get("MAX30105"):
            try:
                # Read one FIFO sample (3 bytes red + 3 bytes IR).
                fifo = self.bus.read_i2c_block_data(0x57, 0x07, 6)
                red = ((fifo[0] << 16) | (fifo[1] << 8) | fifo[2]) & 0x3FFFF
                ir = ((fifo[3] << 16) | (fifo[4] << 8) | fifo[5]) & 0x3FFFF
                self.values["MAX30105"] = f"Red {red} / IR {ir}"
            except Exception as exc:
                self.values["MAX30105"] = f"read error: {exc}"

        if self.found.get("VL53L0X"):
            # Address detection is confirmed, but distance reading requires a
            # VL53L0X driver/API that varies by library and board package.
            self.values["VL53L0X"] = "detected (distance driver not configured)"
        else:
            self.values["VL53L0X"] = "not detected - check power/wiring"

    def status_lines(self):
        self.update()
        lines = []
        for name in ("VL53L0X", "MAX30105", "MLX90614", "MPU6050"):
            if not self.found.get(name):
                lines.append(f"{name}: NOT FOUND")
            else:
                lines.append(f"{name}: {self.values.get(name, 'detected')}")
        return lines

    def close(self):
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass


# ----------------------------- Servo motor control ----------------------------------
class ServoDrive:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_LEFT_GPIO, GPIO.OUT)
        GPIO.setup(SERVO_RIGHT_GPIO, GPIO.OUT)
        self.left = GPIO.PWM(SERVO_LEFT_GPIO, SERVO_FREQUENCY)
        self.right = GPIO.PWM(SERVO_RIGHT_GPIO, SERVO_FREQUENCY)
        self.left.start(0)
        self.right.start(0)
        self.run(0, 0)

    @staticmethod
    def pulse_to_duty(pulse_us):
        return (pulse_us / 20000.0) * 100.0

    def set_servo(self, pwm, command, stop_us, reverse):
        command = max(-100, min(100, int(command)))
        if reverse:
            command = -command
        if command == 0:
            pulse = stop_us
        else:
            offset = int((command / 100.0) * SERVO_MAX_OFFSET_US)
            pulse = stop_us + offset
        pulse = max(SERVO_MIN_US, min(SERVO_MAX_US, pulse))
        pwm.ChangeDutyCycle(self.pulse_to_duty(pulse))

    def run(self, left_value, right_value):
        self.set_servo(self.left, left_value, SERVO_LEFT_STOP_US, SERVO_LEFT_REVERSE)
        self.set_servo(self.right, right_value, SERVO_RIGHT_STOP_US, SERVO_RIGHT_REVERSE)

    def stop(self):
        self.run(0, 0)
        time.sleep(0.15)

    def close(self):
        try:
            self.stop()
            self.left.stop()
            self.right.stop()
        except Exception:
            pass


# ----------------------------- Camera / display --------------------------------------
def open_camera():
    cam = cv.VideoCapture(CAMERA_INDEX, cv.CAP_V4L2)
    if not cam.isOpened():
        cam.release()
        cam = cv.VideoCapture(CAMERA_INDEX)
    if cam.isOpened():
        cam.set(cv.CAP_PROP_FRAME_WIDTH, VIDEO_X)
        cam.set(cv.CAP_PROP_FRAME_HEIGHT, VIDEO_Y)
    return cam


def controlMain(event, x, y, flags, param):
    global mouseX, mouseY, recordOn, cursorColor, mouseCtrlState, oneShot
    global drive
    mouseX, mouseY = x, y
    if event == cv.EVENT_LBUTTONDOWN:
        if not mouseCtrlState:
            mouseCtrlState = True
            cursorColor = GREEN
        elif DIRECTORY_FLAG:
            recordOn = True
            cursorColor = RED
    elif event == cv.EVENT_RBUTTONDOWN:
        mouseCtrlState = False
        recordOn = False
        cursorColor = YELLOW
        drive.stop()
    elif event == cv.EVENT_MBUTTONDOWN:
        oneShot = True


def main():
    global WIN_XL, WIN_XR, WIN_YU, WIN_YD, fileId
    global VARIABLE_MODE, oneShot, recordOn, mL, mR, cursorColor
    global mouseCtrlState, mouseX, mouseY, drive

    drive = ServoDrive()
    sensors = SensorMonitor()
    cam = open_camera()

    if not cam.isOpened():
        drive.close()
        sensors.close()
        GPIO.cleanup()
        raise RuntimeError(
            "카메라를 열 수 없습니다. `libcamera-hello --list-cameras`와 "
            "`ls /dev/video*`로 OV5647 인식 및 V4L2 장치 여부를 확인하세요."
        )

    cv.namedWindow("Out", cv.WINDOW_NORMAL)
    cv.resizeWindow("Out", VIDEO_X, VIDEO_Y)
    cv.setMouseCallback("Out", controlMain)

    menu_view = True
    pre_key = ord(" ")
    record_time = defRecTime
    forward_speed = defForSpeed
    time_memory = cv.getTickCount()

    print("[INFO] Arrow keys: move / Space: stop")
    print("[INFO] Mouse: left click drive/record, right click stop, middle click one photo")
    print("[INFO] ESC or X: exit")
    print("[SAFETY] Test with wheels lifted. Servo stop pulse may need calibration.")

    try:
        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                cv.waitKey(20)
                continue
            frame = cv.resize(frame, (VIDEO_X, VIDEO_Y))
            view = frame.copy()

            # Sensor status panel
            sensor_lines = sensors.status_lines()
            panel_h = 25 + 22 * len(sensor_lines)
            cv.rectangle(view, (5, 5), (VIDEO_X - 5, panel_h), (25, 25, 25), -1)
            for i, line in enumerate(sensor_lines):
                color = GREEN if "NOT FOUND" not in line and "not detected" not in line else (0, 150, 255)
                cv.putText(view, line[:85], (12, 25 + i * 22),
                           cv.FONT_HERSHEY_SIMPLEX, 0.43, color, 1, cv.LINE_AA)

            road_img = view[WIN_YU:WIN_YD, WIN_XL:WIN_XR]
            if road_img.size == 0:
                road_img = view[246:459, 0:639]
            road_img = cv.resize(road_img, (200, 66))
            road_img = cv.cvtColor(road_img, cv.COLOR_BGR2YUV)
            road_img = cv.GaussianBlur(road_img, (3, 3), 0)

            angle = int(mouseX - VIDEO_X / 2)
            steering = int(angle * defMotorAngle)
            mL = max(-100, min(100, forward_speed + steering))
            mR = max(-100, min(100, forward_speed - steering))
            if mouseCtrlState:
                drive.run(mL, mR)

            g = int(angle / 3.6)
            cv.rectangle(view, (WIN_XL, WIN_YU), (WIN_XR, WIN_YD), CYAN, 1)
            cv.rectangle(view, (max(0, min(VIDEO_X - 1, angle + VIDEO_X // 2 - 2)), WIN_YU),
                         (max(0, min(VIDEO_X - 1, angle + VIDEO_X // 2 + 2)), WIN_YD),
                         cursorColor, 2)

            # Determine the image label angle group, as in the original gi.py.
            groups = [
                (-99, -47, "A"), (-46, -22, "B"), (-21, -6, "C"),
                (-5, 5, "D"), (6, 21, "E"), (22, 46, "F"), (47, 99, "G")
            ]
            label_group = "D"
            for low, high, name in groups:
                if low <= g <= high:
                    label_group = name
                    break

            mouse_y_clamped = max(WIN_YU, min(WIN_YD, mouseY))
            n = 10 - int(10 * (mouse_y_clamped - WIN_YU) /
                         max(1, (WIN_YD - WIN_GAP_Y - WIN_YU)))
            n = max(0, min(10, n))
            if VARIABLE_MODE:
                record_time = recTimeList[n]
                forward_speed = REC_LOW_FORWARD_SPEED + n * 5
            else:
                record_time = defRecTime
                forward_speed = defForSpeed

            sign = "+" if g >= 0 else "-"
            save_file = f"{fileNF}{fileId:04d}_{forward_speed:03d}_{sign}{abs(g):02d}.png"

            cv.putText(view, f"{save_file}  Drive L:{mL} R:{mR}",
                       (8, VIDEO_Y - 12), cv.FONT_HERSHEY_SIMPLEX, 0.48, WHITE, 1, cv.LINE_AA)
            cv.putText(view, f"Mode: {'MENU' if menu_view else 'CLEAR'} | "
                             f"{'VARIABLE' if VARIABLE_MODE else 'FIXED'}",
                       (8, VIDEO_Y - 34), cv.FONT_HERSHEY_SIMPLEX, 0.45, YELLOW, 1, cv.LINE_AA)

            if DIRECTORY_FLAG and (recordOn or oneShot):
                now_ticks = cv.getTickCount()
                elapsed = now_ticks - time_memory
                if elapsed > record_time or oneShot:
                    output_path = os.path.join(filePath, save_file)
                    if cv.imwrite(output_path, road_img):
                        fileId += 1
                        print("[SAVE]", output_path)
                    else:
                        print("[WARN] Image save failed:", output_path)
                    time_memory = now_ticks
                    oneShot = False

            cv.imshow("Out", view)
            key = cv.waitKeyEx(1)

            if key in (27, ord("x"), ord("X"), 9):
                break

            # Arrow keys (OpenCV key codes can differ by platform).
            if key in (2490368, 82):  # Up
                recordOn = False
                mouseCtrlState = False
                if pre_key == ord(" "):
                    pre_key = key
                    drive.run(70, 70)
                    cursorColor = GREEN
                else:
                    pre_key = ord(" ")
                    drive.stop()
                    cursorColor = YELLOW
            elif key in (2621440, 84):  # Down
                recordOn = False
                mouseCtrlState = False
                if pre_key == ord(" "):
                    pre_key = key
                    drive.run(-60, -60)
                    cursorColor = GREEN
                else:
                    pre_key = ord(" ")
                    drive.stop()
                    cursorColor = YELLOW
            elif key in (2424832, 81):  # Left
                recordOn = False
                mouseCtrlState = False
                if pre_key == ord(" "):
                    pre_key = key
                    drive.run(-60, 60)
                    cursorColor = GREEN
                else:
                    pre_key = ord(" ")
                    drive.stop()
                    cursorColor = YELLOW
            elif key in (2555904, 83):  # Right
                recordOn = False
                mouseCtrlState = False
                if pre_key == ord(" "):
                    pre_key = key
                    drive.run(60, -60)
                    cursorColor = GREEN
                else:
                    pre_key = ord(" ")
                    drive.stop()
                    cursorColor = YELLOW
            elif key == ord(" "):
                recordOn = False
                mouseCtrlState = False
                pre_key = ord(" ")
                cursorColor = YELLOW
                drive.stop()
            elif key in (ord("c"), ord("C")):
                menu_view = not menu_view
            elif key in (ord("v"), ord("V")):
                VARIABLE_MODE = not VARIABLE_MODE
            elif key in (ord("!"), ord("@"), ord("#"), ord("$"),
                         ord("%"), ord("^"), ord(")")):
                # ROI adjustment keys retained from original gi.py.
                if not fileId:
                    if key == ord("!") and WIN_XL <= WIN_GAP_X:
                        WIN_XL += 2; WIN_XR -= 2
                    elif key == ord("@") and WIN_XL >= 2:
                        WIN_XL -= 2; WIN_XR += 2
                    elif key == ord("#") and (WIN_YD - WIN_YU) > WIN_GAP_Y + 4:
                        WIN_YU += 2
                    elif key == ord("$") and WIN_YU > 152:
                        WIN_YU -= 2
                    elif key == ord("%") and (WIN_YD - WIN_YU) > WIN_GAP_Y + 4:
                        WIN_YD -= 2
                    elif key == ord("^") and WIN_YD <= 477:
                        WIN_YD += 2
                    elif key == ord(")"):
                        WIN_XL, WIN_XR = WIN_XL_ORG, WIN_XR_ORG
                        WIN_YU, WIN_YD = WIN_YU_ORG, WIN_YD_ORG

    finally:
        recordOn = False
        mouseCtrlState = False
        try:
            drive.close()
        finally:
            sensors.close()
            cam.release()
            cv.destroyAllWindows()
            GPIO.cleanup()


if __name__ == "__main__":
    print("OpenCV Version:", cv.__version__)

    if len(sys.argv) >= 3:
        fileId = int(sys.argv[2])

    if len(sys.argv) >= 2:
        imageDir = sys.argv[1]
        if not os.path.exists(imageDir):
            answer = input(f"이미지 저장 디렉터리 '{imageDir}'가 없습니다. 생성할까요? (Y/N): ")
            if answer.lower() == "y":
                os.makedirs(imageDir, exist_ok=True)
                DIRECTORY_FLAG = True
            else:
                DIRECTORY_FLAG = False
        else:
            DIRECTORY_FLAG = True

        if DIRECTORY_FLAG:
            filePath = imageDir
            pickle_path = os.path.join(
                imageDir, f"_{os.path.basename(imageDir)}_track.pickle"
            )
            if os.path.exists(pickle_path):
                try:
                    with open(pickle_path, "rb") as fr:
                        saved = pickle.load(fr)
                    if len(saved) >= 7:
                        WIN_XL, WIN_XR, WIN_YU, WIN_YD = saved[0:4]
                        # Preserve the old lamp-state fields for file compatibility.
                        AUTO_LIGHT, MANUAL_LIGHT = saved[4:6]
                        if len(sys.argv) < 3:
                            fileId = saved[6]
                    print("[INFO] Loaded collection settings:", pickle_path)
                except Exception as exc:
                    print("[WARN] Could not load pickle settings:", exc)

    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        if DIRECTORY_FLAG:
            try:
                saved = [
                    WIN_XL, WIN_XR, WIN_YU, WIN_YD,
                    AUTO_LIGHT, MANUAL_LIGHT, fileId
                ]
                with open(pickle_path, "wb") as fw:
                    pickle.dump(saved, fw)
                print("[INFO] Saved settings:", pickle_path)
            except Exception as exc:
                print("[WARN] Could not save settings:", exc)
