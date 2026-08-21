import time
import math
import smbus2
import pigpio
import RPi.GPIO as GPIO
from hx711 import HX711

#THRESHOLDS

LOAD_LIMIT_KG = 1.7
TILT_THRESHOLD_DEG= 0.8
TILT_OFFSET = 0.0
VIBRATION_THRESHOLD = 0.35

#GPIO PINS

SERVO1_GPIO = 18
SERVO2_GPIO = 12

LED1_GPIO = 23
BUZZER1_GPIO = 16

HX1_DOUT, HX1_SCK = 5, 6
HX2_DOUT, HX2_SCK = 13, 19

HX_CAL_FACTOR = 22370

BMI160_ADDR = 0x68
I2C_BUS = 1

SERVO_OPEN = 2000
SERVO_CLOSE = 1000

#INITIALIZATION

GPIO.setmode(GPIO.BCM)

#outputs
for pin in [LED1_GPIO, BUZZER1_GPIO, LED2_GPIO, BUZZER2_GPIO]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

#pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Run: sudo pigpiod")

pi.set_servo_pulsewidth(SERVO1_GPIO, SERVO_CLOSE)
pi.set_servo_pulsewidth(SERVO2_GPIO, SERVO_CLOSE)

hx1 = HX711(HX1_DOUT, HX1_SCK)
hx2 = HX711(HX2_DOUT, HX2_SCK)

for hx in (hx1, hx2):
    hx.set_reference_unit(HX_CAL_FACTOR)
    hx.reset()
    print("Taring load cells... DO NOT TOUCH")
    time.sleep(2)

    hx1.tare()
    hx2.tare()
  
    print("Tare complete")

bus = smbus2.SMBus(I2C_BUS)

bus.write_byte_data(BMI160_ADDR, 0x7E, 0x11) #accel normal
time.sleep(0.05)
bus.write_data_data(BMI160_AADR, 0x7E, 0x15) #gyro normal
time.sleep(0.05)

print("Bridge Monitoring System READY")

#FUNCTIONS

def read_accel():
  data = bus.read_i2c_block_data(BMI160_ADDR, 0x12, 6)

  x = (data[1] << 8) | data[0]
  y = (data[3] << 8) | data[2]
  z = (data[5] << 8) | data[4]

  if x > 32767: x-= 65536
  if y > 32767: y-= 65536
  if z > 32767: z-= 65536

  return x / 16384.0, y / 16384.0, z / 16384.0 #convert to g

def magnitude(x, y, z):
  return math.sqrt(x*x +y*y + z*z)

def calculate_tilt(x, y, z):
  mag = magnitude(x, y, z)
  if mag == 0:
    return 0
  return math.degrees(math.acos(z / mag))

def read_weights(hx):
  readings = []

  for _ in range(10):
    readings.append(hx.get_weight(1))

  avg = sum(readings) / lens(readings)

  hx.power_down()
  hx.power_up()

  # HARD DEADZONE FILTER
  if abs(avg) < 0.2:
    avg = 0

  return round(avg, 2)

#Alarm Control
def activate_side1():
  GPIO.output(LED1_GPIO, GPIO.HIGH)
  GPIO.output(BUZZER1_GPIO, GPIO.HIGH

def deactivate_side1():
  GPIO.output(LED1_GPIO, GPIO.LOW)
  GPIO.output(BUZZER1_GPIO, GPIO.LOW)

def activate_side2():
  GPIO.output(LED2.GPIO, GPIO.HIGH)
  GPIO.output(BUZZER2_GPIO, GPIO.HIGH)

def deactivate_side2():
  GPIO.output(LED2_GPIO, GPIO.LOW)
  GPIO.output(BUZZER2_GPIO, GPIO.LOW)

def activate_all():
  activate_side1()
  activate_side2()

def deactivate_all():
  deactivate_side1()
  deactivate_side2()

#TILT CALIBRATION
print('calibrating tilt... Keep System Flat')
time.sleep(2)

ax0, ay0, az0 = read_accel()
TILT_OFFSET = calculate_tilt(ax0,ay0, az0)

print(f"Tilt Offset Calibrated: {TILT_OFFSET:.2f} degrees")

#FILTER VARIABLES

motion_counter = 0
stable_counter = 0

MOTION_CONFIRM = 1
STABLE_CONFIRM = 2

#MAIN LOOP

try:
  while True:
    #read sensors
    ax, ay, az = read_accel()
    tilt = abs(calculate_tilt(ax, ay, az) - TILT_OFFSET)

    dynamic acc = abs(magnitude(ax, ay, az) -1.0)

    w1 = read_weight(hx1)
    w2 = read weight(hx2)

    if abs(w1) < 0.05:
      w1 = 0

    if abs(w2) < 0.05:
      w2 = 0

    print(f"Tilt: {tilt:.2f} degrees | Vibration {dynamic_acc:.3f} g")
    print(f"Load 1: {w1} kg | Load 2: {w2} kg")
    print("-"*40)

    #FILTERING

    if tilt> TILT_THRESHOLD_DEG or dynamic_acc > VIBRATION_THRESHOLD:
      motion_counter += 1
      stable_counter = 0
    else:
      stable_counter += 1
      motion_counter = 0

    severe_motion = motion_counter >= MOTION_CONFIRM
    stable = stable_counter >= STABLE_CONFIRM

    #SERVO LOGIC
    servo1_active = w1 > LOAD_LIMIT_KG or severe_motion
    servo2_active = w2 > LOAD_LIMIT_KG or severe_motion

    #Servo Control
    pi.set_servo_pulsewidth(
      SERVO1_GPIO,
      SERVO_OPEN if servo1_active else SERVO_CLOSE
    )
    pi.set_servo_pulsewidth(
      SERVO2_GPIO,
      SERVO_OPEN if servo2_active else SERVO_CLOSE
    )
    #Alarm Control
    if severe_motion:
      activate_all()

    elif stable:
      deactivate all()

      if w1 > LOAD_LIMIT_KG:
        activate_side1()

      if w2 > LOAD_LIMIT_KG:
        activate_side2()

  time.sleep(0.5)

except KeyboardInterrupt:
  print("System stopped")
