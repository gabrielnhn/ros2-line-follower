import RPi.GPIO as GPIO
import time
from DC_Motor_pi import DC_Motor

# hardware pwm: 12, 32, 33, 35

# pins setup
clockwise_pin_1 = 11
counterclockwise_pin_1 = 13
pwm_pin_1 = 12

clockwise_pin_2 = 29
counterclockwise_pin_2 = 31
pwm_pin_2 = 32

motor_1 = DC_Motor(clockwise_pin_1, counterclockwise_pin_1, pwm_pin_1)

motor_2 = DC_Motor(clockwise_pin_2, counterclockwise_pin_2, pwm_pin_2)

try:
    while True:
        motor_1.forward(100)
        motor_2.forward(100)
        time.sleep(3)

        motor_1.stop()
        motor_2.stop()
        time.sleep(1)

        motor_1.backwards(100)
        motor_2.backwards(100)
        time.sleep(3)

        motor_1.stop()
        motor_2.stop()
        time.sleep(1)
except:
    del motor_1
    del motor_2
    GPIO.cleanup()