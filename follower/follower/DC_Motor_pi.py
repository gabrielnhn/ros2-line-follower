import RPi.GPIO as GPIO

PWM_FREQ = 500 # hZ
PIN_MODE = GPIO.BOARD # physical pinout

class DC_Motor:

    def __init__(self, clockwise_pin, counterclockwise_pin, pwm_pin):
        self.clockwise_pin = clockwise_pin
        self.counterclockwise_pin = counterclockwise_pin
        self.pwm_pin = pwm_pin

        GPIO.setmode(PIN_MODE)

        GPIO.setup(self.clockwise_pin, GPIO.OUT)
        GPIO.setup(self.counterclockwise_pin, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        self.pwm_control = GPIO.PWM(self.pwm_pin, PWM_FREQ)
        self.pwm_control.start(0)

    def speed(self, duty_cycle):
        self.pwm_control.ChangeDutyCycle(duty_cycle)

    def forward(self, duty_cycle):
        GPIO.output(self.clockwise_pin, GPIO.HIGH)
        GPIO.output(self.counterclockwise_pin, GPIO.LOW)
        self.speed(duty_cycle)

    def backwards(self, duty_cycle):
        GPIO.output(self.clockwise_pin, GPIO.LOW)
        GPIO.output(self.counterclockwise_pin, GPIO.HIGH)
        self.speed(duty_cycle)

    def stop(self):
        GPIO.output(self.clockwise_pin, GPIO.LOW)
        GPIO.output(self.counterclockwise_pin, GPIO.LOW)
        self.speed(0)

    def short_brake(self):
        GPIO.output(self.clockwise_pin, GPIO.HIGH)
        GPIO.output(self.counterclockwise_pin, GPIO.HIGH)
        self.speed(0)


    def run(self, value):
        """-100 to 100"""

        try:
            if value == 0:
                self.stop()

            elif value > 0:
                if value > 100:
                    value = 100
                
                self.forward(value)
            
            else: # less than 0
                if value < -100:
                    value = -100
                
                self.backwards(-value)
        except Exception as e:
            print(value)
            raise e



    def __del__(self):
        self.pwm_control.stop()
        