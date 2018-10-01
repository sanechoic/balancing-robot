import pyb
from pyb import Pin, Timer, ExtInt
from pyb import LED
from pyb import Pin, Timer, ADC, DAC, LED
import time
from oled_938 import OLED_938
from mpu6050 import MPU6050
from motor import DRIVE

import micropython

micropython.alloc_emergency_exception_buf(100)

pot = ADC(Pin('X11'))

oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64, external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()

imu = MPU6050(1, False)

motor = DRIVE()

# PID tuning
'''
trigger = pyb.Switch()
scale = 2.0
while not trigger():	# Wait to tune Kp
    time.sleep(0.001)
    K_p = pot.read() * scale / 4095		# Use potentiometer to set Kp
    oled.draw_text(0, 40, 'Kp = {:5.2f}'.format(K_p))	# Display live value on oled
    oled.display
while trigger(): pass
while not trigger():	# Wait to tune Ki
    time.sleep(0.001)
    K_i = pot.read() * scale / 4095		# Use pot to set Ki
    oled.draw_text(0, 50, 'Ki: {:5.2f}'.format(K_i))	# Display live value on oled)
    oled.display
while trigger(): pass
while not trigger():	# Wait to tune Kd
    time.sleep(0.001)
    K_d = pot.read() * scale / 4095		# Use pot to set Ki
    oled.draw_text(0, 60, 'Kd: {:5.2f}'.format(K_d))	# Display live value on oled
    oled.display
while trigger(): pass
'''

# Pitch angle calculation using complementary filter
def pitch_estimation(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt*0.001) + (1-alpha)*theta
#	print(pitch_dot)
    return (pitch, pitch_dot)



K_p = 5.41
K_i = 0.2
K_d = 0.3

oled.draw_text(0, 20, 'Kp = {:5.2f}'.format(K_p))	# Display live value on oled
oled.draw_text(0, 30, 'Ki: {:5.2f}'.format(K_i))	# Display live value on oled)
oled.draw_text(0, 40, 'Kd: {:5.2f}'.format(K_d))	# Display live value on oled

oled.display()

alpha = 0.7
pitch = 0
r = 0	# Target (set-point)
e_int = 0
e_diff = 0
v = 0
tic = pyb.micros()
pyb.delay(10)
while True:
    dt = pyb.micros()-tic
    if dt > 5000:
        pitch, pitch_dot = pitch_estimation(pitch, dt, alpha)
        # PID control
        e = pitch - r
        e_int += e
        v = K_p*e + K_i*dt*e_int + K_d*(e_int-e_diff)/dt
        if v > 0:
            motor.right_forward(v)
            motor.left_forward(v)
        if v < 0:
            motor.right_back(v)
            motor.left_back(v)
        e_diff = e_int

        tic = pyb.micros()

