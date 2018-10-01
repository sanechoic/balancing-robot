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
oled.draw_text(0,0, 'Group 8')
oled.draw_text(0, 10, 'Milestone 3: Balencing')
oled.draw_text(0, 20, 'Press USR button')
oled.display()

print('Performing Milestone 3')
print('Waiting for button press')
trigger = pyb.Switch()		# Create trigger switch object
while not trigger():		# Wait for trigger press
	time.sleep(0.001)
while trigger():
	pass			# Wait for release
print('Button pressed - Running')

# PID tuning
pot = pyb.ADC(Pin('X11'))
scale1 = 4
scale2 = 0.6
while not trigger():	# Wait to tune Kp
	time.sleep(0.001)
	K_p = pot.read() * scale1 / 4095		# Use potentiometer to set Kp
	oled.draw_text(0, 30, 'Kp = {:5.5f}'.format(K_p))	# Display live value on oled
	oled.display()
while trigger(): pass
while not trigger():	# Wait to tune Ki
	time.sleep(0.001)
	K_i = pot.read() * scale2 / 4095		# Use pot to set Ki
	oled.draw_text(0, 40, 'Ki = {:5.5f}'.format(K_i))	# Display live value on oled
	oled.display()
while trigger(): pass
while not trigger():	# Wait to tune Kd
	time.sleep(0.001)
	K_d = pot.read() * scale2 / 4095		# Use pot to set Ki
	oled.draw_text(0, 50, 'Kd = {:5.5f}'.format(K_d))	# Display live value on oled
	oled.display()
while trigger(): pass

imu = MPU6050(1, False)

motor = DRIVE()

# Pitch angle calculation using complementary filter
def pitch_estimation(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt/1000000) + (1-alpha)*theta
#	print(pitch_dot)
    print("filtered = " + str(pitch))
    print("imu = " + str(theta))
    pyb.delay(2000)
    return (pitch, pitch_dot)



#K_p = 5.41
#K_i = 0.2
#K_d = 0.3

#oled.draw_text(0, 20, 'Kp = {:5.2f}'.format(K_p))	# Display live value on oled
#oled.draw_text(0, 30, 'Ki: {:5.2f}'.format(K_i))	# Display live value on oled
#oled.draw_text(0, 40, 'Kd: {:5.2f}'.format(K_d))	# Display live value on oled

#oled.display()

alpha = 0.95
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
#        if v > 0:
#            motor.right_back(v)
#            motor.left_back(v)
#        if v < 0:
#            motor.right_forward(v)
#            motor.left_forward(v)
        e_diff = e_int

        tic = pyb.micros()

