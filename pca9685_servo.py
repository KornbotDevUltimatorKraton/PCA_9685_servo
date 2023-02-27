# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import math 
import time
from board import SCL, SDA
import busio
import smbus
# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo,LED
import subprocess
import os
try:
 os.system("sudo systemctl start pigpiod")
except: 
   print("Pigpiod activated!")
   os.system("python3 pca9685_servo.py")
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
AngleDegX  = 0 
AngleDegY = 0 
AngleDegZ = 0    
ledr = LED(17)
ledl = LED(27)
ledr.on()
ledl.on()
factory = PiGPIOFactory()
s2 = AngularServo(12,pin_factory=factory,min_angle=0, max_angle=180)
i2c = busio.I2C(SCL, SDA)
#temp_Raw = subprocess.check_output("vcgencmd measure_temp",shell=True)
#temp_dec = temp_Raw.decode()
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)
#servo = servo.Servo(pca.channels[0])
servo15 = servo.Servo(pca.channels[15])
servo14 = servo.Servo(pca.channels[14])
servo13 = servo.Servo(pca.channels[13])
servo12 = servo.Servo(pca.channels[12])
servo11 = servo.Servo(pca.channels[11])
servo10 = servo.Servo(pca.channels[10])
servo9 = servo.Servo(pca.channels[9])
servo8 = servo.Servo(pca.channels[8])
servo7 = servo.Servo(pca.channels[7])
servo6 = servo.Servo(pca.channels[6])
servo5 = servo.Servo(pca.channels[5])
servo4 = servo.Servo(pca.channels[4])
servo3 = servo.Servo(pca.channels[3])
servo2 = servo.Servo(pca.channels[2])
servo1 = servo.Servo(pca.channels[1])
servoE = servo.Servo(pca.channels[0])
#Sitting mode v    alue initial angle for swing the leg and start walking 
servo7.angle = 180   #Back middle left digitegrate leg sitting  180 -160  
servo13.angle = 180  #Back middle left terminal digitegrate sitting
servo6.angle = 120   #Front right leg elbow 180 -120        
servo5.angle = 30    #Front right abduct shoulder 0 -90
servo4.angle = 150   #Front left abduct shoulder  150- 90
servo3.angle = 0     #Front left leg elbow 0-60
servo2.angle = 180   #Front left shoulder leg 180-0 
servo1.angle = 0     #Front right shoulder leg  0-120
servo8.angle = 0     #Back right middle digitegrate leg sitting 0 - 20  
servo9.angle = 0     #Back right terminal digitegrate leg  0-90
servo10.angle =150   #Back right leg abduct hip 0-90
servo11.angle =140    #Back right hip 180-0 
servo12.angle = 40   #Back left hip  0-180
servo14.angle = 30    #Back left leg abduct 0-90
servo15.angle = 90  #rotate neck
servoE.angle = 90    #middle body rotation
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 # Reading the smbus data from the i2c mpu6050 sensor 
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
 
bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect
 
# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)
 
print("Gyroscope")
print("--------")
 
gyroskop_xout = read_word_2c(0x43)
gyroskop_yout = read_word_2c(0x45)
gyroskop_zout = read_word_2c(0x47)

#Neck part pantilt function for fusion sensor integration part 
def Neckpart_Pantilt(x,y,z):
     s2.angle = 90  
       
#Front right leg free motion control walking and free activity behavier control 
def Walkingfree_behavier(time,initialf,terminalf,initialr,terminalr):
   
      for i in range(initialf,terminalf):
         servo1.angle = i
         servo6.angle = 180-i
         time.sleep(time)
      for i in reversed(range(initialr,terminalr)):
         servo1.angle = i
         servo6.angle = 180-i
         time.sleep(time)
times = 0.002
while True: 
 #Neck motion control 
 s2.angle = 90
 #Front right leg motion control standing
 for te in range(0,10):
   temp_Raw = subprocess.check_output("vcgencmd measure_temp",shell=True)
   temp_dec = temp_Raw.decode()
   print(temp_dec)
   print("Catbot Gyroscope")
   print("---------------------")
   beschleunigung_xout = read_word_2c(0x3b)
   beschleunigung_yout = read_word_2c(0x3d)
   beschleunigung_zout = read_word_2c(0x3f)
   beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
   beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
   beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
   AngleDegX = math.degrees(beschleunigung_xout_skaliert)
   AngleDegY = math.degrees(beschleunigung_yout_skaliert)
   AngleDegZ = math.degrees(beschleunigung_zout_skaliert)
   print("AngleDegX",(AngleDegX))
   print("AngleDegY",(AngleDegY))
   print("AngleDegZ",(AngleDegZ))
   """
   servo10.angle =150 + AngleDegX
   for i in range(0,120):
         servo1.angle = i
         servo6.angle = 160-i
         print("PD_output_forward",(abs(i-120)/120)*50+10+5*AngleDegX,AngleDegX)
         servo5.angle = (abs(i-120)/120)*50+10+2*AngleDegX
         time.sleep(te/1000)
         
   for i in reversed(range(0,120)): 
         servo1.angle = i
         servo6.angle = 160-i
         print("PD_output_reverse",(abs(i-120)/120)*50+10+5*AngleDegX,AngleDegX)
         servo5.angle = (abs(i-120)/120)*50+10+2*AngleDegX
         time.sleep(te/1000)
   time.sleep(0.08) # Time laps for adjust speedeach round 
   """
 #Front left leg motion control standing 
 
"""
 for te in reversed(range(0,10)):
   for i in range(0,120):
         servo1.angle = i
         servo6.angle = 160-i
         print("PD_output_forward",abs(i-120))
         servo5.angle = (abs(i-120)/120)*50+10
         time.sleep(te/1000)

   for i in reversed(range(0,120)):
         servo1.angle = i
         servo6.angle = 160-i
         print("PD_output_reverse",abs(i-120))
         servo5.angle = (abs(i-120)/120)*50+10
         #if i >= 50:
         #     servo5.angle = 10
         #if i <=50:
         #     servo5.angle = 50
         time.sleep(te/1000)
   time.sleep(0.05)            
"""
# We sleep in the loops to give the servo time to move into position.
#for i in range(180):
#    servo7.angle = i
#    time.sleep(0.03)
#for i in range(180):
#    servo7.angle = 180 - i
#    time.sleep(0.03)

# You can also specify the movement fractionally.
#fraction = 0.0
#while fraction < 1.0:
#    servo7.fraction = fraction
#    fraction += 0.01
#    time.sleep(0.03)

pca.deinit()
