from gpiozero import Servo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory() # sudo pigpiod failed
servo = Servo(12, pin_factory=factory) # GPIO-12

print("Start in the middle")
servo.mid()
sleep(5)
print("Go to min")
servo.min()
sleep(5)
print("Go to max")
servo.max()
sleep(5)
print("And back to middle")
servo.mid()
sleep(5)
servo.value = None;
