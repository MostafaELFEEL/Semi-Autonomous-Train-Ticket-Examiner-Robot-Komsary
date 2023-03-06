import rospy
import math
import time
import RPi.GPIO as GPIO
from smbus import SMBus
from signal import signal, SIGTERM, SIGHUP, pause
from rpi_lcd import LCD
import cv2
from std_msgs.msg import Int32
from mfrc522 import SimpleMFRC522

i2c_arduino = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
sensor_left = 5
sensor_middle =6
sensor_right =13
GPIO_TRIGGER = 12
GPIO_ECHO = 16
servoPIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor_left,GPIO.IN)
GPIO.setup(sensor_right,GPIO.IN)
GPIO.setup(sensor_middle,GPIO.IN)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 18 for PWM with 50Hz
p.start(2) # Initialization
lcd = LCD() 
face=0
dummy=6

def visioncallback(data):
    print("data sent=")
    print(data)
    global face
    face=data.data
    print("value of face=")
    print(face)


def safe_exit(signum, frame):
    exit(1)
 
 
 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

def scanning():
    lcd.text("Scanning for passenger", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger.", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger..", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger...", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger..", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger...", 1)
    time.sleep(0.5)
    lcd.text("Scanning for passenger....", 1)
    time.sleep(0.5)
    lcd.text("Scanning complete         ", 1)
    time.sleep(1.5)
    lcd.text("                                     ", 1)
    lcd.text("                                     ", 2)



if __name__ == "__main__":
    global y
    rospy.init_node("code")
    rospy.Subscriber("/visionoutput", Int32, visioncallback)
    vision_pub = rospy.Publisher("/startvision", Int32, queue_size=10)
   # rate = rospy.Rate(10)
    try:
        signal(SIGTERM, safe_exit)
        signal(SIGHUP, safe_exit)
        bus.write_byte(i2c_arduino, 2)
        while not rospy.is_shutdown():
        
            
            dist = distance()
            #print ("Measured Distance = %.1f cm" % dist)
            p.ChangeDutyCycle(6.5)
            
            
          #or ((GPIO.input(sensor_left)==0)and(GPIO.input(sensor_middle)==0)and(GPIO.input(sensor_right)==0)and dummy!=2)  
            if (((GPIO.input(sensor_left)==0)and(GPIO.input(sensor_middle)==1)and(GPIO.input(sensor_right)==0)and dummy!=2))and dist>20:
                print("2")
                bus.write_byte(i2c_arduino, 2)
                dummy=2
                 
            elif (((GPIO.input(sensor_left)==0)and(GPIO.input(sensor_middle)==1)and(GPIO.input(sensor_right)==1)and dummy!=3) or((GPIO.input(sensor_left)==0)and(GPIO.input(sensor_middle)==0)and(GPIO.input(sensor_right)==1)and dummy!=3))and dist>20:
                print("3")
                bus.write_byte(i2c_arduino, 3)
                dummy=3
                time.sleep(0.3)
            elif (((GPIO.input(sensor_left)==1)and(GPIO.input(sensor_middle)==1)and(GPIO.input(sensor_right)==0)and dummy!=4) or((GPIO.input(sensor_left)==1)and(GPIO.input(sensor_middle)==0)and(GPIO.input(sensor_right)==0)and dummy!=4))and dist>20:
                print("4")
                bus.write_byte(i2c_arduino, 4)
                dummy=4
                time.sleep(0.3)
            elif ((GPIO.input(sensor_left)==1)and(GPIO.input(sensor_middle)==1)and(GPIO.input(sensor_right)==1)and dummy!=1)and dist>20:
                print("1")
                bus.write_byte(i2c_arduino, 1)
                dummy=1
                time.sleep(1)
                p.ChangeDutyCycle(4)
                vision_pub.publish(1)
                scanning()
                vision_pub.publish(0)
                time.sleep(1)
                time.sleep(3)
                print("value of detect=")
                print(face)
                if (face==1):
                    reader = SimpleMFRC522()
                    
                    id, text = reader.read()
                    if int(text)<=6:
                        lcd.text("Thank you for your cooperation :)", 1)
                        time.sleep(3)
                    
                    #lcd.text("start arfid code1",1)
                #checking for passenger if true then rfid send and if not then continue
                lcd.text("                                     ", 1)
                p.ChangeDutyCycle(6.5)
                time.sleep(1)
                p.ChangeDutyCycle(9)
                vision_pub.publish(1)
                #send massege to vision
                scanning()
                vision_pub.publish(0)
                time.sleep(1)
                print("value of detect=")
                print(face)
                if (face==1):
                    #lcd.text("start arfid code2",1)
                    reader = SimpleMFRC522()
                    
                    id, text = reader.read()
                    if int(text)<=6:
                        lcd.text("Thank you for your cooperation :)", 1)
                        time.sleep(3)
                #checking for passenger if true then rfid send and if not then continue
                
                lcd.text("                                     ", 1)
                p.ChangeDutyCycle(6.5)
                time.sleep(2)
                bus.write_byte(i2c_arduino, 2)
                time.sleep(2)
                
            elif(dist<20 and dummy!=5):
                bus.write_byte(i2c_arduino, 1)
                print("move away please")
                lcd.text("Please clear the way", 1)
                lcd.text("   Thank you :)   ", 2)
                time.sleep(3)
                lcd.text("                    ", 1)
                lcd.text("                    ", 2)
                lcd.text("                    ", 3)
                lcd.text("                    ", 4)
                dummy=5
             #   p.ChangeDutyCycle(3)
             #   vision_pub.publish(1)
              #  time.sleep(3)
              #  vision_pub.publish(0)
               # if(face==1):
               #     print("please scan your card")
                    #rfid code
                #    print("thank you for your cooperation")
                #    face=0
               # time.sleep(1)
                #p.ChangeDutyCycle(9)
                #vision_pub.publish(1)
              #  time.sleep(3)
                #vision_pub.publish(0)
               # if(face==1):
                 #   print("please scan your card")
                    #rfid code
                 #   print("thank you for your cooperation")
                  #  face=0
                    
                    
                    
              
                
         #   rate.sleep()
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()
        lcd.clear()
