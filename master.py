import serial
import os
import time
import sys
import Adafruit_DHT as dht
import paho.mqtt.client as paho
import json
import RPi.GPIO as GPIO
import socket
from apscheduler.schedulers.blocking import BlockingScheduler
import RPi_I2C_driver
import json


#PROJECT STRINGS
projectname = "Atlantis"
ecname = "EC"
phname = "pH"
tempname = "C"
humidityname = "H"
sensorstart = projectname+"  "+"SEN ON"
controlstart  = projectname+"  "+"CTRL ON"

#INITILIZATION
mylcd = RPi_I2C_driver.lcd() #raspiI2C driver
sched = BlockingScheduler({'apscheduler.job_default.max_instances':'2'}) #increase simulataneous instance to more than 1
GPIO.setmode(GPIO.BOARD) # GPIO.BOARD mode
GPIO.setwarnings(False) # disable warnings - Your not a noob to want warnings
GPIO.setup(13,GPIO.OUT) #     BUZZER
GPIO.setup(15,GPIO.OUT) #     DOUZER -1 (DZ-1)
GPIO.setup(16,GPIO.OUT) #     DOUZER -2 (DZ-2)
GPIO.setup(18,GPIO.OUT) #     EXHAUST_FAN_1 (EX-1)
GPIO.setup(22,GPIO.OUT) #     EXHAUST_FAN_2 (EX-2)
GPIO.setup(29,GPIO.OUT) #     TURN_FAN (OFAN)
GPIO.setup(31,GPIO.OUT) #     AC
GPIO.setup(32,GPIO.OUT) #     
GPIO.setup(33,GPIO.OUT) #     
GPIO.setup(36,GPIO.OUT) #     RACK_MOTOR
GPIO.setup(37,GPIO.OUT) #     RACK_LIGHT

#jsondata structure/model
sensor_data = {'temp': 0, 'humidity': 0 ,'ec':0,'ph':0}

#Splash Screen :P
mylcd.lcd_display_string(projectname,1)
mylcd.lcd_display_string('Water it takes',2)

#MQTT credentials
client = paho.Client("HYP-MASTER-RASPI")
broker = '165.22.209.7' #server ip
port =1883 #default mqtt port
client.connect(broker, 1883, 60)

client.is_connected = True

#Serial Communication for ANALOG INPUT
#ser = serial.Serial('/dev/ttyACM0',9600) #for Arduino Uno ttyACM0
ser = serial.Serial('/dev/ttyUSB0',9600) #for Ardunio Nano ttyUSBo


#Populate & Display sensor data
@sched.scheduled_job('interval', seconds=5)
def sensor_display_job():
        humidity,temperature = dht.read_retry(dht.DHT11, 17) #17 = GPIO.BOARD-11 As dhtLibrary uses BCM.BOARD mode
        humidity = round(humidity, 2)  
        temperature = round(temperature, 2)
        sensor_data['temp'] = temperature
        sensor_data['humidity'] = humidity
        #serial read ec and ph from nano - JSON structure {"ec":"value","ph":"value"} 
        jsondata = ser.readline().decode("utf-8")
        data = json.loads(jsondata)        
        sensor_data['ec'] = data.get('ec')
        sensor_data['ph'] = data.get('ph')
        print(sensor_data) #log sensor_data dict
        mylcd.lcd_display_string('T:'+str(sensor_data['temp'])+'C  H:'+str(sensor_data['humidity']),1) #display row1
        mylcd.lcd_display_string('EC:'+str(sensor_data['ec'])+' pH:'+str(sensor_data['ph']),2) #display row2


# Schedule for MQTT -  publish sensor_data = {'temp': 0, 'humidity': 0 ,'ec':0,'ph':0}
@sched.scheduled_job('interval', seconds=10)
def publish_job(): 
        print('PUBLISHED')
        client.loop_start()
        client.publish('hyp/sv', json.dumps(sensor_data), 1) #publishing to topic hyp/sv 
        client.loop_stop()
       
       
# Schedule for Motor - GPIO 36
@sched.scheduled_job('interval', seconds=3000)
def motor_job():
    GPIO.output(13,True)
    time.sleep(1)
    GPIO.output(13,False)
    if GPIO.input(36) == 0:
        print ("Motor ON")
        GPIO.output(36,True)
        time.sleep(30)
        print ("Motor OFF")
        GPIO.output(36,False)

# TURN ON LIGHTS - GPIO 37
@sched.scheduled_job('cron', day_of_week='mon-sun', hour=9)
def lighton_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)        
        GPIO.output(37,True)
        print ("LIGHTS ON")

# TURN OFF LIGHTS - GPIO 37
@sched.scheduled_job('cron', day_of_week='mon-sun', hour=17)
def liightoff_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)        
        GPIO.output(37,True)
        print ("LIGHTS OFF")

sched.start()

client.disconnect()
