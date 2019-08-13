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
mylcd = RPi_I2C_driver.lcd()
sched = BlockingScheduler({'apscheduler.job_default.max_instances':'2'})
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(13,GPIO.OUT) #     37
GPIO.setup(37,GPIO.OUT) #     37
GPIO.setup(33,GPIO.OUT)#     33
#GPIO.setup(31,GPIO.OUT)#     31
#GPIO.setup(29,GPIO.OUT)#     29
GPIO.setup(7,GPIO.IN)
sensor_data = {'temp': 0, 'humidity': 0 ,'ec':0,'ph':0}

mylcd.lcd_display_string(projectname,1)
mylcd.lcd_display_string('Water it takes',2)




#MQTT
client = paho.Client()
broker = '165.22.209.7'
port =1883
client.connect(broker, 1883, 60)
client.is_connected = True


#Serial Comm
#ser = serial.Serial('/dev/ttyACM0',9600) #for uno
ser = serial.Serial('/dev/ttyUSB0',9600) #for nano

#GPIO.output(37,False)
#GPIO.output(33,False)
#GPIO.output(31,False)
#GPIO.output(29,False)

#Populate sensor data & Display
@sched.scheduled_job('interval', seconds=5)
def sensor_display_job():
        humidity,temperature = dht.read_retry(dht.DHT11, 17)
        humidity = round(humidity, 2)
        temperature = round(temperature, 2)
        sensor_data['temp'] = temperature
        sensor_data['humidity'] = humidity
        #ec= str(float (ser.readline(),16))
        jsondata = ser.readline().decode("utf-8")
        data = json.loads(jsondata)
        
        #print(str(ser.readline()))
        #ph= str(float (ser.readline(),16))
        #data = json.loads(str(ser.readline()))
        sensor_data['ec'] = data.get('ec')
        sensor_data['ph'] = data.get('ph')
        
        print(sensor_data)
        mylcd.lcd_display_string('T:'+str(sensor_data['temp'])+'C  H:'+str(sensor_data['humidity']),1)
        mylcd.lcd_display_string('EC:'+str(sensor_data['ec'])+' pH:'+str(sensor_data['ph']),2)

# Schedule for MQTT -  publish sensor_data = {'temp': 0, 'humidity': 0 ,'ec':0,'ph':0}
@sched.scheduled_job('interval', seconds=10)
def publish_job(): 
        #mylcd.lcd_clear()
        print('PUBLISHED')
        #mylcd.lcd_display_string('PUBLISHED ',2)
        client.loop_start()
        #mylcd.lcd_display_string('T:'+str(sensor_data['temp'])+'C  H:'+str(sensor_data['humidity']),1)
        #mylcd.lcd_display_string('EC:'+str(sensor_data['ec'])+' pH:'+str(sensor_data['ph']),2)
        client.publish('hyp/sv', json.dumps(sensor_data), 1)
        
        client.loop_stop()
       
# Schedule for Motor 
@sched.scheduled_job('interval', seconds=3000)
def motor_job():
    GPIO.output(13,True)
    time.sleep(1)
    GPIO.output(13,False)
    if GPIO.input(37) == 0:
        print ("Motor ON")
        GPIO.output(37,True)
        time.sleep(30)
        print ("Motor OFF")
        GPIO.output(37,False)
    #else:
     #   print 'Motor ON'
      #  GPIO.output(37,True)

# Schedule for Light-vnfkvgfnngd
@sched.scheduled_job('interval', seconds=3000)
def li_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)
        print ("Light ON")
        GPIO.output(33,True)
        time.sleep(3)
        print( "Lights OFF")
        GPIO.output(33,False)

# TURN ON LIGHTS
@sched.scheduled_job('cron', day_of_week='mon-sun', hour=9)
def lighton_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)        
        GPIO.output(33,True)
        print ("LIGHTS ON")

# TURN OFF LIGHTS
@sched.scheduled_job('cron', day_of_week='mon-sun', hour=17)
def liightoff_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)        
        GPIO.output(33,True)
        print ("LIGHTS OFF")

sched.start()






client.disconnect()

