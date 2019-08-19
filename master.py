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
'''
config = {'bz' :13,'dz1' : 15,'dz2' : 16,'exfan1':18,'exfan2' : 22,'ofan' : 29,'ac' : 31,'rm1' : 36,'rl1' : 37}
print(config)
for key in config:
    print(config[key])



#########################################
#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) 2010-2013 Roger Light <roger@atchoo.org>
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Distribution License v1.0
# which accompanies this distribution.
#
# The Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Roger Light - initial implementation
# Copyright (c) 2010,2011 Roger Light <roger@atchoo.org>
# All rights reserved.

# This shows a simple example of an MQTT subscriber.

#import context  # Ensures paho is in PYTHONPATH
import paho.mqtt.client as mqtt
import json


def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))


def on_message(mqttc, obj, msg):
    if msg.topic == 'hyp/ctrl':
        data =json.loads(msg.payload.decode("utf-8"))
        for attribute, value in data.items():
            print(attribute,value)
            #substitute gpio reads and writes here 
    elif msg.topic == 'hyp/state':
        data =json.loads(msg.payload.decode("utf-8"))
        if data['type'] == 'req':
            print("someone requested state bro")
            #here goes publishing code of status
        
        

    
    #print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


def on_publish(mqttc, obj, mid):
    print("mid: " + str(mid))


def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def on_log(mqttc, obj, level, string):
    print(string)


# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.
mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
# mqttc.on_log = on_log
mqttc.connect("165.22.209.7", 1883, 60)
mqttc.subscribe("hyp/ctrl", 0)
mqttc.subscribe("hyp/state", 0)


mqttc.loop_forever()
'''

def on_message(mqttc, obj, msg):
    print(msg.topic)
    if msg.topic == 'hyp/ctrl':
        data =json.loads(msg.payload.decode("utf-8"))
        for attribute, value in data.items():
            if value == 'ON':
                print('SWITCH ON')
                GPIO.output(config[attribute],True)
            elif value == 'OFF':
                print('SWITCH OFF')
                GPIO.output(config[attribute],False)
    elif msg.topic == 'hyp/state':
        print('STATE')
        data =json.loads(msg.payload.decode("utf-8"))
        if data['type'] == 'req':
            resp = {}
            for item,pin in config.items():
                if GPIO.input(pin) is 1:
                    resp[item] = 'ON'
                else:
                    resp[item] = 'OFF'
            print('Before')
            client.publish('hyp/state', json.dumps(resp), 1) #publishing to topic hyp/sv 
            
            print("someone requested state bro")
            #here goes publishing code of status
        
                            
            
            #print(attribute,value)
            #substitute gpio reads and writes here 

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
config = {'bz' :13,'dz1' : 15,'dz2' : 16,'exfan1':18,'exfan2' : 22,'ofan' : 29,'ac' : 31,'rm1' : 36,'rl1' : 37}
print(config)
for key in config:
    print(config[key])
    GPIO.setup(config[key],GPIO.OUT) #     BUZZER
GPIO.setup(33,GPIO.OUT) 
GPIO.setup(32,GPIO.OUT) 
'''
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
'''
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
client.on_message = on_message
client.is_connected = True
client.subscribe('hyp/ctrl')
#Serial Communication for ANALOG INPUT
#ser = serial.Serial('/dev/ttyACM0',9600) #for Arduino Uno ttyACM0
ser = serial.Serial('/dev/ttyUSB0',9600) #for Ardunio Nano ttyUSBo
#client.loop_forever()


@sched.scheduled_job('interval', seconds=1)
def mainsomething_job():
    client.loop_start()


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
        value = str(sensor_data['ec'])

#drowsing systerm part hardcoded part of ec
@sched.scheduled_job('interval', seconds=10)
def sensor_value():        
        if sensor_data['ec'] > str(2):
            GPIO.output(33,True)
            time.sleep(1)
        else:
            GPIO.output(32,True)
            time.sleep(1)
# Schedule for MQTT -  publish sensor_data = {'temp': 0, 'humidity': 0 ,'ec':0,'ph':0}
@sched.scheduled_job('interval', seconds=10)
def publish_job(): 
        print('PUBLISHED')
        
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
