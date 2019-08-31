"""
Demonstrates how to use the background scheduler to schedule a job that executes on 3 second
intervals.
"""
import serial
import os
import time
import sys
import Adafruit_DHT as dht
import paho.mqtt.client as paho
import json
import RPi.GPIO as GPIO
import socket
import RPi_I2C_driver
import json

from apscheduler.schedulers.background import BackgroundScheduler
scheduler = BackgroundScheduler()


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
GPIO.setmode(GPIO.BOARD) # GPIO.BOARD mode
GPIO.setwarnings(False) # disable warnings - Your not a noob to want warnings
config = {'bz' :13,'dz1' : 15,'dz2' : 16,'exfan1':18,'exfan2' : 22,'ofan' : 29,'ac' : 31,'rm1' : 36,'rl1' : 37}
print(config)
for key in config:
    print(config[key])
    GPIO.setup(config[key],GPIO.OUT) #     BUZZER
for key in config:
    print(config[key])
    GPIO.output(config[key],True)
GPIO.output(13,False)
    #GPIO.setup(config[key],GPIO.OUT) #     BUZZER
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


def on_message(mqttc, obj, msg):
    print(msg.topic)
    if msg.topic == 'hyp/ctrl':
        data =json.loads(msg.payload.decode("utf-8"))
        print(data)
        for attribute, value in data.items():
            if value == 'OFF':
                print('SWITCH OFF')
                GPIO.output(config[attribute],True)
            elif value == 'ON':
                print('SWITCH ON')
                GPIO.output(config[attribute],False)
    elif msg.topic == 'hyp/time':
        print('TIME CHANGE')
        data = json.loads(msg.payload.decode("utf-8"))
        if data['item'] == 'mtr':
            print('Rescheduling Motor time to ',data['i'])
            scheduler.reschedule_job("motor_job", trigger='interval', minutes=int(data['i']))
        elif data['item'] == 'light':
            if data['state'] == 'OFF':
                print('Rescheduling Light OFF time to ',data['i'])
                scheduler.reschedule_job("lightoff_job", trigger='cron', day_of_week='mon-sun',hour=int(data['i']))
            elif data['state'] == 'ON':
                print('Rescheduling Light On time to ',data['i'])
                scheduler.reschedule_job("lighton_job", trigger='cron', day_of_week='mon-sun',hour=int(data['i']))
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


#MQTT credentials
client = paho.Client("HYP-MASTER-RASPI")
broker = '165.22.209.7' #server ip
port = 1883 #default mqtt port
try:
    client.connect(broker, 1883, 60)
except: 
    print("oops") 
    
client.on_message = on_message
client.is_connected = True
client.subscribe('hyp/ctrl')
client.subscribe('hyp/state')
client.subscribe('hyp/time')
client.loop_start()
ser = serial.Serial('/dev/ttyUSB0',9600) #for Ardunio Nano ttyUSBo





#Populate & Display sensor data
#@sched.scheduled_job('interval', seconds=5)
def sensor_display_job():
        humidity,temperature = dht.read_retry(dht.DHT11, 17) #17 = GPIO.BOARD-11 As dhtLibrary uses BCM.BOARD mode
        humidity = round(humidity, 2)  
        temperature = round(temperature, 2)
        sensor_data['temp'] = temperature
        sensor_data['humidity'] = humidity
        #serial read ec and ph from nano - JSON structure {"ec":"value","ph":"value"} 
        jsondata = ser.readline().decode("utf-8")
        print('Dina ASKED ',jsondata)
        data = json.loads(jsondata)        
        sensor_data['ec'] = data.get('ec')
        sensor_data['ph'] = data.get('ph')
        print(sensor_data) #log sensor_data dict
        mylcd.lcd_display_string('T:'+str(sensor_data['temp'])+'C  H:'+str(sensor_data['humidity']),1) #display row1
        mylcd.lcd_display_string('EC:'+str(sensor_data['ec'])+' pH:'+str(sensor_data['ph']),2) #display row2
        value = str(sensor_data['ec'])

# Schedule for MQTT -  publish sensor_data = {'temp': 0, 'humidity': 0 ,'ec':0,'ph':0}
#@sched.scheduled_job('interval', seconds=10)
def publish_job(): 
        print('PUBLISHED')
        
        client.publish('hyp/sv', json.dumps(sensor_data), 1) #publishing to topic hyp/sv 
        #client.loop_stop()
 
# Schedule for Motor - GPIO 36
#@sched.scheduled_job('interval', seconds=3000)
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
#@sched.scheduled_job('cron', day_of_week='mon-sun', hour=9)
def lighton_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)        
        GPIO.output(37,False)
        print ("LIGHTS ON")

# TURN OFF LIGHTS - GPIO 37
#@sched.scheduled_job('cron', day_of_week='mon-sun', hour=17)
def lightoff_job():
        GPIO.output(13,True)
        time.sleep(1)
        GPIO.output(13,False)        
        GPIO.output(37,True)
        print ("LIGHTS OFF")
'''
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
       
'''



if __name__ == '__main__':
    scheduler.add_job(sensor_display_job, 'interval', seconds=10, id="sensor_display_job")
    scheduler.add_job(publish_job, 'interval', seconds=60, id="publish_job")
    scheduler.add_job(motor_job, 'interval', minutes=15, id="motor_job")
    scheduler.add_job(lighton_job, 'cron', day_of_week='mon-sun',hour=9, id="lighton_job")
    scheduler.add_job(lightoff_job, 'cron', day_of_week='mon-sun',hour=17, id="lightoff_job")
    scheduler.start()
    print('Press Ctrl+{0} to exit'.format('Break' if os.name == 'nt' else 'C'))

    try:
        # This is here to simulate application activity (which keeps the main thread alive).
        #a = input("Enter your name")
        #if a=="john":
         #   scheduler.reschedule_job("surya", trigger='interval', seconds=1)
         #   print(a)
        while True:
            time.sleep(2)
    except (KeyboardInterrupt, SystemExit):
        # Not strictly necessary if daemonic mode is enabled but should be done if possible
        scheduler.shutdown()
