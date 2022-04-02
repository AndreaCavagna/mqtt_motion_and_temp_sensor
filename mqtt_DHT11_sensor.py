import Adafruit_DHT
import RPi.GPIO as GPIO
import subprocess
import os
import json
import time
import subprocess
import paho.mqtt.client as mqtt
from datetime import datetime
import argparse
import statistics
import math

parser = argparse.ArgumentParser(description='')
parser.add_argument("-c", "--client", type=str, default = 'test_local_temp', help='name of the mqtt client')

args = parser.parse_args()

MOSQUITO_CLIENT_NAME = args.client


# --------- BOARD CONFIGURATION ---------- #


TEMP_LED = 11
INTERNET_CONN_LED = 12
HUM_LED = 13
HEATER_LED = 37

thermostat_pin = 14
led_brightness = 100

GPIO.setmode(GPIO.BOARD)

GPIO.setup(HUM_LED, GPIO.OUT)
GPIO.setup(TEMP_LED, GPIO.OUT)
GPIO.setup(INTERNET_CONN_LED, GPIO.OUT)
GPIO.setup(HEATER_LED, GPIO.OUT)

#GPIO.output(HUM_LED, GPIO.LOW)
#GPIO.output(TEMP_LED, GPIO.LOW)
#GPIO.output(INTERNET_CONN_LED, GPIO.LOW)
#GPIO.output(HEATER_LED, GPIO.LOW)
HUM_LED_PWM = GPIO.PWM(HUM_LED,10)
TEMP_LED_PWM = GPIO.PWM(TEMP_LED,10)
INTERNET_CONN_LED_PWM = GPIO.PWM(INTERNET_CONN_LED,10)
HEATER_LED_PWM = GPIO.PWM(HEATER_LED,10)


HUM_LED_PWM.start(0)
TEMP_LED_PWM.start(0)
INTERNET_CONN_LED_PWM.start(0)
HEATER_LED_PWM.start(0)



HUM_LED_PWM.ChangeDutyCycle(led_brightness)
TEMP_LED_PWM.ChangeDutyCycle(led_brightness)
INTERNET_CONN_LED_PWM.ChangeDutyCycle(led_brightness)
HEATER_LED_PWM.ChangeDutyCycle(led_brightness)


# --------- TIMINGS ---------- #

HUM_MAX_TRESH = 55
HUM_MIN_TRESH = 40

TEMP_MAX_THRES = 25
TEMP_MIN_THRES = 20

LOOP_TIMEOUT = 150

LEDs_OFF_START_TIME = -1
LEDs_OFF_END_TIME = 25

# --------- RECONNECTIONS ---------- #

MAX_BLE_RETRIES = 3

# --------- CURRENT MODES ---------- #

current_hvac_mode = 'off'

# --------- LED BRIGHTNESS ---------- #

LED_BRIGHTNESS_HIGH = 100

LED_BRIGHTNESS_LOW = 0

# --------- END CONFIGURATION ---------- #

def calculate_dew_point(T,RH):
  b = 17.62
  c = 243.12

  def gamma_func(T,RH,b,c):
      return math.log(RH/100) + (b * T)/(c + T)
  
  try:
    return float((c*gamma_func(T,RH,b,c)) / (b - gamma_func(T,RH,b,c)))
  except:
    return -10
  
  
def on_connect(mqttc, obj, flags, rc):
    #print("rc: "+str(rc))
    pass

def on_publish(mqttc, obj, mid):
    #print("mid: "+str(mid))
    pass

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))
        
def on_message(client, userdata, msg):
  msg_payload_decoded = msg.payload.decode("utf-8")
  
  if msg.topic == 'esp32/dht/temperature':
    global espTemp_lastRead 
    espTemp_lastRead = float(msg_payload_decoded)
    
  elif msg.topic == 'homeAssistant/thermostatBedroom/status':
    heater_status = str(msg_payload_decoded)
    global HEATER_LED_PWM
    global led_brightness
    if heater_status == "heat":
      HEATER_LED_PWM.ChangeDutyCycle(led_brightness)
      #GPIO.output(HEATER_LED, GPIO.HIGH)
    elif heater_status == "off":
      #GPIO.output(HEATER_LED, GPIO.LOW)
      HEATER_LED_PWM.ChangeDutyCycle(0)
      
  elif msg.topic == 'homeAssistant/thermostatBedroom/hvac_action':
    global current_hvac_mode
    current_hvac_mode = msg_payload_decoded
      
  elif msg.topic == 'esp32/dht/humidity':
    global espHum_lastRead 
    espHum_lastRead =  float(msg_payload_decoded)
     
  #print("Message received-> " + msg.topic + " " + str(msg_payload_decoded))
  
  
  
def on_disconnect(client, userdata, rc):
    connect_to_broker()
    
    
def connect_to_broker():
    not_connected = True
    while not_connected:
      try:
        client.connect('myhomeipdk.hopto.org', port=1883)
        not_connected = False
        print(datetime.now())
        print('Im connected')
        client.subscribe("esp32/dht/#")
        client.subscribe("homeAssistant/thermostatBedroom/#")
        time.sleep(10)
      except:
        print(datetime.now())
        print('Failed connection')
        time.sleep(3)
  
      
client = mqtt.Client(MOSQUITO_CLIENT_NAME)
connect_to_broker()
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.on_subscribe = on_subscribe
client.loop_start()


bathroom_temp_queue = [0,0,0,0,0,0,0,0,0,0]
bathroom_hum_queue = [0,0,0,0,0,0,0,0,0,0]

flag_first_cycle = True
queue_curr_index = 0

while True:
  try:
            hum_min_flag = False
            temp_min_flag = False
            
            try:
              hum, temp = Adafruit_DHT.read_retry(11, thermostat_pin)
            except:
              print('Error in the dht thermostat')
              time.sleep(10)
              continue
            
            if flag_first_cycle:
              flag_first_cycle = False
              for i in range(len(bathroom_temp_queue)):
                bathroom_temp_queue[i] = temp
                bathroom_hum_queue[i] = hum
            else:
              bathroom_temp_queue[queue_curr_index] = temp
              bathroom_hum_queue[queue_curr_index] = hum
              queue_curr_index += 1
              if queue_curr_index >= len(bathroom_temp_queue):
                queue_curr_index = 0
            
            temp_bathroom_mean = statistics.mean(bathroom_temp_queue)
            hum_bathroom_mean = statistics.mean(bathroom_hum_queue)
              
            
            client.publish("ambient/bathroom/temperature", "{:.1f}". format(temp_bathroom_mean))
            client.publish("ambient/bathroom/humidity", "{:.1f}". format(hum_bathroom_mean))
            client.publish("ambient/bathroom/dew_point", "{:.1f}". format(calculate_dew_point(temp_bathroom_mean,hum_bathroom_mean)))
            
            
            address = "1.1.1.1"
            res = subprocess.call(['ping', '-c', '1', address], stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
            
            '''if res is not 0:
              INTERNET_CONN_LED_PWM.ChangeDutyCycle(led_brightness)
            else:
              INTERNET_CONN_LED_PWM.ChangeDutyCycle(0)'''
              
            
            if datetime.now().hour <= LEDs_OFF_START_TIME and datetime.now().hour >= LEDs_OFF_END_TIME:
              led_brightness = LED_BRIGHTNESS_HIGH
              if temp_bathroom_mean >= TEMP_MAX_THRES:
                TEMP_LED_PWM.ChangeDutyCycle(led_brightness)
              elif temp_bathroom_mean <= TEMP_MIN_THRES:
                temp_min_flag = True
              else:
                TEMP_LED_PWM.ChangeDutyCycle(0)
  
              if hum_bathroom_mean >= HUM_MAX_TRESH:
                HUM_LED_PWM.ChangeDutyCycle(led_brightness)
              elif hum_bathroom_mean <= HUM_MIN_TRESH:
                hum_min_flag = True
              else:
                HUM_LED_PWM.ChangeDutyCycle(0)
                        
                
              if hum_min_flag or temp_min_flag:
                for i in range(50):
                
                  if temp_min_flag:
                    TEMP_LED_PWM.ChangeDutyCycle(led_brightness)
                  if hum_min_flag:
                    HUM_LED_PWM.ChangeDutyCycle(led_brightness) 
                  time.sleep(0.5)
                  
                  if temp_min_flag:
                    TEMP_LED_PWM.ChangeDutyCycle(0)
                  if hum_min_flag:
                    HUM_LED_PWM.ChangeDutyCycle(0) 
                  time.sleep(0.5)
                  
              else:
                time.sleep(LOOP_TIMEOUT)
            else:
              led_brightness = LED_BRIGHTNESS_LOW
              TEMP_LED_PWM.ChangeDutyCycle(led_brightness)
              HUM_LED_PWM.ChangeDutyCycle(led_brightness)
              INTERNET_CONN_LED_PWM.ChangeDutyCycle(led_brightness)
              time.sleep(LOOP_TIMEOUT)
            
                 
  except:
    bathroom_temp_queue = [0,0,0,0,0,0,0,0,0,0]
    bathroom_hum_queue = [0,0,0,0,0,0,0,0,0,0]

    flag_first_cycle = True
    queue_curr_index = 0
    time.sleep(15)
    
    
              