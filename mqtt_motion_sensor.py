import time
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import socket
from datetime import datetime
import argparse

# ------------------- MAIN  CONFIGURATION ------------------- #
    
XIAOMI_THERMOSTAT_MAC = "4c:65:a8:da:89:e9"
MQTT_TOPIC = "ambient/andreaBedroom/motionSensor"
MQTT_SERVER_ADDRESS = "192.168.123.16" # hostname or IP address
MQTT_SERVER_PORT = 1883
MOTION_SENSOR_PIN = 35

# ------------------- TIMINGS ------------------- #


TIME_MOTION_DEPRECATED_MINUTES_DAY = 10
TIME_MOTION_DEPRECATED_MINUTES_NIGHT = 5
TIME_MOTION_DEPRECATED_HOURS_SHUTS_ALL = 4
REFRESH_AVAILABILITY_MINUTES = 5

STARTING_DAYTIME = 8 # starting hour to use TIME_MOTION_DEPRECATED_MINUTES_DAY
ENDING_DAYTIME = 24 # starting hour to use TIME_MOTION_DEPRECATED_MINUTES_NIGTH


PAUSE_SEC_BETWEEN_MOTION_DETECTED = 5
SENSOR_REFRESH_TIME_SEC = 0.5

# ------------------- END CONFIGURATION ------------------- #

parser = argparse.ArgumentParser(description='')
parser.add_argument("-c", "--client", type=str, default = 'MQTT_MOTION_SENSOR', help='name of the mqtt client')
parser.add_argument("-t", "--topic", type=str, default = MQTT_TOPIC, help='mqtt topic to use')
parser.add_argument("-s", "--server", type=str, default = MQTT_SERVER_ADDRESS, help='mqtt server to use')
parser.add_argument("-p", "--sensorPin", type=str, default = MOTION_SENSOR_PIN, help='Pin connected to the motion sensor')

args = parser.parse_args()

MOSQUITO_CLIENT_NAME = args.client

TIME_MOTION_DEPRECATED_SEC_DAY = 60 * TIME_MOTION_DEPRECATED_MINUTES_DAY
TIME_MOTION_DEPRECATED_SEC_NIGHT = 60 * TIME_MOTION_DEPRECATED_MINUTES_NIGHT

TIME_MOTION_DEPRECATED_SEC_SHUTS_ALL = 60 * 60 * TIME_MOTION_DEPRECATED_HOURS_SHUTS_ALL


GPIO.setmode(GPIO.BOARD)

GPIO.setup(args.sensorPin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)


def on_disconnect(client, userdata, rc):
    connect_to_broker()
    
    
def connect_to_broker():
    not_connected = True
    while not_connected:
      try:
        client.connect(MQTT_SERVER_ADDRESS, port = MQTT_SERVER_PORT)
        not_connected = False
        print(datetime.now())
        print('Im connected')
        time.sleep(10)
      except:
        print(datetime.now())
        print('Failed connection')
        time.sleep(3)
  
      
client = mqtt.Client(MOSQUITO_CLIENT_NAME)
connect_to_broker()
client.on_disconnect = on_disconnect
client.loop_start()

last_motion_detected = datetime.now()
not_signaled_state_transition = True
not_signaled_state_transition_shuts_all = True
signaled_availability = True

client.publish(MQTT_TOPIC + "/motionValue", "0")
client.publish(MQTT_TOPIC + "/motionValueRaw", "0")
client.publish(MQTT_TOPIC + "/availability", "online")

time.sleep(2)

last_motion_value_raw = 0

while True:
  try:
    curr_minute = datetime.now().minute
    
    if signaled_availability == False and int(curr_minute) % REFRESH_AVAILABILITY_MINUTES == 0:
      client.publish(MQTT_TOPIC + "/availability", "online")
      signaled_availability = True
    else:
      signaled_availability = False
      
    
    if GPIO.input(MOTION_SENSOR_PIN) is 1:
        
      last_motion_detected = datetime.now()
      #print(GPIO.input(MOTION_SENSOR_PIN))
      #print(last_motion_detected)
      not_signaled_state_transition = True
      not_signaled_state_transition_shuts_all = True
      client.publish(MQTT_TOPIC + "/motionValue", "1")
      client.publish(MQTT_TOPIC + "/availability", "online")
      
      if (last_motion_value_raw is 0):
        client.publish(MQTT_TOPIC + "/motionValueRaw", "1") # use this message to get the raw values from the sensor, so they can be used directly in home assistant
        last_motion_value_raw = 1
      
      time.sleep(PAUSE_SEC_BETWEEN_MOTION_DETECTED)
      
    else:
    
      curr_hour = datetime.now().hour
      
      if (last_motion_value_raw is 1):
        client.publish(MQTT_TOPIC + "/motionValueRaw", "0")
        last_motion_value_raw = 0
      
      if curr_hour >= STARTING_DAYTIME and curr_hour <= ENDING_DAYTIME:
        TIME_MOTION_DEPRECATED_SEC = TIME_MOTION_DEPRECATED_SEC_DAY
      else:
        TIME_MOTION_DEPRECATED_SEC = TIME_MOTION_DEPRECATED_SEC_NIGHT
      
      if not_signaled_state_transition and (datetime.now() - last_motion_detected).total_seconds() > TIME_MOTION_DEPRECATED_SEC:
        not_signaled_state_transition = False
        client.publish("homeAssistant/motion/"+ MOSQUITO_CLIENT_NAME+"/motionValue", "0")
        client.publish("homeAssistant/motion/"+ MOSQUITO_CLIENT_NAME+"/availability", "online")
       
       
      
      elif not_signaled_state_transition_shuts_all:
          ''' HERE YOU CAN SEND A MQTT MESSAGE TO HOME ASSISTANT AND SHUT DOWN ALL THE LIGHTS
          IF NO MOTION IS DETECTED FOR A LONG TIME'''
          time.sleep(1)
       
      else:
        time.sleep(SENSOR_REFRESH_TIME_SEC)
      
  except Exception as e:
    print(e)
    time.sleep(15)
    