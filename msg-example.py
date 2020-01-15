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
import base64
import Queue
import RPi.GPIO as GPIO
from time import sleep
from signal import signal, SIGINT
from sys import exit
import re
import binascii
import struct

# close down this program with ctrl c
def handler(signal_received, frame):
    print("      crl c pressed, cleanup PINS")
    GPIO.cleanup()         # clean up the GPIO PINS
    RUNNING=False          # break the main loop
    mqttc.loop_stop()      # stop the MQTT client
    mqttc.disconnect()     # dissconnect http
    exit(0)                # exit the program

if __name__ == '__main__':
    signal(SIGINT, handler)


PIN = 7                             # the pin to be controlled
PINHIGH = 5                         # number in seconds to hold pin high
RUNNING=True                        # main loop control
#q = Queue.Queue()                  # TODO process queue
GPIO.setmode(GPIO.BOARD)            # set up for pi board mode
GPIO.setup(PIN, GPIO.OUT)           # set the PIN to out mode
mqtt.Client.connected_flag=False    # connection indicator

# TODO create a class and use a queue to process messages
class Loraevent:

    def __init__(self, uuid, name, devtype, msgID, msgType, batt):
        self.uuid=uuid
        self.name=name
        self.devtype=devtype
        self.msgID=msgID
        self.msgType=msgType
        self.batt=batt


# connect ack handler
def on_connect(mqttc, obj, flags, rc):
    print("rc: " + str(rc))
    mqttc.connected_flag=True

# message handler call back
def on_message(mqttc, obj, msg):
    dataMSG = str(msg.payload)                                                                # get the message payload from the raw message
    #print("---------------------------------------------------------------------")
    #print(msg.topic + " " + str(msg.qos) + " " + dataMSG)
    message = json.loads(dataMSG)                                                             # normalize the json format for printing
    dat = message["data"]                                                                     # extract the data from the message json
    #print(dat)
    base_64bytes = dat.encode('ascii')                                                        # encode the data as base64
    message_bytes = base_64bytes.decode('base64')                                             # decode the base64 into message bytes
    #print (struct.unpack('>BBBBBBBB', message_bytes))
    message_bytes = struct.unpack('>BBBBBBBB', message_bytes)                                 #  unpack the bytes into char array
    #print("decoded: ", message_bytes)
    state = GPIO.input(PIN)                                                                   # test the current statew of the GPIO PIN
    devname = message.get("deviceName", "")
    print("Device: ",devname," Message topic: ",msg.topic)
    print("DeviceName: ",devname,"Pin: ",PIN," pin_high: ", state, " DevType: ",message_bytes[0], " msgID: ",message_bytes[1], " AppMSG: ",message_bytes[2])
    print("Decoded: ", message_bytes)
    if state:                                                                                # if the PIN is hight, state is true
       try:                                                                                  # try catch to cleanup pins for restart
           print("pin ",PIN," is already high, set to low")
           GPIO.output(PIN, GPIO.LOW)                                                        # set pin low because already high
           print("pin ", PIN , " state ", GPIO.input(PIN))                                   # print pin state
           print("-----------------------------------------------------------------------------")
       except:
           print("GPIO cmd error")
           GPIO.cleanup()                                                                    # close all pins and release on error

    else:
       try:
           GPIO.output(PIN, GPIO.HIGH)                                                      # PIN is not high set PIN to high
           print("pin ",PIN," pulled high")
           print("pin ",PIN," state ", GPIO.input(PIN))
           sleep(PINHIGH)                                                                   # sleep for PINHIGH amount of seconds
           GPIO.output(PIN, GPIO.LOW)                                                       # PIN is High, set to low
           print("pin ",PIN," pulled low ", GPIO.input(PIN))
           print("-----------------------------------------------------------------------------")
       except KeyboardInterrupt:
           print("Control c presed")
       except:
           print("GPIO cmd error")
           GPIO.cleanup()                                                                   # close all pins and release on error

# public call back triggers on publish success
def on_publish(mqttc, obj, mid):
    print("mid: " + str(mid))

# on subscribe call back on subscription success
def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

# on log call back called on log message from broker
def on_log(mqttc, obj, level, string):
    print(string)


# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.




mqttc = mqtt.Client()                                                                  # create a MQTT client object
mqttc.on_message = on_message                                                          # attach call back to function named "on_message"
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
mqttc.loop_start()                                                                     # (blocking) start checking broker for messages and publish buffer for out going messages
                                                                                       # auto reconnect funtion is active
# Uncomment to enable debug messages
# mqttc.on_log = on_log

mqttc.connect("localhost", 1883, 60)                                                   # connect to the Broker on ip/port/qos
while not mqttc.connected_flag:                                                        # loop till connected
     print("connecting to broker")
     sleep(1)

print("Main loop started, connection established")

mqttc.subscribe("#", 0)                                                                # subscribe to gateway all messages

print("listening for messages")

while(RUNNING):                                                                       # loop to prevent program from 
     sleep(.1)




#mqttc.loop_forever()                                                                 # remove loop_start() and use this for deamon mode
