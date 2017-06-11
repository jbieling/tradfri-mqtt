#!/usr/bin/env python

#
# MIT License
#
# Copyright (c) 2017 Jakob Bieling
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#


from time import sleep
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt

DEBUG = False

MQTT_BROKER = "your.mqtt.broker.here"
MQTT_TOPIC = "bedroom/lights"
MQTT_USERNAME = "user"
MQTT_PASSWORD = "pass"

TX_PIN = 2
TOUCHLINK_RESET_PIN = 3
COLOR_PIN = 4
DIM_DOWN_PIN = 17
DIM_UP_PIN = 22
ONOFF_PIN = 27



POWER_STATE_RANGE = (0,1)
COLOR_STATE_RANGE = (0,2)
BRIGHTNESS_STATE_RANGE = (0,6)


statePower = 1	# 0,1 for on/off
stateBright = BRIGHTNESS_STATE_RANGE [1]

stateColor = 0	# 0, 1, 2 for the three colors
stateHue = 0
stateSat = 0



def clamp (val, range):
    if val < range [0]:
       val = range [0]
    if val > range [1]:
       val = range [1]
    return val

def percentageToBrightnessLevel(percent):
    conversionFactor = (BRIGHTNESS_STATE_RANGE[1] - BRIGHTNESS_STATE_RANGE[0]) / 100.0
    return int(percent * conversionFactor)

def hueSaturationToColorIndex(hue, sat):
    if sat < 10:
        return 0 # normal
    elif hue < 97.5 or hue > 307.5:
        return 1 # warm
    else:
        return 2 # cold

def toggle(pin, triggerDuration=0.05, waitDuration=0.05):
    GPIO.output (pin, False)
    sleep (triggerDuration)
    GPIO.output (pin, True)
    sleep (waitDuration)

def setState(pin, oldState, stateIncrement, newState, stateRange):
    if DEBUG == True:
        print "setState(%i, %i, %i, %i, (%i, %i))" % (pin, oldState, stateIncrement, newState, stateRange [0], stateRange [1])

    newState = clamp (newState, stateRange)

    waitDuration = 1.0
    if pin == COLOR_PIN:
        waitDuration = 2.0

    while oldState != newState:
       if DEBUG == True:
           print "state " + str(oldState) + " " + str(newState)
       toggle(pin, waitDuration = waitDuration)
       oldState = stateRange [0] + (oldState + stateIncrement) % (stateRange [1] - stateRange [0] + 1)

    return newState



def on_connect(mqttc, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_TOPIC + "/#")

def on_message(client, userdata, msg):
    global statePower, stateColor, stateBright, stateHue, stateSat

    print(msg.topic+" "+str(msg.payload))

    if msg.payload == "true":
        newValue = 1
    elif msg.payload == "false":
        newValue = 0
    else:
        try:
            newValue = int(msg.payload)
        except:
            return

    if msg.topic == MQTT_TOPIC + "/power":
        statePower = setState (ONOFF_PIN, statePower, +1, newValue, POWER_STATE_RANGE)
    elif msg.topic == MQTT_TOPIC + "/color":
        stateColor = setState (COLOR_PIN, stateColor, +1, newValue, COLOR_STATE_RANGE)
    elif msg.topic == MQTT_TOPIC + "/hue":
        stateHue = newValue
        stateColor = setState (COLOR_PIN, stateColor, +1, hueSaturationToColorIndex (stateHue, stateSat), COLOR_STATE_RANGE)
    elif msg.topic == MQTT_TOPIC + "/saturation":
        stateSat = newValue
        stateColor = setState (COLOR_PIN, stateColor, +1, hueSaturationToColorIndex (stateHue, stateSat), COLOR_STATE_RANGE)
    elif msg.topic == MQTT_TOPIC + "/brightness":
        newValue = percentageToBrightnessLevel (newValue)
        if newValue > stateBright:
            stateBright = setState (DIM_UP_PIN, stateBright, +1, newValue, BRIGHTNESS_STATE_RANGE)
        elif newValue < stateBright:
            stateBright = setState (DIM_DOWN_PIN, stateBright, -1, newValue, BRIGHTNESS_STATE_RANGE)

    elif msg.topic == MQTT_TOPIC + "/calibrate/auto":
        statePower = POWER_STATE_RANGE[1]
        stateColor = COLOR_STATE_RANGE[0]
        stateBright = BRIGHTNESS_STATE_RANGE[1]
        toggle (ONOFF_PIN, 3.5)        
    elif msg.topic == MQTT_TOPIC + "/calibrate/power":
        statePower = clamp (newValue, POWER_STATE_RANGE)
    elif msg.topic == MQTT_TOPIC + "/calibrate/color":
        stateColor = clamp (newValue, COLOR_STATE_RANGE)
    elif msg.topic == MQTT_TOPIC + "/calibrate/brightness":
        stateBright = clamp (percentageToBrightnessLevel (newValue), BRIGHTNESS_STATE_RANGE)




if __name__ == '__main__':
    try:
        GPIO.setwarnings (True)
        GPIO.setmode (GPIO.BCM)

        GPIO.setup (TOUCHLINK_RESET_PIN, GPIO.OUT, initial=True)
        GPIO.setup (COLOR_PIN, GPIO.OUT, initial=True)
        GPIO.setup (DIM_DOWN_PIN, GPIO.OUT, initial=True)
        GPIO.setup (DIM_UP_PIN, GPIO.OUT, initial=True)
        GPIO.setup (ONOFF_PIN, GPIO.OUT, initial=True)


        client = mqtt.Client(protocol=mqtt.MQTTv31)
        client.on_connect = on_connect
        client.on_message = on_message

        print("Connecting to " + MQTT_BROKER)
        client.username_pw_set (MQTT_USERNAME, MQTT_PASSWORD)
        client.connect(MQTT_BROKER, 1883, 60)

        # Blocking call that processes network traffic, dispatches callbacks and
        # handles reconnecting.
        # Other loop*() functions are available that give a threaded interface and a
        # manual interface.
        client.loop_forever()

    finally:
        GPIO.cleanup()
