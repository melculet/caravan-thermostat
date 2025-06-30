# Thermostat Control Script
# Uses BME280 for room temp and DS18B20 for coolant return temp

import time
import board
import adafruit_bme280
from w1thermsensor import W1ThermSensor
import serial
import RPi.GPIO as GPIO

# Relay GPIO Setup
BURNER_RELAY_PIN = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(BURNER_RELAY_PIN, GPIO.OUT)
GPIO.output(BURNER_RELAY_PIN, GPIO.LOW)

# OneWire for DS18B20
coolant_sensor = W1ThermSensor()

# I2C BME280
i2c = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# UART for Nextion
ser = serial.Serial("/dev/serial0", 9600, timeout=1)

# Settings
setpoint = 22.0  # Default target temperature
hysteresis = 0.5  # Degrees of swing allowed


def read_room_temperature():
    return bme280.temperature


def read_coolant_temperature():
    return coolant_sensor.get_temperature()


def send_to_display(room_temp, coolant_temp, burner_on):
    ser.write(f"room.val={int(room_temp * 10)}\xff\xff\xff".encode())
    ser.write(f"coolant.val={int(coolant_temp * 10)}\xff\xff\xff".encode())
    ser.write(f"burner.pic={1 if burner_on else 0}\xff\xff\xff".encode())


def update_setpoint_from_display():
    ser.write("get setpoint.val\xff\xff\xff".encode())
    if ser.in_waiting:
        try:
            response = ser.read(7)
            val = response[1] + response[2] * 256
            return val / 10.0
        except:
            return None
    return None


def control_loop():
    burner_on = False
    while True:
        room_temp = read_room_temperature()
        coolant_temp = read_coolant_temperature()
        new_setpoint = update_setpoint_from_display()
        if new_setpoint:
            global setpoint
            setpoint = new_setpoint

        if room_temp < (setpoint - hysteresis):
            burner_on = True
        elif room_temp > (setpoint + hysteresis):
            burner_on = False

        GPIO.output(BURNER_RELAY_PIN, GPIO.HIGH if burner_on else GPIO.LOW)
        send_to_display(room_temp, coolant_temp, burner_on)

        time.sleep(2)


try:
    control_loop()
except KeyboardInterrupt:
    GPIO.cleanup()
