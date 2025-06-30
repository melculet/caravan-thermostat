# Caravan Thermostat

This Raspberry Pi-based thermostat system uses a BME280 sensor for room temperature,
a DS18B20 sensor for coolant return temperature, and a relay to control a burner. The
system communicates with a Nextion NX3224T024_011 touchscreen display.

## Features
- Adjustable setpoint via Nextion touchscreen
- Burner control with hysteresis
- Continuous recirculating pump (outside this script)
- Serial communication with Nextion
- Safe to power on/off directly

## Hardware Connections

### Sensors
- **BME280** (Room Temp)
  - SDA → Pin 3 (GPIO2)
  - SCL → Pin 5 (GPIO3)
  - VCC → 3.3V (Pin 1)
  - GND → GND (Pin 6)

- **DS18B20** (Coolant Return Temp)
  - Data → GPIO4 (Pin 7)
  - VCC → 3.3V (Pin 1)
  - GND → GND (Pin 6)
  - Use 4.7kΩ pull-up resistor or equivalent on Data ↔ VCC

### Relay (Burner)
- Signal → GPIO22 (Pin 15)
- VCC/GND as required by relay board

### Nextion Display
- TX → RX (Pin 10, GPIO15)
- RX → TX (Pin 8, GPIO14)
- VCC → 5V (Pin 2)
- GND → GND (Pin 6)

## Software Setup

```bash
sudo apt update
sudo apt install python3-pip git
pip3 install adafruit-circuitpython-bme280 w1thermsensor pyserial
