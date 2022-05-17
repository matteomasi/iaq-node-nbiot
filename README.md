# iaq-node-nbiot

Indoor Air Quality (IAQ) node built on Pycom FiPy hardware, running [MicroPython](https://micropython.org/).

The node is designed to be always on. The data is sent to a MQTT server over the NB-IOT network.

## Hardware

- Pycom FiPy (Espressif ESP32-based and MicroPython enabled development board)
- Sensirion SCD30. NDIR CO₂ gas sensor.
- Sensirion SPS30. Particulate matter (PM) sensor measuring PM1.0, PM2.5, PM4 and PM10.
- Sensirion SGP30. Multi-gas (VOC and CO₂eq) sensor. The Adafruit SGP30 Air Quality Sensor Breakout was used in this project.

## I₂C addresses

| Device | Address |
| - | - |
| SCD30 | 0x61 |
| SPS30 | 0x69 |
| SGP30 | 0x58 |

## Usage

- Rename config_sample.py to config.py
- Edit settings in config.py
- Upload the project to the FiPy board

## MQTT message format

The measurements are sent to the MQTT server and the data is meant to be collected into a InfluxDB database (e.g., with a telegraf instance subscribed to the data topic). To ease this process, the device creates messages formatted into the InfluxDB line protocol, as follows:

```
node01 T=22.6,H=47.3,CO2=387,tVOC=25.3,eCO2=440,tVOC_b=36500,eCO2_b=38024,PM1=7.6,PM2=8.1,PM4=8.1,PM10=8.1
```
