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
