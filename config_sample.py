"""
FiPy NB-IoT NODE - CONFIGURATION

---
IoT PLATFORM - PyCom FiPy NB-IoT Node
---

(c) Matteo M. 2022

"""

# Node name
NODEID = 'node_01'

# Location / Intallation name
INSTALLATIONID = 'sensor_group_01'

# Measurement interval
MEAS_INTERVAL = 300         # seconds

# Watchdog timeout 
WATCHDOG = 600             # seconds

# GSM Signal measurement interval
GSMRSSI_INTERVAL = 3600    # seconds


# GSM MODEM CONFIGURATION
GSMBAND = 20
APN = 'iot.server.com'     # APN 

# MQTT CONFIGURATION
MQTTSERVER = 'mqtt.server.com'
MQTTUSER = 'user'
MQTTPWD = 'password'
MQTTPORT = 1883
MQTT_TOPIC_DATA = 'topic/data'
MQTT_TOPIC_CONFIG = 'topic/config'
MQTT_KEEPALIVE = 60

# Debugging mode
DEBUG_MODE = True

