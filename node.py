"""
FiPy NB-IoT NODE - CLASS

---
IoT PLATFORM - PyCom FiPy NB-IoT Node
---

(c) Matteo M. 2022

"""

import machine
from machine import Timer
from machine import WDT
from machine import I2C
from network import LTE
from mqtt import MQTTClient
import pycom
import adafruit_sgp30
import scd30
import sps30
import socket
import struct
import time
import utime
import math
import config




class Node: 
    """
    Main class for node operation
    """

    def __init__(self):
        """
        Contructor / initialization
        """
        
        # Internal variables
        self._timer = None
        self._wdt = None
        self._cyclecount = 0

        # SGP30 baselines
        self.tVOC_b_rx = 36000
        self.eCO2_b_rx = 36000

        self.baseline_manual = False

        # Readout fail (from sensors) flag
        self.fail_count = 0

        # RSSI
        self.rssi = 0

        # Number of cycles to skip before GSM signal checking
        self.cycles = round(float(config.GSMRSSI_INTERVAL)/config.MEAS_INTERVAL)

        # MQTT Topics
        self.topic_data = config.MQTT_TOPIC_DATA + '/' + config.NODEID          # Data topic: application/data/nodeid/#
        self.topic_config = config.MQTT_TOPIC_CONFIG + '/' + config.NODEID      # Conf topic: application/config/nodeid/#


    def start(self):
        """
        Initial operations
        """
        # Initialize watchdog
        self._wdt = WDT(timeout=config.MEAS_INTERVAL*1000 + config.WATCHDOG*1000)   # Timeout in ms

        # Initialize I2C
        self.i2c = I2C(0, I2C.MASTER)
        self.i2c.init(I2C.MASTER, baudrate=10000)
        time.sleep_ms(200)

        # Initialize sensors
        # SGP30
        self.sgp30 = adafruit_sgp30.Adafruit_SGP30(self.i2c)
        
        # SCD30
        time.sleep_ms(200)
        self.scd30 = scd30.SCD30(self.i2c)
        time.sleep_ms(100)
        self.scd30.set_measurement_interval(2)
        time.sleep_ms(100)
        self.scd30.start_continous_measurement()
        
        
        # SPS30
        time.sleep_ms(200)
        self.sps30 = sps30.SPS30(self.i2c)
        time.sleep_ms(200)
        self.sps30.start_measurement()

        # Done
        self._log('Senosors initialization: done')

        # Set a timer for measurements at a defined time interval 
        self._timer = Timer.Alarm(self.main_loop, config.MEAS_INTERVAL, periodic=True)

    def modem_attach(self):
        """
        GSM modem attach to network
        """
        self._log('Attaching to carrier...')
        self.lte.attach(band=config.GSMBAND, apn=config.APN)
        while not self.lte.isattached():
            time.sleep(0.25)
        self._log('GSM Attached')

        # Feed the watchdog
        self._wdt.feed()    


    def modem_connect(self):
        """
        Start a data session and obtain and IP address
        """
        self._log('Connecting to NB-IOT network...')
        self.lte.connect()
        while not self.lte.isconnected():
            time.sleep(0.25)
        self._log('Connected')
        time.sleep(5.0)
        self._wdt.feed()


    def mqtt_connect(self):
        """
        Establish a connection to MQTT server
        """
        self._log('Connecting to MQTT server')
        self.mqttclient = MQTTClient(config.NODEID, config.MQTTSERVER, user=config.MQTTUSER, password=config.MQTTPWD, port=config.MQTTPORT, keepalive=config.MQTT_KEEPALIVE)
        self.mqttclient.set_callback(self.cmd_handle)   # Set callback for receiving configuration messages on config topic
        self.mqttclient.connect()

        time.sleep(5)

        # Subscribe to config topic
        self._log('Subscribe to topic: ' + self.topic_config + '/#')
        self.mqttclient.subscribe(topic=self.topic_config + '/#')

        time.sleep(1)

    
    def get_rssi(self):
        """
        Get RSSI of GSM connection
        """
        self._log('Get RSSI signal')
        # Check if modem is attached to network
        if not self.lte.isattached():
            self.modem_attach()

        # Should disconnect before sending the AT command
        if self.lte.isconnected():
            self.lte.disconnect()
            time.sleep(5.0)
        
        # Send AT command
        self._log('Send AT command: AT+CSQ')
        rssi_raw = self.lte.send_at_cmd('AT+CSQ')
        self.rssi = self.rssi_parse(rssi_raw)
        self._log('Signal: ' + str(self.rssi) + ' dBm')
        

    def main_loop(self,alarm):
        """
        MAIN LOOP - Send measurements through MQTT/NB-IOT
        """        
        
        # Check if sensors are working
        if len(self.i2c.scan()) < 3:
            self.fail_count += 1
        else:
            self.fail_count = 0     # Reset flag if everything is ok
    
        time.sleep_ms(200)
    
        if self.fail_count > 3:
            machine.reset() # Reboot if the sensors are still not working
        
        
        ###############
        # SGP30
        ###############
        
        # When baseline is received, set it at every cycle
        if self.baseline_manual: 
            self.sgp30.set_iaq_baseline(self.tVOC_b_rx,self.eCO2_b_rx) # Set the baselines on the sensor

        # Take measurements
        tVOC = float(self.sgp30.tvoc)
        eCO2 = float(self.sgp30.co2eq)
        
        tVOC_b = int(self.sgp30.baseline_tvoc)
        eCO2_b = int(self.sgp30.baseline_co2eq)


        ###############
        # SCD30
        ###############
        time.sleep_ms(200)
        scd_meas = self.scd30.read_measurement()
        CO2 = 9999
        T = 9999
        H = 9999

        if not math.isnan(scd_meas[0]):
            CO2 = scd_meas[0]
        if not math.isnan(scd_meas[1]):
            T = scd_meas[1]
        if not math.isnan(scd_meas[2]):
            H = scd_meas[2]


        ###############
        # SPS30
        ###############
        time.sleep_ms(200)

        PM1 = 9999
        PM2 = 9999
        PM4 = 9999
        PM10 = 9999

        if self.sps30.read_measured_values() != self.sps30.MEASURED_VALUES_ERROR:
            PM1 = self.sps30.measurement[0]
            PM2 = self.sps30.measurement[1]
            PM4 = self.sps30.measurement[2]
            PM10 = self.sps30.measurement[3]


        # Build influx line protocol string
        line = config.NODEID + ' '
        line += 'T={:.1f}'.format(T) + ','
        line += 'H={:.1f}'.format(H) + ','
        line += 'CO2={:.0f}'.format(CO2) + ','
        line += 'tVOC={:.1f}'.format(tVOC) + ','
        line += 'eCO2={:.0f}'.format(eCO2) + ','
        line += 'tVOC_b={:.0f}'.format(tVOC_b) + ','
        line += 'eCO2_b={:.0f}'.format(eCO2_b) + ','
        line += 'PM1={:.1f}'.format(PM1) + ','
        line += 'PM2={:.1f}'.format(PM2) + ','
        line += 'PM4={:.1f}'.format(PM4) + ','
        line += 'PM10={:.1f}'.format(PM10)
        
        # Get RSSI
        if self._cyclecount >= self.cycles-1:
            self.get_rssi()
            line += ',rssi={:.0f}'.format(self.rssi)
            self._cyclecount = 0
        else:
            self._cyclecount += 1


        ###### DATA TRANSMISSION ######
        # Initialize GSM
        self._log('Initializing GSM modem...')
        self.lte = LTE()
        self.lte.deinit(detach=True, reset=True) # Always reset modem
        time.sleep(3.0)
        self.lte = LTE()
        self.modem_attach()
        self.modem_connect()
        
        self._log('Sending MQTT messages...')

        # Publish data
        self.mqtt_connect() # Should reconnect each time, because MQTT keepalive does not work
        self.mqttclient.publish(topic=self.topic_data, msg=line)   # Default qos = 0
        self._log('InfluxDB line protocol:' + line) 
        time.sleep(30)  

        # Check for messages
        self._log('Check for incoming MQTT messages')
        self.mqttclient.check_msg()
        time.sleep(5.0)

        # MQTT disconnect
        self._log('Disconnect from MQTT server')
        self.mqttclient.disconnect()
        time.sleep(5.0)

        self._log('Disconnect from NB-IOT network')
        self.lte.disconnect()
        time.sleep(5.0)

        # Feed the watchdog
        self._wdt.feed()


    def cmd_handle(self,topic,msg):
        """
        Handle the command messages received through MQTT
        """
        self._log('Received MQTT message. Topic: '+str(topic)+',msg: '+str(msg))

        # REBOOT
        if ('reboot' in topic) and (int(msg)==1):
            self._log('Received reboot command from MQTT')
            self._log('Rebooting ...')
            time.sleep(1.0)
            machine.reset() # reboot module
        
        # SET SGP30 BASELINE - tVOC (topic: config/nodeid/baseline/tVOC_b)
        if ('tVOC_b' in topic):
            self.tVOC_b_rx = int(msg)
            self._log('SGP30 tVOC baseline value has been received: ' + str(self.tVOC_b_rx))
            self.baseline_manual = True     # If baseline are received, switch mode to manual (e.g, always set the manual baseline)

        # SET SGP30 BASELINE - eCO2 (topic: config/nodeid/baseline/eCO2_b)
        if ('eCO2_b' in topic):
            self.eCO2_b_rx = int(msg)
            self._log('SGP30 eCO2 baseline value has been received: ' + str(self.eCO2_b_rx))
            self.baseline_manual = True    



    def rssi_parse(self,rssi_string):
        """
        Parse the AT RSSI signal string
        According to AT COMMANDS REFERENCE MANUAL p.42
        """
        arr = rssi_string.split(',')
        rssi_raw = int(arr[0][-2:])
        x1 = 2.0
        x2 = 30.0
        y1 = -109.0
        y2 = -53.0
        rssi = y1 + ((rssi_raw - x1) / (x2 - x1)) * (y2 - y1)
        if rssi_raw >= 30:
            rssi = -51.0
        if rssi_raw <= 1:
            rssi = -111.0
        if rssi_raw <= 0:
            rssi = -113.0 

        return rssi


    def _log(self, message, *args):
        """
        Outputs a log message to terminal
        """
        if config.DEBUG_MODE:
            print('[{:>10.3f}] {}'.format(
                utime.ticks_ms() / 1000,
                str(message).format(*args)
                ))