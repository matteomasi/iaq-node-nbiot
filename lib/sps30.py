import struct
import time
import ubinascii

def calculateCRC(input):
    crc = 0xFF
    for i in range (0, 2):
        crc = crc ^ input[i]
        for j in range(8, 0, -1):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc = crc << 1
    crc = crc & 0x0000FF
    return crc

def checkCRC(result):
    for i in range(2, len(result), 3):
        data = []
        data.append(result[i-2])
        data.append(result[i-1])

        crc = result[i]

        if crc == calculateCRC(data):
            crc_result = True
        else:
            crc_result = False
    return crc_result


class SPS30():
    SPS_ADDR = 0x69

    START_MEAS   = [0x00, 0x10]
    STOP_MEAS    = [0x01, 0x04]
    R_DATA_RDY   = [0x02, 0x02]
    R_VALUES     = [0x03, 0x00]
    RW_AUTO_CLN  = [0x80, 0x04]
    START_CLN    = [0x56, 0x07]
    R_ARTICLE_CD = [0xD0, 0x25]
    R_SERIAL_NUM = [0xD0, 0x33]
    RESET        = [0xD3, 0x04]

    NO_ERROR = 1
    ARTICLE_CODE_ERROR = -1
    SERIAL_NUMBER_ERROR = -2
    AUTO_CLN_INTERVAL_ERROR = -3
    DATA_READY_FLAG_ERROR = -4
    MEASURED_VALUES_ERROR = -5

    dict_values = {"pm1p0"  : None,
                   "pm2p5"  : None,
                   "pm4p0"  : None,
                   "pm10p0" : None,
                   "nc0p5"  : None,
                   "nc1p0"  : None,
                   "nc2p5"  : None,
                   "nc4p0"  : None,
                   "nc10p0" : None,
                   "typical": None}
    
    measurement = [None]*10  #Initialize measurement array [PM1, PM2.5, PM4, PM10, etc.]

    def __init__(self, i2c, addr=SPS_ADDR):
        self._i2c = i2c
        self._addr = addr


    def read_auto_cleaning_interval(self):
        result = []

        self._i2c.writeto(self._addr, bytes(self.RW_AUTO_CLN))
        time.sleep_ms(30) # Wait for command execution time
        reply_size = 6
        crc_result = bytearray(reply_size) # Initialize byte array, 6 bytes
        self._i2c.readfrom_into(self._addr, crc_result)

        for i in range(reply_size):
            result.append(crc_result[i])

        if checkCRC(result):
            result = result[0] * pow(2, 24) + result[1] * pow(2, 16) + result[3] * pow(2, 8) + result[4]
            return result
        else:
            return self.AUTO_CLN_INTERVAL_ERROR

    def start_measurement(self):
        self.START_MEAS.append(0x03)
        self.START_MEAS.append(0x00)

        crc = calculateCRC(self.START_MEAS[2:4])
        self.START_MEAS.append(crc)

        self._i2c.writeto(self._addr, bytes(self.START_MEAS))
        time.sleep_ms(30)
        #write = i2c_msg.write(self.SPS_ADDR, self.START_MEAS)
        #self.bus.i2c_rdwr(write)

    def stop_measurement(self):
        
        self._i2c.writeto(self._addr, bytes(self.STOP_MEAS))
        time.sleep_ms(30)
        
        #write = i2c_msg.write(self.SPS_ADDR, self.STOP_MEAS)
        #self.bus.i2c_rdwr(write)

    def read_data_ready_flag(self):
        result = []

        self._i2c.writeto(self._addr, bytes(self.R_DATA_RDY))
        reply_size = 3
        crc_result = bytearray(reply_size)
        self._i2c.readfrom_into(self._addr, crc_result)

        #write = i2c_msg.write(self.SPS_ADDR, self.R_DATA_RDY)
        #self.bus.i2c_rdwr(write)
        #read = i2c_msg.read(self.SPS_ADDR, 3)
        #self.bus.i2c_rdwr(read)

        for i in range(reply_size):
            result.append(crc_result[i])

        if checkCRC(result):
            return result[1]
        else:
            return self.DATA_READY_FLAG_ERROR

    def read_measured_values(self):
        result = []

        self._i2c.writeto(self._addr, bytes(self.R_VALUES))
        reply_size = 60
        crc_result = bytearray(reply_size)
        self._i2c.readfrom_into(self._addr, crc_result)


        #write = i2c_msg.write(self.SPS_ADDR, self.R_VALUES)
        #self.bus.i2c_rdwr(write)
        #read = i2c_msg.read(self.SPS_ADDR, 60)
        #self.bus.i2c_rdwr(read)

        for i in range(reply_size):
            result.append(crc_result[i])

        if checkCRC(result):
            self.parse_sensor_values(result)
            return self.NO_ERROR
        else:
            return self.MEASURED_VALUES_ERROR

    def parse_sensor_values(self, input):
        #pm_list = []
        k = 0
        for i in range (0, len(input), 6):
            value = struct.unpack('>f',bytes(input[i:i+4]))[0]
            self.measurement[k] = value
            k += 1
            #pm_list.append(value)
        
        # Debugging
        #print( struct.unpack('>f',bytes(input[0:4]))[0] )
        #print( struct.unpack('>f',bytes(input[6:10]))[0] )
        
        #for d in self.dict_values.keys():
        #    self.dict_values[d] = pm_list[i]
        #   i += 1