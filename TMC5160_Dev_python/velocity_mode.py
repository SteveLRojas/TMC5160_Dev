import sys
import serial
import time

from queue import Queue
import matplotlib.pyplot as plt
import numpy as np

baud = 115200
port = "COM7"
debug = 0
ser = serial.Serial()

rtmi_num_samples = 2048
rtmi_responses = []
for i in range(8):
    rtmi_responses.append(Queue(maxsize = 1000000))


def tmc_write_reg(address, value):
    address = address | 0x8000
    bytes = address.to_bytes(2, 'big') + value.to_bytes(4, 'big')
    ser.write(bytes)

def tmc_read_reg(address):
    bytes = address.to_bytes(2, 'big')
    ser.write(bytes)
    response = ser.read(5)
    return int.from_bytes(response[1:], 'big')

def tmc_process_rtmi_responses():
    while ser.in_waiting >= 5:
        response = ser.read(5)
        channel_id = int.from_bytes(response[0:1], 'big')
        value = int.from_bytes(response[1:5], 'big')
        if(debug):
            print(f"channel_id: 0x{channel_id:02X}")
            print(f"value: 0x{value:08X}")
        if(channel_id > 7):
            print(f"Bad channel ID: 0x{channel_id:02X}, Value: 0x{value:08X}")
        else:
            rtmi_responses[channel_id].put(value)

def tmc_configure_rtmi(trig_mode, trig_chan, num_chans, continuous_sampling, trigger, num_samples, threshold):
    address = 0xFFF5
    bytes = address.to_bytes(2, 'big') + threshold.to_bytes(4, 'big')
    ser.write(bytes)
    address = 0xFFF6
    bytes = address.to_bytes(2, 'big') + num_samples.to_bytes(4, 'big')
    ser.write(bytes)
    address = 0xFFF7
    value = trigger & 0x01
    value = (value << 1) | (continuous_sampling & 0x01)
    value = value << 2
    value = (value << 4) | (num_chans & 0x0F)
    value = (value << 4) | (trig_chan & 0x0F)
    value = (value << 4) | (trig_mode & 0x0F)
    bytes = address.to_bytes(2, 'big') + value.to_bytes(4, 'big')
    ser.write(bytes)

def main():
    if(debug):
        print("debug enabled")

    ser.baudrate = baud
    ser.port = port
    ser.dsrdtr = False
    ser.dtr = False
    ser.timeout = 1
    ser.open()
    time.sleep(1.0)
    ser.reset_input_buffer()

    #do the thing here
    tmc_write_reg(0x00, 0b000000000000001001)   #GCONF
    tmc_write_reg(0x0A, 0b0010010000000000000000)   #DRV_CONF FILT_ISENSE = 0, DRVSTRENGTH = 2, OTSELECT = 1, BBMCLKS = 0, BBMTIME = 0
    tmc_write_reg(0x0B, 0x80)   #GLOBALSCALER
    tmc_write_reg(0x10, 0b01100000100000001010) #IHOLD_IRUN IHOLD = 6, IRUN = 8, IHOLDDELAY = 10
    tmc_write_reg(0x11, 0x0A)   #TPOWERDOWN
    tmc_write_reg(0x6C, 0b00110000100000001000001110110011) #CHOPCONF

    tmc_write_reg(0x20, 0x01)   #RAMPMODE velocity mode to VMAX
    tmc_write_reg(0x26, 1000)   #AMAX
    tmc_write_reg(0x7FF8, 0x1F) #Pin Clear DRV_EN and all LEDs
    tmc_write_reg(0x27, 50000)   #VMAX
    time.sleep(5.0)
    tmc_write_reg(0x27, 0)  #VMAX
    time.sleep(5.0)
    tmc_write_reg(0x7FF9, 0x1F) #Pin Set DRV_EN and all LEDs
    
    print("Done!")
    print(f"Captured {rtmi_responses[0].qsize()} samples.")
    ser.close()

    if(debug):
        while not rtmi_responses[0].empty():
            print(f"RTMI CH0 response: 0x{rtmi_responses[0].get():08X}")
        while not rtmi_responses[1].empty():
            print(f"RTMI CH1 response: 0x{rtmi_responses[1].get():08X}")
        while not rtmi_responses[2].empty():
            print(f"RTMI CH2 response: 0x{rtmi_responses[2].get():08X}")
        while not rtmi_responses[3].empty():
            print(f"RTMI CH3 response: 0x{rtmi_responses[3].get():08X}")
        while not rtmi_responses[7].empty():
            print(f"RTMI CH7 response: 0x{rtmi_responses[7].get():08X}")
        print(f"serial bytes available: 0x{ser.in_waiting:08X}")


if __name__ == "__main__":
    main()
