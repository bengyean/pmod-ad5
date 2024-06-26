"""Digital interface with Analog Devices 24-bits ADC Sigma Delta"""

# ############################# START OF CODE ################################

# Modified : 14/06/24
# Author   : by Beng KY
# For      : CNRS/APC

# For my application, I intend to read ADC values up at speeds up to 1kHz over
# 2 channels and hence have chosen this Pmod AD5. For testing wise, I have
# connected a potential divider across A1 and A2 on the AD5 and am able to
# read the voltage changes when using in raspberry pi 4.
#
# Here's my python code using the spidev on raspberry pi 4
# (have connected to CE0):

import time
# import sys
# import machine
import RPi.GPIO as GPIO
from gpiozero import DigitalInputDevice, LED
import spidev
# import numpy as np

###############################################################################
# SPI Settings with spidev library

spi = spidev.SpiDev()       # create spi object connecting to

spi.open(0, 0)              # to bus0, cs0
spi.bits_per_word = 8
spi.lsbfirst = False        # MSB first (or big endian) for AD7193
spi.max_speed_hz = 8000000  # speed 5000=5kHz
# spi.mode = 0b00             # spi in mode 0
spi.mode = 0b11             # spi in mode 3

###############################################################################

# For AD7193 device Registers Map
registerMap = [{0x00}, {0x080060}, {0x000117}, {0x000000}]

registerSize = [{1}, {3}, {3}, {3}, {1}, {1}, {3}, {3}]

AD7193_CS_PIN = 8         # define the chipselect CEO in GPIO8 or pin 24
led = LED(21)             # led on GPIO21
rdy_input = DigitalInputDevice(26)  # /RDY interrupt on GPIO26

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(AD7193_CS_PIN, GPIO.OUT)

AD7193_COMM_WEN = 0
AD7193_COMM_WRITE = 0
AD7193_COMM_READ = 1
AD7193_COMM_ADDR = 0
AD7193_COMM_CREAD = 1

AD7193_REG_COMM = 0       # WO, 8-bit
AD7193_REG_STAT = 0       # WO, 8-bit
AD7193_REG_MODE = 1       # WR, 24-bit
AD7193_REG_CONF = 2       # WR, 24-bit
AD7193_REG_DATA = 3       # RO, 24/32-bit
AD7193_REG_ID = 4         # RO, 8-bit
AD7193_REG_GPOCON = 5     # WW, 8-bit
AD7193_REG_OFFSET = 6     # WR, 24-bit
AD7193_REG_FULLSCALE = 7  # WR, 24-bit

# Communication Register Bit Designation (AD7193_REG_COM)
AD7193_COMM_WEN = AD7193_COMM_WEN << 7            # Write Enable
AD7193_COMM_WRITE = AD7193_COMM_WRITE << 6        # Write Operation
AD7193_COMM_READ = AD7193_COMM_READ << 6          # Read Operation
AD7193_COMM_ADDR = (int(AD7193_COMM_ADDR) & 0x7) << 3  # Register Address
AD7193_COMM_CREAD = AD7193_COMM_CREAD << 2  # Continous Read of Data Register

send_buf = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF]


def reverse_bits(byte):
    """Cette fonction inversera l'ordre des bits en un octet si nécessaire."""
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4)
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2)
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1)
    return byte


def reset():
    """Reset the ADC"""
    print('Reset AD7193...')

    GPIO.output(AD7193_CS_PIN, GPIO.LOW)
    led.off()
    time.sleep(0.1)

    n = 0
    while n < 6:
        spi.xfer2([0xFF], 8000000)
        n += 1

    GPIO.output(AD7193_CS_PIN, GPIO.HIGH)
    led.on()
    time.sleep(0.1)


def read_adc_channel(channel):
    """Read ADC Channel"""
    set_channel(channel)

    # write command to initial conversion
    initiate_single_conversion()
    time.sleep(0.1)   # hardcoded wait time for data to be ready
    # should scale the wait time by averaging

    wait_for_adc()

    adc_data = read_adc_data2()
    time.sleep(0.01)

    return adc_data


def set_channel(channel):
    """generate Channel settings bits for Configuration write"""
    shiftvalue = 0x000100
    channel_bits = shiftvalue << channel
    print('channelBits is: ', channel_bits)
    manip_data = 0
    reg_address = AD7193_REG_CONF

    # Write Channel bits to CONF_REG, keeping other bits as is
    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0xFC00FF   # Keep all bit values except Channel bits
    manip_data |= channel_bits

    set_reg_value(reg_address, manip_data,  registerSize[reg_address], 1)
    time.sleep(0.01)


def initiate_single_conversion():
    """Initiate Single Conversion"""
    print('   Initiate Single Conversion... \
          (Device will go into low power mode \
        when conversion complet)')

    manip_data = 0
    reg_address = AD7193_REG_MODE

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0x1FFFFF   # Keep all bit values except Mode bits
    manip_data |= 0x200000   # single conversion mode bits

    set_reg_value(reg_address, manip_data, registerSize[reg_address], 1)

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0xE00000   # Keep only the mode bits
    print('current mode: ', " ".join(hex(n) for n in manip_data))

    # Lire etat du bit RDY du registre status
    rdy_state = 1
    reg_address = AD7193_REG_STAT
    rdy_state = rd_reg_value(reg_address, registerSize[reg_address], 1)
    print('readyState: ', " ".join(hex(n) for n in rdy_state))


def calibrate():
    """Initiate Internal Calibration"""
    print("\nInitiate Internal Calibration, \
          starting with Zero-scale calibration...")

    # Begin Communication cycle, bring CS low manually
    GPIO.output(AD7193_CS_PIN, LOW)
    time.sleep(0.1)

    manip_data = 0
    reg_address = AD7193_REG_MODE

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0x1FFFFF    # keep all bit values except Mode bits
    manip_data |= 0x800000    # internal zero scale calibration

    set_reg_value(reg_address, manip_data, registerSize[reg_address], 1)

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0xE00000    # keep only the mode bits
    print("current mode: ", " ".join(hex(n) for n in manip_data))

    wait_for_adc()
    time.sleep(0.1)		   # Beng, comment it when works

    print("\n\nNow full-scale calibration...")
    # --------------------------

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0x1FFFFF    # keep all bit values except Mode bits
    manip_data |= 0xA00000    # system full scale calibration

    set_reg_value(reg_address, manip_data, registerSize[reg_address], 1)

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0xE00000    # keep only the mode bits
    print("current mode: ", " ".join(hex(n) for n in manip_data))

    wait_for_adc()
    time.sleep(0.1)                # Beng, comment when works

    manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    manip_data &= 0x008000    # keep only the mode bits
    print("SYNC mode: ", " ".join(hex(n) for n in manip_data))

    GPIO.output(AD7193_CS_PIN, HIGH)
    time.sleep(0.1)


def wait_for_adc():
    """Wait ADC while is busy"""
    break_time = 0

    print('\nWainting for Conversion...')

    manip_data = 0
    # reg_address = AD7193_REG_MODE
    rdy_state = 1
    end_time = 0
    init_time = lambda: int(round(time.time() * 1000))   # millis()
    while True:
        reg_address = AD7193_REG_MODE
        manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
        manip_data &= 0xE00000   # keep only the mode bits
        rdy_state = 1
    if manip_data == 0x800000:
        # if in internal zero scale calibration mode
        time.sleep(0.1)   # delay since we're still calibrating
    elif manip_data == 0xA00000:
        # if in internal full scale calibration mode
        time.sleep(0.1)   # delay since we're still calibrating
        reg_address = AD7193_REG_STAT
        rdy_state = rd_reg_value(reg_address, registerSize[reg_address], 1)
        rdy_state &= 0x80      # keep only the ready bit

        if rdy_state == 0x00:
            # Break if ready goes low
            print('breakTime: ')
        if break_time > 15000:
            # Break after five seconds - avoids program hanging up
            print("Data Ready never went low!")
            print("breakTime: ", break_time)
            end_time = lambda: int(round(time.time() * 1000))
            elapsed_time = end_time - init_time
            print("elapsedTime: ", elapsed_time)
            print("current mode: ",  " ".join(hex(n) for n in manip_data))
            # break
        break_time = break_time + 1


#    def magic(num_list):   # [1, 2, 3]
#        """Converting list of ints to one number"""
#        s = map(str, num_list)   # ['1','2','3']
#        s = ''.join(s)          # '123'
#        s = int(s)              # 123
#        return s

# How I'd probably write it:
def magic(num_list):
    """Converting list of ints to one number by removing the first element"""
    num_list = num_list[1:]
    # print('magic_num: ', num_list)
    s = ''.join(map(str, num_list))
    print('join_num: ', s)
    return int(s)


def join_num(num_list):
    """Converting list of ints to one number"""
    # print('join_num: ', num_list)
    s = ''.join(map(str, num_list))
    return int(s)


def swap_data(data):
    """Cette fonction swap une liste"""
    swap_bytes = bytearray(data)
    swap_bytes.reverse()
    return swap_bytes
    # print(swap_bytes)


def rd_reg_value(reg_address, bytes_number, modify_cs):
    """Read but not print register value to maintain compatibility"""
    write_byte = 0
    byte_index = 0
    buffer = 0

    write_byte = AD7193_COMM_READ | AD7193_COMM_ADDR
    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.LOW)

    spi.xfer2(write_byte)
    while byte_index < bytes_number:
        receive_buffer = spi.xfer2(0)
        buffer = (buffer << 8) + receive_buffer
        byte_index += 1

    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.HIGH)

    print('    Write Register Address: ', reg_address)
    return buffer


def set_reg_value(reg_address, register_value, bytes_number, modify_cs):
    """Writes data into a register"""
    command_byte = 0
    tx_buffer = [0x00, 0x00, 0x00, 0x00]

    command_byte = AD7193_COMM_WEN | AD7193_COMM_WRITE | \
        AD7193_COMM_ADDR | AD7193_COMM_CREAD

    tx_buffer[0] = (register_value >> 0) & 0x000000FF
    tx_buffer[1] = (register_value >> 8) & 0x000000FF
    tx_buffer[2] = (register_value >> 16) & 0x000000FF
    tx_buffer[3] = (register_value >> 24) & 0x000000FF
    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.LOW)
        time.sleep(0.1)

    spi.xfer2(command_byte)
    while bytes_number > 0:
        spi.xfer2(tx_buffer[bytes_number - 1], 8000000)
        bytes_number -= 1

    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.HIGH)
        time.sleep(0.1)

    print('    Write Register Address: ', reg_address)
    # print(', command: ', command_byte)
    print(', command: ', " ".join(hex(n) for n in command_byte))
    # print(',      sent: ', register_value)
    print(',      sent: ', " ".join(hex(n) for n in register_value))


# Start read ADC Data
# def unsigned long (ReadADCData()):
# def read_adc_data() -> np.uint32:
def read_adc_data():
    """Read Data from ADC"""
    byte_index = 0
    # ibuffer = 0
    receive_buffer = 0
    int_buffer = 0
    # data_lenght = registerSize[3]
    data_lenght = 3
    # tx_buffer = [0x00, 0x00, 0x00, 0x00]

    # spi.writebytes([0x58])      # command to start read data
    # time.sleep(0.1)

    while byte_index < data_lenght:
        try:
            # [0x58] : Demande une Lecture dans DATA_REG depuis le COM_REG
            # Users write to the COM_REG indicate that they want to read
            # the data and then clock in the 24 bits of data from the data
            #  register and finally pull the chip select line back to a high
            # voltage state. It is in the single conversion to read by step.
            wr_buf = [0x58, 0x00, 0x00, 0x00]
            # To enable continuous read, Instruction 01011100 must be written
            # to the communications register. Then uncomment the below ligne
            wr_buf = [0x5C, 0x00, 0x00, 0x00]
            # swap_wr_buf = swap_data(wr_buf)
            receive_buffer = spi.xfer2(wr_buf, 8000000, 1)
            receive_buffer = receive_buffer[1:]
            int_buffer = join_num(receive_buffer)
            print('int=', int_buffer)
            return int_buffer
        except KeyboardInterrupt:
            spi.close()


def read_adc_data2():
    """Read ADC data"""
    buffer = 0
    reg_address = AD7193_REG_DATA

    buffer = rd_reg_value(reg_address, registerSize[reg_address], 1)
    return buffer


def set_pga_gain(gain):
    """Set Gain"""
    manip_data = 0x00

    print('\nSetting PGA Gain to ', gain)

    if gain == 1 | 0x0:
        pga_setting = 0
    elif gain == 8 | 0x3:
        pga_setting = 0x3
    elif gain == 16 | 0x4:
        pga_setting = 0x4
    elif gain == 32 | 0x5:
        pga_setting = 0x5
    elif gain == 64 | 0x6:
        pga_setting = 0x6
    elif gain == 128 | 0x7:
        pga_setting = 0x7
    else:
        print('\tERROR - Invalid Gain Setting - no changes mode. Valid Gain \
              setting are 1, 8, 16, 32, 64, 128')
        return

    # keep all bit value except gain bits
    manip_data &= 0xF8
    manip_data |= pga_setting
    print('pga setting: ', pga_setting)

    # [0x10] : CONFIG_REG is selected
    # [0x80] : chop is anabled, vref in REFIN1 REFSEL=0, as differential inputs
    # Pseudo Bit=0
    # [0x01] : AIN1(+) and AIN2(-) are selected STAT_REG=000 ?
    # [0x00] : burn=0 burnout currents are uanable, REFDET off, BUF=0 input
    # 50mV below AGND to 50mV above AVDD, U/B=0 bipolar operation, gain = 1
    # When BUF=0 the analog voltage on the analog input pins can be from 50mV
    # below AGND to 50mV above AVDD
    wrdat_buf = [0x10, 0x80, 0x01, manip_data]
    # swap_wrdat_buf = swap_data(wrdat_buf)
    respond = spi.xfer2(wrdat_buf, 8000000, 0)
    print('Chop=1, VRef=Intern, Mode=diff, Set PGA Gain=%', respond)
    time.sleep(0.5)


def data_to_voltage(raw_data) -> float:
    """Convert Data to Voltage"""
    voltage = float(0)
    pga_gain = 0
    m_vref = float(2.5)

    # [0x10] : CONFIG_REG is selected
    # [0x00] : chop is disabled, as differential inputs
    # [0x00] : AIN1(+) and AIN2(-) are selected
    # [0xC0] : burn=1 burnout currents are anable, REFDET on, BUF=0 input 50mV
    # below AGND to 50mV above AVDD, bipolar operation, gain = 1
    # When BUF=0 the analog voltage on the analog input pins can be from 50mV
    # below AGND to 50mV above AVDD
    # send_buf = [0x10, 0x00, 0x00, 0xC0]
    # swap_send_buf = swap_data(send_buf)
    # resp = spi.xfer2(send_buf)
    # print('Set PGA Gain = 1, Buffer = 1', resp)
    # time.sleep(0.5)

    # keep only the polarity bit
    m_polarity = join_num(registerMap[2]) & 0x000008
    print('Polarity', m_polarity)

    # keep only the PGA setting bits
    pga_setting = join_num(registerMap[2]) & 0x000007
    # print('pga setting: ', pga_setting)

    if pga_setting == 0:
        pga_gain = 1
    elif pga_setting == 3:
        pga_gain = 8
    elif pga_setting == 4:
        pga_gain = 16
    elif pga_setting == 5:
        pga_gain = 32
    elif pga_setting == 6:
        pga_gain = 64
    elif pga_setting == 7:
        pga_gain = 128
    else:
        pga_gain = 1

    # print('PGA Gain: ', pga_gain )
    # println(pga_gain)
    set_pga_gain(1)

    if m_polarity == 1:
        # voltage = ((double)raw_data / 16777216 / pga_gain) * m_vref
        voltage = (float(raw_data) / 16777216 / pga_gain) * m_vref
    if m_polarity == 0:
        voltage = (float(raw_data) / float(8388608)) - float(1) * \
            (m_vref / float(pga_gain))
    # return voltage
    print('Voltage=', voltage)


# Fonction: Étalonnage de l'échelle zéro du système. L'utilisateur doit
# connecter l'entrée de l'échelle zéro du système aux broches d'entrée
# du canal sélectionné par les bits CH7 à CH0 dans le registre de
# configuration. RDY passe à l'état# haut lorsque le calibrage est lancé
# et repasse à l'état bas lorsque le calibrage est terminé. L'ADC est placé
# en mode inactif après un calibrage. Le coefficient de décalage mesuré est
# placé dans le registre de décalage du canal sélectionné. Il est recommandé
# d'effectuer un étalonnage à l'échelle zéro du système chaque fois que
# le gain d'un canal est modifié.
#
# [0x08] : Demande une Ecriture dans le MODE_REG depuis le COM_REG
# [0x98] : Zero-scale calib, Continous convertion mode, DAT_STA=1,
# Internal 4.92MHz clock is used, no avarasing
# [0x18] : SIN4 is used, ENPAR=0, CLK_DIV=1 (AVDD is less then 4.75V),
# Single=1, REJ60=0,
# [0x60] : FS=100
#
# def zero_calibration():
    # send_buf = [0x08, 0x98, 0x00, 0x64]
    # # swap_send_buf = swap_data(send_buf)
    # resp = spi.xfer2(swap_send_buf)
    # print('Initiate internal calibration, starting w zero-scale', resp)
    # time.sleep(0.5)


# Étalonnage de la pleine échelle du système. L'utilisateur doit connecter
# l'entrée pleine échelle du système aux broches d'entrée du canal sélectionné
# par les bits CH7 à CH0 dans le registre de configuration. RDY passe à l'état
# haut lorsque le calibrage est lancé et repasse à l'état bas lorsque le
# calibrage est terminé. L'ADC est placé en mode inactif après un calibrage.
# Le coefficient de pleine échelle mesuré est placé dans le registre de pleine
# échelle du canal sélectionné. Il est recommandé d'effectuer un étalonnage à
# pleine échelle chaque fois que le gain d'un canal est modifié.
#
# [0xB8] : Full-scale calib, Continous convertion mode, DAT_STA=1,
# Internal 4.92MHz clock is used, no avarasing
#
# def full_calibration():
    # send_buf = [0x08, 0xB8, 0x00, 0x64]
    # # swap_send_buf = swap_data(send_buf)
    # resp = spi.xfer2(swap_send_buf)
    # print('Full-scale calibration...', resp)
    # time.sleep(0.5)


########################################
# Setting up AD7193
########################################
# resp = spi.xfer2([0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
resp = spi.xfer2([0xFF, 0xFF, 0xFF, 0xFF, 0xFF], 8000000, 100)
# reset()
# resp_list = resp[1:]
# temp = ''.join(map(str, resp_list))
print('resp: ', resp)
# print('Resetting...', resp_list)
time.sleep(0.1)

# [0x08] : Demande une Ecriture dans le MODE_REG depuis le COM_REG
# [0x3C] : Single convertion mode, DAT_STA=1, Internal 4.92MHz clock is
# used, clock is available on pin MCLK2, no avarasing
# [0x3C] : SIN4 is used, ENPAR=1, CLK_DIV=2 (AVDD is less then 4.75V),
# Single=1 for zero latency, REJ60=1 allows simultanous 50Hz/60Hz rejection,
# [0x04] : FS=4
# send_buf = [0x08, 0x3C, 0x3C, 0x04]
send_buf = [AD7193_REG_MODE, 0x3C, 0x3C, 0x04]
# swap_send_buf = swap_data(send_buf)
resp1 = spi.xfer2(send_buf, 8000000, 1)
print('Single conversion, Internale clock, REJ60=1, FS=4', resp1)
time.sleep(0.5)

# [0x10] : CONFIG_REG is selected
# [0x80] : chop is anabled, vref in REFIN1, as differential inputs
# [0x01] : Pseudo Bit=0, AIN1(+) and AIN2(-) are selected STAT_REG=000 ?
# [0x00] : burn=0 burnout currents are uanable, REFDET off, BUF=0 input 50mV
# below AGND to 50mV above AVDD, bipolar operation, gain = 1
# When BUF=0 the analog voltage on the analog input pins can be from 50mV
# below AGND to 50mV above AVDD
# send_buf = [0x10, 0x00, 0x00, 0xC0]
# send_buf = [0x10, 0x80, 0x01, 0x00]
# swap_send_buf = swap_data(send_buf)
# resp = spi.xfer2(send_buf, 8000000, 1, 8)
# print('Set PGA Gain = 1, Buffer = 1', resp)
# time.sleep(0.5)
set_pga_gain(1)

# [0x08] : Demande une Ecriture dans le MODE_REG depuis le COM_REG
# [0x1C] : Continous convertion mode, DAT_STA=1, Internal 4.92MHz clock is
# used, no avarasing
# [0x3C] : SIN4 is used, ENPAR=1, CLK_DIV=2 (AVDD is less then 4.75V),
# Single=1, REJ60=1,
# [0x60] : FS=100
# send_buf = [0x08, 0x1C, 0x3C, 0x60]
# swap_send_buf = swap_data(send_buf)
# resp = spi.xfer2(send_buf, 8000000, 1)
# print('Setting filter rate select bits to 100', resp)
# time.sleep(0.5)


# keep only the polarity bit
# m_polarity = join_num(registerMap[2]) & 0x000008
# print('Polarity', m_polarity)

# keep only the PGA setting bits
# pga_setting = join_num(registerMap[2]) & 0x000007
# print('pga setting: ', pga_setting)

while True:
    # choose channel
    # [0x10] : CONFIG_REG is selected
    # [0x00] : chop is disabled, as differential inputs
    # [0x01] : AIN1 and AIN2 are selected
    # [0x00] : burn=0, no REFDET, BUF=1, bipolar operation, gain = 1
    # send_buf = [0x10, 0x00, 0x00, 0xC0]
    # send_buf = [0x10, 0x00, 0x01, 0x10]
    # swap_send_buf = swap_data(send_buf)
    # resp = spi.xfer2(send_buf)
    # time.sleep(0.1)
    reset()

    # [0x58] : Demande une Lecture dans DATA_REG depuis le COM_REG
    # pylint: disable=C0103
    resp_int = read_adc_data()
    # print('data:', resp_int)
    data_to_voltage(resp_int)
    # """Ecrire pour effectuer la lecture"""
    # resp = spi.xfer2([0x48, 0x00, 0x00, 0x00])
    # time.sleep(0.1)
    # print(registerMap[2])

    # print('data:', resp)
    time.sleep(0.8)

# ############################## END OF CODE #################################
