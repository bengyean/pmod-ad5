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
import spidev
# import numpy as np

###############################################################################
# SPI Settings with spidev library

spi = spidev.SpiDev()       # create spi object connecting to

spi.open(0, 0)              # to bus0, cs0
spi.bits_per_word = 8
spi.lsbfirst = False        # MSB first (or big endian) for AD7193
spi.max_speed_hz = 5000     # speed 5kHz
# spi.mode = 0b00             # spi in mode 0
spi.mode = 0b11             # spi in mode 3

###############################################################################

# For AD7193 device Registers Map
registerMap = [{0x00}, {0x080060}, {0x000117}, {0x000000}]

registerSize = [{1}, {3}, {3}, {3}, {1}, {1}, {3}, {3}]

send_buf = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF]


def reverse_bits(byte):
    """Cette fonction inversera l'ordre des bits en un octet si nécessaire."""
    byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4)
    byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2)
    byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1)
    return byte


# Function ReadADCChannel
def read_adc_channel():
    """pass"""


def set_channel():
    """pass"""


def initiate_single_conversion():
    """pass"""


def wait_for_adc():
    """pass"""


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


# Start read ADC Data
def read_adc_data():
    """Read Data from ADC"""
    byte_index = 0
    # ibuffer = 0
    receive_buffer = 0
    int_buffer = 0
    # data_lenght = registerSize[3]
    data_lenght = 3

    spi.writebytes([0x58])      # command to start read data
    time.sleep(0.1)

    while byte_index < data_lenght:
        try:
            # [0x58] : Demande une Lecture dans DATA_REG depuis le COM_REG
            # Users write to the COM_REG indicate that they want to read
            # the data and then clock in the 24 bits of data from the data
            #  register and finally pull the chip select line back to a high
            # voltage state. It is in the single conversion to read by step.
            wr_buf = [0x00, 0x00, 0x00]
            # To enable continuous read, Instruction 01011100 must be written
            # to the communications register. Then uncomment the below ligne
            # send_buf = [0x5C, 0x00, 0x00, 0x00]
            # swap_send_buf = swap_data(send_buf)
            receive_buffer = spi.xfer2(wr_buf)
            receive_buffer = receive_buffer[1:]
            int_buffer = join_num(receive_buffer)
            print('int=', int_buffer)
            return int_buffer
        except KeyboardInterrupt:
            spi.close()


def data_to_voltage(raw_data) -> float:
    """Convert Data to Voltage"""
    voltage = float(0)
    pga_gain = 0
    m_vref = float(2.5)

    # keep only the polarity bit
    m_polarity = join_num(registerMap[2]) & 0x000008
    # m_polarity = list(map(int, registerMap[2])) & 0x000008
    print('Polarity', m_polarity)

    # keep only the PGA setting bits
    pga_setting = join_num(registerMap[2]) & 0x000007
    # pga_setting = list(map(int, registerMap[2])) & 0x000007
    print('pga setting: ', pga_setting)

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

    print('PGA Gain: ', pga_gain)
    # println(pga_gain)

    if m_polarity == 1:
        # voltage = ((double)raw_data / 16777216 / pga_gain) * m_vref
        voltage = (float(raw_data) / 16777216 / pga_gain) * m_vref
    if m_polarity == 0:
        voltage = (float(raw_data) / float(8388608)) - float(1) * \
            (m_vref / float(pga_gain))
    # return voltage
    print('Voltage=', voltage)


# Fonction: Étalonnage de l'échelle zéro du système. L'utilisateur doit connecter
# l'entrée de l'échelle zéro du système aux broches d'entrée du canal sélectionné
# par les bits CH7 à CH0 dans le registre de configuration. RDY passe à l'état
# haut lorsque le calibrage est lancé et repasse à l'état bas lorsque le calibrage
# est terminé. L'ADC est placé en mode inactif après un calibrage.
# Le coefficient de décalage mesuré est placé dans le registre de décalage du canal
# sélectionné. Il est recommandé d'effectuer un étalonnage à l'échelle zéro du
# système chaque fois que le gain d'un canal est modifié.
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
# haut lorsque le calibrage est lancé et repasse à l'état bas lorsque le calibrage
# est terminé. L'ADC est placé en mode inactif après un calibrage.
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
resp = spi.xfer2([0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
print('Resetting...', resp)
time.sleep(0.5)

# [0x08] : Demande une Ecriture dans le MODE_REG depuis le COM_REG
# [0xC3] : Single convertion mode, DAT_STA=1, Internal 4.92MHz clock is
# used, clock is available on pin MCLK2, no avarasing
# [0xC3] : SIN4 is used, ENPAR=1, CLK_DIV=2 (AVDD is less then 4.75V),
# Single=1 for zero latency, REJ60=1 allows simultanous 50Hz/60Hz rejection,
# [0x60] : FS=96
send_buf = [0x08, 0xC3, 0xC3, 0x60]
# [0x08] : Continous conversion mode, DAT_STA=1, Internal 4.92MHz clock is
# used, clock is available on pin MCLK2
# send_buf = [0x08, 0x18, 0x00, 0x60]
# swap_send_buf = swap_data(send_buf)
resp = spi.xfer2(send_buf)
print('Enable DAT_STA Bit', resp)
time.sleep(0.5)

# [0x10] : CONFIG_REG is selected
# [0x00] : chop is disabled, as differential inputs
# [0x00] : AIN1(+) and AIN2(-) are selected
# [0x00] : burn=1 burnout currents are anable, REFDET on, BUF=0 input 50mV
# below AGND to 50mV above AVDD, bipolar operation, gain = 1
# When BUF=0 the analog voltage on the analog input pins can be from 50mV
# below AGND to 50mV above AVDD
send_buf = [0x10, 0x00, 0x00, 0xC0]
# swap_send_buf = swap_data(send_buf)
resp = spi.xfer2(send_buf)
print('Set PGA Gain = 1, Buffer = 1', resp)
time.sleep(0.5)

# [0x08] : Demande une Ecriture dans le MODE_REG depuis le COM_REG
# [0x18] : Continous convertion mode, DAT_STA=1, Internal 4.92MHz clock is
# used, no avarasing
# [0x18] : SIN4 is used, ENPAR=0, C[0x08, 0x18, 0x00, 0x60]LK_DIV=1 (AVDD is less then 4.75V),
# Single=1, REJ60=0,
# [0x60] : FS=100
send_buf = [0x08, 0x18, 0x00, 0x64]
# send_buf = [0x08, 0x18, 0x00, 0x64]
# swap_send_buf = swap_data(send_buf)
resp = spi.xfer2(send_buf)
print('Setting filter rate select bits to 100', resp)
time.sleep(0.5)

# Étalonnage de l'échelle zéro du système. L'utilisateur doit connecter
# l'entrée de l'échelle zéro du système aux broches d'entrée du canal sélectionné
# par les bits CH7 à CH0 dans le registre de configuration. RDY passe à l'état
# haut lorsque le calibrage est lancé et repasse à l'état bas lorsque le calibrage
# est terminé. L'ADC est placé en mode inactif après un calibrage.
# Le coefficient de décalage mesuré est placé dans le registre de décalage du canal
# sélectionné. Il est recommandé d'effectuer un étalonnage à l'échelle zéro du
# système chaque fois que le gain d'un canal est modifié.
#
# [0x08] : Demande une Ecriture dans le MODE_REG depuis le COM_REG
# [0x98] : Zero-scale calib, Continous convertion mode, DAT_STA=1,
# Internal 4.92MHz clock is used, no avarasing
# [0x18] : SIN4 is used, ENPAR=0, CLK_DIV=1 (AVDD is less then 4.75V),
# Single=1, REJ60=0,
# [0x60] : FS=100
# send_buf = [0x08, 0x98, 0x00, 0x64]
# # swap_send_buf = swap_data(send_buf)
# resp = spi.xfer2(swap_send_buf)
# print('Initiate internal calibration, starting w zero-scale', resp)
# time.sleep(0.5)

# Étalonnage de la pleine échelle du système. L'utilisateur doit connecter
# l'entrée pleine échelle du système aux broches d'entrée du canal sélectionné
# par les bits CH7 à CH0 dans le registre de configuration. RDY passe à l'état
# haut lorsque le calibrage est lancé et repasse à l'état bas lorsque le calibrage
# est terminé. L'ADC est placé en mode inactif après un calibrage.
# Le coefficient de pleine échelle mesuré est placé dans le registre de pleine
# échelle du canal sélectionné. Il est recommandé d'effectuer un étalonnage à
# pleine échelle chaque fois que le gain d'un canal est modifié.
#
# [0xB8] : Full-scale calib, Continous convertion mode, DAT_STA=1,
# Internal 4.92MHz clock is used, no avarasing
# send_buf = [0x08, 0xB8, 0x00, 0x64]
# # swap_send_buf = swap_data(send_buf)
# resp = spi.xfer2(swap_send_buf)
# print('Full-scale calibration...', resp)
# time.sleep(0.5)

while True:
    # choose channel
    # [0x10] : CONFIG_REG is selected
    # [0x00] : chop is disabled, as differential inputs
    # [0x01] : AIN1 and AIN2 are selected
    # [0x00] : burn=0, no REFDET, BUF=1, bipolar operation, gain = 1
    send_buf = [0x10, 0x00, 0x01, 0x10]
    # swap_send_buf = swap_data(send_buf)
    resp = spi.xfer2(send_buf)
    time.sleep(0.1)

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
