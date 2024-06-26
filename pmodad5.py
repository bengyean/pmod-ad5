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
# pylint: disable=E1101
import RPi.GPIO as GPIO
from gpiozero import DigitalInputDevice, LED
import spidev
import numpy as np

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
# registerMap = [{0x00}, {0x080060}, {0x000117}, {0x000000}]
registerMap = np.array([0x00, 0x080060, 0x000117, 0x000000])

# registerSize = [{1}, {3}, {3}, {3}, {1}, {1}, {3}, {3}]
registerSize = np.array([1, 3, 3, 3, 1, 1, 3, 3])

AD7193_CS_PIN = 8         # define the chipselect CEO in GPIO8 or pin 24
led = LED(21)             # led on GPIO21
rdy_input = DigitalInputDevice(26)  # /RDY interrupt on GPIO26
HIGH = 1
LOW = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(AD7193_CS_PIN, GPIO.OUT)

# AD7193_COMM_WEN = 0
# AD7193_COMM_WRITE = 0
# AD7193_COMM_READ = 1
# AD7193_COMM_ADDR = 0
# AD7193_COMM_CREAD = 1

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
AD7193_COMM_WEN = 0 << 7           # Write Enable
AD7193_COMM_WRITE = 0 << 6         # Write Operation
AD7193_COMM_READ = 1 << 6          # Read Operation


def ad7193_comm_addr(x):
    """Return register address"""
    return (x & 0x7) << 3


def ad7193_comm_cread(x):
    """Continous Read of Data Register"""
    return (x & 1) << 2  # if x=1 Continous Read of Data Register


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
    time.sleep(0.5)

    x = 0
    tx = [255]
    for x in range(0, 5):
        spi.writebytes(tx)
        if x == 5:
            break

    GPIO.output(AD7193_CS_PIN, GPIO.HIGH)
    led.on()
    time.sleep(0.5)


def read_adc_channel(channel):
    """Read ADC Channel"""
    set_channel(channel)

    # write command to initial conversion
    initiate_single_conversion()
    time.sleep(0.1)   # hardcoded wait time for data to be ready
    # should scale the wait time by averaging

    wait_for_adc()

    adc_data = rd_adc_data()
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
    # bytenum = int(next(iter(registerSize[reg_address])))
    bytenum = registerSize[reg_address]
    # print('Byte number is: ', bytenum)
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xFC00FF   # Keep all bit values except Channel bits
    manip_data |= channel_bits

    set_reg_value(reg_address, manip_data, bytenum, 1)
    time.sleep(0.01)


def initiate_single_conversion():
    """Initiate Single Conversion"""
    print('   Initiate Single Conversion... ')
    print('(Device will go into low power mode when conversion complet)')

    manip_data = 0
    reg_address = AD7193_REG_MODE

    bytenum = registerSize[reg_address]
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0x1FFFFF   # Keep all bit values except Mode bits
    manip_data |= 0x200000   # single conversion mode bits

    set_reg_value(reg_address, manip_data, bytenum, 1)

    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xE00000   # Keep only the mode bits
    # print('Type of manip_data is: ', type(manip_data))
    # strHex = "0x%0.2" % manip_data
    print('current mode of single conversion: ', hex(manip_data))

    # Lire etat du bit RDY du registre status
    rdy_state = 1
    reg_address = AD7193_REG_STAT
    bytenum = registerSize[reg_address]
    rdy_state = rd_reg_value(reg_address, bytenum, 1)
    print('readyState: ', hex(rdy_state))


def calibrate():
    """Initiate Internal Calibration"""
    print("\nInitiate Internal Calibration, starting with Zero-scale calibration...")

    # Begin Communication cycle, bring CS low manually
    GPIO.output(AD7193_CS_PIN, LOW)
    time.sleep(0.1)

    manip_data = 0
    reg_address = AD7193_REG_MODE

    bytenum = registerSize[reg_address]
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0x1FFFFF    # keep all bit values except Mode bits
    manip_data |= 0x800000    # internal zero scale calibration

    set_reg_value(reg_address, manip_data, bytenum, 1)

    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xE00000    # keep only the mode bits
    print("current mode of zero calibration: ", hex(manip_data))

    wait_for_adc()
    time.sleep(0.1)		   # Beng, comment it when works

    print("\n\nNow full-scale calibration...")
    # --------------------------

    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0x1FFFFF    # keep all bit values except Mode bits
    manip_data |= 0xA00000    # system full scale calibration

    set_reg_value(reg_address, manip_data, bytenum, 1)

    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xE00000    # keep only the mode bits
    print("current mode of full calibration: ", hex(manip_data))

    wait_for_adc()
    time.sleep(0.1)                # Beng, comment when works

    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0x008000    # keep only the mode bits
    print("SYNC mode: ", hex(manip_data))

    GPIO.output(AD7193_CS_PIN, HIGH)
    time.sleep(0.1)


def set_filter_rate(filter_rate):
    """Setting Filter Rate Select Bits to"""
    print("\nSetting Filter Rate Select Bits to ", filter_rate)

    if filter_rate > 0x3ff:
        print("\tERROR - Invalid Filter Rate Setting - no changes made.  Filter Rate is a 10-bit value")
        return

    manip_data = 0
    reg_address = AD7193_REG_MODE

    bytenum = registerSize[reg_address]
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    # keep all bit values except output data rate setting bits
    manip_data &= 0xFFFC00
    manip_data |= filter_rate

    set_reg_value(reg_address, manip_data, bytenum, 1)


def foo_time():
    """Time passed"""
    t = int(round(time.time() * 1000))   # millis()
    return t


def wait_for_adc():
    """Wait ADC while is busy"""
    break_time = 0

    print('\nWainting for Conversion...')

    manip_data = 0
    # reg_address = AD7193_REG_MODE
    rdy_state = 1
    end_time = 0
    # This is a placeholder for correct code for this message.
    # --fail-on=I1101
    # init_time = lambda: int(round(time.time() * 1000))   # millis()
    init_time = foo_time()
    while True:
        reg_address = AD7193_REG_MODE

        bytenum = registerSize[reg_address]
        manip_data = rd_reg_value(reg_address, bytenum, 1)
        manip_data &= 0xE00000   # keep only the mode bits
        rdy_state = 1
        if manip_data == 0x800000:
            # if in internal zero scale calibration mode
            time.sleep(0.1)   # delay since we're still calibrating
        elif manip_data == 0xA00000:
            # if in internal full scale calibration mode
            time.sleep(0.1)   # delay since we're still calibrating
        reg_address = AD7193_REG_STAT
        bytenum = registerSize[reg_address]
        rdy_state = rd_reg_value(reg_address, bytenum, 1)
        rdy_state &= 0x80      # keep only the ready bit

        if rdy_state == 0x00:
            # Break if ready goes low
            print('breakTime: ')
        if break_time > 15000:
            # Break after five seconds - avoids program hanging up
            print("Data Ready never went low!")
            print("breakTime: ", break_time)
            # end_time = lambda: int(round(time.time() * 1000))
            end_time = foo_time()
            elapsed_time = end_time - init_time
            print("elapsedTime: ", elapsed_time)
            print("current mode is in REG_MODE ?: ",  hex(manip_data))
            # break
        break_time = break_time + 1


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
    # receive_buffer = 0

    write_byte = AD7193_COMM_READ | ad7193_comm_addr(reg_address)
    print('write_byte', write_byte)
    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.LOW)

    spi.writebytes([write_byte])   # Envoi le 1er byte de cammande
    # print('Ici')
    while byte_index < bytes_number:
        # Lecture de donnee en mode MSB FIRST
        receive_buffer = spi.xfer2([0], 8000000)
        buffer = (buffer << 8) + receive_buffer[0]
        byte_index += 1

    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.HIGH)

    print('    Write Register Address: ', reg_address)
    return buffer


def set_reg_value(reg_address, register_value, bytes_number, modify_cs):
    """Writes data into a register"""
    command_byte = 0
    tx_buffer = [0x00, 0x00, 0x00, 0x00]

    command_byte = AD7193_COMM_WEN | AD7193_COMM_WRITE | ad7193_comm_addr(reg_address) | ad7193_comm_cread(1)
    print('set reg value command byte is: ', hex(command_byte))
    print('register value is: ', register_value)

    tx_buffer[0] = (register_value >> 0) & 0x000000FF
    tx_buffer[1] = (register_value >> 8) & 0x000000FF
    tx_buffer[2] = (register_value >> 16) & 0x000000FF
    tx_buffer[3] = (register_value >> 24) & 0x000000FF
    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.LOW)
        time.sleep(0.1)

    spi.xfer2([command_byte])   # Envoi le 1er byte de cammande
    # print('bytes_number: ', bytes_number)
    # try:
        # for i in bytes_number:
            # puis envoi octet par octet de data en MSB first
            # spi.xfer2(hex(list(tx_buffer[bytes_number - 1])), 8000000)
            # bytes_number -= 1
            # print("      Loop i: ", i)
    # except TypeError:
        # print("Object must be an iterable")
    while bytes_number > 0:
        # puis envoi octet par octet de data en MSB first
        tx_list = [tx_buffer[bytes_number - 1]]
        spi.xfer2(tx_list, 8000000)
        print('tx_list: ', tx_list)
        bytes_number -= 1

    if modify_cs == 1:
        GPIO.output(AD7193_CS_PIN, GPIO.HIGH)
        time.sleep(0.1)

    print('    Write Register Address: ', reg_address)

    print(', command: ', hex(command_byte))
    # print(', command: ', " ".join(hex(n) for n in command_byte))
    print(',    sent: ', hex(register_value))
    # print(',      sent: ', " ".join(hex(n) for n in register_value))


# Start read ADC Data
# def unsigned long (ReadADCData()):
# def read_adc_data() -> np.uint32:
def rd_adc_data2():
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


def rd_adc_data():
    """Read ADC data"""
    buffer = 0
    reg_address = AD7193_REG_DATA

    bytenum = registerSize[reg_address]
    print("BytesNumber: ", bytenum)
    print("BufferRd: ", buffer)
    buffer = rd_reg_value(reg_address, bytenum, 1)
    return buffer


def app_state_value_to_data(app_value):
    """Append/Configure Pmod AD5 module. Set DAT_STA to 1"""
    print("Enabling DAT_STA Bit (appends status register to data register when reading)")

    manip_data = 0
    reg_address = AD7193_REG_MODE

    bytenum = registerSize[reg_address]
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    type(manip_data)
    manip_data |= 0x100000  # set DAT_STA bit
    type(manip_data)
    hex_data = hex(manip_data)
    int_data = int(hex_data, 16)
    print('manip_data: ', int_data)
    set_reg_value(reg_address, int_data, bytenum, 1)
    # registerSize[3] = 4  # change register size to 4, b/c status is append

    # lire les 24bit du MODE register
    # manip_data = rd_reg_value(reg_address, bytenum, 1)
    if app_value == 1:
        manip_data |= 0x100000  # set DAT_STA bit
        registerSize[3] = 4
        # increase data register size by one since STA_REG (one byte) \
        # is now tacked on at the end of the SPI message
    else:
        # maintain cleared DAT_STA bit
        registerSize[3] = 3  # keep DATA_REG size at the default 3 bytes

    set_reg_value(reg_address, int_data, bytenum, 1)


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

    manip_data = 0
    reg_address = AD7193_REG_CONF

    # keep all bit value except gain bitsf
    # print('Here')
    bytenum = registerSize[reg_address]
    print(bytenum)
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    # manip_data = rd_reg_value(reg_address, 4, 1)
    manip_data &= 0xFFFFF8
    manip_data |= pga_setting
    print('pga setting: ', pga_setting)

    set_reg_value(reg_address, manip_data, bytenum, 1)
    # [0x10] : CONFIG_REG is selected
    # [0x80] : chop is anabled, vref in REFIN1 REFSEL=0, as differential inputs
    # Pseudo Bit=0
    # [0x01] : AIN1(+) and AIN2(-) are selected STAT_REG=000 ?
    # [0x00] : burn=0 burnout currents are uanable, REFDET off, BUF=0 input
    # 50mV below AGND to 50mV above AVDD, U/B=0 bipolar operation, gain = 1
    # When BUF=0 the analog voltage on the analog input pins can be from 50mV
    # below AGND to 50mV above AVDD

    # wrdat_buf = [0x10, 0x80, 0x01, manip_data]
    # swap_wrdat_buf = swap_data(wrdat_buf)
    # respond = spi.xfer2(wrdat_buf, 8000000, 0)
    # print('Chop=1, VRef=Intern, Mode=diff, Set PGA Gain=%', respond)
    # time.sleep(0.5)


def set_polarity(polarity):
    """Set Polarity"""
    print("\nSetting polarity to ", polarity)

    # gain_setting = 0xFFFFF7
    gain_setting = 0
    gpolarity = polarity << 3
    mpolarity_gain = 0

    reg_address = AD7193_REG_CONF

    bytenum = registerSize[reg_address]
    pga_setting = rd_reg_value(reg_address, bytenum, 1)
    pga_setting &= 0x000007  # keep only the PGA setting bits

    # unipolar operation mode
    if polarity == 1:
        # gain_setting = 0x0
        gain_setting = pga_setting
        print("Polarity = ", polarity)

    # bipolar operation mode
    if polarity == 0:
        # gain_setting = 0x0;
        gain_setting = pga_setting
        print("Polarity = ", polarity)

    mpolarity_gain = gpolarity | gain_setting

    # registerMap[reg_address] = ReadRegisterValue(reg_address, \
    # registerSize[reg_address], 1);
    # keep all bit values except polarity bit and gain
    registerMap[reg_address] &= 0xFFFFF0
    registerMap[reg_address] |= mpolarity_gain

    set_reg_value(reg_address, registerMap[reg_address], bytenum, 1)
    time.sleep(0.01)


def set_chop(chop):
    """Setting chop to"""
    # print("Setting chop to", chop)

    # manip_data = 0
    # mchop = 0

    reg_address = AD7193_REG_CONF

    # manip_data = rd_reg_value(reg_address, registerSize[reg_address], 1)
    # keep all bits values except chop bit
    # manip_data &= 0x7FFFFF

    # Chop enabled
    if chop == 1:
        # mchop = manip_data | 0x800000  # set chop bit
        registerMap[reg_address] |= 0x800000   # set chop bit to 1
        print("Chop = ", chop)

    # Chop unabled
    if chop == 0:
        # mchop = manip_data  # clear chop bit
        registerMap[reg_address] &= 0x7FFFFF  # clear chop bit to 0
        print("Chop = ", chop)

    # set_reg_value(reg_address, mChop, registerSize[reg_address], 1)
    bytenum = registerSize[reg_address]
    set_reg_value(reg_address, registerMap[reg_address], bytenum, 1)
    time.sleep(0.01)


def set_averaging(averaging):
    """> to improve rms noise"""
    print("\nSetting Averaging Bits to ", averaging)

    if averaging > 0x3:
        print("\tERROR - Invalid Averaging Setting - no changes made. \
              Avergaging setting is a 2-bit value")
        return

    manip_data = 0
    reg_address = AD7193_REG_MODE

    bytenum = registerSize[reg_address]
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xFCFFFF  # keep all bit values except averaging setting bits
    manip_data |= averaging

    bytenum = registerSize[reg_address]
    set_reg_value(reg_address, manip_data, bytenum, 1)


def set_pseudodiff_inputs(ps_value):
    """Set Pseudo Differential input configuration..."""
    print("Changing differential input configuation...")

    manip_data = 0
    reg_address = AD7193_REG_CONF

    # Bipolar calculation-> ps_value = 0. Unipolar calculation -> ps_value = 1
    bytenum = registerSize[reg_address]
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xFBFFFF  # keep all bit values except psuedo bit

    if ps_value == 1:
        manip_data |= 0x040000  # set psuedo bit
        mpseudo = manip_data >> 18
        print("Pseudo mode is activate")
        print("Pseudo bit = ", mpseudo)
    else:
        # maintain cleared psuedo bit - Added by Beng
        mpseudo = manip_data >> 18
        print("Pseudo bit = ", mpseudo)

    # registerMap[reg_address] = manip_data
    # set_reg_value(reg_address, registerMap[reg_address], bytenum, 1)
    set_reg_value(reg_address, manip_data, bytenum, 1)


def data_to_voltage(raw_data) -> float:
    """Convert Data to Voltage"""
    voltage = float(0)
    pga_gain = 0
    m_vref = float(2.5)

    manip_data = 0
    reg_address = AD7193_REG_CONF
    # mpseudo = 0
    # bpseudo = 0

    bytenum = registerSize[reg_address]
    mcutoff = rd_reg_value(reg_address, bytenum, 1)
    mcutoff &= 0x800000  # keep only Chop bit

    # Bipolar calculation -> value = 0. Unipolar calculation -> value = 1
    manip_data = rd_reg_value(reg_address, bytenum, 1)
    manip_data &= 0xFBFFFF  # keep all bit values except psuedo bit

    # mpseudo = manip_data | 0x040000  # set Pseudo bit

    mpolarity = rd_reg_value(reg_address, bytenum, 1)
    mpolarity &= 0x000008  # keep only polarity bits
    # mpolarity = (mBuffer >> 3)

    pga_setting = rd_reg_value(reg_address, bytenum, 1)
    pga_setting &= 0x000007  # keep only the PGA setting bits

    print("PGASetting = ", pga_setting)

    pga_gain = 0

    # keep only the polarity bit
    # m_polarity = join_num(registerMap[2]) & 0x000008
    # print('Polarity', m_polarity)

    # keep only the PGA setting bits
    # pga_setting = join_num(registerMap[2]) & 0x000007
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

    print("Cutoff = ", mcutoff)
    print("PGA Gain = ", pga_gain)

    # Vref is defined as REFINx(+) - REFINx(-), so either default 2.5V or other
    # other is limited to -(AVdd -1.25V)/gain to +(AVdd -1.25V)/gain
    # REFIN+ inputs themselves limited between 1V and AVdd
    # REFIN- inputs limited to AGND to (AVdd -1V)

    # print('PGA Gain: ', pga_gain )
    # println(pga_gain)
    # set_pga_gain(1)

    if mpolarity == 8:  # unipolar
        mpseudo = manip_data | 0x040000  # set pseudo bit
        bpseudo = mpseudo >> 18
        bpseudo &= 0x000001  # keep only pseudo bit
        print("Pseudo = ", " ".join(hex(n) for n in bpseudo))
        print("\nunipolar calculation")
        # print("Gain was: ", PGAGain)
        print("raw data was: ", raw_data)
        # following equation is a rearrangement of the one listed on pg 33 of\
        # the AD7193 datasheet it was rearranged from:
        # Code = (2^24 * AnalogInputVoltage * PGAGain)/Vref
        # to: AnalogInputVoltage = [Code / (2^24 * PGAGain)] * Vref
        # voltage = ((double)raw_data / 16777216 / pga_gain) * m_vref
        voltage = (float(raw_data) / 16777216 / pga_gain) * m_vref
    if mpolarity == 0:  # bipolar
        mpseudo = manip_data & 0xFBFFFF  # clear pseudo bit
        bpseudo = mpseudo >> 18
        bpseudo &= 0x000001  # keep only pseudo bit
        print("Pseudo = ", " ".join(hex(n) for n in bpseudo))
        print("\nbipolar calculation")
        # print("Gain was: ", PGAGain)
        print("raw data was: ", raw_data)
        # following equation is a rearrangement of the one listed on pg 33
        # of the AD7193 datasheet it was rearranged from:
        # Code = 2^23 * [AnalogInputVoltage * PGAGain)/Vref + 1]
        # to: [(Code / 2^23) - 1] * (Vref / PGAGain)
        voltage = (float(raw_data) / float(8388608)) - float(1) * \
            (m_vref / float(pga_gain))

    return voltage
    # print('Voltage=', voltage)


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


###########################################
# Initialisation/Setting up AD7193/Pmod AD5
###########################################
# resp = spi.xfer2([0xFF, 0xFF, 0xFF, 0xFF, 0xFF], 8000000, 100)
reset()
app_state_value_to_data(1)
set_pga_gain(1)
print('Set up Finalizing...')
# set_averaging(3)
# set_filter_rate(96)
# set_pseudodiff_inputs(0)
# set_polarity(0)
# calibrate()

try:
    while True:
        # resp_int = rd_adc_data()
        # print('data:', resp_int)
        valeur = read_adc_channel(0)
        valeur = valeur >> 8  # Extraction of value
        print("Data value: ", valeur)
        tension = data_to_voltage(valeur)
        print("\n")
        print("Valeur= ", valeur)
        print('\t')  # tabulation
        print("Tension= ", tension, end=" ")
        print("V")

        time.sleep(0.5)

finally:
    spi.close()
    GPIO.cleanup()
# ############################## END OF CODE #################################
