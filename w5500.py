# Reference
# https://github.com/badlyby/w5500_macraw/blob/main/w5500.py
# https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf

from micropython import const
import machine
import time

COMMON_REGISTER = const(0b00)
SN0_REGISTER = const(0b01)
SN0_TX_BUFFER = const(0b10)
SN0_RX_BUFFER = const(0b11)

# Common register
MR = const(0b00) # Mode Register
SIR = const(0x17) # Socket Interrupt Register
SIMR = const(0x18) # Socket Interrupt Mask Register
PHYCFGR = const(0x2e) # PHYsical ConFiGuRatuion
VERSIONR = const(0x39) # VERSION Register

# Socket Registers
SN_MR = const(0x0) # Socket n Mode Register
SN_CR = const(0x1) # Socket n Control Register
SN_IR = const(0x2) # Socket n Interrupt Register
SN_SR = const(0x3) # Socket n Status Register

SN_MSSR = const(0x12) # Socket n Maximum Segment Size Register

SN_RXBUF_SIZE = const(0x1e) # Socket n Receive BUFfer SIZE
SN_TXBUF_SIZE = const(0x1f) # Socket n Transmit BUFfer SIZE

SN_RX_RSR = const(0x26) # Socket n Receive Received Size Register
SN_IMR = const(0x2c) # Socket n Interrupt Mask Register

class NotW5500(Exception):
    pass


class W5500():
    def __init__(self, port, baud, sck, mosi, miso, cs, rst, _int, isr):
        self.port = port
        self.baud = baud
        self.isr = isr
        
        # Pins
        self.sck = sck
        self.mosi = mosi
        self.miso = miso
        self.cs = cs
        self.rst = rst
        self._int = _int
        
        self.sck_pin = machine.Pin(self.sck)
        self.mosi_pin = machine.Pin(self.mosi)
        self.miso_pin = machine.Pin(self.miso)
        self.cs_pin = machine.Pin(self.cs)
        self.rst_pin = machine.Pin(self.rst)
        
        self._int_pin = machine.Pin(
            self._int,
            machine.Pin.IN,
            machine.Pin.PULL_UP
        )
        
        self.spi = machine.SPI(1)
        
    def spi_init(self):
        self.spi.init(
            baudrate = self.baud,
            sck = self.sck_pin,
            mosi = self.mosi_pin,
            miso = self.miso_pin,
            firstbit = machine.SPI.MSB,
            bits = 8,
            polarity = 0,
            phase = 0
        )
        self.cs_pin.value(1)
  
    def spi_deinit(self):
        self.spi.deinit()
    
    def reset(self):
        self.rst_pin.value(0)
        time.sleep_ms(10)
        self.rst_pin.value(1)
        time.sleep_ms(10)
    
    def write8(self, addr, bsb, data):
        self.write(addr, bsb, data.to_bytes(1))
    
    def write16(self, addr, bsb, data):
        self.write(addr, bsb, data.to_bytes(2, byteorder='big'))
    
    def write(self, addr, bsb, data):
        self.cs_pin.value(0)
        self.spi.write(addr.to_bytes(2, byteorder='big'))
        self.spi.write(int(bsb & -0b100 | 0b100).tobytes(1))
        self.spi.write(data)
        self.cs_pin.value(1)

    def read(self, addr, bsb, nbytes):
        self.cs_pin.value(0)
        self.spi.write(addr.to_bytes(2, byteorder='big'))
        self.spi.write(int(bsb & -0b1000).tobytes(1))
        data = self.spi.read(nbytes)
        self.cs_pin.value(1)
        return data
    
    def link_status(self):
        status = int.from_bytes(
            self.read(PHYCFGR, COMMON_REGISTER, 1)
        ) & 0b111
        
        link = [
            (10, "Half"),
            (100, "Half"),
            (10, "Full"),
            (100, "Full")
        ]
        
        return (link[status >> 0x1]) if (status & 0x1) else None
        
        
    
    def init(self):
        self.spi_init()
        self.reset()
        
        # PHY Reset
        self.write8(PHYCFGR, COMMON_REGISTER, 0x00) # Reset
        time.sleep_ms(200) # Idk, 200 seems good
        self.write8(PHYCFGR, COMMON_REGISTER, 0xf8) # Reset: 1, OPMD: 1, OPMDC: All capable, Auto-negotiation enabled 
        
        # SW Reset
        self.write8(MR, COMMON_REGISTER, 0x80) # RST bit
        time.sleep_ms(200) # Idk, 200 seems good
        
        # Check Version
        if self.read(VERSIONR, COMMON_REGISTER, 1)[0] != 0x04:
            raise NotW5500
        
        self.write8(SN_MR, SN0_REGISTER, 0b100) # MAC RAW
        
        self.write8(SN_RXBUF_SIZE, SN0_REGISTER, 16) # 16Kb
        self.write8(SN_TXBUF_SIZE, SN0_REGISTER, 16) # 16Kb
        
        self.write16(SN_MSSR, SN0_REGISTER, 1514) # Ethernet MTU
        
        self.write8(SN_IMR, SN0_REGISTER, 0b0001_1111) # Enable interrupts SEND_OK, TIMEOUT, RECV, DISCON, CON 
        
        # Enable interrupts on socket 0
        self.write8(SIMR, COMMON_REGISTER, 0b1)
        
        # Open socket
        self.write8(SN_CR, SN0_REGISTER, 0x1) # OPEN
        # Could check SN_SR for 0x42 / SN_IR
        # OPEN interrupt is on, no need to manually check
