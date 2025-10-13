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
        self.write(PHYCFGR, COMMON_REGISTER, 0x00) # Reset
        time.sleep_ms(200) # Idk, 200 seems good
        self.write(PHYCFGR, COMMON_REGISTER, 0xf8) # Reset: 1, OPMD: 1, OPMDC: All capable, Auto-negotiation enabled 
        
        # SW Reset
        self.write(MR, COMMON_REGISTER, 0x80) # RST bit
        time.sleep_ms(200) # Idk, 200 seems good
        
        
        # Enable interrupts on socket 0
        self.write(SIMR, COMMON_REGISTER, 0b1)
        
        
        
        
        

