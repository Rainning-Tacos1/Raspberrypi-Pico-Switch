# Reference
# https://github.com/badlyby/w5500_macraw/blob/main/w5500.py
# https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf

from micropython import const, schedule
import machine
import time
import asyncio

COMMON_REGISTER = const(0b00)
SN0_REGISTER = const(0b01)
SN0_TX_BUFFER = const(0b10)
SN0_RX_BUFFER = const(0b11)

# Common register
MR = const(0b00) # Mode Register
IR = const(0x15) # Interrupt Register
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
        self.cs_pin = machine.Pin(self.cs, machine.Pin.OUT)
        self.rst_pin = machine.Pin(self.rst, machine.Pin.OUT)
        self._int_pin = machine.Pin(self._int, machine.Pin.IN, machine.Pin.PULL_UP)
    
        self.isr_flag = asyncio.ThreadSafeFlag()
        
        
        self.spi = machine.SPI(
            id=self.port,
            sck = self.sck_pin,
            mosi = self.mosi_pin,
            miso = self.miso_pin,
        )
        
        self._int_pin = machine.Pin(
            self._int,
            machine.Pin.IN,
            machine.Pin.PULL_UP
        )
        
    async def isr_task(self):
        while True:
            # Wait for interrupt
            await self.isr_flag.wait()
            # Read which socket caused interrupts
            # Only socket 0 should be triggering interrupts bc of the SIMR
            
            #_IR = self.read8(IR, COMMON_REGISTER)
            _SIR = self.read8(SIR, COMMON_REGISTER)
            if _SIR[0] != 0x1: # Interrupt not on socket 0
                return

            SN0_IR = self.read8(SN_IR, SN0_REGISTER)[0]
            
            # Process the interurpt
            str_int = ""
            str_int += "SEND_OK " if SN0_IR & 0b10000 else ""
            str_int += "TIMEOUT " if SN0_IR & 0b01000 else ""
            str_int += "RECV "    if SN0_IR & 0b00100 else ""
            str_int += "DISCON "  if SN0_IR & 0b00010 else ""
            str_int += "CON "     if SN0_IR & 0b00001 else ""
            
            # Acknowledge the interrupt
            self.write8(SN_IR, SN0_REGISTER, SN0_IR)
            
            # Execute user interrupt
            await self.isr(str_int)
        
    def spi_init(self):
        self.spi.init(
            baudrate = self.baud,
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
        self.write(addr, bsb, data.to_bytes(2, 'big'))
    
    def write(self, addr, bsb, data):
        self.cs_pin.value(0)
        self.spi.write(addr.to_bytes(2, 'big'))
        self.spi.write(int((bsb << 3) | 0b100).to_bytes(1))
        self.spi.write(data)
        self.cs_pin.value(1)

    def read8(self, addr, bsb):
        return self.read(addr, bsb, 1)
        
    def read16(self, addr, bsb):
        return self.read(addr, bsb, 2)

    def read(self, addr, bsb, nbytes):
        self.cs_pin.value(0)
        self.spi.write(addr.to_bytes(2, 'big'))
        self.spi.write(int((bsb << 3)).to_bytes(1))
        data = self.spi.read(nbytes)
        self.cs_pin.value(1)
        return data
    
    def link_status(self):
        status = int.from_bytes(
            self.read8(PHYCFGR, COMMON_REGISTER)
        ) & 0b111
        
        link = (
            (10, "Half"),
            (100, "Half"),
            (10, "Full"),
            (100, "Full")
        )
        print(f"LNIK: {status & 0x1}")
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
        if self.read8(VERSIONR, COMMON_REGISTER)[0] != 0x04:
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

        while self.read8(SN_SR, SN0_REGISTER)[0] != 0x42:
            time.sleep_ms(200) # Idk, 200 seems good

        # Attach Interrupt
        self._int_pin.irq(
            lambda pin: self.isr_flag.set(), #self.isr_wrapper,
            machine.Pin.IRQ_FALLING
        )