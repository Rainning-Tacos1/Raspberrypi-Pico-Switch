# Reference
# https://github.com/badlyby/w5500_macraw/blob/main/w5500.py
# https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/W5500_datasheet_v1.0.2_1.pdf

import spidev
import RPi.GPIO as GPIO
import time

COMMON_REGISTER = 0b00
SN0_REGISTER = 0b01
SN0_TX_BUFFER = 0b10
SN0_RX_BUFFER = 0b11

# Common register
MR = 0b00 # Mode Register
IR = 0x15 # Interrupt Register
SIR = 0x17 # Socket Interrupt Register
SIMR = 0x18 # Socket Interrupt Mask Register
PHYCFGR = 0x2e # PHYsical ConFiGuRatuion
VERSIONR = 0x39 # VERSION Register

# Socket Registers
SN_MR = 0x0 # Socket n Mode Register
SN_CR = 0x1 # Socket n Control Register
SN_IR = 0x2 # Socket n Interrupt Register
SN_SR = 0x3 # Socket n Status Register

SN_MSSR = 0x12 # Socket n Maximum Segment Size Register

SN_RXBUF_SIZE = 0x1e # Socket n Receive BUFfer SIZE
SN_TXBUF_SIZE = 0x1f # Socket n Transmit BUFfer SIZE

SN_RX_RSR = 0x26 # Socket n Receive Received Size Register
SN_IMR = 0x2c # Socket n Interrupt Mask Register

class NotW5500(Exception):
	pass


class W5500():
	def __init__(self, port, baud, cs_pin, rst_pin, int_pin, isr):
		self.port = port
		self.baud = baud
		self.isr = isr
		
		# Pins
		self.cs_pin = cs_pin
		self.rst_pin = rst_pin
		self.int_pin = int_pin

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.cs_pin, GPIO.OUT)
		GPIO.setup(self.rst_pin, GPIO.OUT)
		GPIO.setup(self.int_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
			
		self.spi = spidev.SpiDev()
		self.spi.open(self.port, 0)
		self.spi.max_speed_hz = self.baud
		self.spi.mode = 0b00 # SPI Mode 0 (CPOL=0, CPHA=0)
		
		GPIO.output(self.cs_pin, GPIO.HIGH)
		GPIO.output(self.rst_pin, GPIO.HIGH)

	def _isr(self):
		print("INT")
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
		str_int += "RECV "	if SN0_IR & 0b00100 else ""
		str_int += "DISCON "  if SN0_IR & 0b00010 else ""
		str_int += "CON "	 if SN0_IR & 0b00001 else ""
			
		# Acknowledge the interrupt
		self.write8(SN_IR, SN0_REGISTER, SN0_IR)
			
		# Execute user interrupt
		self.isr(str_int)
  
	def reset(self):
		GPIO.output(self.rst_pin, GPIO.LOW)
		time.sleep(0.01)
		GPIO.output(self.rst_pin, GPIO.HIGH)
		time.sleep(0.01)
	
	def write8(self, addr: int, bsb: int, data: int):
		print(f"{data:02X}")
		self.write(addr, bsb, data)
	
	def write16(self, addr: int, bsb: int, data: int):
		self.write(addr, bsb, data)
	
	def write(self, addr: int, bsb: int, data: int) -> None:
		GPIO.output(self.cs_pin, GPIO.LOW)
		self.spi.writebytes(list(
			addr.to_bytes(2, 'big')
		))
		self.spi.writebytes(list(
			int((bsb << 3) | 0b100).to_bytes(1)
		))
		self.spi.writebytes(data)
		GPIO.output(self.cs_pin, GPIO.LOW)

	def read8(self, addr: int, bsb: int):
		return self.read(addr, bsb, 1)
		
	def read16(self, addr: int, bsb: int):
		return self.read(addr, bsb, 2)

	def read(self, addr: int, bsb:int, nbytes: int):
		GPIO.output(self.cs_pin, GPIO.LOW)
		self.spi.writebytes(list(
			addr.to_bytes(2, 'big')
		))
		self.spi.writebytes(list(
			int((bsb << 3)).to_bytes(1)
		))		
		data = self.spi.readbytes(nbytes)
		GPIO.output(self.cs_pin, GPIO.HIGH)
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
		print(f"LINK: {status & 0x1}")
		return (link[status >> 0x1]) if (status & 0x1) else None

	def init(self):
		self.reset()
		
		# PHY Reset
		self.write8(PHYCFGR, COMMON_REGISTER, 0x00) # Reset
		time.sleep(0.2) # Idk, 200ms seems good
		self.write8(PHYCFGR, COMMON_REGISTER, 0xf8) # Reset: 1, OPMD: 1, OPMDC: All capable, Auto-negotiation enabled 
		
		# SW Reset
		self.write8(MR, COMMON_REGISTER, 0x80) # RST bit
		time.sleep(0.2) # Idk, 200ms seems good
		
		# Check Version
		if self.read8(VERSIONR, COMMON_REGISTER)[0] != 0x04:
			pass
			#raise NotW5500
		
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

		#while self.read8(SN_SR, SN0_REGISTER)[0] != 0x42:
		#	time.sleep(0.2) # Idk, 200ms seems good

		# Attach Interrupt
		GPIO.add_event_detect(self.int_pin, GPIO.FALLING, callback=self._isr, bouncetime=None)