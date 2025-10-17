from w5500 import W5500, NotW5500
import queue
import time

msg = queue.Queue()

def isrr(data: str):
	print(data)

w = W5500(
    port = 0,
    baud = 20_000_000,
    cs_pin = 22,
    rst_pin = 25,
    int_pin = 27,
    isr = isrr
)

try:
	w.init()
except NotW5500:
	print("MEh")

print(w.link_status())

print("Inited")
while True:
	time.sleep(1)