import spidev
import time

# LS7366R Commands
CLR_CNTR = 0x20  # Clear counter
RD_CNTR = 0x60   # Read counter
WR_MDR0 = 0x88   # Write to MDR0
WR_MDR1 = 0x90   # Write to MDR1

# MDR0 config: 4X quadrature, free-running count, no index
MDR0_CONF = 0b00000011
# MDR1 config: 4-byte counter, enable counting
MDR1_CONF = 0b00000000

# SPI setup
spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0 (check if you're using CE0 or CE1)
spi.max_speed_hz = 1000000
spi.mode = 0b00

def setup_ls7366r():
    spi.xfer2([WR_MDR0, MDR0_CONF])
    spi.xfer2([WR_MDR1, MDR1_CONF])
    spi.xfer2([CLR_CNTR])  # Clear counter

def read_counter():
    resp = spi.xfer2([RD_CNTR, 0x00, 0x00, 0x00, 0x00])
    count = (resp[1] << 24) | (resp[2] << 16) | (resp[3] << 8) | resp[4]
    return count

# Initialize
setup_ls7366r()

# Loop to read counts
try:
    while True:
        count = read_counter()
        print(f"Count: {count}")
        time.sleep(0.5)

except KeyboardInterrupt:
    spi.close()
    print("SPI closed.")

