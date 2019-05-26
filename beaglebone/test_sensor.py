import time
import smbus
import decimal
bus = smbus.SMBus(1)
addr = 0x5c
data = bus.read_i2c_block_data(addr,0x13)
lux2 = str((data[1] + (256 * data[0])) / 1.2)
print(lux2)
time.sleep(0.3)
data = bus.read_i2c_block_data(0x23,0x13)
lux2 = str((data[1] + (256 * data[0])) / 1.2)
print(lux2)      
lux = decimal.Decimal(lux2).quantize(decimal.Decimal('.01'), rounding=decimal.ROUND_UP)
outlux = str(lux)
print outlux