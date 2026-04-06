import smbus2
bus = smbus2.SMBus(1)  # Or 0
try:
    bus.write_quick(0x26)  # Quick write to check if device responds
    print("Device found at 0x26")
except OSError as e:
    print(f"I2C error: {e}")