from smbus2 import SMBus
import time

I2C_ADDR = 0x26
PWM_CONTROL_REG = 0x07  # 2 bytes per motor (big-endian int16), range -1000 to 1000

def set_motors(bus, m1, m2, m3, m4):
    data = [
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m4 >> 8) & 0xFF, m4 & 0xFF,
    ]
    bus.write_i2c_block_data(I2C_ADDR, PWM_CONTROL_REG, data)

def stop(bus):
    set_motors(bus, 0, 0, 0, 0)

speed = 300  # range: -1000 to 1000

with SMBus(1) as bus:

    print("Forward")
    set_motors(bus, speed, 0, speed, 0)
    time.sleep(1)

    print("Backward")
    set_motors(bus, -speed, 0, -speed, 0)
    time.sleep(1)

    print("Rotate Right")
    set_motors(bus, -speed, 0, speed, 0)
    time.sleep(1)

    print("Rotate Left")
    set_motors(bus, speed, 0, -speed, 0)
    time.sleep(1)

    stop(bus)

print("Test complete")