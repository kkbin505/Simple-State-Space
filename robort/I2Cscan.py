from machine import Pin, I2C

# Initialize I2C on I2C1 using custom pins
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

# Scan for devices
devices = i2c.scan()
print("I2C devices found:", devices)