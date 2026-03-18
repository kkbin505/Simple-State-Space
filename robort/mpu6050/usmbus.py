""" Provides an SMBus class for use on micropython """

try:
    from machine import I2C, Pin
except ImportError:
    raise ImportError("Can't find the micropython machine.I2C class: "
                      "perhaps you don't need this adapter?")


class SMBus(I2C):
    def __init__(self, busNum, scl, sda, freq=400000):
        """
        Initializes the SMBus (I2C) interface.

        :param busNum: 0 or 1
        :param scl: Pin for SCL
        :param sda: Pin for SDA
        :param freq: I2C clock frequency (default: 400kHz)
        """
        super().__init__(id=busNum, scl=Pin(scl), sda=Pin(sda), freq=freq)


    def read_byte_data(self, addr, register):
        """ Read a single byte from register of device at addr
            Returns a single byte """
        return self.readfrom_mem(addr, register, 1)[0]

    def read_i2c_block_data(self, addr, register, length):
        """ Read a block of length from register of device at addr
            Returns a bytes object filled with whatever was read """
        return self.readfrom_mem(addr, register, length)

    def write_byte_data(self, addr, register, data):
        """ Write a single byte from buffer `data` to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(data, int):
            data = bytes([data])
        return self.writeto_mem(addr, register, data)

    def write_i2c_block_data(self, addr, register, data):
        """ Write multiple bytes of data to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(data, int):
            data = bytes([data])
        return self.writeto_mem(addr, register, data)

    # The follwing haven't been implemented, but could be.
    def read_byte(self, *args, **kwargs):
        """ Not yet implemented """
        raise RuntimeError("Not yet implemented")

    def write_byte(self, *args, **kwargs):
        """ Not yet implemented """
        raise RuntimeError("Not yet implemented")

    def read_word_data(self, *args, **kwargs):
        """ Not yet implemented """
        raise RuntimeError("Not yet implemented")

    def write_word_data(self, *args, **kwargs):
        """ Not yet implemented """
        raise RuntimeError("Not yet implemented")



