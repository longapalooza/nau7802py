""" Provides an SMBus class for use on micropython """

try:
    from machine import I2C
except ImportError:
    raise ImportError("Can't find the micropython machine.I2C class: "
                      "perhaps you don't need this adapter?")


class SMBus(I2C):
    """ Provides an 'SMBus' module which supports some of the py-smbus
        i2c methods, as well as being a subclass of machine.I2C

        Hopefully this will allow you to run code that was targeted at
        py-smbus unmodified on micropython.

	    Use it like you would the machine.I2C class:

            import usmbus.SMBus

            bus = SMBus(1, pins=('G15','G10'), baudrate=100000)
            bus.read_byte_data(addr, register)
            ... etc
	"""

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

    # Trying to implement these libraries:
    def read_byte(self, addr):
        """ Read a single byte of data from the device at addr
            Returns a bytes object with the data read """
        return self.readfrom(addr, 1)

    def write_byte(self, addr, data):
        """ Write a single byte of data to the device at addr
            Returns None """
        if isinstance(data, int):
            data = bytes([data])
        return self.writeto(addr, data)

    def read_word_data(self, addr, register):
        """Read a 16-bit word from a given register of a device at addr
        Returns an integer with the data read
        """
        # Read two bytes from the device
        data = self.readfrom_mem(addr, register, 2)

        # Combine the two bytes to form a 16-bit word
        # You may need to adjust the line below depending on the endianness
        # of the device you are working with
        word_data = data[0] | (data[1] << 8)

        return word_data

    def write_word_data(self, addr, register, data):
        """Write a 16-bit word to a given register of a device at addr
        Returns None
        """
        # Break down the 16-bit word into two bytes
        # You may need to adjust the lines below depending on the endianness
        # of the device you are working with
        byte1 = data & 0xFF
        byte2 = (data >> 8) & 0xFF

        # Convert the two bytes to a bytes object
        data_bytes = bytes([byte1, byte2])

        # Write the two bytes to the device
        self.writeto_mem(addr, register, data_bytes)

        