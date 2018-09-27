import threading


class CircularBuffer:
    def __init__(self, maxlen):
        """Self-expanding circular buffer.
        maxlen must be a power of 2. This enables use of bitmask instead of
        expensive modulo operations.
        """
        if not (maxlen != 0 and maxlen & (maxlen - 1) == 0):
            raise ValueError('maxlen must be a power of 2')

        self.maxlen = maxlen
        self.mask = maxlen - 1
        self.buf = bytearray(self.maxlen)
        self.read_idx = 0
        self.write_idx = 0
        self.load = 0
        self.LOAD_FACTOR = 0.8 * self.maxlen  # Fill up to 80% before expanding

    def write_one(self, byte):
        self.buf[self.write_idx] = byte
        self.write_idx = (self.write_idx + 1) & self.mask
        self.load += 1

    def write(self, bytes_):
        if self._will_be_full(len(bytes_)):
            self._expand()
        for byte in bytes_:
            self.write_one(byte)

    def read(self):
        byte = self.buf[self.read_idx]
        self.read_idx = (self.read_idx + 1) & self.mask
        self.load -= 1
        return byte

    def read_until(self, stop_byte, ignore_first_byte):
        """Returns all bytes up to and including stop_byte or end of buffer, whichever
        occurs first.
        If ignore_first_byte is True, the function does not return if the very first byte
        read is the stop byte.
        """
        bytearr = []
        cached_read_idx = self.read_idx
        for i in range(self.read_idx, self.write_idx):
            byte = self.read()
            bytearr.append(byte)
            if byte == stop_byte:
                if ignore_first_byte and i == cached_read_idx:
                    continue
                else:
                    break
        return bytes(bytearr)

    def _will_be_full(self, size):
        """Returns true if buf will be full after size bytes added to buffer"""
        return self.load + size >= self.LOAD_FACTOR

    def _expand(self):
        """Double buffer size."""
        print('Expanding circular buffer from {} to {} bytes'.format(self.maxlen, 2*self.maxlen))
        self.maxlen = 2 * self.maxlen
        self.mask = self.maxlen - 1
        self.LOAD_FACTOR = 0.8 * self.maxlen

        new_buf = bytearray(self.maxlen)
        with threading.RLock():  # Don't allow reads/writes while expanding
            for i in range(len(self.buf)):
                new_buf[i] = self.buf[i]  # Copy data
            self.buf = new_buf
