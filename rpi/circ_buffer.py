"""
Uncomment commented lines to make buffer thread-safe (untested)
"""
# import threading


class CircularBuffer:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.buf = bytearray(self.maxlen)
        # self.locks = [threading.RLock() for _ in range(maxlen)]  # Lock for each element in buffer
        self.read_idx = 0
        self.write_idx = 0

    def write_one(self, byte):
        # with self.locks[self.write_idx]:
        self.buf[self.write_idx] = byte
        self.write_idx = (self.write_idx + 1) % self.maxlen

    def write(self, bytes_):
        for byte in bytes_:
            self.write_one(byte)

    def read(self):
        # with self.locks[self.read_idx]:
        byte = self.buf[self.read_idx]
        self.read_idx = (self.read_idx + 1) % self.maxlen
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
        return bytearr
