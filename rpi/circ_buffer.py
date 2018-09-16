import threading


class CircularBuffer:
    def __init__(self, maxlen):
        self.buf = bytearray(maxlen)
        self.locks = [threading.RLock() for _ in range(maxlen)]  # Lock for each element in buffer
        self.read_idx = 0
        self.write_idx = 0

    def write(byte):
        with self.locks[write_idx]:
            self.buf[self.write_idx] = byte
        self.write_idx = (self.write_idx + 1) % maxlen

    def read():
        with self.locks[read_idx]:
            byte = self.buf[self.read_idx]
        self.read_idx = (self.read_idx + 1) % maxlen
        return byte
