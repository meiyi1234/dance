import unittest

from comm import SerialProtocol
from framing import HFrame


class TestComm(unittest.TestCase):
    def test_data_received(self):
        handshake = HFrame(1).bytes
        proto = SerialProtocol()
        for byte in handshake:
            proto.data_received(byte)


if __name__ == '__main__':
    unittest.main()