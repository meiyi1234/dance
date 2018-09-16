import unittest

from bitstring import BitStream, BitArray

from comm import Frame, IFrame, SFrame, HFrame, get_frame_sort


class TestFrame(unittest.TestCase):
    def setUp(self):
        self.ifr = IFrame(b'\x12\x7D\x4B')
        self.sfr = SFrame(SFrame.Type.RR)
        self.hfr = HFrame()

    def test_escape(self):
        in1 = b'\x12\x3A\x4B'          # No special chars
        in2 = b'\x12\x7D\x4B'          # 1 escape char
        in3 = b'\x12\x7E\x4B'          # 1 start/stop char
        in4 = b'\x12\x7D\x7D\x4B'      # 2 consec escape chars 
        in5 = b'\x7E\x12\x3A\x4B\x7E'  # start and end are escape chars

        out1 = Frame.escape(in1)
        assert(out1 == in1)
        assert(Frame.unescape(out1) == in1)

        out2 = Frame.escape(in2)
        assert(out2 == b'\x12\x7D\x7D\x4B')
        assert(Frame.unescape(out2) == in2)

        out3 = Frame.escape(in3)
        assert(out3 == b'\x12\x7D\x7E\x4B')
        assert(Frame.unescape(out3) == in3)

        out4 = Frame.escape(in4)
        assert(out4 == b'\x12\x7D\x7D\x7D\x7D\x4B')
        assert(Frame.unescape(out4) == in4)

        out5 = Frame.escape(in5)
        assert(out5 == b'\x7E\x12\x3A\x4B\x7E')  # don't escape start and end chars
        assert(Frame.unescape(out5) == in5)

    def test_checksum_valid(self):
        assert(self.ifr.is_checksum_valid())
        assert(self.sfr.is_checksum_valid())
        assert(self.hfr.is_checksum_valid())

    def test_get_frame_sort(self):
        assert(get_frame_sort(BitStream(self.ifr.bitarr[8:24])) == Frame.Sort.I)
        assert(get_frame_sort(BitStream(self.sfr.bitarr[8:24])) == Frame.Sort.S)
        assert(get_frame_sort(BitStream(self.hfr.bitarr[8:24])) == Frame.Sort.H)


if __name__ == '__main__':
    unittest.main()
