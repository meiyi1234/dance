from enum import Enum

import bitstring
from bitstring import BitArray
from crc16 import crc16xmodem


START_STOP_BYTE = 126  # 7E
ESCAPE_BYTE = 125      # 7D


class Frame:
    class Sort(Enum):
        I = 0     # Information
        S = 1     # Supervisory
        H = 3     # Handshake

    SORT = None
    CONTROL_MASK = BitArray('0x0003')
    SFRAME_MASK = BitArray('0x000C')

    def __init__(self, bitarr, info, escaped):
        """Stores escaped frame in self.bytes and unescaped frame in self.bitarr.
        Other useful fields such as control and checksum are also stored as instance
        members.
        """
        if escaped:
            self.bytes = bitarr.bytes
            self.bitarr = BitArray(Frame.unescape(bitarr.bytes))
        else:
            self.bytes = Frame.escape(bitarr.bytes)   # type bytes. Contains full, escaped frame
            self.bitarr = bitarr          # type bitarr. Contains unescaped frame, no start/stop bytes

        self.control = self.bitarr[8:24]   # type bitarr
        self.recv_seq = self.control[0:7].uint
        self.info = self.bitarr[24:-24].bytes if info else None   # type bitarr
        self.checksum = self.bitarr[-24:-8].uint                  # type uint, does NOT recalculate
        if not self.is_checksum_valid():
            raise ValueError('Checksum received ({}) is not equal to checksum calculated ({})'
                .format(self.checksum, self.calc_checksum(self.bitarr[8:-24].bytes)))

    def calc_checksum(self, bytes_):
        return crc16xmodem(bytes_)

    def is_checksum_valid(self):
        return self.checksum == self.calc_checksum(self.bitarr[8:-24].bytes)

    @staticmethod
    def escape(bytes_):
        """Escapes bytes 7D and 7E with 7D EXCEPT for first and last bytes"""
        escaped = []
        for index, byte in enumerate(bytes_):
            if (byte == START_STOP_BYTE
              and not (index == 0 or index == len(bytes_)-1)):
                escaped.append(ESCAPE_BYTE)
                escaped.append(94)  # 7E ^ 20 = uint 94
            elif (byte == ESCAPE_BYTE
              and not (index == 0 or index == len(bytes_)-1)):
                escaped.append(ESCAPE_BYTE)
                escaped.append(93)  # 7D ^ 20 = uint 93
            else:
                escaped.append(byte)

        return bytes(escaped)

    @staticmethod
    def unescape(bytes_):
        """De-escapes bytes 7D and 7E with 7D EXCEPT for first and last bytes"""
        escape_state = 0
        unescaped = []
        for index, byte in enumerate(bytes_):
            # Remove only first escape byte in a pair
            if (byte == ESCAPE_BYTE and escape_state == 0
              and not (index == 0 or index == len(bytes_)-1)):
                escape_state = 1
            # No need for index check since escape state is 1 for 2nd elem onwards
            elif escape_state == 1:
                if byte == 94:     # 7E was xor-ed
                    unescaped.append(START_STOP_BYTE)
                elif byte == 93:   # 7D was xor-ed
                    unescaped.append(ESCAPE_BYTE)
                else:
                    raise ValueError('Byte {} escaped when it should not be'.format(byte))
                escape_state = 0
            else:
                unescaped.append(byte)
                escape_state = 0

        return bytes(unescaped)

    @staticmethod
    def make_frame(bytes_):
        """Creates I, S or H-frame based on bitarr. Bitarr must contain exactly
        one complete frame, including start and stop bytes.
        Frame returned is of type Frame, not its subclass.
        """
        print('bytes_ len: {}'.format(len(bytes_)))
        if (bytes_[0] != START_STOP_BYTE or bytes_[-1] != START_STOP_BYTE):
            raise ValueError('Message does not contain either start byte, stop byte, or both')

        bitarr = BitArray(bytes_)
        control = bitarr[8:24]
        sort = Frame.get_frame_sort(control)
        if sort == Frame.Sort.I:
            frame = Frame(bitarr, info=True, escaped=True)
            frame.__class__ = IFrame
            frame.SORT = Frame.Sort.I
            frame.send_seq = control[8:15].uint

        elif sort == Frame.Sort.S:
            frame = Frame(bitarr, info=False, escaped=True)
            frame.__class__ = SFrame
            frame.SORT = Frame.Sort.S
            frame.TYPE = Frame.get_sframe_type(control)

        elif sort == Frame.Sort.H:
            frame = Frame(bitarr, info=False, escaped=True)
            frame.__class__ = HFrame
            frame.SORT = Frame.Sort.H

        return frame

    @staticmethod
    def get_frame_sort(control):
        """Return Frame.Sort enum (S/I) based on 16-bit packet control bitarray."""
        val = (control & Frame.CONTROL_MASK).uint
        if val == 0 or val == 2:  # 00 or 10
            return Frame.Sort.I
        else:
            return Frame.Sort(val)

    @staticmethod
    def get_sframe_type(control):
        """Return SFrameType enum based on S-frame packet control field."""
        return SFrame.Type((control & Frame.SFRAME_MASK).uint >> 2)


class IFrame(Frame):
    SORT = Frame.Sort.I

    def __init__(self, recv_seq, send_seq, info, p_f=1):
        """Creates an I-frame for sending with info as the payload.
        To classify a received frame, use the Frame.make_frame method.
        info should be ascii-encoded bytes.
        """
        self.send_seq = send_seq

        control_byte1 = bitstring.pack('uint:7, bool', recv_seq, p_f).uint
        control_byte2 = bitstring.pack('uint:7, bool=0', send_seq).uint

        checksum_bytes = bitstring.pack(
            'uint:8, uint:8, bits',
            control_byte1, control_byte2, info
        ).bytes
        checksum = self.calc_checksum(checksum_bytes)  # Calculated over unescaped bytes

        bitarr = BitArray(bitstring.pack(
            'uint:8, uint:8, uint:8, bits, uint:16, uint:8',
            START_STOP_BYTE,
            control_byte1,
            control_byte2,
            info,             # variable length
            checksum,
            START_STOP_BYTE
        ))

        super().__init__(bitarr, info=True, escaped=False)

    def to_ascii(self):
        return self.info.decode('ascii')


class SFrame(Frame):
    class Type(Enum):
        RR = 0    # Receive Ready to accept more I-frames
        REJ = 1   # REJect, Go-Back-N retransmission request for an I-frame
        RNR = 2   # Receive Not Ready to accept more I-frames
        SREJ = 3  # Selective REJect, retransmission request for one I-frame

    SORT = Frame.Sort.S

    def __init__(self, recv_seq, sframe_type, p_f=1):
        """Creates an S-frame for sending with type specified.
        To classify a received frame, use the Frame.make_frame method.
        """
        self.TYPE = sframe_type

        control_byte1 = bitstring.pack('uint:7, bool', recv_seq, p_f).uint
        control_byte2 = bitstring.pack('uint:4=0, uint:2, uint:2=1', sframe_type.value).uint

        checksum_bytes = bitstring.pack(
            'uint:8, uint:8',
            control_byte1, control_byte2
        ).bytes

        bitarr = BitArray(bitstring.pack(
            'uint:8, uint:8, uint:8, uint:16, uint:8',
            START_STOP_BYTE,
            control_byte1,
            control_byte2,
            self.calc_checksum(checksum_bytes),
            START_STOP_BYTE
        ))

        super().__init__(bitarr, info=False, escaped=False)


class HFrame(Frame):
    SORT = Frame.Sort.H

    def __init__(self, send_seq):
        """Creates an H-frame for sending.
        To classify a received frame, use the Frame.make_frame method.
        """
        control_byte1 = bitstring.pack('uint:7, bool=1', send_seq).uint
        control_byte2 = 3  # 0b00000011 (RR, type H-frame)

        checksum_bytes = bitstring.pack(
            'uint:8, uint:8',
            control_byte1, control_byte2
        ).bytes

        bitarr = BitArray(bitstring.pack(
            'uint:8, uint:8, uint:8, uint:16, uint:8',
            START_STOP_BYTE,                     # 0x7E
            control_byte1,                       # receive_seq, p/f
            control_byte2,                       # 0x03
            self.calc_checksum(checksum_bytes),  # crc16xmodem(checksum_bytes)
            START_STOP_BYTE                      # 0x7E
        ))

        super().__init__(bitarr, info=False, escaped=False)
