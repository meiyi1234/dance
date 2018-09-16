import binascii
from enum import Enum
import sys

import bitstring
from bitstring import BitArray
from crc16 import crc16xmodem
import serial


START_STOP_BYTE = 126  # 7E
ESCAPE_BYTE = 125      # 7D

send_sequence = 1
receive_sequence = 0


class Frame:
    class Sort(Enum):
        I = 0     # Information
        S = 1     # Supervisory
        H = 3     # Handshake

    SORT = None

    def __init__(self, bitarr, info, escaped):
        if escaped:
            self.bytes = bitarr.bytes
            self.bitarr = BitArray(Frame.unescape(bitarr.bytes))
        else:
            self.bytes = Frame.escape(bitarr.bytes)   # type bytes. Contains full, escaped frame
            self.bitarr = bitarr                      # type bitarr. Contains unescaped frame

        self.control = self.bitarr[8:24]                          # type bitarr
        self.info = self.bitarr[24:-24].bytes if info else None   # type bitarr
        self.checksum = self.bitarr[-24:-8].uint                  # type uint, does NOT recalculate
        if not self.is_checksum_valid():
            raise ValueError('Checksum received ({}) is not equal to checksum calculated ({})'
                .format(self.checksum, self.calc_checksum(self.bitarr[8:-24].bytes)))

        self.bytes = self.bitarr.bytes   


    def calc_checksum(self, bytes_):
        return crc16xmodem(bytes_)

    @staticmethod
    def escape(bytes_):
        """Escapes bytes 7D and 7E with 7D EXCEPT for first and last bytes"""
        escaped = []
        for index, byte in enumerate(bytes_):
            if ((byte == START_STOP_BYTE or byte == ESCAPE_BYTE) 
              and not (index == 0 or index == len(bytes_)-1)):
                escaped.append(ESCAPE_BYTE)
            escaped.append(byte)

        return bytes(escaped)

    @staticmethod
    def unescape(bytes_):
        """De-escapes bytes 7D and 7E with 7D EXCEPT for first and last bytes"""
        escaped_used = False
        unescaped = []
        for index, byte in enumerate(bytes_):
            if (byte == ESCAPE_BYTE and not escaped_used   # Remove only first escape byte in a pair
              and not (index == 0 or index == len(bytes_)-1)):
                escaped_used = True
            else:
                escaped_used = False
                unescaped.append(byte)
            
        return bytes(unescaped)

    def is_checksum_valid(self):
        return self.checksum == self.calc_checksum(self.bitarr[8:-24].bytes)

    @staticmethod
    def make_frame(bitarr):
        """Creates I, S or H-frame based on bitarr. Bitarr must contain exactly 
        one complete frame, excluding start and stop bytes.
        Frame returned is of type Frame, not its subclass.
        """
        control = bitarr[0:16]
        sort = get_frame_sort(control)
        if sort == Frame.Sort.I:
            frame = Frame(bitarr, info=True, escaped=True)
            frame.SORT = Frame.Sort.I

        elif sort == Frame.Sort.S:
            frame = Frame(bitarr, info=False, escaped=True)
            frame.SORT = Frame.Sort.S
            frame.type = get_sframe_type(control)

        elif sort == Frame.Sort.H:
            frame = Frame(bitarr, info=False, escaped=True)
            frame.SORT = Frame.Sort.H

        return frame


class IFrame(Frame):
    SORT = Frame.Sort.I

    def __init__(self, info, p_f=True):
        """Stores complete I-frame in self.bitstring and self.bitarr."""
        global receive_sequence, send_sequence
        control_byte1 = bitstring.pack('uint:7, bool', receive_sequence, p_f).uint
        control_byte2 = bitstring.pack('uint:7, bool=0', send_sequence).uint

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
            info,
            checksum,
            START_STOP_BYTE
        ))

        super().__init__(bitarr, info=True, escaped=False)



class SFrame(Frame):
    class Type(Enum):
        RR = 0    # Receive Ready to accept more I-frames
        REJ = 1   # REJect, Go-Back-N retransmission request for an I-frame 
        RNR = 2   # Receive Not Ready to accept more I-frames
        SREJ = 3  # Selective REJect, retransmission request for one I-frame  

    SORT = Frame.Sort.S

    def __init__(self, sframe_type, p_f=True):
        """Stores complete S-frame in self.bitstring and self.bitarr."""
        global receive_sequence
        self.type = sframe_type

        control_byte1 = bitstring.pack('uint:7, bool', receive_sequence, p_f).uint
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

    def __init__(self):
        """Stores complete H-frame in self.bitstring and self.bitarr."""
        global receive_sequence
        receive_sequence = 0
        control_byte1 = 1  # receive sequence 0, p/f 1
        control_byte2 = 3  # 0b00000011

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

    
CONTROL_MASK = BitArray('0x0003')
def get_frame_sort(control):
    """Return Frame.Sort enum (S/I) based on 16-bit packet control bitarray."""
    val = (control & CONTROL_MASK).uint
    if val == 0 or val == 2:  # 00 or 10
        return Frame.Sort.I
    else:
        return Frame.Sort(val)


SFRAME_MASK = BitArray('0x000C')
def get_sframe_type(control):
    """Return SFrameType enum based on S-frame packet control field."""
    return SFrame.Type((control & SFRAME_MASK).uint >> 2)


def send(frame, frameness):
    global receive_sequence, send_sequence
    # TODO
    if frame_type == Frame.I:
        send_sequence = (send_sequence + 1) & 0x7F  # 0-127
    elif frame_type == Frame.H:
        receive_sequence = 0
        send_sequence = 1


def receive():
    """State machine to determine current byte type"""

    if byte == START_STOP_BYTE:
        if frame_complete:  # previous frame marked complete, new frame being received
            frame_complete = False
        else:               # stop byte received, this frame is complete
            frame_complete = True





if __name__ == '__main__':
    hf = HFrame()
    print(hf.bitarr)
