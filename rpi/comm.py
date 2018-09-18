import asyncio
from time import sleep

import serial_asyncio

from circ_buffer import CircularBuffer
from framing import START_STOP_BYTE, Frame, SFrame, HFrame


class SerialProtocol(asyncio.Protocol):
    """Based on https://stackoverflow.com/questions/30937042/asyncio-persisent-client-protocol-class-using-queue"""
    def __init__(self):
        self.transport = None
        self.queue = asyncio.Queue()
        self._ready = asyncio.Event()
        asyncio.ensure_future(self._send_messages())

        self.iframe_sent = asyncio.Event()
        self.sending_iframe = False
        self.send_seq = 1   # Secondary increments its own send_seq separately
        self.recv_seq = 0   # Secondary increments its own recv_seq separately
        self.start_stop_count = 0
        self.buf = CircularBuffer(1024)  # 1024-byte buffer

    def connection_made(self, transport):
        """On connection, send a handshake message to the Arduino.
        If testing on Windows, handshake frame in this method will be sent
        before serial port is ready. Sleep 2 seconds before sending a message
        (see https://github.com/pyserial/pyserial-asyncio/issues/3)
        """
        self.transport = transport
        print('Port opened')
        self.transport.write(HFrame(self.recv_seq).bytes)
        self._incr_send_seq()

    async def _send_messages(self):
        """Send messages to the server as they become available."""
        await self._ready.wait()
        while True:
            data = await self.queue.get()
            self.transport.write(data)
            print('Message sent: {}'.format(message))
            if self.sending_iframe:
                self._incr_send_seq()
                self.sending_iframe = False

    async def send_message(self, message):
        """Feed a message to the sender coroutine."""
        await self.queue.put(message)

    async def send_iframe(self, message):
        """Send iframe and mark send_seq for incrementing after sending."""
        self.sending_iframe = True
        await self.send_message(message)

    def data_received(self, data):
        self.buf.write(data)
        self.start_stop_count += data.count(START_STOP_BYTE)

        if self.start_stop_count > 1:   # Read if full frame received
            bytearr = self.buf.read_until(START_STOP_BYTE, ignore_first_byte=True)
            self.start_stop_count -= bytearr.count(START_STOP_BYTE)

            try:
                fr = Frame.make_frame(bytearr)
            except ValueError as e:  # Checksum invalid
                print(e)
                # TODO: send rej frame
                return

            if fr.SORT == Frame.SORT.H:
                if fr.recv_seq == self.send_seq:   # Arduino echoed seq sent
                    print('Received handshake ack, can now send data to the Arduino')
                    self._ready.set()

            elif fr.SORT == Frame.SORT.I:
                print('Acknowledging I-frame')

                self._incr_recv_seq()
                # One or more frames were lost
                if fr.send_seq != self.recv_seq:
                    print('Frame(s) {} missing'.format(
                        [i for i in range(self.recv_seq, fr.send_seq)])
                    )
                    self._decr_recv_seq()
                    # TODO: send rej frame
                    return

                print('\nfr: {}\n'.format(fr))

                # Acknowledge receipt of I-frame
                # TODO: ack every n frames?
                self._ack_iframe()

    def connection_lost(self, exc):
        print('Port closed')
        self.transport.loop.stop()

    def _ack_iframe(self):
        """Send an S-frame with N(R) field set to (self.recv_seq + 1) mod 128.
        N(R) acknowledges that all frames with N(S) values up to N(R)−1 mod 128
        have been received and indicates the N(S) of the next frame it expects
        to receive.
        """
        sfr = SFrame((self.recv_seq + 1) & 0x7F, SFrame.Type.RR)
        self.send_message(sfr.bytes)

    async def _incr_send_seq(self):
        """Increment send sequence number. Must be called after sending an
        I-frame.
        """
        self.send_seq = (self.send_seq + 1) & 0x7F  # 0-127

    def _incr_recv_seq(self):
        self.recv_seq = (self.recv_seq + 1) & 0x7F  # 0-127

    def _decr_recv_seq(self):
        self.recv_seq = (self.recv_seq - 1) & 0x7F  # 0-127


async def feed_message(protocol, bytes_, frame_sort):
    if frame_sort == Frame.SORT.I:
        await protocol.send_iframe(bytes_)
    else:
        await protocol.send_message(bytes_)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    coro = serial_asyncio.create_serial_connection(loop, SerialProtocol, 'COM14', baudrate=38400)
    _, proto = loop.run_until_complete(coro)
    sleep(2)
    message = b'check'
    asyncio.ensure_future(feed_message(proto, message, None))

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print('Closing connection')
    loop.close()
