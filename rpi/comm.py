import asyncio
from time import sleep

import serial_asyncio

from circ_buffer import CircularBuffer
from framing import START_STOP_BYTE, Frame, HFrame


send_sequence = 1
receive_sequence = 0
buf = CircularBuffer(1024)  # 1024-byte buffer


class SerialProtocol(asyncio.Protocol):
    """From https://stackoverflow.com/questions/30937042/asyncio-persisent-client-protocol-class-using-queue"""
    def __init__(self):
        self.transport = None
        self.queue = asyncio.Queue()
        self._ready = asyncio.Event()
        asyncio.ensure_future(self._send_frames())

    def connection_made(self, transport):
        """On connection, send a handshake message to the Arduino.
        The method is called before the serial port is ready on Windows - see
        https://github.com/pyserial/pyserial-asyncio/issues/3
        """
        global receive_sequence

        self.transport = transport
        print('Port opened')
        self.transport.write(HFrame(receive_sequence).bytes)  # Write serial data via transport
        self._ready.set()

    async def _send_frames(self):
        """Send messages to the server as they become available."""
        global send_sequence

        await self._ready.wait()
        while True:
            data = await self.queue.get()
            self.transport.write(data)
            send_sequence = (send_sequence + 1) & 0x7F  # 0-127
            print('Message sent: {}'.format(message))

    async def send_frame(self, frame):
        """Feed a message to the sender coroutine."""
        await self.queue.put(frame)

    def data_received(self, data):
        global buf

        buf.write(data)
        if data.count(START_STOP_BYTE) > 1:
            bytearr = buf.read_until(START_STOP_BYTE, ignore_first_byte=True)
            try:
                fr = Frame.make_frame(bytearr)
            except ValueError as e:
                # TODO: send rej frame
                print(e)

 
    def connection_lost(self, exc):
        print('Port closed')
        self.transport.loop.stop()


async def feed_messages(protocol, message):
    await protocol.send_frame(message)


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    coro = serial_asyncio.create_serial_connection(loop, SerialProtocol, 'COM14', baudrate=38400)
    _, proto = loop.run_until_complete(coro)
    sleep(2)
    message = b'check'
    asyncio.ensure_future(feed_messages(proto, message))
    
    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print('Closing connection')
    loop.close()
