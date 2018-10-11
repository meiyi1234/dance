import asyncio

import serial_asyncio
from bitstring import BitArray

from circ_buffer import CircularBuffer
from framing import START_STOP_BYTE, Frame, IFrame, SFrame, HFrame


csvfile = None


class SerialProtocol(asyncio.Protocol):
    """Based on https://stackoverflow.com/questions/30937042/asyncio-persisent-client-protocol-class-using-queue"""
    def __init__(self):
        self.transport = None
        self.queue = asyncio.Queue()
        self._ready = asyncio.Event()
        asyncio.ensure_future(self._send_messages())

        self._sending_iframe = False
        self._send_handshake_task = None          # Future
        self._ack_iframe_ready = asyncio.Event()  # Trigger task ack iframe
        self._rej_iframe_ready = asyncio.Event()  # Trigger task rej iframe
        self._secondary_ready = asyncio.Event()   # Block message sending unless secondary ready

        self.send_seq = 1   # Secondary increments its own send_seq separately
        self.recv_seq = 0   # Secondary increments its own recv_seq separately
        self.start_stop_count = 0

        self.buf = CircularBuffer(1024)  # 1024-byte buffer
        self.send_buf = {}
        self.send_buf[self.send_seq] = []  # Allow appending to first elem

    async def _send_handshake(self):
        """Send handshake every 2 seconds."""
        while True:
            self.transport.write(HFrame(self.send_seq).bytes)
            print('Sent handshake')
            await asyncio.sleep(2)

    def connection_made(self, transport):
        """On connection, send a handshake message to the Arduino.
        If testing on Windows, handshake frame in this method will be sent
        before serial port is ready. Sleep 2 seconds before sending a message
        (see https://github.com/pyserial/pyserial-asyncio/issues/3)
        """
        self.transport = transport
        print('Port opened')
        # Periodically send handshake
        self._send_handshake_task = asyncio.ensure_future(self._send_handshake())

    async def _send_messages(self):
        """Send messages to the server as they become available. If RNR is received,
        waits until RR is received.
        """
        await self._ready.wait()  # Wait until handshake is complete
        while True:
            await self._secondary_ready.wait()  # Wait if RNR received
            data = await self.queue.get()
            self.transport.write(data)
            print('Message sent: {}\n'.format(BitArray(data)))
            if self._sending_iframe:
                # Write to send buffer in case retransmission requested
                self.send_buf[self.send_seq] = data
                self._incr_send_seq()
                self.send_buf[self.send_seq] = []
                self._sending_iframe = False
            else:  # s/h
                # Store all frames for retransmission if REJ received
                # Since s/h don't have unique send no, store under last iframe seq
                self.send_buf[self.send_seq].append(data)

    async def send_message(self, message):
        """Feed a message to the sender coroutine."""
        await self.queue.put(message)

    async def send_iframe(self, message):
        """Send iframe and mark send_seq for incrementing after sending."""
        self._sending_iframe = True
        await self.send_message(message)

    def data_received(self, data):
        global csvfile

        self.buf.write(data)
        self.start_stop_count += data.count(START_STOP_BYTE)

        if self.start_stop_count > 1:   # Read if full frame received
        #     bytearr = bytearray(START_STOP_BYTE)
        #     if self.buf.read() != START_STOP_BYTE:
        #         # Data not framed by byte, ignore until start byte encountered
        #         # read_until will pop the start byte from the buffer, so
        #         # bytearr is initialised with start byte already
        #         self.buf.read_until(START_STOP_BYTE, ignore_first_byte=False)

            bytearr = self.buf.read_until(START_STOP_BYTE, ignore_first_byte=True)
            self.start_stop_count -= bytearr.count(START_STOP_BYTE)

            print('Received bytes: {}'.format(BitArray(bytearr)))

            try:
                fr = Frame.make_frame(bytearr)
            except ValueError as e:  # Frame error
                print(e)
                print('Requesting retransmission')
                self._rej_iframe_ready.set()  # Send REJ frame
                return

            if fr.SORT == Frame.Sort.H:
                if fr.recv_seq == self.send_seq:   # Arduino echoed seq sent
                    print('Received handshake ack, can now send data to the Arduino')
                    self._send_handshake_task.cancel()  # Stop sending handshake
                    self._ready.set()  # Enable sending messages
                    self._secondary_ready.set()
                    asyncio.ensure_future(self._ack_iframe())  # Enable acks
                    asyncio.ensure_future(self._rej_iframe())  # Enable nacks
                else:
                    print('Handshake recv seq does not match handshake send seq')

            elif fr.SORT == Frame.Sort.I:
                print('Fr send seq: {}'.format(fr.send_seq))

                self._incr_recv_seq()
                # One or more frames were lost
                if fr.send_seq != self.recv_seq:
                    print('Frame(s) {} missing. Requesting retransmission'.format(
                        [i for i in range(self.recv_seq, fr.send_seq)])
                    )
                    self._decr_recv_seq()
                    self._rej_iframe_ready.set()  # Send REJ frame
                    return

                # print('Writing data to file')
                csvfile.write(fr.to_ascii() + '\n')
                csvfile.flush()

                # Acknowledge receipt of I-frame
                # TODO: ack every n frames?
                print('Acknowledging I-frame')
                self._ack_iframe_ready.set()

            else:  # S-frame
                print('Received S-frame')
                if fr.TYPE == SFrame.Type.RR:
                    self._secondary_ready.set()  # Let messages be sent
                    # All frames up to recv_seq acked, del
                    self._clear_send_buf(fr.recv_seq)
                elif fr.TYPE == SFrame.Type.REJ:
                    # Resend frames from fr.recv_seq upto send_seq
                    # (send_seq incremented after last I send, do not include current send no.)
                    for i in range(fr.recv_seq, self.send_seq):
                        self.send_message(self.send_buf[i])  # Dont incr send seq again
                elif fr.TYPE == SFrame.Type.RNR:
                    self._secondary_ready.wait()  # Block sending new messages
                    self._clear_send_buf(fr.recv_seq)

    def connection_lost(self, exc):
        print('Port closed')
        self.transport.loop.stop()

    def _clear_send_buf(self, start):
        for i in range(start):
            del self.send_buf[i]
        # Init to empty list if no elem exists
        self.send_buf[self.send_seq] = (self.send_buf[self.send_seq]
                                        if self.send_seq in self.send_buf
                                        else [])

    async def _ack_iframe(self):
        """Send an S-frame with N(R) field set to (self.recv_seq + 1) mod 128.
        N(R) acknowledges that all frames with N(S) values up to N(R)−1 mod 128
        have been received and indicates the N(S) of the next frame it expects
        to receive.
        Since data_received is synchronous, it cannot call an async function to
        send messages. Therefore it insteads sets self._ack_iframe_ready to
        trigger this function.
        """
        while True:
            await self._ack_iframe_ready.wait()
            sfr = SFrame((self.recv_seq + 1) & 0x7F, SFrame.Type.RR)
            await self.send_message(sfr.bytes)
            self._ack_iframe_ready.clear()

    async def _rej_iframe(self):
        """Send an S-frame with N(R) field set to self.recv_seq mod 128.
        Requests immediate retransmission of all frames from N(R) onwards,
        including S-frames sent after I-frame with N(S) >= N(R).
        Since data_received is synchronous, it cannot call an async function to
        send messages. Therefore it insteads sets self._rej_iframe_ready to
        trigger this function.
        """
        while True:
            await self._rej_iframe_ready.wait()
            print('Sending rej with seq {}'.format(self.recv_seq))
            sfr = SFrame(self.recv_seq, SFrame.Type.REJ)
            await self.send_message(sfr.bytes)
            self._rej_iframe_ready.clear()

    def _incr_send_seq(self):
        """Increment send sequence number. Must be called after sending an
        I-frame.
        """
        self.send_seq = (self.send_seq + 1) & 0x7F  # 0-127

    def _incr_recv_seq(self):
        self.recv_seq = (self.recv_seq + 1) & 0x7F  # 0-127

    def _decr_recv_seq(self):
        self.recv_seq = (self.recv_seq - 1) & 0x7F  # 0-127


async def feed_frame(protocol, frame):
    if frame.SORT == Frame.Sort.I:
        await protocol.send_iframe(frame.bytes)
    else:
        await protocol.send_message(frame.bytes)


if __name__ == '__main__':
    csvfile = open('readings.csv', 'w+', newline='')
    csvfile.seek(0)
    csvfile.truncate()   # remove preivous data
    csvfile.write('AcX 1,AcY 1,AcZ 1,GyX 1,GyY 1,GyZ 1,AcX 2,AcY 2,AcZ 2,GyX 2,GyY 2,GyZ 2,AcX 3,AcY 3,AcZ 3,GyX 3,GyY 3,GyZ 3,voltage,current,power,energy\n')

    loop = asyncio.get_event_loop()
    coro = serial_asyncio.create_serial_connection(loop, SerialProtocol, '/dev/serial0', baudrate=115200)
    _, proto = loop.run_until_complete(coro)
    message = IFrame(proto.recv_seq, proto.send_seq, b'abcdef')
    asyncio.ensure_future(feed_frame(proto, message))

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        print('Closing connection')

    loop.close()
    csvfile.flush()
    csvfile.close()
