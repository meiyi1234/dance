"""
1. Create a python virtual environment if you haven't already with `python3 -m venv client`.
2. `cd` into the folder and `pip3 install pycrypto`.
3. `python3 client.py <ip_address> <port> <aes_key>` will start a client that connects to the 
   server at the port/address specified. Press Enter after starting the client to send 
   a test message, encrypted with aes_key, to the server and exit.

If you are running the server on your laptop, ensure you use the IP address on the network the pi
is connected to.
"""

import base64
import socket
import sys
from time import sleep

from Crypto.Cipher import AES
from Crypto import Random



class Client:
    def __init__(self, ip_addr, port_num, aes_key):
        self.key = bytes(str(aes_key), encoding = "utf8")
        # Connect to TCP socket at ip_addr:port_num
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip_addr, port_num))


    def encrypt(self, message):
        # Make message length multiple of block size
        padded = self._pad(message)
        iv = Random.new().read(16)
        cipher = AES.new(self.key, AES.MODE_CBC, iv)
        return base64.b64encode(iv + cipher.encrypt(padded))


    def _pad(self, s):
        return s + (AES.block_size - len(s) % AES.block_size) * chr(AES.block_size - len(s) % AES.block_size)


    def send(self, action, power_details=None):
        # Fill dummy values if power_details not provided
        p = power_details if power_details else {'voltage': 0, 'current': 0, 'power': 0, 'cumpower': 0}
        # Format message per server expectations
        message = '#{}|{}|{}|{}|{}'.format(action, p['voltage'], p['current'], p['power'], p['cumpower'])
        self.sock.send(self.encrypt(message))


    def end(self):
        self.send('logout')
        self.sock.close()



if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Invalid number of arguments')
        print('python client.py [IP address] [Port] [AES key]')
        sys.exit()

    ip_addr = sys.argv[1]
    port_num = int(sys.argv[2])
    key = sys.argv[3]

    if not(len(key) == 16 or len(key) == 24 or len(key) == 32):
        print("AES key must be either 16, 24, or 32 bytes long")
        sys.exit()

    client = Client(ip_addr, port_num, key)
    action = None
    while action != 'logout':
        print('Enter dance move to send, or `logout` to close connection')
        action = sys.stdin.readline().strip()
        client.send(action, {'voltage': 0, 'current': 1, 'power': 2, 'cumpower': 3})

    sleep(1)
    client.end()
