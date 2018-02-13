#!/usr/bin/env python

import thread
import serial
from time import sleep

#import Adafruit_BBIO.UART as UART
from pycrc.algorithms import Crc
from pycrc.models import CrcModels


class CrcEncoder(object):
    def __init__(self, model='dallas-1-wire'):
        self.params = CrcModels().get_params(model)
        self.encoder = Crc(self.params['width'],
                           self.params['poly'],
                           self.params['reflect_in'],
                           self.params['xor_in'],
                           self.params['reflect_out'],
                           self.params['xor_out'])

    def encode(self, msg):
        crc_hex = self.encoder.bit_by_bit(msg)
        return chr(crc_hex) + msg


class ArduinoBoard(object):
    RESET_PIN = 3 # TODO: find reset pin and make function to reset arduino
    PORT = '/dev/ttyO1'

    def __init__(self, callback=None, debug=False):
        if not callback:
            print('Specify callback')
            return

        self.debug = debug
        self.port = None
        self.callback = callback
        self.lock = thread.allocate_lock()
        self.crc_encoder = CrcEncoder()

        if self.debug:
            print('Trying to open port')
        self._open()
        if self.debug:
            print("Connection established.")
        thread.start_new_thread(self._read_loop, (callback,))

    def __del__(self):
        if self.debug:
           print 'Closing port'
        if self.port and self.port.is_open:
            self.port.close()


    def _reboot_and_check(self):
        # TODO
        pass
    
    def _open(self):
        #UART.setup("UART1")
        self.port = serial.Serial(port=self.PORT, baudrate=115200)
        self.port.close()
        self.port.open()
        return

    def send(self, msg):
        data = self.crc_encoder.encode(msg)
        with self.lock:
            self.port.write(data)

    def _read_line(self):
        with self.lock:
            result = self.port.readline()
        return result

    def _configure(self):
        # TODO
        pass

    def _read_loop(self, callback):
        while True:
            message = self._read_line()
            if message != '':
                thread.start_new_thread(callback, (message,))


def process_messages(message):
    if 'pong' in message:
        print(message)
    if 'cmd' in message:
        print(message)
    pass

if __name__ == "__main__":
    try:
        arduino = ArduinoBoard(callback=process_messages, debug=True)
        sleep(3)
        arduino.send("go(1500,1500,1500);")
        while True:
            arduino.send('ping(0);')
            arduino.send('ligt(300);')
            sleep(0.5)
            arduino.send('ligt(255);')
            sleep(0.5)
            arduino.send('ligt(100);')
            sleep(0.5)
    except KeyboardInterrupt:
        pass
