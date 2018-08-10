#!/usr/bin/env python3
#
# 180810 <fredrik@z80.se>

import serial
from binascii import unhexlify, hexlify
import time


class CommunicationError(Exception):
    pass


class ControllerError(Exception):
    pass


def unpackRecord(buf):
    identifier = buf[0]
    dataLength = int(buf[1:3], 16)
    data = unhexlify(buf[3:-2])
    checksum = int(buf[-2:], 16)
    return identifier, data, dataLength, checksum

def isValidRecord(buf):
    identifier, data, dataLength, checksum = unpackRecord(buf)

    expectedLength = 3 + 2*dataLength + 2

    if len(buf) != expectedLength:
        print('Invalid record length %d != %d' % (len(buf), expectedLength))
        return False

    calculatedChecksum = sum(data) & 0xff
    if checksum != calculatedChecksum:
        print('Bad checksum %02x != %02x' % (checksum, calculatedChecksum))
        return False
    return True


class IBM7545:

    def __init__(self, port):
        self.port = serial.Serial(port, baudrate=4800,
                bytesize=serial.SEVENBITS)

        self.port.dtr = True
        time.sleep(0.1)
        if not self.port.dsr:
            raise CommunicationError('Controller did not assert DSR')


    def command(self, identifier, argument=b'', wantRecords=True):
        pkt = b'%s%s\r\n' % (bytes(identifier), bytes(argument))
        #print('Send', pkt.strip().decode('ascii'))
        self.port.write(pkt)
        return self.handleResponse(wantRecords)


    def commandWithData(self, identifier, data, wantRecords=False):
        pkt = b'%s%02X%s%02X\r\n' % (bytes(identifier), len(data),
            hexlify(bytes(data)).upper(),
            sum(data) & 0xff)
        #print('Send', pkt.strip().decode('ascii'))
        self.port.write(pkt)
        return self.handleResponse(wantRecords)


    def handleResponse(self, wantRecords):
        print('Expecting response')
        buf = b''
        records = []
        while True:
            c = self.port.read(1)
            #print('Read %02x' % c[0])
            if c == b'\x06':
                return
            if c == b'\x04':
                raise ControllerError('EOT')
            elif c == b'\x15':
                raise ControllerError('NAK')
            elif c in (b'\x13', b'\x11'):
                continue
            else:
                buf += c
                if buf.endswith(b'\r\n'):
                    if len(buf) == 2:
                        buf = b''
                        print('Empty packet, clearing')
                    elif buf.strip() == b'EG':
                        self.port.write(b'\x06')
                        return records
                    elif isValidRecord(buf.strip()):
                        self.port.write(b'\x06')
                        record = unpackRecord(buf.strip())
                        records.append(record)
                        buf = b''
                    else:
                        self.port.write(b'\x15')
                        buf = b''


if __name__ == '__main__':
    bert = IBM7545('/dev/ttyUSB0')
    
    bert.command(b'X', b'13')
    time.sleep(0.1)

    _, data, _, _ = bert.command(b'R', b'03')[0]
    print('controller version:', hexlify(data))

    _, data, _, _ = bert.command(b'R', b'01')[0]
    print('machine status:', hexlify(data))

    _, data, _, _ = bert.command(b'R', b'10')[0]
    print('digital io:', hexlify(data))

    try:
        bert.commandWithData(b'T', (0x70, 0x21, 0x0a, 0xa2, 0x80, 0x00, 0x3d, 0x71, 0x21, 0, 0, 0, 0, 0x3d))
        bert.commandWithData(b'T', (0x72, 0x21, 0x0a, 0x00, 0x00, 0x00, 0x3d, 0x73, 0x21, 0, 0, 0, 0, 0x3d, 0x3e))
        # TODO: Wait for XOFF, XON to determine when move has finished
    except ControllerError:
        print('Controller Error')
        time.sleep(0.5)
        _, data, _, _ = bert.command(b'R', b'02')[0]
        print('reject:', hexlify(data))
