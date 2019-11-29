#!/usr/bin/env python3

import sys
import spidev
import crc16
import argparse
import numpy as np


def main(args):
    np.random.seed(0x1337)
    spi = spidev.SpiDev()
    spi.open(args.bus, args.device)
    spi.max_speed_hz = args.speed
    spi.bits_per_word = 16
    dt = np.dtype(np.uint16)
    dt = dt.newbyteorder('>')

    if args.send:
        data = np.array([48713, 25935, 32857, 42796, 38931, 10992, 61444], dtype=np.uint16)#np.random.randint(0, 0xffff, size=7, dtype=dt) #np.arange(1, 8, dtype=dt)
        print(data)
        crc = 0xffff if args.crcerr else crc16.crc16xmodem(data.tobytes())
        data = np.append(data, crc)
        data = data.astype(dt).tobytes()
        spi.writebytes(data)
        print(f"written {len(data)} bytes ({crc})")
    else:
        raw = bytes(spi.readbytes(16))
        msg = np.frombuffer(raw, dtype=dt)
        crc = crc16.crc16xmodem(raw[:-2])
        print(crc)

        if crc == msg[-1]:
            print(f"received {msg[:-1]} ({len(raw)} bytes) [OK]")
        else:
            print(f"crc error: {crc} != {msg[-1]} [FAIL]")

    spi.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="spi tester")
    parser.add_argument("--send", action="store_true", \
	  help="send data")
    parser.add_argument("--crcerr", action="store_true", \
	  help="emulate crc error")
    parser.add_argument("--speed", type=int, \
	  help="speed in Hz", default=200000)
    parser.add_argument("--bus", type=int, \
	  help="spi bus", default=0)
    parser.add_argument("--device", type=int, \
	  help="spi device", default=0)

    args = parser.parse_args()
    sys.exit(main(args))
