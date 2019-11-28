#!/usr/bin/env python3

import sys
import time
import spidev
import crc16
import argparse
import numpy as np

# This is excluding the crc
SPI_MSG_LEN = 1024
BITS_PER_WORD = 16


def main(args):
    fail = 0
    n = 1

    spi = spidev.SpiDev()
    spi.open(args.bus, args.device)
    spi.bits_per_word = BITS_PER_WORD

    for i in range(n):
        spi.max_speed_hz = (i+1) * 1000000

        for j in range(args.n):
            data = np.random.randint(0, 0xffff, size=SPI_MSG_LEN, dtype=np.uint16)
            data[0] = j
            data[1] = args.n
            crc = crc16.crc16xmodem(data.byteswap())
            data = np.append(data, crc)
            data = data.astype(np.uint16).byteswap()
            spi.writebytes(data.tobytes())
            print(f"{j+1}/{args.n} {data.nbytes} bytes ({crc})")
            time.sleep(0.01)


        """
        raw = bytes(spi.readbytes((SPI_MSG_LEN + 1) * 2))
        msg = np.frombuffer(raw, dtype=dt)
        crc = crc16.crc16xmodem(raw[:-2])

        if crc == msg[-1]:
            print(f"{i} {spi.max_speed_hz//1000}KHz {msg[0]}/{args.n}")
        else:
            fail += 1
        """

    spi.close()
    print(f"{fail}/{n} failures")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="spi speed tester")
    parser.add_argument("--n", type=int, \
        help="nof transfers per speed", default=100)
    parser.add_argument("--bus", type=int, \
        help="spi bus", default=0)
    parser.add_argument("--device", type=int, \
        help="spi device", default=0)

    args = parser.parse_args()
    sys.exit(main(args))
