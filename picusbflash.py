#!/usr/bin/env python

import argparse, errno, os, sys
from serial import Serial
from intelhex import IntelHex16bit, IntelHexError

PORT_ENV_VAR = 'PICUSBFLASH_PORT'
CONFIGURATION_WORD_RANGE = range(0x8000*2, 0x8009*2, 2)
FLASH_ROW_LEN = 32

BCMD_ERASE = 0x45
BCMD_RESET = 0x52

# Status codes
STATUS_OK = 1
STATUS_INVALID_COMMAND = 2
STATUS_INVALID_CHECKSUM = 3
STATUS_VERIFY_FAILED = 4

STATUS_MESSAGES = {
    STATUS_INVALID_COMMAND:     'invalid command',
    STATUS_INVALID_CHECKSUM:    'checksum failed; data not written',
    STATUS_VERIFY_FAILED:       'write verification failed'
}


__version__ = '1.0'
__author__ = 'Matt Sarnoff (msarnoff.org)'

def low(n):
    return n & 0xFF

def high(n):
    return (n >> 8) & 0xFF

log_enabled = True
def log(*args):
    if log_enabled:
        print ' '.join(str(a) for a in args)

def error(msg):
    print >> sys.stderr, 'error:', msg

def exit_with_error(code, msg):
    error(msg)
    sys.exit(code)

def warn(msg):
    print >> sys.stderr, 'warning:', msg

def device_error(status):
    error(STATUS_MESSAGES.get(status, 'unknown status code 0x%02x' % status))

def device_set_params(ser, wordaddr, checksum=None):
    log('- Erasing %d words at %04x%s' % (
        FLASH_ROW_LEN,
        wordaddr,
        (', checksum of data to be written is 0x%02x' % checksum) if checksum is not None else ''
    ))
    ser.write(bytearray([low(wordaddr), high(wordaddr), checksum or 0, BCMD_ERASE]))
    status = ord(ser.read(1)[0])
    if status != STATUS_OK:
        device_error(status)
        return False
    else:
        return True

def device_write(ser, databytes):
    log('# Writing %d bytes' % len(databytes))
    ser.write(databytes)
    status = ord(ser.read(1)[0])
    if status != STATUS_OK:
        device_error(status)
        return False
    else:
        return True

def device_reset(ser):
    log('= Resetting device')
    ser.write(bytearray([BCMD_RESET]))
    return True



def main(args):
    global log_enabled
    
    parser = argparse.ArgumentParser(
            description='Uploads firmware (in Intel HEX format) to a PIC16F1xxx using Matt Sarnoff\'s USB bootloader.',
            epilog='If no port is specified using -p, the '+PORT_ENV_VAR+' environment variable is consulted. '+
            """
            The entire range of application program memory (excluding the bootloader region) is erased
            during the programming process. To preserve the contents of High-Endurance Flash across reprograms,
            use the -r option with the appropriate value. (e.g. for the PIC16F1454, which has 8K ROM and 128 bytes
            of HEF, use '-r 8064'.)
            """)
    parser.add_argument('-v', '--version',
            action='version',
            version='%(prog)s version '+__version__+' by '+__author__)
    parser.add_argument('-q', '--quiet',
            action='store_true',
            help='disable logging')
    parser.add_argument('-p', '--port',
            metavar='PORT',
            help='the serial port that the PIC is attached to')
    parser.add_argument('-b', '--bootloader-size',
            metavar='SIZE',
            default=512,
            type=int,
            help='size of the bootloader in words, 512 or 4096 (defaults to 512)')
    parser.add_argument('-r', '--rom-size',
            metavar='SIZE',
            default=8192,
            type=int,
            help='size of the device ROM in words up to 32768 (defaults to 8192)')
    parser.add_argument('inputfile',
            metavar='inputfile',
            nargs=1,
            help='16-bit Intel HEX input file')
    args = parser.parse_args()

    infile = args.inputfile[0]

    if args.quiet is True:
        log_enabled = False

    if args.port is not None:
        port = args.port
    else:
        port = os.getenv(PORT_ENV_VAR)

    if not port:
        exit_with_error(1, 'no serial port specified (use -p or set the '+PORT_ENV_VAR+' environment variable)')

    min_address = args.bootloader_size
    if min_address != 512 and min_address != 4096:
        exit_with_error(2, 'bootloader size must be 512 or 4096')

    max_address = args.rom_size
    if max_address <= min_address or max_address > 32768:
        exit_with_error(3, 'ROM size must be <= 32768 and > bootloader size')

    # open the hex file
    try:
        ih = IntelHex16bit(infile)
    except IOError, e:
        exit_with_error(4, e)
    except IntelHexError, e:
        exit_with_error(5, e)

    if ih.minaddr() < min_address:
        exit_with_error(6, 'hex file starts at 0x%04x, but the minimum allowable start address is 0x%04x' % (
          ih.minaddr(), min_address))

    # check for configuration words and warn that they won't be written
    # IntelHex has no "check if address is populated" method, so we have to use
    # the dictionary representation
    ihdict = ih.todict()
    for byteaddr in CONFIGURATION_WORD_RANGE:
        if ihdict.get(byteaddr) is not None:
            wordaddr = byteaddr/2
            warn('bootloader cannot write to address 0x%x in configuration space' % wordaddr)
            # remove configuration words from hex
            del ih[byteaddr:byteaddr+2]

    ih.padding = 0x3fff
    log('Code range: 0x%04x-0x%04x (%d words)' % (ih.minaddr(), ih.maxaddr(), ih.maxaddr()-ih.minaddr()+1))
    
    # open the serial port
    log('Opening serial port '+port)
    try:
        ser = Serial(port, 38400)    # baud doesn't matter
    except Exception, e:
        exit_with_error(7, e)

    failed = False
    for wordaddr in range(min_address, max_address, FLASH_ROW_LEN):
        row_in_range = wordaddr >= ih.minaddr() and wordaddr <= ih.maxaddr()

        # compute the row checksum
        checksum = None

        if row_in_range:
            words = ih.tobinarray(wordaddr, size=FLASH_ROW_LEN)
            data = reduce(lambda arr,word: arr.extend([low(word), high(word)]) or arr, words, [])
            checksum = -sum(data) & 0xff    # two's complement of lower 8 bits of sum of bytes

        # erase the row
        if not device_set_params(ser, wordaddr, checksum):
            failed = True
            break

        # if the row is within the range of the hex file, program the data
        if row_in_range:
            if not device_write(ser, data):
                failed =  True
                break

    if not failed:
        device_reset(ser)
        log('Done.')

    log('Closing serial port '+port)
    ser.close()

    if failed:
        exit_with_error(127, 'Programming failed')

    return 0



if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
