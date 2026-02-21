#!/usr/bin/env python3
# ===================================================================================
# Project:   puyaisp - Programming Tool for PUYA PY32F0xx Microcontrollers
# Version:   v1.4
# Year:      2023
# Author:    Stefan Wagner
# Github:    https://github.com/wagiminator
# License:   MIT License
# ===================================================================================
#
# Description:
# ------------
# Simple Python tool for flashing PY32F0xx (and maybe other PY32) microcontrollers
# via USB-to-serial converter utilizing the factory built-in embedded boot loader.
#
# Dependencies:
# -------------
# - pyserial
#
# Operating Instructions:
# -----------------------
# You need to install PySerial to use puyaisp.
# Install it via "python3 -m pip install pyserial".
# You may need to install a driver for your USB-to-serial converter.
#
# Connect your USB-to-serial converter to your MCU:
# USB2SERIAL      PY32F0xx
#        RXD <--- PA2 or PA9  or PA14
#        TXD ---> PA3 or PA10 or PA15
#        VCC ---> VCC
#        GND ---> GND
#
# Set your MCU to bootloader mode by using ONE of the following methods:
# - Disconnect your USB-to-serial converter, pull BOOT0 pin (PF4) to VCC (or press
#   and hold the BOOT button, if your board has one), then connect the converter to
#   your USB port. BOOT0 pin (or BOOT button) can be released now.
# - Connect your USB-to-serial converter to your USB port. Pull BOOT0 pin (PF4)
#   to VCC, then pull nRST (PF2) shortly to GND (or press and hold the BOOT button,
#   then press and release the RESET button and then release the BOOT button, if
#   your board has them).
# These steps are not necessary when using a CH340X USB-to-serial converter with 
# control lines for nRST and BOOT0. In this case, the software automatically puts 
# the MCU into bootloader mode.
#
# Run "python3 puyaisp.py -f firmware.bin".

# If the PID/VID of the USB-to-Serial converter is known, it can be defined here,
# which can make the auto-detection a lot faster. If not, comment out or delete.
#PY_VID  = '1A86'
#PY_PID  = '7523'

# Define BAUD rate here, range: 4800 - 1000000, default and recommended: 115200
PY_BAUD = 115200

# Libraries
import sys
import time
import argparse
import serial
from serial import Serial
from serial.tools.list_ports import comports

# ===================================================================================
# Chip Debug Dump
# ===================================================================================

def _hexdump(data, base_addr, bytes_per_line=16):
    lines = []
    for offset in range(0, len(data), bytes_per_line):
        chunk = data[offset:offset + bytes_per_line]
        hex_part = ' '.join('%02X' % b for b in chunk)
        ascii_part = ''.join(chr(b) if 0x20 <= b < 0x7f else '.' for b in chunk)
        if bytes_per_line == 16:
            left = hex_part[:23]
            right = hex_part[24:]
            hex_part = '%-23s  %-23s' % (left, right)
        lines.append('  0x%08X: %-49s |%s|' % (base_addr + offset, hex_part, ascii_part))
    return '\n'.join(lines)

def _decode_rdp(val):
    if val == 0xAA:
        return 'Level 0 (no protection)'
    elif val == 0x55:
        return 'Level 1 (read protection active)'
    else:
        return 'Level 2 (permanent - IRREVERSIBLE!)'

def _decode_bor_level(level):
    thresholds = {
        0: '1.7V', 1: '1.8V', 2: '1.9V', 3: '2.0V',
        4: '2.1V', 5: '2.2V', 6: '2.3V', 7: '2.4V',
    }
    return thresholds.get(level, 'unknown')

def _dump_chip_info(isp):
    sep = '=' * 50
    print()
    print(sep)
    print('  PY32F0xx Chip Debug Dump')
    print(sep)

    # --- Bootloader ---
    print()
    print('--- Bootloader ---')
    print('  Version      : %s' % isp.verstr)
    print('  Chip PID     : 0x%04X%s' % (
        isp.pid,
        ' (PY32F0xx)' if isp.pid == PY_CHIP_PID else ' (unknown chip!)',
    ))

    # --- UID ---
    print()
    print('--- Unique Device ID (0x%08X) ---' % PY_UID_ADDR)
    try:
        uid_area = isp.readflash(PY_UID_ADDR, 128)
        uid_w0 = int.from_bytes(uid_area[0:4], 'little')
        uid_w1 = int.from_bytes(uid_area[4:8], 'little')
        uid_w2 = int.from_bytes(uid_area[8:12], 'little')
        print('  UID[31:0]    : 0x%08X' % uid_w0)
        print('  UID[63:32]   : 0x%08X' % uid_w1)
        print('  UID[95:64]   : 0x%08X' % uid_w2)
        print('  Full UID     : %08X-%08X-%08X' % (uid_w2, uid_w1, uid_w0))
        print()
        print('  UID area hex dump (128 bytes):')
        print(_hexdump(uid_area, PY_UID_ADDR))
    except Exception as ex:
        print('  ERROR: Could not read UID (%s)' % str(ex))

    # --- Option Bytes ---
    print()
    print('--- Option Bytes (0x%08X, 16 bytes) ---' % PY_OPTION_ADDR)
    try:
        ob = list(isp.readflash(PY_OPTION_ADDR, 16))
        print('  Raw hex:')
        print(_hexdump(ob, PY_OPTION_ADDR))

        rdp       = ob[0]
        user      = ob[1]
        n_rdp     = ob[2]
        n_user    = ob[3]
        sdk_strt  = ob[4]
        sdk_end   = ob[5]
        n_sdk_strt = ob[6]
        n_sdk_end  = ob[7]
        wrp       = (ob[12] << 8) | ob[13]
        n_wrp     = (ob[14] << 8) | ob[15]

        print()
        print('  OPTR (Read Protection + User Options):')
        print('    RDP          : 0x%02X → %s' % (rdp, _decode_rdp(rdp)))
        rdp_comp_ok = (rdp ^ n_rdp) == 0xFF
        print('    nRDP         : 0x%02X %s' % (n_rdp, '(valid)' if rdp_comp_ok else '(INVALID complement!)'))

        bor_en   = (user >> 0) & 1
        bor_lev  = (user >> 1) & 0x7
        iwdg_sw  = (user >> 4) & 1
        nrst_mode = (user >> 6) & 1
        nboot1   = (user >> 7) & 1
        print('    USER         : 0x%02X' % user)
        print('      BOR_EN     : %d (%s)' % (bor_en, 'enabled' if bor_en else 'disabled'))
        print('      BOR_LEV    : %d (%s rising threshold)' % (bor_lev, _decode_bor_level(bor_lev)))
        print('      IWDG_SW    : %d (%s)' % (iwdg_sw, 'software watchdog' if iwdg_sw else 'hardware watchdog'))
        print('      NRST_MODE  : %d (%s)' % (nrst_mode, 'GPIO pin' if nrst_mode else 'reset pin'))
        print('      nBOOT1     : %d' % nboot1)
        user_comp_ok = (user ^ n_user) == 0xFF
        print('    nUSER        : 0x%02X %s' % (n_user, '(valid)' if user_comp_ok else '(INVALID complement!)'))

        print()
        print('  SDK Region (protected code area):')
        print('    SDK_STRT     : 0x%02X (page %d → 0x%08X)' % (sdk_strt, sdk_strt & 0x1F, PY_FLASH_ADDR + (sdk_strt & 0x1F) * 0x1000))
        print('    SDK_END      : 0x%02X (page %d → 0x%08X)' % (sdk_end, sdk_end & 0x1F, PY_FLASH_ADDR + (sdk_end & 0x1F) * 0x1000))
        if (sdk_strt & 0x1F) > (sdk_end & 0x1F) or sdk_strt == 0xFF:
            print('    → SDK disabled (start > end or 0xFF)')
        else:
            sdk_size = ((sdk_end & 0x1F) - (sdk_strt & 0x1F) + 1) * 4
            print('    → SDK active: %d KB protected' % sdk_size)

        print()
        print('  Write Protection:')
        print('    WRP          : 0x%04X' % wrp)
        if wrp == 0xFFFF:
            print('    → No sectors write-protected')
        else:
            protected = [i for i in range(16) if not (wrp & (1 << i))]
            print('    → Protected sectors: %s' % ', '.join(str(s) for s in protected))
            print('    → Protected range: %d KB' % (len(protected) * 4))
        wrp_comp_ok = (wrp ^ n_wrp) == 0xFFFF
        print('    nWRP         : 0x%04X %s' % (n_wrp, '(valid)' if wrp_comp_ok else '(INVALID complement!)'))

    except Exception as ex:
        print('  ERROR: Could not read option bytes (%s)' % str(ex))
        print('  This usually means the chip is read-protected (locked).')

    # --- Config Area ---
    print()
    print('--- Config Area (0x%08X) ---' % PY_CONFIG_ADDR)
    try:
        config = isp.readflash(PY_CONFIG_ADDR, 128)
        print(_hexdump(config, PY_CONFIG_ADDR))
    except Exception as ex:
        print('  ERROR: Could not read config area (%s)' % str(ex))

    # --- Memory Map ---
    print()
    print('--- Memory Map ---')
    print('  Flash          : 0x%08X' % PY_FLASH_ADDR)
    print('  SRAM           : 0x%08X' % PY_SRAM_ADDR)
    print('  Bootloader     : 0x%08X' % PY_BOOT_ADDR)
    print('  UID            : 0x%08X' % PY_UID_ADDR)
    print('  Option Bytes   : 0x%08X' % PY_OPTION_ADDR)
    print('  Config         : 0x%08X' % PY_CONFIG_ADDR)
    print()
    print(sep)

# ===================================================================================
# Main Function
# ===================================================================================

def _main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Minimal command line interface for PY32 IAP')
    parser.add_argument('-u', '--unlock',   action='store_true', help='unlock chip (remove read protection)')
    parser.add_argument('-l', '--lock',     action='store_true', help='lock chip (set read protection)')
    parser.add_argument('-e', '--erase',    action='store_true', help='perform chip erase (implied with -f)')
    parser.add_argument('-o', '--rstoption',action='store_true', help='reset option bytes')
    parser.add_argument('-G', '--nrstgpio', action='store_true', help='make nRST pin a GPIO pin')
    parser.add_argument('-R', '--nrstreset',action='store_true', help='make nRST pin a RESET pin')
    parser.add_argument('-d', '--dump',     action='store_true', help='dump chip debug info (UID, option bytes, config)')
    parser.add_argument('-f', '--flash',    help='write BIN file to flash and verify')
    args = parser.parse_args(sys.argv[1:])

    # Check arguments
    if not any( (args.rstoption, args.unlock, args.lock, args.erase, args.nrstgpio, args.nrstreset, args.flash, args.dump) ):
        print('No arguments - no action!')
        sys.exit(0)

    # Establish connection to MCU via USB-to-serial converter
    try:
        print('Connecting to MCU via USB-to-serial converter ...')
        isp = Programmer()
        print('SUCCESS: Connection established via', isp.port + '.')
    except Exception as ex:
        sys.stderr.write('ERROR: ' + str(ex) + '!\n')
        sys.exit(1)

    # Performing actions
    try:
        # Get chip info
        print('Getting chip info ...')
        isp.readinfo()
        if isp.pid == PY_CHIP_PID:
            print('SUCCESS: Found PY32F0xx with bootloader v' + isp.verstr + '.')
        else:
            print('WARNING: Chip with PID 0x%04x is not a PY32F0xx!' % isp.pid)

        # Dump chip debug info
        if args.dump:
            _dump_chip_info(isp)
            isp.reset()
            print('DONE.')
            sys.exit(0)

        # Unlock chip
        if args.unlock:
            print('Unlocking chip ...')
            isp.unlock()
            print('SUCCESS: Chip is unlocked.')
            print('INFO: Other options are ignored!')
            isp.reset()
            print('DONE.')
            sys.exit(0)

        # Read option bytes and check, if chip is locked
        print('Reading OPTION bytes ...')
        isp.readoption()
        print('SUCCESS:', isp.optionstr + '.')

        # Perform chip erase
        if (args.erase) or (args.flash is not None):
            print('Performing chip erase ...')
            isp.erase()
            print('SUCCESS: Chip is erased.')

        # Flash binary file
        if args.flash is not None:
            print('Flashing', args.flash, 'to MCU ...')
            with open(args.flash, 'rb') as f: data = f.read()
            isp.writeflash(PY_CODE_ADDR, data)
            print('Verifying ...')
            isp.verifyflash(PY_CODE_ADDR, data)
            print('SUCCESS:', len(data), 'bytes written and verified.')

        # Manipulate OPTION bytes (only for identified chips)
        if isp.pid == PY_CHIP_PID and any( (args.rstoption, args.nrstgpio, args.nrstreset, args.lock) ):
            if args.rstoption:
                print('Setting OPTION bytes to default values ...')
                isp.resetoption()
            if args.nrstgpio:
                print('Setting nRST pin as GPIO in OPTION bytes ...')
                isp.nrst2gpio()
            if args.nrstreset:
                print('Setting nRST pin as RESET in OPTION bytes ...')
                isp.nrst2reset()
            if args.lock:
                print('Setting read protection in OPTION BYTES ...')
                isp.lock()
            print('Writing OPTION bytes ...')
            isp.writeoption()
            print('SUCCESS: OPTION bytes written.')
            isp.reset()
        else:
            isp.run()

    except Exception as ex:
        sys.stderr.write('ERROR: ' + str(ex) + '!\n')
        isp.reset()
        sys.exit(1)

    print('DONE.')
    sys.exit(0)

# ===================================================================================
# Programmer Class
# ===================================================================================

class Programmer(Serial):
    def __init__(self):
        # BAUD rate:  4800 - 1000000bps (default: 115200), will be auto-detected
        # Data frame: 1 start bit, 8 data bit, 1 parity bit set to even, 1 stop bit
        super().__init__(baudrate = PY_BAUD, parity = serial.PARITY_EVEN, timeout = 1)
        self.identify()

    # Identify port of programmer and enter programming mode
    def identify(self):
        for p in comports():
            if (('PY_VID' not in globals()) or (PY_VID in p.hwid)) and (('PY_PID' not in globals()) or (PY_PID in p.hwid)):
                self.port = p.device
                try:
                    self.open()
                except:
                    continue
                self.boot()
                self.reset_input_buffer()
                self.write([PY_SYNCH])
                if not self.checkreply():
                    self.close()
                    continue
                return
        raise Exception('No MCU in boot mode found')

    # Send command
    def sendcommand(self, command):
        self.write([command, command ^ 0xff])
        if not self.checkreply():
            raise Exception('Device has not acknowledged the command 0x%02x' % command)

    # Send address
    def sendaddress(self, addr):
        stream = addr.to_bytes(4, byteorder='big')
        parity = 0x00
        for x in range(4):
            parity ^= stream[x]
        self.write(stream)
        self.write([parity])
        if not self.checkreply():
            raise Exception('Failed to send address')

    # Check if device acknowledged
    def checkreply(self):
        reply = self.read(1)
        return (len(reply) == 1 and reply[0] == PY_REPLY_ACK)

    #--------------------------------------------------------------------------------

    # Start bootloader
    def boot(self):
        # 1. Pull BOOT0 (RTS) physically HIGH
        self.rts = False  # False often results in a High physical pin
        # 2. Pulse NRST (DTR) physically LOW
        self.dtr = True   # True often results in a Low physical pin
        time.sleep(0.1)
        # 3. Release Reset
        self.dtr = False
        time.sleep(0.1)

    # Reset and disconnect
    def reset(self):
        # 1. Pull BOOT0 (RTS) physically LOW
        self.rts = True
        # 2. Pulse NRST (DTR)
        self.dtr = True
        time.sleep(0.1)
        self.dtr = False
        self.close()
        
    # Start firmware and disconnect
    def run(self):
        self.sendcommand(PY_CMD_GO)
        self.sendaddress(PY_CODE_ADDR)
        self.dtr = True
        self.close()

    #--------------------------------------------------------------------------------

    # Read info stream
    def readinfostream(self, command):
        self.sendcommand(command)
        size = self.read(1)[0]
        stream = self.read(size + 1)
        if not self.checkreply():
            raise Exception('Failed to read info')
        return stream

    # Get chip info
    def readinfo(self):
        self.ver    = self.readinfostream(PY_CMD_GET)[0]
        self.verstr = '%x.%x' % (self.ver >> 4, self.ver & 7)
        self.pid    = int.from_bytes(self.readinfostream(PY_CMD_PID), byteorder='big')

    # Read UID
    def readuid(self):
        return self.readflash(PY_UID_ADDR, 128)

    # Read OPTION bytes
    def readoption(self):
        try:
            self.option = list(self.readflash(PY_OPTION_ADDR, 16))
        except:
            raise Exception('Chip is locked')
        self.optionstr = 'OPTR: 0x%04x, SDKR: 0x%04x, WRPR: 0x%04x' % \
                         (( (self.option[ 0] << 8) + self.option[ 1], \
                            (self.option[ 4] << 8) + self.option[ 5], \
                            (self.option[12] << 8) + self.option[13] ))

    # Write OPTION bytes
    def writeoption(self):
        self.writeflash(PY_OPTION_ADDR, self.option)

    # Reset OPTION bytes
    def resetoption(self):
        self.option = list(PY_OPTION_DEFAULT)

    # Set read protection in OPTION bytes
    def lock(self):
        self.option[0]  = 0x55
        self.option[2]  = 0xaa

    # Set nRST pin as GPIO in OPTION bytes
    def nrst2gpio(self):
        self.option[1] |= 0x40
        self.option[3] &= 0xbf

    # Set nRST pin as RESET in OPTION bytes
    def nrst2reset(self):
        self.option[1] &= 0xbf
        self.option[3] |= 0x40

    # Unlock (clear) chip and reset
    def unlock(self):
        self.sendcommand(PY_CMD_R_UNLOCK)
        if not self.checkreply():
            raise Exception('Failed to unlock chip')

    #--------------------------------------------------------------------------------

    # Erase whole chip
    def erase(self):
        self.sendcommand(PY_CMD_ERASE)
        self.write(b'\xff\xff\x00')
        if not self.checkreply():
            raise Exception('Failed to erase chip')

    # Read flash
    def readflash(self, addr, size):
        data = bytes()
        while size > 0:
            blocksize = size
            if blocksize > PY_BLOCKSIZE: blocksize = PY_BLOCKSIZE
            self.sendcommand(PY_CMD_READ)
            self.sendaddress(addr)
            self.sendcommand(blocksize - 1)
            data += self.read(blocksize)
            addr += blocksize
            size -= blocksize
        return data

    # Write flash
    def writeflash(self, addr, data):
        size = len(data)
        while size > 0:
            blocksize = size
            if blocksize > PY_BLOCKSIZE: blocksize = PY_BLOCKSIZE
            block = data[:blocksize]
            parity = blocksize - 1
            for x in range(blocksize):
                parity ^= block[x]
            self.sendcommand(PY_CMD_WRITE)
            self.sendaddress(addr)
            self.write([blocksize - 1])
            self.write(block)
            self.write([parity])
            if not self.checkreply():
                raise Exception('Failed to write to address 0x%08x' % addr)
            data  = data[blocksize:]
            addr += blocksize
            size -= blocksize

    # Verify flash
    def verifyflash(self, addr, data):
        flash = self.readflash(addr, len(data))
        if set(flash) != set(data):
            raise Exception('Verification failed')

# ===================================================================================
# Device Constants
# ===================================================================================

# Device and Memory constants
PY_CHIP_PID     = 0x440
PY_BLOCKSIZE    = 128
PY_FLASH_ADDR   = 0x08000000
PY_CODE_ADDR    = 0x08000000
PY_SRAM_ADDR    = 0x20000000
PY_BOOT_ADDR    = 0x1fff0000
PY_UID_ADDR     = 0x1fff0e00
PY_OPTION_ADDR  = 0x1fff0e80
PY_CONFIG_ADDR  = 0x1fff0f00

# Command codes
PY_CMD_GET      = 0x00
PY_CMD_VER      = 0x01
PY_CMD_PID      = 0x02
PY_CMD_COMMANDS = 0x03
PY_CMD_READ     = 0x11
PY_CMD_WRITE    = 0x31
PY_CMD_ERASE    = 0x44
PY_CMD_GO       = 0x21
PY_CMD_W_LOCK   = 0x63
PY_CMD_W_UNLOCK = 0x73
PY_CMD_R_LOCK   = 0x82
PY_CMD_R_UNLOCK = 0x92

# Reply codes
PY_REPLY_ACK    = 0x79
PY_REPLY_NACK   = 0x1f
PY_REPLY_BUSY   = 0xaa

# Other codes
PY_SYNCH        = 0x7f

# Default option bytes
PY_OPTION_DEFAULT = b'\xaa\xbe\x55\x41\xff\x00\x00\xff\xff\xff\xff\xff\xff\xff\x00\x00'

# ===================================================================================

if __name__ == "__main__":
    _main()