#!/usr/bin/python
#
# NXP LPC111x ISP flash programmer
#
# Copyright (c) 2012 Alexander Taradov <taradov@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# python lpc111xisp.py simplemesh.bin -p COM3 -e -d

import binascii
import optparse
import serial
import sys
import os

devInfo = {
  0x0444102b: ('LPC1114FHN33/301', 32768,  [4096]*8),
  0x0444502b: ('LPC1114FHN33/201', 32768,  [4096]*8),
  0x2540102b: ('LPC1114FHN33/302', 32768,  [4096]*8),
 	0x2540902b: ('LPC1114FHN33/202', 32768,  [4096]*8),
}

chunkSize = 1024 # 256 | 512 | 1024 | 4096
lineSize = 45
ramAddr = 0x10000200

def error(msg):
  print 'Error: %s' % msg
  sys.exit(-1)

class CheckFailed(Exception):
  pass

def check(cond, msg = None):
  if not cond:
    if msg:
      error(msg)
    else:
      raise CheckFailed

def str2list(s):
  return [ord(c) for c in s]

def list2str(l):
  return ''.join([chr(i) for i in l])

def read(port):
  data = ''
  while True:
    t = port.read()
    if not t:
      return None
    if t == '\r':
      port.read()
      if options.debug:
        print '<- %s' % data
      return data
    data += t

def write(port, data):
  port.write(data + '\r\n')
  if options.debug:
    print '-> %s' % data
  check(read(port) == data)

def sectorFromAddr(addr):
  aAddr = 0
  for i in range(len(sectors)):
    aAddr += sectors[i]
    if aAddr > addr:
      return i
  return -1

def eraseFlash(port):
  write(port, 'P %d %d' % (0, len(sectors)-1))
  check(read(port) == '0')
  write(port, 'E %d %d' % (0, len(sectors)-1))
  check(read(port) == '0')

def updateChecksum(data):
  data = [ord(d) for d in data]

  cs = 0
  for i in range(7):
    cs += (data[i*4 + 3] << 24) | (data[i*4 + 2] << 16) | (data[i*4 + 1] << 8) | data[i*4 + 0]

  data[7*4 + 3] = ((-cs) >> 24) & 0xff
  data[7*4 + 2] = ((-cs) >> 16) & 0xff
  data[7*4 + 1] = ((-cs) >> 8) & 0xff
  data[7*4 + 0] = (-cs) & 0xff

  return ''.join([chr(d) for d in data])

def writeFlash(port):
  f = open(imageName, 'rb')

  dstAddr = 0
  data = f.read(chunkSize)
  data = updateChecksum(data)
  while len(data):
    sys.stdout.write('.')
    sys.stdout.flush()
    # Number of bytes to be written. Count should be a multiple of 4.
    while (len(data) % lineSize != 0) or (len(data) % 4 != 0):
      data += chr(0)
    linesNo = len(data) / lineSize

    # Write image to RAM
    write(port, 'W %d %d' % (ramAddr, len(data)))
    check(read(port) == '0')

    cLine = 0
    cSum = 0
    uu = []
    while data:
      line, data = data[0:45], data[45:]
      cSum += sum(str2list(line))
      uu += [binascii.b2a_uu(line).replace(' ', '`').strip()]
      cLine += 1
      if cLine % 20 != 0 and cLine != linesNo:
        continue

      retries = 3
      while True:
        for line in uu:
          write(port, line)
        write(port, str(cSum))
        resp = read(port)
        if resp == 'OK':
          break
        elif resp == 'RESEND':
          retries -= 1
          if retries == 0:
            print 'Error: too many checksum errors while sending data'
            check(0)
        else:
          print 'Error: no response from host'
          check(0)

      uu = []
      cSum = 0

    # Now image is in RAM. Write it to flash.
    sect = sectorFromAddr(dstAddr)
    write(port, 'P %d %d' % (sect, sect))
    check(read(port) == '0')
    write(port, 'C %d %d %d' % (dstAddr, ramAddr, chunkSize))
    check(read(port) == '0')

    # Flash is written.
    dstAddr += chunkSize
    data = f.read(chunkSize)
  f.close()
  print ''

#
# Main program starts here
#
parser = optparse.OptionParser(usage = 'usage: %prog [options] file')
parser.add_option('-d', '--debug', dest = 'debug', help = 'enable debug output', default = False, action = 'store_true')
parser.add_option('-p', '--port',  dest = 'port',  help = 'communication port [default /dev/ttyUSB0]', default = '/dev/ttyUSB0', metavar = "PORT")
parser.add_option('-b', '--baud',  dest = 'baud',  help = 'communication baudrate [default 38400]', default = 38400, metavar = "BAUD")
parser.add_option('-f', '--freq',  dest = 'freq',  help = 'crystal frequency (in kHz) [default 12000]', default = 12000, metavar = "FREQ")
parser.add_option('-e', '--erase', dest = 'erase', help = 'erase flash before programming', default = False, action = 'store_true')
(options, args) = parser.parse_args()

if len(args) != 1:
  error('invalid image file name')

imageName = args[0]
try:
  fileSize = os.path.getsize(imageName)
except OSError, msg:
  error(msg)

port = serial.Serial(options.port, options.baud, xonxoff = 0, timeout = 1)

try:
  port.setRTS(1)

  while True:
    port.setDTR(1)
    port.setDTR(0)

    sys.stdout.write('.')
    sys.stdout.flush()
    try:
      port.write('?')
      check(read(port) == 'Synchronized')

      write(port, 'Synchronized')
      check(read(port) == 'OK')

      write(port, str(options.freq))
      check(read(port) == 'OK')
      break
    except CheckFailed:
      pass

  port.setRTS(0)
  print ''

  write(port, 'J')
  check(read(port) == '0')
  devId = int(read(port))
  if devId in devInfo:
    if options.debug:
      print '%s found' % devInfo[devId][0]
  else:
    error('unknown device (id = %08x)' % devId)

  if fileSize > devInfo[devId][1]:
    error('image too big for this device')
  sectors = devInfo[devId][2]

  write(port, 'U 23130')
  check(read(port) == '0')

  if options.erase:
    eraseFlash(port)

  writeFlash(port)

  write(port, 'G 0 A')
  check(read(port) == '0')

except CheckFailed:
  print 'Stopping'

except KeyboardInterrupt:
  print 'Exit'

port.setDTR(1)
port.setDTR(0)
port.close()
