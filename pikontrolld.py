#!/usr/bin/env python
# pikontroll daemon
# using information from http://yoestuve.es/blog/communications-between-python-and-stellarium-stellarium-telescope-protocol/
# jes 2018

import math
import logging
import asyncore, socket
import serial
from time import sleep, time
from string import replace
from bitstring import BitArray, BitStream, ConstBitStream
import coords
import altazimuth
import re
 
logging.basicConfig(level=logging.DEBUG, format="%(filename)s: %(funcName)s - %(levelname)s: %(message)s")

class PikonMotor:
    # ser is the serial port to write to, n is the motor index
    def __init__(self, ser, n, stepsper360):
        self.ser = ser
        self.n = n
        self.stepsper360 = stepsper360
        self.trim = 0

    def target(self, degrees):
        logging.debug("Motor target %f\n" % (degrees))
        steps = float(degrees) / 360.0 * float(self.stepsper360) + self.trim
        (cursteps,_,_,_,_,_) = self.read()
        halfturn = self.stepsper360/2
        while steps > cursteps+halfturn:
            steps -= self.stepsper360
        while steps < cursteps-halfturn:
            steps += self.stepsper360
        logging.debug(">>> target %d %d\n" % (self.n, steps))
        self.ser.write("target %d %d\n" % (self.n, steps))

    # return (pos, dir, last, target, havetarget, ok)
    # where:
    #   pos is the current step count
    #   dir is the current motion direction
    #   last is the number of ms since the last step
    #   target is the current target
    #   havetarget is 1 if the motor is aiming for the target and 0 if it's ignoring it
    #   ok is 1 if the last "test" was a success, and 0 if not, or if there was no test
    # blocks until a response is received over serial (or until the serial timeout is reached and then it probably fails)
    def read(self):
        self.ser.write("read %d\n" % (self.n))
        resp = self.ser.readline()
        regex = re.compile("motor \d+: pos=(-?\d+) dir=(-?\d+) last=(-?\d+) target=(-?\d+) havetarget=([01]) ok=([01])")
        m = regex.match(resp)
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5)), int(m.group(6)))

    # return current position in degrees
    # (blocks on serial)
    def currentpos(self):
        (pos,_,_,_,_,_) = self.read()
        degrees = float(pos - self.trim) * 360.0 / float(self.stepsper360)
        while degrees < 0:
            degrees += 360
        while degrees > 360:
            degrees -= 360
        logging.debug("Current degrees on motor %d is %f" % (self.n, degrees))
        return degrees
 
## \brief Implementation of the server side connection for 'Stellarium Telescope Protocol'
#
#  Manages the execution thread to the server side connection with Stellarium
class Telescope_Channel(asyncore.dispatcher):
 
    ## Class constructor
    #
    # \param conn_sock Connection socket
    def __init__(self, conn_sock, server):
        self.server = server
        self.last_update = time()
        self.have_target = False
        asyncore.dispatcher.__init__(self, conn_sock)
 
    ## Indicates the socket is readable
    #
    # \return Boolean True/False
    def readable(self):
        return True
 
    ## Indicates the socket is writable
    #
    # \return Boolean True/False
    def writable(self):
        # we want to write if it's more than 0.1 second since the last update
        return (time() - self.last_update) > 0.1
 
    ## Close connection handler
    #
    def handle_close(self):
        logging.debug("Disconnected")
        self.close()
 
    ## Reading socket handler
    #    
    # Reads and processes client data, and throws the proper signal with coordinates as parameters
    def handle_read(self):
        #format: 20 bytes in total. Size: intle:16
        #Incomming messages comes with 160 bytes..
        data0 = self.recv(160);
        if data0:            
            data = ConstBitStream(bytes=data0, length=160)
            #print "All: %s" % data.bin
 
            msize = data.read('intle:16')
            mtype = data.read('intle:16')
            mtime = data.read('intle:64')
 
            # RA: 
            ant_pos = data.bitpos
            ra = data.read('hex:32')
            data.bitpos = ant_pos
            ra_uint = data.read('uintle:32')
 
            # DEC:
            ant_pos = data.bitpos
            dec = data.read('hex:32')
            data.bitpos = ant_pos
            dec_int = data.read('intle:32')

 
            logging.debug("Size: %d, Type: %d, Time: %d, RA: %d (%s), DEC: %d (%s)" % (msize, mtype, mtime, ra_uint, ra, dec_int, dec))
            (sra, sdec, stime) = coords.eCoords2str(float("%f" % ra_uint), float("%f" % dec_int), float("%f" %  mtime))

            dec_degrees = dec_int * float(90.0)/float(1073741824)

            self.target = (coords.rad_2_hour(coords.hourStr_2_rad(sra)), dec_degrees)
            self.have_target = True
            self.drive_to_target()

    def drive_to_target(self):
        if not self.have_target:
            return
        (ra, dec) = self.target
        (azdegrees, altdegrees) = altazimuth.ComputeAltAzimuth(altazimuth.GetNow(), -2.50029, 51.45528, ra, dec)
        logging.debug("TARGET ANGLES: ALTITUDE = %f, AZIMUTH = %f" % (altdegrees, azdegrees))
        self.server.target(altdegrees, azdegrees)

    ## Updates the field of view indicator in Stellarium
    #
    # \param ra Right ascension in signed string format
    # \param dec Declination in signed string format
    def act_pos(self, ra, dec):
        (ra_p, dec_p) = coords.rad_2_stellarium_protocol(ra, dec)
 
        times = 10 #Number of times that Stellarium expects to receive new coords //Absolutly empiric..
        for i in range(times):
            self.move(ra_p, dec_p)

    # read current position from motors and send it to stellarium
    def send_current_pos(self):
        (altdegrees, azdegrees) = self.server.currentpos()
        (ra_hours, dec_degrees) = altazimuth.ComputeRaDec(altazimuth.GetNow(), -2.50029, 51.45528, azdegrees, altdegrees)
        logging.debug("Ra_hours = %f, Dec_degrees = %f" % (ra_hours, dec_degrees))
        ra_rad = round((ra_hours * 15 * math.pi) / 180, 6)
        dec_rad = dec_degrees * math.pi / 180
        (ra, dec) = coords.rad_2_stellarium_protocol(ra_rad, dec_rad)
        for i in range(10):
            self.move(ra, dec)
        logging.debug("Alt = %f, Az = %f\n" % (altdegrees, azdegrees))
 
    ## Sends to Stellarium equatorial coordinates
    #
    #  Receives the coordinates in float format. Obtains the timestamp from local time
    #
    # \param ra right ascension
    # \param dec declination
    def move(self, ra, dec):
        msize = '0x1800'
        mtype = '0x0000'
        localtime = ConstBitStream(replace('int:64=%r' % time(), '.', ''))
        #print "move: (%d, %d)" % (ra, dec)
 
        sdata = ConstBitStream(msize) + ConstBitStream(mtype)
        sdata += ConstBitStream(intle=localtime.intle, length=64) + ConstBitStream(uintle=ra, length=32)
        sdata += ConstBitStream(intle=dec, length=32) + ConstBitStream(intle=0, length=32)
        buffer = sdata
        self.send(buffer.bytes)
 
    ## Transmission handler
    #
    def handle_write(self):
        logging.debug("Handle write")
        self.drive_to_target()
        self.send_current_pos()
        self.last_update = time()
 
## \brief Implementation of the server side communications for 'Stellarium Telescope Protocol'.
#
#  Each connection request generate an independent execution thread as instance of Telescope_Channel
class Telescope_Server(asyncore.dispatcher):
 
    ## Class constructor
    #
    # \param port Port to listen on
    def __init__(self, port=10001):
        asyncore.dispatcher.__init__(self, None)
        self.tel = None
        self.port = port
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=10)
        except:
            self.ser = serial.Serial("/dev/ttyUSB1", 9600, timeout=10)
        self.azMotor = PikonMotor(self.ser, 0, 294912)
        self.altMotor = PikonMotor(self.ser, 1, 314572.8)
 
    ## Starts thread
    #
    # Sets the socket to listen on
    def run(self):
        logging.info(self.__class__.__name__+" running.")
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind(('0.0.0.0', self.port))
        self.listen(1)
        self.connected = False
        while True:
            asyncore.loop(0.1)

    def target(self, altdegrees, azdegrees):
        logging.debug("Target %f, %f" % (altdegrees, azdegrees))
        self.altMotor.target(altdegrees)
        self.azMotor.target(azdegrees)
 
    # return (altdegrees, azdegrees)
    def currentpos(self):
        return (self.altMotor.currentpos(), self.azMotor.currentpos())

    ## Handles incomming connection
    #
    # Stats a new thread as Telescope_Channel instance, passing it the opened socket as parameter
    def handle_accept(self):
        self.conn, self.addr = self.accept()
        logging.debug('%s Connected', self.addr)
        self.connected = True
        self.tel = Telescope_Channel(self.conn, self)
 
    ## Closes the connection
    #
    def close_socket(self):
        if self.connected:
            self.conn.close()
 
#Run a Telescope Server
if __name__ == '__main__':
    try:
        Server = Telescope_Server()
        Server.run()
    except KeyboardInterrupt:
        logging.debug("\nBye!")
