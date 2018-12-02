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
        self.port = port
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=10)
        except:
            self.ser = serial.Serial("/dev/ttyUSB1", 9600, timeout=10)
        # consume any output from the uno
        while True:
            l = self.ser.readline()
            if l == "Ready.\r\n" or l == "":
                break
        self.azMotor = PikonMotor(self.ser, 0, 294912)
        self.altMotor = PikonMotor(self.ser, 1, 314572.8)
        self.target = (0, 0)
        self.pikon = Pikontroll_Server(self)
 
    ## Starts thread
    #
    # Sets the socket to listen on
    def run(self):
        logging.info(self.__class__.__name__+" running.")
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind(('0.0.0.0', self.port))
        self.listen(1)
        self.pikon.begin()
        while True:
            asyncore.loop(0.1)

    def target(self, altdegrees, azdegrees):
        logging.debug("Target %f, %f" % (altdegrees, azdegrees))
        self.altMotor.target(altdegrees)
        self.azMotor.target(azdegrees)
        self.target = (altdegrees, azdegrees)

    def set_trim(self, motor, steps):
        if motor == 0:
            self.azMotor.trim = steps
            self.azMotor.target(self.target[1])
        else:
            self.altMotor.trim = steps
            self.azMotor.target(self.target[0])

    # azMotor is motor 0, altMotor is motor 1
    def get_trim(self):
        return (self.azMotor.trim, self.altMotor.trim)

    def set_focus(self, focus):
        self.ser.write("servo %d\n" % (focus))

    def get_focus(self):
        self.ser.write("readservo\n")
        resp = self.ser.readline()
        logging.debug("resp = " + resp)
        regex = re.compile("servo: us=(-?\d+) min=(-?\d+) max=(-?\d+)")
        m = regex.match(resp)
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)))
 
    # return (altdegrees, azdegrees)
    def currentpos(self):
        return (self.altMotor.currentpos(), self.azMotor.currentpos())

    ## Handles incomming connection
    #
    # Stats a new thread as Telescope_Channel instance, passing it the opened socket as parameter
    def handle_accept(self):
        conn, addr = self.accept()
        logging.debug('%s Connected', addr)
        Telescope_Channel(conn, self)

class Pikontroll_Channel(asyncore.dispatcher):
    def __init__(self, conn, server):
        self.server = server
        self.buf = ''
        asyncore.dispatcher.__init__(self, conn)

    def readable(self):
        return True

    def writable(self):
        return False

    def handle_read(self):
        data = self.recv(1024)
        if not data:
            return

        self.buf += data
        lines = self.buf.split("\n")
        # leave the last partial line in self.buf, and process the complete lines
        self.buf = lines[-1]
        lines = lines[:-1]

        for l in lines:
            self.process(l)

    def handle_write(self):
        return None

    def handle_close(self):
        logging.debug("Pikontroll channel disconnected")
        self.close()

    def process(self, line):
        params = line.split(" ")

        if params[0] == "trim":
            self.cmd_trim(params)
        elif params[0] == "focus":
            self.cmd_focus(params)
        elif params[0] == "coords":
            self.cmd_coords(params)
        elif params[0] == "help":
            self.cmd_help(params)
        else:
            self.send("error: don't recognise '" + params[0] + "'\n")

    def cmd_trim(self, params):
        if len(params) < 2 or len(params) > 3:
            self.send("error: usage: trim M [N]\n")
            return

        motor = int(params[1])
        if motor < 0 or motor > 1:
            self.send("error: motor should be either 0 or 1\n")
            return

        if len(params) == 3:
            trim = int(params[2])
            self.server.telescope_server.set_trim(motor, trim)

        steps = self.server.telescope_server.get_trim()[motor]
        self.send("ok: trim %d %d\n" % (motor, steps))

    def cmd_focus(self, params):
        if len(params) < 1 or len(params) > 2:
            self.send("error: usage: focus [N]\n")
            return

        if len(params) == 2:
            focus = int(params[1])
            self.server.telescope_server.set_focus(focus)

        (focus,focusmin,focusmax) = self.server.telescope_server.get_focus()
        self.send("ok: focus %d in %d to %d\n" % (focus, focusmin, focusmax))

    def cmd_coords(self, params):
        (altdegrees, azdegrees) = self.server.telescope_server.currentpos()
        self.send("ok: coords %f %f\n" % (altdegrees, azdegrees))

    def cmd_help(self, params):
        self.send("commands:\n"
            "trim M    - report trim for motor N (0,1)\n"
            "trim M N  - set trim for motor M (0,1) to N steps\n"
            "focus     - report focus setting\n"
            "focus N   - set focus setting to N\n"
            "coords    - report current target coordinates\n"
            "help      - show this help\n")

class Pikontroll_Server(asyncore.dispatcher):
    def __init__(self, telescope_server, port=10002):
        asyncore.dispatcher.__init__(self, None)
        self.telescope_server = telescope_server
        self.port = port

    def begin(self):
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind(('0.0.0.0', self.port))
        self.listen(1)

    def handle_accept(self):
        conn, addr = self.accept()
        logging.debug('%s Connected to pikontroll port', addr)
        Pikontroll_Channel(conn, self)
 
#Run a Telescope Server
if __name__ == '__main__':
    try:
        Server = Telescope_Server()
        Server.run()
    except KeyboardInterrupt:
        logging.debug("\nBye!")
