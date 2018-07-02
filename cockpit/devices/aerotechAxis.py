#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2018 Mick Phillips <mick.phillips@gmail.com>
## Copyright (C) 2018 Ian Dobbie <ian.dobbie@bioch.ox.ac.uk>
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.


"""Cockpit aerotechAxis

This module creates a simple stage-positioning device.

Sample config entry:
  [aerotech]
  type: AerotechZStage
  ipAddress: 192.168.0.5
  port: 8000
  softlimits: -45000, 5000


"""

from . import device
from cockpit import events
import cockpit.handlers.stagePositioner
import socket
import re
from time import sleep

NAME_STRING = 'aerotech mover'
LIMITS_PAT = r"(?P<limits>\(?\s*[-]?\d*\s*,\s*[-]?\d*\s*\)?)"

## Characters used by the SoloistCP controller.
# CommandTerminatingCharacter
CTC = 10
# CommandSuccessCharacter
CSC = 37
# CommandInvalidCharacter
CIC = 33
# CommandFaultCharacter
CFC = 35
# CommandTimeOutCharacter
CTOC = 36
# Map character to meaning
RESPONSE_CHARS = {
    CSC: b'command success',
    CIC: b'command invalid',
    CFC: b'command caused fault',
    CTOC: b'command timeout',
    }


class AerotechZStage(device.Device):
    def __init__(self, name, config={}):
        device.Device.__init__(self, name, config)
        try :
            limitString = config.get('softlimits', '')
            parsed = re.search(LIMITS_PAT, limitString)
            if not parsed:
                # Could not parse config entry.
                raise Exception('Bad config: Aerotech Limits.')
                # No transform tuple
            else:
                lstr = parsed.groupdict()['limits']
                self.softlimits=eval(lstr)
        except:
            print ("No softlimits section setting default limits")
            self.softlimits = (-30000,7000)

        # Subscribe to abort events.
        events.subscribe('user abort', self.onAbort)
        # The cockpit axis does this stage moves along.
        self.axis = int(config.get('axis', 2))
        # Socket used to communicate with controller.
        self.socket = None
        # Last known position (microns)
        self.position = None
        # Axis acceleration (mm / s^2)
        self.acceleration = 200
        # Axis maximum speed (mm / s^2)
        self.speed = 20

    
    ## Send a command to the Aerotech SoloistCP and fetch the response.
    def command(self, cmd):
        # The terminated command string.
        cmdstr = cmd + bytes([CTC])
        # A flag indicating that a retry is needed.
        retry = False

        ## Try to send the command.
        try:
            self.socket.send(cmdstr)
        except socket.error as e:
            retry = True
        except:
            raise

        ## Was there a connection error?
        if retry:
            self.openConnection()
            try:
                self.socket.send(cmdstr)
            except:
                raise

        ## Fetch and parse the response
        response = self.socket.recv(256)
        # Response status character:
        response_char = response[0]
        if response_char == 255:
            # stray character
            response_char = response[1]
            response_data = response[2:]
        else:
            response_data = response[1:]
        
        # Did the controlle report a problem?
        if response_char != CSC and response in RESPONSE_CHARS:
            raise Exception('Aerotech controller error - %s.' 
                            % RESPONSE_CHARS[response_char].decode())
        elif response_char != CSC:
            raise Exception('Aerotech controller error - unknown response %s' % chr(response_char))
        else:
            return response_data

       
    def getHandlers(self):
        result = []
        axis = self.axis
        #IMD 2015-03-02 changed hard limits to reflect DeepSIM should go into config file
        minVal = self.softlimits[0]
        maxVal = self.softlimits[1]
        handler = cockpit.handlers.stagePositioner.PositionerHandler(
            "%d %s" % (axis, NAME_STRING), "%d stage motion" % axis, False, 
            {'moveAbsolute': self.moveAbsolute,
                'moveRelative': self.moveRelative, 
                'getPosition': self.getPosition, 
                'getMovementTime': self.getMovementTime,
                'cleanupAfterExperiment': self.cleanup,
                'setSafety': self.setSafety},
                axis, [10, 50, 100, 500, 1000, 5000],
                1, (minVal, maxVal), (minVal, maxVal))
        result.append(handler)
        return result
    
    
    ## Initialize the axis.
    def initialize(self):
        # Open a connection to the controller.
        self.openConnection()
        self.position = self.command(b'CMDPOS').decode()
        self.speed = int(self.command(b'GETPARM(71)'))
        self.acceleration = int(self.command(b'GETPARM(72'))


    ## Publish our current position.
    def makeInitialPublications(self):
        axis = self.axis
        events.publish('stage mover', '%d %s' % (axis, NAME_STRING), axis,
                self.position)


    ## User clicked the abort button; stop moving.
    def onAbort(self):
        self.command(b'ABORT')
        axis = self.axis
        events.publish('stage stopped', '%d %s' % (axis, NAME_STRING))


    ## Move the stage to a given position.
    def moveAbsolute(self, axis, pos):
        self.command(b'ENABLE')
        self.command(b'MOVEABS D %f F %f'
                        % (pos / 1000.0, self.speed))
        events.publish('stage mover', '%d %s' % (axis, NAME_STRING), axis, self.position)
        # Wait until the move has finished - status bit 2 is InPosition.
        while not int(self.command(b'AXISSTATUS')) & (1 << 2):
            sleep(0.1)
        self.position = self.command(b'CMDPOS').decode()
        events.publish('stage mover', '%d %s' % (axis, NAME_STRING), axis, self.position)
        events.publish('stage stopped', '%d mover' % axis)
        self.command (b'DISABLE')


    ## Move the stage piezo by a given delta.
    def moveRelative(self, axis, delta):
        self.command(b'ENABLE')
        self.command(b'MOVEINC D %f F %f'
                        % (delta / 1000.0, self.speed))
        events.publish('stage mover', '%d %s' % (axis, NAME_STRING), axis, self.position)
        # Wait until the move has finished - status bit 2 is InPosition.
        while not int(self.command(b'AXISSTATUS')) & (1 << 2):
            sleep(0.1)
        self.position = self.command(b'CMDPOS').decode()
        events.publish('stage mover', '%d %s' % (axis, NAME_STRING), axis, self.position)
        events.publish('stage stopped', '%d %s' % (axis, NAME_STRING))
        self.command (b'DISABLE')


    ## Get the current piezo position.
    def getPosition(self, axis):
        return float(self.position)*1000.0


    ## Get the amount of time it would take the mover to move from the 
    # initial position to the final position, as well
    # as the amount of time needed to stabilize after that point, 
    # both in milliseconds. This is needed when setting up timings for 
    # experiments.
    def getMovementTime(self, axis, start, end):
        # top speed mm/s
        v = self.speed
        # acceleration mm/s**2
        a = self.acceleration
        # displacement - passed as um
        dS = abs(start - end) / 1000.0
        # acceleration / braking distance
        S_acc = 0.5 * v**2 / a
        
        # Determine time required for move, in ms.
        if dS < 2 * S_acc:
            dt = 1000 * 2 * (dS / a)**0.5
        else:
            dt = 1000 * ((dS / v) + (v / a))

        # Add a few ms to allow for any network latency (measured at < 10ms).
        dt += 10

        # The stage slows gradually to a stop so settling time is small.
        # Allow one or two servo cycles - servo rate is 1kHz.
        return (dt, 2)


    def openConnection(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.ipAddress, self.port))
        except:
            raise


    ## Set the soft motion safeties for one of the movers. Note that the 
    # PositionerHandler provides its own soft safeties on the cockpit side; 
    # this function just allows you to propagate safeties to device control
    # code, if applicable.
    def setSafety(self, axis, value, isMax):
        pass


    ## Cleanup after an experiment. For a real mover, this would probably 
    # just mean making sure we were back where we were before the experiment
    # started.
    def cleanup(self, axis, isCleanupFinal):
        self.moveAbsolute(axis, self.position)
