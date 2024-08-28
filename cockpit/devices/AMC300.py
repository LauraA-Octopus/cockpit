from cockpit import events
import cockpit.gui.guiUtils
import cockpit.gui.device
from cockpit.devices.microscopeDevice import MicroscopeBase
import cockpit.handlers.stagePositioner
import Pyro4
from cockpit.devices.device import Device
import threading
import cockpit.util.threads
from cockpit.util import valueLogger

import datetime
import operator
import time
import wx

DEFAULT_LIMITS = ((100, 100, 1000), (5000, 5000, 4900))
LOGGING_PERIOD = 30

class AMC300(MicroscopeBase, Device):
    def __init__(self, name, config):
        super().__init__(name, config)
        ## Connection to the XYZ stage controller (serial.Serial instance).
        self._proxy = Pyro4.Proxy(config.get('uri'))
        ## Lock around sending commands to the XYZ stage controller.
        self.xyzLock = threading.Lock()
        ## Cached copy of the stage's position.
        self.positionCache = (None, None)
        ## Target positions for movement in X, Y and Z.
        self.motionTargets = [None, None, None]
        ## Flag to show that sendPositionUpdates is running.
        self.sendingPositionUpdates = False
        ## Status dict updated by remote.
        self.status = {}
        ## Keys for status items that should be logged
        #self.logger = valueLogger.ValueLogger(name, keys=list(map('t_'.__add__, self._temperature_names)))
        try:
            xlim = self._proxy.get_value_limits('MotorSetpointX')
            ylim = self._proxy.get_value_limits('MotorSetpointY')
            zlim = self._proxy.get_value_limits('MotorSetpointZ')
        except:
            xlim, ylim, zlim = zip(*DEFAULT_LIMITS)
        # _proxy may return (0,0) if it can't query the hardware.
        if not any (xlim):
            xlim, _ = zip(*DEFAULT_LIMITS)
        if not any (ylim):
            _, ylim = zip(*DEFAULT_LIMITS)
        if not any (zlim):
            _, zlim = zip(*DEFAULT_LIMITS)
        self.hardlimits = tuple(zip(xlim, ylim, zlim))
        self.softlimits = self.hardlimits

        events.subscribe(events.USER_ABORT, self.onAbort)


    def finalizeInitialization(self):
        """Finalize device initialization."""
        self.statusThread = threading.Thread(target=self.pollStatus, name="AMC-status")
        events.subscribe(events.COCKPIT_INIT_COMPLETE, self.statusThread.start)


    def pollStatus(self):
        """Fetch the status from the remote and update the UI.

        Formerly, the remote periodically pushed status to a cockpit
        server. This caused frequent Pyro timeout errors when cockpit
        was busy doing other things.
        """
        lastTemps = [None]
        lastTime = 0

        while True:
            time.sleep(1)
            try:
                status = self._proxy.get_status()
            except Pyro4.errors.ConnectionClosedError:
                # Some dumb Pyro bug.
                continue

            if status.get('connected', False):
                self.status.update(status)
                self.sendPositionUpdates()
            else:
                self.status['connected'] = False
            self.updateUI()

    def initialize(self):
        """Initialize the device."""
        super().initialize()
        self.getPosition(shouldUseCache = False)
        self.updateSettings()


    def onAbort(self, *args):
        """Actions to do in the event of an abort."""
        pass


    def getHandlers(self):
        """Generate and return device handlers."""
        result = []
        # zip(*limits) transforms ((x0,y0),(x1,y1)) to ((x0,x1),(y0,y1))
        for axis, (minPos, maxPos) in enumerate(zip(*self.softlimits)):
            result.append(
                cockpit.handlers.stagePositioner.PositionerHandler(
                    "%d AMC300 mover" % axis, "%d stage motion" % axis, True,
                    {'moveAbsolute': self.moveAbsolute,
                         'moveRelative': self.moveRelative,
                         'getPosition': self.getPosition},
                    axis,
                    (minPos, maxPos), # hard limits
                    (minPos, maxPos) # soft limits
                    )
                )
        return result


    def makeUI(self, parent):
        """Make cockpit user interface elements."""
        ## A list of value displays for temperatures.
        # Panel, sizer and a device label.
        self.panel = wx.Panel(parent, style=wx.BORDER_RAISED)
        self.panel.SetDoubleBuffered(True)
        panel = self.panel
        panel.Sizer = wx.BoxSizer(wx.HORIZONTAL)
        left_sizer = wx.BoxSizer(wx.VERTICAL)
        right_sizer = wx.BoxSizer(wx.VERTICAL)
        panel.Sizer.Add(left_sizer)
        panel.Sizer.Add(right_sizer)

        self.elements = {}
        
        # Settings button
        adv_button = wx.Button(parent=self.panel, label='settings')
        adv_button.Bind(wx.EVT_LEFT_UP, self.showSettings)
        left_sizer.Add(adv_button, flag=wx.EXPAND)
        panel.Fit()
        return panel


    def moveAbsolute(self, axis, pos):
        """Move a stage axis to new position, pos."""
        pos = int(pos)
        if axis == 0:
            newPos = (pos, None, None)
        elif axis == 1:
            newPos = (None, pos, None)
        elif axis == 2:
            newPos = (None, None, pos)
        else:
            # Arguments were wrong. Just return, since raising an
            # exception can kill the mosaic.
            return
        with self.xyzLock:
            # moveToXYZ(x, y, z), where None indicates no change.
            self._proxy.move_to(*newPos)
        self.motionTargets[axis] = pos
        self.sendPositionUpdates()

    def moveRelative(self, axis, delta):
        """Move stage to a position relative to the current position."""
        if delta:
            curPos = self.positionCache[axis]
            self.moveAbsolute(axis, curPos + delta)


    @cockpit.util.threads.callInNewThread
    def sendPositionUpdates(self):
        """Send XYZ stage positions until it stops moving."""
        if self.sendingPositionUpdates is True:
            # Already sending updates.
            return
        self.sendingPositionUpdates = True
        moving = True
        # Send positions at least once.
        while moving:
            # Need this thread to sleep to give UI a chance to update.
            # Sleep at start of loop to allow stage time to respond to
            # move request so remote.isMoving() returns True.
            time.sleep(0.1)
            # Update position cache before publishing STAGE_MOVER so
            # that the UI gets the new position when it queries it.
            self.getPosition(shouldUseCache=False)
            for axis in [0, 1, 2]:
                events.publish(events.STAGE_MOVER, axis)
            moving = self._proxy.is_moving()

        for axis in (0, 1, 2):
            events.publish(events.STAGE_STOPPED, '%d AMC mover' % axis)
            self.motionTargets = [None, None, None]
        self.sendingPositionUpdates = False
        return


    def getPosition(self, axis=None, shouldUseCache=True):
        """Return the position of one or both axes.

        If axis is None, return positions of both axes.
        Query the hardware if shouldUseCache is False.
        """
        if not shouldUseCache:
            # Occasionally, at the start on an experiment, a
            # ConnectionClosedError is thrown here. I think this is something
            # to do with Pyro reusing connections and those connections getting
            # closed on the remote due to frequent traffic in the status thread.
            # Workaround: retry a few times in the event of a ConnectionClosedError.
            success = False
            failCount = 0
            while not success:
                try:
                    position = self._proxy.get_position()
                    success = True
                except Pyro4.errors.ConnectionClosedError:
                    if failCount < 5:
                        failCount += 1
                    else:
                        raise
                except:
                    raise
            self.positionCache = (position['X'], position['Y'], position['Z'])
        if axis is None:
            return self.positionCache
        else:
            return self.positionCache[axis]


    def updateUI(self):
        """Update user interface elements."""
        status = self.status
        if not status.get('connected', False):
            self.panel.Disable()
            return
        self.panel.Enable()

    def makeInitialPublications(self):
        """Send initial device publications."""
        self.sendPositionUpdates()