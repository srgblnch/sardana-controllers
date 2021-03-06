#!/usr/bin/env python2.5

#############################################################################
##
## file :    PmacLTPIOCtrl.py
##
## description : 
##
## project :    Sardana-Controllers/IORegisterController
##
## developers history: zreszela, sblanch
##
## copyleft :    Cells / Alba Synchrotron
##               Bellaterra
##               Spain
##
#############################################################################
##
## This file is part of Sardana.
##
## This is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
##
## This software is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
###########################################################################
import copy, logging

import PyTango
from sardana import pool
from sardana.pool import PoolUtil
from sardana.pool.controller import IORegisterController

def _to_binary(number, bits):
    str_binary_number = ''
    for pos in range(bits):
        index = bits - pos - 1
        shifted_number = number / pow(2,index)
        bit = '0'
        if shifted_number & 1:
            bit = '1'
        str_binary_number = str_binary_number+bit
    return str_binary_number

class PmacLTPIOController(IORegisterController):
    """ This controller provides the state of the air supply and the air pads of pmac axes ltp to the pool.
    +) first axis is the state of air supply
    +) second axis is the state of air pads for top axis
       -)   0: top axis lifted
       -)  63: top axis landed
    +) third axis is the state of air pads for bottom axis
       -) 0: bottom axis lifted
       -) 3: bottom axis landed
    """
    
    gender = ""
    model  = ""
    organization = "CELLS - ALBA"
    image = ""
    icon = ""
    logo = "ALBA_logo.png"

    # The PMAC Device Server to get the info
    ctrl_properties = {'PmacDevName':{'Description':'The Pmac Tango DS',
                                      'Type':str}
                      }
    
    MaxDevice = 3

    axis_attributes = {"Labels":{'Type':str,'Access':'ReadWrite'}}
    
    def __init__(self, inst, props, *args, **kwargs):
        IORegisterController.__init__(self, inst, props, *args, **kwargs)
        self.pmacEth = PoolUtil().get_device(self.inst_name, self.PmacDevName)

    def AddDevice(self, axis):
        self._log.debug('AddDevice %d' % axis)

    def DeleteDevice(self, axis):
        self._log.debug('DeleteDevice %d' % axis)

    def StateOne(self, axis):
        try:
            value = self.ReadOne(axis)
            bits = 1
            if axis == 2: bits = 6
            elif axis == 3: bits = 2 
            str_bin_value = _to_binary(value, bits)
        except Exception, e:
            self._log.error(str(e))
            if axis == 1: 
                value = '?'
                str_bin_value = '?'
            elif axis == 2: 
                value = '??'
                str_bin_value = '??????'
            elif axis == 3:
                value == "?"
                str_bin_value = '?' 

        # THE STATE CAN BE ON (everything OK), OR ALARM (not all OK)
        state = PyTango.DevState.ON
        status = 'Hardware value: %s.' % str_bin_value
        if axis == 1 and value != 1:
            state = PyTango.DevState.ALARM
            status += ' The air supply is NOT OK'
        elif axis == 2 and value != 0:
            state = PyTango.DevState.ALARM
            status += ' Some air pads are not lifted.'
        elif axis == 3 and value != 0:
            state = PyTango.DevState.ALARM
            status += ' Some air pads are not lifted.'
        return (state, status)

    def ReadOne(self, axis):
	#cast to int cause PmacEthDS returns DevDouble in GetMVariable command
        value = int(self.pmacEth.command_inout("getmvariable",100))
        if axis == 1:
            value &= 1
        elif axis == 2:
            value >>= 8
            value &= 63
        elif axis == 3:
            value >>= 14
            value &= 3
        return value

    def WriteOne(self, axis, value):
        if axis == 1:
            PyTango.Except.throw_exception("I/O error",
                                           "Axis nr 1 is read-only",
                                           "PmacLTPIOCtrl.WriteOne()")
        if axis == 2:
            current_3axis = self.ReadOne(3)
            write_value = value + (current_3axis << 6)
        if axis == 3:
            current_2axis = self.ReadOne(2)
            write_value = (value << 6) + current_2axis
        self.pmacEth.command_inout("setmvariable", [101, write_value])

    def GetAxisExtraPar(self, axis, name):
        if name.lower() == 'labels':
            if axis == 1: return "closed:0 opened:1"
            elif axis == 2: return "lifted:0 landed:63"
            elif axis == 3: return "lifted:0 landed:3"
            else: PyTango.Except.throw_exception("PmacLTPIOCtrl GetExtraAttributePar() exception",
                                           "%d is not allowed axis nr" % axis,
                                           "PmacLTPIOCtrl.GetExtraAttributePar()")

    def SetAxisExtraPar(self,axis, name, value):
        pass
        
    def SendToCtrl(self,in_data):
        return "Not implemented yet"
    
class SimuPmacLTPIOController(IORegisterController):
    """ This controller provides the state of the air supply and the airpads to the pool.
    +) first axis is the state of air supply
    +) second axis is the state of airpads for top axis
       -)   0: top axis lifted
       -)  63: top axis landed
    +) third axis is the state of airpads for bottom axis
       -) 0: bottom axis lifted
       -) 3: bottom axis landed
    """
    
    gender = ""
    model  = ""
    organization = "CELLS - ALBA"
    image = ""
    icon = ""
    logo = "ALBA_logo.png"
    
    MaxDevice = 3

    axis_attributes = {"Labels":{'Type':str,'Access':'ReadWrite'},}
                                      
    def __init__(self, inst, props, *args, **kwargs):
        IORegisterController.__init__(self, inst, props, *args, **kwargs)
        self._log.setLogLevel(logging.DEBUG)
        self._log.debug('In SimuPmacLTPIOController.__init__()')
        self.array = 1
        
    def AddDevice(self, axis):
        self._log.debug('AddDevice %d' % axis)

    def DeleteDevice(self, axis):
        self._log.debug('DeleteDevice %d' % axis)

    def StateOne(self, axis):
        try:
            value = self.ReadOne(axis)
            bits = 1
            if axis == 2: bits = 6
            elif axis == 3: bits = 2 
            str_bin_value = _to_binary(value, bits)
        except Exception, e:
            self._log.error(str(e))
            if axis == 1: 
                value = '?'
                str_bin_value = '?'
            elif axis == 2: 
                value = '??'
                str_bin_value = '??????'
            elif axis == 3:
                value == "?"
                str_bin_value = '?' 


        state = PyTango.DevState.ON
        status = 'Hardware value: %s' % str_bin_value
        if axis == 1 and value != 1:
            state = PyTango.DevState.ALARM
            status += ' The air supply is NOT OK'
        elif axis == 2 and value != 0:
            state = PyTango.DevState.ALARM
            status += ' Some air pads are not lifted'
        elif axis == 3 and value != 0:
            state = PyTango.DevState.ALARM
            status += ' Some air pads are not lifted'
        return (state, status)

    def ReadOne(self, axis):
        value = copy.copy(self.array)
        if axis == 1:
            value &= 1
        elif axis == 2:
            value >>= 8
            value &= 63
        elif axis == 3:
            value >>= 14
            value &= 3
        return int(value)

    def WriteOne(self, axis, value):
        if axis == 1:
            PyTango.Except.throw_exception("PmacLTPIOCtrl WriteOne() exception",
                                           "Axis nr 1 is read-only",
                                           "PmacLTPIOCtrl.WriteOne()")
        if axis == 2:
            current_3axis = self.ReadOne(3)
            write_value = (value << 8) + (current_3axis << 14) + 1
        if axis == 3:
            current_2axis = self.ReadOne(2)
            write_value = (value << 14) + (current_2axis << 8) + 1
        self.array = copy.copy(write_value)

    def GetAxisExtraPar(self, axis, name):
        if name.lower() == 'labels':
            if axis == 1: return "closed:0 opened:1"
            elif axis == 2: return "lifted:0 landed:63"
            elif axis == 3: return "lifted:0 landed:3"
            else: PyTango.Except.throw_exception("PmacLTPIOCtrl GetExtraAttributePar() exception",
                                           "%d is not allowed axis nr" % axis,
                                           "PmacLTPIOCtrl.GetExtraAttributePar()")            

    def SetAxisExtraPar(self, axis, name, value):
        pass
#        if name.lower() == 'labels':
#            PyTango.Except.throw_exception("PmacLTPIOCtrl SerExtraAttributePar() exception",
#                                           "Labels is read only attribute",
#                                           "PmacLTPIOCtrl.SetExtraAttributePar()")
        
    def SendToCtrl(self,in_data):
        return "Not implemented yet"

