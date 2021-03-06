import PyTango
import time, os

from sardana import State, DataAccess
from sardana.pool.controller import TwoDController
from sardana.pool.controller import Type, Access, Description, DefaultValue
from sardana.pool import PoolUtil

ReadOnly = DataAccess.ReadOnly
ReadWrite = DataAccess.ReadWrite

class EigerPSICtrl(TwoDController):
    "This class is the Tango Sardana Two D controller for the Eiger PSI"


    ctrl_extra_attributes = {
                             'TangoDevice':{Type:'PyTango.DevString',Access:ReadOnly}
                             }
			     
    class_prop = {'RootDeviceName':{Type:str,Description:'The root name of the EigerPSI Tango devices'},
                  'TangoHost':{Type:str,Description:'The tango host where searching the devices'},}
			     
    MaxDevice = 97

    def __init__(self,inst,props, *args, **kwargs):
        self.TangoHost = None
        TwoDController.__init__(self,inst,props, *args, **kwargs)
        print "PYTHON -> TwoDController ctor for instance",inst

        self.ct_name = "EigerPSICtrl/" + self.inst_name
        if self.TangoHost == None:
            self.db = PyTango.Database()
        else:
            self.node = self.TangoHost
            self.port = 10000
            if self.TangoHost.find( ':'):
                lst = self.TangoHost.split(':')
                self.node = lst[0]
                self.port = int( lst[1])                           
            self.db = PyTango.Database(self.node, self.port)
        name_dev_ask =  self.RootDeviceName + "*"
	self.devices = self.db.get_device_exported(name_dev_ask)
        self.max_device = 0
        self.tango_device = []
        self.proxy = []
        self.device_available = []
	for name in self.devices.value_string:
            self.tango_device.append(name)
            self.proxy.append(None)
            self.device_available.append(0)
            self.max_device =  self.max_device + 1
        self.started = False
        
        
    def AddDevice(self,ind):
        TwoDController.AddDevice(self,ind)
        if ind > self.max_device:
            print "False index"
            return
        proxy_name = self.tango_device[ind-1]
        if self.TangoHost == None:
            proxy_name = self.tango_device[ind-1]
        else:
            proxy_name = str(self.node) + (":%s/" % self.port) + str(self.tango_device[ind-1])
        self.proxy[ind-1] = PyTango.DeviceProxy(proxy_name)
        self.device_available[ind-1] = 1
        
    def DeleteDevice(self,ind):
        TwoDController.DeleteDevice(self,ind)
        self.proxy[ind-1] =  None
        self.device_available[ind-1] = 0
        
    def StateOne(self,ind):
        if  self.device_available[ind-1] == 1:
            sta = self.proxy[ind-1].command_inout("State")
            if sta == PyTango.DevState.ON:
                tup = (sta,"Eiger ready")
            elif sta == PyTango.DevState.MOVING:
                tup = (sta,"Eiger busy")
            elif sta == PyTango.DevState.FAULT:
                tup = (sta,"Camera in FAULT state")
            return tup

    def PreReadAll(self):
        pass

    def PreReadOne(self,ind):
        pass

    def ReadAll(self):
        pass

    def ReadOne(self,ind):
        tmp_value = [(-1,), (-1,)]
        if self.device_available[ind-1] == 1:
            return tmp_value

    def PreStartAll(self):
        pass
    
    def PreStartOne(self, ind, value):
        return True
            
    def StartOne(self,ind, position=None):
        self.proxy[ind-1].command_inout("StartAcquisition")
        
    def AbortOne(self,ind):
        pass

    def LoadOne(self, ind, value):
        self.proxy[ind-1].write_attribute("ExposureTime",value)

    def GetAxisPar(self, ind, par_name):
        if par_name == "data_source":
            data_source = str(self.tango_device[ind -1]) + "/LastImage"
            return data_source
 
    def GetExtraAttributePar(self,ind,name):
        return 0

    def SetExtraAttributePar(self,ind,name,value):
        pass
        
    def SendToCtrl(self,in_data):
        return "Nothing sent"
        
    def __del__(self):
        print "PYTHON -> EigerPSICtrl/",self.inst_name,": dying"

        
if __name__ == "__main__":
    obj = TwoDController('test')
