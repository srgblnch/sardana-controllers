#ifndef _OMSCTRL_H
#define _OMSCTRL_H

#include <pool/MotCtrl.h>
#include <tango.h>

extern "C"
{
	Controller *_create_OmsVme58Ctrl(const char *,vector<Controller::Properties> &);
}


class OmsVme58Ctrl:public MotorController
{
public:
	OmsVme58Ctrl(const char *,vector<Controller::Properties> &);
	virtual ~OmsVme58Ctrl();

	virtual void AddDevice(int32_t );
	virtual void DeleteDevice(int32_t );

	virtual void PreStartAll();
	virtual void StartOne(int32_t, double);
	virtual	void StartAll();
	virtual void AbortOne(int32_t );

	virtual double ReadOne(int32_t );
	
	virtual void DefinePosition(int32_t ,double);
	virtual void StateOne(int32_t, Controller::CtrlState *);
	
	virtual Controller::CtrlData GetPar(int32_t, string &);
	virtual void SetPar(int32_t, string &, Controller::CtrlData &);
	
	virtual Controller::CtrlData GetExtraAttributePar(int32_t, string &);
	virtual void SetExtraAttributePar(int32_t, string &, Controller::CtrlData &);
	
	virtual string SendToCtrl(string &);
					
protected:
	void bad_data_type(string &);

	vector<double> 		wanted_mot_pos;
	vector<int32_t>		wanted_mot;

	struct MotorData 
	{
	  Tango::DeviceProxy	*proxy;
	  bool			device_available;
	  std::string		tango_device;
	};
	
	int32_t max_device;
	
	std::map<int32_t, MotorData*> motor_data; 
	

	stringstream            convert_stream;
};

#endif /* _OMSCTRL_H */
