#ifndef _HASYLABMOTOR_H_
#define _HASYLABMOTOR_H_

#include <tango.h>
#include <pool/MotCtrl.h>

extern "C"
{
	/**
	 * The create controller method for HasylabMotor controller.
	 */	
	Controller *_create_HasylabMotor(const char *,vector<Controller::Properties> &);
}

/**
 * @brief A  Motor controller for HasylabMotor
 */
class HasylabMotor:public MotorController
{
	
public:
	/// Constructor
	HasylabMotor(const char *,vector<Controller::Properties> &);
	/// Destructor
	virtual ~HasylabMotor();

	/**
	 *	@name Management
	 *	Controller add/remove devices related methods
	 */
	//@{
	
	/**
	 * @brief adds (activates) a device in the controller given by the index.
	 * 
	 * @param idx the device index to be added (starts with 1).
	 */
	virtual void AddDevice(int32_t );
	
	/**
	 * @brief removes a device in the controller given by the index.
	 * 
	 * @param idx the device index to be removed (starts with 1).
	 */
	virtual void DeleteDevice(int32_t );


	virtual void PreStartAll();
	virtual void StartOne(int32_t, double);
	virtual	void StartAll();
	virtual void AbortOne(int32_t );

	virtual double ReadOne(int32_t );
	
	virtual void DefinePosition(int32_t ,double);


	/**
	 * @brief StateOne.
	 * 
	 * @param idx         [in] device index (starts with 1).
	 * @param ctrl_state [out] pointer to the state object that will contain the
	 *                         controller state.  
	 */	
	virtual void StateOne(int32_t, Controller::CtrlState *);
	//@}

	virtual Controller::CtrlData GetPar(int32_t, string &);
	virtual void SetPar(int32_t, string &, Controller::CtrlData &);
	
	
	/**
	 *	@name Extra Attributes
	 *	Extra attributes related methods.
	 */
	//@{
	/** 
	 * @brief Sets the given extra attribute parameter with the given value on
	 *        the given device index.
	 * 
	 * @param idx       [in] device index (starts at 1)
	 * @param attr_name [in] extra attribute name
	 * @param ctrl_data [in] new value reference object
	 */
	
	virtual Controller::CtrlData GetExtraAttributePar(int32_t, string &);
	virtual void SetExtraAttributePar(int32_t, string &, Controller::CtrlData &);
	
	//@}

	/**
	 * @brief Sends the given string to the controller.
	 * 
	 * @param the_str the string to be sent.
	 * 
	 * @return a string with the controller response.
	 */
	virtual string SendToCtrl(string &);
					
protected:
	void bad_data_type(string &);

	vector<double> 		wanted_mot_pos;
	vector<int32_t>		wanted_mot;

	struct HasylabMotorData 
	{
	  Tango::DeviceProxy	*proxy;
	  bool			device_available;
	  std::string		tango_device;
	};
	
	int32_t max_device;
	
	std::map<int32_t, HasylabMotorData*> motor_data; 
	
	std::string tmp_parameter1name;
	int flag_not_set_memorized_parameter1;	
	std::string tmp_extraparametername;
	int flag_extraparameter_active;

    stringstream            convert_stream;
};

#endif /*_HASYLABMOTOR_H_*/
