/*
Motor driver support for the Huber controller.
*/

#ifndef HuberDriver_H
#define HuberDriver_H

// controller-specific parameters yet
#define HuberResetString		"RESET"
#define HuberResetAndHomeString		"RESET_AND_HOME"
#define HuberDisconnectString	"DISCONNECT"
#define HuberErrorCodeString	"ERROR_CODE"

class epicsShareClass HuberAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  HuberAxis(class HuberController *pC, int axis, char axisLabel);
  asynStatus setPosition(double position);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  bool has_error();
private:
  int axisNo;
  HuberController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  double stepSize_;      /**< Encoder increment value obtained with SU? command _or_ resolution, set at boot time */
                         /*   with SMC100CreateController command */
  char axisLabel; /** label for the axis*/
  bool has_error_;
friend class HuberController;
};

class HuberController : public asynMotorController {
public:
  HuberController(const char *portName, const char *SMC100PortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  HuberAxis* getAxis(asynUser *pasynUser);
  HuberAxis* getAxis(int axisNo);
  void setTerminationChars(const char *eosIn, int eosInlen, const char *eosOut, int eosOutlen);

  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  asynStatus sendCommand(const char * query);
  asynStatus sendQuery(const char * query, bool hasEotEnding);

  bool is_moving();
private:
	bool has_error_;
	bool is_moving_;
	bool axis_x_homing_;
	bool axis_y_homing_;
	bool home_axis_x_;
	bool home_axis_y_;

	int polls_count_;

	// number of times the error flag should be refreshed manually. 10 is arbitary bt works.
	static const int REFRESH_ERROR_FOR_POLL_COUNTS = 10;

	#define FIRST_Huber_PARAM reset_
	int reset_; // int param
	int reset_and_home_; // int param
	int disconnect_; // int param
	int error_code_; // int param
	#define LAST_Huber_PARAM error_code_
	

friend class HuberAxis;
	
};

#define NUM_Huber_PARAMS (&LAST_Huber_PARAM - &FIRST_Huber_PARAM + 1)
#endif  // HuberDriver_H
