/*
Motor driver support for the SM300 controller.
*/


// controller-specific parameters yet
#define NUM_SM300_PARAMS 4
#define SM300ResetString		"RESET"
#define SM300ResetAndHomeString		"RESET_AND_HOME"
#define SM300DisconnectString	"DISCONNECT"
#define SM300ErrorCodeString	"ERROR_CODE"

class epicsShareClass SM300Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  SM300Axis(class SM300Controller *pC, int axis, char axisLabel);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  bool has_error();
private:
  SM300Controller *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  double stepSize_;      /**< Encoder increment value obtained with SU? command _or_ resolution, set at boot time */
                         /*   with SMC100CreateController command */
  char axisLabel; /** label for the axis*/
  bool has_error_;
friend class SM300Controller;
};

class SM300Controller : public asynMotorController {
public:
  SM300Controller(const char *portName, const char *SMC100PortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  void report(FILE *fp, int level);
  SM300Axis* getAxis(asynUser *pasynUser);
  SM300Axis* getAxis(int axisNo);
  void setTerminationChars(const char *eosIn, int eosInlen, const char *eosOut, int eosOutlen);

  virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  virtual asynStatus poll();  
  asynStatus sendCommand(const char * querry);
  asynStatus sendQuerry(const char * querry, bool hasEotEnding);
  bool has_error();
  bool is_moving();
  void homeAxis(const char axis);
private:
	asynStatus perform_reset();

	bool has_error_;
	bool is_moving_;
	int reset_;
	int reset_and_home_;
	int disconnect_;
	int error_code_;
	bool axis_x_homing_;
	bool axis_y_homing_;
	bool home_axis_x_;
	bool home_axis_y_;


friend class SM300Axis;
	
};
