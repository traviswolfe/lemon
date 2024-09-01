#define PIN_WING_UP 1
#define PIN_WING_DOWN 2
#define PIN_AIRBRAKE_UP 3
#define PIN_AIRBRAKE_DOWN 4
#define PIN_THROTTLE 5
#define PIN_STEERING 6
#define PIN_BRAKE 0
#define PIN_ACTUATOR_EXTEND 9
#define PIN_ACTUATOR_RETRACT 10
#define PIN_SOLENOID 11
#define PIN_SOLENOID_UNUSED 12

//Timers
#define AERO_TIMER 1250
#define AIRBRAKE_MAX_POWER_TIME 2100
#define JITTER_COMPENSATION 7

//Modes
#define AUTO 0    //Auto mode, uses sensors to enter and exist DRS
#define PTDRS 1   //Push-To-DRS mode, uses button to enter and sensors to exit DRS
#define PAHDRS 2  //Push-and-Hold DRS mode, uses button to enter and exit DRS
#define OTHER 3   //Static state, uses toggle switches

int requestWingUp = 0;
int requestWingDown = 0;
int requestAirbrakeUp = 0;
int requestAirbrakeDown = 0;
int sensorThrottle = 0;
int sensorSteering = 0;
int sensorBrake = 0;

int requestWingUpLast = 0;
int requestWingDownLast = 0;
int requestAirbrakeUpLast = 0;
int requestAirbrakeDownLast = 0;
int sensorThrottleLast = 0;
int sensorSteeringLast = 0;
int sensorBrakeLast = 0;

unsigned long requestWingUpTime = 0;
unsigned long requestWingDownTime = 0;
unsigned long requestAirbrakeUpTime = 0;
unsigned long requestAirbrakeDownTime = 0;
unsigned long sensorThrottleTime = 0;
unsigned long sensorSteeringTime = 0;
unsigned long sensorBrakeTime = 0;

int pinvalue = 0;
int lastMode = 0;
bool drsButton = false;

bool wingUp = false;
bool wingUpCurrentState = false;
bool airbrakeUp = false;
bool airbrakeUpCurrentState = false;
bool airbrakeLocked = false;
bool airbrakeMoving = false;
bool aeroPossible = false;
bool aeroEnabled = false;
unsigned long aeroPossibleSince = 0;
unsigned long airbrakeStateSince = 0;

void setup()
{
  //Set the states of all the pins so we can read them
  //
  //Input type of INPUT_PULLUP adds a ~20k resistor to 5v on the input pin. This way, when no contact is made, the pin will
  //read HIGH. When contact is made, the pin is grounded, and will read LOW.
  pinMode(PIN_WING_UP, INPUT_PULLUP);       //Digital read LOW means wing elevated requested
  pinMode(PIN_WING_DOWN, INPUT_PULLUP);     //Digital read LOW means wing stowed requested
  pinMode(PIN_AIRBRAKE_UP, INPUT_PULLUP);   //Digital read LOW means airbrake forced up requested
  pinMode(PIN_AIRBRAKE_DOWN, INPUT_PULLUP); //Digital read LOW means airbrake forced down requested
  pinMode(PIN_THROTTLE, INPUT_PULLUP);      //Digital read LOW means throttle is active
  pinMode(PIN_STEERING, INPUT_PULLUP);      //Digital read LOW means steering wheel is straight
  pinMode(PIN_BRAKE, INPUT_PULLUP);         //Digital read LOW means brake light is active
  pinMode(PIN_ACTUATOR_EXTEND, OUTPUT);     //PWM signal to extend actuator. Digital HIGH for 100%. RETRACT pin must be off.
  pinMode(PIN_ACTUATOR_RETRACT, OUTPUT);    //PWM signal to retract actuator. Digital HIGH for 100%. EXTEND pin must be off.
  pinMode(PIN_SOLENOID, OUTPUT);            //Digital write LOW to extend the wing, HIGH to stow the wing.
  pinMode(PIN_SOLENOID_UNUSED, OUTPUT);     //Unused pin, however it is connected to the h-bridge. Must be set low

  //Set initial state of wing extended, air brake disabled
  digitalWrite(PIN_SOLENOID, LOW);
  digitalWrite(PIN_SOLENOID_UNUSED, LOW);
  digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
  digitalWrite(PIN_ACTUATOR_RETRACT, HIGH);
  airbrakeUpCurrentState = false;
  wingUpCurrentState = true;
  airbrakeStateSince = millis();
  airbrakeMoving = true;
}

void loop()
{
  //Read all inputs. This also compensates for bounce and jitter.
  readInputs();

  //Check DRS button state
  drsButton = (requestWingUp == LOW && requestWingDown == LOW);

  //If DRS button is pressed, we cannot read the requested mode (because requestWingUp and requestWingDown are both
  //LOW). We will have to use the last used mode in this case.
  
  //Determine the mode we should be operating in, and then run that mode. Reset states/timers if changed.
  if((requestWingUp == HIGH && requestWingDown == HIGH && requestAirbrakeUp == HIGH && requestAirbrakeDown == HIGH) || (drsButton && lastMode == AUTO))
  {
    checkLastMode(AUTO);
    handleAuto();
  }
  else if((requestWingUp == LOW && requestWingDown == HIGH && requestAirbrakeUp == HIGH && requestAirbrakeDown == HIGH) || (drsButton && lastMode == PTDRS))
  {
    checkLastMode(PTDRS);
    handlePTDRS();
  }
  else if((requestWingUp == LOW && requestWingDown == HIGH && requestAirbrakeUp == HIGH && requestAirbrakeDown == LOW) || (drsButton && lastMode == PAHDRS))
  {
    checkLastMode(PAHDRS);
    handlePAHDRS();
  }
  else
  {
    checkLastMode(OTHER);
    handleOther();
  }

  //Write outputs. Will only write if requested state has changed to save on clock cycles.
  handleOutput();
}

void readInputs()
{
  readInput(PIN_WING_UP, &requestWingUp, &requestWingUpLast, &requestWingUpTime);
  readInput(PIN_WING_DOWN, &requestWingDown, &requestWingDownLast, &requestWingDownTime);
  readInput(PIN_AIRBRAKE_UP, &requestAirbrakeUp, &requestAirbrakeUpLast, &requestAirbrakeUpTime);
  readInput(PIN_AIRBRAKE_DOWN, &requestAirbrakeDown, &requestAirbrakeDownLast, &requestAirbrakeDownTime);
  readInput(PIN_THROTTLE, &sensorThrottle, &sensorThrottleLast, &sensorThrottleTime);
  readInput(PIN_STEERING, &sensorSteering, &sensorSteeringLast, &sensorSteeringTime);
  readInput(PIN_BRAKE, &sensorBrake, &sensorBrakeLast, &sensorBrakeTime);
}
void readInput(int pin, int* value, int* lastvalue, unsigned long* lasttime)
{
  pinvalue = digitalRead(pin);
  if(pinvalue != *lastvalue)
  {
    *lasttime = millis();
    *lastvalue = pinvalue;
  }
  if( (*value != pinvalue) && (millis() > *lasttime + JITTER_COMPENSATION))
  {
    *value = pinvalue;
  }
}

void handleOutput()
{
  //Check if the new state is different than the old state. If it is, send the new state to the corresponding pins.
  if(wingUp == true && wingUpCurrentState == false)
  {
    digitalWrite(PIN_SOLENOID, LOW);
    wingUpCurrentState = true;
  }
  else if(wingUp == false && wingUpCurrentState == true)
  {
    digitalWrite(PIN_SOLENOID, HIGH);
    wingUpCurrentState = false;
  }

  //For the airbrake, we only want to send full power for a few seconds. DC motors draw more power when stalled, and can
  //overheat when given 100% power continuously when stalled.
  if(airbrakeUp == true && airbrakeUpCurrentState == false)
  {
    digitalWrite(PIN_ACTUATOR_RETRACT, LOW);
    digitalWrite(PIN_ACTUATOR_EXTEND, HIGH);
    airbrakeStateSince = millis();
    airbrakeUpCurrentState = true;
    airbrakeMoving = true;
  }
  else if(airbrakeUp == false && airbrakeUpCurrentState == true)
  {
    digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
    digitalWrite(PIN_ACTUATOR_RETRACT, HIGH);
    airbrakeStateSince = millis();
    airbrakeUpCurrentState = false;
    airbrakeMoving = true;
  }
  else if(airbrakeMoving == true && ((airbrakeStateSince + AIRBRAKE_MAX_POWER_TIME) < millis()))
  {
    digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
    digitalWrite(PIN_ACTUATOR_RETRACT, LOW);
    airbrakeMoving = false;
  }
}

void checkLastMode(int mode)
{
  if(lastMode != mode)
  {
    resetAll();
    lastMode = mode;
  }
}

void handleAuto()
{
  //Auto Mode
  //Will use sensors to enter and exit DRS
  //Steering wheel must be straight, brake must not be depressed, and gas pedal must be depressed for AERO_TIMER milliseconds
  //Once brake pedal or steering wheel changes, it will exit aero mode. If brake pedal is depressed, air brake will remain
  //engaged until brake pedal is released.

  //Check if we should enter into aero mode
  if(sensorThrottle == LOW && sensorSteering == LOW && sensorBrake == HIGH)
  {
    //Throttle depressed, steering wheel is straight, brake is not active
    if(aeroPossible == false)
    {
      //If aero was previously not possible, store the time that it became possible. We will use this to track how long it
      //has been
      aeroPossible = true;
      aeroPossibleSince = millis();
    }
    else if(aeroEnabled == false)
    {
      //If aero was previously possible, but still has not been enabled, check if enough time has passed for us to enable it
      if(aeroPossibleSince + AERO_TIMER < millis())
      {
        aeroEnabled = true;
      }
    }
  }
  //If the steering wheel is turned or brake depressed, exit aero mode
  else if(sensorSteering == HIGH || sensorBrake == LOW)
  {
    aeroEnabled = false;
    aeroPossible = false;
  }
  //If steering wheel has been released, aero is no longer possible (but may still be enabled)
  else
  {
    aeroPossible = false;
  }

  //If we exited aero, and locked the air brake, but we are not longer pressing the brake pedal, then unlock the airbrake
  if(sensorBrake == HIGH && aeroEnabled == false && airbrakeLocked == true)
  {
    airbrakeLocked = false;
  }

  //Aero state should be set. Set wing and airbrake states.
  if(aeroEnabled == true)
  {
    wingUp = false;
    airbrakeUp = true;
    airbrakeLocked = true;
  }
  else
  {
    if(airbrakeLocked == true)
    {
      airbrakeUp = true;
    }
    else
    {
      airbrakeUp = false;
    }
    wingUp = true;
  }
}

void handlePTDRS()
{
  //Push-to-DRS Mode
  //Will use button to start DRS, and sensors to exit. Will only exit with brake pedal.

  //Enter aero when DRS button is pressed
  if(drsButton)
  {
    aeroEnabled = true;
  }

  //If brake depressed, exit aero mode
  if(sensorBrake == LOW)
  {
    aeroEnabled = false;
    aeroPossible = false;
  }

  //If we exited aero, and locked the air brake, but we are not longer pressing the brake pedal, then unlock the airbrake
  if(sensorBrake == HIGH && aeroEnabled == false && airbrakeLocked == true)
  {
    airbrakeLocked = false;
  }

  //Aero state should be set. Set wing and airbrake states.
  if(aeroEnabled == true)
  {
    wingUp = false;
    airbrakeUp = true;
    airbrakeLocked = true;
  }
  else
  {
    if(airbrakeLocked == true)
    {
      airbrakeUp = true;
    }
    else
    {
      airbrakeUp = false;
    }
    wingUp = true;
  }
  
}

void handlePAHDRS()
{
  //Push-and-Hold DRS. Will only engage DRS while button is actively being held.

  if(drsButton)
  {
    wingUp = false;
    airbrakeUp = true;
  }
  else
  {
    wingUp = true;
    airbrakeUp = false;
  }
}

void handleOther()
{
  //Other/no DRS
  //Will respect toggle switches, but no DRS will engage.

  if(requestWingDown == LOW)
  {
    wingUp = false;
  }
  if(requestWingUp == LOW)
  {
    wingUp = true;
  }
  
  //Hack, if drs button is pressed while in this mode, ignore it!
  if(drsButton)
  {
	  wingUp = wingUpCurrentState;
  }
  
  if(requestAirbrakeUp == LOW)
  {
    airbrakeUp = true;
  }
  if(requestAirbrakeDown == LOW)
  {
    airbrakeUp = false;
  }
}

void resetAll()
{
  //Set initial state of wing extended, air brake disabled
  airbrakeUp = false;
  wingUp = true;

  aeroPossible = false;
  aeroEnabled = false;
  airbrakeLocked = false;
  aeroPossibleSince = 0;
}
