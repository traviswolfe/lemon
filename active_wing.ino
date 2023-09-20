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

#define AERO_TIMER 1200
#define AERO_LOCKED 2400
#define AERO_EXIT_TIMER 20
#define AIRBRAKE_MAX_POWER_TIME 1200
#define AIRBRAKE_DUTY_CYCLE 39  //0-255 range, 39 is ~15% duty cycle

int requestWingUp = 0;
int requestWingDown = 0;
int requestAirbrakeUp = 0;
int requestAirbrakeDown = 0;
int sensorThrottle = 0;
int sensorSteering = 0;
int sensorBrake = 0;

bool wingUp = false;
bool wingUpCurrentState = false;
bool airbrakeUp = false;
bool airbrakeUpCurrentState = false;
bool airbrakeLocked = false;
bool aeroPossible = false;
bool aeroEnabled = false;
bool aeroLocked = false;
bool aeroExitPossible = false;
unsigned long aeroPossibleSince = 0;
unsigned long aeroExitPossibleSince = 0;
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
}

void loop()
{
  //Read all inputs
  requestWingUp = digitalRead(PIN_WING_UP);
  requestWingDown = digitalRead(PIN_WING_DOWN);
  requestAirbrakeUp = digitalRead(PIN_AIRBRAKE_UP);
  requestAirbrakeDown = digitalRead(PIN_AIRBRAKE_DOWN);
  sensorThrottle = digitalRead(PIN_THROTTLE);
  sensorSteering = digitalRead(PIN_STEERING);
  sensorBrake = digitalRead(PIN_BRAKE);

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
    else if(aeroLocked == false)
    {
      //If aero is enabled, but has not been locked, check if enough time has passed for us to lock it
      if(aeroPossibleSince + AERO_LOCKED < millis())
      {
        aeroLocked = true;
      }
    }
    //Since aero is currently possible, reset the flag for exiting (if it was ever set)
    aeroExitPossible = false;
  }
  //Either the brake is depressed, the steering wheel has turned, or the throttle is no longer depressed
  else
  {
    //If aero is currently enabled, handle if we need to reset it after the timeout has elapsed
    if(aeroEnabled == true)
    {
      //If aero is locked and throttle was released, we're not going to reset
      if(sensorBrake == LOW || sensorSteering == HIGH || (sensorThrottle == HIGH && aeroLocked == false))
      {
        if(aeroExitPossible == false)
        {
          //If exit was previously not possible, store the time it becaome possible.
          aeroExitPossible = true;
          aeroExitPossibleSince = millis();
        }
        else if(aeroExitPossibleSince + AERO_EXIT_TIMER < millis())
        {
            //It has been possible to exit for AERO_EXIT_TIMER milliseconds consecutively. Exit aero.
            resetAero();
        }
      }
    }
    else
    {
      resetAero();
    }
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
    if(aeroLocked == true)
    {
      airbrakeUp = true;
      airbrakeLocked = true;
    }
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

  //Override wing and airbrake state with any requested mode. If, for whatever reason, UP and DOWN are both requested
  //because of a wiring issue we will default to airbrake off and the wing being up.
  if(requestWingDown == LOW)
  {
    wingUp = false;
  }
  if(requestWingUp == LOW)
  {
    wingUp = true;
  }
  if(requestAirbrakeUp == LOW)
  {
    airbrakeUp = true;
  }
  if(requestAirbrakeDown == LOW)
  {
    airbrakeUp = false;
  }

  //We should now have our state for the wing and airbrake. Check if the new state is different than the old state. If it
  //is, send the new state to the corresponding pins.
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
  //overheat when given 100% power continuously when stalled. Instead, send the rated duty cycle of the motor via PWM.
  if(airbrakeUp == true && airbrakeUpCurrentState == false)
  {
    digitalWrite(PIN_ACTUATOR_RETRACT, LOW);
    digitalWrite(PIN_ACTUATOR_EXTEND, HIGH);
    airbrakeUpCurrentState = true;
    airbrakeStateSince = millis();
  }
  else if(airbrakeUp == false && airbrakeUpCurrentState == true)
  {
    digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
    digitalWrite(PIN_ACTUATOR_RETRACT, HIGH);
    airbrakeUpCurrentState = false;
    airbrakeStateSince = millis();
  }
  else if(airbrakeStateSince + AIRBRAKE_MAX_POWER_TIME < millis())
  {
    if(airbrakeUpCurrentState == true)
    {
      //If the airbrake is up, and the brake pedal is pressed, send full power to hold state.
      if(sensorBrake == LOW && airbrakeLocked == true)
      {
        digitalWrite(PIN_ACTUATOR_EXTEND, HIGH);
      }
      else
      {
        analogWrite(PIN_ACTUATOR_EXTEND, AIRBRAKE_DUTY_CYCLE);
      }
    }
    else
    {
      analogWrite(PIN_ACTUATOR_RETRACT, AIRBRAKE_DUTY_CYCLE);
    }
  }
}
//Helper function to reset all aero-associated variables
void resetAero()
{
  aeroPossible = false;
  aeroEnabled = false;
  aeroLocked = false;
  aeroPossibleSince = 0;
}
