#define PIN_WING_UP 1
#define PIN_WING_DOWN 2
#define PIN_AIRBRAKE_UP 3
#define PIN_AIRBRAKE_DOWN 4
#define PIN_THROTTLE 5
#define PIN_STEERING 6
#define PIN_BRAKE 0
#define PIN_ACTUATOR_EXTEND 10
#define PIN_ACTUATOR_RETRACT 11
#define PIN_SOLENOID 12
#define PIN_SOLENOID_UNUSED 13

#define AERO_TIMER 1500
#define AERO_LOCKED 3000
#define AERO_EXIT_TIMER 20

int requestWingUp = 0;
int requestWingDown = 0;
int requestAirbrakeUp = 0;
int requestAirbrakeDown = 0;
int sensorThrottle = 0;
int sensorSteering = 0;
int sensorBrake = 0;

bool wingUp = false;
bool airbrakeUp = false;
bool airbrakeLocked = false;
bool aeroPossible = false;
bool aeroEnabled = false;
bool aeroLocked = false;
bool aeroExitPossible = false;
int aeroPossibleSince = -1;
int aeroExitPossibleSince = -1;

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
  pinMode(PIN_STEERING, INPUT);             //Digital read LOW means steering wheel is straight
  pinMode(PIN_BRAKE, INPUT_PULLUP);         //Digital read LOW means brake light is active
  pinMode(PIN_ACTUATOR_EXTEND, OUTPUT);     //PWM signal to extend actuator. Digital HIGH for 100%. RETRACT pin must be off.
  pinMode(PIN_ACTUATOR_RETRACT, OUTPUT);    //PWM signal to retract actuator. Digial HIGH for 100%. EXTEND pin must be off.
  pinMode(PIN_SOLENOID, OUTPUT);            //Digital write LOW to extend the wing, HIGH to stow the wing.
  pinMode(PIN_SOLENOID_UNUSED, OUTPUT);     //Unused pin, however it is connected to the h-bridge. Must be set low

  //Set initial state of wing extended, air brake disabled
  digitalWrite(PIN_SOLENOID, LOW);
  digitalWrite(PIN_SOLENOID_UNUSED, LOW);
  digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
  digitalWrite(PIN_ACTUATOR_RETRACT, HIGH);
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
      if(aeroPossibleSince + AERO_LOCKED > millis())
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

  //We should now have our state for the wing and airbrake. Send the signal to the actuator and solenoid
  if(wingUp == true)
  {
    digitalWrite(PIN_SOLENOID, LOW);
  }
  else
  {
    digitalWrite(PIN_SOLENOID, HIGH);
  }
  if(airbrakeUp == true)
  {
    digitalWrite(PIN_ACTUATOR_RETRACT, LOW);
    digitalWrite(PIN_ACTUATOR_EXTEND, HIGH);
  }
  else
  {
    digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
    digitalWrite(PIN_ACTUATOR_RETRACT, HIGH);
  }
}

//Helper function to reset all aero-associated variables
void resetAero()
{
  aeroPossible = false;
  aeroEnabled = false;
  aeroLocked = false;
  aeroPossibleSince = -1;
}
