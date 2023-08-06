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

#define AERO_TIMER 1500
#define AERO_LOCKED 3000

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
int aeroPossibleSince = -1;

int counter = 0;
int timer = 0;

void setup()
{
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
  pinMode(PIN_ACTUATOR_RETRACT, OUTPUT);    //PWM signal to retract actuator. Digial HIGH for 100%. EXTEND ping must be off.
  pinMode(PIN_SOLENOID, OUTPUT);            //Digital write LOW to extend the wing, HIGH to stow the wing.

  //Set initial state of wing extended, air brake disabled
  digitalWrite(PIN_SOLENOID, LOW);
  digitalWrite(PIN_ACTUATOR_EXTEND, LOW);
  digitalWrite(PIN_ACTUATOR_RETRACT, HIGH);

  //Serial.begin(9600);
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
      //If aero was previously not possible, store the time that it became possible
      //we will use this to track how long it has been
      aeroPossible = true;
      aeroPossibleSince = millis();
    }
    else if(aeroEnabled == false)
    {
      //If aero was previously possible, but still has not been enabled, check if
      //enough time has passed for us to enable it
      if(aeroPossibleSince + AERO_TIMER > millis())
      {
        aeroEnabled = true;
      }
    }
    else if(aeroLocked == false)
    {
      //If aero is enabled, but has not been locked, check if enough time has passed
      //for us to lock it
      if(aeroPossibleSince + AERO_LOCKED > millis())
      {
        aeroLocked = true;
      }
    }
  }

  if(sensorBrake == LOW)
  {
    //Brake has been depressed
    resetAero();
  }
  else if(aeroEnabled == false && airbrakeLocked == true)
  {
    //If aero is disabled, and we are no longer braking, disable the airbrake
    airbrakeLocked = false;
  }
  if(sensorSteering == HIGH)
  {
    //Steering wheel is not straight
    resetAero();
  }
  if(sensorThrottle == HIGH)
  {
    //Throttle is no longer depressed
    if(aeroLocked == false)
    {
      //Only reset aero if aero did not get locked, so we can shift
      resetAero();
    }
  }

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

  //Override with any requested mode. If, for whatever reason, UP and DOWN are both requested because of a wiring issue
  //we will default to airbrake off and the wing being up.
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
  //Serial.println(String(millis()) + ":   wingup:" + String(wingUp) + "   airbrakeup:" + String(airbrakeUp));
  //Serial.println(String(millis()) + ":   sensorThrottle:" + String(sensorThrottle) + "   sensorSteering:" + String(sensorSteering) + "   sensorBrake:" + String(sensorBrake));
  //delay(1000);
}

void resetAero()
{
    aeroPossible = false;
    aeroEnabled = false;
    aeroLocked = false;
    aeroPossibleSince = -1;
}
