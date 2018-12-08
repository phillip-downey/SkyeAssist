/*
   Developed by P. Downey, V. Patel, & A. Alvi for Upper-Limb Prosthesis Design Final Project, UC Berkeley 2018
   Based upon code structure developed by Professor Hannah Stuart and teaching team for ME292C, UC Berkeley 2018
*/

// Define drive motor functionality variables ----------------------------------
#define readA bitRead(PIND,3) //digital 3
#define readB bitRead(PIND,2) //digital 2
#define LED_PIN 13 //internal LED
#define M1PWM 9 //ports from the UNO to the motor driver
#define M1DIR 7 //ports from the UNO to the motor driver
#define M_EN 4
#define vibPWM 6
#define pot_pin A2
#define redLED 13
#define greenLED 12
#define black_button A3
#define red_button A4
const byte encoderPinA = 3; //encoder output, channel A, digital pin2
const byte encoderPinB = 2; //encoder output, channel B, digital pin3
volatile long count = 0; //current number of total encoder counts from both A and B channels
static int State = 0; //introduce the variable that we will use to switch between cases

// Define switch variables
const int switch_pin = 5;

// Define velocity function variables
int lastTime = 0; //initialize this for the velocity estimation function
int lastPos = 0; //initialize this for the velocity estimation function

//Position feedback variable setup
float err = 0;
float pos_ref = 0; // position reference input to the control loop

// Define potentiometer read variables
float pot_pos = 0;
float pot_ref = 0;
float pot_offset = 0;
float count_start = 0;
float map_gain = 4.2; //gain to convert from potentiometer to encoder position

// Define vibrating motor variables
unsigned long vib_end_time = 0;
bool vib_enabled = false;

// Integral control variables
float kp = 1; 
float ki = 0.05; 
float err_sum = 0; //Integral of the error
float u = 0; //input to the motor (PWM)

// Button override grasp variables
int buttonPWM = 100;

// Function Definitions --------------------------------------------------------

void isrA() { //increment or decrement the encoder count if channel A changes
  if (readB != readA) { // if A and B are not equal
    count ++; //increment
  } else {
    count --; // or else decrement
  }
}
void isrB() { //increment or decrement the encoder count if channel B changes
  if (readA == readB) { //if A and B are equal (high or low)
    count ++; //increment
  } else {
    count --; //or else decrement
  }
}

void pollPot() { //reset the global variable to the current position of the potentiometer
  pot_pos = analogRead(pot_pin);
}

bool pollSwitch() { //Don't need to worry about debouncing for this application
  if (digitalRead(switch_pin) == 1) {
    return true;
  }
  else {
    return false;
  }
}

float estimateVelocity() { //returns the absolute value of the velocity (+ only)
  int t = millis();
  if (t - lastTime >= 100) {
    int pos = count;
    float vel = abs((pos - lastPos) * 1000 / (t - lastTime));
    lastPos = pos;
    lastTime = t;
    return vel;
  }
}

void check_vib_timer() { //check if it is time to turn off the vibrating motor (allows us to not use a "delay" in the loop)
  if (millis() > vib_end_time) {
    analogWrite(vibPWM, 0);
    vib_enabled = false;
  }
}

void turn_on_vib(int duration) { //turn on the vibrating motor and define the end time
  vib_end_time = millis() + duration;
  analogWrite(vibPWM, 200);
  vib_enabled = true;
}

void checkLEDs() { //Turn on/off the correct LEDs for the current state
  if (State == 1) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  }
  else if (State == 2) {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  }
}

bool poll_red_button() { //returns true if the red button is being pushed
  if (analogRead(red_button) < 500) {
    return true;
  }
  else return false;
}

bool poll_black_button() { //returns true if the black button is being pushed
  if (analogRead(black_button) > 500) {
    return true;
  }
  else return false;
}

void setup() {
  Serial.begin(9600);
  //  while (not Serial) { //wait for Serial to conenct
  //    ;
  //  }
  pinMode(LED_PIN, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(M_EN, INPUT_PULLUP);
  digitalWrite(M_EN, LOW); //enable the motors
  pinMode(M1PWM, OUTPUT);
  digitalWrite(M1PWM, LOW);
  pinMode(M1DIR, OUTPUT);
  digitalWrite(M1DIR, LOW);

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  pinMode(black_button, INPUT_PULLUP);
  pinMode(red_button, INPUT_PULLUP);

  //Encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, CHANGE); //encoder channel A initialization
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, CHANGE); //encoder channel B initialization
} //End of setup loop

void loop() {
  checkLEDs(); //ensure correct LEDs are on for the current state

  if (vib_enabled) { //if the vib motor is on
    check_vib_timer(); //Check the timer to see if it's time to turn it off yet
  }

  switch (State) {
    case 0: //Init state, always start here & don't return to it
      Serial.println("State 0");
      pollPot(); //set the global variable for potentiometer position
      pot_offset = pot_pos; //Let this be the "zero" mark for the potentiometer
      count_start = count; //Let this be the "zero" mark for the drive motor
      State = 1; //Set the program in to the "augment" state
      err_sum = 0; //This is used for the integral controller
      break;
      
    //*************************************************************************
    //*************************************************************************
    
    case 1: //Using pot sensor to augment the grasp
      Serial.println("State 1");

      pollPot(); //Update the potentiometer position variable
      pot_ref = pot_pos - pot_offset; //this keeps track of how far and what direction we are from "zero"
      pot_ref = -pot_ref; // Flip the logic in this state so that the roation direction is correct
      pos_ref = count_start + map_gain * (pot_ref); //convert from potentiometer value to position (motor encoder) value
      err = pos_ref - count; //set the error as how far we are from the reference

      err_sum += err; //Maintain the error sum for integral control
      u = kp * err + ki * err_sum; //Input to the motor is the sum of the two control laws
      if (u > 0) { //pick motor direction based on value of the input
        digitalWrite(M1DIR, HIGH);
      }
      else {
        digitalWrite(M1DIR, LOW);
      }

      if (abs(u) > 10) { //If we are far enough away from the reference (helps avoid squealing)
        int manipulated_pwm = constrain(abs(u), 1, 255); //limit the pwm command to be [1:255]
        analogWrite(M1PWM, manipulated_pwm); //apply the feedback controller input
      }
      else { //if it's already close to the goal
        analogWrite(M1PWM, 0); //apply 0 so that it doesn't "whine"
      }

      if (poll_black_button()) { 
        digitalWrite(M1DIR, HIGH); //black button commands the motor one way
        analogWrite(M1PWM, buttonPWM); //apply the designated pwm
        err_sum = 0; //reset the integral value in the pot controller
        pollPot(); //reset pot global variable
        pot_offset = pot_pos; // As if we are coming in to the state again (pot "zero")
        count_start = count; // As if we are coming in to the state again (motor "zero")

      }

      if (poll_red_button()) { 
        digitalWrite(M1DIR, LOW); //red button commands the motor the other way
        analogWrite(M1PWM, buttonPWM);
        err_sum = 0; //reset the integral value in the pot controller
        pollPot(); //reset pot global variable
        pot_offset = pot_pos; // As if we are coming in to the state again (pot "zero")
        count_start = count; // As if we are coming in to the state again (motor "zero")
      }

      if (!pollSwitch()) { //if it is false: reset all the control variables and change states
        turn_on_vib(100); //pulse the vibrating motor for 100 ms
        pollPot();
        pot_offset = pot_pos;
        count_start = count;
        err_sum = 0;
        State = 2;
      }
      break;
      
    //*************************************************************************
    //*************************************************************************
    
    case 2: //Maintain grasp
      Serial.println("State 2");
      pollPot();
      pot_ref = pot_pos - pot_offset;
      pos_ref = count_start + map_gain * (pot_ref);
      err = pos_ref - count;

      err_sum += err;
      u = kp * err + ki * err_sum;

      if (u > 0) {
        digitalWrite(M1DIR, HIGH);
      }
      else {
        digitalWrite(M1DIR, LOW);
      }

      if (abs(u) > 10) { //if there's a notable difference
        int manipulated_pwm = constrain(abs(u), 1, 255);
        //Serial.println(manipulated_pwm);
        analogWrite(M1PWM, manipulated_pwm); //apply the feedback controller
      }
      else { //if it's already close to the goal
        analogWrite(M1PWM, 0); //apply 0 so that it doesn't "whine"
      }

      if (pollSwitch()) { //if the switch is true
        turn_on_vib(100);
        pollPot();
        pot_offset = pot_pos;
        count_start = count;
        State = 1;
      }
      break;
      
    //*************************************************************************
    //*************************************************************************

    default: //Shouldn't ever come in to this state
      Serial.println("Default State");
      analogWrite(M1PWM, 0); //brake the motors for safety
      break;

  } //end of switch statement
} //end of for loop
