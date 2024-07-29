/*
 *  This is version 2.0 of the Bluey motor control software. This is basically a wrapper for the
 *  a ROS node to talk to the two thrusters and an (optional) underwater light. Also critically
 *  it talks to a Fly Sky RC controller.  This document describes connection to a FS-16 controller
 *  Other controlers may be different.
 *
 *  Transmitter configuration:
 *  The current code listens to the two joysticks (forward and back only) which are used to provide
 *  direct control of the thrusters. And the two switches on the upper left hand side of the controller.
 *
 *  Before using the controller, ensure that the controller is set up with FAILSAFE enabled on channel 6 at -100%
 *  Nothing will work well if you do not. To do this
 *  * turn the controller on
 *  * Enable the AUX buttons with button A assigned to channel 5 and button B assigned to channel 6
 *  * Enable failsafe -100 on channel 
 *  * hold the OK button (two up from the power switch) on for a few seconds until the display shows two icons
 *    they are system and setup. Use the UP and DOWN buttons on the left hand side of the display so that SYSTEM
 *    is selected. Push the OK button. This will bring up the SYSTEM menu.
 *  * use the UP AND DOWN buttons to choose RX setup and press the OK button.
 *  * use the UP and DOWN buttons to select Failsafe and press the OK button
 *  * use the UP and DOWN buttons tp select channel 6
 *  * use the UP AND DOWN buttons to select ON
 *  * use VRA switch that the bar goes to the left (it will either be the left or right)
 *  * hold the CANCEL button (above the power switch) to set the mode. Use cancel button
 *  * after this you should see -100% on the display for Failsafe sets
 *
 *  Operational Mode
 *  The thruster system for the robot is one of three modes
 *  *  MODE_ROS - the system is being controlled by ROS
 *  *  MODE_MANUAL - the system is being controlled directly by the remote controller
 *  *  MODE_ESTOP - the system is in estop mode. It will do nothing until it is taken out of this mode
 *
 *  The second switch from the left is the ESTOP switch. If it is towards you (backward) the robot is in estop mode.
 *  It will also go into estop mode if signal from the controller is ever lost.
 *
 *  The switch on the left is the ROS/MANUAL mode. ROS is in the forward position. You can swap from one state to the other
 *  while running. Moving into MODE_MANUAL when running ROS nodes will not stop the ROS world (and the node publishes the current
 *  state of this switch so your ROS code can respond accordingly). Similarly switching into ROS mode will cause your ROS code to
 *  control the robot.
 *  
 *  If the robot is in ESTOP mode, and you wish to put it in a running mode
 *  * move the second from the left switch UP
 *  * ensure that the controller is on
 *  * toggle the switch on the left up and down twice so that it ends in manual mode. Then the robot will leave MODE_ESTOP
 *    and move to MODE_MANUAL
 *
 * Version 3.0
 *      - code cleanup and simplification
 * Version 2.1
 *      - display even when not yet syncd to remote
 *      - better power on behaviour (planning for actual ESC connection)
 *  Version 2.0
 *      - general code cleanup and documentation.
 *  Version 1.0
 *      - initial system
 */

 /*
  * This relies heavily on the WM decoding of RC receiver code by Kelvin Nelson 24/07/2019.
  * That library provided a software-based failsafe handling algorithm. Here we used the failsafe provided
  * by FLYSKY.
  */




#include <Wire.h>
#include <Servo.h>

// comment out the following for runtime
//#define DEBUG


const int MODE_ROS = 1;
const int MODE_MAN = 2;
const int MODE_ESTOP = 3;
int mode = MODE_ESTOP;  // we start in stop mode unless told otherwise

const int LEFT_STICK = 0;
const int RIGHT_STICK = 1;
const int RUN_MODE = 2;
const int ESTOP_STATE = 3;

// the input pins used on the arduino 
const int pwmPIN[]={4, 5, 6, 7}; 
const int nChannels = 4;

// min, midpoint and max values for the RC channels used. You can probably just set to 1000, 1500 and 2000
int rc_min[4] = { 1004, 1060, 1000, 1000};
int rc_mid[4] = { 1496, 1496, 1500, 1500};
int rc_max[4] = { 2000, 2000, 2000, 2000};

# define LIGHT_OFF 1100
# define LIGHT_MAX 1900
# define LIGHT_RANGE (LIGHT_MAX - LIGHT_OFF)
# define THRUSTER_MAX_REV 1100
# define THRUSTER_MAX_FWD 1900
# define THRUSTER_STOP 1500
# define THRUSTER_RANGE (THRUSTER_MAX_FWD - THRUSTER_STOP)

// references for the servos
Servo port;
Servo starboard;
Servo light;
const int port_pin = 8;
const int starboard_pin = 9;
const int light_pin = 10;

int estop_recover = 0;

const float gain = 0.8; // used to do program hacking

float ros_port = 0;
float ros_starboard = 0;
float ros_light = 0;

// high performance access to the PWM inputs


/*
 *    GLOBAL PWM DECODE VARIABLES
 */

volatile int PW[nChannels];                        // an array to store pulsewidth measurements
volatile boolean prev_pinState[nChannels];         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;                    // the time of the current pin change interrupt
volatile unsigned long pwmTimer[nChannels];        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[nChannels];               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;                      // flag when all RC receiver channels have received a new pulse
unsigned long pwmPeriod[nChannels];                // period, mirco sec, between two pulses on each pin

byte pwmPIN_reg[nChannels];                        // each of the input pins expressed as a position on it's associated port register
byte pwmPIN_port[nChannels];                       // identify which port each input pin belongs to (0 = PORTB, 1 = PORTC, 2 = PORTD)

// FUNCTION USED TO TURN ON THE INTERRUPTS ON THE RELEVANT PINS
// code from http://playground.arduino.cc/Main/PinChangeInterrupt

void pciSetup(byte pin){
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

// FUNCTION USED TO FIND THE PIN POSITION ON EACH PORT REGISTER: helps the interrupt service routines, ISR, run faster

void pwmPIN_to_port(){
  for (int i = 0; i < nChannels; i++){

    // determine which port and therefore ISR (PCINT0_vect, PCINT1_vect or PCINT2_vect) each pwmPIN belongs to.
                                                                  pwmPIN_port[i] = 1;    // pin belongs to PCINT1_vect (PORT C)
    if (pwmPIN[i] >= 0 && pwmPIN[i] <= 7)                         pwmPIN_port[i] = 2;    // pin belongs to PCINT2_vect (PORT D)
    else if (pwmPIN[i] >= 8 && pwmPIN[i] <= 13)                   pwmPIN_port[i] = 0;    // pin belongs to PCINT0_vect (PORT B)

    // covert the pin number (i.e. pin 11 or pin A0) to the pin position in the port register. There is most likely a better way of doing this using a macro...
    // (Reading the pin state directly from the port registers speeds up the code in the ISR)
    
    if(pwmPIN[i] == 0 || pwmPIN[i] == A0 || pwmPIN[i] == 8)         pwmPIN_reg[i] = 0b00000001;
    else if(pwmPIN[i] == 1 || pwmPIN[i] == A1 || pwmPIN[i] == 9)    pwmPIN_reg[i] = 0b00000010;
    else if(pwmPIN[i] == 2 || pwmPIN[i] == A2 || pwmPIN[i] == 10)   pwmPIN_reg[i] = 0b00000100;
    else if(pwmPIN[i] == 3 || pwmPIN[i] == A3 || pwmPIN[i] == 11)   pwmPIN_reg[i] = 0b00001000;
    else if(pwmPIN[i] == 4 || pwmPIN[i] == A4 || pwmPIN[i] == 12)   pwmPIN_reg[i] = 0b00010000;
    else if(pwmPIN[i] == 5 || pwmPIN[i] == A5 || pwmPIN[i] == 13)   pwmPIN_reg[i] = 0b00100000;
    else if(pwmPIN[i] == 6)                                         pwmPIN_reg[i] = 0b01000000;
    else if(pwmPIN[i] == 7)                                         pwmPIN_reg[i] = 0b10000000;
    
  }
}

// SETUP OF PIN CHANGE INTERRUPTS

void setup_pwmRead(){
  
  for(int i = 0; i < nChannels; i++){              // run through each input pin
    pciSetup(pwmPIN[i]);                        // enable pinchange interrupt for pin
  }
  pwmPIN_to_port();                             // determines the port for each input pin
                                                // pwmPIN_to_port() also coverts the pin number in pwmPIN[] (i.e. pin 11 or pin A0) to the pin position in the port register (i.e. 0b00000001) for use in the ISR.                                         
} 

// INTERRUPT SERVICE ROUTINES (ISR) USED TO READ PWM INPUT

// the PCINT0_vect (B port register) reacts to any changes on pins D8-13.
// the PCINT1_vect (C port register)          ""        ""         A0-A5.
// the PCINT2_vect (D port register)          ""        ""         D0-7.

// port registers are used to speed up if statements in ISR code:
// https://www.arduino.cc/en/Reference/PortManipulation http://tronixstuff.com/2011/10/22/tutorial-arduino-port-manipulation/
// http://harperjiangnew.blogspot.co.uk/2013/05/arduino-port-manipulation-on-mega-2560.html


// READ INTERRUPTS ON PINS D8-D13: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT0_vect){                                                 // this function will run if a pin change is detected on portB
  
  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < nChannels; i++){                            // run through each of the channels
    if (pwmPIN_port[i] == 0){                                     // if the current channel belongs to portB
      
      if(prev_pinState[i] == 0 && PINB & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINB & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == nChannels) RC_data_rdy = HIGH;                  
      }
    }
  }
}

// READ INTERRUPTS ON PINS A0-A5: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT1_vect){                                                 // this function will run if a pin change is detected on portC

  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < nChannels; i++){                            // run through each of the channels
    if (pwmPIN_port[i] == 1){                                     // if the current channel belongs to portC
      
      if(prev_pinState[i] == 0 && PINC & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINC & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                             // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                         // flag that new data is available
        if(i+1 == nChannels) RC_data_rdy = HIGH;
      }
    }
  }
}

// READ INTERRUPTS ON PINS D0-7: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT2_vect){                                                 // this function will run if a pin change is detected on portD

  pciTime = micros();                                             // Record the time of the PIN change in microseconds

  for (int i = 0; i < nChannels; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 2){                                     // if the current channel belongs to portD
      
      if(prev_pinState[i] == 0 && PIND & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PIND & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == nChannels) RC_data_rdy = HIGH;
      }
    }
  }
}

/*
 *  RC OUTPUT FUNCTIONS
 */

boolean RC_avail(){
    boolean avail = RC_data_rdy;
    RC_data_rdy = LOW;                          // reset the flag
    return avail;
}

float RC_decode(int ch){
  
  if(ch < 0 || ch >= nChannels) return 0;     // if channel number is out of bounds return zero.                 

  // determine the pulse width calibration for the RC channel. The default is 1000, 1500 and 2000us.
  
  int min = rc_min[ch];
  int max = rc_max[ch];
  int mid = rc_mid[ch];
  
  float ch_output = calibrate(PW[ch], min, mid, max);       // calibrate the pulse width to the range -1 to 1.
  
  return ch_output;                                 
}

/*
 *  Receiver Calibration
 */

 // NEED TO SPEED UP

float calibrate(float Rx, int Min, int Mid, int Max){
  float calibrated;
  if (Rx >= Mid)
  {
    calibrated = map(Rx, Mid, Max, 0, 1000);  // map from 0% to 100% in one direction
    if(calibrated > 1000)
      calibrated = 1000;
  } else if (Rx == 0) {
    calibrated = 0;                           // neutral
  } else {
    calibrated = map(Rx, Min, Mid, -1000, 0); // map from 0% to -100% in the other direction
    if(calibrated < -1000)
      calibrated = -1000;
  }
  return calibrated * 0.001;
}


/*
 *  Quick print function of Rx channel input
 */

#ifdef DEBUG
void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
  for (int i = 0; i < nChannels; i++){
    Serial.print(" ch");Serial.print(i+1);
    Serial.print("  ");
    if(PW[i] < 1000) Serial.print(" ");
    Serial.print(PW[i]);
  }
  Serial.print(" ");
}
#endif

void print_decimal2percentage(float dec){
  int pc = dec*100;
  // the number and text will take up 6 charactors i.e ___3%_ or -100%_
  if (pc >= 0) Serial.print(" ");
  if (abs(pc) < 100) Serial.print(" ");
  if (abs(pc) < 10) Serial.print(" ");
  Serial.print(" ");Serial.print(pc);Serial.print("% ");
}

/*
 * GENERIC PWM FUNCTIONS
 */

unsigned long pin_time;
float pin_pwm;
float pin_period;

boolean PWM_read(int ch){
  if(ch < 0 && ch >= nChannels) return false;

  boolean avail = pwmFlag[ch];
  if (avail == HIGH){
    pwmFlag[ch] = LOW;
    noInterrupts();
    pin_time = pwmTimer[ch];
    pin_pwm = PW[ch];
    pin_period = pwmPeriod[ch];
    interrupts();
  }
  return avail;
}

unsigned long PWM_time(){return pin_time;}
float PWM_period(){return pin_period;}
float PWM(){return pin_pwm;}

float PWM_freq(){
  float freq;
  return freq = 1000000 / pin_period;  // frequency Hz
}

float PWM_duty(){
  float duty;
  duty = pin_pwm/pin_period;
  return duty;
}


// serial input buffer

// inputs of the form ED2LLLLLRRRRRBBBBB\n  LLLLL and RRRRR are signed integer right right justified with proceedings blanks
void process_input(char *data)
{
  char lefts[6], rights[6], lights[6];

  if(mode != MODE_ROS)
      return;

  if((strlen(data) == 18) && !strncmp(data, "ED2", 3)) {
    strncpy(lefts, data + 3, 5); lefts[5] = '\0';
    strncpy(rights, data + 8, 5); rights[5] = '\0';
    strncpy(lights, data + 13, 5); lights[5] = '\0';
    int right = atoi(rights);
    int left = atoi(lefts);
    int light = atoi(lights);

    ros_port = (float) left / 1000;
    ros_starboard = (float) right / 1000;
    ros_light = (float) light / 1000;

    // map to range -1..+1 for thrusters and 0..+1 for the light
    if(ros_port < -1)
      ros_port = -1;
    if(ros_port > 1)
      ros_port = 1;
    if(ros_starboard < -1)
      ros_starboard = -1;
    if(ros_starboard > 1)
      ros_starboard = 1;
    if (light < 0){
      light = 0;
    }
    if (light > 1) {
      light = 1;
    }
  }
}

const int MAX_BUFFER_LEN = 50;

void send_status(float left, float right)
{
  static char output_buffer[MAX_BUFFER_LEN]; // for efficiency

  strcpy(output_buffer, "ED2_001, ");
  
  switch(mode){
    case MODE_ROS:
      strcat(output_buffer, "ROS, ");
      break;
    case MODE_MAN:
      strcat(output_buffer, "MAN, ");
      break;
    case MODE_ESTOP:
      strcat(output_buffer, "STP, ");
  }

  char buf[15];

  dtostrf(left, 4, 6, buf);
  strcat(output_buffer, buf);
  strcat(output_buffer, ", ");

  dtostrf(right, 4, 6, buf);
  strcat(output_buffer, buf);
  strcat(output_buffer, ", ");

  dtostrf(ros_port, 4, 6, buf);
  strcat(output_buffer, buf);
  strcat(output_buffer, ", ");

  dtostrf(ros_starboard, 4, 6, buf);
  strcat(output_buffer, buf);
  strcat(output_buffer, ", ");

  dtostrf(ros_light, 4, 6, buf);
  strcat(output_buffer, buf);

  Serial.println(output_buffer);
}

void process_input(const byte inByte)
{
  static char input_buffer[MAX_BUFFER_LEN];
  static unsigned int pos = 0;

  switch(inByte) {
    case '\n':
      input_buffer[pos] = '\0';
      process_input(input_buffer);
      pos = 0;
      break;
    case '\r':
      break;
    default:
      if(pos < (MAX_BUFFER_LEN-1))
        input_buffer[pos++] = inByte;
  }
}

void setup() {
    Serial.begin(9600);

    setup_pwmRead();

# ifdef DEBUG
    Serial.println("Connecting to port ESC (no delay)");
# endif
    port.attach(port_pin);
    port.writeMicroseconds(THRUSTER_STOP);
# ifdef DEBUG
    Serial.println("Connecting to starboard ESC plus delay");
# endif
    starboard.attach(starboard_pin);
    starboard.writeMicroseconds(THRUSTER_STOP);
    delay(7000);
# ifdef DEBUG
    Serial.println("Connecting to light");
# endif
    light.attach(light_pin);
    light.writeMicroseconds(LIGHT_OFF);
# ifdef DEBUG
    Serial.println("Setup complete");
# endif
}


void loop() {
  
  static float rc_in[nChannels];  // keep old values and start with random ones

  while(Serial.available() > 0)
    process_input(Serial.read());

  if(RC_avail()) {
    for (int i = 0; i<nChannels; i++){ 
      rc_in[i] = RC_decode(i);
    }
  }  
  
# ifdef DEBUG
  print_RCpwm();
  Serial.print(" Mode "); Serial.print(mode); Serial.print(" ");
#else
  switch(mode){
    case MODE_ROS:
      send_status(ros_port, ros_starboard);
      break;
    case MODE_MAN:
      send_status(rc_in[LEFT_STICK], rc_in[RIGHT_STICK]);
      break;
    case MODE_ESTOP:
      send_status(0, 0);
      break;
  }
# endif

  // make the switches, switches
  if(rc_in[RUN_MODE] > 0)
    rc_in[RUN_MODE] = 1;
  else
    rc_in[RUN_MODE] = 0;
  if(rc_in[ESTOP_STATE] > 0)
    rc_in[ESTOP_STATE] = 1;
  else
    rc_in[ESTOP_STATE] = 0;

# ifdef DEBUG
  Serial.print("run mode "); Serial.print((int) rc_in[RUN_MODE]); Serial.print(" ");
  Serial.print("estop_state "); Serial.print((int) rc_in[ESTOP_STATE]); Serial.print(" ");
# endif

  // go to estop?
  if(rc_in[ESTOP_STATE] <= 0) {
    mode = MODE_ESTOP;
    estop_recover = 0;
    port.writeMicroseconds(THRUSTER_STOP);
    starboard.writeMicroseconds(THRUSTER_STOP);
  }

  switch(mode){
  case MODE_ROS:
    if(rc_in[RUN_MODE] > 0) {
      mode = MODE_MAN;
# ifdef DEBUG
      Serial.print("In ROS MODE moving to Manual ");
# endif
    } else {
      int posa = THRUSTER_STOP + gain * ros_port * THRUSTER_RANGE;
      port.writeMicroseconds(posa);
      int posb = THRUSTER_STOP + gain * ros_starboard * THRUSTER_RANGE;
      starboard.writeMicroseconds(posb);
      int posl = LIGHT_OFF + gain * ros_light * LIGHT_RANGE;
      light.write(posl);
# ifdef DEBUG
      Serial.print("In ROS mode port "); Serial.print(posa); Serial.print(" starboard "); Serial.print(posb); Serial.print(" light "); Serial.print(posl); Serial.print(" ");
# endif
    }
    break;
  case MODE_MAN:
    if(rc_in[RUN_MODE] <= 0) {
      mode = MODE_ROS;
# ifdef DEBUG
      Serial.print("In MAN MODE moving to ROS ");
# endif
    } else {
# ifdef DEBUG
      Serial.print("In MAN Mode ");  Serial.print(rc_in[LEFT_STICK]); Serial.print(" "); Serial.print(rc_in[RIGHT_STICK]); Serial.print(" ");
# endif
      int posa = THRUSTER_STOP + gain * rc_in[LEFT_STICK] * THRUSTER_RANGE;
      port.writeMicroseconds(posa);
      int posb = THRUSTER_STOP + gain * rc_in[RIGHT_STICK] * THRUSTER_RANGE;
      starboard.writeMicroseconds(posb);
      int posl = LIGHT_OFF;
      light.write(posl);
# ifdef DEBUG
      Serial.print("port "); Serial.print(posa); Serial.print(" starboard "); Serial.print(posb); Serial.print(" light "); Serial.print(posl); Serial.print(" ");
# endif
    }
    break;
  case MODE_ESTOP:
# ifdef DEBUG
      Serial.print("In ESTOP mode with recover mode "); Serial.print(estop_recover); Serial.print(" ");
# endif
      switch(estop_recover){
      case 0:
        if(rc_in[RUN_MODE] > 0)
          estop_recover = 1;
        break;
      case 1:
        if(rc_in[RUN_MODE] <= 0)
          estop_recover = 2;
        break;
      case 2:
        if(rc_in[RUN_MODE] > 0){
          mode = MODE_MAN;
          estop_recover = 0;
          port.writeMicroseconds(THRUSTER_STOP);
          starboard.writeMicroseconds(THRUSTER_STOP);
# ifdef DEBUG
          Serial.print("Moving to manual mode ");
# endif
        }
    }
  }
# ifdef DEBUG
  Serial.println("");
# endif
}
