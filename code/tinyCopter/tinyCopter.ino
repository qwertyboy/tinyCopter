/////////////////////////////////////////////////////////
//					               //
//	     tinyCopter v1 - August 2012               //
//						       //
//	   www.qwertyboydesign.wordpress.com           //
//        youtube.com/user/qwertyboy1234567899         //
//						       //
/////////////////////////////////////////////////////////
//   _   _              _____            _             //
//  | | (_)            / ____|          | |            //
//  | |_ _ _ __  _   _| |     ___  _ __ | |_ ___ _ __  //
//  | __| | '_ \| | | | |    / _ \| '_ \| __/ _ \ '__| //
//  | |_| | | | | |_| | |___| (_) | |_) | ||  __/ |    //
//   \__|_|_| |_|\__, |\_____\___/| .__/ \__\___|_|    //
//                __/ |           | |                  //
// Nathan Duprey |___/            |_|        Version 1 //
//						       //
/////////////////////////////////////////////////////////


#include "Wire.h"      //I2Cdevlib depends on the Wire library
#include "I2Cdev.h"    //General operations for the sensor libraries
#include "ADXL345.h"   //ADXL345 library
#include "ITG3200.h"   //ITG-3200 library
#include "ascii_art.h" //Library for custom ASCII art
//#include "motors.h"    //Library for motor control
//#include "receiver.h"  //Library for reading receiver

#include "C:\Documents and Settings\Nathan\arduino-1.0.1-rc2-xmegaduino-beta4-original\libraries\AVR1305\pmic_driver.c"
#include "C:\Documents and Settings\Nathan\arduino-1.0.1-rc2-xmegaduino-beta4-original\libraries\AVR1305\pmic_driver.h"
#include "C:\Documents and Settings\Nathan\arduino-1.0.1-rc2-xmegaduino-beta4-original\libraries\AVR1306\TC_driver.c"
#include "C:\Documents and Settings\Nathan\arduino-1.0.1-rc2-xmegaduino-beta4-original\libraries\AVR1306\TC_driver.h"

ADXL345 accel;         //Using default addresses for both devices
ITG3200 gyro;

int accel_xout, accel_yout, accel_zout;  //Containers for raw acceleration data
int gyro_xout, gyro_yout, gyro_zout;     //Containers for raw gyroscope data

#define status_led1 13 //Pins for the status LEDs
#define status_led2 12

#define MOTOR1 0
#define MOTOR2 0
#define MOTOR3 0
#define MOTOR4 0

////////////////Target Angle Calculations////////////////

volatile int roll_input;
volatile int pitch_input;
volatile int throttle_input;
volatile int yaw_input;

#define ROLL_MIN 582
#define ROLL_MID 756
#define ROLL_MAX 950
#define PITCH_MIN 587
#define PITCH_MID 747
#define PITCH_MAX 926
#define THROTTLE_MIN 565
#define THROTTLE_MID 738
#define THROTTLE_THRESHOLD 0
#define THROTTLE_MAX 923
#define YAW_MIN 560
#define YAW_MID 750
#define YAW_MAX 921

#define XANGLE_RANGE 30
#define YANGLE_RANGE 30
#define ZRATE_RANGE 30

float TARGET_ZRATE = 0.0;
float throttle = 0.0;
float TARGET_XANGLE = 0.0;
float TARGET_YANGLE = 0.0;

float ACCEL_XANGLE;
float ACCEL_YANGLE;

float GYRO_XRATE;
float GYRO_YRATE;
float GYRO_ZRATE;
float GYRO_XANGLE;
float GYRO_YANGLE;
float GYRO_ZANGLE;

float filter_xterm[3] = {0,0,0};
float filter_yterm[3] = {0,0,0};
#define timeConstant 0.1
float COMPLEMENTARY_XANGLE;
float COMPLEMENTARY_YANGLE;

///////////////////////PID Control///////////////////////

float KP;
float KD;
float KI;
float ZKP;
float ZKD;

float XERROR;
float YERROR;
float ZERROR;

float PREVIOUS_XERROR = 0;
float PREVIOUS_YERROR = 0;
float PREVIOUS_ZERROR = 0;

float XDIFFERENTIAL = 0;
float YDIFFERENTIAL = 0;
float ZDIFFERENTIAL = 0;

float XINTEGRAL;
float YINTEGRAL;

float PID_XOUTPUT;
float PID_YOUTPUT;
float PID_ZOUTPUT;

//////////////////////Motor Control//////////////////////

#define MOTOR_MIN 510
#define MOTOR_MAX 950
#define MOTOR_RANGE 440

float motor1_speed;
float motor2_speed;
float motor3_speed;
float motor4_speed;

/////////////////////////////////////////////////////////

void setup(){
  pinMode(status_led1, OUTPUT);  //Status LEDs are outputs
  pinMode(status_led2, OUTPUT);
  
  //pinMode(MOTOR1, OUTPUT);  //Motors are outputs
  //pinMode(MOTOR2, OUTPUT);
  //pinMode(MOTOR3, OUTPUT);
  //pinMode(MOTOR4, OUTPUT);
  
  Wire.begin();       //We are using TWIC on an xmega A4
  
  Serial.begin(38400); //Start serial for debugging and such
  print_tinyCopter_ASCII();
  
  //Now we are going to initialize our sensors
  Serial.println("Initializing sensors...");
  accel.initialize();
  accel.setRange(0x1);  // +/-4g range
  gyro.initialize();
  
  delay(100);          //Wait a little bit for things to settle
  
  //We are going to test the connections to our sensors
  Serial.println("Testing sensors...");
  
  bool accel_test_status = accel.testConnection();
  byte accel_range = accel.getRange();
  delay(100);           //I think we need to wait for the transaction to take place before starting the next one
  bool gyro_test_status = gyro.testConnection();
  
  switch(accel_test_status){
    case true:
    Serial.println("Accelerometer all good!");
    Serial.print("Range: ");
    Serial.println(accel_range);
    break;
    
    case false:
    Serial.println("Accelerometer is broken!");
    while(1){
      digitalWrite(status_led1, HIGH);
      delay(100);
      digitalWrite(status_led1, LOW);
      delay(100);
    }
    break;
  }
  
  switch(gyro_test_status){
    case true:
    Serial.println("Gyroscope all good!");
    Serial.println("");
    break;
    
    case false:
    Serial.println("Gyroscope is broken!");
    while(1){
      digitalWrite(status_led2, HIGH);
      delay(100);
      digitalWrite(status_led2, LOW);
      delay(100);
    }
    break;
  }
  
  cli();
  setup_motorPWM_timer();
  setup_receiverCapture_timer();
  setup_mainloop_timer();
  sei();
}

void loop(){
}





//Function to set up interrupt for the main control loop
void setup_mainloop_timer(){
  TC_SetPeriod(&TCD1, 0x270F);                        //Set period register at 9999 to create 400Hz interrupt
  TC1_ConfigClockSource(&TCD1, TC_CLKSEL_DIV8_gc);    //Use F_CPU/8 as timer clock
  TC1_SetOverflowIntLevel(&TCD1, TC_OVFINTLVL_HI_gc); //Enable high-level interrupt on overflow
  
  //TCD1.CTRLA = (1<<0x08);      //Set timer E0 to use F_CPU/8 as clock - figure out why this doesn't work
  //TCD1.PER = 0x270F;            //Set period register at 9999 to cause an interrupt at 400Hz - this works
  //TCD1.INTCTRLA |= (1<<0X03);   //Set up timer as highest level interrupt (besides NMI) - figure out why this doesn't work
  //PMIC_EnableHighLevel();
  
  //ISR(TCD1_OVF_vect){       //This is what the ISR funtion should look like
  //}
  Serial.println("Main loop timer set up!");
}





//Function to set up interrupt for MOTOR1 PWM
void setup_motorPWM_timer(){
  PORTD.DIRSET = 0xFF;                               //Set PORTD pins as outputs. I thought the compare channels
                                                     //automatically did this...
  
  TC_SetPeriod(&TCD0, 0x270F);                       //Set period register at 9999 to create 50Hz(20ms) interrupt
  TC0_ConfigClockSource(&TCD0, TC_CLKSEL_DIV64_gc);  //Use F_CPU/64 as timer clock
  TC0_ConfigWGM(&TCD0, TC_WGMODE_SS_gc);             //Set timer for single slope PWM
  
  TC0_EnableCCChannels(&TCD0, TC0_CCAEN_bm);         //Enable compare channel A/MOTOR1
  TC0_EnableCCChannels(&TCD0, TC0_CCBEN_bm);         //Enable compare channel B/MOTOR2
  TC0_EnableCCChannels(&TCD0, TC0_CCCEN_bm);         //Enable compare channel C/MOTOR3
  TC0_EnableCCChannels(&TCD0, TC0_CCDEN_bm);         //Enable compare channel D/MOTOR4
  
  TC_SetCompareA(&TCD0, 0x1FE);                          //Set the compare register at 0 to start
  TC_SetCompareB(&TCD0, 0x1FE);                          //Set the compare register at 0 to start
  TC_SetCompareC(&TCD0, 0x1FE);                          //Set the compare register at 0 to start
  TC_SetCompareD(&TCD0, 0x1FE);                          //Set the compare register at 0 to start
                                                     //0x1FE(510) is minimum, 3FC(1020) is maximum(so far)
  
  Serial.println("Motor PWM timer set up!");
}





void setup_receiverCapture_timer(){  
  PORTE.DIRCLR = 0xFF;                              //Configure PE0-3 for input
  
  PORTE.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;           //Configure PE0 for triggering on both edges
  PORTE.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;           //Configure PE1 for triggering on both edges
  PORTE.PIN2CTRL = PORT_ISC_BOTHEDGES_gc;           //Configure PE2 for triggering on both edges
  PORTE.PIN3CTRL = PORT_ISC_BOTHEDGES_gc;           //Configure PE3 for triggering on both edges
  
  TC_SetPeriod(&TCE0, 0x7FFF);                      //Clear MSB of period register for detecting edge polarity 0x7FFF
  TC0_ConfigClockSource(&TCE0, TC_CLKSEL_DIV64_gc); //Use F_CPU/64 for a resolution of 2us
  
  TC0_EnableCCChannels(&TCE0, TC0_CCAEN_bm);        //Enable capture channel A
  TC0_EnableCCChannels(&TCE0, TC0_CCBEN_bm);        //Enable capture channel B
  TC0_EnableCCChannels(&TCE0, TC0_CCCEN_bm);        //Enable capture channel C
  TC0_EnableCCChannels(&TCE0, TC0_CCDEN_bm);        //Enable capture channel D

  EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN0_gc;         //Select PE0 as input to event channel 0
  EVSYS.CH1MUX = EVSYS_CHMUX_PORTE_PIN1_gc;         //Select PE1 as input to event channel 1
  EVSYS.CH2MUX = EVSYS_CHMUX_PORTE_PIN2_gc;         //Select PE2 as input to event channel 2
  EVSYS.CH3MUX = EVSYS_CHMUX_PORTE_PIN3_gc;         //Select PE3 as input to event channel 3
  
  TCE0.CTRLD |= TC_EVACT_PW_gc;                      //Enable pulse width capture event
  TCE0.CTRLD |= TC_EVSEL_CH0_gc;                     //Set event source to channel 0. still a bit confused here
  
  Serial.println("Receiver capture timer set up!");
}





//Main loop interrupt
volatile int pulse_widthA = 0;
volatile int pulse_widthB = 0;
volatile int pulse_widthC = 0;
volatile int pulse_widthD = 0;

volatile unsigned int counter = 0;
volatile int state = 0;

//400Hz Control loop
ISR(TCD1_OVF_vect){
  cli();
  
  getGyroRates();
  getAccelAngles();
  
  dataFilter();
  
  updatePID();
  updateMotors();
  
  sei();
}





//Capture channel A
//volatile int pulse_width = 0;
//ISR(TCE0_CCD_vect){
//  pulse_width = TCE0.CCD;
//  Serial.println(pulse_width);
//}

//Read RC receiver and get target angles
void getTargetAngles(){
  yaw_input = TCE0.CCD;
  TARGET_ZRATE = ZRATE_RANGE * (yaw_input - YAW_MID)/(YAW_MAX - YAW_MIN);
  
  throttle_input = TCE0.CCC;
  if(throttle_input <= THROTTLE_THRESHOLD){
    throttle = 0.0;
  }else{
    throttle = (throttle_input - THROTTLE_MIN)/(THROTTLE_MAX - THROTTLE_MIN);
  }
  
  roll_input = TCE0.CCA;
  TARGET_XANGLE = XANGLE_RANGE * (roll_input - ROLL_MID)/(ROLL_MAX - ROLL_MIN);
  
  pitch_input = TCE0.CCB;
  TARGET_YANGLE = YANGLE_RANGE * (pitch_input - PITCH_MID)/(PITCH_MAX - PITCH_MIN);
}

//Read accelerometer and calculate angles
void getAccelAngles(){
  accel.getAcceleration(&accel_xout, &accel_yout, &accel_zout);
  
  ACCEL_XANGLE = 57.295 * atan(accel_yout / sqrt(pow(accel_zout, 2) + pow(accel_xout, 2)));
  ACCEL_YANGLE = 57.295 * atan(-accel_xout / sqrt(pow(accel_zout, 2) + pow(accel_yout, 2)));
}

//Read gyroscope and calculate angles
void getGyroRates(){
  gyro.getRotation(&gyro_xout, &gyro_yout, &gyro_zout);
  
  GYRO_XRATE = gyro_xout / 66.5;
  GYRO_YRATE = gyro_yout / 66.5;
  GYRO_ZRATE = gyro_zout / 66.5;
  
  GYRO_XANGLE += GYRO_XRATE * 0.0025;
  GYRO_YANGLE += GYRO_YRATE * 0.0025;
  GYRO_ZANGLE += GYRO_ZRATE * 0.0025;
}

//2nd order complementary filter
void dataFilter(){
  filter_xterm[0] = (ACCEL_XANGLE - COMPLEMENTARY_XANGLE) * timeConstant * timeConstant;
  filter_yterm[0] = (ACCEL_YANGLE - COMPLEMENTARY_YANGLE) * timeConstant * timeConstant;
  
  filter_xterm[2] = (0.0025 * filter_xterm[0]) + filter_xterm[2];
  filter_yterm[2] = (0.0025 * filter_yterm[0]) + filter_yterm[2];
  
  filter_xterm[1] = filter_xterm[2] + (ACCEL_XANGLE - COMPLEMENTARY_XANGLE) * 2 * timeConstant + GYRO_XRATE;
  filter_yterm[1] = filter_yterm[2] + (ACCEL_YANGLE - COMPLEMENTARY_YANGLE) * 2 * timeConstant + GYRO_YRATE;
  
  COMPLEMENTARY_XANGLE = (0.0025 * filter_xterm[1]) + COMPLEMENTARY_XANGLE;
  COMPLEMENTARY_YANGLE = (0.0025 * filter_yterm[1]) + COMPLEMENTARY_YANGLE;
}

//PID algorithm
void updatePID(){
  PREVIOUS_XERROR = XERROR;
  PREVIOUS_YERROR = YERROR;
  PREVIOUS_ZERROR = ZERROR;

  XERROR = TARGET_XANGLE - COMPLEMENTARY_XANGLE;
  YERROR = TARGET_YANGLE - COMPLEMENTARY_YANGLE;
  ZERROR = TARGET_ZRATE - GYRO_ZRATE;

  //XDIFFERENTIAL = (XERROR - PREVIOUS_XERROR)/dt;	
  //YDIFFERENTIAL = (YERROR - PREVIOUS_YERROR)/dt;
  //ZDIFFERENTIAL = (ZERROR - PREVIOUS_ZERROR)/dt;

  XDIFFERENTIAL = -GYRO_XRATE;
  YDIFFERENTIAL = -GYRO_YRATE;
  ZDIFFERENTIAL = -GYRO_ZRATE;

  XINTEGRAL += XERROR * 0.0025;
  YINTEGRAL += YERROR * 0.0025;

  if(XINTEGRAL > 0.5) {XINTEGRAL = 0.5;}
  else if(XINTEGRAL < -0.5) {XINTEGRAL = -0.5;}
  if(YINTEGRAL > 0.5) {YINTEGRAL = 0.5;}
  else if(YINTEGRAL < -0.5) {YINTEGRAL = -0.5;}

  PID_XOUTPUT = XERROR * KP + XDIFFERENTIAL * KD + XINTEGRAL * KI;
  PID_YOUTPUT = YERROR * KP + YDIFFERENTIAL * KD + YINTEGRAL * KI;
  PID_ZOUTPUT = ZERROR * ZKP; + ZDIFFERENTIAL * ZKD;
  if(PID_ZOUTPUT > 1000){PID_ZOUTPUT = 1000;}
  else if (PID_ZOUTPUT < -1000){PID_ZOUTPUT = -1000;}	
}

void updateMotors(){
  if(throttle == 0.0){
    TC_SetCompareA(&TCD0, 0x1FE);                          //Set the compare register at 510
    TC_SetCompareB(&TCD0, 0x1FE);                          //Set the compare register at 510
    TC_SetCompareC(&TCD0, 0x1FE);                          //Set the compare register at 510
    TC_SetCompareD(&TCD0, 0x1FE);                          //Set the compare register at 510
  }else{
    motor1_speed = 0.7071 * PID_XOUTPUT + -0.7071 * PID_YOUTPUT + PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE * throttle;
    motor2_speed = 0.7071 * PID_XOUTPUT + 0.7071 * PID_YOUTPUT - PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE * throttle;
    motor3_speed = -0.7071 * PID_XOUTPUT + 0.7071 * PID_YOUTPUT + PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE * throttle;
    motor4_speed = -0.7071 * PID_XOUTPUT + -0.7071 * PID_YOUTPUT - PID_ZOUTPUT + MOTOR_MIN + MOTOR_RANGE * throttle;
    
    if(motor1_speed > MOTOR_MAX)  {motor1_speed = MOTOR_MAX;}
    if(motor2_speed > MOTOR_MAX)  {motor2_speed = MOTOR_MAX;}
    if(motor3_speed > MOTOR_MAX)  {motor3_speed = MOTOR_MAX;}
    if(motor4_speed > MOTOR_MAX)  {motor4_speed = MOTOR_MAX;}
    
    if(motor1_speed < MOTOR_MIN)  {motor1_speed = MOTOR_MIN;}
    if(motor2_speed < MOTOR_MIN)  {motor2_speed = MOTOR_MIN;}
    if(motor3_speed < MOTOR_MIN)  {motor3_speed = MOTOR_MIN;}
    if(motor4_speed < MOTOR_MIN)  {motor4_speed = MOTOR_MIN;}
    
    TC_SetCompareA(&TCD0, motor1_speed);
    TC_SetCompareB(&TCD0, motor2_speed);
    TC_SetCompareC(&TCD0, motor3_speed);
    TC_SetCompareD(&TCD0, motor4_speed);
  }
}

void calibrateESC(){
  int x = 0;
  
  for(x = 0; x < 400; x++){
    TC_SetCompareA(&TCD0, MOTOR_MAX);
    TC_SetCompareB(&TCD0, MOTOR_MAX);
    TC_SetCompareC(&TCD0, MOTOR_MAX);
    TC_SetCompareD(&TCD0, MOTOR_MAX);
    
    delayMicroseconds(250);
  }
  
  for(x = 0; x < 400; x++){
    TC_SetCompareA(&TCD0, MOTOR_MIN);
    TC_SetCompareB(&TCD0, MOTOR_MIN);
    TC_SetCompareC(&TCD0, MOTOR_MIN);
    TC_SetCompareD(&TCD0, MOTOR_MIN);
    
    delayMicroseconds(250);
  }
}
