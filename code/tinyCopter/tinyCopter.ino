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

#define MOTOR1
#define MOTOR2
#define MOTOR3
#define MOTOR4

void setup(){
  pinMode(status_led1, OUTPUT);  //Status LEDs are outputs
  pinMode(status_led2, OUTPUT);
  
  //pinMode(MOTOR1, OUTPUT);  //Motors are outputs
  //pinMode(MOTOR2, OUTPUT);
  //pinMode(MOTOR3, OUTPUT);
  //pinMode(MOTOR4, OUTPUT);
  
  Wire.begin();       //We are using TWIE on an xmega A4
  
  Serial.begin(38400); //Start serial for debugging and such
  print_tinyCopter_ASCII();
  
  //Now we are going to initialize our sensors
  Serial.println("Initializing sensors...");
  accel.initialize();
  gyro.initialize();
  
  delay(100);          //Wait a little bit for things to settle
  
  //We are going to test the connections to our sensors
  Serial.println("Testing sensors...");
  
  bool accel_test_status = accel.testConnection();
  delay(100);           //I think we need to wait for the transaction to take place before starting the next one
  bool gyro_test_status = gyro.testConnection();
  
  switch(accel_test_status){
    case true:
    Serial.println("Accelerometer all good!");
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
volatile unsigned int counter = 0;
volatile int state = 0;
ISR(TCD1_OVF_vect){
  pulse_widthA = TCE0.CCA;
  
  if(counter >= 2000){
    TC_SetCompareA(&TCD0, pulse_widthA);
    //Serial.println(TCD0.CCA);
  }
  
  if(counter >= 2400){
    TC_SetCompareB(&TCD0, 620);
  }
  
  if(counter >= 2800){
    TC_SetCompareC(&TCD0, 620);
  }
  
  if(counter >= 3200){
    TC_SetCompareD(&TCD0, 620);
  }
  
  counter += 1;
  //digitalWrite(18, state);
  //state = !state;
  
  
  //pulse_widthA = TCE0.CCA;
  //pulse_widthB = TCE0.CCB;
  Serial.println(pulse_widthA);
  //Serial.println(pulse_widthB);
  //TC_SetCompareA(&TCD0, pulse_widthA);
}





//Capture channel A
//volatile int pulse_width = 0;
//ISR(TCE0_CCD_vect){
//  pulse_width = TCE0.CCD;
//  Serial.println(pulse_width);
//}
