/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

//HEAD Positions
#define HEAD_PITCH_MAX_POS 100
#define HEAD_PITCH_MIN_POS 450
int head_pitch_current_pos = 200;
int head_pitch_pin_number = 0; 

#define HEAD_YAW_MAX_POS 355
#define HEAD_YAW_MIN_POS 100
int head_yaw_current_pos = 230;
int head_yaw_pin_number = 0;


//LEFT SHOULDER & BICEPS
#define LEFT_FRONT_RAISE_MAX_POS 100
#define LEFT_FRONT_RAISE_MIN_POS 290
int left_front_raise_current_pos = 290;
int left_front_raise_pin_number = 4;

#define LEFT_LATERAL_RAISE_MAX_POS 315
#define LEFT_LATERAL_RAISE_MIN_POS 200
int left_lateral_raise_current_pos = 200;
int left_lateral_raise_pin_number = 5;


#define LEFT_TORSIONAL_MAX_POS 220
#define LEFT_TORSIONAL_MIN_POS 460
int left_torsional_current_pos = 340;
int left_torsional_pin_number = 6;


#define LEFT_BICEPS_MAX_POS 180
#define LEFT_BICEPS_MIN_POS 305
int left_biceps_current_pos = 180;
int left_biceps_pin_number = 3;


//RIGHT SHOULDER & BICEPS
#define RIGHT_FRONT_RAISE_MAX_POS 380
#define RIGHT_FRONT_RAISE_MIN_POS 200
int right_front_raise_current_pos = 200;
int right_front_pin_number = 4;

#define RIGHT_LATERAL_RAISE_MAX_POS 230
#define RIGHT_LATERAL_RAISE_MIN_POS 345
int right_lateral_raise_current_pos = 345;
int right_lateral_raise_pin_number = 5;


#define RIGHT_TORSIONAL_MAX_POS 460
#define RIGHT_TORSIONAL_MIN_POS 240
int right_torsional_current_pos = 350;
int right_torsional_pin_number = 6;

#define RIGHT_BICEPS_MAX_POS 450
#define RIGHT_BICEPS_MIN_POS 320
int right_biceps_current_pos = 320;
int right_biceps_pin_number = 3;


//LEFT HAND Positions
#define LEFT_WRIST_MAX_POS 450
#define LEFT_WRIST_MIN_POS 320
int left_wrist_current_pos = 320;
int left_wrist_pin_number = 10;


#define LEFT_THUMP_MAX_POS 450
#define LEFT_THUMP_MIN_POS 320
int left_thump_current_pos = 320;
int left_thump_pin_number = 11;


#define LEFT_INDEX_MAX_POS 450
#define LEFT_INDEX_MIN_POS 320
int left_index_current_pos = 320;
int left_index_pin_number = 12;


#define LEFT_MIDDLE_MAX_POS 450
#define LEFT_MIDDLE_MIN_POS 320
int left_middle_current_pos = 320;
int left_middle_pin_number = 13;

#define LEFT_RING_MAX_POS 450
#define LEFT_RING_MIN_POS 320
int left_ring_current_pos = 320;
int left_ring_pin_number = 14;

#define LEFT_PINKY_MAX_POS 450
#define LEFT_PINKY_MIN_POS 320
int left_pinky_current_pos = 320;
int left_pinky_pin_number = 15;


//RIGHT HAND Positions
#define RIGHT_WRIST_MAX_POS 450
#define RIGHT_WRIST_MIN_POS 320
int right_wrist_current_pos = 320;
int right_wrist_pin_number = 10;


#define RIGHT_THUMP_MAX_POS 450
#define RIGHT_THUMP_MIN_POS 320
int right_thump_current_pos = 320;
int right_thump_pin_number = 11;


#define RIGHT_INDEX_MAX_POS 450
#define RIGHT_INDEX_MIN_POS 320
int right_index_current_pos = 320;
int right_index_pin_number = 12;

#define RIGHT_MIDDLE_MAX_POS 450
#define RIGHT_MIDDLE_MIN_POS 320
int right_middle_current_pos = 320;
int right_middle_pin_number = 13;


#define RIGHT_RING_MAX_POS 450
#define RIGHT_RING_MIN_POS 320
int right_ring_current_pos = 320;
int right_ring_pin_number = 14;


#define RIGHT_PINKY_MAX_POS 450
#define RIGHT_PINKY_MIN_POS 320
int right_pinky_current_pos = 320;
int right_pinky_pin_number = 15;



//SoftwareSerial MyBlue(2, 3); // RX | TX 
int flag = 2; 
int counter = 320 ; // to control by bluetooth and change duty cycle 
int DutyToDegree = 0 ; // to transfer from Duty to Degree 
char x=0;
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
// you can also call it with a different address you want
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
 // MyBlue.begin(9600); 
  Serial.println("8 channel Servo test!");

  pwm1.begin();
  pwm2.begin();
//  pwm.setPWM(15, 0, counter); //rest
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
pwm1.setPWM(15, 0,counter); //rest
  delay(10);
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
pwm2.setPWM(15, 0,counter); //rest
  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm1.setPWM(n, 0, pulse);
  pwm2.setPWM(n, 0, pulse);
}

void loop() 
{

   //Blue() ; 
   
   if (Serial.available()){
    x=Serial.read();

    if(x=='a'){
      if(head_pitch_current_pos<HEAD_PITCH_MIN_POS){
    head_pitch_current_pos+=5;
     pwm2.setPWM(head_pitch_pin_number, 0,head_pitch_current_pos); //rest
      }
    }
    else if(x=='b'){
      if(head_pitch_current_pos>HEAD_PITCH_MAX_POS){
      head_pitch_current_pos-=5;
     pwm2.setPWM(head_pitch_pin_number, 0,head_pitch_current_pos); //rest
      }
    }
    else if(x=='c'){
      if(head_yaw_current_pos<HEAD_YAW_MAX_POS){
    head_yaw_current_pos+=5;
     pwm2.setPWM(head_yaw_pin_number, 0,head_yaw_current_pos); //rest
      }
    }
    else if(x=='d'){
      if(head_yaw_current_pos>HEAD_YAW_MIN_POS){
      head_yaw_current_pos-=5;
     pwm2.setPWM(head_yaw_pin_number, 0,head_yaw_current_pos); //rest
      }
    }
      
    else if(x=='e'){
      if(left_lateral_raise_current_pos<LEFT_LATERAL_RAISE_MAX_POS){
     left_lateral_raise_current_pos+=5;
     pwm1.setPWM(left_lateral_raise_pin_number, 0,left_lateral_raise_current_pos); //rest
      }
    }
    else if(x=='f'){
      if(left_lateral_raise_current_pos>LEFT_LATERAL_RAISE_MIN_POS){
      left_lateral_raise_current_pos-=5;
     pwm1.setPWM(left_lateral_raise_pin_number, 0,left_lateral_raise_current_pos); //rest
      }
    }
    else if(x=='g'){
      if(left_front_raise_current_pos<LEFT_FRONT_RAISE_MIN_POS){
      left_front_raise_current_pos+=5;
    pwm1.setPWM(left_front_raise_pin_number, 0,left_front_raise_current_pos); //rest
      }
    }
    else if(x=='h'){
      if(left_front_raise_current_pos>LEFT_FRONT_RAISE_MAX_POS){
      left_front_raise_current_pos-=5;
    pwm1.setPWM(left_front_raise_pin_number, 0,left_front_raise_current_pos); //rest
      }
    }
      
    else if(x=='i'){
      if(left_torsional_current_pos<LEFT_TORSIONAL_MIN_POS){
      left_torsional_current_pos+=5;
    pwm1.setPWM(6, 0,left_torsional_current_pos); //rest
      }
    }
    else if(x=='j'){
      if(left_torsional_current_pos>LEFT_TORSIONAL_MAX_POS){
      left_torsional_current_pos-=5;
    pwm1.setPWM(6, 0,left_torsional_current_pos); //rest
      }
    }

    else if(x=='k'){
      if(left_biceps_current_pos<LEFT_BICEPS_MIN_POS){
      left_biceps_current_pos+=5;
    pwm1.setPWM(3, 0,left_biceps_current_pos); //rest
      }
    }
    else if(x=='l'){
      if(left_biceps_current_pos>LEFT_BICEPS_MAX_POS){
      left_biceps_current_pos-=5;
    pwm1.setPWM(3, 0,left_biceps_current_pos); //rest
      }
    }
    else if(x=='m'){
      if(right_lateral_raise_current_pos<RIGHT_LATERAL_RAISE_MIN_POS){//RIGHT_LATRAL_RISE_MAX_POS
      right_lateral_raise_current_pos+=5;
    pwm2.setPWM(5, 0,right_lateral_raise_current_pos); //rest
      }
    }
    else if(x=='n'){
      if(right_lateral_raise_current_pos>RIGHT_LATERAL_RAISE_MAX_POS){
      right_lateral_raise_current_pos-=5;
    pwm2.setPWM(5, 0,right_lateral_raise_current_pos); //rest
      }
    }

    else if(x=='o'){
     if(right_front_raise_current_pos<RIGHT_FRONT_RAISE_MAX_POS){
     right_front_raise_current_pos+=5;
    pwm2.setPWM(12, 0,right_front_raise_current_pos); //rest
      }
    }
    else if(x=='p'){
      if(right_front_raise_current_pos>RIGHT_FRONT_RAISE_MIN_POS){
      right_front_raise_current_pos-=5;
    pwm2.setPWM(12, 0,right_front_raise_current_pos); //rest
      }
    }
      
    else if(x=='q'){
      if(right_torsional_current_pos<RIGHT_TORSIONAL_MAX_POS){
      right_torsional_current_pos+=5;
    pwm2.setPWM(14, 0,right_torsional_current_pos); //rest
      }
    }
    else if(x=='r'){
      if(right_torsional_current_pos>RIGHT_TORSIONAL_MIN_POS){
      right_torsional_current_pos-=5;
    pwm2.setPWM(14, 0,right_torsional_current_pos); //rest
      }
    }

    else if(x=='s'){
      if(right_biceps_current_pos<RIGHT_BICEPS_MAX_POS){
      right_biceps_current_pos+=5;
    pwm2.setPWM(3, 0,right_biceps_current_pos); //rest
      }
    }
    else if(x=='t'){
      if(right_biceps_current_pos>RIGHT_BICEPS_MIN_POS){
      right_biceps_current_pos-=5;
    pwm2.setPWM(3, 0,right_biceps_current_pos); //rest
      }
    }
    else if(x=='1'){
      if(left_index_current_pos<LEFT_INDEX_MAX_POS){
      left_index_current_pos+=5;
    pwm2.setPWM(3, 0,left_index_current_pos); //rest
      }
    }
    else if(x=='2'){
      if(left_index_current_pos>LEFT_INDEX_MIN_POS){
      left_index_current_pos-=5;
    pwm2.setPWM(3, 0,left_index_current_pos); //rest
      }
    }
    else if(x=='t'){
      if(left_middle_current_pos<LEFT_MIDDLE_MAX_POS){
      left_middle_current_pos+=5;
    pwm2.setPWM(3, 0,right_biceps_current_pos); //rest
      }
    }
    else if(x=='t'){
      if(left_middle_current_pos>LEFT_MIDDLE_MIN_POS){
      left_midddle_current_pos-=5;
    pwm2.setPWM(3, 0,right_biceps_current_pos); //rest
      }
    }
    else{
        x='x';
      }
    }
