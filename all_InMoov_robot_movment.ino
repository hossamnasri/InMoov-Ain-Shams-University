/* ================================================================= //
//  Ain Shams University - Faculty of Engineering                    //
//  Ain Shams Virtal Hospital                                        //
//  HCM - Human Centered Mechatronics Lab.                           //
//  InMoov Robot Project                                             //
//                                                                   //
//  This is a unique work for InMoov Robot                           //
//  at Human Centered Mechatronics (HCM) Lab - Ain Shams University  //
//  by:                                                              //
//  ☑ Aly Mostafa Hafez                                              //
//  ☑ Hossam Nasr Elghareeb                                          //
//                                                                   //
//  Date: JUNE 2022                                                  //
//                                                                   //
//  Hardware used:                                                   //
//  · Arduino Mega                                                   //
//  · 2 PCA9685 I2C Servo Drivers                                    //
//  · 24 Servomotors                                                 //
//  · HC-05 Bluetooth Module                                         //
//                                                                   //
//  Please check the datasheet example of PCA9685,                   //
//  it was written by Limor Fried/Ladyada for Adafruit Industries,   //
//  find at: http://www.adafruit.com/products/815                    //
//                                                                   //
// ================================================================- */

#include  <Wire.h>
#include  <Adafruit_PWMServoDriver.h>
#include  <SoftwareSerial.h>

/*****************************************************/
/*                  IMPORANT NOTE:                   */
/*    ALL POSITIONS VALUES ARE HARDWARE-LIMITED      */
/*       BY THE OUTER POTENTIOMETERS ANGLES          */
/*             THAT HAS BEEN DETECTED                */
/*      AFTER SERVOS HACKING AND CALIBRATION.        */
/*****************************************************/

// ===== HEAD Positions Limitations ===== //
#define HEAD_PITCH_MAX_POS          100  // Pitch
#define HEAD_PITCH_MIN_POS          300 
#define HEAD_YAW_MAX_POS            355  // Yaw
#define HEAD_YAW_MIN_POS            100
#define HEAD_JAW_MAX_POS            355  // Jaw
#define HEAD_JAW_MIN_POS            100
#define HEAD_EYES_MAX_POS           355  // Eyes
#define HEAD_EYES_MIN_POS           100

// ===== LEFT SHOULDER AND BICEPS Positions Limitations ===== //
#define LEFT_FRONT_RAISE_MAX_POS    100  // Front
#define LEFT_FRONT_RAISE_MIN_POS    290
#define LEFT_LATERAL_RAISE_MAX_POS  315  // Lateral
#define LEFT_LATERAL_RAISE_MIN_POS  200
#define LEFT_TORSIONAL_MAX_POS      220  // Torsional
#define LEFT_TORSIONAL_MIN_POS      460
#define LEFT_BICEPS_MAX_POS         180  // Bi
#define LEFT_BICEPS_MIN_POS         305

// ===== RIGHT SHOULDER AND BICEPS Positions Limitations ===== //
#define RIGHT_FRONT_RAISE_MAX_POS   380  // Front
#define RIGHT_FRONT_RAISE_MIN_POS   200
#define RIGHT_LATERAL_RAISE_MAX_POS 230  //Lateral   
#define RIGHT_LATERAL_RAISE_MIN_POS 345
#define RIGHT_TORSIONAL_MAX_POS     460  //Torsional
#define RIGHT_TORSIONAL_MIN_POS     240
#define RIGHT_BICEPS_MAX_POS        450  //Bi
#define RIGHT_BICEPS_MIN_POS        320


// Note: MAX position refers to Spread
// Note: MIN position refers to Fist

// ===== LEFT HAND Positions Limitations ===== //
#define LEFT_WRIST_MAX_POS          250  //Wrist
#define LEFT_WRIST_MIN_POS          400
#define LEFT_THUMP_MAX_POS          600  //Thump
#define LEFT_THUMP_MIN_POS          150
#define LEFT_INDEX_MAX_POS          600  //Index
#define LEFT_INDEX_MIN_POS          150
#define LEFT_MIDDLE_MAX_POS         600  //Middle
#define LEFT_MIDDLE_MIN_POS         150
#define LEFT_RING_MAX_POS           600  //Ring
#define LEFT_RING_MIN_POS           150
#define LEFT_PINKY_MAX_POS          480  //Pinky
#define LEFT_PINKY_MIN_POS          150

// ===== RIGHT HAND Positions Limitations ===== //
#define RIGHT_WRIST_MAX_POS         100  //Wrist
#define RIGHT_WRIST_MIN_POS         400
#define RIGHT_THUMP_MAX_POS         420  //Thump
#define RIGHT_THUMP_MIN_POS         100
#define RIGHT_INDEX_MAX_POS         480  //Index
#define RIGHT_INDEX_MIN_POS         100
#define RIGHT_MIDDLE_MAX_POS        480  //Middle
#define RIGHT_MIDDLE_MIN_POS        100
#define RIGHT_RING_MAX_POS          480  //Ring
#define RIGHT_RING_MIN_POS          100
#define RIGHT_PINKY_MAX_POS         370  //Pinky
#define RIGHT_PINKY_MIN_POS         100


/************************************************/

// ===== HEAD Pin Numbers ===== //
int head_pitch_pin_number    = 12;
int head_yaw_pin_number      = 12;
int head_jaw_pin_number      = 13;
int head_eyes_pin_number     = 13;


// ===== SHOULDERS & BICEPS Pin Numbers ===== //
int front_raise_pin_number   = 8;
int lateral_raise_pin_number = 9;
int torsional_pin_number     = 7;
int biceps_pin_number        = 6;

// ===== HANDS Pin Numbers ===== //
int wrist_pin_number         = 5;
int thump_pin_number         = 4;
int index_pin_number         = 3;
int middle_pin_number        = 2;
int ring_pin_number          = 1;
int pinky_pin_number         = 0;

/****************************************/

// Initial Position for Each Motor
int head_pitch_initial_pos          = 200;
int head_yaw_initial_pos            = 230;
int head_jaw_initial_pos            = 230;
int head_eyes_initial_pos           = 230;

int left_front_raise_initial_pos    = 290;
int left_lateral_raise_initial_pos  = 200;
int left_torsional_initial_pos      = 340;
int left_biceps_initial_pos         = 180; //180
int right_front_raise_initial_pos   = 200;
int right_lateral_raise_initial_pos = 345;
int right_torsional_initial_pos     = 350;
int right_biceps_initial_pos        = 320;

int left_wrist_spread_pos           = 320;
int left_thump_spread_pos           = 600;
int left_index_spread_pos           = 600;
int left_middle_spread_pos          = 600;
int left_ring_spread_pos            = 600;
int left_pinky_spread_pos           = 480;

int left_wrist_fist_pos             = 150;
int left_thump_fist_pos             = 150;
int left_index_fist_pos             = 150;
int left_middle_fist_pos            = 150;
int left_ring_fist_pos              = 150;
int left_pinky_fist_pos             = 150;

int right_wrist_spread_pos          = 320;
int right_thump_spread_pos          = 420;
int right_index_spread_pos          = 480;
int right_middle_spread_pos         = 480;
int right_ring_spread_pos           = 480;
int right_pinky_spread_pos          = 370;

int right_wrist_fist_pos            = 320;
int right_thump_fist_pos            = 100;
int right_index_fist_pos            = 100;
int right_middle_fist_pos           = 100;
int right_ring_fist_pos             = 100;
int right_pinky_fist_pos            = 100;

/****************************************/

//SoftwareSerial MyBlue(2, 3); // RX | TX
int  flag = 2;
int  counter = 320 ; // to control by bluetooth and change duty cycle
int  DutyToDegree = 0 ; // to transfer from Duty to Degree
char z = 0;
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver RightI2C = Adafruit_PWMServoDriver(0x40);
// you can also call it with a different address you want
Adafruit_PWMServoDriver LeftI2C = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

#define SERVOMIN   150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX   600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN      600   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX      2400  // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  // MyBlue.begin(9600);
  Serial.println("8 channel Servo test!");

  RightI2C.begin();
  LeftI2C.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  RightI2C.setOscillatorFrequency(27000000);
  RightI2C.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  RightI2C.setPWM(15, 0, counter); //rest
  delay(10);
  LeftI2C.setOscillatorFrequency(27000000);
  LeftI2C.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  LeftI2C.setPWM(15, 0, counter); //rest
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
  RightI2C.setPWM  (n, 0, pulse);
  LeftI2C.setPWM   (n, 0, pulse);
}

void loop()
{
  //Blue() ;

  if (Serial.available()) {
    z = Serial.read();

    if (z == 'a') {
      if (head_pitch_initial_pos < HEAD_PITCH_MIN_POS) {
        head_pitch_initial_pos += 5;
        RightI2C.setPWM(head_pitch_pin_number, 0, head_pitch_initial_pos); //rest
      }
    }
    else if (z == 'b') {
      if (head_pitch_initial_pos > HEAD_PITCH_MAX_POS) {
        head_pitch_initial_pos -= 5;
        RightI2C.setPWM(head_pitch_pin_number, 0, head_pitch_initial_pos); //rest
      }
    }
    else if (z == 'c') {
      if (head_yaw_initial_pos < HEAD_YAW_MAX_POS) {
        head_yaw_initial_pos += 5;
        LeftI2C.setPWM(head_yaw_pin_number, 0, head_yaw_initial_pos); //rest
      }
    }
    else if (z == 'd') {
      if (head_yaw_initial_pos > HEAD_YAW_MIN_POS) {
        head_yaw_initial_pos -= 5;
        LeftI2C.setPWM(head_yaw_pin_number, 0, head_yaw_initial_pos); //rest
      }
    }
    else if (z == 'e') {
      if (left_front_raise_initial_pos < LEFT_FRONT_RAISE_MIN_POS) {
        left_front_raise_initial_pos += 5;
        LeftI2C.setPWM(front_raise_pin_number, 0, left_front_raise_initial_pos); //rest
        Serial.println(left_front_raise_initial_pos);
      }
    }
    else if (z == 'f') {
      if (left_front_raise_initial_pos > LEFT_FRONT_RAISE_MAX_POS) {
        left_front_raise_initial_pos -= 5;
        LeftI2C.setPWM(front_raise_pin_number, 0, left_front_raise_initial_pos); //rest
        Serial.println(left_front_raise_initial_pos);
      }
    }
    else if (z == 'g') {
      if (left_lateral_raise_initial_pos < LEFT_LATERAL_RAISE_MAX_POS) {
        left_lateral_raise_initial_pos += 5;
        LeftI2C.setPWM(lateral_raise_pin_number, 0, left_lateral_raise_initial_pos); //rest
        Serial.println(left_lateral_raise_initial_pos);
      }
    }
    else if (z == 'h') {
      if (left_lateral_raise_initial_pos > LEFT_LATERAL_RAISE_MIN_POS) {
        left_lateral_raise_initial_pos -= 5;
        LeftI2C.setPWM(lateral_raise_pin_number, 0, left_lateral_raise_initial_pos); //rest
        Serial.println(left_lateral_raise_initial_pos);
      }
    }
    else if (z == 'i') {
      if (left_torsional_initial_pos < LEFT_TORSIONAL_MIN_POS) {
        left_torsional_initial_pos += 5;
        LeftI2C.setPWM(torsional_pin_number, 0, left_torsional_initial_pos); //rest
        Serial.println(left_torsional_initial_pos);
      }
    }
    else if (z == 'j') {
      if (left_torsional_initial_pos > LEFT_TORSIONAL_MAX_POS) {
        left_torsional_initial_pos -= 5;
        LeftI2C.setPWM(torsional_pin_number, 0, left_torsional_initial_pos); //rest
        Serial.println(left_torsional_initial_pos);
      }
    }

    else if (z == 'k') {
      if (left_biceps_initial_pos < LEFT_BICEPS_MIN_POS) {
        left_biceps_initial_pos += 5;
        LeftI2C.setPWM(biceps_pin_number, 0, left_biceps_initial_pos); //rest
        Serial.println(left_biceps_initial_pos);
      }
    }
    else if (z == 'l') {
      if (left_biceps_initial_pos > LEFT_BICEPS_MAX_POS) {
        left_biceps_initial_pos -= 5;
        LeftI2C.setPWM(biceps_pin_number, 0, left_biceps_initial_pos); //rest
        Serial.println(left_biceps_initial_pos);
      }
    }
    else if (z == 'm') {
      if (right_front_raise_initial_pos < RIGHT_FRONT_RAISE_MAX_POS) {
        right_front_raise_initial_pos += 5;
        RightI2C.setPWM(front_raise_pin_number, 0, right_front_raise_initial_pos); //rest
        Serial.println(right_front_raise_initial_pos);
      }
    }
    else if (z ==  'n') {
      if (right_front_raise_initial_pos > RIGHT_FRONT_RAISE_MIN_POS) {
        right_front_raise_initial_pos -= 5;
        RightI2C.setPWM(front_raise_pin_number, 0, right_front_raise_initial_pos); //rest
        Serial.println(right_front_raise_initial_pos);
      }
    }
    else if (z == 'o') {
      if (right_lateral_raise_initial_pos < RIGHT_LATERAL_RAISE_MIN_POS) {
        right_lateral_raise_initial_pos += 5;
        RightI2C.setPWM(lateral_raise_pin_number, 0, right_lateral_raise_initial_pos); //rest
        Serial.println(right_lateral_raise_initial_pos);
      }
    }
    else if (z == 'p') {
      if (right_lateral_raise_initial_pos > RIGHT_LATERAL_RAISE_MAX_POS) {
        right_lateral_raise_initial_pos -= 5;
        RightI2C.setPWM(lateral_raise_pin_number, 0, right_lateral_raise_initial_pos); //rest
        Serial.println(right_lateral_raise_initial_pos);
      }
    }
    else if (z == 'q') {
      if (right_torsional_initial_pos < RIGHT_TORSIONAL_MAX_POS) {
        right_torsional_initial_pos += 5;
        RightI2C.setPWM(torsional_pin_number, 0, right_torsional_initial_pos); //rest
        Serial.println(right_torsional_initial_pos);
      }
    }
    else if (z == 'r') {
      if (right_torsional_initial_pos > RIGHT_TORSIONAL_MIN_POS) {
        right_torsional_initial_pos -= 5;
        RightI2C.setPWM(torsional_pin_number, 0, right_torsional_initial_pos); //rest
        Serial.println(right_torsional_initial_pos);
      }
    }

    else if (z == 's') {
      if (right_biceps_initial_pos < RIGHT_BICEPS_MAX_POS) {
        right_biceps_initial_pos += 5;
        RightI2C.setPWM(biceps_pin_number, 0, right_biceps_initial_pos); //rest
        Serial.println(right_biceps_initial_pos);
      }
    }
    else if (z == 't') {
      if (right_biceps_initial_pos > RIGHT_BICEPS_MIN_POS) {
        right_biceps_initial_pos -= 5;
        RightI2C.setPWM(biceps_pin_number, 0, right_biceps_initial_pos); //rest
        Serial.println(right_biceps_initial_pos);
      }
    }
    else if (z == 'u') {
      if (head_jaw_initial_pos < HEAD_JAW_MIN_POS) {
        head_jaw_initial_pos += 5;
        RightI2C.setPWM(head_jaw_pin_number, 0, head_jaw_initial_pos); //rest
      }
    }
    else if (z == 'v') {
      if (head_jaw_initial_pos > HEAD_JAW_MAX_POS) {
        head_jaw_initial_pos -= 5;
        RightI2C.setPWM(head_jaw_pin_number, 0, head_jaw_initial_pos); //rest
      }
    }
    else if (z == 'w') {
      if (head_eyes_initial_pos < HEAD_EYES_MIN_POS) {
        head_eyes_initial_pos += 5;
        RightI2C.setPWM(head_eyes_pin_number, 0, head_eyes_initial_pos); //rest
      }
    }
    else if (z == 'x') {
      if (head_eyes_initial_pos > HEAD_EYES_MAX_POS) {
        head_eyes_initial_pos -= 5;
        RightI2C.setPWM(head_eyes_pin_number, 0, head_eyes_initial_pos); //rest
      }
    }
    //The following condition spreads all of the LEFT hand fingers
    else if (z == '1') {
      LeftI2C.setPWM(thump_pin_number,   0 , left_thump_spread_pos);
      LeftI2C.setPWM(index_pin_number,   0 , left_index_spread_pos);
      LeftI2C.setPWM(middle_pin_number,  0 , left_middle_spread_pos);
      LeftI2C.setPWM(ring_pin_number,    0 , left_ring_spread_pos);
      LeftI2C.setPWM(pinky_pin_number,   0 , left_pinky_spread_pos);
    }
    //The following condition fists all of the left hand fingers
    else if (z == '2') {
      LeftI2C.setPWM(thump_pin_number,   0 , left_thump_fist_pos);
      LeftI2C.setPWM(index_pin_number,   0 , left_index_fist_pos);
      LeftI2C.setPWM(middle_pin_number,  0 , left_middle_fist_pos);
      LeftI2C.setPWM(ring_pin_number,    0 , left_ring_fist_pos);
      LeftI2C.setPWM(pinky_pin_number,   0 , left_pinky_fist_pos);
    }
    //The following condition spreads all of the RGIHT hand fingers
    else if (z == '3') {
      RightI2C.setPWM(thump_pin_number,  0 , right_thump_spread_pos);
      RightI2C.setPWM(index_pin_number,  0 , right_index_spread_pos);
      RightI2C.setPWM(middle_pin_number, 0 , right_middle_spread_pos);
      RightI2C.setPWM(ring_pin_number,   0 , right_ring_spread_pos);
      RightI2C.setPWM(pinky_pin_number,  0 , right_pinky_spread_pos);
    }
    //The following condition fists all of the RIGHT hand fingers
    else if (z == '4') {
      RightI2C.setPWM(thump_pin_number,  0 , right_thump_fist_pos);
      RightI2C.setPWM(index_pin_number,  0 , right_index_fist_pos);
      RightI2C.setPWM(middle_pin_number, 0 , right_middle_fist_pos);
      RightI2C.setPWM(ring_pin_number,   0 , right_ring_fist_pos);
      RightI2C.setPWM(pinky_pin_number,  0 , right_pinky_fist_pos);
    }
    /*else if(z == 'y'){
      if( > ){
                               +/-= counter_vlue;
         I2C.setPWM(pin_number, 0,PWM); //rest
      }
      }*/
    else {
      z = 'z';
    }
  }
}
