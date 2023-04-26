#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "VL53L0X.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

VL53L0X tof;

#define MODE 3

#define TURN 4

#define TIME 6000
#define TIMEWALKING 70
#define TIMEROTATING 70
#define RISE_LEG 45
#define DISTANCE 20

struct servo
{
  int num;

  float pwm_90;
  float pwm_0;

  int pwm_now;
};

typedef struct
{
  int state, new_state;

  unsigned long tes, tis;
} fsm_t;

uint8_t turn = 0;
  
fsm_t fsm_0, fsm_1, fsm_2, fsm_3, fsm_4, fsm_5;
  
servo servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7, servo_8;

unsigned long entry, end;

float distance;

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) // if the state changed tis is reset
  {  
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

void setup() 
{
  servo_1.num = 3;
  servo_2.num = 2;
  servo_3.num = 1;
  servo_4.num = 0;
  servo_5.num = 7;
  servo_6.num = 6;
  servo_7.num = 5;
  servo_8.num = 4;

  servo_1.pwm_90 = 124;
  servo_2.pwm_90 = 432;
  servo_3.pwm_90 = 447;
  servo_4.pwm_90 = 265;
  servo_5.pwm_90 = 243;
  servo_6.pwm_90 = 278;
  servo_7.pwm_90 = 305;
  servo_8.pwm_90 = 435;

  servo_1.pwm_0 = 312;
  servo_2.pwm_0 = 240;
  servo_3.pwm_0 = 255;
  servo_4.pwm_0 = 450;
  servo_5.pwm_0 = 56;
  servo_6.pwm_0 = 475;
  servo_7.pwm_0 = 487;
  servo_8.pwm_0 = 255;

  Serial.begin(115200);

  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();

  //tof.setBus(&Wire1);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  /*tof.setTimeout(500);

  if(!tof.init()) 
  {
    Serial.println(("Failed to detect and initialize VL53L0X!"));
  }

  tof.startContinuous();  */

  set_state(fsm_0, 0);
  set_state(fsm_1, 11);
  set_state(fsm_2, 10);
  set_state(fsm_3, 10);
  set_state(fsm_4, 9);
  set_state(fsm_5, 9);

}

//-------------------------------CÁLCULO PWM--------------------------------------------------------------------------------------------

void pwm_calculation(servo& servo, float angle)
{
  servo.pwm_now = servo.pwm_0 + angle * (servo.pwm_90 - servo.pwm_0) / 90;
}

//-------------------------------POSIÇÕES WALK FORWARD----------------------------------------------------------------------------------

void initial_position()
{
  pwm_calculation(servo_5, 90);
  pwm_calculation(servo_7, 90);
}

void second_position()
{
  pwm_calculation(servo_2, 60);
  pwm_calculation(servo_6, RISE_LEG);
}

void third_position()
{
  pwm_calculation(servo_6, 90);
}

void fourth_position()
{
  pwm_calculation(servo_1, 45);
  pwm_calculation(servo_2, 45);
  pwm_calculation(servo_3, 60);
  pwm_calculation(servo_4, 0);
}

void fifth_position()
{
  pwm_calculation(servo_3, 0);
  pwm_calculation(servo_7, RISE_LEG);
}

void sixth_position()
{
  pwm_calculation(servo_7, 90);
}

void seventh_position()
{
  pwm_calculation(servo_4, 60);
  pwm_calculation(servo_8, RISE_LEG);
}

void eighth_position()
{
  pwm_calculation(servo_8, 90);
}

void ninth_position()
{
  pwm_calculation(servo_1, 60);
  pwm_calculation(servo_2, 0);
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_4, 45);
}

void tenth_position()
{
  pwm_calculation(servo_1, 0);
  pwm_calculation(servo_5, RISE_LEG);
}

void walk_forward_prepare_first()
{
  pwm_calculation(servo_4, 45);
  pwm_calculation(servo_8, 60);
}

void walk_forward_prepare_second()
{
  pwm_calculation(servo_1, 0);
  pwm_calculation(servo_5, 60);
  pwm_calculation(servo_8, 90);
}

void walk_forward_prepare_third()
{
  pwm_calculation(servo_2, 0);
  pwm_calculation(servo_6, 60);
  pwm_calculation(servo_5, 90);
}

void walk_forward_prepare_fourth()
{
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_7, 60);
  pwm_calculation(servo_6, 90);
}

//-------------------------------POSIÇÕES WALK RIGHT------------------------------------------------------------------------------------

void walk_right_initial_position()
{
  pwm_calculation(servo_7, 90);
}

void walk_right_second_position()
{
  pwm_calculation(servo_1, 30);
  pwm_calculation(servo_5, 45);
}

void walk_right_third_position()
{
  pwm_calculation(servo_5, 90);
}

void walk_right_fourth_position()
{
  pwm_calculation(servo_1, 45);
  pwm_calculation(servo_2, 90);
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_4, 0);
}

void walk_right_fifth_position()
{
  pwm_calculation(servo_4, 90);
  pwm_calculation(servo_8, 45);
}

void walk_right_sixth_position()
{
  pwm_calculation(servo_8, 90);
}

void walk_right_seventh_position()
{
  pwm_calculation(servo_2, 30);
  pwm_calculation(servo_6, 45);
}

void walk_right_eighth_position()
{
  pwm_calculation(servo_6, 90);
}

void walk_right_ninth_position()
{
  pwm_calculation(servo_1, 90);
  pwm_calculation(servo_2, 45);
  pwm_calculation(servo_3, 30);
  pwm_calculation(servo_4, 45);
}

void walk_right_tenth_position()
{
  pwm_calculation(servo_3, 90);
  pwm_calculation(servo_7, 45);
}

void walk_right_prepare_first()
{
  pwm_calculation(servo_4, 45);
  pwm_calculation(servo_8, 60);
}

void walk_right_prepare_second()
{
  pwm_calculation(servo_1, 90);
  pwm_calculation(servo_5, 60);
  pwm_calculation(servo_8, 90);
}

void walk_right_prepare_third()
{
  pwm_calculation(servo_2, 45);
  pwm_calculation(servo_6, 60);
  pwm_calculation(servo_5, 90);
}

void walk_right_prepare_fourth()
{
  pwm_calculation(servo_3, 90);
  pwm_calculation(servo_7, 60);
  pwm_calculation(servo_6, 90);
}

//-------------------------------POSIÇÕES WALK LEFT-------------------------------------------------------------------------------------

void walk_left_initial_position()
{
  pwm_calculation(servo_6, 90);
  pwm_calculation(servo_7, 90);
}

void walk_left_second_position()
{
  pwm_calculation(servo_4, 30);
  pwm_calculation(servo_8, 45);
}

void walk_left_third_position()
{
  pwm_calculation(servo_8, 90);
}

void walk_left_fourth_position()
{
  pwm_calculation(servo_1, 30);
  pwm_calculation(servo_2, 45);
  pwm_calculation(servo_3, 90);
  pwm_calculation(servo_4, 45);
}

void walk_left_fifth_position()
{
  pwm_calculation(servo_1, 90);
  pwm_calculation(servo_5, 45);
}

void walk_left_sixth_position()
{
  pwm_calculation(servo_5, 90);
}

void walk_left_seventh_position()
{
  pwm_calculation(servo_3, 30);
  pwm_calculation(servo_7, 45);
}

void walk_left_eighth_position()
{
  pwm_calculation(servo_7, 90);
}

void walk_left_ninth_position()
{
  pwm_calculation(servo_1, 45);
  pwm_calculation(servo_2, 30);
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_4, 90);
}

void walk_left_tenth_position()
{
  pwm_calculation(servo_2, 90);
  pwm_calculation(servo_6, 45);
}

void walk_left_prepare_first()
{
  pwm_calculation(servo_4, 90);
  pwm_calculation(servo_8, 60);
}

void walk_left_prepare_second()
{
  pwm_calculation(servo_1, 45);
  pwm_calculation(servo_5, 60);
  pwm_calculation(servo_8, 90);
}

void walk_left_prepare_third()
{
  pwm_calculation(servo_2, 90);
  pwm_calculation(servo_6, 60);
  pwm_calculation(servo_5, 90);
}

void walk_left_prepare_fourth()
{
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_7, 60);
  pwm_calculation(servo_6, 90);
}

//-------------------------------POSIÇÕES ROTATE RIGHT----------------------------------------------------------------------------------

void right_first_leg_1()
{
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_7, 60);
}

void right_first_leg_2()
{
  pwm_calculation(servo_7, 90);
}

void right_second_leg_1()
{
  pwm_calculation(servo_2, 45);
  pwm_calculation(servo_6, 60);
}

void right_second_leg_2()
{
  pwm_calculation(servo_6, 90);
}

void right_third_leg_1()
{
  pwm_calculation(servo_1, 45);
  pwm_calculation(servo_5, 60);
}

void right_third_leg_2()
{
  pwm_calculation(servo_5, 90);
}

void right_forth_leg_1()
{
  pwm_calculation(servo_4, 45);
  pwm_calculation(servo_8, 60);
}

void right_forth_leg_2()
{
  pwm_calculation(servo_8, 60);
}

void right_rotate_position()
{
  pwm_calculation(servo_1, 0);
  pwm_calculation(servo_2, 90);
  pwm_calculation(servo_3, 90);
  pwm_calculation(servo_4, 0);
  pwm_calculation(servo_5, 90);
  pwm_calculation(servo_6, 90);
  pwm_calculation(servo_7, 90);
  pwm_calculation(servo_8, 90);
}

//-------------------------------POSIÇÕES ROTATE LEFT----------------------------------------------------------------------------------

void left_first_leg_1()
{
  pwm_calculation(servo_3, 45);
  pwm_calculation(servo_7, 60);
}

void left_first_leg_2()
{
  pwm_calculation(servo_7, 90);
}

void left_second_leg_1()
{
  pwm_calculation(servo_2, 45);
  pwm_calculation(servo_6, 60);
}

void left_second_leg_2()
{
  pwm_calculation(servo_6, 90);
}

void left_third_leg_1()
{
  pwm_calculation(servo_1, 45);
  pwm_calculation(servo_5, 60);
}

void left_third_leg_2()
{
  pwm_calculation(servo_5, 90);
}

void left_forth_leg_1()
{
  pwm_calculation(servo_4, 45);
  pwm_calculation(servo_8, 60);
}

void left_forth_leg_2()
{
  pwm_calculation(servo_8, 90);
}

void left_rotate_position()
{
  pwm_calculation(servo_1, 90);
  pwm_calculation(servo_2, 0);
  pwm_calculation(servo_3, 0);
  pwm_calculation(servo_4, 90);
  pwm_calculation(servo_5, 90);
  pwm_calculation(servo_6, 90);
  pwm_calculation(servo_7, 90);
  pwm_calculation(servo_8, 90);
}

void loop() 
{
    unsigned long cur_time = millis();

    tof.readRangeContinuousMillimeters() * 1e-1;
    
    fsm_0.tis = cur_time - fsm_0.tes;
    fsm_1.tis = cur_time - fsm_1.tes;
    fsm_2.tis = cur_time - fsm_2.tes;
    fsm_3.tis = cur_time - fsm_3.tes;
    fsm_4.tis = cur_time - fsm_4.tes;
    fsm_5.tis = cur_time - fsm_5.tes;

   //-------------------------------MÁQUINA 0-----------------------------------------------------------------------------------------

    if(fsm_0.state == 0 && MODE == 1 && distance < DISTANCE)
    {
      fsm_0.new_state = 1;
    }
    else if(fsm_0.state == 1 && turn == TURN)
    {
      fsm_0.new_state = 0;
      turn = 0;
    }
    else if(fsm_0.state == 0 && MODE == 2 && distance < DISTANCE)
    {
      fsm_0.new_state = 2;
    }
    else if(fsm_0.state == 2 && distance > (DISTANCE + 5))
    {
      fsm_0.new_state = 3;
    }
    else if(fsm_0.state == 3 && fsm_0.tis > TIME)
    {
      fsm_0.new_state = 0;
    }
    else if(fsm_0.state == 0 && MODE == 3 && distance < DISTANCE)
    {
      fsm_0.new_state = 4;
      entry = cur_time;
    }
    else if(fsm_0.state == 4 && distance > (DISTANCE + 5))
    {
      fsm_0.new_state = 5;
    }
    else if(fsm_0.state == 5 && fsm_0.tis > TIME)
    {
      fsm_0.new_state = 6;
      end = cur_time;
    }
    else if(fsm_0.state == 6 && turn == TURN)
    {
      fsm_0.new_state = 7;
      turn = 0;
    }
    else if(fsm_0.state == 7 && distance < DISTANCE)
    {
      fsm_0.new_state = 8;
    }
    else if(fsm_0.state == 8 && distance > (DISTANCE + 5))
    {
      fsm_0.new_state = 9;
    }
    else if(fsm_0.state == 9 && fsm_0.tis > TIME)
    {
      fsm_0.new_state = 10;
    }
    else if(fsm_0.state == 10 && turn == TURN)
    {
      fsm_0.new_state = 11;
      turn = 0;
    }
    else if(fsm_0.state == 11 && fsm_0.tis > (end - entry))
    {
      fsm_0.new_state = 0;
    }
  
    //-------------------------------MÁQUINA WALKING FORWARD---------------------------------------------------------------------------

    if(fsm_1.state == 0 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 1;
    }
    else if(fsm_1.state == 1 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 2;
    }
    else if(fsm_1.state == 2 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 3;
    }
    else if(fsm_1.state == 3 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 4;
    }
    else if(fsm_1.state == 4 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 5;
    }
    else if(fsm_1.state == 5 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 6;
    }
    else if(fsm_1.state == 6 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 7;
    }
    else if(fsm_1.state == 7 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 8;
    }
    else if(fsm_1.state == 8 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 9;
    }
    else if(fsm_1.state == 9 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 0;
    }
    else if(fsm_0.state != 0)
    {
      fsm_1.new_state = 10;
    }
    else if(fsm_1.state == 10 && fsm_0.state == 0)
    {
      fsm_1.new_state = 11;
    }
    else if(fsm_1.state == 11 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 12;
    }
    else if(fsm_1.state == 12 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 13;
    }
    else if(fsm_1.state == 13 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 14;
    }
    else if(fsm_1.state == 14 && fsm_1.tis > TIMEWALKING && fsm_0.state == 0)
    {
      fsm_1.new_state = 0;
    }

    //-------------------------------MÁQUINA WALKING RIGHT-----------------------------------------------------------------------------

    if(fsm_2.state == 0 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 1;
    }
    else if(fsm_2.state == 1 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 2;
    }
    else if(fsm_2.state == 2 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 3;
    }
    else if(fsm_2.state == 3 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 4;
    }
    else if(fsm_2.state == 4 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 5;
    }
    else if(fsm_2.state == 5 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 6;
    }
    else if(fsm_2.state == 6 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 7;
    }
    else if(fsm_2.state == 7 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 8;
    }
    else if(fsm_2.state == 8 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 9;
    }
    else if(fsm_2.state == 9 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 0;
    }
    else if(fsm_0.state != 2 && fsm_0.state != 3 && fsm_0.state != 4 && fsm_0.state != 5 && fsm_0.state != 7 && fsm_0.state != 8 && fsm_0.state != 9)
    {
      fsm_2.new_state = 10;
    }
    else if(fsm_2.state == 10 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 11;
    }
    else if(fsm_2.state == 11 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 12;
    }
    else if(fsm_2.state == 12 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 13;
    }
    else if(fsm_2.state == 13 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 14;
    }
    else if(fsm_2.state == 14 && fsm_2.tis > TIMEWALKING && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      fsm_2.new_state = 0;
    }

    //-------------------------------MÁQUINA WALKING LEFT------------------------------------------------------------------------------

    if(fsm_3.state == 0 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 1;
    }
    else if(fsm_3.state == 1 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 2;
    }
    else if(fsm_3.state == 2 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 3;
    }
    else if(fsm_3.state == 3 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 4;
    }
    else if(fsm_3.state == 4 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 5;
    }
    else if(fsm_3.state == 5 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 6;
    }
    else if(fsm_3.state == 6 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 7;
    }
    else if(fsm_3.state == 7 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 8;
    }
    else if(fsm_3.state == 8 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 9;
    }
    else if(fsm_3.state == 9 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 0;
    }
    else if(fsm_0.state != 11)
    {
      fsm_3.new_state = 10;
    }
    else if(fsm_3.state == 10 && fsm_0.state == 11)
    {
      fsm_3.new_state = 11;
    }
    else if(fsm_3.state == 11 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 12;
    }
    else if(fsm_3.state == 12 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 13;
    }
    else if(fsm_3.state == 13 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 14;
    }
    else if(fsm_3.state == 14 && fsm_3.tis > TIMEWALKING && fsm_0.state == 11)
    {
      fsm_3.new_state = 0;
    }

    //-------------------------------MÁQUINA ROTATE RIGHT------------------------------------------------------------------------------

    if(fsm_4.state == 0 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 1;
    }
    else if(fsm_4.state == 1 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 2;
    }
    else if(fsm_4.state == 2 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 3;
    }
    else if(fsm_4.state == 3 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 4;
    }
    else if(fsm_4.state == 4 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 5;
    }
    else if(fsm_4.state == 5 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 6;
    }
    else if(fsm_4.state == 6 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 7;
    }
    else if(fsm_4.state == 7 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 8;
    }
    else if(fsm_4.state == 8 && fsm_4.tis > TIMEROTATING && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      turn++;
      fsm_4.new_state = 0;
    }
    else if(fsm_0.state != 1 && fsm_0.state != 10)
    {
      fsm_4.new_state = 9;
    }
    else if(fsm_4.state == 9 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      fsm_4.new_state = 0;
    }

    //-------------------------------MÁQUINA ROTATE LEFT-------------------------------------------------------------------------------

    if(fsm_5.state == 0 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 1;
    }
    else if(fsm_5.state == 1 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 2;
    }
    else if(fsm_5.state == 2 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 3;
    }
    else if(fsm_5.state == 3 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 4;
    }
    else if(fsm_5.state == 4 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 5;
    }
    else if(fsm_5.state == 5 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 6;
    }
    else if(fsm_5.state == 6 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 7;
    }
    else if(fsm_5.state == 7 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      fsm_5.new_state = 8;
    }
    else if(fsm_5.state == 8 && fsm_5.tis > TIMEROTATING && fsm_0.state == 6)
    {
      turn++;
      fsm_5.new_state = 0;
    }
    else if(fsm_0.state != 6)
    {
      fsm_5.state = 9;
    }
    else if(fsm_5.state == 9 && fsm_0.state == 6)
    {
      fsm_5.state = 0;
    }

    //-------------------------------UPDATES STATES------------------------------------------------------------------------------------

    set_state(fsm_0, fsm_0.new_state);
    set_state(fsm_1, fsm_1.new_state);
    set_state(fsm_2, fsm_2.new_state);
    set_state(fsm_3, fsm_3.new_state);
    set_state(fsm_4, fsm_4.new_state);
    set_state(fsm_5, fsm_5.new_state);

    //-------------------------------UPDATES ACTIONS WALKING FORWARD-------------------------------------------------------------------

    if(fsm_1.state == 0 && fsm_0.state == 0)
    {
      initial_position();
    }
    else if(fsm_1.state == 1 && fsm_0.state == 0)
    {
      second_position();
    }
    else if(fsm_1.state == 2 && fsm_0.state == 0)
    {
      third_position();
    }
    else if(fsm_1.state == 3 && fsm_0.state == 0)
    {
      fourth_position();
    }
    else if(fsm_1.state == 4 && fsm_0.state == 0)
    {
      fifth_position();
    }
    else if(fsm_1.state == 5 && fsm_0.state == 0)
    {
      sixth_position();
    }
    else if(fsm_1.state == 6 && fsm_0.state == 0)
    {
      seventh_position();
    }
    else if(fsm_1.state == 7 && fsm_0.state == 0)
    {
      eighth_position();
    }
    else if(fsm_1.state == 8 && fsm_0.state == 0)
    {
      ninth_position();
    }
    else if(fsm_1.state == 9 && fsm_0.state == 0)
    {
      tenth_position();
    }
    else if(fsm_1.state == 11 && fsm_0.state == 0)
    {
      walk_forward_prepare_first();
    }
    else if(fsm_1.state == 12 && fsm_0.state == 0)
    {
      walk_forward_prepare_second();
    }
    else if(fsm_1.state == 13 && fsm_0.state == 0)
    {
      walk_forward_prepare_third();
    }
    else if(fsm_1.state == 14 && fsm_0.state == 0)
    {
      walk_forward_prepare_fourth();
    }

    //-------------------------------UPDATES ACTIONS WALKING RIGHT-----------------------------------------------------------------

    if(fsm_2.state == 0 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_initial_position();
    }
    else if(fsm_2.state == 1 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_second_position();
    }
    else if(fsm_2.state == 2 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_third_position();
    }
    else if(fsm_2.state == 3 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_fourth_position();
    }
    else if(fsm_2.state == 4 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_fifth_position();
    }
    else if(fsm_2.state == 5 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_sixth_position();
    }
    else if(fsm_2.state == 6 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_seventh_position();
    }
    else if(fsm_2.state == 7 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_eighth_position();
    }
    else if(fsm_2.state == 8 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_ninth_position();
    }
    else if(fsm_2.state == 9 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_tenth_position();
    }
    else if(fsm_2.state == 11 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_prepare_first();
    }
    else if(fsm_2.state == 12 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_prepare_second();
    }
    else if(fsm_2.state == 13 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_prepare_third();
    }
    else if(fsm_2.state == 14 && (fsm_0.state == 2 || fsm_0.state == 3 || fsm_0.state == 4 || fsm_0.state == 5 || fsm_0.state == 7 || fsm_0.state == 8 || fsm_0.state == 9))
    {
      walk_right_prepare_fourth();
    }

    //-------------------------------UPDATES ACTIONS WALKING LEFT----------------------------------------------------------------

    if(fsm_3.state == 0 && fsm_0.state == 11)
    {
      walk_left_initial_position();
    }
    else if(fsm_3.state == 1 && fsm_0.state == 11)
    {
      walk_left_second_position();
    }
    else if(fsm_3.state == 2 && fsm_0.state == 11)
    {
      walk_left_third_position();
    }
    else if(fsm_3.state == 3 && fsm_0.state == 11)
    {
      walk_left_fourth_position();
    }
    else if(fsm_3.state == 4 && fsm_0.state == 11)
    {
      walk_left_fifth_position();
    }
    else if(fsm_3.state == 5 && fsm_0.state == 11)
    {
      walk_left_sixth_position();
    }
    else if(fsm_3.state == 6 && fsm_0.state == 11)
    {
      walk_left_seventh_position();
    }
    else if(fsm_3.state == 7 && fsm_0.state == 11)
    {
      walk_left_eighth_position();
    }
    else if(fsm_3.state == 8 && fsm_0.state == 11)
    {
      walk_left_ninth_position();
    }
    else if(fsm_3.state == 9 && fsm_0.state == 11)
    {
      walk_left_tenth_position();
    }
    else if(fsm_3.state == 11 && fsm_0.state == 11)
    {
      walk_left_prepare_first();
    }
    else if(fsm_3.state == 12 && fsm_0.state == 11)
    {
      walk_left_prepare_second();
    }
    else if(fsm_3.state == 13 && fsm_0.state == 11)
    {
      walk_left_prepare_third();
    }
    else if(fsm_3.state == 14 && fsm_0.state == 11)
    {
      walk_left_prepare_fourth();
    }

    //-------------------------------UPDATES ACTIONS ROTATE RIGHT------------------------------------------------------------------
  
    if(fsm_4.state == 0 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_first_leg_1();
    }
    else if(fsm_4.state == 1 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_first_leg_2();
    }
    else if(fsm_4.state == 2 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_second_leg_1();
    }
    else if(fsm_4.state == 3 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_second_leg_2();
    }
    else if(fsm_4.state == 4 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_third_leg_1();
    }
    else if(fsm_4.state == 5 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_third_leg_2();
    }
    else if(fsm_4.state == 6 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_forth_leg_1();
    }
    else if(fsm_4.state == 7 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_forth_leg_2();
    }
    else if(fsm_4.state == 8 && (fsm_0.state == 1 || fsm_0.state == 10))
    {
      right_rotate_position();
    }

    //-------------------------------UPDATES ACTIONS ROTATE LEFT-------------------------------------------------------------------

    if(fsm_5.state == 0 && fsm_0.state == 6)
    {
      left_first_leg_1();
    }
    else if(fsm_5.state == 1 && fsm_0.state == 6)
    {
      left_first_leg_2();
    }
    else if(fsm_5.state == 2 && fsm_0.state == 6)
    {
      left_second_leg_1();
    }
    else if(fsm_5.state == 3 && fsm_0.state == 6)
    {
      left_second_leg_2();
    }
    else if(fsm_5.state == 4 && fsm_0.state == 6)
    {
      left_third_leg_1();
    }
    else if(fsm_5.state == 5 && fsm_0.state == 6)
    {
      left_third_leg_2();
    }
    else if(fsm_5.state == 6 && fsm_0.state == 6)
    {
      left_forth_leg_1();
    }
    else if(fsm_5.state == 7 && fsm_0.state == 6)
    {
      left_forth_leg_2();
    }
    else if(fsm_5.state == 8 && fsm_0.state == 6)
    {
      left_rotate_position();
    }

    //-------------------------------SET THE OUTPUTS-----------------------------------------------------------------------------------

    pwm.setPWM(servo_1.num, 0, servo_1.pwm_now);
    pwm.setPWM(servo_2.num, 0, servo_2.pwm_now);
    pwm.setPWM(servo_3.num, 0, servo_3.pwm_now);
    pwm.setPWM(servo_4.num, 0, servo_4.pwm_now);
    pwm.setPWM(servo_5.num, 0, servo_5.pwm_now);
    pwm.setPWM(servo_6.num, 0, servo_6.pwm_now);
    pwm.setPWM(servo_7.num, 0, servo_7.pwm_now);
    pwm.setPWM(servo_8.num, 0, servo_8.pwm_now);

    Serial.println(distance);

  /*Serial.println("Servo1:" );
  Serial.println(servo_1.pwm_now);
  Serial.println("Servo2:" );
  Serial.println(servo_2.pwm_now);
  Serial.println("Servo3:");
  Serial.println(servo_3.pwm_now);
  Serial.println("Servo4:");
  Serial.println(servo_4.pwm_now);*/  
}