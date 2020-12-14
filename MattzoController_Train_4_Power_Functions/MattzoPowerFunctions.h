/*
* Lego Power Functions Infrared Control for Arduino
* original source from https://github.com/jurriaan/Arduino-PowerFunctions
*
* see http://www.philohome.com/pf/LEGO_Power_Functions_RC_v120.pdf for more info
* Based on SuperCow's code (http://forum.arduino.cc/index.php?topic=38142.0)
*
* Released under MIT License
* 
* Library and some elements renamed by MattzoBricks to avoid conflicts with 
* the integrated PowerFunctions library within the Legoino library
*/

#ifndef MattzoPowerFunctions_h
#define MattzoPowerFunctions_h

#include <stdio.h>
#include "Arduino.h"

#define PF_COMBO_DIRECT_MODE 0x01
#define PF_SINGLE_PIN_CONTINUOUS 0x2
#define PF_SINGLE_PIN_TIMEOUT 0x3
#define PF_SINGLE_OUTPUT 0x4
#define PF_SINGLE_EXT 0x6
#define PF_ESCAPE 0x4

#define PF_IR_CYCLES(num) (uint16_t)((1.0 / 38000.0) * 1000 * 1000 * num)

// By spec CHECKSUM is a XOR based on the 4-bit triplet you are sending
#define PF_CHECKSUM() (0xf ^ _nib1 ^ _nib2 ^ _nib3)

#define PF_START_STOP PF_IR_CYCLES(39)
#define PF_HIGH_PAUSE PF_IR_CYCLES(21)
#define PF_LOW_PAUSE PF_IR_CYCLES(10)
#define PF_HALF_PERIOD PF_IR_CYCLES(0.5)
#define PF_MAX_MESSAGE_LENGTH PF_IR_CYCLES(522) // 2 * 45 + 16 * 27

//PWM speed steps
enum struct MattzoPowerFunctionsPwm
{
  FLOAT = 0x0,
  FORWARD1 = 0x1,
  FORWARD2 = 0x2,
  FORWARD3 = 0x3,
  FORWARD4 = 0x4,
  FORWARD5 = 0x5,
  FORWARD6 = 0x6,
  FORWARD7 = 0x7,
  BRAKE = 0x8,
  REVERSE7 = 0x9,
  REVERSE6 = 0xA,
  REVERSE5 = 0xB,
  REVERSE4 = 0xC,
  REVERSE3 = 0xD,
  REVERSE2 = 0xE,
  REVERSE1 = 0xF
};

enum struct MattzoPowerFunctionsPort
{
  RED = 0x0,
  BLUE = 0x1
};

class MattzoPowerFunctions
{
public:
  MattzoPowerFunctions(uint8_t pin, uint8_t channel);
  MattzoPowerFunctions(uint8_t pin);
  void single_pwm(MattzoPowerFunctionsPort port, MattzoPowerFunctionsPwm pwm);
  void single_pwm(MattzoPowerFunctionsPort port, MattzoPowerFunctionsPwm pwm, uint8_t channel);
  void single_increment(MattzoPowerFunctionsPort port);
  void single_increment(MattzoPowerFunctionsPort port, uint8_t channel);
  void single_decrement(MattzoPowerFunctionsPort port);
  void single_decrement(MattzoPowerFunctionsPort port, uint8_t channel);
  void combo_pwm(MattzoPowerFunctionsPwm redPwm, MattzoPowerFunctionsPwm bluePwm);
  void combo_pwm(MattzoPowerFunctionsPwm redPwm, MattzoPowerFunctionsPwm bluePwm, uint8_t channel);
  MattzoPowerFunctionsPwm speedToPwm(byte speed);

private:
  void pause(uint8_t count, uint8_t channel);
  void send_bit();
  void send(uint8_t channel);
  void start_stop_bit();

  void toggle();

  uint8_t _channel;
  uint8_t _pin;
  uint8_t _nib1, _nib2, _nib3;
  uint8_t _toggle;
};

#endif
// END OF FILE
