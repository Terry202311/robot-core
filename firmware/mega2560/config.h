#ifndef ORP_CONFIG_H
#define ORP_CONFIG_H

// ============================================================
// OpenRobotPlatform - Arduino Mega2560 Hardware Configuration
// ============================================================

// -------------------- PS2 Controller --------------------------

#define PS2_DAT 50
#define PS2_CMD 51
#define PS2_CLK 52
#define PS2_SEL 53

#define PS2_PRESSURES false
#define PS2_RUMBLE false

// -------------------- TB6612 Motor Driver ---------------------

// Motor A
#define PWMA 3
#define AIN1 5
#define AIN2 4

// Motor B
#define PWMB 10
#define BIN1 8
#define BIN2 9

// Motor C
#define PWMC 11
#define CIN1 12
#define CIN2 13

// Motor D
#define PWMD 2
#define DIN1 6
#define DIN2 7

#define STBY 14

// -------------------- Encoder Pins ----------------------------

#define LF_A 22
#define LF_B 23

#define RF_A 24
#define RF_B 25

#define LB_A 26
#define LB_B 27

#define RB_A 28
#define RB_B 29

// -------------------- OLED ------------------------------------

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C
#define OLED_RESET_PIN -1

// -------------------- Motor Parameters ------------------------

#define DEFAULT_SPEED_TARGET 160

#define MAX_ACC_STEP 5
#define MAX_DEC_STEP 10
#define SPEED_DEADZONE 4

#define PWM_MIN 0
#define PWM_MAX 255

// -------------------- Joystick Parameters ---------------------

#define JOY_MID 128
#define JOY_MIN 93
#define JOY_MAX 163

// -------------------- Serial ----------------------------------

#define SERIAL_BAUDRATE 115200

#endif
