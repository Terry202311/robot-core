#ifndef ORP_MOTOR_H
#define ORP_MOTOR_H

#include <Arduino.h>

enum class MotorDirection
{
    STOP = 0,
    FORWARD,
    BACKWARD
};

class MotorController
{
public:
    MotorController();

    void begin();

    void update();

    void setSpeedTarget(int speed);

    int getSpeedTarget() const;

    void stop();

    void forward();

    void backward();

    void moveLeft();

    void moveRight();

    void spinLeft();

    void spinRight();

    void driftLeft();

    void driftRight();

    void setWheelTargets(
        int motor_a,
        int motor_b,
        int motor_c,
        int motor_d
    );

    int getMotorAOutput() const;
    int getMotorBOutput() const;
    int getMotorCOutput() const;
    int getMotorDOutput() const;

private:
    int speed_target_;

    int pwm_a_now_;
    int pwm_b_now_;
    int pwm_c_now_;
    int pwm_d_now_;

    int pwm_a_target_;
    int pwm_b_target_;
    int pwm_c_target_;
    int pwm_d_target_;

    void setMotorDirections(
        MotorDirection motor_a,
        MotorDirection motor_b,
        MotorDirection motor_c,
        MotorDirection motor_d
    );

    void setMotorDirection(
        uint8_t input_1_pin,
        uint8_t input_2_pin,
        MotorDirection direction
    );

    int rampValue(int current, int target) const;

    int applyDeadzoneAndLimit(int value) const;
};

#endif
