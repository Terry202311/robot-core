#include "motor.h"

#include "config.h"

MotorController::MotorController()
    : speed_target_(DEFAULT_SPEED_TARGET),
      pwm_a_now_(0),
      pwm_b_now_(0),
      pwm_c_now_(0),
      pwm_d_now_(0),
      pwm_a_target_(0),
      pwm_b_target_(0),
      pwm_c_target_(0),
      pwm_d_target_(0)
{
}

void MotorController::begin()
{
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(CIN1, OUTPUT);
    pinMode(CIN2, OUTPUT);

    pinMode(DIN1, OUTPUT);
    pinMode(DIN2, OUTPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(PWMC, OUTPUT);
    pinMode(PWMD, OUTPUT);

    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);

    setMotorDirections(
        MotorDirection::STOP,
        MotorDirection::STOP,
        MotorDirection::STOP,
        MotorDirection::STOP
    );

    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    analogWrite(PWMC, 0);
    analogWrite(PWMD, 0);

    stop();
}

void MotorController::update()
{
    pwm_a_now_ = rampValue(pwm_a_now_, pwm_a_target_);
    pwm_b_now_ = rampValue(pwm_b_now_, pwm_b_target_);
    pwm_c_now_ = rampValue(pwm_c_now_, pwm_c_target_);
    pwm_d_now_ = rampValue(pwm_d_now_, pwm_d_target_);

    pwm_a_now_ = applyDeadzoneAndLimit(pwm_a_now_);
    pwm_b_now_ = applyDeadzoneAndLimit(pwm_b_now_);
    pwm_c_now_ = applyDeadzoneAndLimit(pwm_c_now_);
    pwm_d_now_ = applyDeadzoneAndLimit(pwm_d_now_);

    analogWrite(PWMA, pwm_a_now_);
    analogWrite(PWMB, pwm_b_now_);
    analogWrite(PWMC, pwm_c_now_);
    analogWrite(PWMD, pwm_d_now_);
}

void MotorController::setSpeedTarget(int speed)
{
    speed_target_ = constrain(speed, PWM_MIN, PWM_MAX);
}

int MotorController::getSpeedTarget() const
{
    return speed_target_;
}

void MotorController::stop()
{
    pwm_a_target_ = 0;
    pwm_b_target_ = 0;
    pwm_c_target_ = 0;
    pwm_d_target_ = 0;
}

void MotorController::forward()
{
    setMotorDirections(
        MotorDirection::FORWARD,
        MotorDirection::FORWARD,
        MotorDirection::FORWARD,
        MotorDirection::FORWARD
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        speed_target_,
        speed_target_
    );
}

void MotorController::backward()
{
    setMotorDirections(
        MotorDirection::BACKWARD,
        MotorDirection::BACKWARD,
        MotorDirection::BACKWARD,
        MotorDirection::BACKWARD
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        speed_target_,
        speed_target_
    );
}

void MotorController::moveLeft()
{
    setMotorDirections(
        MotorDirection::BACKWARD,
        MotorDirection::FORWARD,
        MotorDirection::BACKWARD,
        MotorDirection::FORWARD
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        speed_target_,
        speed_target_
    );
}

void MotorController::moveRight()
{
    setMotorDirections(
        MotorDirection::FORWARD,
        MotorDirection::BACKWARD,
        MotorDirection::FORWARD,
        MotorDirection::BACKWARD
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        speed_target_,
        speed_target_
    );
}

void MotorController::spinLeft()
{
    setMotorDirections(
        MotorDirection::BACKWARD,
        MotorDirection::FORWARD,
        MotorDirection::BACKWARD,
        MotorDirection::FORWARD
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        speed_target_,
        speed_target_
    );
}

void MotorController::spinRight()
{
    setMotorDirections(
        MotorDirection::FORWARD,
        MotorDirection::BACKWARD,
        MotorDirection::FORWARD,
        MotorDirection::BACKWARD
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        speed_target_,
        speed_target_
    );
}

void MotorController::driftLeft()
{
    setMotorDirections(
        MotorDirection::STOP,
        MotorDirection::STOP,
        MotorDirection::FORWARD,
        MotorDirection::FORWARD
    );

    setWheelTargets(
        0,
        0,
        speed_target_,
        speed_target_
    );
}

void MotorController::driftRight()
{
    setMotorDirections(
        MotorDirection::FORWARD,
        MotorDirection::FORWARD,
        MotorDirection::STOP,
        MotorDirection::STOP
    );

    setWheelTargets(
        speed_target_,
        speed_target_,
        0,
        0
    );
}

void MotorController::setWheelTargets(
    int motor_a,
    int motor_b,
    int motor_c,
    int motor_d
)
{
    pwm_a_target_ = constrain(abs(motor_a), PWM_MIN, PWM_MAX);
    pwm_b_target_ = constrain(abs(motor_b), PWM_MIN, PWM_MAX);
    pwm_c_target_ = constrain(abs(motor_c), PWM_MIN, PWM_MAX);
    pwm_d_target_ = constrain(abs(motor_d), PWM_MIN, PWM_MAX);
}

int MotorController::getMotorAOutput() const
{
    return pwm_a_now_;
}

int MotorController::getMotorBOutput() const
{
    return pwm_b_now_;
}

int MotorController::getMotorCOutput() const
{
    return pwm_c_now_;
}

int MotorController::getMotorDOutput() const
{
    return pwm_d_now_;
}

void MotorController::setMotorDirections(
    MotorDirection motor_a,
    MotorDirection motor_b,
    MotorDirection motor_c,
    MotorDirection motor_d
)
{
    setMotorDirection(AIN1, AIN2, motor_a);
    setMotorDirection(BIN1, BIN2, motor_b);
    setMotorDirection(CIN1, CIN2, motor_c);
    setMotorDirection(DIN1, DIN2, motor_d);
}

void MotorController::setMotorDirection(
    uint8_t input_1_pin,
    uint8_t input_2_pin,
    MotorDirection direction
)
{
    switch (direction)
    {
        case MotorDirection::FORWARD:
            digitalWrite(input_1_pin, LOW);
            digitalWrite(input_2_pin, HIGH);
            break;

        case MotorDirection::BACKWARD:
            digitalWrite(input_1_pin, HIGH);
            digitalWrite(input_2_pin, LOW);
            break;

        case MotorDirection::STOP:
        default:
            digitalWrite(input_1_pin, LOW);
            digitalWrite(input_2_pin, LOW);
            break;
    }
}

int MotorController::rampValue(int current, int target) const
{
    if (target > current + MAX_ACC_STEP)
    {
        return current + MAX_ACC_STEP;
    }

    if (target < current - MAX_DEC_STEP)
    {
        return current - MAX_DEC_STEP;
    }

    return target;
}

int MotorController::applyDeadzoneAndLimit(int value) const
{
    if (abs(value) < SPEED_DEADZONE)
    {
        return 0;
    }

    return constrain(value, PWM_MIN, PWM_MAX);
}
