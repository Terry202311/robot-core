#include "config.h"
#include "motor.h"

MotorController motors;

unsigned long last_action_time = 0;
uint8_t test_step = 0;

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    motors.begin();
    motors.setSpeedTarget(DEFAULT_SPEED_TARGET);

    Serial.println("ORP Mega2560 motor module ready");

    last_action_time = millis();
}

void loop()
{
    motors.update();

    const unsigned long now = millis();

    if (now - last_action_time < 3000)
    {
        return;
    }

    last_action_time = now;

    switch (test_step)
    {
        case 0:
            Serial.println("TEST: forward");
            motors.forward();
            break;

        case 1:
            Serial.println("TEST: stop");
            motors.stop();
            break;

        case 2:
            Serial.println("TEST: backward");
            motors.backward();
            break;

        case 3:
            Serial.println("TEST: stop");
            motors.stop();
            break;

        default:
            motors.stop();
            test_step = 0;
            return;
    }

    test_step++;
}
