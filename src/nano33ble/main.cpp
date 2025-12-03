#include <Arduino.h>
#include <Servo.h>

#include <array>

const int servo_pin = 12;
const int echo_pin = 8;
const int trig_pin = 9;

Servo servo;

void rotate_servo(int position_deg)
{
    if (position_deg > 180)
    {
        position_deg = 180;
    }
    if (position_deg < 0)
    {
        position_deg = 0;
    }
    servo.write(position_deg);
}

int get_distance_cm()
{
    // turn off
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);

    // generate signal
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    auto duration = pulseIn(echo_pin, HIGH);
    auto distance_cm = static_cast<int>(duration * 0.034 / 2.0);

    return distance_cm;
}

void setup()
{
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);

    servo.attach(servo_pin);
    servo.write(0);

    Serial.begin(9600);
    delay(500);
}

void loop()
{
    std::array<int, 4> positions_deg{0, 90, 180, 90};

    for (const auto position_deg : positions_deg)
    {
        rotate_servo(position_deg);
        auto distance_cm = get_distance_cm();

        Serial.print(position_deg);
        Serial.print(' ');
        Serial.println(distance_cm);

        delay(600);
    }
}