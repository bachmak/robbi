#include <Arduino.h>
#include <Servo.h>

#include <array>

const int servo_pin = 12;
const int trig_pin = 8;
const int echo_pin = 9;
const int measurement_delay_ms = 600;

Servo servo;

void rotate_servo(int position_deg)
{
    position_deg = std::min(position_deg, 180);
    position_deg = std::max(position_deg, 0);
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
    struct Angle
    {
        int adjusted;
        int reported;
    };

    std::array<Angle, 4> positions_deg{
        {
            {0, 180},
            {80, 90},
            {170, 0},
            {80, 90},
        }};

    for (const auto position_deg : positions_deg)
    {
        rotate_servo(position_deg.adjusted);

        delay(measurement_delay_ms);

        auto distance_cm = get_distance_cm();

        Serial.print(position_deg.reported);
        Serial.print(' ');
        Serial.println(distance_cm);
    }
}