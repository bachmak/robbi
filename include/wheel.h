#pragma once

#include "common.h"

#include <Servo.h>

#include <chrono>
#include <string>

struct WheelSettings
{
    Pin control_pin{0};
    Pin feedback_pin{0};

    Pwm min_pwm{40};
    Pwm stop_pwm{90};
    Pwm max_pwm{150};

    Speed speed_dead_range{5};

    Meter circumference{0.22};

    Us feedback_pwm_min{1000};
    Us feedback_pwm_max{1200};

    float feedback_pwm_duty_cycle_min = 0.029;
    float feedback_pwm_duty_cycle_max = 0.971;

    std::string label = "default-wheel";
};

class WheelAttachment
{
public:
    WheelAttachment(Servo &servo, Pin pin) : servo(servo) { servo.attach(pin.v); }
    ~WheelAttachment() { servo.detach(); }

    WheelAttachment(const WheelAttachment &) = delete;
    WheelAttachment &operator=(const WheelAttachment &) = delete;

private:
    friend class Wheel;
    Servo &servo;
};

class Wheel
{
public:
    explicit Wheel(const WheelSettings &settings);

public:
    Meter circumference() const { return settings_.circumference; }
    Speed current_speed() const { return current_speed_; }

    WheelAttachment attach() { return WheelAttachment(servo_, settings_.control_pin); }

    void rotate(WheelAttachment &wa, Speed speed);
    Degree read_angle() const;

private:
    Servo servo_;
    WheelSettings settings_;
    Speed current_speed_;
};
