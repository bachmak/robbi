#pragma once

#include "utils/non_copyable.h"
#include "utils/control.h"
#include "utils/math.h"

#include <Servo.h>

#include <limits>

struct MotorSettings
{
    utils::control::PIDSettings pid_settings = {
        .G = 1,
        .T_i = 1,
        .T_d = 0,
        .out_min = std::numeric_limits<float>::min(),
        .out_max = std::numeric_limits<float>::max(),
    };

    float speed_filter_alpha = 0.1f;

    Pin control_pin{0};
    Pin feedback_pin{0};

    Us feedback_pwm_min{1000};
    Us feedback_pwm_max{1200};

    float feedback_pwm_duty_cycle_min = 0.029f;
    float feedback_pwm_duty_cycle_max = 0.971f;

    Us min_pwm{1300};
    Us stop_pwm{1500};
    Us max_pwm{1700};

    Us fwd_pwm_deadband{30};
    Us bwd_pwm_deadband{30};

    float fwd_pwm_gain = 1.0f;
    float bwd_pwm_gain = 1.0f;
};

class Motor
{
public:
    explicit Motor(const MotorSettings &settings);
    ~Motor();

    NON_COPYABLE(Motor)

public:
    void set_target_speed(DegSec speed);
    DegSec get_real_speed() const;
    void update(Us dt);

private:
    MotorSettings settings_;
    utils::control::PID pid_;
    utils::math::Ema speed_filter_;
    DegSec target_speed_{0};
    Degree last_angle_;
    Servo servo_;
};
