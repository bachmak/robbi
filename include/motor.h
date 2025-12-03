#pragma once

#include "utils/non_copyable.h"
#include "utils/control.h"
#include "utils/math.h"

#include <Servo.h>

#include <limits>
#include <string_view>
#include <string>

struct MotorSettings
{
    std::string name = "motor";

    utils::control::PIDSettings pid_settings = {
        .G = 1,
        .T_i = 0,
        .T_d = 0,
        .out_min = std::numeric_limits<float>::lowest(),
        .out_max = std::numeric_limits<float>::max(),
        .deadband = 0.001,
    };

    float speed_filter_alpha = 0.1f;

    Pin control_pin{0};
    Pin feedback_pin{0};

    Us feedback_pwm_min{1000};
    Us feedback_pwm_max{1200};

    float feedback_pwm_duty_cycle_min = 0.029f;
    float feedback_pwm_duty_cycle_max = 0.971f;

    Us pwm_min{1400};
    Us pwm_stop{1500};
    Us pwm_max{1600};

    Us pwm_deadband_fwd{30};
    Us pwm_deadband_bwd{30};

    float pwm_gain_fwd = 1.0f;
    float pwm_gain_bwd = 1.0f;
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

    void configure(std::string_view setting, float value);

    void set_stop(bool value) { stop_ = value; }

private:
    MotorSettings settings_;
    utils::control::PID pid_;
    utils::math::Ema speed_filter_;
    DegSec target_speed_{0};
    Degree last_angle_;
    Servo servo_;
    bool stop_ = false;
};
