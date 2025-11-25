#include "robot.h"

#include "wheel_utils.h"
#include "time_utils.h"
#include "common_utils.h"
#include "io_utils.h"

#include <Arduino.h>

Robot::Robot(const RobotSettings &settings)
    : wheels_(settings.left_wheel_settings, settings.right_wheel_settings), settings_(settings) {}

void Robot::move(const Meter target_pos, const Meter init_pos)
{
    const auto direction = Speed{init_pos < target_pos ? 1 : -1};

    auto &[left, right] = wheels_;
    auto left_wa = left.attach();
    auto right_wa = right.attach();

    auto angle_left = right.read_angle();
    auto angle_right = left.read_angle();

    wheel_utils::rotate(left, left_wa, direction);
    wheel_utils::rotate(right, right_wa, -direction);

    auto cur_pos = init_pos;

    while (abs(cur_pos - target_pos) > Meter{0.01})
    {
        // TODO: complete implementation
        delay(2000);

        angle_left = wheel_utils::get_full_angle(left, angle_left);
        angle_right = wheel_utils::get_full_angle(right, angle_right);

        auto cur_pos_left = wheel_utils::to_distance(left, angle_left);
        auto cur_pos_right = wheel_utils::to_distance(right, angle_right);

        io_utils::debug("cur_pos: left=%f, right=%f", cur_pos_left, cur_pos_right);

        cur_pos = cur_pos_left;
    }

    wheel_utils::stop(left, left_wa);
    wheel_utils::stop(right, right_wa);
}

void Robot::rotate(Degree target_pos, Degree init_pos)
{
    // TODO: complete implementation
    (void)target_pos;
    (void)init_pos;
}

void Robot::rotate(Meter left, Meter right, Speed speed)
{
    auto left_deg = wheel_utils::to_angle(wheels_.left, left);
    auto right_deg = wheel_utils::to_angle(wheels_.right, right);
    rotate(left_deg, right_deg, speed);
}

void Robot::rotate(Degree distance_left, Degree distance_right, Speed speed)
{
    auto progress_left = 0.0f;
    auto progress_right = 0.0f;

    auto &w_left = wheels_.left;
    auto &w_right = wheels_.right;

    auto wa_left = w_left.attach();
    auto wa_right = w_right.attach();

    const auto init_left = w_left.read_angle();
    const auto init_right = w_right.read_angle();

    auto angle_left = init_left;
    auto angle_right = init_right;

    auto avg_real_speed_left = 0.0f;
    auto avg_real_speed_right = 0.0f;

    io_utils::debug("angle_left=%f, angle_right=%f", angle_left.v, angle_right.v);

    const auto target_left = angle_left + distance_left;
    const auto target_right = angle_right - distance_right;

    io_utils::debug("target_left=%f, target_right=%f", target_left.v, target_right.v);

    const auto dir_left = Speed{distance_left > Degree{0} ? 1 : -1};
    const auto dir_right = Speed{distance_right > Degree{0} ? 1 : -1};

    io_utils::debug("dir_left=%d, dir_right=%d", dir_left.v, dir_right.v);

    const auto init_speed_left = dir_left * speed;
    const auto init_speed_right = -(dir_right * speed);

    io_utils::debug("init_speed_left=%d, init_speed_right=%d", init_speed_left.v, init_speed_right.v);

    auto clock = time_utils::StopWatch{};
    auto loop_clock = time_utils::LoopStopWatch{};

    while ((progress_left < 1.0f || progress_right < 1.0f) && clock.time_passed() < settings_.max_move_time)
    {
        const auto diff = progress_right - progress_left;
        const auto delta_speed = Speed{settings_.p_controller_gain * diff};

        const auto delta_left = diff > 0 ? delta_speed : Speed{0};
        const auto delta_right = diff < 0 ? delta_speed : Speed{0};

        const auto speed_left = [&]
        {
            if (progress_left > 1.0f || abs(target_left - angle_left) < Degree{avg_real_speed_left * loop_clock.average().count()})
            {
                return Speed{0};
            }
            return init_speed_left + dir_left * delta_left;
        }();

        const auto speed_right = [&]
        {
            if (progress_right > 1.0f || abs(angle_right - target_right) < Degree{avg_real_speed_right * loop_clock.average().count()})
            {
                return Speed{0};
            }
            return init_speed_right + dir_right * delta_right;
        }();

        if (speed_left == Speed{0} && speed_left == Speed{0})
        {
            break;
        }

        w_left.rotate(wa_left, speed_left);
        w_right.rotate(wa_right, speed_right);

        const auto new_angle_left = wheel_utils::get_full_angle(w_left, angle_left);
        const auto new_angle_right = wheel_utils::get_full_angle(w_right, angle_right);

        const auto loop_time = loop_clock.tick();

        const auto real_speed_left = abs(new_angle_left - angle_left).v / loop_time.count();
        const auto real_speed_right = abs(new_angle_right - angle_right).v / loop_time.count();

        avg_real_speed_left = common_utils::ema(real_speed_left, avg_real_speed_left, settings_.speed_ema_filter_coef);
        avg_real_speed_right = common_utils::ema(real_speed_right, avg_real_speed_right, settings_.speed_ema_filter_coef);

        angle_left = new_angle_left;
        angle_right = new_angle_right;

        progress_left = std::abs(((angle_left - init_left) / distance_left).v);
        progress_right = std::abs(((angle_right - init_right) / distance_right).v);

        io_utils::debug(
            "Left: {progress=%.3f, pos=%.2f, pwm=%d, speed=%.2f, avg_speed=%.2f}. "
            "Right: {progress=%.3f, pos=%.2f, pwm=%d, speed=%.2f, avg_speed=%.2f}. "
            "Loop time = %d ms",
            progress_left, angle_left.v, speed_left.v, real_speed_left, avg_real_speed_left,
            progress_right, angle_right.v, speed_right.v, real_speed_right, avg_real_speed_right,
            static_cast<int>(loop_time.count()));
    }

    w_left.rotate(wa_left, Speed{0});
    w_right.rotate(wa_right, Speed{0});

    delay(settings_.wait_after_move.count());

    angle_left = wheel_utils::get_full_angle(w_left, angle_left);
    angle_right = wheel_utils::get_full_angle(w_right, angle_right);

    io_utils::info(
        "Rotation done. Total time = %d s, Average control loop time = %d ms\n"
        "Left wheel: {distance = %.2f deg / %.2f m, target = %f deg, actual = %f deg, error = %.2f deg / %.2f m, accuracy = %.2f%%}.\n"
        "Right wheel: {distance = %.2f deg / %.2f m, target = %f deg, actual = %f deg, error = %.2f deg / %.2f m, accuracy = %.2f%%}.",
        static_cast<int>(std::chrono::duration_cast<Sec>(clock.time_passed()).count()),
        static_cast<int>(loop_clock.average().count()),

        distance_left.v, wheel_utils::to_distance(w_left, distance_left).v, target_left.v, angle_left.v,
        (angle_left - target_left).v, wheel_utils::to_distance(w_left, angle_left - target_left).v,
        abs((angle_left - target_left) / distance_left).v * 100,

        distance_right.v, wheel_utils::to_distance(w_right, distance_right).v, target_right.v, angle_right.v,
        (angle_right - target_right).v, wheel_utils::to_distance(w_right, angle_right - target_right).v,
        abs((angle_right - target_right) / distance_right).v * 100);
}
