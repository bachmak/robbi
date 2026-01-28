#pragma once

namespace utils::math
{
    class Ema
    {
    public:
        explicit Ema(float alpha) : alpha_(alpha) {}

    public:
        float update(float new_value)
        {
            last_value_ += alpha_ * (new_value - last_value_);
            return last_value_;
        }

        float last_value() const { return last_value_; }

        void set_alpha(float alpha) { alpha_ = alpha; }

    private:
        float alpha_;
        float last_value_;
    };
}