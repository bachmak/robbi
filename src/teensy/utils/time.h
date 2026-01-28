#include "types.h"

#include <Arduino.h>

namespace utils::time
{
    class StopWatch
    {
    public:
        explicit StopWatch(Ms start = Ms{millis()}) : start(start) {}

        Ms time_passed(Ms curr = Ms{millis()}) const
        {
            return curr - start;
        }

        void reset(Ms curr = Ms{millis()})
        {
            start = curr;
        }

    private:
        Ms start;
    };

    class Timer
    {
    public:
        explicit Timer(Ms duration, Ms start = Ms{millis()}) : duration(duration), stop_watch(start) {}

        bool is_over(Ms curr = Ms{millis()}) const
        {
            return stop_watch.time_passed(curr) > duration;
        }

        void reset(Ms curr = Ms{millis()})
        {
            stop_watch.reset(curr);
        }

    private:
        Ms duration;
        StopWatch stop_watch;
    };

    class LoopStopWatch
    {
    public:
        Ms tick(Ms curr = Ms{millis()})
        {
            auto loop = curr - last;
            common += loop;
            iterations++;
            last = curr;
            return loop;
        }

        Ms average() const
        {
            return Ms{static_cast<int>(static_cast<double>(common.count()) / iterations)};
        }

    private:
        Ms last = Ms{millis()};
        Ms common = Ms{0};
        int iterations = 0;
    };

    inline float to_sec(Us t)
    {
        using R = std::ratio_divide<Us::period, Sec::period>;
        constexpr auto num = R::num;
        constexpr auto den = R::den;

        return static_cast<float>(t.count()) * num / den;
    }
}