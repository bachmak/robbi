#include "common.h"

#include <Arduino.h>

namespace time_utils
{
    class StopWatch
    {
    public:
        Ms time_passed(Ms curr = Ms{millis()}) const
        {
            return curr - start;
        }

    private:
        Ms start = Ms{millis()};
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
}