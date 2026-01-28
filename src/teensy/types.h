#pragma once

#include "utils/strong_type.h"

#include <chrono>
#include <cmath>

using Ns = std::chrono::nanoseconds;
using Us = std::chrono::microseconds;
using Ms = std::chrono::milliseconds;
using Sec = std::chrono::seconds;

using Degree = utils::strong_type::StrongType<float, struct DegreeTag>;
using Meter = utils::strong_type::StrongType<float, struct MeterTag>;
using Pin = utils::strong_type::StrongType<int, struct PinTag>;
using Pwm = utils::strong_type::StrongType<int, struct PwmTag>;
using DegSec = utils::strong_type::StrongType<float, struct DegSecTag>;
using MetSec = utils::strong_type::StrongType<float, struct MetSecTag>;