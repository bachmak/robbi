#include "utils/debug.h"

#include "utils/io.h"

#include <Arduino.h>
#include <CrashReport.h>

namespace utils::debug {

void crash_report() {
  if (CrashReport) {
    auto p = utils::io::StringPrint{};
    CrashReport.printTo(p);
    utils::io::debug("there is a crash report:\n%s", p.buffer.c_str());
  }
}

Blinker::Blinker(Pin pin) : pin_(pin) { pinMode(pin_.v, OUTPUT); }

void Blinker::update() { digitalWrite(pin_.v, std::exchange(state_, !state_)); }

} // namespace utils::debug