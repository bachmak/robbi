#pragma once

#include "types.h"
#include "utils/non_copyable.h"

namespace utils::debug {

void crash_report();

class Blinker {
public:
  explicit Blinker(Pin pin);

  NON_COPYABLE(Blinker)

public:
  void update();

private:
  Pin pin_;
  bool state_{false};
};

} // namespace utils::debug