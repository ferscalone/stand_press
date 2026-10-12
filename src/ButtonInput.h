#pragma once

#include <Arduino.h>

class ButtonInput {
 public:
  ButtonInput(uint8_t pin, uint32_t debounceMs);

  void begin();
  bool update();

 private:
  uint8_t pin_;
  uint32_t debounceMs_;
  bool stableState_ = HIGH;
  bool lastRead_ = HIGH;
  uint32_t lastEdgeMs_ = 0;
};
