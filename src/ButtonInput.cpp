#include "ButtonInput.h"

ButtonInput::ButtonInput(uint8_t pin, uint32_t debounceMs)
    : pin_(pin), debounceMs_(debounceMs) {}

void ButtonInput::begin() {
  pinMode(pin_, INPUT_PULLUP);
  stableState_ = digitalRead(pin_);
  lastRead_ = stableState_;
  lastEdgeMs_ = millis();
}

bool ButtonInput::update() {
  const bool current = digitalRead(pin_);

  if (current != lastRead_) {
    lastRead_ = current;
    lastEdgeMs_ = millis();
  }

  if ((millis() - lastEdgeMs_) >= debounceMs_ && stableState_ != current) {
    stableState_ = current;
    if (stableState_ == LOW) {
      return true;
    }
  }

  return false;
}
