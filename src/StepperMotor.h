#pragma once

#include <Arduino.h>

class StepperMotor {
 public:
  StepperMotor(uint8_t stepPlusPin, uint8_t stepMinusPin, uint8_t dirPlusPin, uint8_t dirMinusPin);

  void begin();
  void stop();
  void setDirection(bool forward);
  void moveSteps(int32_t steps, uint32_t stepIntervalUs);
  int32_t getExpectedSteps() const;
  float getExpectedDegrees() const;
  void resetExpectedPosition();

 private:
  void pulseStep(uint32_t stepIntervalUs);
  void writeStep(bool active);
  void writeDir(bool forward);

  uint8_t stepPlusPin_;
  uint8_t stepMinusPin_;
  uint8_t dirPlusPin_;
  uint8_t dirMinusPin_;
  bool directionForward_ = true;
  int32_t expectedSteps_ = 0;
};

