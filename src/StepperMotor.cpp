#include "StepperMotor.h"

#include "Config.h"

StepperMotor::StepperMotor(uint8_t stepPlusPin, uint8_t stepMinusPin, uint8_t dirPlusPin, uint8_t dirMinusPin)
    : stepPlusPin_(stepPlusPin),
      stepMinusPin_(stepMinusPin),
      dirPlusPin_(dirPlusPin),
      dirMinusPin_(dirMinusPin) {}

void StepperMotor::begin() {
  pinMode(stepPlusPin_, OUTPUT);
  pinMode(stepMinusPin_, OUTPUT);
  pinMode(dirPlusPin_, OUTPUT);
  pinMode(dirMinusPin_, OUTPUT);

  // ENABLE: оба вывода в 1 - ток через оптопару EN не идёт, драйвер включён.
  pinMode(Config::PIN_EN_P, OUTPUT);
  pinMode(Config::PIN_EN_N, OUTPUT);
  digitalWrite(Config::PIN_EN_P, HIGH);
  digitalWrite(Config::PIN_EN_N, HIGH);

  if (Config::DRIVER_PLUS_PINS_HIGH) {
    // «Общий плюс»: выводы «+» постоянно в 1, сигнал - на выводах «-» (активный 0).
    digitalWrite(stepPlusPin_, HIGH);
    digitalWrite(dirPlusPin_, HIGH);
  } else {
    digitalWrite(stepMinusPin_, LOW);
    digitalWrite(dirMinusPin_, LOW);
  }
  writeStep(false);
  writeDir(true);
}

// Оптопара STEP: active=true - ток идёт (импульс), false - покой.
void StepperMotor::writeStep(bool active) {
  if (Config::DRIVER_PLUS_PINS_HIGH) {
    digitalWrite(stepMinusPin_, active ? LOW : HIGH);
  } else {
    digitalWrite(stepPlusPin_, active ? HIGH : LOW);
  }
}

// Оптопара DIR: forward=true - ток идёт (как прежнее DIR+ = 1 при DIR- = 0).
void StepperMotor::writeDir(bool forward) {
  if (Config::DRIVER_PLUS_PINS_HIGH) {
    digitalWrite(dirMinusPin_, forward ? LOW : HIGH);
  } else {
    digitalWrite(dirPlusPin_, forward ? HIGH : LOW);
  }
}

void StepperMotor::stop() {
  writeStep(false);
}

void StepperMotor::setDirection(bool forward) {
  directionForward_ = forward;
  writeDir(forward);
}

void StepperMotor::moveSteps(int32_t steps, uint32_t stepIntervalUs) {
  if (steps == 0) {
    return;
  }

  const bool forward = steps > 0;
  setDirection(forward);

  const int32_t count = abs(steps);
  for (int32_t i = 0; i < count; ++i) {
    pulseStep(stepIntervalUs);
    expectedSteps_ += forward ? 1 : -1;
  }
}

int32_t StepperMotor::getExpectedSteps() const {
  return expectedSteps_;
}

float StepperMotor::getExpectedDegrees() const {
  return expectedSteps_ * Config::DEGREES_PER_STEP;
}

void StepperMotor::resetExpectedPosition() {
  expectedSteps_ = 0;
}

void StepperMotor::pulseStep(uint32_t stepIntervalUs) {
  writeStep(true);
  delayMicroseconds(stepIntervalUs / 2);
  writeStep(false);
  delayMicroseconds(stepIntervalUs / 2);
}

