#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include "Config.h"

class EncoderTracker {
 public:
  bool begin(TwoWire& wire);
  bool update();
  void captureLogicalZero();
  // Действующий домашний угол (runtime, из NVS / калибровки нуля). Нужен только для
  // отображения относительного угла ДО первого захвата логического нуля.
  void setHomeAngle(float homeAngleDeg);

  float getAbsoluteDegrees() const;
  float getRelativeDegrees() const;
  float getRelativeCounts() const;
  int32_t getMultiTurnCounts() const;

  static float normalizeDegrees(float degrees);
  static float shortestDeltaDegrees(float from, float to);

 private:
  AS5600 encoder_{&Wire};
  bool initialized_ = false;
  bool logicalZeroCaptured_ = false;
  float homeAngleDeg_ = Config::HOME_ABSOLUTE_ANGLE_DEG;
  int16_t lastRaw_ = -1;
  int32_t multiTurnCounts_ = 0;
  int32_t zeroCounts_ = 0;
};
