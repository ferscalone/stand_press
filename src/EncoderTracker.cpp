#include "EncoderTracker.h"

#include "Config.h"

bool EncoderTracker::begin(TwoWire& wire) {
  wire.begin(Config::PIN_AS5600_SDA, Config::PIN_AS5600_SCL);
  // Программный режим направления (без аргумента). Раньше здесь стоял begin(4):
  // библиотека воспринимала 4 как аппаратный DIR-пин AS5600 и делала на GPIO4
  // pinMode(OUTPUT)+digitalWrite(LOW), а GPIO4 - это PUL+ драйвера. При схеме «общий
  // плюс» (PUL+ постоянно в 1) это намертво гасило бы импульсы шага.
  encoder_.begin();

  const uint16_t raw = encoder_.rawAngle();
  if (raw > 4095) {
    return false;
  }

  lastRaw_ = static_cast<int16_t>(raw);
  multiTurnCounts_ = raw;
  zeroCounts_ = raw;
  initialized_ = true;
  return true;
}

bool EncoderTracker::update() {
  if (!initialized_) {
    return false;
  }

  const uint16_t raw = encoder_.rawAngle();
  if (raw > 4095) {
    return false;
  }

  int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(lastRaw_);
  if (delta > 2048) {
    delta -= 4096;
  } else if (delta < -2048) {
    delta += 4096;
  }

  multiTurnCounts_ += delta;
  lastRaw_ = static_cast<int16_t>(raw);
  return true;
}

void EncoderTracker::setHomeAngle(float homeAngleDeg) {
  homeAngleDeg_ = homeAngleDeg;
}

void EncoderTracker::captureLogicalZero() {
  zeroCounts_ = multiTurnCounts_;
  logicalZeroCaptured_ = true;
}

float EncoderTracker::getAbsoluteDegrees() const {
  const int32_t raw = ((lastRaw_ % 4096) + 4096) % 4096;
  return raw * (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
}

float EncoderTracker::getRelativeDegrees() const {
  // До калибровки логического нуля (то есть до первого homing после перепрошивки)
  // zeroCounts_ всё ещё равен позиции при запуске, поэтому multiTurnCounts_ - zeroCounts_
  // равно 0 где бы ни находился вал - что читается как «в домашней позиции», даже когда
  // это не так. До тех пор вместо этого сообщается истинное отклонение от ИЗВЕСТНОГО угла
  // домашней позиции, чтобы отображаемое отклонение отражало реальность (например, вал,
  // припаркованный в мёртвой зоне за домашней позицией, показывает свою реальную угловую
  // ошибку, а не 0). Это значение ТОЛЬКО для отображения; детектирование проскальзывания
  // использует getRelativeCounts() и не затрагивается.
  //
  // Знак ОБЯЗАН совпадать с калиброванным путём (multiTurnCounts_ - zeroCounts_ ≈
  // absDeg - HOME): shortestDeltaDegrees(HOME, absDeg) = absDeg - HOME, так что дуга
  // циферблата поворачивается одинаково до и после homing. Перестановка аргументов меняет
  // знак и зеркалит дугу.
  if (!logicalZeroCaptured_) {
    return shortestDeltaDegrees(homeAngleDeg_, getAbsoluteDegrees());
  }
  return getRelativeCounts() * (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
}

float EncoderTracker::getRelativeCounts() const {
  return static_cast<float>(multiTurnCounts_ - zeroCounts_);
}

int32_t EncoderTracker::getMultiTurnCounts() const {
  return multiTurnCounts_;
}

float EncoderTracker::normalizeDegrees(float degrees) {
  while (degrees >= 360.0f) {
    degrees -= 360.0f;
  }
  while (degrees < 0.0f) {
    degrees += 360.0f;
  }
  return degrees;
}

float EncoderTracker::shortestDeltaDegrees(float from, float to) {
  float delta = normalizeDegrees(to) - normalizeDegrees(from);
  if (delta > 180.0f) {
    delta -= 360.0f;
  } else if (delta < -180.0f) {
    delta += 360.0f;
  }
  return delta;
}
