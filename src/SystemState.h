#pragma once

enum class SystemState : uint8_t {
  BOOT,
  HOMING,
  IDLE,
  SEEK_LOAD,
  HOLD_LOAD,
  RETURN_HOME,
  CALIBRATE,  // калибровка нуля кулачков: медленное вращение до команды stop
  ERROR
};

inline const char* toString(SystemState state) {
  switch (state) {
    case SystemState::BOOT: return "BOOT";
    case SystemState::HOMING: return "HOMING";
    case SystemState::IDLE: return "IDLE";
    case SystemState::SEEK_LOAD: return "SEEK_LOAD";
    case SystemState::HOLD_LOAD: return "HOLD_LOAD";
    case SystemState::RETURN_HOME: return "RETURN_HOME";
    case SystemState::CALIBRATE: return "CALIBRATE";
    case SystemState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

