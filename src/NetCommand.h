#pragma once

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

namespace NetCommand {

// CALIB_* - команды режима калибровки нуля кулачков. Они НЕ разбираются из
// текстовых кадров WS / параметра cmd: их ставит в очередь только эндпойнт
// POST /v1/calib после проверки токена входа (см. NetworkManager.cpp). Аргумент
// команды (arg) передаётся вместе с ней в onCommand():
//   CALIB_ENTER  - войти в режим (из IDLE / ERROR), вал стоит;
//   CALIB_RUN    - непрерывное вращение, arg = +1 / -1;
//   CALIB_HALT   - остановить вращение / поворот (режим сохраняется);
//   CALIB_JOG    - повернуть на arg шагов (со знаком направления);
//   CALIB_SAVE   - принять текущий угол за новый ноль и выйти;
//   CALIB_CANCEL - выйти, ноль не менять, вернуться к прежнему нулю.
enum class Command {
    NONE, SEEK, HOME, STOP, TARE,
    CALIB_ENTER, CALIB_RUN, CALIB_HALT, CALIB_JOG, CALIB_SAVE, CALIB_CANCEL
};

inline bool ieq(const char* a, const char* b) {
    while (*a && *b) {
        if (tolower(static_cast<unsigned char>(*a)) !=
            tolower(static_cast<unsigned char>(*b))) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

inline Command parseCommand(const char* s) {
    if (!s) return Command::NONE;
    if (ieq(s, "seek")) return Command::SEEK;
    if (ieq(s, "home")) return Command::HOME;
    if (ieq(s, "stop")) return Command::STOP;
    if (ieq(s, "tare")) return Command::TARE;
    return Command::NONE;
}

inline const char* commandName(Command c) {
    switch (c) {
        case Command::SEEK: return "SEEK";
        case Command::HOME: return "HOME";
        case Command::STOP: return "STOP";
        case Command::TARE: return "TARE";
        case Command::CALIB_ENTER: return "CALIB_ENTER";
        case Command::CALIB_RUN: return "CALIB_RUN";
        case Command::CALIB_HALT: return "CALIB_HALT";
        case Command::CALIB_JOG: return "CALIB_JOG";
        case Command::CALIB_SAVE: return "CALIB_SAVE";
        case Command::CALIB_CANCEL: return "CALIB_CANCEL";
        default:            return "NONE";
    }
}

}  // namespace NetCommand
