#pragma once

#include <Arduino.h>
#include <ctype.h>
#include <string.h>

namespace NetCommand {

// CALIB_PLUS / CALIB_MINUS - запуск калибровки нуля кулачков (медленное вращение в
// сторону роста / уменьшения угла энкодера); завершается командой STOP.
enum class Command { NONE, SEEK, HOME, STOP, TARE, CALIB_PLUS, CALIB_MINUS };

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
    if (ieq(s, "calib_plus")) return Command::CALIB_PLUS;
    if (ieq(s, "calib_minus")) return Command::CALIB_MINUS;
    return Command::NONE;
}

inline const char* commandName(Command c) {
    switch (c) {
        case Command::SEEK: return "SEEK";
        case Command::HOME: return "HOME";
        case Command::STOP: return "STOP";
        case Command::TARE: return "TARE";
        case Command::CALIB_PLUS: return "CALIB_PLUS";
        case Command::CALIB_MINUS: return "CALIB_MINUS";
        default:            return "NONE";
    }
}

}  // namespace NetCommand
