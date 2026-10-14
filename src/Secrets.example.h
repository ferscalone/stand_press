#pragma once

// Учётные данные WiFi стенда. Шаблон: скопируйте в Secrets.h и впишите свою сеть.
namespace Secrets {
constexpr const char* WIFI_SSID = "CHANGE_ME";
constexpr const char* WIFI_PASSWORD = "CHANGE_ME";

// Учётные записи для входа в режим калибровки нуля (логин, пароль).
struct CalibUser {
  const char* login;
  const char* password;
};
constexpr CalibUser CALIB_USERS[] = {
    {"CHANGE_ME", "CHANGE_ME"},
};
}  // namespace Secrets
