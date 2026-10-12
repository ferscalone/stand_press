#pragma once

#include <Arduino.h>
#include <HX711.h>

#include "Config.h"

class LoadCellManager {
 public:
  bool begin();
  void update();
  // Задаёт смещение нуля (сырые отсчёты), используемое в формуле кг. Проталкивается в
  // каждом цикле задачей loadTask из оперативного runtime-хранилища gOffset (которое
  // обновляет операторское перетарирование) ПЕРЕД update(), так что перетарирование
  // вступает в силу уже на следующем же отсчёте. Оставлено обычным членом, чтобы
  // LoadCellManager оставался свободен от глобальных переменных (без обращения к extern
  // getOffset()); offset принадлежит .ino, которая и подаёт его сюда.
  void setOffset(long offset);
  float getMassKg() const;
  bool isReady() const;
  long getRawValue() const;       // мгновенный последний отсчёт (для диагностики)
  long getFilteredRaw() const;    // медианный сырой для управления, без сглаживания EMA

 private:
  long computeMedian() const;

  HX711 hx711_;
  bool ready_ = false;         // begin() отработала; разрешает чтения в update()
  bool sampleReady_ = false;   // получен первый корректный отсчёт в допустимом диапазоне
  // Оперативное смещение нуля (сырые отсчёты), применяемое в формуле кг. Инициализируется
  // значением по умолчанию из Config.h при первом запуске; перезаписывается каждый цикл
  // задачей loadTask через setOffset() из авторитетного хранилища gOffset (инициализируется
  // из NVS, обновляется перетарированием).
  long offset_ = static_cast<long>(Config::HX711_OFFSET);
  float filteredKg_ = 0.0f;
  long rawValue_ = 0;

  long medianBuf_[Config::HX711_MEDIAN_WINDOW] = {0};
  uint8_t medianCount_ = 0;
  uint8_t medianIndex_ = 0;
  long filteredRaw_ = 0;
  bool filteredRawInit_ = false;
};
