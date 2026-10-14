#include "Telemetry.h"

#include "Config.h"

// ArduinoJson нет в lib_deps, поэтому JSON собирается через snprintf в буфер на стеке.
// Формат фиксированный и компактный; NET_WS_BUFFER_SIZE (512) - буфер отправки WebSocket,
// поэтому запас относительно него делает одиночную отправку дешёвой.

// Поля телеметрии читаются через getTelemetrySnapshot() (объявлена в Telemetry.h,
// определена в .ino), которая копирует их все под одной критической секцией. Это
// заменяет прежний набор отдельных extern-геттеров, каждый из которых блокировал/
// разблокировал независимо и мог давать рваное чтение.

String buildTelemetryJson() {
  // Берётся ОДИН атомарный снимок всех разделяемых полей телеметрии, чтобы JSON был
  // действительно внутренне согласован: каждое поле ниже берётся из одного и того же
  // момента, без вклинивания смены состояния между чтениями (прежний подход с отдельными
  // геттерами этого не гарантировал, хотя комментарий это утверждал).
  TelemetrySnapshot snap;
  getTelemetrySnapshot(snap);

  const uint32_t uptimeMs = millis();
  const float targetKg = snap.targetKg;

  char buf[512];
  const int written = snprintf(buf, sizeof(buf),
           "{\"state\":\"%s\",\"load_kg\":%.3f,\"target_kg\":%.3f,"
           "\"range_min_kg\":%.3f,\"range_max_kg\":%.3f,"
           "\"load_raw\":%ld,\"load_ready\":%d,\"uptime_ms\":%lu,"
           "\"encoder_abs_deg\":%.2f,\"encoder_rel_deg\":%.2f,\"encoder_fault\":%d,"
           "\"load_offset\":%ld,\"tare_ok\":%d,\"home_abs_deg\":%.2f,\"motor_en\":%d,\"calib_busy\":%d}",
           toString(snap.state),
           static_cast<double>(snap.loadKg),
           static_cast<double>(targetKg),
           static_cast<double>(snap.rangeMinKg),
           static_cast<double>(snap.rangeMaxKg),
           snap.loadRaw,
           snap.loadReady ? 1 : 0,
           static_cast<unsigned long>(uptimeMs),
           static_cast<double>(snap.encAbsDeg),
           static_cast<double>(snap.encRelDeg),
           snap.encoderFault ? 1 : 0,
           snap.loadOffset,
           snap.tareOk ? 1 : 0,
           static_cast<double>(snap.homeAbsDeg),
           snap.motorEnabled ? 1 : 0,
           snap.calibBusy ? 1 : 0);
  if (written < 0 || written >= static_cast<int>(sizeof(buf))) {
    Serial.println("[TELEMETRY][WARN] JSON truncated, buf too small");
  }
  return String(buf);
}

String buildEventJson(const char* event, const char* detail) {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"event\":\"%s\",\"detail\":\"%s\",\"uptime_ms\":%lu}",
           event, detail, static_cast<unsigned long>(millis()));
  return String(buf);
}
