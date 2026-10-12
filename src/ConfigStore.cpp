#include "ConfigStore.h"

#include <Preferences.h>
#include <math.h>

// Пространство имён + ключ NVS. Пространство имён и ключ должны быть <= 15 символов
// (ограничение NVS). Вся тройка конфигурации хранится под ОДНИМ ключом как упакованный
// blob, поэтому сохранение - это один nvs_commit (одна запись во flash) вместо трёх;
// это важно, потому что save() выполняется синхронно в служебной задаче AsyncTCP внутри
// обработчика POST, и единый commit сокращает время блокировки этой задачи примерно
// втрое.
namespace {
constexpr const char* kNamespace = "loadcfg";
constexpr const char* kKeyBlob = "cfg";
constexpr const char* kKeyHome = "home";

// Раскладка blob конфигурации во flash. Обычный POD, копируется memcpy в/из байтов NVS.
// Обратная совместимость определяется по РАЗМЕРУ (см. load()): СТАРАЯ раскладка - первые
// 3 float (target/min/max); НОВАЯ раскладка добавляет `offset` (ноль HX711, сырые
// отсчёты). Чтение blob СТАРОГО размера => offset отсутствует => вызывающий сохраняет
// своё значение по умолчанию; blob НОВОГО размера => offset авторитетен. НИКОГДА не
// переставляйте и не вставляйте поля перед `offset` - приём с проверкой размера зависит
// от того, что старые поля остаются первыми.
struct StoredConfigV1 {
  float targetKg;
  float minKg;
  float maxKg;
};
struct StoredConfig {
  float targetKg;
  float minKg;
  float maxKg;
  float offset;
};
}  // namespace

const char* validateConfig(float targetKg, float minKg, float maxKg) {
  if (isnan(targetKg) || isnan(minKg) || isnan(maxKg)) return "nan";
  if (isinf(targetKg) || isinf(minKg) || isinf(maxKg)) return "inf";
  if (minKg < 0.0f) return "min<0";
  if (!(minKg < maxKg)) return "min>=max";
  if (!(targetKg >= minKg && targetKg <= maxKg)) return "target out of range";
  return nullptr;
}

namespace ConfigStore {

bool load(ConfigTriple& out, const ConfigTriple& defaults) {
  Preferences prefs;
  // Открытие только для чтения; если пространство имён ещё не существует, begin() всё
  // равно возвращается, и чтение blob ниже просто не находит ключа.
  prefs.begin(kNamespace, /*readOnly=*/true);
  // Чтение в НОВУЮ (бо́льшую) структуру. getBytes копирует не более sizeof(blob) байт и
  // возвращает ФАКТИЧЕСКИЙ сохранённый размер, так что старый blob из 3 float отличается
  // от нового из 4 float исключительно по возвращённой длине.
  StoredConfig blob{};
  const size_t got = prefs.getBytes(kKeyBlob, &blob, sizeof(blob));
  prefs.end();

  // Распознаётся либо устаревшая тройка (3 float, без offset), либо текущий blob (4
  // float, offset присутствует). Всё остальное (отсутствующий ключ -> 0, либо усечённый
  // / больший blob из будущего) считается «не сохранённым»: используются значения по
  // умолчанию.
  const bool hasTriple = (got == sizeof(StoredConfigV1)) || (got == sizeof(StoredConfig));
  const bool hasOffset = (got == sizeof(StoredConfig));  // только blob НОВОЙ раскладки
  if (!hasTriple) {
    out = defaults;
    return false;
  }

  out.targetKg = blob.targetKg;
  out.minKg = blob.minKg;
  out.maxKg = blob.maxKg;

  // Совместимость по offset: сохранённый offset несёт только blob НОВОЙ раскладки. СТАРЫЙ
  // blob оставляет offset отсутствующим -> сохраняется значение по умолчанию вызывающего,
  // чтобы состояние перетарирования пережило обновление прошивки поверх устаревшего blob
  // NVS. offset НОВОГО blob авторитетен только если проходит проверку диапазона; дикое
  // значение (например, из повреждённой ячейки flash) откатывается к offset по умолчанию
  // БЕЗ отклонения тройки кг, поскольку offset - сущность, независимая от окна нагрузки.
  if (hasOffset) {
    if (blob.offset >= Config::HX711_OFFSET_MIN && blob.offset <= Config::HX711_OFFSET_MAX) {
      out.offset = blob.offset;
    } else {
      Serial.printf("[CFG][WARN] persisted offset %.0f out of range, using default %.0f\n",
                    static_cast<double>(blob.offset), static_cast<double>(defaults.offset));
      out.offset = defaults.offset;
    }
  } else {
    out.offset = defaults.offset;
  }

  // Отклонение повреждённой сохранённой тройки: откат к значениям по умолчанию, чтобы
  // плохой blob NVS никогда не заклинил SEEK в невозможном окне. (validateConfig НЕ
  // учитывает offset; некорректный offset уже обработан выше без отбрасывания тройки.)
  if (validateConfig(out.targetKg, out.minKg, out.maxKg) != nullptr) {
    Serial.println("[CFG][WARN] persisted config invalid, using defaults");
    out = defaults;
    return false;
  }
  return true;
}

void save(const ConfigTriple& cfg) {
  // ВНИМАНИЕ: выполняется синхронно в служебной задаче AsyncTCP (внутри обработчика POST
  // /v1/config). putBytes делает ОДИН nvs_commit (одна запись во flash, ~десятки мс) и
  // блокирует эту задачу на всё её время - намеренно допустимо для этой малонагруженной
  // локальной панели с единственным оператором. Не вызывать на горячем пути.
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    Serial.println("[CFG][WARN] NVS open failed, not persisting");
    return;
  }
  StoredConfig blob{cfg.targetKg, cfg.minKg, cfg.maxKg, cfg.offset};
  // ВНИМАНИЕ: putBytes (nvs_set_blob) НЕ дедуплицируется - в отличие от скалярных putFloat/
  // putInt, запись blob всегда фиксируется во flash, даже если байты идентичны. Поэтому
  // каждый здешний вызов расходует один цикл NVS. Это допустимо, так как save()
  // достигается только из операторского POST /v1/config или POST /v1/tare (редко); НЕ
  // вызывать на горячем/периодическом пути в расчёте на то, что идентичные записи
  // бесплатны. Вызывающие, которые намерены изменить только ОДНУ сущность (тройку кг ЛИБО
  // offset), ОБЯЗАНЫ сперва заполнить другое поле `cfg` ТЕКУЩИМ актуальным значением,
  // иначе эта запись единого blob затрёт его.
  prefs.putBytes(kKeyBlob, &blob, sizeof(blob));
  prefs.end();
  Serial.printf("[CFG] persisted target=%.3f min=%.3f max=%.3f offset=%.0f\n",
                static_cast<double>(cfg.targetKg), static_cast<double>(cfg.minKg),
                static_cast<double>(cfg.maxKg), static_cast<double>(cfg.offset));
}

float loadHomeAngle(float fallback) {
  Preferences prefs;
  prefs.begin(kNamespace, /*readOnly=*/true);
  const float v = prefs.getFloat(kKeyHome, NAN);
  prefs.end();
  if (isnan(v) || v < 0.0f || v >= 360.0f) {
    return fallback;
  }
  return v;
}

void saveHomeAngle(float homeAngleDeg) {
  Preferences prefs;
  if (!prefs.begin(kNamespace, /*readOnly=*/false)) {
    Serial.println("[CFG][WARN] NVS open failed, home angle not persisted");
    return;
  }
  prefs.putFloat(kKeyHome, homeAngleDeg);
  prefs.end();
  Serial.printf("[CFG] persisted home angle %.2f deg\n", static_cast<double>(homeAngleDeg));
}

}  // namespace ConfigStore
