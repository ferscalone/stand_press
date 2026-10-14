#include <Arduino.h>
#include <Wire.h>

#include "ButtonInput.h"
#include "Config.h"
#include "ConfigStore.h"
#include "EncoderTracker.h"
#include "LoadCellManager.h"
#include "NetworkManager.h"
#include "StepperMotor.h"
#include "SystemState.h"
#include "Telemetry.h"

// Политика выбора направления для homing / возврата в домашнюю позицию. На этом
// стенде + шаги увеличивают угол энкодера и сильнее зажимают объект между плитами
// (в сторону стенки / больше нагрузки), - шаги сбрасывают нагрузку (в сторону
// мёртвой зоны). Homing
// не должен двигаться в сторону стенки. BY_LOAD - обычная политика; несмотря на
// унаследованное имя, направление теперь выбирается по УГЛУ + геометрии дуги
// (см. homeStepDir), а не по тензодатчику. Политики ALWAYS_* задают фиксированное
// направление и зарезервированы для диагностики / будущего использования.
enum class HomeDirPolicy : uint8_t {
  BY_LOAD,       // на основе угла+дуги: на дуге нагрузки -> шаг -; в мёртвой зоне -> +
  ALWAYS_MINUS,  // всегда - (зарезервировано: явный авто-возврат из-под нагрузки)
  ALWAYS_PLUS    // всегда + (диагностика/будущее)
};

// Причина остановки parkIntoWindow. Вызывающий код (motorTask) превращает это
// ровно в ОДНО событие телеметрии: IN_WINDOW -> "park", WALL/STEP_BUDGET -> "wall"
// (оба означают "не удалось достичь цели, припарковано на максимально достижимой
// нагрузке"). Объявлено здесь (вверху, рядом с HomeDirPolicy), а не рядом с
// MeasureMode, чтобы тип был уже определён к моменту, где сборка Arduino вставляет
// автоматически сгенерированные прототипы функций (сразу после includes).
enum class ParkReason : uint8_t { IN_WINDOW, WALL, STEP_BUDGET };

struct SharedData {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile SystemState state = SystemState::BOOT;
  volatile float loadKg = 0.0f;
  volatile bool loadReady = false;
  volatile bool encoderReady = false;
  volatile float encoderAbsoluteDeg = 0.0f;
  volatile float encoderRelativeDeg = 0.0f;
  volatile float expectedPositionDeg = 0.0f;
  volatile bool buttonPressed = false;
  volatile bool encoderFault = false;
  volatile float encoderErrorDeg = 0.0f;
  volatile uint32_t lastLoadUpdateMs = 0;
  volatile long loadRawInstant = 0;
  volatile long loadRawFiltered = 0;
};

SharedData gShared;

// Изменяемая во время работы конфигурация управления нагрузкой (цель + допустимый
// диапазон), задаётся через POST /v1/config и сохраняется в NVS. У неё СВОЙ
// spinlock (не gShared.mux), чтобы чтение/запись конфигурации никогда не
// конкурировали с горячей критической секцией энкодера/телеметрии. Инициализируется
// значениями по умолчанию из Config.h, затем перекрывается из NVS при загрузке.
struct RuntimeConfig {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile float targetKg = Config::TARGET_LOAD_CENTER_KG;
  volatile float minKg = Config::TARGET_LOAD_MIN_KG;
  volatile float maxKg = Config::TARGET_LOAD_MAX_KG;
};
RuntimeConfig gConfig;

// Изменяемое во время работы смещение нуля HX711 (сырые отсчёты), задаётся
// перетарированием оператора (POST /v1/tare или WS "tare") и сохраняется в NVS.
// У него СВОЙ spinlock - НЕ gShared.mux и НЕ gConfig.mux - так как оно читается на
// ОБОИХ ядрах в горячих путях (loadTask на ядре 0 каждый цикл, rawToKg() в
// motorTask на ядре 1, телеметрия в сетевой задаче) и не должно конкурировать с
// этими критическими секциями или вкладываться в них. Инициализируется значением
// по умолчанию для первой загрузки из Config.h, затем перекрывается из NVS при
// загрузке (см. setup()). Точно повторяет схему runtime-хранилища gConfig.
struct RuntimeOffset {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile float offset = Config::HX711_OFFSET;
};
RuntimeOffset gOffset;

// Атомарное чтение оперативного смещения. Безопасно из любого ядра/задачи через spinlock.
float getOffset() {
  portENTER_CRITICAL(&gOffset.mux);
  const float offset = gOffset.offset;
  portEXIT_CRITICAL(&gOffset.mux);
  return offset;
}

// Атомарная запись оперативного смещения. Вызывается из пути перетарирования
// (сетевая задача, ядро 0) параллельно с чтениями rawToKg() задачи мотора (ядро 1)
// через spinlock.
void setOffset(float offset) {
  portENTER_CRITICAL(&gOffset.mux);
  gOffset.offset = offset;
  portEXIT_CRITICAL(&gOffset.mux);
}

// Атомарный снимок оперативной конфигурации (одна критическая секция), чтобы
// вызывающий код видел target/min/max одного момента - без разорванного чтения,
// если POST /v1/config придёт в середине использования.
void getConfigSnapshot(ConfigTriple& out) {
  portENTER_CRITICAL(&gConfig.mux);
  out.targetKg = gConfig.targetKg;
  out.minKg = gConfig.minKg;
  out.maxKg = gConfig.maxKg;
  portEXIT_CRITICAL(&gConfig.mux);
}

// Атомарная запись всех трёх значений конфигурации. Вызывающий код ОБЯЗАН сначала
// проверить тройку значений (validateConfig). Безопасно вызывать из сетевой задачи
// (ядро 0) параллельно с чтениями задачи мотора (ядро 1) через spinlock.
void setConfig(float targetKg, float minKg, float maxKg) {
  portENTER_CRITICAL(&gConfig.mux);
  gConfig.targetKg = targetKg;
  gConfig.minKg = minKg;
  gConfig.maxKg = maxKg;
  portEXIT_CRITICAL(&gConfig.mux);
}

// Действующий домашний угол (логический ноль), абсолютные градусы AS5600. Засеивается
// из NVS в setup() и переопределяется калибровкой нуля кулачков (состояние
// CALIBRATE). Свой spinlock: пишет motorTask, читают motorTask и сетевая телеметрия.
struct RuntimeHomeAngle {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile float deg = Config::HOME_ABSOLUTE_ANGLE_DEG;
};
RuntimeHomeAngle gHomeAngle;

float getHomeAngleDeg() {
  portENTER_CRITICAL(&gHomeAngle.mux);
  const float deg = gHomeAngle.deg;
  portEXIT_CRITICAL(&gHomeAngle.mux);
  return deg;
}

void setHomeAngleDeg(float deg) {
  portENTER_CRITICAL(&gHomeAngle.mux);
  gHomeAngle.deg = deg;
  portEXIT_CRITICAL(&gHomeAngle.mux);
}

ButtonInput gButton(Config::PIN_BUTTON, Config::BUTTON_DEBOUNCE_MS);
EncoderTracker gEncoder;
LoadCellManager gLoadCell;
StepperMotor gMotor(Config::PIN_STEP_P, Config::PIN_STEP_N, Config::PIN_DIR_P, Config::PIN_DIR_N);

TaskHandle_t gLoadTaskHandle = nullptr;
TaskHandle_t gMotorTaskHandle = nullptr;
int32_t gHomeReferenceSteps = 0;
float gReturnHomeBestErrorDeg = 0.0f;
uint32_t gReturnHomeLastProgressMs = 0;
// Устанавливается обработчиком кнопки (loop, независимо от ядра), чтобы попросить
// motorTask восстановиться из ERROR через повторный homing. motorTask владеет
// энкодером/мотором, поэтому сам homing выполняется там, чтобы избежать гонки
// I2C/мотора между ядрами.
volatile bool gRehomeRequested = false;
// Политика, которую motorTask использует для запрошенного повторного homing /
// homing при загрузке (устанавливается до подъёма gBootHomeRequested /
// gRehomeRequested). Обычно BY_LOAD, которая основана на угле+дуге (см.
// homeStepDir): ей никогда не нужен тензодатчик для выбора направления.
volatile HomeDirPolicy gRehomePolicy = HomeDirPolicy::BY_LOAD;
// Устанавливается setup(), чтобы попросить motorTask выполнить ОДНОРАЗОВЫЙ homing
// при загрузке. Отделено от gRehomeRequested (путь восстановления из ERROR), чтобы
// два намерения оставались различимыми: этот запускается из начального состояния
// HOMING и при неудаче переходит в ERROR, как это делал прежний блокирующий homing
// при загрузке. Homing выполняется в motorTask (не в setup()), поэтому setup()
// возвращается немедленно, а диагностический loop() + сетевой стек поднимаются и
// остаются активными ВО ВРЕМЯ homing.
volatile bool gBootHomeRequested = false;

// ── Калибровка нуля кулачков (состояние CALIBRATE) ───────────────────────────
// Пишут onCommand (сетевые команды, задача loop) и кнопка; читает и исполняет
// motorTask (calibrateTick).
// Непрерывное вращение: 0 - стоим, +1 / -1 - крутим до команды «стоп».
volatile int8_t gCalibRunDir = 0;
// Остаток поворота на заданный угол, в шагах со знаком направления. Нажатия
// суммируются; motorTask снимает по одному шагу. Свой spinlock: пишут и читают
// разные задачи.
portMUX_TYPE gCalibMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t gCalibJogPending = 0;
// Запрос «сохранить ноль»: motorTask останавливает вал и принимает текущий угол за
// новый ноль.
volatile bool gCalibFinishRequested = false;
// Запрос отмены: остановить, ноль не менять, вернуться к прежнему.
volatile bool gCalibCancelRequested = false;
// Запрос «стоп»: прекратить вращение / поворот, остаться в режиме калибровки.
volatile bool gCalibHaltRequested = false;
// Точка старта движения (шаги мотора) и стартовая нагрузка - для защит по ходу и
// по нагрузке; время последней активности - для выхода по бездействию.
volatile int32_t gCalibStartSteps = 0;
volatile float gCalibStartLoadKg = 0.0f;
volatile uint32_t gCalibActivityMs = 0;

void addCalibJog(int32_t steps) {
  portENTER_CRITICAL(&gCalibMux);
  gCalibJogPending += steps;
  portEXIT_CRITICAL(&gCalibMux);
}

void clearCalibJog() {
  portENTER_CRITICAL(&gCalibMux);
  gCalibJogPending = 0;
  portEXIT_CRITICAL(&gCalibMux);
}

int32_t getCalibJogPending() {
  portENTER_CRITICAL(&gCalibMux);
  const int32_t v = gCalibJogPending;
  portEXIT_CRITICAL(&gCalibMux);
  return v;
}

// Снимает один шаг из остатка поворота. Возвращает его направление (+1 / -1) или 0,
// если поворачивать нечего.
int8_t takeCalibJogStep() {
  int8_t dir = 0;
  portENTER_CRITICAL(&gCalibMux);
  if (gCalibJogPending > 0) {
    dir = 1;
    --gCalibJogPending;
  } else if (gCalibJogPending < 0) {
    dir = -1;
    ++gCalibJogPending;
  }
  portEXIT_CRITICAL(&gCalibMux);
  return dir;
}
// Базовая линия текущего минимума нагрузки для реактивной защиты от стенки в
// RETURN_HOME. Сохраняется между тиками motorTask; сбрасывается в -1 (не захвачено)
// при переходе В RETURN_HOME и отслеживается вниз по мере сброса нагрузки.
// См. moveTowardHome().
float gReturnHomeBaselineKg = -1.0f;

uint32_t getLastLoadUpdateMs() {
  portENTER_CRITICAL(&gShared.mux);
  const uint32_t lastLoadUpdateMs = gShared.lastLoadUpdateMs;
  portEXIT_CRITICAL(&gShared.mux);
  return lastLoadUpdateMs;
}

void setState(SystemState newState) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.state = newState;
  portEXIT_CRITICAL(&gShared.mux);
  // Немедленно отправить переход WS-клиентам дашборда (центральная узловая точка:
  // каждое изменение состояния проходит через это место - из loop(), motorTask,
  // обработчика сетевых команд и setup()). queueEvent безопасен для задач и
  // неблокирующий: он копирует JSON во внутреннюю очередь NetworkManager и
  // немедленно возвращается (никаких textAll/mutex/heap на этом - возможно, с
  // ядра 1 - вызывающем коде); loop() опустошает её в сетевой задаче. Переходы до
  // begin() (из setup()/runHoming(), пока NetworkManager::begin() ещё не создал
  // очередь) безопасно отбрасываются по замыслу: queueEvent проверяет очередь на null.
  NetworkManager::queueEvent(buildEventJson("state", toString(newState)).c_str());
}

SystemState getState() {
  portENTER_CRITICAL(&gShared.mux);
  const SystemState state = gShared.state;
  portEXIT_CRITICAL(&gShared.mux);
  return state;
}

void updateEncoderShared(bool ready, float absoluteDeg, float relativeDeg, float expectedDeg, bool encoderFault, float encoderErrorDeg) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.encoderReady = ready;
  gShared.encoderAbsoluteDeg = absoluteDeg;
  gShared.encoderRelativeDeg = relativeDeg;
  gShared.expectedPositionDeg = expectedDeg;
  gShared.encoderFault = encoderFault;
  gShared.encoderErrorDeg = encoderErrorDeg;
  portEXIT_CRITICAL(&gShared.mux);
}

void updateLoadShared(float loadKg, bool ready, long rawInstant, long rawFiltered) {
  portENTER_CRITICAL(&gShared.mux);
  gShared.loadKg = loadKg;
  gShared.loadReady = ready;
  gShared.loadRawInstant = rawInstant;
  gShared.loadRawFiltered = rawFiltered;
  gShared.lastLoadUpdateMs = millis();
  portEXIT_CRITICAL(&gShared.mux);
}

float getLoadKg() {
  portENTER_CRITICAL(&gShared.mux);
  const float loadKg = gShared.loadKg;
  portEXIT_CRITICAL(&gShared.mux);
  return loadKg;
}

long getLoadRaw() {
  portENTER_CRITICAL(&gShared.mux);
  const long loadRaw = gShared.loadRawInstant;
  portEXIT_CRITICAL(&gShared.mux);
  return loadRaw;
}

long getLoadFilteredRaw() {
  portENTER_CRITICAL(&gShared.mux);
  const long loadRaw = gShared.loadRawFiltered;
  portEXIT_CRITICAL(&gShared.mux);
  return loadRaw;
}

// Аксессоры снимка только для чтения для построителя телеметрии (Telemetry.cpp).
// Сведены к минимуму: только поля, нужные дашборду, копируются под блокировкой.
bool getLoadReady() {
  portENTER_CRITICAL(&gShared.mux);
  const bool ready = gShared.loadReady;
  portEXIT_CRITICAL(&gShared.mux);
  return ready;
}

float getEncoderAbsoluteDeg() {
  portENTER_CRITICAL(&gShared.mux);
  const float deg = gShared.encoderAbsoluteDeg;
  portEXIT_CRITICAL(&gShared.mux);
  return deg;
}

float getEncoderRelativeDeg() {
  portENTER_CRITICAL(&gShared.mux);
  const float deg = gShared.encoderRelativeDeg;
  portEXIT_CRITICAL(&gShared.mux);
  return deg;
}

bool getEncoderFault() {
  portENTER_CRITICAL(&gShared.mux);
  const bool fault = gShared.encoderFault;
  portEXIT_CRITICAL(&gShared.mux);
  return fault;
}

// Атомарный снимок телеметрии: копирует каждое поле, о котором сообщает дашборд,
// из gShared в ОДНОЙ критической секции, чтобы buildTelemetryJson() не мог смешать
// значения из разных моментов (отдельные геттеры выше каждый берут/освобождают
// блокировку по отдельности, что допускало бы разорванное чтение). Повторяет блок
// [DIAG] в loop(), который уже делает снимок таким образом.
void getTelemetrySnapshot(TelemetrySnapshot& out) {
  portENTER_CRITICAL(&gShared.mux);
  out.state = gShared.state;
  out.loadKg = gShared.loadKg;
  out.loadRaw = gShared.loadRawInstant;
  out.loadReady = gShared.loadReady;
  out.encAbsDeg = gShared.encoderAbsoluteDeg;
  out.encRelDeg = gShared.encoderRelativeDeg;
  out.encoderFault = gShared.encoderFault;
  portEXIT_CRITICAL(&gShared.mux);

  // Готовность к перетарированию, вычисленная из ТОГО ЖЕ момента gShared,
  // захваченного выше (все четыре входа - поля gShared, поэтому без лишней
  // блокировки и без разорванного чтения относительно остального снимка). Это
  // ТОЧНО повторяет проверку-ограничитель tryRetare - IDLE, вал в пределах
  // RETARE_MAX_REL_DEG от домашней позиции, нагрузка готова, |нагрузка| в пределах
  // RETARE_MAX_ABS_KG - чтобы состояние доступности на дашборде совпадало с тем,
  // что /v1/tare действительно примет. Оставлено как производный флаг телеметрии,
  // НЕ второй источник истины: tryRetare сам перепроверяет ограничитель по свежим значениям.
  out.tareOk = (out.state == SystemState::IDLE) &&
               (fabsf(out.encRelDeg) <= Config::RETARE_MAX_REL_DEG) &&
               out.loadReady &&
               (fabsf(out.loadKg) <= Config::RETARE_MAX_ABS_KG);

  // Поля конфигурации берутся из gConfig под ЕГО СОБСТВЕННОЙ блокировкой - в
  // отдельной критической секции (никогда не вложенной в gShared.mux), чтобы
  // избежать любого риска нарушения порядка блокировок.
  portENTER_CRITICAL(&gConfig.mux);
  out.targetKg = gConfig.targetKg;
  out.rangeMinKg = gConfig.minKg;
  out.rangeMaxKg = gConfig.maxKg;
  portEXIT_CRITICAL(&gConfig.mux);

  // Актуальное смещение берётся из gOffset под ЕГО собственной блокировкой -
  // третья независимая критическая секция (никогда не вложенная в gShared.mux или gConfig.mux).
  portENTER_CRITICAL(&gOffset.mux);
  out.loadOffset = static_cast<long>(gOffset.offset);
  portEXIT_CRITICAL(&gOffset.mux);

  out.homeAbsDeg = getHomeAngleDeg();
  out.motorEnabled = gMotor.isEnabled();
  out.calibBusy = (gCalibRunDir != 0) || (getCalibJogPending() != 0);
}

bool waitForFreshLoadSample(uint32_t previousUpdateMs, uint32_t timeoutMs = Config::HX711_CONTROL_SAMPLE_FRESHNESS_MS) {
  const uint32_t startMs = millis();
  while ((millis() - startMs) < timeoutMs) {
    if (getLastLoadUpdateMs() != previousUpdateMs) {
      return true;
    }
    delay(2);
  }
  return false;
}

float rawToKg(long raw) {
  // Читается ОПЕРАТИВНОЕ смещение (gOffset), а не константа из Config.h:
  // перетарирование оператора обновляет gOffset (и NVS), и это преобразование в
  // управляющем пути должно использовать актуальный ноль. getOffset() берёт
  // spinlock gOffset, поэтому это безопасно из motorTask (ядро 1) параллельно с
  // записью перетарирования в сетевой задаче.
  return (static_cast<float>(raw) - getOffset()) / Config::HX711_CALIBRATION_FACTOR;
}

// Профиль измерения: сколько ждать, пока механизм/HX711 придёт в покой после
// шага, и сколько мгновенных сырых отсчётов усреднять.
enum class MeasureMode : uint8_t {
  COARSE,  // мёртвая зона: объект ещё не зажат, нагрузка почти не меняется -> минимальное ожидание
  FAST,    // объект зажат, но нагрузка всё ещё ниже окна -> короткое ожидание
  PRECISE  // близко/внутри окна, решение о парковке -> полное успокоение, больше отсчётов
};

// Выбирает направление homing на каждый шаг для `policy` по абсолютному углу
// энкодера `absoluteDeg`.
//
// Направление выбирается по УГЛУ и ГЕОМЕТРИИ ДУГИ стенда, никогда не по нагрузке.
// Две причины, почему нагрузка не может выбирать направление:
//   1. Тензодатчик НЕНАДЁЖЕН для определения, с какой стороны от домашней позиции
//      находится вал: нагрузка прикладывается через механизм (рычаг ~2.66:1),
//      поэтому вал, физически находящийся выше домашней позиции под реальной
//      нагрузкой, может показывать ~0 кг (по наблюдениям на стенде: abs=113.64 deg,
//      выше домашней позиции 106.08, но при этом load_kg=0.018).
//   2. Даже чисто кратчайший угол здесь неверен: геометрия АСИММЕТРИЧНА. Нагрузка
//      существует только на КОРОТКОЙ дуге от домашней позиции вверх (+) до стенки
//      (~150-160 deg abs). Остальная часть окружности - это ДЛИННАЯ мёртвая зона.
//      Из мёртвой зоны КРАТЧАЙШИЙ угол назад к домашней позиции может пройти ЧЕРЕЗ
//      стенку (например, на 234 deg кратчайший составляет -128 deg, прямо в стенку
//      на ~151). Наблюдалось, как вал делал именно это и уходил в ERROR об стенку.
//
// Поэтому: вычисляется смещение от домашней позиции в направлении +, 0..360.
//   * смещение в (0, LOAD_ARC_WIDTH_DEG]  -> на ДУГЕ НАГРУЗКИ -> шаг - вниз к домашней
//     позиции (сбрасывает нагрузку, короткий путь);
//   * иначе                              -> МЁРТВАЯ ЗОНА -> шаг + вверх к домашней
//     позиции ДЛИННЫМ путём через 0/360, чтобы никогда не пересекать стенку.
//
// Имя enum BY_LOAD сохранено для совместимости исходников, но теперь означает
// "на основе угла+дуги". ALWAYS_MINUS / ALWAYS_PLUS задают фиксированное направление (зарезервировано).
//
// ОБНОВЛЕНО: правило «мёртвая зона -> длинным путём в +» отменено по результатам
// работы на стенде. Теперь BY_LOAD всегда ведёт вал в одну сторону -
// Config::HOME_DIRECTION (по умолчанию «-»), при необходимости почти полный оборот.
// Единственное исключение - небольшой перелёт за ноль (до HOME_OVERSHOOT_BAND_DEG):
// тогда короткий ход обратно, а не лишний оборот.
int32_t homeStepDir(HomeDirPolicy policy, float absoluteDeg) {
  if (policy == HomeDirPolicy::ALWAYS_MINUS) return -Config::HOME_STEP_CHUNK;
  if (policy == HomeDirPolicy::ALWAYS_PLUS) return Config::HOME_STEP_CHUNK;
  const int32_t dir = Config::HOME_DIRECTION;
  const float delta = EncoderTracker::shortestDeltaDegrees(getHomeAngleDeg(), absoluteDeg);
  // Перелёт: вал уже прошёл ноль в направлении dir (знак delta совпадает с dir).
  if ((delta * dir) > 0.0f && fabsf(delta) <= Config::HOME_OVERSHOOT_BAND_DEG) {
    return -dir * Config::HOME_STEP_CHUNK;
  }
  return dir * Config::HOME_STEP_CHUNK;
}

// Вал на дуге нагрузки (0 < смещение от дома в «+» <= LOAD_ARC_WIDTH_DEG). Только
// здесь движение к дому гарантированно сбрасывает нагрузку, поэтому только здесь
// подъём нагрузки при homing считается заклиниванием.
bool onLoadArc(float absoluteDeg) {
  const float plusOffset = EncoderTracker::normalizeDegrees(absoluteDeg - getHomeAngleDeg());
  return (plusOffset > 0.0f) && (plusOffset <= Config::LOAD_ARC_WIDTH_DEG);
}

// Оставшийся путь до дома по тому маршруту, которым его ведёт homeStepDir (в
// градусах, >= 0). Нужен детекторам застревания: при длинном пути кратчайший угол
// до дома сначала РАСТЁТ, и мерить прогресс по нему нельзя.
float homeRemainingDeg(float absoluteDeg) {
  const int32_t dir = Config::HOME_DIRECTION;
  const float delta = EncoderTracker::shortestDeltaDegrees(getHomeAngleDeg(), absoluteDeg);
  if ((delta * dir) > 0.0f && fabsf(delta) <= Config::HOME_OVERSHOOT_BAND_DEG) {
    return fabsf(delta);
  }
  // Путь в направлении dir: при dir = -1 это смещение от дома в «+», при dir = +1 -
  // смещение в «-».
  return EncoderTracker::normalizeDegrees(-dir * (absoluteDeg - getHomeAngleDeg()));
}

// Выполняет одно измерение нагрузки в покое. Мотор ОБЯЗАТЕЛЬНО должен быть уже
// остановлен к моменту вызова. Ожидает успокоения, чтобы механизм и HX711 (всплески
// от драйвера) успокоились, затем усредняет несколько *мгновенных* сырых отсчётов.
// Намеренно игнорирует канал с медианным фильтром, который отстаёт на 100+ мс и
// сразу после шага сообщает устаревшие значения. Возвращает усреднённые кг в *outKg.
// Возвращаемое bool - это `quiet`: true, если отсчёты сошлись в пределах
// SETTLED_MAX_SPREAD_KG (механизм перестал двигаться), false, если они всё ещё
// дрейфовали (вызывающему коду следует подождать и попробовать снова).
//
// Число шагов мотора НЕ зависит от режима; меняются только ожидание и число
// отсчётов, поэтому подход ускоряется без изменения числа шагов.
bool measureSettledKg(float* outKg, MeasureMode mode = MeasureMode::PRECISE) {
  uint32_t settleMs;
  uint8_t sampleCount;
  switch (mode) {
    case MeasureMode::COARSE:
      settleMs = Config::SETTLE_AFTER_STEP_COARSE_MS;
      sampleCount = Config::SETTLED_SAMPLE_COUNT_COARSE;
      break;
    case MeasureMode::FAST:
      settleMs = Config::SETTLE_AFTER_STEP_FAST_MS;
      sampleCount = Config::SETTLED_SAMPLE_COUNT_FAST;
      break;
    case MeasureMode::PRECISE:
    default:
      settleMs = Config::SETTLE_AFTER_STEP_PRECISE_MS;
      sampleCount = Config::SETTLED_SAMPLE_COUNT;
      break;
  }
  delay(settleMs);

  const uint32_t startMs = millis();
  float sumKg = 0.0f;
  float minKg = 1.0e9f;
  float maxKg = -1.0e9f;
  uint8_t collected = 0;
  uint32_t prevUpdateMs = getLastLoadUpdateMs();

  while (collected < sampleCount) {
    waitForFreshLoadSample(prevUpdateMs, Config::HX711_CONTROL_SAMPLE_FRESHNESS_MS);
    prevUpdateMs = getLastLoadUpdateMs();

    const long raw = getLoadRaw();
    if (raw > 0) {  // raw==0 - недопустимый отсчёт HX711, встречается в логах; пропускается
      const float kg = rawToKg(raw);
      sumKg += kg;
      if (kg < minKg) minKg = kg;
      if (kg > maxKg) maxKg = kg;
      ++collected;
    }

    if ((millis() - startMs) > Config::SETTLED_MAX_WAIT_MS) {
      break;
    }
    delay(Config::SETTLED_SAMPLE_SPACING_MS);
  }

  if (collected == 0) {
    *outKg = getLoadKg();  // откат к тому, что есть; вызывающий код остаётся осторожным
    return false;
  }

  *outKg = sumKg / static_cast<float>(collected);
  const float spread = maxKg - minKg;
  return spread <= Config::SETTLED_MAX_SPREAD_KG;
}

// Перетарирование оператора: перезахват нуля HX711 (смещения) в логическом нуле.
// Выполняется в СЕТЕВОЙ задаче (из пути onCommand WS "tare" И, синхронно, из REST-
// обработчика POST /v1/tare) - НИКОГДА не двигает мотор, поэтому безопасно вне ядра
// мотора: motorTask продолжает работать, но в IDLE не двигается, а здесь только
// ЧИТАЕТСЯ нагрузка и ЗАПИСЫВАЕТСЯ gOffset (оба под защитой spinlock).
//
// Порядок проверок-ограничителей важен - *reason устанавливается в ПЕРВЫЙ
// неудачный токен, и происходит немедленный возврат, чтобы оператор узнал одну
// наиболее релевантную причину:
//   1. state == IDLE                              -> иначе "not_idle"
//   2. |encoder_rel_deg| <= RETARE_MAX_REL_DEG    -> иначе "not_home"
//   3. load_ready                                 -> иначе "not_ready"
//   4. |нагрузка в покое кг| <= RETARE_MAX_ABS_KG -> иначе "load_too_high"
// Измерение выполняется, только если ВСЕ проходят: усредняется RETARE_SAMPLE_COUNT
// СВЕЖИХ, попадающих в диапазон (raw>0) отсчётов getLoadRaw() в покое - повторно
// используется идиома waitForFreshLoadSample + пропуск при raw>0 из
// measureSettledKg, БЕЗ шага мотора. Среднее округляется до нового смещения и
// проверяется на разумность относительно HX711_OFFSET_MIN..MAX ("offset_insane"),
// чтобы дикий отсчёт HX711 никогда не мог испортить сохранённый ноль.
//
// При успехе: setOffset(new), запись в NVS вместе с ТЕКУЩЕЙ тройкой конфигурации
// (чтобы сохранение единым blob не затёрло target/min/max), запись *newOffset и
// возврат true. `reason` может быть null (пути WS он не нужен); `newOffset` тоже
// может быть null. Используются только аксессоры снимка (getState/
// getEncoderRelativeDeg/getLoadReady/getLoadKg/getLoadRaw) - без прямого доступа к gShared.
bool tryRetare(String* reason, long* newOffset) {
  auto fail = [&](const char* token) -> bool {
    if (reason != nullptr) *reason = token;
    return false;
  };

  // Проверка 1: только в IDLE. В любом состоянии движения/удержания показание не
  // является истинным нулём (или мотор в середине движения), поэтому
  // перетарирование захватило бы ложное смещение.
  if (getState() != SystemState::IDLE) return fail("not_idle");

  // Проверка 2: вал должен быть физически в домашней позиции. rel_deg - это угол с
  // момента последнего захваченного логического нуля; близость к 0 означает, что
  // стенд действительно в механической домашней позиции, где "нет нагрузки" осмысленно.
  if (fabsf(getEncoderRelativeDeg()) > Config::RETARE_MAX_REL_DEG) return fail("not_home");

  // Проверка 3: тензодатчик должен был выдать корректный отсчёт; без него getLoadKg
  // - это просто значение по умолчанию 0.0, и измерение ниже было бы бессмысленным.
  if (!getLoadReady()) return fail("not_ready");

  // Проверка 4: текущая сообщаемая нагрузка должна быть уже около нуля. Если она
  // показывает реальную нагрузку, стенд НЕ в логическом нуле (что-то зажато /
  // давит на датчик); перетарирование здесь обнулило бы настоящую нагрузку.
  // Используются актуальные отфильтрованные кг.
  if (fabsf(getLoadKg()) > Config::RETARE_MAX_ABS_KG) return fail("load_too_high");

  // Все проверки пройдены. Усредняется RETARE_SAMPLE_COUNT свежих, попадающих в
  // диапазон сырых отсчётов в покое - тот же цикл со свежим отсчётом + пропуском
  // при raw>0, что и в measureSettledKg, но БЕЗ шага мотора. Короткое ограниченное
  // ожидание на отсчёт не даёт зависнуть, если HX711 замолкает; если не удаётся
  // собрать ни одного корректного отсчёта - выход вместо доверия устаревшему значению.
  double sumRaw = 0.0;
  uint8_t collected = 0;
  uint32_t prevUpdateMs = getLastLoadUpdateMs();
  const uint32_t startMs = millis();
  while (collected < Config::RETARE_SAMPLE_COUNT) {
    waitForFreshLoadSample(prevUpdateMs, Config::HX711_CONTROL_SAMPLE_FRESHNESS_MS);
    prevUpdateMs = getLastLoadUpdateMs();

    // Используется сырое значение после МЕДИАННОГО фильтра, а не мгновенный
    // getLoadRaw(): захват нуля - единственное место, обходившее медиану шириной 5,
    // которая защищает управляющий путь, поэтому одиночный всплеск рассинхронизации
    // HX711 (например, ~1.9M против истинного ~114530) мог бы пройти raw>0, увести
    // среднее внутрь ограничителя разумности [OFFSET_MIN,OFFSET_MAX] и оказаться
    // СОХРАНЁННЫМ - смещая каждое последующее значение кг. getLoadReady() выше
    // гарантирует, что медиана инициализирована, поэтому откат к rawValue_ никогда не применяется.
    const long raw = getLoadFilteredRaw();
    if (raw > 0) {  // raw==0 - недопустимый отсчёт HX711, встречается в логах; пропускается
      sumRaw += static_cast<double>(raw);
      ++collected;
    }

    // Жёсткая граница: RETARE_SAMPLE_COUNT отсчётов по ~LOAD_TASK_PERIOD_MS каждый
    // укладываются заметно меньше секунды; общее ожидание ограничивается, чтобы
    // застрявший датчик не мог бесконечно блокировать сетевую задачу.
    if ((millis() - startMs) > Config::SETTLED_MAX_WAIT_MS) {
      break;
    }
    delay(Config::SETTLED_SAMPLE_SPACING_MS);
  }

  if (collected == 0) return fail("not_ready");

  const long candidate = lroundf(static_cast<float>(sumRaw / static_cast<double>(collected)));

  // Ограничение разумности захваченного нуля: дикий сырой отсчёт (например,
  // рассинхронизация HX711) никогда не должен быть записан как смещение, иначе
  // каждое последующее значение кг было бы мусором И оно было бы сохранено. Вне
  // диапазона -> отклонить, оставив старое смещение нетронутым.
  if (candidate < static_cast<long>(Config::HX711_OFFSET_MIN) ||
      candidate > static_cast<long>(Config::HX711_OFFSET_MAX)) {
    return fail("offset_insane");
  }

  // Фиксация: сначала обновляется актуальное смещение, затем сохранение. Сохраняется
  // ТЕКУЩАЯ тройка конфигурации с прикреплённым новым смещением, так как ConfigStore
  // использует единый blob - чтение актуальной тройки заранее не даёт сохранению
  // затереть target/min/max.
  setOffset(static_cast<float>(candidate));

  ConfigTriple cfg;
  getConfigSnapshot(cfg);       // заполняет target/min/max (поле offset остаётся по умолчанию)
  cfg.offset = static_cast<float>(candidate);
  ConfigStore::save(cfg);

  if (newOffset != nullptr) *newOffset = candidate;
  Serial.printf("[TARE] re-tared: offset=%ld (from %u samples)\n", candidate,
                static_cast<unsigned>(collected));
  return true;
}

bool getButtonEvent() {
  portENTER_CRITICAL(&gShared.mux);
  const bool pressed = gShared.buttonPressed;
  gShared.buttonPressed = false;
  portEXIT_CRITICAL(&gShared.mux);
  return pressed;
}

void signalButtonEvent() {
  portENTER_CRITICAL(&gShared.mux);
  gShared.buttonPressed = true;
  portEXIT_CRITICAL(&gShared.mux);
}

bool checkEncoderSlip(float* outErrorDeg = nullptr) {
  const bool encoderOk = gEncoder.update();
  const float expectedDeg = gMotor.getExpectedDegrees();

  if (!encoderOk) {
    updateEncoderShared(false, 0.0f, 0.0f, expectedDeg, true, 0.0f);
    return true;
  }

  const float expectedCounts = static_cast<float>(gMotor.getExpectedSteps()) * Config::ENCODER_COUNTS_PER_STEP;
  const float actualCounts = gEncoder.getRelativeCounts();
  const float errorCounts = actualCounts - expectedCounts;
  const float error = errorCounts * (Config::DEGREES_PER_REV / Config::ENCODER_COUNTS_PER_REV);
  const bool fault = fabsf(error) > Config::ENCODER_POSITION_ERROR_DEG;
  const float actualDeg = gEncoder.getRelativeDegrees();

  if (outErrorDeg != nullptr) {
    *outErrorDeg = error;
  }

  updateEncoderShared(true, gEncoder.getAbsoluteDegrees(), actualDeg, expectedDeg, fault, error);
  return fault;
}

bool runHoming(HomeDirPolicy policy) {
  setState(SystemState::HOMING);
  Serial.println("[STATE] HOMING");

  if (!gEncoder.update()) {
    Serial.println("[ERROR] Encoder read failed during homing");
    return false;
  }

  delay(Config::HOMING_SETTLE_MS);

  // Базовая линия текущего минимума нагрузки для реактивной защиты от стенки,
  // захватывается, как только выбрано направление - (сброс) (-1 = ещё не захвачено).
  // Сброс тянет нагрузку вниз, поэтому минимум отслеживает подлинный нижний уровень
  // сброса; реальный набор в стенку поднимается выше него на запас. Один блокирующий
  // цикл, обычная локальная переменная.
  float baselineLoadKg = -1.0f;
  uint32_t safetyCounter = 0;
  // Детектор застревания: лучший (наименьший) оставшийся путь до дома и момент
  // последнего приближения к нему.
  float bestRemainingDeg = 1e9f;
  uint32_t lastProgressMs = millis();
  while (true) {
    if (!gEncoder.update()) {
      Serial.println("[ERROR] Encoder read failed during homing move");
      return false;
    }

    const float absoluteDeg = gEncoder.getAbsoluteDegrees();
    const float homeErrorDeg = EncoderTracker::shortestDeltaDegrees(absoluteDeg, getHomeAngleDeg());

    const float remainingDeg = homeRemainingDeg(absoluteDeg);
    if ((bestRemainingDeg - remainingDeg) >= Config::RETURN_HOME_MIN_PROGRESS_DEG) {
      bestRemainingDeg = remainingDeg;
      lastProgressMs = millis();
    } else if ((millis() - lastProgressMs) >= Config::HOMING_STALL_TIMEOUT_MS) {
      gMotor.stop();
      Serial.printf("[ERROR] Homing stalled: no progress toward home (err=%.2f deg)\n", homeErrorDeg);
      NetworkManager::queueEvent(buildEventJson("error", "homing_stalled").c_str());
      return false;
    }

    // Публикация актуальной позиции энкодера в общее состояние на каждом шаге
    // homing, чтобы [DIAG] и API /v1/status показывали движение вала ВО ВРЕМЯ homing
    // (этот цикл теперь выполняется в motorTask; loop() на другом ядре сообщает данные параллельно).
    updateEncoderShared(true, absoluteDeg, gEncoder.getRelativeDegrees(),
                        gMotor.getExpectedDegrees(), false, homeErrorDeg);

    if (fabsf(homeErrorDeg) <= Config::ENCODER_HOME_WINDOW_DEG) {
      gMotor.stop();
      // В окне дома вал стоит намеренно - это не застревание.
      lastProgressMs = millis();

      // Не принимать домашнюю позицию по одному удачному отсчёту. После
      // переключения питания драйвера вал может ещё расслабляться или механически
      // догонять на короткий момент. Требуется несколько последовательных отсчётов
      // энкодера в пределах окна после короткого успокоения перед захватом логического нуля.
      delay(Config::HOMING_VERIFY_SETTLE_MS);

      bool confirmedHome = true;
      for (uint8_t i = 0; i < Config::HOMING_VERIFY_SAMPLES; ++i) {
        if (!gEncoder.update()) {
          Serial.println("[ERROR] Encoder read failed during homing verify");
          return false;
        }

        const float verifyDeg = gEncoder.getAbsoluteDegrees();
        const float verifyErrorDeg = EncoderTracker::shortestDeltaDegrees(verifyDeg, getHomeAngleDeg());
        if (fabsf(verifyErrorDeg) > Config::ENCODER_HOME_WINDOW_DEG) {
          confirmedHome = false;
          break;
        }

        delay(10);
      }

      if (confirmedHome) {
        break;
      }

      continue;
    }

    // Направление - всегда Config::HOME_DIRECTION (кроме небольшого перелёта), см.
    // homeStepDir. Нагрузка здесь никогда не выбирает направление.
    const int32_t stepDir = homeStepDir(policy, absoluteDeg);

    // Реактивная защита от стенки: когда идёт сброс нагрузки (движение -), но
    // нагрузка вместо этого ПОДНИМАЕТСЯ над базовой линией прохода (текущим
    // минимумом) на запас, это движение в стенку с другой стороны / заклинивший
    // механизм -> остановка и ERROR. Именно ПОДЪЁМ над базовой линией, а не
    // абсолютный уровень, поэтому законный сброс реальной нагрузки (~2 кг)
    // допустим. Только на дуге нагрузки: вне её (обход по кругу) подъём нагрузки -
    // штатный заход кулачка, а застревание ловит детектор выше.
    if (stepDir < 0 && onLoadArc(absoluteDeg)) {
      const float loadKg = getLoadKg();
      if (baselineLoadKg < 0.0f || loadKg < baselineLoadKg) {
        baselineLoadKg = loadKg;
      }
      if (loadKg > baselineLoadKg + Config::HOMING_LOAD_RISE_MARGIN_KG) {
        gMotor.stop();
        Serial.println("[ERROR] Homing load rose while shedding load - wall/jam");
        NetworkManager::queueEvent(buildEventJson("error", "homing_load_rise").c_str());
        return false;
      }
    }

    gMotor.moveSteps(stepDir, Config::HOME_STEP_INTERVAL_US);

    if (++safetyCounter > (Config::HOME_MAX_TRAVEL_STEPS / Config::HOME_STEP_CHUNK)) {
      Serial.println("[ERROR] Homing failed: target home angle not reached");
      return false;
    }
  }

  gEncoder.captureLogicalZero();
  gMotor.resetExpectedPosition();
  gHomeReferenceSteps = 0;
  updateEncoderShared(true, gEncoder.getAbsoluteDegrees(), gEncoder.getRelativeDegrees(), gMotor.getExpectedDegrees(), false, 0.0f);

  Serial.printf("[HOMING] Moved to configured home angle %.2f deg\n", getHomeAngleDeg());
  return true;
}

void moveTowardHome() {
  // Возврат в домашнюю позицию основан на угле энкодера: остановка только тогда,
  // когда вал физически вернулся к заданному домашнему углу. Ожидаемые шаги мотора
  // всё ещё используются как жёсткий предел хода, но им не доверяют как условию остановки.
  const float absoluteDeg = gEncoder.getAbsoluteDegrees();
  const float homeErrorDeg = EncoderTracker::shortestDeltaDegrees(absoluteDeg, getHomeAngleDeg());
  const float absHomeErrorDeg = fabsf(homeErrorDeg);
  // Прогресс меряется по оставшемуся пути маршрута homing (всегда в одну сторону),
  // а не по кратчайшему углу - см. homeRemainingDeg.
  const float remainingDeg = homeRemainingDeg(absoluteDeg);

  if ((gReturnHomeLastProgressMs == 0U) ||
      ((gReturnHomeBestErrorDeg - remainingDeg) >= Config::RETURN_HOME_MIN_PROGRESS_DEG)) {
    gReturnHomeBestErrorDeg = remainingDeg;
    gReturnHomeLastProgressMs = millis();
  } else if ((millis() - gReturnHomeLastProgressMs) >= Config::RETURN_HOME_STALL_TIMEOUT_MS) {
    gMotor.stop();
    setState(SystemState::ERROR);
    Serial.printf("[ERROR] Return home stalled: no encoder progress (err=%.2f deg)\n", homeErrorDeg);
    NetworkManager::queueEvent(buildEventJson("error", "return_home_stalled").c_str());
    return;
  }

  if (absHomeErrorDeg <= Config::ENCODER_HOME_WINDOW_DEG) {
    gMotor.stop();
    delay(Config::RETURN_HOME_VERIFY_SETTLE_MS);

    bool confirmedHome = true;
    for (uint8_t i = 0; i < Config::RETURN_HOME_VERIFY_SAMPLES; ++i) {
      if (!gEncoder.update()) {
        Serial.println("[ERROR] Encoder read failed during return-home verify");
        setState(SystemState::ERROR);
        NetworkManager::queueEvent(buildEventJson("error", "encoder_read_verify").c_str());
        return;
      }

      const float verifyDeg = gEncoder.getAbsoluteDegrees();
      const float verifyErrorDeg = EncoderTracker::shortestDeltaDegrees(verifyDeg, getHomeAngleDeg());
      if (fabsf(verifyErrorDeg) > Config::ENCODER_HOME_WINDOW_DEG) {
        confirmedHome = false;
        break;
      }

      delay(10);
    }

    if (confirmedHome) {
      gMotor.resetExpectedPosition();
      gEncoder.captureLogicalZero();
      gHomeReferenceSteps = 0;
      gReturnHomeBestErrorDeg = 0.0f;
      gReturnHomeLastProgressMs = 0;
      setState(SystemState::IDLE);
      updateEncoderShared(true, gEncoder.getAbsoluteDegrees(), gEncoder.getRelativeDegrees(), gMotor.getExpectedDegrees(), false, 0.0f);
      Serial.println("[HOME] Reached home angle by encoder, state -> IDLE");
      return;
    }
  }

  // Защита: ограничивает общий ход, чтобы возврат, который никогда не достигает
  // домашней позиции, не мог вращаться бесконечно. Это жёсткая защита, отличная от
  // предупреждения WARN о расхождении энкодера.
  if (abs(gMotor.getExpectedSteps() - gHomeReferenceSteps) > static_cast<int32_t>(Config::HOME_MAX_TRAVEL_STEPS)) {
    gMotor.stop();
    setState(SystemState::ERROR);
    Serial.println("[ERROR] Home return travel limit exceeded");
    NetworkManager::queueEvent(buildEventJson("error", "home_travel_limit").c_str());
    return;
  }

  // Направление - всегда Config::HOME_DIRECTION (кроме небольшого перелёта), см.
  // homeStepDir. Не на основе нагрузки.
  const int32_t stepDir = homeStepDir(HomeDirPolicy::BY_LOAD, absoluteDeg);

  // Реактивная защита от стенки: сброс нагрузки (движение -), но нагрузка
  // ПОДНИМАЕТСЯ над базовой линией прохода (текущим минимумом в
  // gReturnHomeBaselineKg) на запас, означает движение в стенку / заклинивание.
  // Именно ПОДЪЁМ над базовой линией, а не абсолютный уровень, поэтому нормальный
  // возврат из-под реальной удерживаемой нагрузки (~2 кг) сбрасывается корректно.
  // Только на дуге нагрузки (см. runHoming).
  if (stepDir < 0 && onLoadArc(absoluteDeg)) {
    const float loadKg = getLoadKg();
    if (gReturnHomeBaselineKg < 0.0f || loadKg < gReturnHomeBaselineKg) {
      gReturnHomeBaselineKg = loadKg;
    }
    if (loadKg > gReturnHomeBaselineKg + Config::HOMING_LOAD_RISE_MARGIN_KG) {
      gMotor.stop();
      setState(SystemState::ERROR);
      Serial.println("[ERROR] Return home load rose while shedding load - wall/jam");
      NetworkManager::queueEvent(buildEventJson("error", "homing_load_rise").c_str());
      return;
    }
  }

  gMotor.moveSteps(stepDir, Config::HOME_STEP_INTERVAL_US);
}

// ── Калибровка нуля кулачков: один такт состояния CALIBRATE ──────────────────
// Вызывается из motorTask (владелец мотора/энкодера) на каждом такте, пока состояние
// CALIBRATE. В режиме калибровки вал стоит (мотор включён и держит положение), пока
// оператор не попросит движения:
//   * непрерывное вращение (gCalibRunDir = +1 / -1) до «стоп»;
//   * поворот на заданный угол (остаток в gCalibJogPending, по шагу за такт).
// Плюс запросы:
//   * «сохранить» (gCalibFinishRequested): остановка, успокоение, текущий абсолютный
//     угол энкодера становится НОВЫМ домашним углом, сохраняется в NVS -> IDLE.
//     Событие "calib": "home=<новый> prev=<старый>".
//   * «отмена» (gCalibCancelRequested): ноль НЕ меняется, homing к прежнему нулю.
//   * «стоп» (gCalibHaltRequested, кнопка на стенде): прекратить движение.
// Защиты (движение прекращается, режим сохраняется, событие "calib"):
//   * непрерывное вращение дольше CALIB_MAX_TRAVEL_STEPS -> "aborted_travel";
//   * рост нагрузки -> "aborted_load".
// Бездействие дольше CALIB_IDLE_TIMEOUT_MS -> выход в IDLE без движения, "timeout".
void calibrateTick() {
  static uint8_t overloadTicks = 0;
  static bool wasMoving = false;

  if (gCalibFinishRequested) {
    gCalibFinishRequested = false;
    gCalibCancelRequested = false;
    gCalibRunDir = 0;
    clearCalibJog();
    overloadTicks = 0;
    wasMoving = false;
    gMotor.stop();
    delay(Config::HOMING_VERIFY_SETTLE_MS);
    if (!gEncoder.update()) {
      setState(SystemState::ERROR);
      Serial.println("[ERROR] Encoder read failed while capturing calibrated zero");
      NetworkManager::queueEvent(buildEventJson("error", "calib_encoder_read").c_str());
      return;
    }
    const float prevHomeDeg = getHomeAngleDeg();
    const float newHomeDeg = gEncoder.getAbsoluteDegrees();
    setHomeAngleDeg(newHomeDeg);
    gEncoder.setHomeAngle(newHomeDeg);
    gEncoder.captureLogicalZero();
    gMotor.resetExpectedPosition();
    gHomeReferenceSteps = 0;
    ConfigStore::saveHomeAngle(newHomeDeg);
    updateEncoderShared(true, newHomeDeg, gEncoder.getRelativeDegrees(), gMotor.getExpectedDegrees(), false, 0.0f);
    setState(SystemState::IDLE);
    char detail[48];
    snprintf(detail, sizeof(detail), "home=%.2f prev=%.2f",
             static_cast<double>(newHomeDeg), static_cast<double>(prevHomeDeg));
    NetworkManager::queueEvent(buildEventJson("calib", detail).c_str());
    Serial.printf("[CALIB] New zero captured: home %.2f deg (was %.2f) -> IDLE\n",
                  static_cast<double>(newHomeDeg), static_cast<double>(prevHomeDeg));
    return;
  }

  if (gCalibCancelRequested) {
    gCalibCancelRequested = false;
    gCalibRunDir = 0;
    clearCalibJog();
    overloadTicks = 0;
    wasMoving = false;
    gMotor.stop();
    // Через IDLE: ветка gRehomeRequested в motorTask принимает запрос только из
    // IDLE/ERROR и сама ведёт вал к ПРЕЖНЕМУ (не изменённому) нулю.
    setState(SystemState::IDLE);
    gRehomePolicy = HomeDirPolicy::BY_LOAD;
    gRehomeRequested = true;
    NetworkManager::queueEvent(buildEventJson("calib", "cancelled").c_str());
    Serial.println("[CALIB] Cancelled, zero unchanged, re-homing to previous zero");
    return;
  }

  if (gCalibHaltRequested) {
    gCalibHaltRequested = false;
    gCalibRunDir = 0;
    clearCalibJog();
  }

  // Что делаем на этом такте: шаг непрерывного вращения, шаг поворота или стоим.
  int8_t dir = 0;
  if (gCalibRunDir != 0) {
    dir = gCalibRunDir;
    // Защита от забытого вращения.
    if (abs(gMotor.getExpectedSteps() - gCalibStartSteps) >= Config::CALIB_MAX_TRAVEL_STEPS) {
      gCalibRunDir = 0;
      dir = 0;
      NetworkManager::queueEvent(buildEventJson("calib", "aborted_travel").c_str());
      Serial.println("[CALIB] Rotation stopped: travel limit reached");
    }
  } else {
    dir = takeCalibJogStep();
  }

  if (dir == 0) {
    if (wasMoving) {
      wasMoving = false;
      overloadTicks = 0;
      gMotor.stop();
      gCalibActivityMs = millis();
    }
    if ((millis() - gCalibActivityMs) >= Config::CALIB_IDLE_TIMEOUT_MS) {
      setState(SystemState::IDLE);
      NetworkManager::queueEvent(buildEventJson("calib", "timeout").c_str());
      Serial.println("[CALIB] Idle timeout, leaving calibration (zero unchanged)");
    }
    return;
  }
  wasMoving = true;

  // Защита по нагрузке: кулачки вдавливаются в плату. Порог отсчитывается и от
  // абсолютного уровня, и от нагрузки в начале движения, чтобы можно было
  // двигаться в сторону сброса, даже когда кулачки уже поджимают плату.
  const float loadKg = getLoadKg();
  const float limitKg = max(Config::CALIB_MAX_LOAD_KG,
                            static_cast<float>(gCalibStartLoadKg) + Config::CALIB_LOAD_RISE_KG);
  if (getLoadReady() && loadKg > limitKg) {
    if (++overloadTicks >= Config::CALIB_OVERLOAD_TICKS) {
      overloadTicks = 0;
      gCalibRunDir = 0;
      clearCalibJog();
      gMotor.stop();
      NetworkManager::queueEvent(buildEventJson("calib", "aborted_load").c_str());
      Serial.printf("[CALIB] Motion stopped: load %.3fkg above limit %.3fkg\n",
                    static_cast<double>(loadKg), static_cast<double>(limitKg));
      return;
    }
  } else {
    overloadTicks = 0;
  }

  gMotor.moveSteps(dir, Config::CALIB_STEP_INTERVAL_US);
  gCalibActivityMs = millis();
}

void loadTask(void* parameter) {
  (void)parameter;
  gLoadCell.begin();

  for (;;) {
    // Актуальное runtime-смещение нуля передаётся в тензодатчик ДО чтения, чтобы
    // перетарирование (обновляющее gOffset) вступило в силу уже на следующем
    // вычислении kg здесь. Приведение к long: смещение - это ноль в сырых
    // отсчётах, а LoadCellManager хранит/использует его как long (чтобы
    // LoadCellManager не зависел от глобального gOffset - смещением владеет .ino
    // и передаёт его сюда).
    gLoadCell.setOffset(static_cast<long>(getOffset()));
    gLoadCell.update();
    updateLoadShared(gLoadCell.getMassKg(), gLoadCell.isReady(), gLoadCell.getRawValue(), gLoadCell.getFilteredRaw());
    vTaskDelay(pdMS_TO_TICKS(Config::LOAD_TASK_PERIOD_MS));
  }
}

// Выполняет один физический шаг мотора в направлении `dir` (+1 добавляет
// нагрузку, -1 сбрасывает нагрузку), останавливается и возвращает фактическое
// перемещение энкодера (в градусах, знаковое кратчайшее). Используется и для
// продвижения, и для обнаружения застрявшего/проскальзывающего вала у стенки.
// `stepIntervalUs` задаёт скорость импульса: быстрый импульс для дальнего
// подхода, медленный высокомоментный импульс для аккуратного финального
// центрирования.
float stepAndMeasureEncoderMove(int8_t dir, uint32_t stepIntervalUs) {
  gEncoder.update();
  const float beforeDeg = gEncoder.getAbsoluteDegrees();

  gMotor.moveSteps(dir > 0 ? 1 : -1, stepIntervalUs);
  gMotor.stop();

  gEncoder.update();
  const float afterDeg = gEncoder.getAbsoluteDegrees();
  return EncoderTracker::shortestDeltaDegrees(beforeDeg, afterDeg);
}

// Продвигается по одному шагу к ЦЕНТРУ окна целевой нагрузки и паркуется в
// точке, где нагрузка В ПОКОЕ ближе всего к центру, затем останавливается и
// остаётся на месте.
//
// Все решения принимаются по устоявшемуся мгновенному отсчёту (мотор
// остановлен, механизм расслаблен) - никогда по запаздывающей медиане, что и
// заставляло прежний контроллер колебаться. Ключевое поведение согласно
// заданному управлению:
//   * всегда шагать к центру: ниже него -> добавить нагрузку, выше -> сбросить;
//   * далеко от центра нагрузка наращивается БЫСТРО (быстрый импульс +
//     минимальное успокоение); финальное центрирование около окна всегда
//     переключается на МЕДЛЕННЫЙ высокомоментный импульс и полное успокоение,
//     чтобы центр выбирался аккуратно. Случайный небольшой перелёт на быстрой
//     фазе не страшен - аккуратная фаза его исправляет;
//   * если ОДИН шаг перескакивает через центр, паркуется на той стороне - до
//     или после шага - что ближе к центру, и ПРЕКРАЩАЕТ поиск;
//   * если вал перестаёт вращаться (механическая стенка / проскальзывание),
//     паркуется на максимально достижимой нагрузке. Это НИКОГДА не ошибка: на
//     этом стенде стенка - это зажатый между плитами объект, и «насколько
//     хватает хода» - допустимая, ожидаемая точка парковки.
//
// Скорость измерения масштабируется по близости к зоне действия: отсчёт COARSE
// (почти без ожидания), пока объект ещё не зажат и нагрузка плоская;
// FAST, когда нагрузка растёт; PRECISE около центра, где принимается решение о
// парковке. Число шагов мотора при этом НЕ меняется.
//
// Чисто прерывается (возвращает false) только если состояние покидает
// `activePhase` (например, кнопка запросила RETURN_HOME). При успехе паркует
// мотор, записывает финальную устоявшуюся нагрузку в *outKg и возвращает true.
// ERROR при этом не поднимается.
bool parkIntoWindow(SystemState activePhase, float* outKg, ParkReason* outReason) {
  gMotor.stop();

  // Один атомарный снимок конфигурации для выбора начального режима до цикла,
  // повторяющий поитерационный снимок, взятый ниже.
  ConfigTriple cfg0;
  getConfigSnapshot(cfg0);

  float settledKg = 0.0f;
  // Начальный режим измерения выбирается по расстоянию до центра.
  MeasureMode mode = MeasureMode::COARSE;
  measureSettledKg(&settledKg, mode);
  if (settledKg >= Config::SEEK_ENGAGED_LOAD_KG) {
    mode = (fabsf(settledKg - cfg0.targetKg) <= Config::SEEK_PRECISE_BAND_KG)
               ? MeasureMode::PRECISE
               : MeasureMode::FAST;
    measureSettledKg(&settledKg, mode);
  }

  uint8_t stallCount = 0;      // подряд идущих шагов без ВПЕРЁД-движения вала
  uint8_t noGainSteps = 0;     // подряд идущих шагов добавления нагрузки без НАКОПЛЕННОГО прироста пика
  int32_t stepsTaken = 0;
  // Максимальная устоявшаяся нагрузка, достигнутая за этот поиск. Парковка
  // происходит здесь при любом условии стенки, а НЕ по текущему отсчёту - как
  // только объект проскальзывает/отпускает, текущая нагрузка уже обрушилась, но
  // пик - это истинная «максимально достижимая нагрузка».
  float peakKg = settledKg;
  // Опорное значение, против которого детектор плато измеряет НАКОПЛЕННЫЙ
  // прирост. Сбрасывается на текущий пик всякий раз, когда пик продвигается на
  // WALL_LOAD_GAIN_KG, чтобы детектор спрашивал «поднялся ли пик на GAIN за
  // последние N шагов» (здоровый медленный подъём ~ESTIMATED_KG_PER_STEP за шаг
  // ДЕЙСТВИТЕЛЬНО накапливается сверх порога), а не «побил ли ОДИН шаг пик на
  // GAIN» (чего медленный подъём никогда не делает).
  float gainReferenceKg = settledKg;

  // Базовое проскальзывание вала относительно мотора, захваченное на входе в
  // поиск. Детектор (3) измеряет проскальзывание, развившееся ЗА ЭТОТ поиск
  // (slipErrorDeg - baseline), а не накопленный дрейф с последнего homing -
  // иначе повторный поиск из HOLD (который не выполняет повторный homing)
  // начинался бы с большим остаточным проскальзыванием и ложно срабатывал на
  // первом шаге.
  float baselineSlipDeg = 0.0f;
  checkEncoderSlip(&baselineSlipDeg);

  for (;;) {
    // Поля энкодера в [DIAG] поддерживаются актуальными, пока работает этот
    // блокирующий цикл. Поля нагрузки обновляются сами через loadTask на другом
    // ядре. Захватывается ошибка положения вала относительно мотора (ожидаемые
    // шаги минус фактическое перемещение энкодера), чтобы детектор стенки по
    // проскальзыванию ниже видел отставание вала.
    float slipErrorDeg = 0.0f;
    checkEncoderSlip(&slipErrorDeg);

    // Один атомарный снимок конфигурации на итерацию, чтобы target/min/max,
    // используемые в решениях этого шага, были взаимно согласованы, даже если
    // POST /v1/config приходит в середине поиска.
    ConfigTriple cfg;
    getConfigSnapshot(cfg);

    if (getState() != activePhase) {
      *outKg = settledKg;
      *outReason = ParkReason::IN_WINDOW;
      return false;
    }

    // Внутри окна И на/выше центра, либо уже настолько близко к центру,
    // насколько позволяет один шаг -> запарковано. (Пока значение ниже центра,
    // шагание к центру продолжается даже изнутри окна, пока не будет достигнут
    // центр, либо шаг его не перелетит, либо стенка не остановит.)
    const bool inWindow = settledKg >= cfg.minKg && settledKg <= cfg.maxKg;
    if (inWindow && settledKg >= cfg.targetKg) {
      gMotor.stop();
      *outKg = settledKg;
      *outReason = ParkReason::IN_WINDOW;
      return true;
    }

    // Бюджет шагов исчерпан без достижения окна. Парковка на текущем месте
    // (максимально достижимая нагрузка) вместо ошибки, чтобы оператор всегда мог
    // затем вернуться в домашнюю позицию кнопкой.
    if (stepsTaken >= Config::SEEK_MAX_STEPS) {
      gMotor.stop();
      *outKg = settledKg;
      Serial.printf("[PARK] Step budget reached, parking at %.3fkg\n", settledKg);
      *outReason = ParkReason::STEP_BUDGET;
      return true;
    }

    // Всегда шагать к центру. Пока далеко (режимы измерения COARSE/FAST),
    // используется БЫСТРЫЙ импульс; для аккуратного центрирования (режим PRECISE)
    // выполняется переход на МЕДЛЕННЫЙ высокомоментный импульс.
    const int8_t dir = (settledKg < cfg.targetKg) ? 1 : -1;
    const float beforeKg = settledKg;
    const uint32_t pulseUs =
        (mode == MeasureMode::PRECISE) ? Config::SEEK_STEP_INTERVAL_US : Config::SEEK_STEP_INTERVAL_FAST_US;

    // Состояние перепроверяется непосредственно перед шагом мотора. Проверка в
    // начале цикла (выше) может пройти, а затем сетевой STOP -> HOLD_LOAD
    // приходит на ядро 0 во время успокоения ниже; без этой проверки-ограничителя
    // один лишний физический шаг сработал бы за позицию STOP до того, как это
    // заметит следующая проверка в начале цикла. Это жёсткая заморозка, поэтому
    // прерывание идёт ДО шага, а не после.
    if (getState() != activePhase) {
      *outKg = settledKg;
      *outReason = ParkReason::IN_WINDOW;
      return false;
    }

    const float encMoveDeg = stepAndMeasureEncoderMove(dir, pulseUs);
    ++stepsTaken;
    measureSettledKg(&settledKg, mode);
    // Режим измерения переоценивается по текущему положению.
    if (settledKg < Config::SEEK_ENGAGED_LOAD_KG) {
      mode = MeasureMode::COARSE;
    } else if (fabsf(settledKg - cfg.targetKg) <= Config::SEEK_PRECISE_BAND_KG) {
      if (mode != MeasureMode::PRECISE) {
        mode = MeasureMode::PRECISE;
        measureSettledKg(&settledKg, mode);
      }
    } else {
      mode = MeasureMode::FAST;
    }

    // ── Обнаружение механической стенки ──────────────────────────────────────
    // Стенка - это зажатый между плитами объект; когда дальше продвинуться
    // нельзя, парковка происходит на МАКСИМАЛЬНОЙ достигнутой нагрузке (peakKg).
    // Это нормальный, ожидаемый исход на этом стенде, а НЕ ошибка - homing/возврат
    // остаётся доступным после. Работают ТРИ независимых детектора, поскольку
    // режимы отказа физически различны и ни один по отдельности их не покрывает:
    //   (1) плато нагрузки - пик нагрузки перестаёт расти, хотя шаги добавляются;
    //   (2) обрушение нагрузки - объект проскальзывает/отпускает, нагрузка падает ниже пика;
    //   (3) проскальзывание вала/мотора - вал больше не может следовать заданным шагам.
    // Плюс классический угловой стопор (вал перестаёт вращаться) для жёсткой,
    // тихой стенки. Детекторы на основе нагрузки применяются только при
    // ДОБАВЛЕНИИ нагрузки (dir > 0); сброс к более низкой цели - другой режим,
    // обрабатываемый через crossedCenter.

    // Отслеживание текущего пика.
    const bool addingLoad = dir > 0;
    if (settledKg > peakKg) {
      peakKg = settledKg;
    }

    auto parkAtWall = [&](const char* why) -> void {
      gMotor.stop();
      *outKg = peakKg;
      Serial.printf("[PARK] Wall (%s): parking at peak load %.3fkg (current %.3fkg, seek-slip %.2fdeg)\n",
                    why, static_cast<double>(peakKg), static_cast<double>(settledKg),
                    static_cast<double>(slipErrorDeg - baselineSlipDeg));
      *outReason = ParkReason::WALL;
    };

    // Детекторы на основе нагрузки (обрушение, плато) имеют смысл лишь после
    // того, как объект ЗАЖАТ и нагрузка действительно откликается на
    // движение. На этом стенде в начале поиска есть большая мёртвая зона: вал
    // проворачивается на много градусов (abs 106 -> 120+), пока нагрузка держится
    // плоской у нуля, а механизм выбирает люфт. В этой фазе нагрузка закономерно
    // НЕ растёт, поэтому плато/обрушение ложно сработали бы и запарковали на ~0 кг
    // (наблюдавшийся баг «паркуется через пару шагов»). До задействования поиск
    // ведётся только детекторами углового стопора / проскальзывания вала ниже
    // (вал там ещё свободно вращается, поэтому ни один не срабатывает). Опорное
    // значение плато удерживается на текущей нагрузке, пока объект не зажат,
    // чтобы счётчик стартовал с чистого листа в момент зажатия объекта.
    const bool engaged = settledKg >= Config::SEEK_ENGAGED_LOAD_KG;

    // (2) Обрушение нагрузки: резкое падение существенно ниже пика == объект
    // отпустил / проскользнул за стенку. Немедленная парковка на пике (максимум).
    if (addingLoad && engaged && peakKg - settledKg > Config::WALL_LOAD_DROP_KG) {
      parkAtWall("load-collapse");
      return true;
    }

    // (1) Плато нагрузки: пик нагрузки больше не растёт, ПОКА ВАЛ НАПРЯГАЕТСЯ в
    // стенку. Тест напряжения (seek-slip > WALL_PLATEAU_MIN_SLIP_DEG) отделяет
    // настоящую стенку от обычного медленного раннего наращивания: около стенки
    // вал не может провернуться дальше и отстаёт от мотора (проскальзывание
    // растёт); при свободном раннем нагружении вал вращается легко
    // (проскальзывание ~0), а нагрузка лишь мягко растёт - это НЕ должно
    // считаться плато, как бы медленно она ни поднималась. Критерий не зависит от
    // уровня (стенка не фиксирована): «вал нагружен И нагрузка не откликается»,
    // никогда не абсолютный уровень в кг. Тест НАКОПЛЕННОГО прироста: сброс только
    // когда пик поднимается на WALL_LOAD_GAIN_KG над опорным; настоящее плато его
    // никогда не продвигает, поэтому как только вал напрягается, счётчик доходит
    // до предела.
    const bool shaftStraining = fabsf(slipErrorDeg - baselineSlipDeg) > Config::WALL_PLATEAU_MIN_SLIP_DEG;
    if (addingLoad && peakKg > gainReferenceKg + Config::WALL_LOAD_GAIN_KG) {
      // Пик продвинулся: честный прогресс, сброс независимо от проскальзывания.
      gainReferenceKg = peakKg;
      noGainSteps = 0;
    } else if (addingLoad && shaftStraining) {
      // Нет прогресса И вал давит с усилием -> это стенка.
      if (++noGainSteps >= Config::WALL_NO_GAIN_STEPS) {
        parkAtWall("load-plateau");
        return true;
      }
    } else {
      // Свободное раннее наращивание (вал не напряжён) или сброс (dir<0): плато
      // НЕ накапливается, а опорное значение привязывается к текущей нагрузке,
      // чтобы счётчик стартовал с чистого листа, когда вал действительно начнёт
      // напрягаться. Для сброса это также не даёт устаревшему опорному значению
      // историчного пика ложно сработать при позднем повышении цели в середине
      // поиска (peakKg монотонен).
      noGainSteps = 0;
      gainReferenceKg = settledKg;
    }

    // (3) Проскальзывание вала/мотора: вал отстал от заданных шагов слишком
    // сильно, чтобы когда-либо догнать - он застрял в стенке. Измеряется
    // ОТНОСИТЕЛЬНО базового значения на входе в поиск, чтобы повторный поиск из
    // HOLD (без повторного homing, с большим остаточным проскальзыванием)
    // оценивался по проскальзыванию, развившемуся за ЭТОТ поиск, а не с homing.
    if (addingLoad && fabsf(slipErrorDeg - baselineSlipDeg) > Config::WALL_SLIP_DEG) {
      parkAtWall("shaft-slip");
      return true;
    }

    // (4) Угловой стопор: вал совершил (почти) нулевое движение ВПЕРЁД за этот
    // шаг. Считается «нет продвижения вперёд», а НЕ «нет движения»: обратное
    // проскальзывание (encMoveDeg противоположно dir) НЕ сбрасывает счётчик - тот
    // сброс и был багом, из-за которого пила молола бесконечно. Движение вперёд
    // сверх порога сбрасывает счётчик.
    const float forwardMoveDeg = encMoveDeg * static_cast<float>(dir);
    if (forwardMoveDeg < Config::SEEK_STALL_MIN_STEP_DEG) {
      if (++stallCount >= Config::SEEK_STALL_LIMIT) {
        parkAtWall("angular-stall");
        return true;
      }
    } else {
      stallCount = 0;
    }

    // Перескочил ли этот один шаг через центр? Если да, то один шаг грубее
    // расстояния до центра: парковка на той стороне, что ближе к центру, и
    // прекращение поиска (НЕ шагать взад-вперёд в погоне за точным центром).
    const bool crossedCenter =
        ((beforeKg < cfg.targetKg) && (settledKg >= cfg.targetKg)) ||
        ((beforeKg > cfg.targetKg) && (settledKg <= cfg.targetKg));
    if (crossedCenter) {
      const float distBefore = fabsf(beforeKg - cfg.targetKg);
      const float distAfter = fabsf(settledKg - cfg.targetKg);
      if (distBefore <= distAfter) {
        // Точка до этого шага была ближе к центру: аккуратный возврат шагом на
        // неё (медленный импульс).
        stepAndMeasureEncoderMove(static_cast<int8_t>(-dir), Config::SEEK_STEP_INTERVAL_US);
        measureSettledKg(&settledKg, MeasureMode::PRECISE);
      }
      gMotor.stop();
      *outKg = settledKg;
      *outReason = ParkReason::IN_WINDOW;
      return true;
    }
  }
}

void motorTask(void* parameter) {
  (void)parameter;
  uint8_t encoderFaultCounter = 0;
  uint8_t encoderReadErrorCounter = 0;
  SystemState lastState = getState();
  uint32_t lastWarnMs = 0;
  // Нагрузка в кг в точке, где контроллер запарковался (SEEK/HOLD). После
  // парковки шаги больше НЕ выполняются, пока устоявшаяся нагрузка сама не уйдёт
  // за этот уровень более чем на HOLD_REACQUIRE_MARGIN_KG, то есть нагрузка
  // изменилась сама по себе.
  float parkedLoadKg = 0.0f;
  bool holdParked = false;

  for (;;) {
    // Однократный boot homing, запрошенный из setup(). Выполняется ЗДЕСЬ (не в
    // setup()), поэтому setup() вернулся сразу и loop()+сеть уже работают: [DIAG],
    // WiFi, кнопка и удалённые команды действуют ВО ВРЕМЯ этого homing, а если он
    // застрянет у стенки, оператор всё равно это видит и может вмешаться. При
    // успехе -> IDLE, при неудаче -> ERROR (восстанавливается кнопкой/по сети),
    // как и в прежнем блокирующем поведении boot homing.
    if (gBootHomeRequested) {
      gBootHomeRequested = false;
      if (runHoming(gRehomePolicy)) {
        setState(SystemState::IDLE);
        gHomeReferenceSteps = gMotor.getExpectedSteps();
        lastState = SystemState::IDLE;
        Serial.println("[STATE] IDLE");
      } else {
        setState(SystemState::ERROR);
        lastState = SystemState::ERROR;
        Serial.println("[ERROR] Boot homing failed");
        NetworkManager::queueEvent(buildEventJson("error", "boot_home_failed").c_str());
      }
      continue;
    }

    // Повторный homing по запросу кнопки/сети: выполняется здесь (motorTask
    // владеет энкодером/мотором), а не в loop(), чтобы избежать межъядерной гонки.
    // Восстанавливает из ERROR через homing (направление на основе угла и
    // геометрии дуги, поэтому безопасно под нагрузкой).
    if (gRehomeRequested) {
      // Состояние читается ДО сброса флага. HOME из IDLE выставляет этот флаг, не
      // меняя состояние, и ядро 0 может затем перевести IDLE -> SEEK_LOAD (кнопка
      // или сетевой `seek`) до того, как выполнится этот такт. Захват rehomeFrom
      // сначала, затем сброс, сохраняет согласованность решения с моментом
      // обслуживания запроса (само вытеснение намерения это не устраняет - оно
      // обрабатывается проверкой-ограничителем и логом ниже).
      const SystemState rehomeFrom = getState();
      gRehomeRequested = false;
      // Повторный homing по запросу из ERROR (восстановление) ИЛИ из IDLE
      // (идемпотентный home: runHoming завершается за ноль шагов, если уже в
      // домашней позиции, и доходит до неё, если вал сместился в простое). Оба
      // при успехе приводят к IDLE, при неудаче - к ERROR. Если состояние было
      // вытеснено (например, `seek` встрял и перевёл в SEEK_LOAD), повторный
      // homing намеренно отбрасывается - но это логируется, чтобы отброшенное
      // намерение HOME было видно, а не пропало молча.
      if (rehomeFrom == SystemState::ERROR || rehomeFrom == SystemState::IDLE) {
        holdParked = false;
        if (runHoming(gRehomePolicy)) {
          setState(SystemState::IDLE);
          gHomeReferenceSteps = gMotor.getExpectedSteps();
          lastState = SystemState::IDLE;
          Serial.println("[STATE] IDLE");
        } else {
          setState(SystemState::ERROR);
          Serial.println("[ERROR] Re-homing failed, still in ERROR");
          NetworkManager::queueEvent(buildEventJson("error", "rehome_failed").c_str());
        }
        continue;
      } else {
        Serial.printf("[NET] re-home dropped: state changed to %s before service\n",
                      toString(rehomeFrom));
      }
    }

    // Диагностические счётчики сбрасываются при любом переходе состояния, чтобы
    // рассогласование, накопленное в одном состоянии (например, SEEK_LOAD), не
    // переводило сразу следующее состояние (например, RETURN_HOME) в ERROR ещё до
    // его старта.
    const SystemState currentState = getState();
    if (currentState != lastState) {
      encoderFaultCounter = 0;
      encoderReadErrorCounter = 0;
      // ПРИМЕЧАНИЕ: holdParked здесь намеренно НЕ сбрасывается. SEEK -> HOLD -
      // это переход состояния, но ветка SEEK уже запарковалась и выставила
      // holdParked=true; сброс здесь заставил бы самый первый такт HOLD снова
      // вызвать parkIntoWindow (избыточная повторная парковка при крошечном
      // дрейфе нагрузки). holdParked сбрасывается явно при старте нового поиска
      // (ветка SEEK_LOAD) и после завершения повторной парковки.
      gReturnHomeBestErrorDeg = 0.0f;
      gReturnHomeLastProgressMs = 0;
      lastState = currentState;
      if (currentState == SystemState::RETURN_HOME) {
        const float absoluteDeg = gEncoder.getAbsoluteDegrees();
        gReturnHomeBestErrorDeg = homeRemainingDeg(absoluteDeg);
        gReturnHomeLastProgressMs = millis();
        // Новый прогон RETURN_HOME: базовое значение нагрузки для реактивной
        // защиты сбрасывается, чтобы оно заново захватило текущий минимум для
        // этого прогона. (Направление пересчитывается по углу каждый такт,
        // поэтому фиксатора направления для сброса нет.)
        gReturnHomeBaselineKg = -1.0f;
      }
    }

    float encoderError = 0.0f;
    const bool fault = checkEncoderSlip(&encoderError);

    portENTER_CRITICAL(&gShared.mux);
    const bool encoderReady = gShared.encoderReady;
    portEXIT_CRITICAL(&gShared.mux);

    if (!encoderReady) {
      ++encoderReadErrorCounter;
      if (encoderReadErrorCounter >= Config::ENCODER_READ_ERROR_LIMIT) {
        setState(SystemState::ERROR);
        gMotor.stop();
        Serial.println("[ERROR] Encoder read timeout/failure");
        NetworkManager::queueEvent(buildEventJson("error", "encoder_read_timeout").c_str());
      }
    } else {
      encoderReadErrorCounter = 0;
    }

    if (fault && encoderReady) {
      ++encoderFaultCounter;
      const SystemState state = getState();
      if (state == SystemState::HOMING) {
        // Критичным остаётся только стартовый HOMING: он ведётся по углу
        // энкодера, поэтому рассогласование там - настоящая проблема. Во время
        // RETURN_HOME предупреждение о рассогласовании остаётся только
        // диагностическим; отдельная проверка-ограничитель отсутствия прогресса в
        // moveTowardHome() явно обрабатывает застрявший / проскальзывающий возврат.
        if (encoderFaultCounter >= Config::ENCODER_ERROR_LIMIT) {
          setState(SystemState::ERROR);
          gMotor.stop();
          Serial.printf("[ERROR] Encoder mismatch %.2f deg\n", encoderError);
          NetworkManager::queueEvent(buildEventJson("error", "encoder_mismatch").c_str());
        }
      } else {
        // Только диагностика в SEEK_LOAD/HOLD_LOAD/RETURN_HOME. Сильное
        // ограничение частоты, чтобы лог оставался читаемым; текущее
        // рассогласование в любом случае всегда видно в [DIAG].
        if ((millis() - lastWarnMs) >= Config::WARN_THROTTLE_MS) {
          lastWarnMs = millis();
          Serial.printf("[WARN] Encoder mismatch %.2f deg\n", encoderError);
        }
      }
    } else {
      encoderFaultCounter = 0;
    }

    // Автоотключение мотора без нагрузки: в покое, если нагрузка не больше
    // MOTOR_RELEASE_MAX_KG дольше MOTOR_RELEASE_DELAY_MS, драйвер отключается через
    // EN. Включается обратно сам - первым же шагом (StepperMotor::moveSteps).
    if (Config::MOTOR_AUTO_RELEASE) {
      static uint32_t noLoadSinceMs = 0;
      const SystemState s = getState();
      const bool still = (s == SystemState::IDLE || s == SystemState::HOLD_LOAD ||
                          s == SystemState::ERROR);
      const bool noLoad = getLoadReady() && fabsf(getLoadKg()) <= Config::MOTOR_RELEASE_MAX_KG;
      if (still && noLoad && !gRehomeRequested && !gBootHomeRequested) {
        if (noLoadSinceMs == 0) noLoadSinceMs = millis() | 1U;
        if (gMotor.isEnabled() && (millis() - noLoadSinceMs) >= Config::MOTOR_RELEASE_DELAY_MS) {
          gMotor.setEnabled(false);
          NetworkManager::queueEvent(buildEventJson("motor", "released").c_str());
          Serial.println("[MOTOR] released (no load)");
        }
      } else {
        noLoadSinceMs = 0;
      }
    }

    switch (getState()) {
      case SystemState::BOOT:
      case SystemState::HOMING:
      case SystemState::ERROR:
        gMotor.stop();
        break;

      case SystemState::IDLE:
        break;

      case SystemState::SEEK_LOAD: {
        // Продвижение по одному шагу в окно и парковка в точке, ближайшей к
        // центру. Все решения используют устоявшиеся мгновенные отсчёты в покое.
        // Новый поиск начинается незапаркованным (этот флаг не сбрасывается при
        // переходах состояний, поэтому он выставляется здесь явно).
        holdParked = false;
        float parkKg = 0.0f;
        ParkReason reason = ParkReason::IN_WINDOW;
        if (parkIntoWindow(SystemState::SEEK_LOAD, &parkKg, &reason)) {
          parkedLoadKg = parkKg;
          holdParked = true;
          setState(SystemState::HOLD_LOAD);
          Serial.printf("[STATE] HOLD_LOAD parked at %.3fkg\n", parkedLoadKg);
          char detailBuf[24];
          snprintf(detailBuf, sizeof(detailBuf), "%.3fkg", static_cast<double>(parkedLoadKg));
          // Ровно одно событие по причине: в окне -> "park"; стенка/бюджет
          // шагов -> "wall" (цель не достигнута, парковка на максимально
          // достижимой нагрузке). Это заменяет прежнюю двойную отправку
          // (parkIntoWindow + здесь).
          const char* evt = (reason == ParkReason::IN_WINDOW) ? "park" : "wall";
          NetworkManager::queueEvent(buildEventJson(evt, detailBuf).c_str());
        }
        // Если parkIntoWindow вернул false, значит либо возник ERROR, либо
        // изменилось состояние (например, кнопка -> RETURN_HOME); здесь делать
        // нечего.
        break;
      }

      case SystemState::HOLD_LOAD: {
        // После парковки HOLD - это жёсткая заморозка: мотор удерживает свой
        // угол и больше НЕ трогается, как бы ни менялась измеряемая нагрузка.
        // Единственный выход из HOLD - кнопка (-> RETURN_HOME).
        //
        // Повторная калибровка / повторная парковка здесь намеренно ОТКЛЮЧЕНА. На
        // этом стенде тензодатчик измеряет объект, зажатый между плитами, а
        // угол энкодера задаёт силу зажима. Последующее изменение отсчёта - это
        // ИЗМЕРЕНИЕ этого объекта (например, он расслабляется, ползёт или это как
        // раз то, что нужно записать) - а НЕ возмущение, которое надо обнулять.
        // Шаг мотора для «коррекции» нагрузки изменил бы саму силу зажима, которая
        // и измеряется, и испортил бы результат. Поэтому после подтверждённой
        // парковки позиция удерживается неизменной, а нагрузке позволяется быть
        // какой есть.
        //
        // Прежняя логика обнаружения дрейфа + повторной парковки сохранена ниже,
        // закомментированной, как запись того, чего НЕ следует делать в этом
        // применении:
        //
        //   float settledKg = 0.0f;
        //   const bool quiet = measureSettledKg(&settledKg);
        //   if (!quiet) break;  // ещё устаканивается
        //   const bool driftedUp   = settledKg > (parkedLoadKg + Config::HOLD_REACQUIRE_MARGIN_KG);
        //   const bool driftedDown = settledKg < (parkedLoadKg - Config::HOLD_REACQUIRE_MARGIN_KG);
        //   const bool outOfWindow = settledKg < Config::TARGET_LOAD_MIN_KG ||
        //                            settledKg > Config::TARGET_LOAD_MAX_KG;
        //   if (holdParked && (driftedUp || driftedDown) && outOfWindow) {
        //     Serial.printf("[HOLD] Load drifted to %.3fkg, re-acquiring\n", settledKg);
        //     holdParked = false;
        //   }
        //   if (!holdParked) {
        //     float parkKg = 0.0f;
        //     if (parkIntoWindow(SystemState::HOLD_LOAD, &parkKg)) {
        //       parkedLoadKg = parkKg;
        //       holdParked = true;
        //       Serial.printf("[HOLD] Re-parked at %.3fkg\n", parkedLoadKg);
        //     }
        //   }
        (void)parkedLoadKg;
        (void)holdParked;
        gMotor.stop();
        break;
      }

      case SystemState::RETURN_HOME:
        moveTowardHome();
        break;

      case SystemState::CALIBRATE:
        calibrateTick();
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(Config::MOTOR_TASK_PERIOD_MS));
  }
}

void setup() {
  delay(250);
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32-S3 load stand controller booting");
  Serial.println("[BOOT] Serial online");
  Serial.println("========================================");

  // Громкое предупреждение, если учётные данные WiFi всё ещё стандартная
  // заглушка. При WIFI_SSID == "CHANGE_ME" сетевой стек никогда не подключится,
  // поэтому дашборд и все удалённые команды молча недоступны. Только
  // предупреждение; загрузка продолжается, так что физическая кнопка работает.
  if (strcmp(Config::WIFI_SSID, "CHANGE_ME") == 0) {
    Serial.println("========================================");
    Serial.println("[BOOT][WARN] WiFi SSID is still the placeholder CHANGE_ME - network/dashboard will NOT connect.");
    Serial.println("[BOOT][WARN] Edit Config.h WIFI_SSID/WIFI_PASSWORD and reflash to enable remote control.");
    Serial.println("========================================");
  }

  gButton.begin();
  Serial.println("[BOOT] Stepper driver power applied");
  gMotor.begin();
  Serial.printf("[BOOT] Waiting %lu ms for driver power settle\n", static_cast<unsigned long>(Config::DRIVER_POWER_SETTLE_MS));
  delay(Config::DRIVER_POWER_SETTLE_MS);

  // Домашний угол (логический ноль): сохранённый калибровкой нуля кулачков либо
  // значение по умолчанию из Config.h. Загружается ДО homing, который к нему ведёт.
  {
    const float homeDeg = ConfigStore::loadHomeAngle(Config::HOME_ABSOLUTE_ANGLE_DEG);
    setHomeAngleDeg(homeDeg);
    gEncoder.setHomeAngle(homeDeg);
    Serial.printf("[CFG] home angle %.2f deg\n", static_cast<double>(homeDeg));
  }

  const bool encoderOk = gEncoder.begin(Wire);
  if (!encoderOk) {
    setState(SystemState::ERROR);
    Serial.println("[ERROR] AS5600 init failed");
  } else {
    // Homing здесь НЕ запускается (он блокировал бы setup() до достижения
    // домашней позиции). Вместо этого он запрашивается, а выполняет его motorTask
    // на ядре 1. Это ключевое решение: setup() теперь возвращается быстро,
    // поэтому диагностический loop() и сетевой стек поднимаются сразу и остаются
    // активными ВО ВРЕМЯ homing - [DIAG], WiFi, кнопка и удалённые команды
    // работают, пока стенд ещё выполняет homing (или застрял у стенки), вместо
    // того чтобы быть мёртвыми до завершения homing. motorTask владеет
    // энкодером/мотором, поэтому запуск homing там также исключает межъядерную
    // гонку по I2C.
    setState(SystemState::HOMING);
    gRehomePolicy = HomeDirPolicy::BY_LOAD;
    gBootHomeRequested = true;
    Serial.println("[BOOT] Homing requested (runs in motorTask; logs/WiFi stay live)");
  }

  // Загрузка сохранённой конфигурации цели/диапазона И смещения нуля HX711 из
  // NVS с откатом к умолчаниям Config.h (и первичным заполнением NVS при первой
  // загрузке). Выполняется до старта задач, чтобы самый первый SEEK использовал
  // заданное окно, А первое чтение задачи нагрузки использовало сохранённый
  // (перетарируемый) ноль, а не устаревшее умолчание.
  {
    ConfigTriple defaults{Config::TARGET_LOAD_CENTER_KG, Config::TARGET_LOAD_MIN_KG,
                          Config::TARGET_LOAD_MAX_KG, Config::HX711_OFFSET};
    ConfigTriple loaded;
    const bool fullyPersisted = ConfigStore::load(loaded, defaults);
    setConfig(loaded.targetKg, loaded.minKg, loaded.maxKg);
    // Runtime-смещение заполняется тем, что разрешил load(): сохранённый blob
    // НОВОГО формата даёт перетарированный ноль; устаревший/отсутствующий blob
    // оставляет defaults.offset (значение первой загрузки из Config.h). loadTask
    // передаёт это в тензодатчик.
    setOffset(loaded.offset);
    if (!fullyPersisted) {
      ConfigStore::save(loaded);  // заполнить NVS, чтобы значения были стабильны далее
    }
    Serial.printf("[CFG] active target=%.3f min=%.3f max=%.3f offset=%.0f (%s)\n",
                  static_cast<double>(loaded.targetKg), static_cast<double>(loaded.minKg),
                  static_cast<double>(loaded.maxKg), static_cast<double>(loaded.offset),
                  fullyPersisted ? "from NVS" : "defaults");
  }

  // Запуск loadTask (ядро 0) и motorTask (ядро 1). motorTask выполняет boot
  // homing, запрошенный выше, на своих первых итерациях; loop() на ядре 0 сразу
  // начинает выдавать [DIAG] и прокачивать сеть.
  xTaskCreatePinnedToCore(loadTask, "LoadTask", 4096, nullptr, 1, &gLoadTaskHandle, Config::LOAD_TASK_CORE);
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 6144, nullptr, 2, &gMotorTaskHandle, Config::MOTOR_TASK_CORE);

  // Сетевой менеджер: запускает WiFi (неблокирующе), HTTP-сервер и точку WS.
  // Больше не заблокирован вызовом homing, поэтому поднимается сразу.
  NetworkManager::begin();
  Serial.println("[BOOT] NetworkManager started");
}

void loop() {
  if (gButton.update()) {
    signalButtonEvent();
    Serial.println("[BUTTON] Press detected");
  }

  if (getButtonEvent()) {
    const SystemState state = getState();
    if (state == SystemState::IDLE) {
      gHomeReferenceSteps = gMotor.getExpectedSteps();
      setState(SystemState::SEEK_LOAD);
      Serial.println("[STATE] SEEK_LOAD");
    } else if (state == SystemState::SEEK_LOAD || state == SystemState::HOLD_LOAD) {
      setState(SystemState::RETURN_HOME);
      Serial.println("[STATE] RETURN_HOME");
    } else if (state == SystemState::CALIBRATE) {
      // Кнопка на стенде во время калибровки = «стоп»: прекратить вращение / поворот.
      // Ноль сохраняется только с панели (после входа по паролю).
      gCalibHaltRequested = true;
      Serial.println("[CALIB] halt requested (button)");
    } else if (state == SystemState::ERROR) {
      // Восстановление из ERROR кнопкой вместо необходимости физического RST.
      // Сам повторный homing выполняется в motorTask (он владеет
      // энкодером/мотором), поэтому здесь он лишь запрашивается. Направление на
      // основе угла и геометрии дуги (BY_LOAD).
      gRehomePolicy = HomeDirPolicy::BY_LOAD;
      gRehomeRequested = true;
      Serial.println("[STATE] ERROR -> re-home requested");
    }
  }

  static uint32_t lastReportMs = 0;
  if ((millis() - lastReportMs) >= Config::SERIAL_REPORT_MS) {
    lastReportMs = millis();

    portENTER_CRITICAL(&gShared.mux);
    const SystemState state = gShared.state;
    const float absDeg = gShared.encoderAbsoluteDeg;
    const float relDeg = gShared.encoderRelativeDeg;
    const float expDeg = gShared.expectedPositionDeg;
    const float loadKg = gShared.loadKg;
    const long loadRaw = gShared.loadRawInstant;
    const long loadRawFiltered = gShared.loadRawFiltered;
    const bool encFault = gShared.encoderFault;
    const float encErr = gShared.encoderErrorDeg;
    portEXIT_CRITICAL(&gShared.mux);

    const bool netUp = NetworkManager::isWifiConnected();
    const String netIp = NetworkManager::wifiIp();
    const uint8_t wsClients = NetworkManager::wsClientCount();

    Serial.printf(
        "[DIAG] state=%s abs=%.2f rel=%.2f exp=%.2f load=%.3fkg raw=%ld raw_f=%ld enc_err=%.2f fault=%d net=%s ip=%s ws=%u\n",
        toString(state), absDeg, relDeg, expDeg, loadKg, loadRaw, loadRawFiltered, encErr, encFault ? 1 : 0,
        netUp ? "UP" : "DOWN", netIp.c_str(), static_cast<unsigned>(wsClients));
  }

  // Прокачка очереди сетевых команд и потоковая передача телеметрии клиентам WS.
  // Команды диспетчеризуются через NetworkManager::onCommand()
  // (переопределён ниже).
  NetworkManager::loop();

  delay(5);
}

// Не-weak переопределение NetworkManager::onCommand. Weak-заглушка в
// NetworkManager.cpp лишь логирует; эта же реально управляет конечным автоматом
// тремя явными командами:
//   * SEEK  - идти к целевой нагрузке. Только из IDLE (в этом прогоне).
//             Повторяет 1-е нажатие кнопки.
//   * HOME  - вернуться домой / восстановиться. Из SEEK_LOAD/HOLD_LOAD ->
//             RETURN_HOME; из ERROR или IDLE -> повторный homing
//             (gRehomeRequested; motorTask выполняет runHoming). HOME из IDLE -
//             идемпотентный повторный homing: runHoming завершается за ноль
//             шагов, если вал уже в пределах ENCODER_HOME_WINDOW_DEG от домашней
//             позиции, и доходит до неё, если он сместился.
//   * STOP  - жёсткая заморозка на месте (диагностика), без возврата домой. Из
//             SEEK_LOAD -> HOLD_LOAD (parkIntoWindow прерывается при смене
//             состояния; motorTask замораживает мотор). В прочих случаях
//             игнорируется.
// Команды из несовместимого состояния логируются и игнорируются - оператор
// получает запись, но ничто не движется. 200 от REST означает ПОСТАВЛЕНО В
// ОЧЕРЕДЬ, а не выполнено; поштучное подтверждение (ack) на команду - отдельная
// возможность.
void NetworkManager::onCommand(NetCommand::Command cmd, int32_t arg) {
  Serial.printf("[NET] cmd=%s arg=%ld\n", NetCommand::commandName(cmd), static_cast<long>(arg));

  const SystemState state = getState();

  // WS ack: каждая команда порождает ровно одно событие "ack", чтобы клиент
  // WebSocket узнал приём/отклонение и причину, не опрашивая телеметрию. Реальное
  // решение знает только onCommand (путь постановки в очередь - нет), поэтому ack
  // отправляется здесь. detail = "<cmd>:ok" при приёме, "<cmd>:ignored_<STATE>"
  // при отклонении - суффикс состояния сообщает оператору причину. Повторно
  // используется buildEventJson (event,detail), без новой обвязки. Примечание:
  // это рассылается ВСЕМ клиентам ws (textAll), а не только отправителю - у
  // AsyncWebSocket нет дешёвого поклиентского ответа из пути опустошения очереди;
  // приемлемо для дашборда примерно с одной вкладкой. Вызывающие по REST уже
  // получают поштучный 200/400/503 на запрос и в этом не нуждаются.
  const char* cmdName = NetCommand::commandName(cmd);
  auto ackAccepted = [&]() {
    char detail[48];
    snprintf(detail, sizeof(detail), "%s:ok", cmdName);
    NetworkManager::queueEvent(buildEventJson("ack", detail).c_str());
  };
  auto ackIgnored = [&]() {
    char detail[48];
    snprintf(detail, sizeof(detail), "%s:ignored_%s", cmdName, toString(state));
    NetworkManager::queueEvent(buildEventJson("ack", detail).c_str());
  };

  switch (cmd) {
    case NetCommand::Command::SEEK:
      if (state == SystemState::IDLE) {
        gHomeReferenceSteps = gMotor.getExpectedSteps();
        setState(SystemState::SEEK_LOAD);
        Serial.println("[STATE] SEEK_LOAD (net)");
        ackAccepted();
      } else if (state == SystemState::HOLD_LOAD) {
        // Повторный поиск «на лету» к последним применённым цели/диапазону.
        // Ветка SEEK_LOAD в motorTask снова входит в parkIntoWindow, который
        // измеряет ТЕКУЩЕЕ (уже нагруженное) положение и шагает в любую сторону к
        // новой цели (dir = settledKg < cfg.targetKg ? +1 : -1), поэтому это и
        // добавляет, и сбрасывает нагрузку. Поитерационный getConfigSnapshot
        // автоматически подхватывает цель, изменённую через POST /v1/config.
        //
        // Намеренно НЕ перезахватывается gHomeReferenceSteps: это логическое
        // начало домашней позиции (истинный ноль), которое проверка-ограничитель
        // хода RETURN_HOME вычитает в moveTowardHome (см. проверку
        // HOME_MAX_TRAVEL_STEPS). Перезахват здесь сместил бы начало на
        // нагруженную позицию и обошёл бы эту защиту при последующем возврате.
        setState(SystemState::SEEK_LOAD);
        Serial.printf("[STATE] SEEK_LOAD (net, re-seek from HOLD, drift=%ld)\n",
                      static_cast<long>(gMotor.getExpectedSteps() - gHomeReferenceSteps));
        ackAccepted();
      } else {
        Serial.printf("[NET] SEEK ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::HOME:
      if (state == SystemState::SEEK_LOAD || state == SystemState::HOLD_LOAD) {
        setState(SystemState::RETURN_HOME);
        Serial.println("[STATE] RETURN_HOME (net)");
        ackAccepted();
      } else if (state == SystemState::ERROR || state == SystemState::IDLE) {
        gRehomePolicy = HomeDirPolicy::BY_LOAD;
        gRehomeRequested = true;
        Serial.println("[STATE] re-home requested (net)");
        ackAccepted();
      } else if (state == SystemState::CALIBRATE) {
        // Отмена калибровки: ноль не меняется, вал возвращается к прежнему нулю.
        gCalibCancelRequested = true;
        Serial.println("[CALIB] cancel requested (net, home)");
        ackAccepted();
      } else {
        Serial.printf("[NET] HOME ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::STOP:
      if (state == SystemState::SEEK_LOAD) {
        setState(SystemState::HOLD_LOAD);
        Serial.println("[STATE] HOLD_LOAD (net, stop)");
        ackAccepted();
      } else if (state == SystemState::CALIBRATE) {
        // «Стоп» в калибровке - только остановка движения (доступна без входа: это
        // безопасность). Ноль сохраняется отдельной командой CALIB_SAVE.
        gCalibHaltRequested = true;
        Serial.println("[CALIB] halt requested (net)");
        ackAccepted();
      } else {
        Serial.printf("[NET] STOP ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::TARE: {
      // Операторское перетарирование по пути WS (кнопка дашборда). Приём/отказ
      // решается ПРОВЕРКОЙ-ОГРАНИЧИТЕЛЕМ в tryRetare, а не одним состоянием,
      // поэтому обобщённый ackIgnored() (сообщающий состояние) НЕ используется -
      // detail ack вместо этого несёт конкретный токен проверки
      // ("tare:ignored_<reason>"). СТАРОЕ смещение захватывается ДО tryRetare,
      // чтобы событие 'tare' при успехе могло сообщить прежний ноль. tryRetare
      // сам сохраняет в NVS.
      const long prevOffset = static_cast<long>(getOffset());
      String reason;
      long newOffset = 0;
      if (tryRetare(&reason, &newOffset)) {
        NetworkManager::queueEvent(buildEventJson("ack", "tare:ok").c_str());
        char detail[48];
        snprintf(detail, sizeof(detail), "off=%ld prev=%ld", newOffset, prevOffset);
        NetworkManager::queueEvent(buildEventJson("tare", detail).c_str());
        Serial.printf("[STATE] TARE ok (net): off=%ld prev=%ld\n", newOffset, prevOffset);
      } else {
        char detail[48];
        snprintf(detail, sizeof(detail), "tare:ignored_%s", reason.c_str());
        NetworkManager::queueEvent(buildEventJson("ack", detail).c_str());
        Serial.printf("[NET] TARE ignored: %s\n", reason.c_str());
      }
      break;
    }

    // ── Калибровка нуля кулачков. Эти команды приходят ТОЛЬКО через POST /v1/calib
    // после проверки токена входа (NetworkManager.cpp); здесь - проверка состояния.
    case NetCommand::Command::CALIB_ENTER:
      // Вход в режим: только из покоя (IDLE) или из аварии (ERROR - на случай, когда
      // с новыми кулачками не проходит сам homing). Вал стоит до команды движения.
      if (state == SystemState::IDLE || state == SystemState::ERROR) {
        gCalibRunDir = 0;
        clearCalibJog();
        gCalibFinishRequested = false;
        gCalibCancelRequested = false;
        gCalibHaltRequested = false;
        gRehomeRequested = false;
        gCalibActivityMs = millis();
        setState(SystemState::CALIBRATE);
        Serial.println("[STATE] CALIBRATE (net)");
        ackAccepted();
      } else {
        Serial.printf("[NET] CALIB_ENTER ignored in state %s\n", toString(state));
        ackIgnored();
      }
      break;

    case NetCommand::Command::CALIB_RUN:
      if (state == SystemState::CALIBRATE && (arg == 1 || arg == -1)) {
        clearCalibJog();
        gCalibHaltRequested = false;
        gCalibStartSteps = gMotor.getExpectedSteps();
        gCalibStartLoadKg = getLoadKg();
        gCalibActivityMs = millis();
        gCalibRunDir = static_cast<int8_t>(arg);
        Serial.printf("[CALIB] run %c\n", arg > 0 ? '+' : '-');
        ackAccepted();
      } else {
        ackIgnored();
      }
      break;

    case NetCommand::Command::CALIB_JOG:
      // Поворот на заданный угол. Во время непрерывного вращения не принимается -
      // панель сначала шлёт «стоп».
      if (state == SystemState::CALIBRATE && gCalibRunDir == 0 && arg != 0) {
        if (getCalibJogPending() == 0) {
          gCalibStartLoadKg = getLoadKg();
        }
        gCalibHaltRequested = false;
        gCalibActivityMs = millis();
        addCalibJog(arg);
        Serial.printf("[CALIB] jog %ld steps\n", static_cast<long>(arg));
        ackAccepted();
      } else {
        ackIgnored();
      }
      break;

    case NetCommand::Command::CALIB_HALT:
      if (state == SystemState::CALIBRATE) {
        gCalibHaltRequested = true;
        ackAccepted();
      } else {
        ackIgnored();
      }
      break;

    case NetCommand::Command::CALIB_SAVE:
      if (state == SystemState::CALIBRATE) {
        gCalibFinishRequested = true;
        Serial.println("[CALIB] save requested (net)");
        ackAccepted();
      } else {
        ackIgnored();
      }
      break;

    case NetCommand::Command::CALIB_CANCEL:
      if (state == SystemState::CALIBRATE) {
        gCalibCancelRequested = true;
        Serial.println("[CALIB] cancel requested (net)");
        ackAccepted();
      } else {
        ackIgnored();
      }
      break;

    case NetCommand::Command::NONE:
    default:
      break;
  }
}
