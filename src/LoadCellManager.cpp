#include "LoadCellManager.h"

#include "Config.h"

bool LoadCellManager::begin() {
  hx711_.begin(Config::PIN_HX711_DOUT, Config::PIN_HX711_SCK);
  hx711_.set_scale(Config::HX711_CALIBRATION_FACTOR);
  // НЕ зашивать Config::HX711_OFFSET в драйвер HX711: путь get_units не используется для
  // управления (кг вычисляется вручную ниже из оперативного члена offset_, который
  // loadTask держит в синхронизации с перетарируемым gOffset). Инициализация offset
  // драйвера здесь только вводила бы в заблуждение - она никогда не участвует в тех кг,
  // что реально используются.
  ready_ = true;
  sampleReady_ = false;
  filteredKg_ = 0.0f;
  rawValue_ = 0;
  medianCount_ = 0;
  medianIndex_ = 0;
  filteredRaw_ = 0;
  filteredRawInit_ = false;
  return true;
}

void LoadCellManager::setOffset(long offset) {
  // Простое присваивание. Вызывается из loadTask (ядро 0) каждый цикл перед update(), так
  // что offset, используемый формулой кг, отслеживает оперативный gOffset (обновляемый
  // операторским перетарированием). Блокировка здесь не нужна: член трогается только в
  // задаче нагрузки; межъядерная передача происходит на стороне gOffset.
  offset_ = offset;
}

void LoadCellManager::update() {
  if (!ready_ || !hx711_.is_ready()) {
    return;
  }

  // Одно чтение за цикл; кг выводится из отфильтрованного сырого значения ниже (без
  // второго чтения HX711 через get_units).
  const long raw = hx711_.read_average(Config::HX711_AVERAGE_SAMPLES);
  rawValue_ = raw;

  // Пропуск заведомо некорректных чтений (например, raw==0, наблюдавшийся в логах), чтобы
  // они не отравляли медианное окно.
  if (raw <= 0) {
    return;
  }

  // Скользящая медиана убирает одиночные выбросы шума драйвера без добавления длинного
  // хвоста затухания после снятия нагрузки.
  medianBuf_[medianIndex_] = raw;
  medianIndex_ = static_cast<uint8_t>((medianIndex_ + 1) % Config::HX711_MEDIAN_WINDOW);
  if (medianCount_ < Config::HX711_MEDIAN_WINDOW) {
    ++medianCount_;
  }

  const long med = computeMedian();
  filteredRaw_ = med;
  filteredRawInit_ = true;

  const float kg = (static_cast<float>(filteredRaw_) - static_cast<float>(offset_)) / Config::HX711_CALIBRATION_FACTOR;
  if (kg < Config::HX711_MIN_VALID_KG || kg > Config::HX711_MAX_VALID_KG) {
    return;
  }

  filteredKg_ += Config::HX711_FILTER_ALPHA * (kg - filteredKg_);
  // Только сейчас получен реальный отсчёт HX711 в допустимом диапазоне. Готовность
  // означает «корректный отсчёт существует» (а не просто «begin() отработала»), так что
  // классификация нагрузки при запуске никогда не доверяет значению по умолчанию 0.0 кг
  // от неисправного/медленного датчика.
  sampleReady_ = true;
}

long LoadCellManager::computeMedian() const {
  const uint8_t n = medianCount_;
  long tmp[Config::HX711_MEDIAN_WINDOW];
  for (uint8_t i = 0; i < n; ++i) {
    tmp[i] = medianBuf_[i];
  }
  // Сортировка вставками (малое n).
  for (uint8_t i = 1; i < n; ++i) {
    const long key = tmp[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

float LoadCellManager::getMassKg() const {
  return filteredKg_;
}

bool LoadCellManager::isReady() const {
  return sampleReady_;
}

long LoadCellManager::getRawValue() const {
  return rawValue_;
}

long LoadCellManager::getFilteredRaw() const {
  return filteredRawInit_ ? filteredRaw_ : rawValue_;
}
