#include "NetworkManager.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

#include "Config.h"
#include "ConfigStore.h"
#include "NetCommand.h"
#include "Telemetry.h"

// Определено в .ino (владеет gConfig). Атомарно записывает оперативную конфигурацию.
extern void setConfig(float targetKg, float minKg, float maxKg);
// Определено в .ino (владеет gOffset). Читает текущее смещение нуля HX711 - нужно
// здесь, чтобы save() из /v1/config (единый blob) не затёр перетарированное
// смещение устаревшим значением по умолчанию. Само ПЕРЕТАРИРОВАНИЕ отсюда НЕ
// вызывается: /v1/tare ставит команду TARE в очередь, и она выполняется на задаче
// loop через onCommand() (см. обработчик о том, почему она не выполняется
// напрямую на задаче AsyncTCP).
extern float getOffset();

namespace {

AsyncWebServer sServer(Config::NET_HTTP_PORT);
AsyncWebSocket sWs(Config::NET_WS_PATH);

QueueHandle_t sCmdQueue = nullptr;
uint32_t sLastTelemetryMs = 0;
bool sMdnsStarted = false;

// Если связь STA остаётся разорванной дольше этого времени, сторож в loop()
// заново запускает WiFi.begin(). Также это минимальный интервал между попытками
// переподключения. Держится локально (не в Config.h), чтобы не трогать общую
// конфигурацию.
constexpr uint32_t kWifiReconnectTimeoutMs = 10000;

// Очередь широковещательной рассылки событий по возможности. queueEvent() (любая
// задача) копирует строку JSON в один из этих фиксированных слотов; loop() (сетевая
// задача) вычерпывает их и выполняет собственно sWs.textAll(), снимая с
// вызывающего (например, motorTask на ядре 1) стоимость мьютекса + кучи у textAll.
struct EventMsg {
  char json[160];
};
QueueHandle_t sEventQueue = nullptr;

// Применяет необязательный статический IP из Config перед WiFi.begin(). Возвращает
// false (и пишет в лог), если какая-либо адресная строка некорректна или
// WiFi.config() отвергнут, чтобы вызывающий откатился на DHCP, а не остался тихо
// без адреса.
bool applyStaticIpIfEnabled() {
  if (!Config::NET_USE_STATIC_IP) {
    return false;
  }
  IPAddress ip, gw, mask, dns;
  if (!ip.fromString(Config::NET_STATIC_IP) ||
      !gw.fromString(Config::NET_STATIC_GATEWAY) ||
      !mask.fromString(Config::NET_STATIC_SUBNET) ||
      !dns.fromString(Config::NET_STATIC_DNS)) {
    Serial.println("[NET][WARN] static IP config malformed, falling back to DHCP");
    return false;
  }
  if (!WiFi.config(ip, gw, mask, dns)) {
    Serial.println("[NET][WARN] WiFi.config() rejected static IP, falling back to DHCP");
    return false;
  }
  Serial.printf("[NET] static IP configured: %s (gw %s)\n",
                Config::NET_STATIC_IP, Config::NET_STATIC_GATEWAY);
  return true;
}

// Обработчик событий WiFi. Переподключение по событию - основной путь
// восстановления: SDK выдаёт STA_DISCONNECTED в момент разрыва связи, поэтому
// переассоциация происходит сразу, вместо ожидания до kWifiReconnectTimeoutMs, пока
// заметит сторож в loop(). STA_GOT_IP сбрасывает флаг mDNS, чтобы ответчик заново
// анонсировался на свежей связи (см. перезапуск mDNS в loop()). Выполняется на
// задаче WiFi/событий; держать неблокирующим (только лог + переподключение).
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[NET] WiFi got IP %s\n", WiFi.localIP().toString().c_str());
      sMdnsStarted = false;  // заново анонсировать mDNS на новой связи
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[NET] WiFi disconnected (reason %u), reconnecting\n",
                    info.wifi_sta_disconnected.reason);
      WiFi.reconnect();
      break;
    default:
      break;
  }
}

// Неблокирующая постановка разобранной команды в общую очередь. Возвращает false
// для NONE (нераспознано) или если очередь полна/отсутствует. И обработчик WS, и
// эндпойнт REST /v1/command проходят через это, чтобы поведение постановки в
// очередь оставалось одинаковым для всех транспортов.
bool enqueueCommand(NetCommand::Command cmd) {
  if (cmd == NetCommand::Command::NONE) {
    return false;
  }
  if (!sCmdQueue) {
    return false;
  }
  const NetCommand::Command qCmd = cmd;
  return xQueueSend(sCmdQueue, &qCmd, 0) == pdTRUE;
}

// Отправляет тело JSON с заголовком Access-Control-Allow-Origin:*, чтобы дашборд,
// отдаваемый с другого источника (ноутбук оператора или другой хост), мог
// обращаться к /v1/* через fetch().
void sendJsonCors(AsyncWebServerRequest* request, int code, const String& body) {
  AsyncWebServerResponse* r = request->beginResponse(code, "application/json", body);
  r->addHeader("Access-Control-Allow-Origin", "*");
  request->send(r);
}

// WS onEvent направляет входящие текстовые кадры сюда. Разбор идёт через
// NetCommand::parseCommand, а результат ставится в sCmdQueue. Результаты NONE
// (нераспознанный текст) отбрасываются молча - слоты очереди занимают только
// корректные команды, поэтому клиент с некорректным поведением не может забить
// очередь мусором.
void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                   AwsEventType type, void* arg, uint8_t* data, size_t len) {
  (void)server;
  (void)arg;

  switch (type) {
    case WS_EVT_CONNECT: {
      // Ограничение числа одновременных клиентов, чтобы память WS оставалась
      // ограниченной. Сброс новейшего клиента груб, но прост; ожидается, что
      // дашборд - это одна вкладка браузера.
      if (sWs.count() > Config::NET_MAX_WS_CLIENTS) {
        client->close();
      }
      break;
    }
    case WS_EVT_DISCONNECT:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;

    case WS_EVT_DATA: {
      // Рассматриваются только текстовые кадры. Двоичные кадры и частичные
      // фрагменты игнорируются - протокол дашборда - это построчные текстовые
      // команды.
      const AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
      if (!info || !info->final || info->opcode != WS_TEXT) {
        break;
      }
      // Завершающий нуль, чтобы parseCommand мог использовать помощники в стиле
      // strcmp. Кадры крошечные (start/stop/park), поэтому буфера на стеке
      // достаточно.
      char buf[64];
      const size_t copyLen = (len < sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
      memcpy(buf, data, copyLen);
      buf[copyLen] = '\0';

      // Обрезка хвостовых пробельных символов CR/LF, чтобы "start\n" из терминала
      // тоже работал.
      size_t end = copyLen;
      while (end > 0 && (buf[end - 1] == '\r' || buf[end - 1] == '\n' ||
                         buf[end - 1] == ' ')) {
        buf[--end] = '\0';
      }

      // Неблокирующая постановка в очередь через общий помощник; если очередь
      // полна, команда отбрасывается. Колбэк события WS никогда не блокируется (он
      // выполняется на задаче AsyncTCP). NONE (мусорный текст) тоже возвращает
      // false, но это ожидаемо - предупреждать только когда действительно
      // разобранную команду пришлось отбросить из-за полной очереди.
      const NetCommand::Command parsed = NetCommand::parseCommand(buf);
      if (parsed != NetCommand::Command::NONE && !enqueueCommand(parsed)) {
        Serial.println("[NET] cmd queue full, dropped");
      }
      break;
    }
  }
}

}  // namespace

void NetworkManager::begin() {
  // Создать очередь команд раньше всего, что могло бы в неё что-то отправить.
  sCmdQueue = xQueueCreate(Config::NET_CMD_QUEUE_SIZE, sizeof(NetCommand::Command));

  // Очередь широковещательной рассылки событий по возможности, вычерпывается в loop().
  sEventQueue = xQueueCreate(8, sizeof(EventMsg));

  // Режим станции, persistent выключен (сохранять учётные данные-заглушки в SDK
  // не нужно), автопереподключение включено, чтобы SDK переассоциировался после
  // кратковременного пропадания точки доступа, и стабильное имя хоста, чтобы
  // дашборд мог достучаться через http://esp32-s3.local, как только поднимется
  // WiFi (mDNS запускается лениво в loop()).
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  // Зарегистрировать обработчик событий до begin(), чтобы не упустить ранний
  // disconnect/got-IP. Переподключение по событию - основной путь восстановления
  // (опрашивающий сторож в loop() - более медленная подстраховка).
  WiFi.onEvent(onWifiEvent);

  // Напечатать MAC STA один раз. Он стабилен (заводской eFuse, не рандомизирован),
  // поэтому резервирование фиксированного IP для этого MAC в настройках DHCP точки
  // доступа/роутера даёт предсказуемый адрес в любой сети - предпочтительная
  // альтернатива жёстко заданному статическому IP. setHostname() должен
  // предшествовать begin(), чтобы вступить в силу.
  WiFi.setHostname(Config::NET_HOSTNAME);
  Serial.printf("[NET] STA MAC %s (reserve a DHCP lease for this MAC)\n",
                WiFi.macAddress().c_str());

  // Необязательный статический IP (по умолчанию выключен). Применяется до begin();
  // при любой проблеме пишет в лог и возвращает false, оставляя в силе DHCP.
  applyStaticIpIfEnabled();

  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

  // Не блокироваться на подключении здесь - задачи нагрузки/двигателя работают на
  // другом ядре, и стенд должен подняться даже если WiFi медленный или
  // отсутствует. loop() просто не будет рассылать телеметрию, пока WiFi не поднят.
  sWs.onEvent(handleWsEvent);
  sServer.addHandler(&sWs);

  // Минимальный корень-заглушка. Простого текстового ответа-идентификатора
  // достаточно, чтобы подтвердить, что сервер жив, и дать curl цель, отвечающую
  // 200.
  sServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "Load Stand API");
  });

  // ── REST API (v1) ──────────────────────────────────────────────────────────
  // Обнаруживается по http://esp32-s3.local/v1/..., как только поднят mDNS. Все
  // ответы несут Access-Control-Allow-Origin:*, чтобы к ним мог обращаться дашборд
  // вне устройства.

  // GET /v1/health - дешёвая проба живости.
  sServer.on("/v1/health", HTTP_GET, [](AsyncWebServerRequest* request) {
    String body = "{\"status\":\"ok\",\"uptime_ms\":";
    body += String(millis());
    body += "}";
    sendJsonCors(request, 200, body);
  });

  // GET /v1/status - полный снимок телеметрии (тот же JSON, что шлёт поток WS).
  sServer.on("/v1/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    sendJsonCors(request, 200, buildTelemetryJson());
  });

  // POST /v1/command - принимает команду только из параметра "cmd" (query
  // ?cmd=start или тело application/x-www-form-urlencoded cmd=start). Обработчика
  // сырого тела намеренно НЕТ: ESPAsyncWebServer освобождает request->_tempObject
  // через C free(), поэтому спрятанная там String утекала бы своим буфером кучи на
  // каждом запросе (запускается удалённо). Корректные команды ставятся в ту же
  // очередь, что использует обработчик WS; неизвестные команды возвращают 400,
  // полная очередь - 503.
  sServer.on(
      "/v1/command", HTTP_POST, [](AsyncWebServerRequest* request) {
        // getParam("cmd") покрывает строку запроса; getParam("cmd", true)
        // покрывает тело POST в form-urlencoded, оба разбираются в параметры
        // AsyncWebServer.
        String value;
        if (request->hasParam("cmd")) {
          value = request->getParam("cmd")->value();
        } else if (request->hasParam("cmd", true)) {
          value = request->getParam("cmd", true)->value();
        } else {
          sendJsonCors(request, 400, "{\"error\":\"missing cmd\"}");
          return;
        }
        value.trim();

        const NetCommand::Command cmd = NetCommand::parseCommand(value.c_str());
        if (cmd == NetCommand::Command::NONE) {
          sendJsonCors(request, 400, "{\"error\":\"unknown command\"}");
          return;
        }
        // Сообщать об успехе только когда команда действительно поставлена в
        // очередь; полная очередь не должна выдавать себя за принятую команду
        // (ложный 200).
        if (!enqueueCommand(cmd)) {
          sendJsonCors(request, 503, "{\"error\":\"queue full\"}");
          return;
        }
        String body = "{\"accepted\":\"";
        body += NetCommand::commandName(cmd);
        body += "\"}";
        sendJsonCors(request, 200, body);
      });

  // Предварительный (preflight) запрос для POST выше, чтобы браузер разрешил
  // кросс-доменный fetch() с пользовательским Content-Type.
  sServer.on("/v1/command", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* r = request->beginResponse(204);
    r->addHeader("Access-Control-Allow-Origin", "*");
    r->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    r->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(r);
  });

  // POST /v1/config - задаёт целевую/мин/макс нагрузку (кг) во время работы.
  // Параметры ТОЛЬКО через query (?target=..&min=..&max=..) или form-urlencoded;
  // обработчика сырого тела НЕТ (во избежание той же утечки кучи). Требуются все
  // три. Проверяется на стороне прошивки (относительность + ноль; жёсткого потолка
  // нет). При успехе: атомарно применяется к gConfig + сохраняется в NVS.
  sServer.on("/v1/config", HTTP_POST, [](AsyncWebServerRequest* request) {
    // Взять параметр из query ИЛИ тела формы. Возвращает параметр (или nullptr,
    // если отсутствует), чтобы вызывающий мог отличить «отсутствует» от «плохое
    // значение».
    auto getParam = [request](const char* name) -> const AsyncWebParameter* {
      if (request->hasParam(name)) return request->getParam(name);
      if (request->hasParam(name, true)) return request->getParam(name, true);
      return nullptr;
    };
    // Разобрать значение присутствующего параметра как float, отвергая пустые
    // токены / токены с мусором на конце (strtof возвращает 0 для "abc"; проверка
    // конечного указателя ловит "2.0x").
    auto parseF = [](const AsyncWebParameter* p, float* out) -> bool {
      const String& v = p->value();
      char* endp = nullptr;
      const float f = strtof(v.c_str(), &endp);
      if (endp == v.c_str() || *endp != '\0') return false;
      *out = f;
      return true;
    };

    // Сначала наличие: любой отсутствующий параметр -> "missing param". Затем
    // разбор значений: любой присутствующий, но неразбираемый параметр ->
    // "bad param". Один проход, без повторных проверок.
    const AsyncWebParameter* pt = getParam("target");
    const AsyncWebParameter* pmn = getParam("min");
    const AsyncWebParameter* pmx = getParam("max");
    if (!pt || !pmn || !pmx) {
      sendJsonCors(request, 400, "{\"error\":\"missing param\"}");
      return;
    }
    float target, mn, mx;
    if (!parseF(pt, &target) || !parseF(pmn, &mn) || !parseF(pmx, &mx)) {
      sendJsonCors(request, 400, "{\"error\":\"bad param\"}");
      return;
    }

    const char* reason = validateConfig(target, mn, mx);
    if (reason != nullptr) {
      String body = "{\"error\":\"invalid\",\"detail\":\"";
      body += reason;
      body += "\"}";
      sendJsonCors(request, 400, body);
      return;
    }

    setConfig(target, mn, mx);
    // Сохранить новую тройку кг, но перенести ТЕКУЩЕЕ действующее смещение в blob:
    // ConfigStore использует единый blob, поэтому сохранение, оставившее смещение
    // на значении по умолчанию, затёрло бы перетарированный оператором ноль.
    // Захватить getOffset() и приложить его.
    ConfigTriple applied{target, mn, mx};
    applied.offset = getOffset();
    ConfigStore::save(applied);

    char body[96];
    snprintf(body, sizeof(body),
             "{\"applied\":{\"target\":%.3f,\"min\":%.3f,\"max\":%.3f}}",
             static_cast<double>(target), static_cast<double>(mn),
             static_cast<double>(mx));
    sendJsonCors(request, 200, body);
  });

  sServer.on("/v1/config", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* r = request->beginResponse(204);
    r->addHeader("Access-Control-Allow-Origin", "*");
    r->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    r->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(r);
  });

  // POST /v1/tare - операторское ПЕРЕТАРИРОВАНИЕ нуля HX711. Параметров НЕТ: новое
  // смещение ИЗМЕРЯЕТСЯ с тензодатчика в покое, а не передаётся.
  //
  // Это СТАВИТ В ОЧЕРЕДЬ команду TARE (в точности как /v1/command), а не вызывает
  // tryRetare() напрямую. Две причины:
  //   1. Цикл усреднения в покое у tryRetare() блокируется на ~700 мс типично / до
  //      SETTLED_MAX_WAIT_MS в худшем случае. Выполнение этого на единственной
  //      задаче AsyncTCP застопорило бы ВСЁ обслуживание HTTP/WS на всё это окно -
  //      нарушение контракта короткого встроенного save() примерно в 30 раз и
  //      реальный удар по доступности для многоклиентского дашборда.
  //   2. Защита (IDLE + в домашней позиции + нагрузка около нуля) и фиксация
  //      смещения должны быть атомарны относительно машины состояний. При
  //      выполнении на AsyncTCP команда SEEK от кнопки/WS на задаче loop могла бы
  //      перейти в SEEK_LOAD и нагрузить датчик ВО ВРЕМЯ ~700 мс усреднения,
  //      отравив или сохранив плохой ноль (TOCTOU). Постановка в очередь
  //      сериализует перетарирование относительно button-SEEK, WS-SEEK и WS-TARE,
  //      которые ВСЕ уже направляются на задачу loop через onCommand().
  // Путь очереди уже несёт TARE (WS тоже его использует); ветка TARE в onCommand()
  // выполняет tryRetare() на задаче loop и сообщает результат защиты асинхронно
  // через события WS "ack"/"tare" (detail = offset/prev или причина отказа).
  // Поэтому здесь возвращается 202 Accepted (ПОСТАВЛЕНО В ОЧЕРЕДЬ, ещё не
  // выполнено) - дашборд читает фактический результат из события WS, а не из этого
  // тела HTTP. Полная очередь -> 503.
  sServer.on("/v1/tare", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!enqueueCommand(NetCommand::Command::TARE)) {
      sendJsonCors(request, 503, "{\"error\":\"queue full\"}");
      return;
    }
    sendJsonCors(request, 202, "{\"accepted\":\"TARE\"}");
  });

  sServer.on("/v1/tare", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* r = request->beginResponse(204);
    r->addHeader("Access-Control-Allow-Origin", "*");
    r->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    r->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(r);
  });

  sServer.begin();
}

void NetworkManager::loop() {
  // Вычерпать все поставленные в очередь команды и разослать их. Ограниченный
  // цикл, чтобы поток команд не заморил остальную часть loop() - не более одной
  // глубины очереди за вызов.
  if (sCmdQueue) {
    NetCommand::Command cmd;
    UBaseType_t remaining = uxQueueMessagesWaiting(sCmdQueue);
    while (remaining-- > 0) {
      if (xQueueReceive(sCmdQueue, &cmd, 0) == pdTRUE) {
        onCommand(cmd);
      }
    }
  }

  // Сторож переподключения WiFi - теперь МЕДЛЕННАЯ ПОДСТРАХОВКА за onWifiEvent().
  // Обработчик события переассоциируется сразу по STA_DISCONNECTED, а
  // setAutoReconnect(true) покрывает большинство разрывов; этот опрашивающий путь
  // срабатывает только если ассоциация всё ещё отсутствует после
  // kWifiReconnectTimeoutMs (зависший supplicant, так и не выдавший пригодного
  // события разрыва). sLastWifiOkMs отмечает последний момент, когда связь была
  // или была попытка повтора, ограничивая повторы одним разом в интервал.
  static uint32_t sLastWifiOkMs = 0;
  static bool sWasConnected = false;
  const uint32_t nowMs = millis();

  if (WiFi.status() == WL_CONNECTED) {
    sLastWifiOkMs = nowMs;
    // Переход свежего (пере)подключения: сбросить флаг mDNS, чтобы ответчик заново
    // анонсировался на новой связи (старый экземпляр MDNS привязан к предыдущему
    // соединению).
    if (!sWasConnected) {
      sMdnsStarted = false;
      sWasConnected = true;
    }
  } else {
    sWasConnected = false;
    if (nowMs - sLastWifiOkMs >= kWifiReconnectTimeoutMs) {
      Serial.println("[NET] WiFi down, re-driving begin()");
      WiFi.disconnect();
      WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
      // Ограничение частоты: считать эту попытку как «сейчас ок», чтобы выждать
      // полный интервал перед следующим повтором.
      sLastWifiOkMs = nowMs;
    }
    // Не подключены - рассылать нечего; вычерпывание выше уже отработало.
    return;
  }

  // Запустить mDNS теперь, когда WiFi поднят. WiFi.begin() неблокирующий, поэтому
  // сделать это в begin() нельзя - ответчику нужен живой интерфейс с IP.
  // Анонсировать HTTP-службу, чтобы http://esp32-s3.local разрешалось и REST API
  // был обнаружим. Защёлкивать sMdnsStarted только при УСПЕХЕ, чтобы временный
  // сбой был повторён на следующем проходе loop (а не убил esp32-s3.local
  // навсегда).
  if (!sMdnsStarted) {
    // Сначала снести любой предыдущий ответчик. При переподключении старый
    // экземпляр MDNS привязан к предыдущему интерфейсу/IP; повторный вызов begin()
    // без end() оставляет esp32-s3.local указывающим на устаревшую связь
    // (сообщавшаяся причина того, что ".local перестаёт разрешаться после
    // пропадания точки доступа"). end() при первом проходе - пустая операция.
    MDNS.end();
    if (MDNS.begin(Config::NET_HOSTNAME)) {
      MDNS.addService("http", "tcp", Config::NET_HTTP_PORT);
      Serial.printf("[NetManager] mDNS responder started: http://%s.local\n",
                    Config::NET_HOSTNAME);
      sMdnsStarted = true;
    } else {
      Serial.println("[NetManager] mDNS start failed, will retry");
    }
  }

  // Вычерпать поставленные в очередь события на каждый клиент WS. Делается здесь
  // (сетевая задача), чтобы задача двигателя/управления никогда не платила
  // стоимость мьютекса + кучи у textAll. Ограничено, как и вычерпывание команд.
  // Защищено проверкой числа клиентов ниже.
  if (sEventQueue && sWs.count() > 0) {
    EventMsg msg;
    UBaseType_t evRemaining = uxQueueMessagesWaiting(sEventQueue);
    while (evRemaining-- > 0) {
      if (xQueueReceive(sEventQueue, &msg, 0) == pdTRUE) {
        sWs.textAll(msg.json);
      }
    }
  }

  if (sWs.count() == 0) {
    return;
  }
  const uint32_t now = millis();
  if (now - sLastTelemetryMs < Config::NET_TELEMETRY_INTERVAL_MS) {
    return;
  }
  sLastTelemetryMs = now;

  // buildTelemetryJson() возвращает String, опирающуюся на буфер стека; для
  // асинхронной отправки она копируется в буфер кучи (отправка WS внутренне
  // ставится в очередь и может пережить локальную String). textAll рассылает
  // каждому подключённому клиенту.
  const String json = buildTelemetryJson();
  if (json.length() > 0) {
    sWs.textAll(json);
  }
}

void NetworkManager::queueEvent(const char* json) {
  // Безопасно для задач, неблокирующе. Скопировать JSON в фиксированный слот и
  // поставить в очередь; собственно sWs.textAll() происходит в loop() на сетевой
  // задаче. Никаких выделений кучи сверх копии фиксированной структуры, никакого
  // textAll, никакого мьютекса здесь - можно безопасно вызывать из motorTask
  // (ядро 1), не добавляя задержки движения.
  if (!sEventQueue || !json) {
    return;
  }
  EventMsg msg;
  strlcpy(msg.json, json, sizeof(msg.json));
  // По возможности: молча отбросить, если очередь полна.
  xQueueSend(sEventQueue, &msg, 0);
}

void NetworkManager::broadcastEvent(const String& json) {
  // Переписано как тонкая неблокирующая обёртка над queueEvent, чтобы любой
  // существующий вызывающий (возможно, на задаче двигателя) больше не платил
  // стоимость мьютекса + кучи у textAll напрямую. Кадр вычерпывается и
  // отправляется из loop().
  queueEvent(json.c_str());
}

bool NetworkManager::isWifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

uint8_t NetworkManager::wsClientCount() {
  return static_cast<uint8_t>(sWs.count());
}

String NetworkManager::wifiIp() {
  if (WiFi.status() != WL_CONNECTED) {
    return String("0.0.0.0");
  }
  return WiFi.localIP().toString();
}

// Слабая реализация по умолчанию: просто пишет команду в лог. Ожидается, что .ino
// переопределит её реальной реализацией, управляющей машиной состояний
// (START/STOP/PARK).
__attribute__((weak)) void NetworkManager::onCommand(NetCommand::Command cmd) {
  Serial.printf("[NetManager] cmd=%s (default stub)\n", NetCommand::commandName(cmd));
}
