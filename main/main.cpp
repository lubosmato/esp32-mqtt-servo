#include "essentials/config.hpp"
#include "essentials/device_info.hpp"
#include "essentials/esp32_storage.hpp"
#include "essentials/mqtt.hpp"
#include "essentials/settings_server.hpp"
#include "essentials/wifi.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.hpp"
#include "simple_logger.hpp"

#include <algorithm>
#include <chrono>

namespace es = essentials;
using namespace std::chrono_literals;

const char* TAG_APP = "app";

extern const uint8_t mqttCertBegin[] asm("_binary_cert_pem_start");
extern const uint8_t mqttCertEnd[] asm("_binary_cert_pem_end");

struct App {
  es::DeviceInfo deviceInfo{};

  es::Esp32Storage configStorage{"config"};
  es::Config config{configStorage};
  es::Config::Value<std::string> ssid = config.get<std::string>("ssid");
  es::Config::Value<std::string> wifiPass = config.get<std::string>("wifiPass");

  es::Esp32Storage mqttStorage{"mqtt"};
  es::Config mqttConfig{mqttStorage};
  es::Config::Value<std::string> mqttUrl = mqttConfig.get<std::string>("url", "mqtt://127.0.0.1:1883");
  es::Config::Value<std::string> mqttUser = mqttConfig.get<std::string>("user", "");
  es::Config::Value<std::string> mqttPass = mqttConfig.get<std::string>("pass", "");

  es::Wifi wifi{};
  std::unique_ptr<es::Mqtt> mqtt{};
  std::vector<std::unique_ptr<es::Mqtt::Subscription>> subs{};
  Servo servo{GPIO_NUM_32};

  es::SettingsServer settingsServer{80,
    "Remote Servo",
    "1.0.1",
    {
      {"WiFi SSID", ssid},
      {"WiFi Password", wifiPass},
      {"MQTT URL", mqttUrl},
      {"MQTT Username", mqttUser},
      {"MQTT Password", mqttPass},
    }};

  void run() {
    wifi.connect(*ssid, *wifiPass);
    logW(TAG_APP, "Waiting for wifi connection...");
    int tryCount = 0;
    while (!wifi.isConnected() && tryCount < 1000) {
      tryCount++;
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!wifi.isConnected()) {
      logE(TAG_APP, "Couldn't connect to the wifi. Starting WiFi AP with settings server for 60s.");
      settingsServer.start();
      wifi.startAccessPoint("esp32", "12345678", es::Wifi::Channel::Channel5);
      int tryCount = 0;
      while (tryCount < 60) {
        tryCount++;
        logW(TAG_APP, "Waiting for configuration...");
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      esp_restart();
    }

    auto mqttCert =
      std::string_view{reinterpret_cast<const char*>(mqttCertBegin), std::size_t(mqttCertEnd - mqttCertBegin)};
    std::string mqttPrefix = "esp32/" + deviceInfo.uniqueId();

    std::string url = *mqttUrl;
    std::string user = *mqttUser;
    std::string pass = *mqttPass;
    es::Mqtt::ConnectionInfo mqttInfo{url, mqttCert, user, pass};
    es::Mqtt::LastWillMessage lastWill{"last/will", "Bye", es::Mqtt::Qos::Qos0, false};

    mqtt = std::make_unique<es::Mqtt>(
      mqttInfo,
      std::string_view{mqttPrefix},
      30s,
      lastWill,
      [this]() {
        logI(TAG_APP, "MQTT is connected!");
        publishDeviceInfo();
      },
      []() { logI(TAG_APP, "MQTT is disconnected!"); },
      1024 * 30);

    // subscribe to "ping" topic and react to it by sending "pong" message back
    subs.emplace_back(mqtt->subscribe("ping", es::Mqtt::Qos::Qos0, [this](const es::Mqtt::Data& chunk) {
      logI(TAG_APP, "got ping message");
      mqtt->publish("pong", chunk.data, es::Mqtt::Qos::Qos0, false);
    }));

    subs.emplace_back(mqtt->subscribe<int>("move", es::Mqtt::Qos::Qos0, [this](std::optional<int> angle) {
      if (!angle) {
        logW(TAG_APP, "angle is not integer");
        return;
      }
      logI(TAG_APP, "moving to %d", *angle);
      servo.move(*angle);
      logI(TAG_APP, "done moving");
    }));

    subs.emplace_back(mqtt->subscribe<int>("push", es::Mqtt::Qos::Qos0, [this](std::optional<int> milliseconds) {
      if (!milliseconds) {
        logW(TAG_APP, "milliseconds is not integer");
        return;
      }

      if (*milliseconds <= 0 || *milliseconds > 10000) {
        logW(TAG_APP, "out of range (0, 10000> ms");
        return;
      }

      logI(TAG_APP, "pushing for %d", *milliseconds);
      servo.move(37);
      vTaskDelay(pdMS_TO_TICKS(*milliseconds));
      servo.move(20);
      logI(TAG_APP, "done pushing");
    }));

    servo.move(0);

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(10000));
      publishDeviceInfo();
    }
  }

  void publishDeviceInfo() {
    const auto rssi = wifi.rssi();
    if (rssi) mqtt->publish("info/rssi", *rssi, es::Mqtt::Qos::Qos0, false);

    mqtt->publish("info/freeHeap", deviceInfo.freeHeap(), es::Mqtt::Qos::Qos0, false);
    mqtt->publish("info/totalHeap", deviceInfo.totalHeap(), es::Mqtt::Qos::Qos0, false);
    mqtt->publish("info/uptime", deviceInfo.uptime(), es::Mqtt::Qos::Qos0, false);
  }
};

extern "C" void app_main() {
  try {
    App{}.run();
  } catch (const std::exception& e) {
    logE(TAG_APP, "EXCEPTION: %s", e.what());
  } catch (...) {
    logE(TAG_APP, "UNKNOWN EXCEPTION");
  }
  esp_restart();
}
