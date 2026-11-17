#pragma once
// Copy to Secrets.hpp (git-ignored) and fill in. Used by Shared/Maintenance.hpp and flash.ps1 -Ota.
namespace Secrets {
  // 2.4 GHz network the connectors join in maintenance mode: a phone or laptop hotspot,
  // or home Wi-Fi if the car parks in range. The laptop running flash.ps1 -Ota must be on it too.
  inline constexpr const char *MAINTENANCE_WIFI_SSID = "";
  inline constexpr const char *MAINTENANCE_WIFI_PASSWORD = "";

  // ArduinoOTA upload password
  inline constexpr const char *OTA_PASSWORD = "";
}
