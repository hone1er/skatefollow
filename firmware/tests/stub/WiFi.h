#pragma once
enum wifi_mode_t { WIFI_STA };
struct WiFiStub { bool mode(wifi_mode_t); bool disconnect(); };
extern WiFiStub WiFi;
