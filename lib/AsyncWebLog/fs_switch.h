#pragma once

#ifdef FS_LITTLEFS
#pragma message("Info : USE 'LITTLEFS' Filesystem")
#include <LittleFS.h>
#define myFS LittleFS
#else
#pragma message("Info : USE 'SPIFFS' Filesystem")
#ifdef ESP32
#include <SPIFFS.h>
#endif
#define myFS SPIFFS
#endif 


