#ifndef AsyncWebLog_h
#define AsyncWebLog_h

#include "Arduino.h"
#include "stdlib_noniso.h"
#include <functional>

#ifdef ESP32
  #include <HTTPClient.h>
  #include <AsyncTCP.h>
  #include <ESPmDNS.h>
  #include <WiFi.h>
  #include <HTTPClient.h> 
  #include <Preferences.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <ESP8266mDNS.h>
#endif

#include "ESPAsyncWebServer.h"

#include "fs_switch.h"

#ifdef DEBUG_PRINT
#define debug_begin(...) Serial.begin(__VA_ARGS__);
#define debug_print(...) Serial.print(__VA_ARGS__);
#define debug_write(...) Serial.write(__VA_ARGS__);
#define debug_println(...) Serial.println(__VA_ARGS__);
#define debug_printf(...) Serial.printf(__VA_ARGS__);
#else
#define debug_begin(...)
#define debug_print(...)
#define debug_printf(...)
#define debug_write(...)
#define debug_println(...)
#endif

//#define WEBLOG_BUFFER_SIZE 512


// by JG: point to Webpage in SPIFF
#define LOG_URL "/log.html"

class AsyncWebLogClass : public Print
{
public:
    void begin(AsyncWebServer *server, const char* url = LOG_URL);

    // implement virtual funtions from Print Class
    size_t write (uint8_t);
    size_t write (const uint8_t* buffer, size_t size);
 
    // by JG:  log only oneway server-->client
    //void msgCallback(RecvMsgHandler _recv);
   
private:
    //String sWeblog;
    AsyncWebServer *_server = NULL;
    AsyncEventSource *_events=NULL; // use Server Send Events (SSE)
    //AsyncWebSocket *_ws;     // maybe alternative websocket

    // by JG:  only for websocket, SSE is only oneway
    //RecvMsgHandler _RecvFunc = NULL;

    #if defined(WEBSERIAL_DEBUG)
        void debug_println(const char* message);
    #endif
};
#ifdef WEB_APP
extern AsyncWebLogClass AsyncWebLog;
#else
#pragma message("Info : NO WEB_APP --> USE HardwareSerial")
extern HardwareSerial AsyncWebLog;
#endif

#endif
