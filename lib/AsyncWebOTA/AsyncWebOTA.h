#ifndef AsyncWebOTA_h
#define AsyncWebOTA_h

#include "Arduino.h"
#include "stdlib_noniso.h"
#include <functional>

#include "WiFi.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "Update.h"
#include "SPIFFS.h"





////////////////////////////////////////


// by JG:  log only oneway server-->client
//typedef std::function<void(uint8_t *data, size_t len)> RecvMsgHandler;

// Uncomment to enable webserial debug mode
// #define WEBSERIAL_DEBUG 1
// only println for Strings
//#define MAXIMAL_WEBLOG

// by JG: point to Webpage in SPIFF
#define OTA_URL "/ota.html"

class AsyncWebOTAClass
{
public:
    void begin(AsyncWebServer *server, const char* url = OTA_URL);

    // by JG:  log only oneway server-->client
    //void msgCallback(RecvMsgHandler _recv);

    void progress(int iprogress);

private:
    AsyncWebServer *_server;
    AsyncEventSource *_events; // use Server Send Events (SSE)
    
    // by JG: not implemented ...and no need for Logging ;-)
    // if you need two-way communication use websocket
    //AsyncWebSocket *_ws;     
    //RecvMsgHandler _RecvFunc = NULL;
    
    #if defined(WEBSERIAL_DEBUG)
        void DEBUG_WEB_SERIAL(const char* message);
    #endif
};
extern AsyncWebOTAClass AsyncWebOTA;
#endif
