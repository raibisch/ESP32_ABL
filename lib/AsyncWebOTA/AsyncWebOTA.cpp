#include "AsyncWebOTA.h"


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


/* 
Logger page for use with AsyncWebServer
this code is mostly inspired by
https://github.com/ayushsharma82/WebSerial
...but implemented with Server Send Events (SSE) as Plugin for AsyncWebServer


include this script im Webpage 'log.html' or include it in PROG_MEM definition of HTML-Code
<script>
  var stopscroll=false;
  
  function clearText()
  {
    document.getElementById("logtext").value="";
  }
  function toggleScroll()
  {
     stopscroll=!stopscroll;
  }
 
  function addText(text)
  { 
    text = text.replace("<LF>","\n");
    document.getElementById("logtext").value += (text);
    if (stopscroll==false)
    {
      document.getElementById("logtext").scrollTop =  document.getElementById("logtext").scrollHeight;
    }
  }
 
  if (!!window.EventSource) 
  {
   var source = new EventSource('/logevents');
 
   source.addEventListener('open', function(e) {
     console.log("Events Connected");
   }, false);
 
   source.addEventListener('error', function(e) {
     if (e.target.readyState != EventSource.OPEN) {
       console.log("Events Disconnected");
     }
   }, false);
 
   source.addEventListener('message', function(e) {
     console.log("message", e.data);
  
   }, false);
 
   source.addEventListener('webprint', function(e) {
     console.log("logprint", e.data);
     addText(e.data);
   }, false);
  }
 </script>
*/

/// zunächst den alten Code benutzen
// https://lastminuteengineers.com/esp32-ota-web-updater-arduino-ide/

#define SEE_OTA "/otaevents"

// **************************************************************
// Include this bevore server.
//
void AsyncWebOTAClass::begin(AsyncWebServer *server, const char* url)
{
    _server = server;
    _events = new  AsyncEventSource(SEE_OTA);

  
  /* ... jetzt alles in WebOTA
         =====================
  //Route  for ota.html 
  server.on("/ota.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   Serial.println("Request /ota.html");
   request->send(SPIFFS, "/oTA.html", String(), false, setHtmlVar);
  });

  // Handle Web Server Events (SSE)
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }

    client->send("hi!", NULL, millis(), 10000);
  });
  server.addHandler(&events);
  */

  
  // SPIFFS (DATA) Formular-Seite
  _server->on("/ota.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   //Serial.println("/otam.html request content lenght:" + String(request->contentLength()));
   //AsyncWebServerResponse * response = request->beginResponse(SPIFFS,"/ota.html", String(), false, NULL);
     request->send(SPIFFS, "/ota.html", String(), false, NULL);
   //Serial.println("Request /ota.html");
   /*
   if (request->args() > 0)
   {
        Serial.println("OTA-No of args:" + request->args());
        Serial.println("OTA-Argument: " + request->argName(0));
         uint8_t i = 0;
         String s  = request->arg(i);
        Serial.println("OTA-Value: " + s);

   } 
  else
  {
    Serial.println("*** ERROR: no filename selected !");
  }
  */
  });


  /** Code von Andreas:
  *handling uploading firmware file *
  _server->on("/update", HTTP_POST, []() {
    _server->sendHeader("Connection", "close");
    _server->send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = _server->upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // flashing firmware to ESP
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
   **/

  // for test without update
  //#define NO_REAL_OTA
  _server->on("/ota_update", HTTP_POST, [&](AsyncWebServerRequest *request) 
  {
                // the request handler is triggered after the upload has finished... 
                // create the response, add header, and send response
                AsyncWebServerResponse *response = request->beginResponse((Update.hasError())?500:200, "text/plain", (Update.hasError())?"FAIL":"Upload OK!");
                response->addHeader("Connection", "close");
                response->addHeader("Access-Control-Allow-Origin", "*");
                request->send(response);
                 yield();
                 delay(1000);
                 yield();
                 ESP.restart();
            }, [&](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                //Upload handler chunks in data
                if (!index) 
                {   
                    Serial.print("request->args:");
                    Serial.println(request->args());
          
                    Serial.println(filename);
                    // z.Z. nur für Programm Daten (U_FLASH)
                    int cmd = U_FLASH;
                    if (filename.startsWith("spiff"))
                    {
                      cmd = U_SPIFFS;
                      Serial.println("cmd=U_SPIFFS (Upload Fileystem tor SPIFFS)");
                    }
                    else
                    {
                      Serial.println("cmd=U_FLASH (Upload Program)");
                    }
                   
#ifndef NO_REAL_OTA               
                    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
                    { // Start with max available size
                        Update.printError(Serial);
                        return request->send(400, "text/plain", "OTA could not begin!");
                    }
#endif
                    Serial.println("UPDATE: Start");
                }

                // Write chunked data to the free sketch space
                if(len)
                {
                    Serial.println("UPDATE lenth:" + String(len));
#ifndef NO_REAL_OTA
                    if (Update.write(data, len) != len) 
                    {
                        return request->send(400, "text/plain", "OTA length does not match!");
                    }
#endif
                }
                    
                if (final) 
                { // if the final flag is set then this is the last frame of data
                    if (!Update.end(true)) { //true to set the size to the current progress
                        Update.printError(Serial);
                        return request->send(400, "text/plain", "Could not end OTA");
                    }
                    Serial.println("UPDATE: END");
                }
                else
                {
                    return;
                }
            });

  
  
   /*noch anpassen
   //handling uploading firmware file 
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      // flashing firmware to ESP
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  */

 
   // Handle Web Server Events
  _events->onConnect([](AsyncEventSourceClient *client)
  {
    if(client->lastId())
    {
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    client->send("hi!", NULL, millis(), 10000);
  });

  _server->addHandler(_events);

    #if defined(WebOTA_DEBUG)
        DEBUG_WEB_SERIAL("Attached AsyncWebServer with SSE");
    #endif
}


void AsyncWebOTAClass::progress(int iprogress){
    String sprogress = String(iprogress);
    _events->send(String(sprogress).c_str(),"otaprogress", millis());
}


#if defined(WebLog_DEBUG)
    void WebOTAClass::DEBUG_WEB_SERIAL(const char* message){
        Serial.println("[WebOTA] "+message);
    }
#endif

AsyncWebOTAClass AsyncWebOTA;