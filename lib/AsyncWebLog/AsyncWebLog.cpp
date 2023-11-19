#include "AsyncWebLog.h"

/* 
by JG (Juergen Goldmann)
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
 
   source.addEventListener('Webprint', function(e) {
     console.log("logprint", e.data);
     addText(e.data);
   }, false);
  }
 </script>
*/

// **************************************************************
/// @brief Start AsyncWeblog, include this before server call
/// @param server 
/// @param url 
void AsyncWebLogClass::begin(AsyncWebServer *server, const char* url)
{
    _server = server;
    _events = new  AsyncEventSource("/logevents");


  //Route  for log.html 
  _server->on("/log.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   debug_println("Request /log.html");
   request->send(SPIFFS, "/log.html", String(), false, NULL);
   // for static Webpage :
   //AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", LOGPAGE_HTML, LOGPAGE_HTML_SIZE);
   //response->addHeader("Content-Encoding","gzip");
   //request->send(response);  
  });

   // Handle Web Server Events
  _events->onConnect([](AsyncEventSourceClient *client)
  {
    if(client->lastId())
    {
      debug_printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    client->send("hi!", NULL, millis(), 10000);
  });

  _server->addHandler(_events);

    #if defined(WebLog_DEBUG)
        DEBUG_Web_SERIAL("Attached AsyncWebServer with SSE");
    #endif
}

#ifndef MINI_WEBLOG
void AsyncWebLogClass::print(String m){
    _events->send(m.c_str(),"logprint", millis());
}

void AsyncWebLogClass::print(const char *m){
    _events->send(m,"logprint", millis());
}

void AsyncWebLogClass::print(char *m){
    _events->send(m,"logprint", millis());
}

void AsyncWebLogClass::print(int m){
    _events->send(String(m).c_str(),"logprint", millis());
}

void AsyncWebLogClass::print(uint8_t m){
    _events->send(String(m).c_str(),"logprint", millis());
}

void AsyncWebLogClass::print(uint16_t m){
    _events->send(String(m).c_str(),"logprint", millis());
}


void AsyncWebLogClass::print(double m){
    _events->send(String(m).c_str(),"logprint", millis());
}

void AsyncWebLogClass::print(float m){
    _events->send(String(m).c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(const char *m)
{
    String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(char *m)
{
   String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(int m){
    String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(uint8_t m)
{
    String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(uint16_t m)
{
    String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(uint32_t m)
{
     String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(float m)
{
    String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}

void AsyncWebLogClass::println(double m)
{
    String s= String(m)+"<LF>";
    _events->send(s.c_str(),"logprint", millis());
}
#endif

// Print with New Line
// by JG: don't kill me for the lazy String convertion
void AsyncWebLogClass::println(String m)
{
    String s = m +"<LF>";
    _events->send(s.c_str(),"logprint", millis(),10000);        
}


#if defined(WebLog_DEBUG)
    void AsyncWebLogClass::DEBUG_Web_SERIAL(const char* message){
        debug_println("[WebLog] "+message);
    }
#endif

AsyncWebLogClass AsyncWebLog;