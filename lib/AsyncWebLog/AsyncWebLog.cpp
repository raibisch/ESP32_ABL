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
    _events = new AsyncEventSource("/logevents");

  //Route  for log.html 
  _server->on("/log.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   debug_println("Request /log.html");
   request->send(myFS, "/log.html", String(), false, NULL);
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

// overwrite Print func
size_t AsyncWebLogClass::write(uint8_t m) 
{
  debug_println("no non-bufferd write on AsyncWebLog");
  return 0;
  /**
  if (!_events)
    return 0;
  String s = "";
  s += (const char) m;
  s.remove('\r');
  s.replace("\n", "<LF>");
  _events->send((const char*) s.c_str(), "logprint", millis(), 10000);
  debug_println("no non-bufferd write on AsyncWebLog");
  return s.length();
  */
}

size_t AsyncWebLogClass::write(const uint8_t* buffer, size_t size) 
{
   if (!_events)
    return 0;
  String s = (const char*) buffer;
  if (s.indexOf("\r\n") >= 0)
  {
   s.replace("\r\n", "<LF>");
   //s.replace("\r","");
   delay(5); // wegen doppelter Ausgabe evt. noch etwas groesser machen ?!
  }
  
  _events->send((const char*) s.c_str(),"logprint", millis(), 10000);
  return(s.length());
}

#if defined(WebLog_DEBUG)
    void AsyncWebLogClass::DEBUG_Web_SERIAL(const char* message){
        debug_println("[WebLog] "+message);
    }
#endif

#ifdef WEB_APP
AsyncWebLogClass AsyncWebLog;
#else
HardwareSerial AsyncWebLog(1); // no Webserver --> use serial1 
#endif