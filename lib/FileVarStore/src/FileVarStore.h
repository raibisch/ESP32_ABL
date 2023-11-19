#ifndef FILEVARSTORE_H
#define FILEVARSTORE_H

#include <Arduino.h>
#include <SPIFFS.h>


// Wandelt Source-Code Variablen-Nam varVALUE_i_ABL_minAmpere en in String um :-))
#define GETVARNAME(Variable) (#Variable)

// now def in platformio.ini
//#define DEBUG 1    // SET TO 0 OUT TO REMOVE TRACES

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


class FileVarStore
{
  public:
    FileVarStore();
    FileVarStore (String filename);
    ~FileVarStore ();
  
    bool isLoaded();
    bool Load();
    bool Save(String s);
     
protected:
    String _filename;
    String _sBuf;

    virtual void GetVariables(){};
    String GetVarString(String name);
    int32_t GetVarInt(String name);
    int32_t GetVarInt(String name, int32_t defaultvalue);
    float GetVarFloat(String name);

    bool SetVar(String sKey, int32_t iVal); // noch nicht implementiert
    bool _isLoaded = false;
};

#endif