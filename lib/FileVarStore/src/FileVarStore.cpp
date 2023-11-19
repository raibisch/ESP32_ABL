#include "FileVarStore.h"


//----------------------------------------------------public Functions----------------------------------------------------

//default constructor 
FileVarStore::FileVarStore()
{
  _filename = "/config.txt";
}

// constructor with custom filename
FileVarStore::FileVarStore(String fn)
{
  _filename = "/"+ fn;
}

/*-----------------------------
destructor for class, not needed by Arduino but for complete class. Calls Arduino end function
*/
FileVarStore::~FileVarStore()
{
  
}

bool FileVarStore::Load()
{
   File configFile = SPIFFS.open(_filename);
  if (!configFile) 
  {
    debug_println("Failed to open config file");
    _isLoaded = false;
    return false;
  }

  size_t size = configFile.size();
  debug_printf("size of config.txt:%d \n", size);
  if (size > 1024) {
    debug_println("Config file size is too large");
    _isLoaded = false;
    return false;
  }

  _sBuf = configFile.readString();
  _sBuf.replace(" ","");

  GetVariables(); // virtual !!

  configFile.close();
  debug_println("config.txt load OK!");
  _isLoaded = true;
  return true;
}

bool FileVarStore::isLoaded()
{
  return _isLoaded;
}

bool FileVarStore::Save(String s)
{
  File configFile = SPIFFS.open(_filename, "w");
  if (!configFile) 
  {
    debug_println("ERROR: Failed to open config file for writing");
    return false;
  }
  configFile.print(s);
 
  configFile.close();
  debug_println("config.txt save OK!");
   // daten neu laden
  Load();
  return true;
}

// private member
String FileVarStore::GetVarString(String sK)
{
  int posStart = _sBuf.indexOf(sK);
  if (posStart < 0)
  {
    debug_println("key:'"+sK+ "' not found!!");
    return "";
  }
  //Serial.printf("PosStart1:%d", posStart);
  posStart = _sBuf.indexOf('=', posStart);
  //Serial.printf("PosStart2:%d", posStart);
  int posEnd   = _sBuf.indexOf(';', posStart);
  String sVal = _sBuf.substring(posStart+1,posEnd); 
  
  debug_print(sK+":");
  debug_print(sVal); 
  debug_printf(" length:%d\r\n", sVal.length());
  return sVal;
}

int32_t FileVarStore::GetVarInt(String sKey)
{   
  
  int32_t val = 0;
  int posStart = _sBuf.indexOf(sKey);
  if (posStart < 0)
  {
    debug_println("key:'"+sKey+ "' not found!!");
    return -1;
  }
  //Serial.printf("PosStart1:%d", posStart);
  posStart = _sBuf.indexOf('=', posStart);
  //Serial.printf("PosStart2:%d", posStart);
  int posEnd   = _sBuf.indexOf('\n', posStart);
  String sVal = _sBuf.substring(posStart+1,posEnd); 
  
  debug_print(sKey+":");
 
  val = sVal.toInt();
  debug_println(val); 
  return val;
 
}

int32_t FileVarStore::GetVarInt(String sKey, int32_t defaultvalue)
{   
  
  int32_t val = 0;
  int posStart = _sBuf.indexOf(sKey);
  if (posStart < 0)
  {
     debug_print("key:'"+sKey+ "' not found set defaultvalue: ");
     debug_println(val); 
    val = defaultvalue;
    return -1;
  }
  //Serial.printf("PosStart1:%d", posStart);
  posStart = _sBuf.indexOf('=', posStart);
  //Serial.printf("PosStart2:%d", posStart);
  int posEnd   = _sBuf.indexOf('\n', posStart);
  String sVal = _sBuf.substring(posStart+1,posEnd); 
  
  debug_print (sKey+":");
 
  val = sVal.toInt();
  debug_println(val); 
  return val;
 
}


float FileVarStore::GetVarFloat(String sKey)
{
  float val = 0;
  int posStart = _sBuf.indexOf(sKey);
  if (posStart < 0)
  {
    debug_println("key:'"+sKey+ "' not found!!");
    return -1;
  }
  //Serial.printf("PosStart1:%d", posStart);
  posStart = _sBuf.indexOf('=', posStart);
  //Serial.printf("PosStart2:%d", posStart);
  int posEnd   = _sBuf.indexOf('\n', posStart);
  String sVal = _sBuf.substring(posStart+1,posEnd); 
  
  debug_print (sKey+":");
 
  val = sVal.toFloat();
  debug_println(val); 
  return val;
}

bool FileVarStore::SetVar(String sKey, int32_t iVal)
{
  debug_println("FileVarStor::SetVar !!!not implemented!!!");
  return true;
}





