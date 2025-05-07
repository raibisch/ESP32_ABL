
#include <Arduino.h>
#include "SPI.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include "WiFiGeneric.h"
#include "math.h"
#include <AsyncTCP.h>

#ifdef USE_ETH_INSTEAD_WIFI
#include <ETH.h>
#endif

#ifdef MQTT_ENABLE
#include <MQTT.h>
#endif

// now set in platformio.ini
//#define DEBUG_PRINT 1    // SET TO 0 OUT TO REMOVE TRACES
//#define DEBUG_WITHOUT_ABL 1 //calculate at "no com"


#ifdef DEBUG_PRINT
#pragma message("Info : DEBUG_PRINT=1")
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


#ifndef WITHOUT_TEMP
#include "driver/temp_sensor.h"
#endif


#include <ESPAsyncWebServer.h>
#include "ESP32Time.h"

// for EEPROM Emulation
#include <Preferences.h>

#include "FileVarStore.h"
#include "AsyncWebLog.h"
#include "AsyncWebOTA.h"

#ifdef DEF_S2_MINI
#pragma message(": Info for CuteCom: Set DTR for Serial-communication")
#endif

#define ABL_ENABLE_TX HIGH
#define ABL_ENABLE_RX LOW

/*
Set I-Soll:
=======
6A:
:01100014000102006474

7A:
:01100014000102007563

8A:
:01100014000102008553

9A:
:01100014000102009642

10A:
:0110001400010200A731

11A:
:0110001400010200B721

12A:
:0110001400010200C810

13A:
0110001400010200D9FF

14A:
:0110001400010200E9EF

15A:
:0110001400010200FADE

16A:
:01100014000102010BCC

*/

const char* ABL_TX_POLL_MESSAGE   = ":0103002E0005C9";
const char* ABL_TX_SET_6A         = ":01100014000102006474";
const char* ABL_TX_SET_8A         = ":01100014000102008553";
const char* ABL_TX_SET_10A        = ":0110001400010200A632";                                
const char* ABL_TX_SET_12A        = ":0110001400010200C810";
const char* ABL_TX_SET_14A        = ":0110001400010200E9EF";
const char* ABL_TX_SET_15A        = ":0110001400010200FADE";
const char* ABL_TX_SET_16A        = ":01100014000102010BCC";
const char* ABL_TX_SET_DISABLE    = ":0110001400010203E8ED";  

HardwareSerial Serial_ABL(1);

ESP32Time rtc(0);  

AsyncWebServer server(80);
#ifdef MQTT_ENABLE
WiFiClient client;
#endif
const char* PARAM_MESSAGE = "message";


enum ABL_POLL_STATUS {
  POLL_Current,      // implemented and tested
  SET_Current,       // implemented and tested
  SET_StopCharge,    // implemented and tested (ABL_TX_SET_DISABLE)
  SET_RestartCharge, // implemented and tested (SET_CURRENT after SET_CURRENT x3E8)
  SET_DISABLE_WB,    // todo... not implemented
  SET_ENABLE_WB,     // todo... not implemented
  POLL_FIRMWARE      // todo... not implemented
};


// !! same nummer of elements as ABL_STATUS_STRING !!
enum ABL_STATUS { 
ABL_A1=0,     // not connected,  Waiting for EV 
ABL_B1,       // connected, EV is asking for charging
ABL_B2,       // connected, EV hat the permission to charge (=charge finished)
ABL_C2,       // connected, EV in charging 
ABL_C3,       // connected, EV in charging reduced current
ABL_C4,       // connected, EV in charging reduced current
ABL_UNVALID,  // 
ABL_TIMEOUT   // no Rx after 2x Polltimes
};

const String ABL_STATUS_STRING[] = { 
 
"A1",     // not connected,  Waiting for EV 
"B1",     // connected, EV is asking for charging
"B2",     // connected, EV hat the permission to charge
"C2",     // connected, EV in charging 
"C3",     // connected, EV in charging reduced current
"C4",     // connected, EV in charging reduced current
"??",     // unvalid
"noCom"   // timeout or no connection
};

/*
not extra handeled but displayed status messages (from ABL docu):
=================================================================
E0 Outlet disabled
E1 Production test
E2 EVCC setup mode
E3 Bus idle
F1 Unintended closed contact (Welding)
F2 Internal error
F3 DC residual current detected
F4 Upstream communication timeout
F5 Lock of socket failed
F6 CS out of range
F7 State D requested by EV
F8 CP out of range
F9 Overcurrent detected
F10 Temperature outside limits
F11 Unintended opened contactd



dump:

TX:0103002E0005C9
Next Polling in :28sec
RX>01030A2EC2910B03E803E803E8A5

Status:C2 Ipwm:0B I1:03E8 I2:03E8 I3:03E8
Status:C2 Ipwm:0B I1:03E8 I2:03E8 I3:03E8
Ipwm=0.00 I=0.00+0.00+0.00+ Isum=0.00

Ipwm=0.00 I=0.00+0.00+0.00+ Isum=0.00

Next Polling in :26sec

*/
static unsigned long ABL_PollTime_old = 0;
static unsigned long ABL_StatusSec_old = 0;
static unsigned long ABL_kwh_StartTime = 0;
static uint64_t ABL_Wh_Sum_akt = 0;
static uint64_t ABL_Wh_Sum_old = 0;

// VALUES from ABL
// jetzt über Config
// varABL_i_U_netz
//static uint ABL_rx_VOLT= 230;          // default = 230V

static float     ABL_rx_Isum = 0.0;     // return value from ABL: summ of 3 phases or zero if no sensor availible
static uint16_t  ABL_rx_Ipwm = 0;       // return value from ABL: set from ABL_tx_Icmax
static float ABL_rx_kW = 0.0;
static uint64_t ABL_rx_Wh = 0; // calculated value in Wh *NOT* kWh
static String ABL_rx_status = ABL_STATUS_STRING[ABL_UNVALID];
static String ABL_rx_status_old = "---";
static bool ABL_rx_aktiv = 0;

static enum ABL_POLL_STATUS ABL_tx_status = POLL_Current;
// Values to ABL
static uint16_t ABL_tx_Icmax   = 6;
static String ABL_rx_String = "";          // a String to hold incoming data
static uint16_t ABL_rx_timeoutcount = 0;  
static String ABL_sChargeTime = "--:--:--";

static uint16_t SYS_RestartCount = 0;
static uint16_t SYS_TimeoutCount = 0;
static uint SYS_ChargeCount = 0;
static String SYS_Version = "V 1.3.0";
static String SYS_CompileTime =  __DATE__;
static String SYS_IP = "0.0.0.0";


#ifdef MQTT_ENABLE
#define MQTT_PAYLOAD_SIZE 256      // maximum MQTT message size
#define HOSTNAME_SIZE 30      // maximum hostname size
#define STATUS_INTERVAL 10000 // MQTT status message interval [ms]
#define DATA_INTERVAL   20000 // MQTT data message interval [ms]
const char MQTT_CLIENTID[] = "ABL";
#endif
     


static long int HexString2int(String s)
{
  const String hexDigits = "0123456789ABCDEF";
  long int result = 0;
  s.toUpperCase();
  for (int i = 0; i < s.length(); i++) {
    result <<= 4;
    result |= hexDigits.indexOf(s[i]);
  }
  return result;
}


// from comment "milenko-s"
String uint64ToString(uint64_t input)
{
  String result = "";
  uint8_t base = 10;

  do
  {
    char c = input % base;
    input /= base;
    c += '0';
    result = c + result;
  } while (input);
  return result;
}

  
/* ------------------until now not used--------------------------
static String ByteToAscii(byte input) 
{
  char singleChar, out;
  char asciiString[2+1] {0,0,0}; 
  sprintf(asciiString, "%2X", input);
  return String(asciiString);
}

char ByteToAscii(const char *input) 
{
  char singleChar, out;
  memcpy(&singleChar, input, 2);
  sprintf(&out, "%c", (int)strtol(&singleChar, NULL, 16));
  return out;
}

static byte calculateLRC(String s)
{
    byte LRC = 0;
    for (int i = 0; i < s.length(); i++)
    {
        LRC ^= (byte)s[i];
    }
    return LRC;
}
---------------------------not used -------------------------*/


////////////////////////////////////////////
/// @brief init builtin LED
////////////////////////////////////////////
void initLED()
{
#ifndef WITHOUT_LED
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH);
#endif
}

/// @brief  set builtin LED
/// @param i = HIGH / LOW
void setLED(uint8_t i)
{
   debug_printf("LED light %d\r\n", i);
#ifndef WITHOUT_LED
  digitalWrite(LED_GPIO, i);
#endif
}


//////////////////////////////////////////////////////
/// @brief  expand Class "FileVarStore" with variables
//////////////////////////////////////////////////////
class ABL_FileVarStore : public FileVarStore
{
 public:  
  // Device-Parameter
   String varDEVICE_s_Name  = "Wallbox";

   // ABL-Parameter
   uint16_t varABL_i_A_soll_high = 14;
   uint16_t varABL_i_A_soll_low  = 6;
   uint16_t varABL_i_U_netz = 230;
   uint32_t varABL_i_Scantime_ms = 30000;
   uint32_t varABL_i_logtime_ms  = 2000;
   uint16_t varABL_i_Watt_16A;
   uint16_t varABL_i_Watt_12A;
   uint16_t varABL_i_Watt_10A;
   uint16_t varABL_i_Watt_08A;
   uint16_t varABL_i_Watt_06A;

  // Wifi-Parameter
   String varWIFI_s_Mode    = "STA"; // STA=client connect with Router,  AP=Access-Point-Mode (needs no router)
   String varWIFI_s_Password= "mypassword";
   String varWIFI_s_SSID    = "myssid";

#ifdef MQTT_ENABLE
  uint16_t varMQTT_i_PORT = 1883; 
  String varMQTT_s_HOST = "192.168.2.22";
  String varMQTT_s_USER = "";
  String varMQTT_s_PASS = "";
  String varMQTT_s_TOPIC_OUT   = "abl/out/";
  String varMQTT_s_PAYLOAD_OUT ="{\"status\":%s, \"ipm\":%d, \"kw\":%d, \"kwh\":%d, \"whsum\":%d}"; // change to whatever you want !
  String varMQTT_s_TOPIC_IN    ="abl/in/"; // requested payload: {"imax":<value>}  e.g. {"imax:12}
#endif
     

 protected:
   void GetVariables()
   {
     varDEVICE_s_Name     = GetVarString(GETVARNAME(varDEVICE_s_Name));
     varWIFI_s_Mode       = GetVarString(GETVARNAME(varWIFI_s_Mode)); //STA or AP
     varWIFI_s_Password   = GetVarString(GETVARNAME(varWIFI_s_Password));
     varWIFI_s_SSID       = GetVarString(GETVARNAME(varWIFI_s_SSID));

     varABL_i_U_netz      = GetVarInt(GETVARNAME(varABL_i_U_netz));
     varABL_i_A_soll_low  = GetVarInt(GETVARNAME(varABL_i_A_soll_low));
     varABL_i_A_soll_high = GetVarInt(GETVARNAME(varABL_i_A_soll_high),16);
     varABL_i_Scantime_ms = GetVarInt(GETVARNAME(varABL_i_Scantime_ms),30000);  

     varABL_i_Watt_16A    = GetVarInt(GETVARNAME(varABL_i_Watt_16A),(230*2*16));
     varABL_i_Watt_12A    = GetVarInt(GETVARNAME(varABL_i_Watt_12A),(230*2*12));
     varABL_i_Watt_10A    = GetVarInt(GETVARNAME(varABL_i_Watt_10A),(230*2*10));
     varABL_i_Watt_08A    = GetVarInt(GETVARNAME(varABL_i_Watt_08A),(230*2*8));
     varABL_i_Watt_06A    = GetVarInt(GETVARNAME(varABL_i_Watt_06A),(230*2*6));
#ifdef MQTT_ENABLE
     varMQTT_i_PORT       = GetVarInt(GETVARNAME(varMQTT_i_PORT),1883);
     varMQTT_s_HOST       = GetVarString(GETVARNAME(varMQTT_s_HOST)); 
     varMQTT_s_USER       = GetVarString(GETVARNAME(varMQTT_s_USER)); 
     varMQTT_s_PASS       = GetVarString(GETVARNAME(varMQTT_s_PASS));
     varMQTT_s_TOPIC_OUT  = GetVarString(GETVARNAME(varMQTT_s_TOPIC_OUT));
     varMQTT_s_PAYLOAD_OUT= GetVarString(GETVARNAME(varMQTT_s_PAYLOAD_OUT));
     varMQTT_s_TOPIC_IN   = GetVarString(GETVARNAME(varMQTT_s_TOPIC_IN));
#endif  

   }
};
ABL_FileVarStore varStore;

void initFileVarStore()
{
  varStore.Load();
}



/// @brief do not wait for next polling periode
static void forcePolling()
{
      ABL_PollTime_old  = millis() + varStore.varABL_i_Scantime_ms;
}

 
#ifdef MQTT_ENABLE
// Generate MQTT client instance
// N.B.: Default message buffer size is too small!
MQTTClient mqttclient(MQTT_PAYLOAD_SIZE);

void mqtt_messageReceived(String &topic, String &payload) 
{
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
   debug_println("* MQTT: incoming: " + topic + " - " + payload);
   
   uint a = payload.substring(8,9).toInt();
   AsyncWebLog.println("MQTT SET-Imax:" + String(a)+ "A");
   if ((a >=6 && a <=16) || (a==0))
   {
      forcePolling();
      ABL_tx_Icmax = a;
      ABL_tx_status = SET_Current; // next Send Protocol
   }
   else
   {
      AsyncWebLog.println("**ERROR Imax out of range!");
   }
  ABL_tx_Icmax = payload.toInt();
}


/// @brief setup MQTT-client
void mqtt_setup()
{
  debug_println("* MQTT: connecting... ");
  mqttclient.begin(varStore.varMQTT_s_HOST.c_str(), varStore.varMQTT_i_PORT, client);
  mqttclient.onMessage(mqtt_messageReceived);
  while (!mqttclient.connect(MQTT_CLIENTID, varStore.varMQTT_s_USER.c_str(), varStore.varMQTT_s_PASS.c_str()))
  {
        Serial.print(".");
        delay(1000);
  }
  debug_println("* MQTT: connected!");

  debug_printf("* MQTT suscribe %s\r\n", varStore.varMQTT_s_TOPIC_IN);
  mqttclient.subscribe(varStore.varMQTT_s_TOPIC_IN);
}

/// @brief  MQTT run in loop()
inline void mqtt_loop()
{
    mqttclient.loop();
    delay(10);  // <- fixes some issues with WiFi stability

    if (!mqttclient.connected()) 
    {
      mqtt_setup();
    }
    char str[80] = {};
    // cheat: '%' in config.txt is replaces with '&' because web-edit does not allow % in string.  (varStore 'GetVarString' function does replace)
    // in config.txt:  varMQTT_s_PAYLOAD_OUT  ={"status":"&s", "ipm":&d, "w":&d, "wh":&d, "whsum":&d};
    // result -->                          str={"status":"%s", "ipm":%d, "w":%d, "wh":%d, "whsum":%d}
    // change if you want an other output !
    // ... or add more single publish messages for DOMOTICZ
    sprintf(str, varStore.varMQTT_s_PAYLOAD_OUT.c_str(), ABL_rx_status, ABL_rx_Ipwm, (int)(ABL_rx_kW*1000.0), (int)ABL_rx_Wh, (int)ABL_Wh_Sum_akt);
    debug_printf("* MQTT Topic_out:%s Payload:%s\r\n", varStore.varMQTT_s_TOPIC_OUT, str);
    mqttclient.publish(varStore.varMQTT_s_TOPIC_OUT, str);
   
}
#endif // MQTT
         

/// @brief ABL Rx-Timeout after 2 polling periodes
bool testTimeount()
{
        ABL_rx_timeoutcount++;

// DEBUG: Simulate status        
#ifdef DEBUG_WITHOUT_ABL
#pragma message("Info : DEBUG_WITHOUT_ABL=1")
        //ABL_rx_Ipwm = 16;
        ABL_rx_Ipwm = 10;
        debug_printf("ABL_rx_timeoutcount:%d\r\n", ABL_rx_timeoutcount);
        if (ABL_rx_timeoutcount < 2)
        {
          ABL_rx_status = ABL_STATUS_STRING[ABL_A1];
        }
        else
        if (ABL_rx_timeoutcount < 3)
        {
          ABL_rx_status = ABL_STATUS_STRING[ABL_C2];
        }
        else
        if (ABL_rx_timeoutcount < 4)
        {
          ABL_rx_status = "?";
        } 
        else
        if (ABL_rx_timeoutcount < 8)
        {
          ABL_rx_status = ABL_STATUS_STRING[ABL_C2];
        } 
        else
        if (ABL_rx_timeoutcount < 9)
        {
          ABL_rx_status = ABL_STATUS_STRING[ABL_B2];
        }
        else
        if (ABL_rx_timeoutcount < 10)
        {
          ABL_rx_status = "?";
        }
        else
        if(ABL_rx_timeoutcount < 11)
        {
          ABL_rx_status = ABL_STATUS_STRING[ABL_A1];
        }
        else
        if(ABL_rx_timeoutcount < 12)
        {
          ABL_rx_status = "F1"; // simulate Error Message
        }
        else
        {
          ABL_rx_timeoutcount = 0;
          ABL_rx_status = ABL_STATUS_STRING[ABL_B1];
        }
        return false;
#else
        if (ABL_rx_timeoutcount > 2)
        {
          ABL_rx_status = ABL_STATUS_STRING[ABL_TIMEOUT];
          return true;
        }
        return false;
#endif
}

// ---- EEPROM Simulation to save Values over Reset and Poweroff-------
Preferences hist;

void initHistory()
{
   hist.begin("history", false); // use history
   ABL_Wh_Sum_old = hist.getULong64("whsum",0);
   ABL_Wh_Sum_akt = ABL_Wh_Sum_old;
   debug_printf("restored ABL_Wh_Sum_old= %d\r\n",ABL_Wh_Sum_old);
   SYS_RestartCount = hist.getInt("restart",0);
   SYS_TimeoutCount = hist.getInt("timeout",0);
   SYS_ChargeCount =  hist.getInt("charge",0);
   debug_printf("restored = %d\r\n",SYS_RestartCount);
}


bool set_Wh_Sum(unsigned long whs)
{
  hist.putULong64("whsum", whs);
  return true;
}


bool saveHistory()
{
   ABL_Wh_Sum_old = ABL_Wh_Sum_akt;
   debug_printf("ABL_Wh_Sum_akt:%d\r\n",ABL_Wh_Sum_akt)
   hist.putULong64("whsum",ABL_Wh_Sum_akt);
   hist.putUInt("restart", SYS_RestartCount);
   hist.putUInt("timeout", SYS_TimeoutCount);
   return true;
}
// ----  END EPROM Simulation -----------------------------------------

//////////////////////////////////////////////////////////
static double Wh = 0;
/// @brief calculate aktual an total (sum) of W/h
/// @param polltime_ms 
/// @return 
/////////////////////////////////////////////////////////
void calculate_kWh()
{
   if (ABL_rx_status.startsWith("?")) // no calulation at unvalid data
   {
    return;
   }

   if (ABL_rx_status.startsWith("no")) // no calulation at timeout
   {
    return;
   }

   if (ABL_rx_status_old != ABL_rx_status) // Status is changing
   {

    // end charging
    if (ABL_rx_status_old.startsWith("C"))
    {
      saveHistory();
      hist.putInt("charge",SYS_ChargeCount++);
    }
    
    // start charging
    if (ABL_rx_status.startsWith("C"))
    {
      saveHistory();
      rtc.setTime(1);
      ABL_PollTime_old = 1;
      ABL_rx_Isum = 0;
      ABL_rx_kW = 0;
    }
    else
    // End of Charging  by JG 24.3.2024 set kw=0 at
    if (ABL_rx_status.startsWith("B"))
    {
      ABL_rx_kW = 0;
    }
    else 
    if (ABL_rx_status.startsWith("A"))
    {
      saveHistory();
      ABL_rx_Isum = 0;
      ABL_rx_kW = 0;
      ABL_rx_Wh = 0;
      ABL_sChargeTime = "        ";
    }
    ABL_rx_status_old = ABL_rx_status;
   } // End Status is changing

   if (ABL_rx_status.startsWith("C")) // is charging
   {
      if ( ABL_rx_Isum == 0) // virtual I-values: calculation with Watt Values from config-values
      {
           switch (ABL_rx_Ipwm)
           { 
            case 6:
            ABL_rx_kW = varStore.varABL_i_Watt_06A / 1000.0;
           break; 

           case 8:
            ABL_rx_kW = varStore.varABL_i_Watt_08A / 1000.0;
           break; 

          case 10:
            ABL_rx_kW = varStore.varABL_i_Watt_10A / 1000.0;
            break;  

           case 12:
            ABL_rx_kW = varStore.varABL_i_Watt_12A / 1000.0;
           break;

           case 16:
             ABL_rx_kW = varStore.varABL_i_Watt_16A / 1000.0;
           break;

           default: // use standard for all other values ... or... expand your code ;-)
            ABL_rx_kW = (230*2*ABL_rx_Ipwm) / 1000.0;
           break;
           }
      }
      else // real I-values from ABL
      { 
          ABL_rx_kW = uint32_t(ABL_rx_Isum*varStore.varABL_i_U_netz) / 1000.0;
      }
      
      ABL_sChargeTime = rtc.getTime();
      Wh = (ABL_rx_kW * (rtc.getEpoch()*1000)) / (double)3600.0;

      ABL_rx_Wh = round(Wh);  
      debug_printf("W/h:%d\r\n", ABL_rx_Wh);
      ABL_Wh_Sum_akt = ABL_Wh_Sum_old + ABL_rx_Wh;
    
  } 
}


/// @brief Init ABL communication over RS485
void ABL_init()
{
    debug_printf("ABL_init: parameters: ABL_RXD_GPIO=%d, ABL_TXT_GPIO=%d, ABL_RX_LOW_ENABLE_GPIO=%d\r\n", ABL_RXD_GPIO, ABL_TXT_GPIO, ABL_RX_LOW_ENABLE_GPIO);
    debug_println("ABL_init: start to initialize...");
    Serial_ABL.begin(38400, SERIAL_8E1, ABL_RXD_GPIO, ABL_TXT_GPIO);
    ABL_rx_String="";
    //ABL_rx_String.reserve(200);
    pinMode(ABL_RX_LOW_ENABLE_GPIO, OUTPUT_OPEN_DRAIN);
    digitalWrite(ABL_RX_LOW_ENABLE_GPIO, 0);
    delay(1);
    //Serial_ABL.flush();
    debug_println("ABL_init: check serial connection...");
    while(Serial_ABL.available())
    {
        Serial_ABL.read();
    }
    debug_println("ABL_init: OK!");
}

/// @brief Send to ABL
/// @param s 
void ABL_Send(ABL_POLL_STATUS s)
{
 String tx = "";

 switch (s)
 {
  case POLL_Current:
    tx = String(ABL_TX_POLL_MESSAGE);
  break;

  case SET_Current:
    switch (ABL_tx_Icmax)
    {
      case 0:
        tx = String(ABL_TX_SET_DISABLE);
      break;
      
      case 5:
      case 6:
       tx = String(ABL_TX_SET_6A);
      break;

      case 7:
      case 8:
         tx = String(ABL_TX_SET_8A);
      break;

      case 9:
      case 10:
          tx = String(ABL_TX_SET_10A);
      break;

      case 11:
      case 12:
           tx = String(ABL_TX_SET_12A);
      break;

      case 13:
      case 14:
          tx = String(ABL_TX_SET_14A);
      break;

      case 15:
         tx = String(ABL_TX_SET_15A);
      break;

      case 16:
          tx = String(ABL_TX_SET_16A);
      break;

      default:
          tx = String(ABL_TX_SET_6A);
      break;
    }
  break; // End Set_Current

  //todo: additional Tx-Commands
  // 
  // ModifyState: Register 0x05
  // Enable:  A1A1
  // Disable: E0E0 
  
  // GetFirmware: Register 0x01
  
  default:
  break;
 } // end of swtich(s) --> ABL_POLL_STATUS

 AsyncWebLog.println("TX" + tx);
 digitalWrite(ABL_RX_LOW_ENABLE_GPIO,1);
 for (int i =0; i< tx.length(); i++)
 {
  Serial_ABL.write(tx[i]);
 }
 Serial_ABL.write("\r\n");
 Serial_ABL.flush(true);
 digitalWrite(ABL_RX_LOW_ENABLE_GPIO, 0); // Switch from TX to RX
 delay(1000);
 // empty the first rx input because of possible trash after first ABL-wakeup call
 while (Serial_ABL.available()) 
 {
    // cppcheck-suppress unreadVariable
    char inChar = (char)Serial_ABL.read();
 }
 // send 2x for wakeup from sleep
 digitalWrite(ABL_RX_LOW_ENABLE_GPIO,1); // Switch form Rx to TX
 for (int i =0; i< tx.length(); i++)
 {
  Serial_ABL.write(tx[i]);
 }
 Serial_ABL.write("\r\n");
 Serial_ABL.flush(true);

 digitalWrite(ABL_RX_LOW_ENABLE_GPIO, 0); // switch from Tx to Rx
}

 /// @brief Parse the received Data from ABL
 /// @param s 
 bool ABL_ParseReceive(String s)
 {
/*
Register 0x002E Read current (full)
   r/w Reg leng, LRC
   r  46d   5
:01 03 002E 0005 C9   

Send: 
:0103002E0005C9
Answer: (Status A1 = not connected)
>01030A2EA1108503E803E803E8CD


                      0=not connected
                      0=not reduced
                      0=EN2 open
                      1=EN1 closes 
      Cnt Reg    State   0001       Imax       Ip1  Ip2  Ip3  LRC
      10                         133dx0.06
                                 = 8A
>01 03 0A 2E      A1     10        85          03E8 03E8 03E8 CD
cnt:               0      1         2          3 4  5 6  7 8 
>01 03 0A 2E      A1     10        64          03E8 03E8 03E8 EE
                                 100dx0.06
                                 =  6A
*/
  bool ret = false;
  if (s.startsWith(">01030A2E") && s.length()>=28 )  // 2E = Read current (full)
  {
       AsyncWebLog.println("Status:" + s.substring(9,11)+ " Ipwm:" + s.substring(13,15) + " I1:" + s.substring(15,19) +  + " I2:" + s.substring(19,23) + " I3:" + s.substring(23,27));  
       ABL_rx_status = s.substring(9,11);
       try
       {
         ABL_rx_Ipwm = (uint16_t)(round(HexString2int(s.substring(12,15)) * (float)0.06));
         uint32_t i1,i2,i3 =0;
        
         i1 = HexString2int(s.substring(15,19));
         if (i1 > 900)
         {i1=0;}

         i2 = HexString2int(s.substring(19,23));
         if (i2 > 900)
         {i2=0;}

         i3 = HexString2int(s.substring(23,27));
         if (i3 > 900)
         {i3=0;}
         
         ABL_rx_Isum = (i1 +i2 + i3) / 10;

         AsyncWebLog.println("Ipwm="+ String(ABL_rx_Ipwm)+  " I="+ String(i1/10.0)+ "+" + String(i2/10.0) +"+" +String(i3/10.0) +"+ Isum="+ String(ABL_rx_Isum)  + "\r\n");
         ret = true;
      
       }
       catch(const std::exception& e)
       {
         ABL_rx_Isum = 0;
         AsyncWebLog.println("*Exeption*");
       }   
  }
  else
  if (s.startsWith(">011000140001DA")) // 14 = set Imax OK (duty cycle)
  {
    // >011000140001DA
    AsyncWebLog.println("*RX Set-Imax OK*");
    ABL_tx_status = POLL_Current;  // next Command
    forcePolling();
  }
  else
  if (s.startsWith(">01"))
  {
     AsyncWebLog.println("*RX-UNKNOWN!:" + s);
  }
  else
  {
    AsyncWebLog.println("*RX-UNVALID!");
    ABL_rx_status = ABL_STATUS_STRING[ABL_UNVALID];
  }

  ABL_rx_String = "";
  return ret;
 }



void serialEventABL() 
{
    while (Serial_ABL.available()) 
    {
     char inChar = (char)Serial_ABL.read();
     ABL_rx_String += inChar;
     if (inChar == '\n') 
     {
      AsyncWebLog.println("RX"+ ABL_rx_String);
      if (ABL_ParseReceive(ABL_rx_String))
      {
        ABL_rx_timeoutcount = 0;
      }
      ABL_rx_String = "";
      delay(1000);
      setLED(0); // LED off
    }
   } // while
}

#ifdef USE_ETH_INSTEAD_WIFI
//////////////////////////////////////////
/// @brief Init Ethernet
/////////////////////////////////////////
void initEthernet()
{
    debug_print("Starting ETH interface...");
    ETH.begin();
    delay(200);
    ETH.setHostname(varStore.varDEVICE_s_Name.c_str());

    debug_print("ETH MAC: ");
    debug_print(ETH.macAddress());
    debug_print("IP Address: ");
    debug_print(ETH.localIP());
    SYS_IP = ETH.localIP().toString();
    return;
}

//////////////////////////////////////////
/// @brief Manage the Wifi Connection
/////////////////////////////////////////
void handleEthernetConnection() 
{
}
#endif // USE_ETH_INSTEAD_WIFI

//////////////////////////////////////////
/// @brief Init Wifi
/////////////////////////////////////////
void initWifi()
{
  // Test mit AP !!!!!!!!!!!!!!!!!!!!!
  //varStore.varWIFI_s_Mode="AP";
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // API Info: https://docs.espressif.com/projects/esp-idf/en/v4.4.6/esp32/api-reference/network/esp_wifi.html
   if (varStore.varWIFI_s_Mode == "AP")
   {
    delay(100);
    debug_println("INFO-WIFI:AP-Mode");
    WiFi.softAP(varStore.varDEVICE_s_Name.c_str());   
    debug_print("IP Address: ");
    SYS_IP = WiFi.softAPIP().toString();
    debug_println(SYS_IP);
   }
   else
   {
    debug_printf("INFO-WIFI:STA-Mode\r\n");
    WiFi.mode(WIFI_STA);
   
    WiFi.setHostname(varStore.varDEVICE_s_Name.c_str());
    WiFi.begin(varStore.varWIFI_s_SSID.c_str(), varStore.varWIFI_s_Password.c_str());
    int i = 0;
    delay(500);
    debug_printf("SSID:%s\r\n", varStore.varWIFI_s_SSID);
    ///debug_printf("Passwort:%s\r\n", varStore.varWIFI_s_Password);
    while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (i < 5))
    {
        debug_printf(".");
        setLED(i%2);
        i++;  
        delay(300);
    }
    delay(300);
    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
      debug_printf("Get WiFi-Power:%d\r\n",WiFi.getTxPower())
      debug_printf("Get WiFi-RSSI:%d\r\n",WiFi.RSSI());
      
      debug_print("IP Address: ");
      SYS_IP = WiFi.localIP().toString();
      debug_println(SYS_IP);
      return;
    }
    else
    {
      ESP.restart();
    }
   }

  return;
}

//////////////////////////////////////////
/// @brief Manage the Wifi Connection
/////////////////////////////////////////
void handleWifiConnection() 
{
    // Test if wifi is lost from router
    if ((varStore.varWIFI_s_Mode == "STA") && (WiFi.status() != WL_CONNECTED))
    {
        debug_println("Reconnecting to WiFi...");
        delay(100);
        if (!WiFi.reconnect())
        {
            saveHistory();
            hist.putInt("restart",SYS_RestartCount++);
            delay(200);
            ESP.restart();
        } 
    }
}

static String readString(File s) 
{
  String ret;
  int c = s.read();
  while (c >= 0) {
    ret += (char)c;
    c = s.read();
  }
  return ret;
}

int getIntStatus()
{
   long intValue = strtol(ABL_rx_status.c_str(), NULL, 16);
   return intValue;
}


// -------------------- WEBSERVER ----------------------------------------------
// -----------------------------------------------------------------------------
//


/// @brief replace placeholder "%<variable>%" in HTML-Code
/// @param var 
/// @return String
String setHtmlVar(const String& var)
{
  debug_print("func:setHtmlVar: ");
  debug_println(var);
 
  if (var == "CONFIG") // read config.txt
  {
    if (!SPIFFS.exists("/config.txt")) 
    {
     return String(F("Error: File 'config.txt' not found!"));
    }
    // read "config.txt" 
    fs::File configfile = SPIFFS.open("/config.txt", "r");   
    String sConfig;     
    if (configfile) 
    {
      sConfig = readString(configfile);
      configfile.close();
    }
    else 
    { // no "config.txt"
      sConfig = "";
    }
    return sConfig;
  } 
  else
  if (var== "DEVICEID")
  {
    return varStore.varDEVICE_s_Name;
  }
  if (var== "I-HIGH")
  {
    return "I-HIGH(" + String(varStore.varABL_i_A_soll_high) + "A)";
  }
  else
   if (var== "DEVICEID")
  {
    return varStore.varDEVICE_s_Name;
  }
  else
  if (var== "I-LOW")
  {
    return "I-LOW(" + String(varStore.varABL_i_A_soll_low) + "A)";
  }
  else
  if (var == "KWHSUM")
  {
    float f = 0.00;
    if (ABL_Wh_Sum_akt > 0)
    {
      f = ABL_Wh_Sum_akt / 1000.00;
    }
    return String(f);
  } 
  else
  if (var == "INFO")
  {
#ifndef WITHOUT_TEMP
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2;  // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    float temp_celsius = 0;
    temp_sensor_read_celsius(&temp_celsius);
    String temp = String(temp_celsius);
#else
    String temp = "(unknown)";
#endif // WITHOUT_TEMP
  
     return "Version    :"   + SYS_Version + 
            "\nBuild      :" + SYS_CompileTime + 
            "\nTemp(C)    :" + temp +
            "\nIP-Addr    :" + SYS_IP +
            "\nTimeout-Cnt:" + SYS_TimeoutCount + 
            "\nCharge-Cnt :" + SYS_ChargeCount + 
            "\nRestart-Cnt:" + String(SYS_RestartCount) + 
            "\nRSSI       :" + String(WiFi.RSSI()) + 
            "\nIpwm       :" + String(ABL_rx_Ipwm) + 
            "\nStatus     :" + ABL_rx_status;
  }
  else
  if (var == "IMAX")
  {
     return String(ABL_tx_Icmax);
  }
     

  return String();
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

// for "/" and "/index" handle post
void Handle_Index_Post(AsyncWebServerRequest *request)
{
   debug_println("Argument: " + request->argName(0));
   debug_println("Value: ");
   //uint8_t i = 0;
   //String s  = request->arg(i);
   //debug_println(s);
   //String sRet = "";
   if (request->argName(0) == "ihigh")
   {
     //ABL_Send("high");
     forcePolling();
     ABL_tx_Icmax = varStore.varABL_i_A_soll_high;
     ABL_tx_status = SET_Current;
   }
   else
   if (request->argName(0) == "ilow")
   {
       //sRet = ABL_Send("low");
       forcePolling();
       ABL_tx_Icmax = (int)varStore.varABL_i_A_soll_low;
       ABL_tx_status = SET_Current;
   }
   else
   if (request->argName(0) == "pause")
   {
       //sRet = ABL_Send("low");
       forcePolling();
       ABL_tx_Icmax = 0;
       ABL_tx_status = SET_Current;
      AsyncWebLog.println("CHARGE-PAUSE");

    }

   AsyncWebLog.println("SET-Current:" + String(ABL_tx_Icmax) + "A");
   
   request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
}

void initWebServer()
{ 
  debug_print("Init web server...\n");
  //Route for root / web page
  server.on("/",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
  });


  //Route for root /index web page
  server.on("/index.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
  });

  

  // > Version V1.2
  //Route for stored values
  server.on("/setcurrent.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/setcurrent.html", String(), false, setHtmlVar);
  });

  

  //Route for setup web page
  server.on("/setup.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/setup.html", String(), false, setHtmlVar);
  });


  //Route for config web page
  server.on("/config.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/config.html", String(), false, setHtmlVar);
  });

  
  //Route for stored values
  server.on("/setvalues.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/setvalues.html", String(), false, setHtmlVar);
  });

  // > Version V1.2
  //Route for Info-page
  server.on("/info.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/info.html", String(), false, setHtmlVar);
  });
  

  // Route for style-sheet
  server.on("/style.css",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/style.css", String(), false);
  });


  // fetch GET
  server.on("/fetch", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    
    // fetch is also used for setting Imax with URL-Variable
    // example: 
    // http://<your-ip>/fetch?imax=8 --> set Imax to 8A
    //
    // valid values: imax=0..6..8..10..12..14..15..16 
    // specal case imax=0 --> Pause charging
    if (request->args() > 0)
    {
    debug_println("GET-Argument: " + request->argName(0));
    debug_print("Value: ");
    uint8_t i = 0;
    String s  = request->arg(i);
    debug_println(s);
    if (request->argName(0) == "imax")
    {
       forcePolling();
       uint a = String(request->arg(i)).toInt();
       AsyncWebLog.println("GET-SET-Imax:" + String(a)+ "A");
       if ((a >=6 && a <=16) || (a==0))
       {
           ABL_tx_Icmax = a;
           ABL_tx_status = SET_Current; // next Send Protocol
       }
       else
       {
        AsyncWebLog.println("**ERROR Imax out of range!");
       }
    }
    }

    // return actual values
    // REMARK: if you change Imax it needs one more GET to return the actual value of Imax
    String s = String(ABL_rx_Ipwm)+',' + String(ABL_rx_kW) + ','+ ABL_rx_status+ ',' + String(ABL_rx_Wh/1000.0) + ',' + String(ABL_Wh_Sum_akt) + ',' +String(ABL_sChargeTime);
    request->send(200, "text/plain", s);
    //debug_println("server.on /fetch: "+ s);
  });

  // fetch GET
  server.on("/fetchkv", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    String s = "Imax=" + String(ABL_rx_Ipwm) + " A \n" + 
               "ActPower=" + String(ABL_rx_kW) + " kW\n" +
               "Status=" + String(ABL_rx_status) + "\n" +
               "IntStatus=" + String(getIntStatus()) + "\n" +
               "ActWork=" + String(ABL_rx_Wh/1000.0) + " kW/h\n" +
               "SumWork=" + String(ABL_Wh_Sum_akt) + " W/h\n" +
               "ChargeTime=" + String(ABL_sChargeTime) + "\n";
    request->send(200, "text/plain", s);
  });

  // config.txt GET
  server.on("/config.txt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/config.txt", "text/html", false);
  });

  
  // config.txt GET
  server.on("/reboot.html", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(404, "text/plain", "RESTART !");
    saveHistory();
    ESP.restart();
  });

  //.. some code for the navigation icons
  server.on("/home.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/home.png", String(), false);
  });

  server.on("/file-list.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/file-list.png", String(), false);
  });
  server.on("/settings.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/settings.png", String(), false);
  });
  server.on("/current.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/current.png", String(), false);
  });

  // ...a lot of code only for icons and favicons ;-))
  server.on("/manifest.json",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/manifest.json", String(), false);
  });
  server.on("/favicon.ico",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/favicon.ico", String(), false);
  });
  server.on("/apple-touch-icon.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/apple-touch-icon.png", String(), false);
  });
  server.on("/android-chrome-192x192.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/android-chrome-192x192.png", String(), false);
  });
  server.on("/android-chrome-384x384.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/android-chrome-384x384.png", String(), false);
  });

  // ------------ POSTs --------------------------------------------------------------------------
  // root (/) POST
  server.on("/",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    Handle_Index_Post(request);
  });

  // index.html POST
  server.on("/index.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    Handle_Index_Post(request);
  });
  
  // > Version V1.2 set current in extra page
  // index.html POST
  server.on("/setcurrent.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    Handle_Index_Post(request);
  });
  
  
  // config.html POST
  server.on("/config.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
   //debug_println("Argument: " + request->argName(0));
   //debug_println("Value: ");
   uint8_t i = 0;
   String s  = request->arg(i);
   debug_println(s);
   if (request->argName(0) == "saveconfig")
   {
       varStore.Save(s);
       varStore.Load();
   }
   //debug_println("Request /index3.html");
   request->send(SPIFFS, "/config.html", String(), false, setHtmlVar);
  });
  
  
  // setvalues.html POST
  server.on("/setvalues.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
   debug_println("Argument: " + request->argName(0));
   debug_print("Value: ");
   uint8_t i = 0;
   String s  = request->arg(i);
   debug_println(s);
   if (request->argName(0) == "kwh") // kW/h Sum
   {
      ABL_Wh_Sum_akt = u_long(s.toFloat()*1000);
      debug_print("ABL_Wh_Sum_akt: ");
      debug_println(ABL_Wh_Sum_akt);
      set_Wh_Sum(ABL_Wh_Sum_akt);
      ABL_Wh_Sum_old = ABL_Wh_Sum_akt;
      saveHistory();
   }    
   request->send(SPIFFS, "/setvalues.html", String(), false, setHtmlVar);
  });
  
  

   /*
   // Send a GET request to <IP>/get?message=<message>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else  // Serious problem{
            message = "No message sent";
        }
        request->send(200, "text/plain", "Return Value from GET: " + message);
  });

  // Send a POST request to <IP>/post with a form field message set to <message>
  server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request){
        String message;
        if (request->hasParam(PARAM_MESSAGE, true)) {
            message = request->getParam(PARAM_MESSAGE, true)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, POST: " + message);
  });
  */

  
  server.onNotFound(notFound);

  AsyncWebLog.begin(&server);
  AsyncWebOTA.begin(&server);
  server.begin();
}


void initSPIFFS()
{
  if (!SPIFFS.begin())
  {
   debug_println("*** ERROR: SPIFFS Mount failed");
  } 
  else
  {
   debug_println("* INFO: SPIFFS Mount succesfull");
  }
}


// ------------------((ABL_rx_status.startsWith("C")) && --------------------------
void setup()
{
  delay(800);
  debug_begin(115200);  
  delay(800);
  debug_println("Setup-Start");
  initSPIFFS();
#ifndef WITHOUT_LED
  initLED();
#endif
  initFileVarStore();
  initHistory();
  delay(200);
#ifdef USE_ETH_INSTEAD_WIFI
  initEthernet();
#else
  initWifi();
#endif
  delay(200);
#ifdef MQTT_ENABLE
  mqtt_setup();
#endif  
  initWebServer();
  ABL_init();
  delay(400);
  forcePolling();
  rtc.setTime(0);
}


static time_t charge_time;
static unsigned long log_timer =0;
static unsigned long tmp_poll_time_ms = 0;
static unsigned long now = 0;
static String s;
void loop()
{
    now = millis();
  
    if (ABL_rx_status.startsWith("C"))
    { tmp_poll_time_ms = 10000;}  // while charging 10sec polling.
    else
    { tmp_poll_time_ms = varStore.varABL_i_Scantime_ms;} // value > 30sec reduces standby power 


    if ((now - ABL_PollTime_old) >= tmp_poll_time_ms)
    {
       ABL_PollTime_old  = now;
       log_timer = tmp_poll_time_ms / 1000;
       setLED(1);
       ABL_Send(ABL_tx_status);
       testTimeount(); 
    }
    else
    {
        serialEventABL();
    }


    if ((now -ABL_StatusSec_old) >= varStore.varABL_i_logtime_ms)
    {
      ABL_StatusSec_old = now;
      log_timer = log_timer - (varStore.varABL_i_logtime_ms/1000);
      calculate_kWh();

      if (ABL_rx_status.startsWith("C"))
      { 
        s = ABL_sChargeTime + " Wh-akt:" + ABL_rx_Wh + " next Tx in:" + log_timer + "sec";
      }
      else
      {
        s = "Next Tx in :" + String(log_timer) + "sec";
      }
      AsyncWebLog.println(s);

#ifdef USE_ETH_INSTEAD_WIFI
      handleEthernetConnection();
#else
      handleWifiConnection();
#endif
    }

#ifdef MQTT_ENABLE
    mqtt_loop();
#endif              
}

