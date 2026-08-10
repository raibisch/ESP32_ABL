
#include <Arduino.h>
#include "SPI.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include "WiFiGeneric.h"



#include "math.h"
// use math round() not arduino macro !!
#undef round

#include <ESPping.h>
//#include <AsyncTCP.h>

#ifdef USE_ETH_INSTEAD_WIFI
#include <ETH.h>
#endif

#ifdef MQTT_ENABLE
#include <MQTT.h>
#endif

// set in platformio.ini !
//#define DEBUG_PRINT 1       
//#define DEBUG_WITHOUT_ABL 1 //calculate at "no com"

// ************************************
// 20.3.26 Erweiterung auf 2 Wallboxen
// set in platformio.ini !
// ************************************
//#define WB_COUNT 1
//#define WB_MAX 2
// ************************************

#define SW_VERSION "2.3.1"

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
//#include "driver/temperature_sensor.h"
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
===========
all values are hardcoded with Checksum !

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

DISABLE:
:0110001400010203E8ED

*/


// BEDINGTE COMPILIERUNG: CORE-KONFIGURATION
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined (CONFIG_IDF_TARGE_ESP32)
    // Konfiguration für ESP32-S3 (Dual-Core)
    #define BOARD_NAME "ESP32-S3"
    #define USE_DUAL_CORE 1

#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    // Konfiguration für ESP32-S2 (Single-Core)
    #define BOARD_NAME "ESP32-S2"
    #define USE_DUAL_CORE 0
#else
    #error "Dieses Board wird ird von diesem Code nicht unterstützt!"
#endif



HardwareSerial Serial_ABL(1);

ESP32Time rtc[] = {0,0};

//ESP32Time rtc1(0); 
//ESP32Time rtc2(0); 

AsyncWebServer server(80);
QueueHandle_t msgQueue; 

#ifdef MQTT_ENABLE
WiFiClient client;
#endif
const char* PARAM_MESSAGE = "message";
bool bInitFileOK = false;


enum ABL_POLL_STATUS {
  POLL_Current,      // implemented and tested
  SET_Current,       // implemented and tested
  //SET_StopCharge,    // implemented and tested (ABL_TX_SET_DISABLE)
  //SET_RestartCharge, // implemented and tested (SET_CURRENT after SET_CURRENT x3E8)
  //SET_DISABLE_WB,    // todo... not implemented
  //SET_ENABLE_WB,     // todo... not implemented
  POLL_FIRMWARE      // todo... not implemented
};

// !! same nummer of elements as ABL_STATUS_STRING !!
enum ABL_STATUS { 
ABL_A1=0,     // not connected,  Waiting for EV 
ABL_B1,       // connected, EV is asking for charging
ABL_B2,       // connected, EV has the permission to charge (=charge finished)
ABL_C2,       // connected, EV in charging 
ABL_C3,       // connected, EV in charging reduced current
ABL_C4,       // connected, EV in charging reduced current
ABL_UNVALID,  // 
ABL_TIMEOUT,  // no Rx after 2x Polltimes
ABL_PAUSE     // Pause
};

const String ABL_STATUS_STRING[] = {  
"A1",     // not connected,  Waiting for EV 
"B1",     // connected, EV is asking for charging
"B2",     // connected, EV hat the permission to charge
"C2",     // connected, EV in charging 
"C3",     // connected, EV in charging reduced current
"C4",     // connected, EV in charging reduced current
"??",     // unvalid
"nocom",  // timeout or no connection
"pause "  // Pause
};


// Index für ABL_TX_MSG
enum ABL_TX_MSG_INDEX
{
  TX_POLL=0,
  TX_SET_6A,
  TX_SET_7A,
  TX_SET_8A,
  TX_SET_9A,
  TX_SET_10A,
  TX_SET_11A,
  TX_SET_12A,
  TX_SET_13A,
  TX_SET_14A,
  TX_SET_15A,
  TX_SET_16A,
  TX_SET_DISABLE,
  TX_index_count
};

/*
LRC-Berechnung
==============
Online Calculater:  http://www.metools.info/encoding/ecod127.html

Bei allen Befehlen muss das abschließende LRC Byte hinzugefügt werden.
Dazu muss man dort die Option Hex auswählen und dann den Befehl ohne Doppelpunkt z.B. 
aus der Abfrage der Firmware, hier also 010300010002 einfügen.
Dann erhält man unter Calculation result (Hex) den gewünschten Wert. Diese hängt man an den Befehl.
Hier also :010300010002+F9 also :010300010002F9
*/

const char* ABL_TX_MSG[] = {
 // Bus-Nr 01
 ":0103002E0005C9",          // POLL
 ":01100014000102006474",    // SET_6A
 ":01100014000102007563",    // SET_7A
 ":01100014000102008553",    // SET_8A
 ":01100014000102009642",    // SET_9A
 ":0110001400010200A632",    // SET_10A 
 ":0110001400010200B721",    // SET_11A                        
 ":0110001400010200C810",    // SET_12A
 ":0110001400010200D9FF",    // SET_13A
 ":0110001400010200E9EF",    // SET_14A
 ":0110001400010200FADE",    // SET_15A
 ":01100014000102010BCC",    // SET_16A
 ":0110001400010203E8ED",    // SET_DISABLE
 // Bus-Nr 02
 ":0203002E0005C8",          // POLL
 ":02100014000102006473",    // SET_6A
 ":02100014000102007562",    // SET_7A
 ":02100014000102008552",    // SET_8A
 ":02100014000102009641",    // SET_9A
 ":0210001400010200A631",    // SET_10A  
 ":0210001400010200B720",    // SET_11A                       
 ":0210001400010200C80F",    // SET_12A
 ":0210001400010200D9FE",    // SET_13A
 ":0210001400010200E9EE",    // SET_14A
 ":0210001400010200FADD",    // SET_15A
 ":02100014000102010BCB",    // SET_16A
 ":0210001400010203E8EC"     // SET_DISABLE
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


uint log_timer = 0;

#define POLLTIME_FAST  1500;
#define POLLTIME_ACTIV 3000
unsigned long tmp_poll_time_ms  = 10000;

unsigned long ABL_PollTime_old;
unsigned long ABL_StatusSec_old;

bool ABL_Par14aLimit_aktiv = false;

// new: 20.4.2026 extented to 2 Wallboxes
// define variable arrays to 2 !! independend from WB_COUNT setting !!
#define WB_MAX 2
uint64_t ABL_Wh_Sum_akt[WB_MAX];             
uint64_t ABL_Wh_Sum_old[WB_MAX];
bool ABL_PauseFlag[WB_MAX] = {false, false}; //new: 9.3.2026
float     ABL_rx_Isum[WB_MAX] = {0.0,0.0};   // return value from ABL: summ of 3 phases or zero if no sensor availible
uint16_t  ABL_rx_Ipwm[WB_MAX] = {0,0};       // return value from ABL: set from ABL_tx_Icmax
float     ABL_rx_kW[WB_MAX] = {0.0, 0.0};
uint64_t  ABL_rx_Wh[WB_MAX] = {0,0}; // calculated value in Wh *NOT* kWh
String    ABL_rx_status[WB_MAX] = {ABL_STATUS_STRING[ABL_UNVALID], ABL_STATUS_STRING[ABL_UNVALID]};
String    ABL_rx_status_old[WB_MAX] = {"---", "..."};
bool      ABL_rx_aktiv[WB_MAX] = {0,0};
ABL_POLL_STATUS ABL_tx_status[WB_MAX] = {ABL_POLL_STATUS::POLL_Current, ABL_POLL_STATUS::POLL_Current};
String ABL_sChargeTime[WB_MAX] = {"00:00:00","00:00:00"};

// now wait for polling
bool ABL_forcePollFlag[WB_MAX] = {false,false};

// new in V2.2 restore Limit-WB if Prio-WB has finnished charging
uint16_t ABL_LoadBal_Icmax[WB_MAX] = {6,6};
bool     ABL_LoadBal_pause[WB_MAX] = {false,false};
bool     ABL_LoadBal_active[WB_MAX] = {false,false};

// Values to ABL
uint16_t ABL_tx_Icmax[WB_MAX]   = {0,0};


String ABL_rx_String = "";          // a String to hold incoming data
uint16_t ABL_rx_timeoutcount[WB_MAX] = {0,0};  

int32_t SYS_ChargeCount[WB_MAX] = {0,0};
int32_t SYS_RestartCount = 0;
uint16_t SYS_TimeoutCount = 0;

String SYS_Version     = SW_VERSION;
String SYS_CompileDate = __DATE__;
String SYS_CompileTime = __TIME__;

String SYS_IP = "0.0.0.0";


#ifdef MQTT_ENABLE
#define MQTT_PAYLOAD_SIZE 256      // maximum MQTT message size
#define HOSTNAME_SIZE 30      // maximum hostname size
#define STATUS_INTERVAL 10000 // MQTT status message interval [ms]
#define DATA_INTERVAL   20000 // MQTT data message interval [ms]
const char MQTT_CLIENTID[] = "ABL";
#endif
     
#define ABL_SERIAL_BUFFER_SIZE 300
// Struktur für die Nachricht (Kein dynamic memory leak)
struct SerialMessage {
    char data[ABL_SERIAL_BUFFER_SIZE]; 
};


/// @brief Separater FreeRTOS Task für das Einlesen von Serial_ABL
/// @param pvParameters 
void serial_ABL_ReaderTask(void * pvParameters) {
    SerialMessage incomingMsg;
    int index = 0;

    for(;;) {
        while (Serial_ABL.available() > 0) 
        {
            char c = Serial_ABL.read();
            if (c == '\n' || c == '\r') 
            {
                if (index > 0) {
                    incomingMsg.data[index] = '\0'; // Nullterminator setzen
                   
                    // todo by JG: entweder die Daten hier direkt auswerten
                    //   oder
                    // Nachricht in die Queue schieben... und in der Loop auslesn
                    xQueueSend(msgQueue, &incomingMsg, pdMS_TO_TICKS(10));     
                    index = 0; 
                    //delay(1);
                }
            } 
            else if (index < ABL_SERIAL_BUFFER_SIZE) 
            {
                //delay(1);
                incomingMsg.data[index] = c;
                index++;
            }
            //delay(1);
        }
        
        // WICHTIG: 5ms Pause für den WiFi-Stack (Besonders kritisch beim Single-Core S2!)
        // original vTaskDelay(pdMS_TO_TICKS(5));
        //vTaskDelay(pdMS_TO_TICKS(10));
        delay(10);
    }
}





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
   //debug_printf("LED:%d\r\n", i);
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
   String varDEVICE_s_Name    = "ABL-WB";
   String varDEVICE_s_Name02  = "";

   // ABL-Parameter
   uint16_t varABL_i_A_soll_high = 14;
   uint16_t varABL_i_A_soll_low  = 6;
   uint16_t varABL_i_U_netz = 230;
   uint32_t varABL_i_Scantime_ms = 30000;
   uint32_t varABL_i_logtime_ms  = 1000;

   uint16_t varABL_i_Watt_16A;
   uint16_t varABL_i_Watt_15A;
   uint16_t varABL_i_Watt_14A;
   uint16_t varABL_i_Watt_13A;
   uint16_t varABL_i_Watt_12A;
   uint16_t varABL_i_Watt_11A;
   uint16_t varABL_i_Watt_10A;
   uint16_t varABL_i_Watt_09A;
   uint16_t varABL_i_Watt_08A;
   uint16_t varABL_i_Watt_07A;
   uint16_t varABL_i_Watt_06A;

  
   uint16_t varABL2_i_Watt_16A;
   uint16_t varABL2_i_Watt_15A;
   uint16_t varABL2_i_Watt_14A;
   uint16_t varABL2_i_Watt_13A;
   uint16_t varABL2_i_Watt_12A;
   uint16_t varABL2_i_Watt_11A;
   uint16_t varABL2_i_Watt_10A;
   uint16_t varABL2_i_Watt_09A;
   uint16_t varABL2_i_Watt_08A;
   uint16_t varABL2_i_Watt_07A;
   uint16_t varABL2_i_Watt_06A;

   // 18.5.26 new in V2.0
   uint16_t varABL_i_Phase_count=3;
   uint16_t varABL2_i_Phase_count=1;

   // 13.5.26 new: in V 2.0 Load-Management for 2 Wallboxes
   uint16_t varABL_i_I_limit = 16;
#if WB_COUNT == 2
   uint16_t varABL_i_LoadBal_mode  = 0;  // 0= 50% per Box, 1=WB1 Prio 2=WB2 Prio
#else
   uint16_t varABL_i_LoadBal_mode  = 1;  // 0= 50% per Box, 1=WB1 Prio 2=WB2 Prio
#endif
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
     varDEVICE_s_Name02   = GetVarString(GETVARNAME(varDEVICE_s_Name02));
     
     varWIFI_s_Mode       = GetVarString(GETVARNAME(varWIFI_s_Mode)); //STA or AP or API_STA
     varWIFI_s_Password   = GetVarString(GETVARNAME(varWIFI_s_Password));
     varWIFI_s_SSID       = GetVarString(GETVARNAME(varWIFI_s_SSID));

     varABL_i_U_netz      = GetVarInt(GETVARNAME(varABL_i_U_netz));
     varABL_i_A_soll_low  = GetVarInt(GETVARNAME(varABL_i_A_soll_low));
     varABL_i_A_soll_high = GetVarInt(GETVARNAME(varABL_i_A_soll_high),16);
     varABL_i_Scantime_ms = GetVarInt(GETVARNAME(varABL_i_Scantime_ms),30000);  

     varABL_i_Phase_count  = GetVarInt(GETVARNAME(varABL_i_Phase_count),3);
     varABL_i_I_limit      = GetVarInt(GETVARNAME(varABL_i_I_limit),16);
  #if WB_COUNT ==2
     varABL_i_LoadBal_mode= GetVarInt(GETVARNAME(varABL_i_LoadBal_mode),0);
  #endif
     varABL_i_Watt_16A    = GetVarInt(GETVARNAME(varABL_i_Watt_16A),(230*varABL_i_Phase_count*16));
     varABL_i_Watt_15A    = GetVarInt(GETVARNAME(varABL_i_Watt_15A),(230*varABL_i_Phase_count*15));
     varABL_i_Watt_14A    = GetVarInt(GETVARNAME(varABL_i_Watt_14A),(230*varABL_i_Phase_count*14));
     varABL_i_Watt_13A    = GetVarInt(GETVARNAME(varABL_i_Watt_13A),(230*varABL_i_Phase_count*13));
     varABL_i_Watt_12A    = GetVarInt(GETVARNAME(varABL_i_Watt_12A),(230*varABL_i_Phase_count*12));
     varABL_i_Watt_11A    = GetVarInt(GETVARNAME(varABL_i_Watt_11A),(230*varABL_i_Phase_count*11));
     varABL_i_Watt_10A    = GetVarInt(GETVARNAME(varABL_i_Watt_10A),(230*varABL_i_Phase_count*10));
     varABL_i_Watt_09A    = GetVarInt(GETVARNAME(varABL_i_Watt_09A),(230*varABL_i_Phase_count*9));
     varABL_i_Watt_08A    = GetVarInt(GETVARNAME(varABL_i_Watt_08A),(230*varABL_i_Phase_count*8));
     varABL_i_Watt_07A    = GetVarInt(GETVARNAME(varABL_i_Watt_07A),(230*varABL_i_Phase_count*7));
     varABL_i_Watt_06A    = GetVarInt(GETVARNAME(varABL_i_Watt_06A),(230*varABL_i_Phase_count*6));
#if WB_COUNT == 2
     varABL2_i_Phase_count = GetVarInt(GETVARNAME(varABL2_i_Phase_count),1);
     varABL2_i_Watt_16A   = GetVarInt(GETVARNAME(varABL2_i_Watt_16A),(230*varABL2_i_Phase_count*16));
     varABL2_i_Watt_15A   = GetVarInt(GETVARNAME(varABL2_i_Watt_15A),(230*varABL2_i_Phase_count*15));
     varABL2_i_Watt_14A   = GetVarInt(GETVARNAME(varABL2_i_Watt_14A),(230*varABL2_i_Phase_count*14));
     varABL2_i_Watt_13A   = GetVarInt(GETVARNAME(varABL2_i_Watt_13A),(230*varABL2_i_Phase_count*13));
     varABL2_i_Watt_12A   = GetVarInt(GETVARNAME(varABL2_i_Watt_12A),(230*varABL2_i_Phase_count*12));
     varABL2_i_Watt_11A   = GetVarInt(GETVARNAME(varABL2_i_Watt_11A),(230*varABL2_i_Phase_count*11));
     varABL2_i_Watt_10A   = GetVarInt(GETVARNAME(varABL2_i_Watt_10A),(230*varABL2_i_Phase_count*10));
     varABL2_i_Watt_09A   = GetVarInt(GETVARNAME(varABL2_i_Watt_09A),(230*varABL2_i_Phase_count*9));
     varABL2_i_Watt_08A   = GetVarInt(GETVARNAME(varABL2_i_Watt_08A),(230*varABL2_i_Phase_count*8));
     varABL2_i_Watt_07A   = GetVarInt(GETVARNAME(varABL2_i_Watt_07A),(230*varABL2_i_Phase_count*7));
     varABL2_i_Watt_06A   = GetVarInt(GETVARNAME(varABL2_i_Watt_06A),(230*varABL2_i_Phase_count*6));
#endif
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

bool initFileVarStore()
{
  return varStore.Load();
}


void forcePolling()
{
   for (size_t i = 0; i < WB_COUNT; i++)
   {
     ABL_forcePollFlag[i] = true;
   }
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
      ABL_tx_status01 = SET_Current; // next Send Protocol
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

  debug_printf("* MQTT suscribe %s\r\n", varStore.varMQTT_s_TOPIC_IN.c_str());
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
bool inline testTimeount(uint_fast16_t wb_ix)
{
        ABL_rx_timeoutcount[wb_ix]++;

// DEBUG: Simulate status
// !!! only for index "0"
#ifdef DEBUG_WITHOUT_ABL
#define SIMULATE_WB_IX 1
#pragma message("Info : DEBUG_WITHOUT_ABL=1")
        
        if (ABL_PauseFlag[SIMULATE_WB_IX] && ABL_rx_status[SIMULATE_WB_IX].startsWith("C"))
        {
          ABL_rx_status[SIMULATE_WB_IX] = "B2";
        }
        //ABL_rx_Ipwm = 16;
        ABL_rx_Ipwm[SIMULATE_WB_IX] = 6;
        debug_printf("ABL_rx_timeoutcount:%d\r\n", ABL_rx_timeoutcount);

        AsyncWebLog.print("ABL_rx_timeoutcount:");
        AsyncWebLog.println(String(ABL_rx_timeoutcount[SIMULATE_WB_IX]));
        
        if (ABL_rx_timeoutcount[SIMULATE_WB_IX] < 1)
        {
          ABL_rx_status[SIMULATE_WB_IX] = ABL_STATUS_STRING[ABL_A1];
        }
        else
        if (ABL_rx_timeoutcount[SIMULATE_WB_IX] < 3)
        {
          ABL_rx_status[SIMULATE_WB_IX] = ABL_STATUS_STRING[ABL_C2];
        }
        /*
        else
        if (ABL_rx_timeoutcount[0] < 4)
        {
          ABL_rx_status[0] = "?";
        } 
        */
        else
        if (ABL_rx_timeoutcount[SIMULATE_WB_IX] < 8)
        {
          ABL_rx_status[SIMULATE_WB_IX] = ABL_STATUS_STRING[ABL_C2];
        } 
        else
        if (ABL_rx_timeoutcount[SIMULATE_WB_IX] < 9)
        {
          ABL_rx_status[SIMULATE_WB_IX] = ABL_STATUS_STRING[ABL_B2];
        }
        /*
        else
        if (ABL_rx_timeoutcount[0] < 10)
        {
          ABL_rx_status[0] = "?";
        }
        else
        if(ABL_rx_timeoutcount[0] < 11)
        {
          ABL_rx_status[0] = ABL_STATUS_STRING[ABL_A1];
        }
        else
        if(ABL_rx_timeoutcount[0] < 12)
        {
          ABL_rx_status[0] = "F1"; // simulate Error Message
        }
        */
        else
        {
          ABL_rx_timeoutcount[SIMULATE_WB_IX] = 0;
          ABL_rx_status[SIMULATE_WB_IX] = ABL_STATUS_STRING[ABL_B2];
        }
        return false;
#else
        if (ABL_rx_timeoutcount[wb_ix] > 3)
        {
          ABL_rx_status[wb_ix] = ABL_STATUS_STRING[ABL_TIMEOUT];
          ABL_rx_Ipwm[wb_ix] = 0;
          return true;
        }
        return false;
#endif
}

// ---- EEPROM Simulation to save Values over Reset and Poweroff-------
Preferences hist;


void inline initHistory()
{
   hist.begin("history", false); // use history
   ABL_Wh_Sum_old[0] = hist.getULong64("whsum",0);
   ABL_Wh_Sum_akt[0] = ABL_Wh_Sum_old[0];
   ABL_Wh_Sum_old[1] = hist.getULong64("whsum02",0);
   ABL_Wh_Sum_akt[1] = ABL_Wh_Sum_old[1];

   debug_printf("restored ABL_Wh_Sum_old WB1= %d\r\n",ABL_Wh_Sum_old[0]);
   debug_printf("restored ABL_Wh_Sum_old WB2= %d\r\n",ABL_Wh_Sum_old[1]);
   SYS_RestartCount = hist.getInt("restart",0);
   SYS_TimeoutCount = hist.getInt("timeout",0);
   SYS_ChargeCount[0] =  hist.getInt("charge",0);
   SYS_ChargeCount[1] =  hist.getInt("charge2,0");
   debug_printf("restored charge count 1= %d\r\n",SYS_ChargeCount[0]);
   debug_printf("restored charge count 2= %d\r\n",SYS_ChargeCount[1]);

   // Reset values
   hist.putInt("restart",0);
}

bool set_Wh_Sum(uint_fast16_t wb_ix, unsigned long whs)
{
  if (wb_ix ==0)
  {
   hist.putULong64("whsum", whs);
  }
  else
  {
   hist.putLong64("whsum02", whs);
  }

  return true;
}


bool saveHistory()
{
   ABL_Wh_Sum_old[0] = ABL_Wh_Sum_akt[0];
   ABL_Wh_Sum_old[1] = ABL_Wh_Sum_akt[1];
   debug_printf("ABL_Wh_Sum_akt01:%d\r\n",ABL_Wh_Sum_akt[0])
   debug_printf("ABL_Wh_Sum_akt02:%d\r\n",ABL_Wh_Sum_akt[1])
   hist.putULong64("whsum",ABL_Wh_Sum_akt[0]);
   hist.putULong64("whsum02",ABL_Wh_Sum_akt[1]);
   //hist.putUInt("restart", SYS_RestartCount);
   //hist.putUInt("timeout", SYS_TimeoutCount);
   return true;
}
// ----  END EPROM Simulation -----------------------------------------


//////////////////////////////////////////////////////////
static double Wh = 0;
/// @brief calculate aktual an total (sum) of W/h
/// @param polltime_ms 
/// @return 
/////////////////////////////////////////////////////////
void calculate_kWh(uint_fast16_t wb_ix)
{
   //String sPrint;
   //sPrint.reserve(100);
   //sPrint = "[Calc]WB";
   //sPrint += wb_ix;
   //sPrint += " rx_status:";
   //sPrint += ABL_rx_status[wb_ix];
   if (ABL_PauseFlag[wb_ix])
   {
    //sPrint += " pause";
   }
  
   //AsyncWebLog.println(sPrint);
   if (ABL_rx_status[wb_ix].startsWith("?")) // no calulation at unvalid data
   {
    forcePolling();
    return;
   }

   if (ABL_rx_status[wb_ix].indexOf("no") >= 0) // no calulation at timeout
   {
    forcePolling();
    return;
   }

   if (ABL_rx_Ipwm[wb_ix] > 16)  // unvalid Ipwm
   {
    forcePolling();
    return;
   }


   if (ABL_rx_status_old[wb_ix] != ABL_rx_status[wb_ix]) // Status is changing
   {
    // end charging
    if (ABL_rx_status_old[wb_ix].indexOf("C") >= 0)
    {
      saveHistory();
      hist.putInt("charge",(SYS_ChargeCount[wb_ix])++);
    }
    else
    // start charging
    if (ABL_rx_status[wb_ix].indexOf("C") >= 0)
    {
      saveHistory();
      rtc[wb_ix].setTime(1);
      forcePolling();
      ABL_rx_Isum[wb_ix] = 0;
      ABL_rx_kW[wb_ix]   = 0;
    }
    else
    if (ABL_rx_status[wb_ix].indexOf("A") >= 0)
    {
      saveHistory();
      ABL_rx_Isum[wb_ix] = 0;
      ABL_rx_kW[wb_ix]   = 0;
      ABL_rx_Wh[wb_ix]   = 0;
      ABL_sChargeTime[wb_ix] = "00:00:00";
    }
    else
     // End of Charging  by JG 24.3.2024 set kw=0 at
    if (ABL_rx_status[wb_ix].indexOf("B") >= 0)
    {
      ABL_rx_kW[wb_ix] = 0;
      ABL_rx_Isum[wb_ix] = 0;
    }

    ABL_rx_status_old[wb_ix] = ABL_rx_status[wb_ix];
   } // End Status is changing


  // calculate kWh values from 'varStore.varABL_i_U_netz * varStore.varABL_i_Phase_count' 
  //#define CALC_POWER_U_PHASE
  
   if ((ABL_rx_status[wb_ix].indexOf("C") >= 0) && ABL_PauseFlag[wb_ix]==false) // is charging
   {
      //Serial.printf("WBx:%d status:%s Isum:%2.2f, Ipwm:%d", wb_ix, ABL_rx_status[wb_ix].c_str(), ABL_rx_Isum[wb_ix], ABL_rx_Ipwm[wb_ix]);
      if (ABL_rx_Isum[wb_ix] == 0) // virtual I-values: calculation with Watt Values from config-values
      {
  #ifndef CALC_POWER_U_PHASE
        if (wb_ix == 0)
        {
          switch (ABL_rx_Ipwm[0])
          {
          case 6:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_06A / 1000.0;
            break;
          case 7:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_07A / 1000.0;
            break;
          case 8:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_08A / 1000.0;
            break;
          case 9:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_09A / 1000.0;
            break;
          case 10:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_10A / 1000.0;
            break;
          case 11:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_11A / 1000.0;
            break;
          case 12:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_12A / 1000.0;
            break;
          case 13:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_13A / 1000.0;
            break;
          case 14:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_14A / 1000.0;
            break;
          case 15:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_15A / 1000.0;
            break;
          case 16:
            ABL_rx_kW[0] = varStore.varABL_i_Watt_16A / 1000.0;
            break;
          default: 
            ABL_rx_kW[0] = 0;
            break;
          }
        }
        else
        {
          switch (ABL_rx_Ipwm[1])
          {
          case 6:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_06A / 1000.0;
            break;
          case 7:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_07A / 1000.0;
            break;
          case 8:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_08A / 1000.0;
            break;
          case 9:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_09A / 1000.0;
            break;
          case 10:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_10A / 1000.0;
            break;
          case 11:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_11A / 1000.0;
            break;
          case 12:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_12A / 1000.0;
            break;
          case 13:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_13A / 1000.0;
            break;
          case 14:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_14A / 1000.0;
            break;
          case 15:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_15A / 1000.0;
            break;
          case 16:
            ABL_rx_kW[1] = varStore.varABL2_i_Watt_16A / 1000.0;
            break;
          default: 
            ABL_rx_kW[1] = 0;
            break;
          }
        }
#else
        // calculate kWh from PhaseCount and U_Netz variables
        if (ABL_rx_Ipwm[wb_ix] <= 16)
             {
              if (wb_ix == 0)
               {ABL_rx_kW[0] = (varStore.varABL_i_U_netz * varStore.varABL_i_Phase_count * ABL_rx_Ipwm[0]) / 1000.0;}
#if WB_COUNT == 2
              else
               {ABL_rx_kW[1] = (varStore.varABL_i_U_netz * varStore.varABL2_i_Phase_count * ABL_rx_Ipwm[1]) / 1000.0;}
#endif
             }
#endif           
      }
      else // real I-values from ABL
      { 
          ABL_rx_kW[wb_ix] = uint32_t(ABL_rx_Isum[wb_ix]*varStore.varABL_i_U_netz) / 1000.0;
      }
      
      ABL_sChargeTime[wb_ix] = rtc[wb_ix].getTime();
      Wh = (ABL_rx_kW[wb_ix] * (rtc[wb_ix].getEpoch()*1000)) / 3600.0;
      
      ABL_rx_Wh[wb_ix] = round(Wh);  
      debug_printf("W/h:%d\r\n", ABL_rx_Wh);
      ABL_Wh_Sum_akt[wb_ix] = ABL_Wh_Sum_old[wb_ix] + ABL_rx_Wh[wb_ix];

      //sPrint= "\r\n[Calc_C]WB"+ String(wb_ix+1) + " "+ ABL_sChargeTime[wb_ix] + " Wh-akt:" + ABL_rx_Wh[wb_ix]+ " Wh-Sum:" + ABL_Wh_Sum_akt[wb_ix];
      AsyncWebLog.printf("[CALC]WB%d %s Wh-akt:%d kW-Sum:%4.3f\r\n",wb_ix+1, ABL_sChargeTime[wb_ix].c_str(),ABL_rx_Wh[wb_ix], float(ABL_Wh_Sum_akt[wb_ix])/1000.0);

  }
  else
  {
   ABL_rx_kW[wb_ix] = 0;
   if (ABL_PauseFlag[wb_ix])
   {
        ABL_rx_Isum[wb_ix] = 0;
   } 
  } 

    
}

// 13.5.26 new: Load-Balance for 2 Wallboxes

/// @brief Set Imax with Load-Balance Limits and Modes
/// @param wb_ix 
/// @param icmax 
void setIcmax(uint_fast16_t wb_ix, uint16_t icmax)
{
  uint16_t imaxOther = 0;
  // Test Par14a Limit
  int inputLimit = digitalRead(PAR14LIMIT_GPIO);
  uint16_t Imax14aLimit = round(4200.0 / (varStore.varABL2_i_Phase_count * varStore.varABL_i_U_netz));
  if ((inputLimit == 0) && (icmax > Imax14aLimit))
  {
    ABL_tx_Icmax[wb_ix] = Imax14aLimit;
    AsyncWebLog.printf("[14aLimit] ON! : Limit to 4.2kW\r\n");
  }
  else
  if (icmax > varStore.varABL_i_I_limit)
  {
    ABL_tx_Icmax[wb_ix] = varStore.varABL_i_I_limit;
  }
  else
  {
    ABL_tx_Icmax[wb_ix] = icmax;
  }

  //              PAUSE 
  // =========================================================
  if (icmax == 0) 
  {
    ABL_tx_Icmax[wb_ix] = 0;
    ABL_PauseFlag[wb_ix] = true;
   AsyncWebLog.printf("[setPAUSE] WB%d\r\n",wb_ix+1);
         debug_printf("[setPAUSE] WB%d\r\n",wb_ix+1);
  }
  else
#if WB_COUNT == 1
  AsyncWebLog.printf("[SET   ] Icmax:%d\r\n ",icmax);
  ABL_PauseFlag[0] = false;
  ABL_tx_Icmax[0] = icmax;
#else
  // varABL_i_LoadBal_mode==0   -->  Load Balance WB01 WB02 50%
  // =========================================================
  if (varStore.varABL_i_LoadBal_mode == 0)
  {
    for (size_t i = 0; i < WB_COUNT; i++)
    {
     if (i != wb_ix)
     {
       imaxOther += ABL_tx_Icmax[i];
       AsyncWebLog.printf("[LoadBal:0] calc2 WB%d ImaxOther:%d\r\n",wb_ix+1, imaxOther);
             debug_printf("[LoadBal:0] calc2 WB%d ImaxOther:%d\r\n",wb_ix+1, imaxOther);
     }
    }
    
    AsyncWebLog.printf("[LoadBal:0] set WB%d ImaxOther:%d\r\n",wb_ix+1, imaxOther);
          debug_printf("[LoadBal:0] set WB%d ImaxOther:%d\r\n",wb_ix+1, imaxOther);

    if (icmax > (varStore.varABL_i_I_limit - imaxOther))
    {
      // Möglichkeit A: Rest ist nur noch verfügbar
      //ABL_tx_Icmax[wb_ix] = varStore.varABL_i_LoadBal_Imax - imaxFree;
      
      // Möglichkeit B: 50% Aufteilung --> z.Z. realisiert !
      for (size_t i = 0; i < WB_COUNT; i++)
      {
        ABL_tx_Icmax[i] = varStore.varABL_i_I_limit / 2;
        ABL_tx_status[i] = SET_Current;
      }
      
      AsyncWebLog.printf("[LoadBalMode0] LIMIT ! WB%d to Imax:%d\r\n",wb_ix+1, ABL_tx_Icmax[wb_ix]);
            debug_printf("[LoadBalMode0] LIMIT ! WB%d to Imax:%d\r\n",wb_ix+1, ABL_tx_Icmax[wb_ix]);
    }
    else
    {
     
      AsyncWebLog.printf("[LoadBalMode0] NO limit WB%d to Imax:%d\r\n",wb_ix+1, ABL_tx_Icmax[wb_ix]); 
            debug_printf("[LoadBalMode0] NO LIMIT WB%d to Imax:%d\r\n",wb_ix+1, ABL_tx_Icmax[wb_ix]); 
    }
  }
#endif
   AsyncWebLog.printf("[setIcmax] WB%d  Icmax=%d\r\n", wb_ix+1, ABL_tx_Icmax[wb_ix]);
   ABL_forcePollFlag[wb_ix] = true;
   ABL_tx_status[wb_ix] = SET_Current;

}


/// @brief Reset Pause of Limited-WB
/// z.Z. nur für 2 Wallboxen realisiert !!

void testLoadBal()
{
  
   if (ABL_PauseFlag[0] == true)
   {
      AsyncWebLog.printf("[LoadBalMode1] WB01=PAUSE -->WB02 unlocked! \r\n");
      if (ABL_LoadBal_pause[1] == true)
      {
        ABL_LoadBal_pause[1] = false;
        setIcmax(1,ABL_LoadBal_Icmax[1]);
      }
      return;
   } 

  // z.Z. nur for Prio WB01 realisiert !!
  if (varStore.varABL_i_LoadBal_mode == 1)
  {
   // CHARGE ('C') -----------------------------------  
   if (ABL_rx_status[0].indexOf('C') >= 0)
   {
    AsyncWebLog.printf("[LoadBalMode1] WB01=charge[C2]-->WB02 locked! \r\n");
    ABL_LoadBal_active[0] = true;
    ABL_LoadBal_pause[1]  = true;
    if ((ABL_LoadBal_pause[1]== false) || (ABL_rx_status[1].indexOf('C') >= 0))
    {
      ABL_LoadBal_pause[1] = true;
      ABL_LoadBal_Icmax[1] = ABL_tx_Icmax[1];
      setIcmax(1,0);
    }
   }
   else 
   // OPEN ('A') ---------------------------------------
   if (ABL_rx_status[0].indexOf('A') >= 0)
   {
     AsyncWebLog.printf("[LoadBalMode1] WB01=open[A1]-->WB02 unlocked! \r\n");
     ABL_LoadBal_active[0] = false;
     ABL_LoadBal_pause[1]  = false;
   }
   else
   // CONNECTED ('B') ----------------------------------
   if ((ABL_rx_status[0].indexOf('B') >= 0))
   { 
     // war vorher aktiv (=Ladeende)
     if (ABL_LoadBal_active[0] == true)
     {
       ABL_LoadBal_active[0] = false;
       ABL_LoadBal_pause[1]  = false;
       setIcmax(1,ABL_LoadBal_Icmax[1]);
     } 
     else
     if ((ABL_rx_status[1].indexOf('C') >= 0) || (ABL_PauseFlag[1] == false))
     {
         ABL_LoadBal_pause[1] = true;
         ABL_LoadBal_Icmax[1] = ABL_tx_Icmax[1];
         setIcmax(1,0);
     }

     AsyncWebLog.printf("[LoadBalMode1] WB01=connected[B2]-->WB02Pause=%d\r\n",ABL_LoadBal_pause[1]);    
   }
   
   
    
  } // End LoadBal_mode == 1
}


/// @brief 
/// @param wb_ix 
/// @param iwatt  in Watt NOT kW !!
void setPmax(uint_fast16_t wb_ix, uint16_t iwatt)
{
   
   uint nPhase = varStore.varABL_i_Phase_count;
   uint ILimit = varStore.varABL_i_I_limit;
   if (wb_ix == 1)
   {
     nPhase = varStore.varABL2_i_Phase_count;
   }
   uint16_t iAmpere = round(float(iwatt) / float(nPhase * varStore.varABL_i_U_netz));
   if (iAmpere > ILimit)
   {
     iAmpere= ILimit;
     AsyncWebLog.printf("[SetPmax] Warning: LIMIT-I:%d WB%d P:%dW I:%dA\r\n",ILimit, wb_ix+1, iwatt, iAmpere);   
   }
   else
   {
     AsyncWebLog.printf("[SetPmax]WB%d P:%dW I:%dA\r\n", wb_ix+1, iwatt, iAmpere);
   }
   
   setIcmax(wb_ix, iAmpere);
}



/// @brief  Limit to 4.2kW for German regulation
/// @param  wb_ix (0..1)
void testPar14aLimit(uint_fast16_t wb_ix)
{
    // Liest den aktuellen Status des Pins ein
  int inputLimit = digitalRead(PAR14LIMIT_GPIO);
  if (inputLimit == 0)
  {
    if (ABL_Par14aLimit_aktiv == false)
    {
      ABL_Par14aLimit_aktiv = true;
      setPmax(wb_ix, 4200);
    }
    AsyncWebLog.printf("[Par14aLimit] Input-Limit: %d\r\n", ABL_Par14aLimit_aktiv);
 }
 else
 {
   ABL_Par14aLimit_aktiv = false;
 }
}

#define ABL_RX_BUFFER_SIZE 512
/// @brief Init ABL communication over RS485
void inline ABL_init()
{
    Serial.printf("ABL_init: parameters: ABL_RXD_GPIO=%d, ABL_TXT_GPIO=%d, ABL_RX_LOW_ENABLE_GPIO=%d\r\n", ABL_RXD_GPIO, ABL_TXT_GPIO, ABL_RX_LOW_ENABLE_GPIO);
    debug_println("ABL_init: start to initialize...");

    // You MUST set the buffer size before begin
    Serial_ABL.setRxBufferSize(ABL_RX_BUFFER_SIZE); 
    Serial_ABL.begin(38400, SERIAL_8E1, ABL_RXD_GPIO, ABL_TXT_GPIO);
    Serial_ABL.setTimeout(200);
    
    ABL_rx_String="";
    ABL_rx_String.reserve(512);
    pinMode(ABL_RX_LOW_ENABLE_GPIO, OUTPUT_OPEN_DRAIN);
    digitalWrite(ABL_RX_LOW_ENABLE_GPIO, 0);
    delay(100);
    //Serial_ABL.flush();
    debug_println("ABL_init: check serial connection...");
    //while(Serial_ABL.available())
    //{
    //    Serial_ABL.read();
    //}
    
     // Queue für maximal 10 Nachrichten erstellen
    msgQueue = xQueueCreate(10, sizeof(SerialMessage));
    // BEDINGTE COMPILIERUNG: Task-Erstellung je nach Core-Anzahl
    #if (USE_DUAL_CORE == 1)
        // ESP-S3 Task fest auf Core 0 (isoliert vom Webserver)
        xTaskCreatePinnedToCore(
            serial_ABL_ReaderTask, "SerialABLTask", 3072, NULL, 1, NULL, 0
        );
        Serial.println("Task auf Core 0 gestartet (Dual-Core Modus).");
    #else
        // ESP32-S2 FreeRTOS Time-Slicing auf dem einen Core regeln
        xTaskCreate(
            serial_ABL_ReaderTask, "SerialABLTask", 3072, NULL, 1, NULL
        );
        Serial.println("Task im Multitasking-Modus gestartet (Single-Core Modus).");
    #endif
    forcePolling();
    ABL_PollTime_old  = 0;
    ABL_StatusSec_old = 0;
    debug_println("ABL_init: OK!");
}

/// @brief Send to ABL
/// @param s 
void ABL_Send(uint8_t wb_ix,  ABL_POLL_STATUS s)
{
 String tx = "";

 uint_fast16_t msg_ix_offset = wb_ix* ABL_TX_MSG_INDEX::TX_index_count;

 switch (s)
 {
  case POLL_Current:
    ABL_forcePollFlag[wb_ix] = 0;
    tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_POLL + msg_ix_offset]);
  break;

  case SET_Current:
    ABL_PauseFlag[wb_ix] = false;
    switch (ABL_tx_Icmax[wb_ix])
    {
      case 0:
        ABL_PauseFlag[wb_ix] = true;
        tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_DISABLE + msg_ix_offset]);
      break;
   
      case 6:
       tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_6A + msg_ix_offset]);
      break;

      case 7:
        tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_7A + msg_ix_offset]);
      break;

      case 8:
          tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_8A + msg_ix_offset]);
      break;

      case 9:
           tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_9A + msg_ix_offset]);
      break;

      case 10:
         tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_10A + msg_ix_offset]);
      break;
         
      case 11:
          tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_11A + msg_ix_offset]);
      break;

      case 12:
           tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_12A + msg_ix_offset]);
      break;

      case 13:
          tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_13A + msg_ix_offset]);
      break;

      case 14:
          tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_14A + msg_ix_offset]);
      break;

      case 15:
         tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_15A + msg_ix_offset]);
      break;

      case 16:
         tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_16A + msg_ix_offset]);
      break;

      default:
         tx = String(ABL_TX_MSG[ABL_TX_MSG_INDEX::TX_SET_6A + msg_ix_offset]);
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

 AsyncWebLog.printf("TX%s\r\n", tx.c_str());
 debug_printf("TX %s\r\n",tx.c_str());
 digitalWrite(ABL_RX_LOW_ENABLE_GPIO,1);
 for (int i =0; i< tx.length(); i++)
 {
  delay(1);
  Serial_ABL.write(tx[i]);
 }
 Serial_ABL.write("\r\n");
 Serial_ABL.flush(true);
 digitalWrite(ABL_RX_LOW_ENABLE_GPIO, 0); // Switch from TX to RX
 delay(1000);
 // empty the first rx input because of possible trash after first ABL-wakeup call

 //while (Serial_ABL.available() > 0) 
 //{
 //   // cppcheck-suppress unreadVariable
 //   char inChar = (char)Serial_ABL.read();
 //}
 
 // send 2x for wakeup from sleep
 digitalWrite(ABL_RX_LOW_ENABLE_GPIO,1); // Switch form Rx to TX
 for (int i =0; i< tx.length(); i++)
 {
  //delay(1);
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
  uint_fast16_t wb_ix = 0;
  //String sLog = "";
  //sLog.reserve(250);
  AsyncWebLog.printf("RX%s\r\n", s.c_str());
  
  if (s.startsWith(">0",0))
  {
   if (s.startsWith("01",1))
   {
     wb_ix = 0;
   }
   if (s.startsWith("02",1))
   {
    wb_ix = 1;
   }
  }
  else
  {
    AsyncWebLog.printf("[RX  ]Parse **START-UNVALID!\r\n");
    ABL_rx_status[wb_ix] = ABL_STATUS_STRING[ABL_UNVALID];
    return false;
  }


  if(s.indexOf("03A02E") && s.length()>=28)
  {
     
       //sLog = "\r\nStatus:" + s.substring(9,11)+ " Ipwm:" + s.substring(13,15) + " I1:" + s.substring(15,19) +  + " I2:" + s.substring(19,23) + " I3:" + s.substring(23,27);
       //AsyncWebLog.println(sLog);  
       ABL_rx_status[wb_ix] = s.substring(9,11);
       try
       {
         ABL_rx_Ipwm[wb_ix] = (uint16_t)(round(HexString2int(s.substring(12,15)) * (float)0.06));
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
         
         ABL_rx_Isum[wb_ix] = (i1 +i2 + i3) / 10;
         
         //sLog = "WB" + String(wb_ix+1) + " state:" + ABL_rx_status[wb_ix] + " Ipwm:"+ String(ABL_rx_Ipwm[wb_ix])+  " I:"+ String(i1/10.0)+ "+" + String(i2/10.0) +"+" +String(i3/10.0) +"=Isum:"+ String(ABL_rx_Isum[wb_ix]);
         AsyncWebLog.printf("[RX  ]WB%d Parser Status:%s, Ipwm:%d, Isum:%3.2f\r\n",wb_ix+1,ABL_rx_status[wb_ix].c_str(), ABL_rx_Ipwm[wb_ix], ABL_rx_Isum[wb_ix]);
         ret = true;
    
       }
       catch(const std::exception& e)
       {
         ABL_rx_Isum[wb_ix] = 0;
         AsyncWebLog.printf("[RX  ]WB%d Parser**Exeption!\r\n", wb_ix+1);
       }   
  }
  else
  if (s.indexOf("1000140001"))
  //if (s.startsWith(">011000140001DA")) // 14 = set Imax OK (duty cycle)
  {
    // >011000140001DA
    AsyncWebLog.printf("[RX  ]WB%d Parser Set-Imax OK\r\n", wb_ix+1);
    ABL_tx_status[wb_ix] = POLL_Current;  // next Command
    forcePolling();
  }
  //else
  //if (s.startsWith(">01"))
  //{
  //   AsyncWebLog.println("*RX-UNKNOWN!:" + s);
  //}
  else
  {
    AsyncWebLog.printf("[RX  ]WB%d Parser **UNVALID!\r\n", wb_ix+1);
    ABL_rx_status[wb_ix] = ABL_STATUS_STRING[ABL_UNVALID];
    ret = false;
  }

  ABL_rx_String = "";
  return ret;
 }


// old !!
/*
void serialEventABL() 
{
    static char inChar;
    vTaskDelay(pdMS_TO_TICKS(5));
    while (Serial_ABL.available() > 0) 
    {
     inChar = (char)Serial_ABL.read();
     ABL_rx_String += inChar;
     debug_print(inChar);
     if (inChar == '\n') 
     {
      //debug_printf("RX:%s\r\n",ABL_rx_String.c_str());
      //AsyncWebLog.println("RX"+ ABL_rx_String);
      if (ABL_ParseReceive(ABL_rx_String))
      {
       
        if (ABL_rx_String.startsWith("01",1))
         {ABL_rx_timeoutcount[0] = 0;}
        else if (ABL_rx_String.startsWith("02",1))
         {ABL_rx_timeoutcount[1] = 0;}
      }
      ABL_rx_String = "";
      //delay(100);
      //setLED(0); // LED off
    }
    vTaskDelay(pdMS_TO_TICKS(5));
   } // while
}
*/

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

void inline initWifi(bool bSetAP)
{
  // Test mit AP !!!!!!!!!!!!!!!!!!!!!
  //varStore.varWIFI_s_Mode="AP";
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // API Info: https://docs.espressif.com/projects/esp-idf/en/v4.4.6/esp32/api-reference/network/esp_wifi.html 
   if (bSetAP || (varStore.varWIFI_s_Mode == "AP"))
   {
    Serial.print("WIFI:AP-Mode ");
    varStore.varWIFI_s_Mode = "AP";

    WiFi.softAP("ESP_ABL_AP",varStore.varWIFI_s_Password.c_str());   
    Serial.print("IP Address: ");
    SYS_IP = WiFi.softAPIP().toString();
    Serial.println(SYS_IP);
    delay(1000); 
   }
   else
   {
    Serial.print("WIFI:STA-Mode\r\n");
    WiFi.mode(WIFI_STA);
  
    if (varStore.varWIFI_s_SSID.length() < 2)
    {
      varStore.varWIFI_s_SSID = "noSSID";
    }
    WiFi.setHostname(varStore.varDEVICE_s_Name.c_str());
  
    wl_status_t wifiStatus;
    wifiStatus = WiFi.begin(varStore.varWIFI_s_SSID.c_str(), varStore.varWIFI_s_Password.c_str());
    delay(500);
    int i = 0;
    Serial.printf("SSID:%s\r\n", varStore.varWIFI_s_SSID.c_str());
    ///debug_printf("Passwort:%s\r\n", varStore.varWIFI_s_Password);
    while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (i < 20))
    {
        Serial.print(".");
        setLED(i%2);
        i++;  
        delay(300);
    }
    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
      Serial.printf("Get WiFi-Power:%d\r\n",WiFi.getTxPower());
      Serial.printf("Get WiFi-RSSI:%d\r\n",WiFi.RSSI());
      
      Serial.printf("IP Address: ");
      SYS_IP = WiFi.localIP().toString();
      Serial.println(SYS_IP);
      return;
    }
    else
    {
      WiFi.disconnect(false, true);
      Serial.printf("\r\n******** no valid login to:'%s' *******************************\r\n", varStore.varWIFI_s_SSID.c_str());
      Serial.printf(    "******** connect to local AP:'ESP_ABL_AP' IP: 192.168.4.1 *****\r\n");
      varStore.varWIFI_s_Mode = "AP";
      WiFi.mode(WIFI_AP);
      WiFi.softAP("ESP_ABL_AP", NULL, 6, 0, 4,false);
      Serial.print("IP Address: ");
      SYS_IP = WiFi.softAPIP().toString();
      Serial.println(SYS_IP);
      return;
    }
   }

  return;
}

//////////////////////////////////////////
/// @brief Manage the Wifi Connection
/////////////////////////////////////////
void testWifiConnection() 
{
    if (varStore.varWIFI_s_Mode == "STA")
    {
      debug_printf("[WiFi] Mode:STA IP:%s RSSI:%d\r\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      if  (WiFi.status() != WL_CONNECTED)
      {
        // Force reconnect without freezing the main loop execution
        SYS_RestartCount++;
        hist.putInt("restart",SYS_RestartCount);
        WiFi.disconnect();
        WiFi.begin(varStore.varWIFI_s_SSID, varStore.varWIFI_s_Password);
      }
     
#ifdef PINGTEST
      // V2.1 new: optional extra test with ping to GatewayIP  
      if (Ping.ping(WiFi.gatewayIP(),1) > 0)
      {
       debug_printf("[PING ] response time : %d/%.2f/%d ms\r\n", Ping.minTime(), Ping.averageTime(), Ping.maxTime());
       AsyncWebLog.printf("[PING ] response: %d ms\r\n", Ping.minTime());
      } 
      else 
      {
       debug_println(" Ping Error !");
       SYS_RestartCount++;
       hist.putInt("restart",SYS_RestartCount);
       WiFi.disconnect();
       WiFi.begin(varStore.varWIFI_s_SSID, varStore.varWIFI_s_Password);
      }
#endif
    }   
    else
    {
      // remove debug log of Passwort if you dont want to show it in log !
      debug_printf("[WiFi] Mode:AP IP:%s Password:%s\r\n", WiFi.softAPIP().toString().c_str(), varStore.varWIFI_s_Password.c_str());
    }
    // Test if wifi is lost from router
 
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

int getIntStatus(uint_fast16_t wb_ix)
{
   long intValue = strtol(ABL_rx_status[wb_ix].c_str(), NULL, 16);
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
  if (var== "DEVICEID02")
  {
    return varStore.varDEVICE_s_Name02;
  }
  else
  if (var== "DEVICEID")
  {
    return varStore.varDEVICE_s_Name;
  }
  else
  if (var== "I-HIGH")
  {
    return "I-HIGH(" + String(varStore.varABL_i_A_soll_high) + "A)";
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
    if (ABL_Wh_Sum_akt[0] > 0)
    {
      f = ABL_Wh_Sum_akt[0] / 1000.00;
    }
    return String(f);
  } 
   if (var == "KWHSUM02")
  {
    float f = 0.00;
    if (ABL_Wh_Sum_akt[1] > 0)
    {
      f = ABL_Wh_Sum_akt[1] / 1000.00;
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
  
     return   "Version     :"   + SYS_Version + 
            "\nBuild       :" + SYS_CompileDate + " "+ SYS_CompileTime + 
            "\nPlatform    :" + ESP.getChipModel() +
            "\nTemp(C)     :" + temp +
            "\nIP-Addr     :" + SYS_IP +
            "\nRestart-Cnt :" + SYS_RestartCount + 
            "\nRSSI        :" + String(WiFi.RSSI()) + 
  #if WB_COUNT == 2          
            "\nLoadBal-Mode:" + String(varStore.varABL_i_LoadBal_mode) +
            "\nLoadBal-Isum:" + String(varStore.varABL_i_I_limit) +
            "\nPhase-Count :" + String(varStore.varABL_i_Phase_count) + 
            "\nPhase-Count2:" + String(varStore.varABL2_i_Phase_count) + 
        
            "\nTimeoutCnt01:" + ABL_rx_timeoutcount[0] +
            "\nTimeoutCnt02:" + ABL_rx_timeoutcount[1] + 
            "\nChargeCnt01 :" + SYS_ChargeCount[0] + 
            "\nChargeCnt02 :" + SYS_ChargeCount[1] + 
  
            "\nIpwm01      :" + String(ABL_rx_Ipwm[0]) + 
            "\nIpwm02      :" + String(ABL_rx_Ipwm[1]) + 
            "\nStatus1     :" + ABL_rx_status[0] +
            "\nStatus2     :" + ABL_rx_status[1];
  #else 
                    
            "\nPhase-Count :" + String(varStore.varABL_i_Phase_count) + 
            "\nTimeoutCnt  :" + ABL_rx_timeoutcount[0] +
            "\nChargeCnt   :" + SYS_ChargeCount[0] + 
            "\nIpwm        :" + String(ABL_rx_Ipwm[0]) + 
            "\nStatus      :" + ABL_rx_status[0];
  #endif

  }
  else
  if (var == "IMAX")
  {
     
     if (ABL_PauseFlag[0] == false)
     {
       ABL_tx_Icmax[0] = ABL_rx_Ipwm[0];
     }
     return String(ABL_tx_Icmax[0]);
  }
  else
  if (var == "IMAX2")
  {
     if (ABL_PauseFlag[1] == false)
     {
       ABL_tx_Icmax[1] = ABL_rx_Ipwm[1];
     }
     return String(ABL_tx_Icmax[1]);
    
  }
  else 
  if (var == "ILIMIT")
  { 
    uint16_t iLimit = varStore.varABL_i_I_limit;
    // german Par14a Limit 4.2kW
    int readLimit = digitalRead(PAR14LIMIT_GPIO);
    if (readLimit == 0)
    {
      if (varStore.varABL_i_Phase_count == 2)
      {
        iLimit = 9;
      }
      else
      if (varStore.varABL_i_Phase_count == 3)
      {
         iLimit = 6;
      }
    }
    return String(iLimit);
  }
  else 
  if (var == "UGRID")
  {
    return String(varStore.varABL_i_U_netz);
  }
  else 
  if (var == "NPHASE")
  {
    return String(varStore.varABL_i_Phase_count);
  }
  if (var == "NPHASE2")
  {
    return String(varStore.varABL2_i_Phase_count);
  }
   else 
  if (var == "PMAX")
  {
     if (ABL_PauseFlag[0] == false)
     {
       ABL_tx_Icmax[0] = ABL_rx_Ipwm[0];
     }
    return String(round((varStore.varABL_i_Phase_count * varStore.varABL_i_U_netz * ABL_tx_Icmax[0])/1000));
  }
  if (var == "PMAX2")
  {
     if (ABL_PauseFlag[1] == false)
     {
       ABL_tx_Icmax[1] = ABL_rx_Ipwm[1];
     }
    return String(round((varStore.varABL2_i_Phase_count * varStore.varABL_i_U_netz * ABL_tx_Icmax[1])/1000));
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
   uint8_t wb_ix_local = 0;
   //String s  = request->arg(i);
   //debug_println(s);
   //String sRet = "";
   if (request->argName(0) == "ihigh")
   {
     //ABL_Send("high");
     //ABL_tx_Icmax[0] = varStore.varABL_i_A_soll_high;
     setIcmax(0, varStore.varABL_i_A_soll_high);
     AsyncWebLog.println("SET-Current WB1:" + String(ABL_tx_Icmax[0]) + "A");
   }
   else
   if (request->argName(0) == "ilow")
   {
       //sRet = ABL_Send("low");
       //ABL_tx_Icmax[0] = varStore.varABL_i_A_soll_low;
       setIcmax(0, varStore.varABL_i_A_soll_low);
       AsyncWebLog.println("SET-Current WB1:" + String(ABL_tx_Icmax[0]) + "A");
   }
   else
   if (request->argName(0) == "pause")
   {
       //sRet = ABL_Send("low");
       //ABL_tx_Icmax[0] = 0;
       setIcmax(0, 0);    
       AsyncWebLog.println("CHARGE-PAUSE WB1");
    }

  // 2. Wallbox
   if (request->argName(0) == "ihigh2")
   {
     //ABL_Send("high");
     wb_ix_local = 1;
     //ABL_tx_Icmax[1] = varStore.varABL_i_A_soll_high;
     setIcmax(1, varStore.varABL_i_A_soll_high);
     AsyncWebLog.println("SET-Current WB2:" + String(ABL_tx_Icmax[1]) + "A");
   }
   else
   if (request->argName(0) == "ilow2")
   {
       //sRet = ABL_Send("low");
       wb_ix_local = 1;
       //ABL_tx_Icmax[1] = (int)varStore.varABL_i_A_soll_low;
       setIcmax(1, varStore.varABL_i_A_soll_low);
       AsyncWebLog.println("SET-Current WB2:" + String(ABL_tx_Icmax[1]) + "A");
   }
   else
   if (request->argName(0) == "pause2")
   {
       //sRet = ABL_Send("low");
       wb_ix_local = 1;
       setIcmax(1, 0);
       AsyncWebLog.println("CHARGE-PAUSE WB2");
    }
   
   if (wb_ix_local == 0)
   { 
#if WB_COUNT == 2
     request->send(SPIFFS, "/index1.html", String(), false, setHtmlVar);
#else
    request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
#endif
   }
   else
   {
    request->send(SPIFFS, "/index2.html", String(), false, setHtmlVar);
   }
}

void initWebServer()
{ 
  debug_print("Init web server...\n");
    //Route for root / web page
  server.on("/",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
    if (bInitFileOK) 
    { 
#if WB_COUNT == 2
     request->send(SPIFFS, "/index1.html", String(), false, setHtmlVar);
#else
    request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
#endif
    }
    else
    { // "rescue page" for first run without valid Wifi-credentials or no valid data-system
      request->redirect("/ota_ap.html");
      //request->send(202, "text/plain", "goto: http://192.168.4.1/ota_ap.html");
    }
  });

   
  // Notfall Website 'ota_ap.html' für Software-Upload
  static const char* staticOTA_site PROGMEM = R"(
<!DOCTYPE HTML><html lang="de"><head>
<title>OTA-Update</title><meta charset="UTF-8"></head>
<body><form method='POST' action='/ota_update' enctype='multipart/form-data'></p><div>Select a file 'myFS.bin' (myFS-Data) or 'firmware.bin' (Program)</div><input class='container' type='file' name='update' accept='.bin'><input type='submit' name="startupdate" value='START-UPLOAD'><BR></form>
</p><a class="buttonlink" href="reboot.html">REBOOT!</a>
</body></html>
)";
  server.on("/ota_ap.html", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(200, "text/html", staticOTA_site);
  });


  //Route for root /index web page
  server.on("/index.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   #if WB_COUNT == 2
     request->send(SPIFFS, "/index1.html", String(), false, setHtmlVar);
#else
    request->send(SPIFFS, "/index.html", String(), false, setHtmlVar);
#endif
  });

  
  //Route for root /index web page
  server.on("/index2.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/index2.html", String(), false, setHtmlVar);
  });


  // > Version V1.2
  //Route for stored values
  server.on("/setpower.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/setpower.html", String(), false, setHtmlVar);
  });

  server.on("/setpower2.html",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/setpower2.html", String(), false, setHtmlVar);
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
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/style.css", String(), false);
  });


  // fetch GET
  server.on("/fetch", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    
    // fetch is also used for setting Imax with URL-Variable
    // example: 
    // http://<your-ip>/fetch?imax=8 --> set Imax to 8A
    // http://<your-ip>/fetch?pmax=3000 --> set Pmax to 3kW
    //
    // valid values: imax=0...6-16 
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
        uint a = String(request->arg(i)).toInt();
        AsyncWebLog.println("SET-Imax WB01:" + String(a)+ "A");
        if ((a >=6 && a <=16) || (a==0))
        {
            setIcmax(0,a);
        }
        else
        {
         AsyncWebLog.println("[fetch] **ERROR Imax WB1 out of range!");
        }
     }
     else if (request->argName(0) == "pmax")
     {
       uint a = String(request->arg(i)).toInt();
             debug_printf("[fetch] SET-Pmax WB1: %dWatt\r\n",a);
       AsyncWebLog.printf("[fetch] SET-Pmax WB1: %dWatt\r\n",a);
       setPmax(0,a);
       ABL_tx_status[0] = SET_Current; // next Send Protocol
     }
    }
    
    String sStatus;
    if (ABL_PauseFlag[0])
    {
      sStatus = ABL_STATUS_STRING[ABL_PAUSE];
    }
    sStatus += ABL_rx_status[0];
    // return actual values
    // REMARK: if you change Imax it needs one more GET to return the actual value of Imax
    String s = String(ABL_rx_Ipwm[0])+',' + String(ABL_rx_kW[0]) + ','+ sStatus+ ',' + String(ABL_rx_Wh[0]/1000.0) + ',' + String(ABL_Wh_Sum_akt[0]) + ',' +String(ABL_sChargeTime[0]);
    request->send(200, "text/plain", s);
    //debug_println("server.on /fetch: "+ s);

});

  // fetch GET
  server.on("/fetch2", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    
    // fetch is also used for setting Imax with URL-Variable
    // example: 
    // http://<your-ip>/fetch?imax=8 --> set Imax to 8A
    //
    // valid values: imax=0..6-16 
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
       uint a = String(request->arg(i)).toInt();
       AsyncWebLog.println("SET-Imax WB2:" + String(a)+ "A");
       if ((a >=6 && a <=16) || (a==0))
       {
           setIcmax(1,a);
       }
       else
       {
        AsyncWebLog.println("**ERROR Imax WB2 out of range!");
       }
     }
     else if (request->argName(0) == "pmax")
     {
       uint a = String(request->arg(i)).toInt();
             debug_printf("SET-Pmax WB2: %dWatt\r\n",a);
       AsyncWebLog.printf("SET-Pmax WB2: %dWatt\r\n",a);
       setPmax(1,a);
     }

    }
    
    String sStatus;
    if (ABL_PauseFlag[1])
    {
      sStatus = ABL_STATUS_STRING[ABL_PAUSE];
    }
    sStatus += ABL_rx_status[1];
    // return actual values
    // REMARK: if you change Imax it needs one more GET to return the actual value of Imax
    String s = String(ABL_rx_Ipwm[1])+',' + String(ABL_rx_kW[1]) + ','+ sStatus + ',' + String((ABL_rx_Wh[1])/1000.0) + ',' + String(ABL_Wh_Sum_akt[1]) + ',' +String(ABL_sChargeTime[1]);
    debug_println("server.on /fetch2: "+ s);
    request->send(200, "text/plain", s);
  }
);
 
  // fetch GET
  server.on("/fetchjson", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    // z.Z. nur für Wallbox-01
    const uint_fast16_t wb_ix = 0;
    // return actual values
    // REMARK: if you change Imax it needs one more GET to return the actual value of Imax
    String s = "{\"ipwm\": "      + String(ABL_rx_Ipwm[wb_ix])+ ", "+ 
                 "\"kw\": "       + String(ABL_rx_kW[wb_ix])  + ", "+ 
                 "\"status\": \"" + ABL_rx_status[wb_ix]     + "\", "+ 
                 "\"wh\": "       + String(ABL_rx_Wh[wb_ix]/1000.0) + ", " + 
                 "\"whsum\": "    + String(ABL_Wh_Sum_akt[wb_ix])   + ", " + 
                 "\"ctime\": \""  + String(ABL_sChargeTime[wb_ix]) + "\"}\r\n";
    request->send(200, "text/json", s);
    //debug_println("server.on /fetch: "+ s);
  });

  // fetch GET
  server.on("/fetchkv", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    String s = "Imax=" + String(ABL_rx_Ipwm[0]) + " A \n" + 
               "ActPower=" + String(ABL_rx_kW[0]) + " kW\n" +
               "Status=" + String(ABL_rx_status[0]) + "\n" +
               "IntStatus=" + String(getIntStatus(0)) + "\n" +
               "ActWork=" + String((ABL_rx_Wh[0])/1000.0) + " kW/h\n" +
               "SumWork=" + String(ABL_Wh_Sum_akt[0]) + " W/h\n" +
               "ChargeTime=" + String(ABL_sChargeTime[0]) + "\n";
    request->send(200, "text/plain", s);
  });

  // fetch GET
  server.on("/fetchkv2", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    // Wallbox-02
    String s = "Imax=" + String(ABL_rx_Ipwm[1]) + " A \n" + 
               "ActPower=" + String(ABL_rx_kW[1]) + " kW\n" +
               "Status=" + String(ABL_rx_status[1]) + "\n" +
               "IntStatus=" + String(getIntStatus(1)) + "\n" +
               "ActWork=" + String((ABL_rx_Wh[1])/1000.0) + " kW/h\n" +
               "SumWork=" + String(ABL_Wh_Sum_akt[1]) + " W/h\n" +
               "ChargeTime=" + String(ABL_sChargeTime[1]) + "\n";
    request->send(200, "text/plain", s);
  });
 
  // config.txt GET
  server.on("/config.txt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/config.txt", "text/html", false);
  });

  // config.txt GET
  server.on("/reboot.html", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    #if WB_COUNT == 2
      request->send(200, "text/html", "<a href='/index1.html'>START</a>");
#else
     request->send(200, "text/html", "<a href='/index.html'>START</a>");
#endif
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
   server.on("/wb.png",          HTTP_GET, [](AsyncWebServerRequest *request)
  {
   request->send(SPIFFS, "/wb.png", String(), false);
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

  // index1.html POST
  server.on("/index1.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    Handle_Index_Post(request);
  }); 

  server.on("/index2.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    Handle_Index_Post(request);
  });
  
  // > Version V1.2 set current in extra page
  // index.html POST
  server.on("/setpower.html",          HTTP_POST, [](AsyncWebServerRequest *request)
  {
    Handle_Index_Post(request);
  });
  
   // > Version V1.2 set current in extra page
  // index.html POST
  server.on("/setpower2.html",          HTTP_POST, [](AsyncWebServerRequest *request)
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
   if (request->argName(0) == "kwh") // 01 kW/h Sum
   {
      ABL_Wh_Sum_akt[0] = u_long(s.toFloat()*1000);
      debug_print("ABL_Wh_Sum_akt: ");
      debug_println(ABL_Wh_Sum_akt[0]);
      set_Wh_Sum(0,ABL_Wh_Sum_akt[0]);
      ABL_Wh_Sum_old[0] = ABL_Wh_Sum_akt[0];
      saveHistory();
   } 
   else
   if (request->argName(0) == "kwh02") // 02 kW/h Sum
   {
      ABL_Wh_Sum_akt[1] = u_long(s.toFloat()*1000);
      debug_print("ABL_Wh_Sum_akt: ");
      debug_println(ABL_Wh_Sum_akt[1]);
      set_Wh_Sum(1,ABL_Wh_Sum_akt[1]);
      ABL_Wh_Sum_old[1] = ABL_Wh_Sum_akt[1];
      saveHistory();
   } 
      
   request->send(SPIFFS, "/setvalues.html", String(), false, setHtmlVar);
  });

  
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

/// @brief  only used in main-loop
void inline set_tmp_poll_time_ms()
{
  
#if WB_COUNT == 2
    if ((ABL_rx_status[0].indexOf('A') >= 0) && (ABL_rx_status[1].indexOf('A') >= 0))
#else 
    if ((ABL_rx_status[0].indexOf('A') >= 0))
#endif
    { 
      tmp_poll_time_ms = varStore.varABL_i_Scantime_ms;} // value > 30sec reduces standby power 
    else
    {
       tmp_poll_time_ms = POLLTIME_ACTIV;
    }

    if (ABL_forcePollFlag[0])
    {
      tmp_poll_time_ms = POLLTIME_FAST;
    }
#if WB_COUNT == 2
    else
    if (ABL_forcePollFlag[1])
    {
      tmp_poll_time_ms = POLLTIME_FAST;
    }
#endif
   
     
}


// ------------------((ABL_rx_status.startsWith("C")) && --------------------------
void setup()
{
  Serial.begin(115200);
  delay(2000); // Zeit für USB-CDC Initialisierung beim Booten
  Serial.println("*** ABL: Setup-Start ***");
  initSPIFFS();
#ifndef WITHOUT_LED
  initLED();
#endif

  pinMode(PAR14LIMIT_GPIO, INPUT_PULLUP); // new: 4.2kW limit (german Par.14a)

  bInitFileOK = initFileVarStore();
  initHistory();
  ABL_init();

  #ifdef USE_ETH_INSTEAD_WIFI
  initEthernet();
#else
  initWifi(!bInitFileOK);
  delay(1000);
#endif
#ifdef MQTT_ENABLE
  mqtt_setup();
#endif  
  initWebServer();
  delay(1000);
  rtc[0].setTime(0);
  rtc[1].setTime(0);

  delay(1000);
  Serial.println("*** ABL: Setup End ***");
  testPar14aLimit(0);
}


String lastReceivedData = "--";
uint last_wb_poll_ix = 0;
int main_iBlink;
void loop()
{
    set_tmp_poll_time_ms();

    if (millis() > (ABL_PollTime_old + tmp_poll_time_ms))
    {
      AsyncWebLog.printf("[Poll] Time:%dms\r\n", tmp_poll_time_ms);
  #if WB_COUNT == 2
      if (last_wb_poll_ix >= WB_COUNT-1)
      {last_wb_poll_ix = 0;}
      else
      {last_wb_poll_ix++;}

      testLoadBal();
  #else
      last_wb_poll_ix = 0;
  #endif

      testPar14aLimit(last_wb_poll_ix);

      ABL_PollTime_old  = millis();
      log_timer = tmp_poll_time_ms / 1000;
      ABL_Send(last_wb_poll_ix,ABL_tx_status[last_wb_poll_ix]);
      testTimeount(last_wb_poll_ix); // Test Rx Timeout to ABL-Wallbox
      // Test Network connection
#ifdef USE_ETH_INSTEAD_WIFI
      testEthernetConnection();
#else
      testWifiConnection();
#endif
    }
    else
    if (millis() - varStore.varABL_i_logtime_ms > ABL_StatusSec_old)
    { 
      //calculate_kWh(last_wb_poll_ix);
      setLED(main_iBlink%2);
      main_iBlink++;
      ABL_StatusSec_old = millis();

      for (size_t i = 0; i < WB_COUNT; i++)
      {
        calculate_kWh(i);
      }

      if (log_timer > 0)
      {log_timer--;}
      else
      {log_timer = tmp_poll_time_ms / 1000;}
#if WB_COUNT == 2         
      uint8_t ii= 0;
      if (last_wb_poll_ix == 0)
      {ii=1;}
      AsyncWebLog.printf("[Poll]WB%d next Tx sec:%d\r\n",ii+1, log_timer);
            debug_printf("[Poll]WB%d next Tx sec:%d\r\n",ii, log_timer);
#else
       AsyncWebLog.printf("[Poll] next Tx sec:%d\r\n",log_timer);
             debug_printf("[Poll] next Tx sec:%d\r\n",log_timer);
#endif
    }
    else
    {
       // old:
       //serialEventABL();
      SerialMessage receivedMsg;
      // Prüfen (non-blocking, 0ms), ob Daten in der Queue liegen
      if (xQueueReceive(msgQueue, &receivedMsg, 0) == pdTRUE) 
      {
        lastReceivedData = String(receivedMsg.data);
        xQueueReset(msgQueue);
        if (ABL_ParseReceive(lastReceivedData));
        {
          if (lastReceivedData.startsWith("01",1))
           {ABL_rx_timeoutcount[0] = 0;}
          else if (lastReceivedData.startsWith("02",1))
           {ABL_rx_timeoutcount[1] = 0;}
        }
        Serial.print("RX");
        Serial.println(lastReceivedData);
      }
    }
#ifdef MQTT_ENABLE
    mqtt_loop();
#endif  
   delay(5);
  
}
