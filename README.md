# ABL-Wallbox WebControl

[![License](https://img.shields.io/badge/license-EUPL1.2-green)](https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)

![Android-App](/pict/Screenshot_menu_android.png)

Monitor and control your ABL-Wallbox with an WEB-Application and integrate it in your homeautomation software with simple REST-Interface (see example for DOMOTICZ below) with less than 10€. 

The user interface was designed very simply to enable switching between two charging currents (possible application: reduced charging current for operation with a PV system)

With the WEB-API the state and consumption values of the Wallbox could be monitored and charge-current could be set from external applications like homeautomation software.

Additional functions (Setup) in Web-Interface

* CONFIG-DATA: set and store parameter and WiFi-credentials
* EVENT-LOG: Serial debug logging
* OTA-UPDATE: Software update (Over the air update)
* HISTORY: Set and Store total kW/h sum in internal FLASH

### WEB-API for external Software

#### Fetch aktual Values and State

**GET-Values:**
`http:<your-ip>/fetch`

Receive:
`8.00,0.00,A1,0.00,103980`

decoded:
`<Imax [A]>,<aktual Power [kW]>,<Status>,<aktual Work [kW/h]>,<Sum Work [kW/h]>`

**Set value Imax:**
`http:<your-ip>/fetch?imax=xx` (xx= 6,8,10,12,14,16)

#### Domoticz Integration
...see above 'WEB-API' for other external integration (e.g. other homeautomation software)

![Domoticz](/pict/domoticz.png)

Example LUA polling script for Domoticz (abl_polling.lua)

```LUA
-- parser handling data with the following format
-- A test with curl would be : curl -X POST -d "28,48,2" 'http://192.168.1.17:8080/json.htm?type=command&param=udevices&script=example.lua'

-- This function split a string according to a defined separator
-- Entries are returned into an associative array with key values 1,2,3,4,5,6...
local function split(str, sep)
 if sep == nil then
  sep = "%s"
 end
 local t={} ; i=1
 local regex = string.format("([^%s]+)", sep)
  for str in str:gmatch(regex) do
   t[i] = str
   i = i + 1
  end
 return t
end

-- Retrieve the request content
s = request['content'];

print("ABL-Wallbox-Poller:"..s);
-- Split the content into an array of values
local values = split(s, ",");

-- Update devices 
-- ACHTUNG: Index beginnt in LUA mit 1

--                    xx  mit aktueller IDX anpassen
--                    vv
domoticz_updateDevice(63,'',values[1]) -- Imax
domoticz_updateDevice(65,'',tonumber(values[2])*1000) -- Watt !!
domoticz_updateDevice(62,'ABL-Status',values[3]) -- Status-String
domoticz_updateDevice(66,'',values[4]) -- kW/h
domoticz_updateDevice(67,'',values[5]) -- kW/h SUM
```

#### Hardware

This example is realised with an 'ESP32-S2 mini' board and an 'MAX485 TTL to RS485' Converter. 
...but any other ESP-Board could be used with small modifications in the code and platformio.ini.

![Fritzing](/pict/esp32_abl_fritzing.png) ![prototype](/pict/platine_prototype.jpg) 
(board is cut to fit in a DIN hat rail case)

## Helpful Infos and links and investigations (most in german)

#### Software

my shared investigations and notes:
https://github/raibisch/doc/knowhow.txt

...others sources
https://github.com/ThKls/Wallbox-Test

https://www.goingelectric.de/forum/viewtopic.php?f=34&t=38749&start=60

https://www.goingelectric.de/forum/viewtopic.php?p=1550459#

#### Hardware

![prototype](/pict/ABL_Modbus_Connector.jpg)
connect RS485 to Pin1 (=A) and Pin2 (=B) of left RJ45-Connector in Wallbox
the termination Jumper may be optional (worked for me with and without)
...more infos:
https://github.com/evcc-io/evcc/discussions/2801

## Licence

[Licensed under the European Union Public License (EUPL)-1.2-or-later](https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)

[Why licensed under EUPL-1.2: it is compatible to GPL and compatible to EU-rights and regulations](https://joinup.ec.europa.eu/collection/eupl/join-eupl-licensing-community)

## Download and Installation

* To download click the DOWNLOAD ZIP button, rename the uncompressed folder 'ESP32_ABL'
...or clone it with git 

#### PlatformIO and Arduino-IDE

Projekt was build and testet with PlatformIO.

Take care to upload the 'data' folder to the SPIFFS filesystem 
see: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/
