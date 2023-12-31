# ABL-Wallbox Web-App

[![License](https://img.shields.io/badge/license-EUPL1.2-green)](https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12)

![Android-App](/pict/ABL_webcont_charging.png)

![Android-App](/pict/ABL_webcont_b2.png) ![Android-App](/pict/ABL_webcont_setIpwm.png) ![Android-App](/pict/ABL_webcont_eventlog.png) ![Android-App](/pict/ABL_webcont_update.png) 

## Supported Hardware

- eMH1 11kW ( old version) --> Power calculation with internal current values
- eMH1 11kW ( > Q2-2021 no internal current sensor)  --> 'virtual' Power calculation
- eMH1 22kW --> Power calculation with internal current values
- eMH2 22kW (StandAlone Modus) --> Power calculation with internal current values

## Functions in Web-Interface

- INDEX-Page: display wallbox-status and actual parameter (I-max, kw-charge, kW/h, charging-time)
- SET-Current: "Quick-set" predefined Ipwm values to ABL-Box
- CONFIG-DATA: set and store parameter and WiFi-credentials and the the "Quick-Set" charge-current
- EVENT-LOG: Serial debug logging
- UPDATE: over-the-air (Wifi) Software update
- HISTORY: Set and Store total kW/h sum in internal FLASH

Monitor and control your ABL-Wallbox with an WEB-Application and integrate it (optional) in your homeautomation software with simple REST-Interface (see example for DOMOTICZ below) for less than 10€.

## 'Virtual' consumption for ABL-Boxes without phase current sensor

For (new) ABL-Boxes without internal per phase current sensor, the current could "pre-measured" external and defined in the APP-configuration per Imax setting. So it is possible to have a "poor man's" power and consumption measurement. If you charge every time the same car, the typical power for a defined Ipwm is near to the real measurement with a current-sensor, but with current sensor there is also an error, because the voltage 'U' for power calulation (P=U*(I1+I2+I2) is not measured. 

I get the power from my "offical" smartmeter and set them into the config-file.

If the Box has a current-sensor it was automatic detected and the value is calculated based on the current sum of the 3-phases...For future versions it would be possible to expand the software to get the power or current over the WEB-API from an external meter.

## Manual Charge-Current-Setting

The user interface was designed very simply to enable manual switching between two charging currents (possible application: reduced charging current for operation with a PV system).
The two values could be individual defined in the 'config-data' page (for external setting use the WEB-API)

## External Charge-Current-Setting and reading values (WEB-API) 

With the WEB-API the state and consumption values of the Wallbox could be monitored and charge-current could be set from 6A up to 16A in external applications like homeautomation software.

### Fetch aktual Values and State

`http:<your-ip>/fetch`

Receive:
`8.00,0.00,A1,0.00,103980,00:00:00`

decoded:
`<Imax [A]>,<aktual Power [kW]>,<Status>,<aktual Work [kW/h]>,<Sum Work [W/h]>,<charge-time>`

### Set value Imax

`http:<your-ip>/fetch?imax=xx` (xx= 6,8,10,12,14,16)

### Domoticz Integration

...see above 'WEB-API' for other external integration (e.g. other homeautomation software)

![Domoticz](/pict/domoticz.png)

#### Domoticz Configuration
![Domoticzpoller](/pict/domoticz_poller.png)

LUA polling script for Domoticz (ablpoller.lua)

```LUA
-- Polling of ABL-Web-App
--
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
domoticz_updateDevice(67,'',values[5]) -- W/h SUM
```

## Hardware

This example is realised with an 'ESP32-S2 mini' board and an 'MAX485 TTL to RS485' Converter. 
...but any other ESP-Board could be used with small modifications in the code and platformio.ini.

![Fritzing](/pict/esp32_abl_fritzing.png) ![prototype](/pict/platine_prototype.jpg) 
(board is cut to fit in a DIN hat rail case)

## Helpful Infos and links and investigations (most in german)

### Software

my shared investigations and notes:
[my-doc](/doc/knowhow.txt)

...others sources
https://github.com/ThKls/Wallbox-Test

https://www.goingelectric.de/forum/viewtopic.php?f=34&t=38749&start=60

https://www.goingelectric.de/forum/viewtopic.php?p=1550459#

### Hardware

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

### PlatformIO and Arduino-IDE

Projekt was build and testet with PlatformIO.

Take care to upload the 'data' folder to the SPIFFS filesystem 
see: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/

### Flash Program (without PlatformIO or Arduino-IDE)

If you do not want to compile the program from source:

for the ESP32-S2 mini board I supply the actual firmware-version
- got to subfolder `firmware/lolin_s2_mini/V1_2`

- put ESP32-S2 board to flash-mode: 
- disconnect USB then press and hold "0"-button
- reconnect USB hold "0"-button
- then release "0"-button
--> now the board is in program-mode

- run `flash.sh` (for Linux) or `flash.bat` (for Windows)
(needs 'esptool.py') --> ask google

- after flashing reset or disconnect USB
- search WIFI connetions for "ABLWALLBOX"
- connect (without password) 
- start your webbrowser at "192.168.4.1" (this is the startpage for the APP)
  to connect to your home-route navigate to "Setup" --> "Config-Data" and change:

```config
varWIFI_s_Mode=STA; 
varWIFI_s_Password=mypassword;
varWIFI_s_SSID=myrouter;
```

## Version History
V1.0 initial version, first test with real charging car (VW ID.3) 

V1.1 ABL-Box without internal current-sensor: calculate power and consumption from premeasured config-values. 

V 1.2 index-page redesign, fixes for kW/h-calculation, fetch kW/h sum, Info-page, display chargetime, bugfixes.

## todo, ideas
- MQTT-client (does someone need this ?)
- get actual Power [kW] and Work [kW/h] from external meter
- add connection to Tibber API
