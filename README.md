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
- INFO: Version, Build, Temp(ESP-intern), IP, Timeout, Charge-Cnt, RSSI


Monitor and control your ABL-Wallbox with an WEB-Application and integrate it (optional) in your homeautomation software with simple REST-Interface (see example for DOMOTICZ below) for less than 10€.

## 'Virtual' consumption for ABL-Boxes without phase current sensor

For (new) ABL-Boxes without internal per phase current sensor, the current could "pre-measured" external and defined in the APP-configuration per Imax setting. So it is possible to have a "poor man's" power and consumption measurement. If you charge every time the same car, the typical power for a defined Ipwm is near to the real measurement with a current-sensor, but with current sensor there is also an error, because the voltage 'U' for power calulation (P=U*(I1+I2+I2) is not measured. 

I get the power from my "offical" smartmeter and set them into the config-file.

If the Box has a current-sensor it was automatic detected and the value is calculated based on the current sum of the 3-phases...For future versions it would be possible to expand the software to get the power or current over the WEB-API from an external meter.

## Manual Charge-Current-Setting

The user interface was designed very simply to enable manual switching between two charging currents (possible application: ABL-Wallbox-kWh Gesamt
ABL-Wallbox-kWh Gesamtreduced charging current for operation with a PV system).
The two values could be individual defined in the 'config-data' page (for external setting use the WEB-API)

## External Charge-Current-Setting and reading values (WEB-API) 

With the WEB-API the state and consumption values of the Wallbox could be monitored and charge-current could be set from 6A up to 16A in external applications like homeautomation software.

### Fetch aktual Values and State

`http:<your-ip>/fetch`

Receive:
`8.00,0.00,A1,0.00,103980,00:00:00`

decoded:
`<Imax [A]>,<aktual Power [kW]>,<Status>,<aktual Work [kW/h]>,<Sum Work [W/h]>,<charge-time>`

### Fetch actual Values and State in key=value pair

This includes the "Status" as an Integer Value, too.

`http:<your-ip>/fetchkv`

Receive:
```
Imax=16 A 
ActPower=11 kW
Status=C2
IntStatus=194
ActWork=0.92 kW/h
SumWork=2926 W/h
ChargeTime=00:15:02
```

### Set value Imax

`http:<your-ip>/fetch?imax=xx` (xx= 6,8,10,12,14,16)

## Home-Assistent Integration
...see above 'WEB-API' for other external integration (e.g. other homeautomation software)

![HOMEASSI](/pict/homeassi.png)

add to **configuration.yaml**:

```YAML
rest:
    scan_interval: 20
    resource: http://192.168.2.108/fetch
    sensor:
      - name: "ABL-Wallbox Imax"
        unique_id : "sensor_abl_imax"
        icon: "mdi:ev-station"
        value_template: '{{value.split(",")[0]}}'
        unit_of_measurement: "A"
        device_class: current

      - name: "ABL-Wallbox kw"
        unique_id : "sensor_abl_kw"
        icon: "mdi:ev-station"
        value_template: '{{value.split(",")[1]}}'
        unit_of_measurement: "kW"
        device_class: power

      - name: "ABL-Wallbox-Status"
        unique_id : "sensor_abl_status"
        icon: "mdi:ev-station"
        value_template: '{{value.split(",")[2]}}'
      
      - name: "ABL-Wallbox-kWh akt."
        unique_id : "sensor_abl_kwhakt"
        icon: "mdi:ev-station"
        value_template: '{{float(value.split(",")[3])/1000}}'
        unit_of_measurement: kWh
        device_class: energy
     
      - name: "ABL-Wallbox-kWh Gesamt"
        unique_id : "sensor_abl_kwhsum"
        icon: "mdi:ev-station"
        value_template: '{{float(value.split(",")[4])/1000}}'
        unit_of_measurement: kWh
        device_class: energy
        state_class: total

rest_command:
  abl_set_imax_6a:
    url: "http://192.168.2.108/fetch?imax=6"
    method: GET
  abl_set_imax_16a:
    url: "http://192.168.2.108/fetch?imax=16"
    method: GET
```

## Domoticz Integration

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

V1.2.2 Bugfix for 'virtual kW/h-calculation' and Modbus timeouts

## Ideas:
- MQTT-client 
- get actual Power [kW] and Work [kW/h] from external meter
- get actual current from current sensor and calulate Power and Work
- store more history data (last charging data)
- get history-time from ntp (could work only in 'STA' network mode)


## other extentions:
- add Ethernet port to esp32
https://www.roboter-bausatz.de/p/wt32-eth01-esp32-modul-mit-ethernet-bluetooth-wifi
https://mischianti.org/esp32-ethernet-w5500-with-plain-http-and-ssl-https/

