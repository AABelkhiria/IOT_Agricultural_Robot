# IOT_Agricultural_Robot

## Introduction IOT
The internet of things (IoT) is a concept that defines the possibility of internet connected objects and the interaction between them.

This project is about gathering humidity, temperature, occupancy and light intensity data from sensors connected to NodeMcu device.
The data gathered from the device is transferred to an online MQTT broker.

## Nodemcu programming

#### NodeMcu

NodeMcu is an open-source firmware and development kit.
NodeMcu is low cost and WI-FI enabled (using ESP8266) , it integrates GPIO, PWM, IIC, 1-wire and ADC.

The NodeMcu board is programmed with Arduino framework using PlatformIO IDE.

#### Sensors
Building this IoT node required 3 types of sensors:

* DHT11 humidity and temperature sensor.
* Photo-resistor as a light intensity sensor.
* Button as an occupancy sensor.

After gathering all the sensors, we connect them to the board’s pins which number is sufficient for operating all the sensors mentioned above.

#### PlatformIO IDE
Platformio IDE is an open-source integrated development environment for IoT.
It supports more than 400 embedded boards , 20 development platforms and 10 frameworks.
In this project we will be using PlatformIO with Atom which is Github’s text editor.

#### Data gathering

##### Humidity and Temperature

```cpp
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN    2       
#define DHTTYPE   DHT11   

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

void setup()
{
  Serial.begin(9600); 
  dht.begin();
  sensor_t sensor;
  delayMS = sensor.min_delay / 1000;
}

void loop()
{
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature))
    Serial.println("Error reading temperature!");
  else
  {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println(" *C");
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity))
    Serial.println("Error reading humidity!");
  else
  {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  }
}
```

##### Occupancy

```cpp
#include <ESP8266WiFi.h>

bool a = false;  
int switchPin = 14; 

void occupancy()
{
  a = !a ;
}

void setup(
{
  Serial.begin();
}

void loop()
{
  if (digitalRead(switchPin))
  {
    occupancy();
    Serial.print(a)
  };
}
```

##### Brightness

```cpp
#include <ESP8266WiFi.h>

void setup(){}
    
int light()
{
  int sensorValue = analogRead(A0);
  return sensorValue;
}

void loop()
{
  Serial.print(light());
}
```
        
## OTA compatibility

Over-the-air (OTA) programming is used in this project to update the firmware of the device.
This method is very useful in case of limited or no physical access to the device.

OTA may be done using:
* through the IDE
* Web browser
* HTTP Server

The OTA method I chose for this project is via Web browser , I think that this method is user-friendly.

```cpp
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

const char* host     = "esp8266-webupdate";
const char* ssid     = "........";
const char* password = "........";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void setup(void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting Sketch...");
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    Serial.println("WiFi failed, retrying.");
  }

  MDNS.begin(host);

  httpUpdater.setup(&httpServer);
  httpServer.begin();

  MDNS.addService("http", "tcp", 80);
  Serial.printf("Open http://esp8266-webupdate.local/update \n");
}

void loop(void)
{
  httpServer.handleClient();
}
```

After uploading this firmware with USB serial , the device is going to connect to the WIFI hotspot and boot sketch , after that it is possible to update the firmware through [this URL](http://esp8266-webupdate.local/update) .

## Data transferring

The data gathered from sensors is transferred with MQTT (Message Queuing Telemetry Transport) which is a publish-subscribe messaging protocol.

Nodemcu is compatible with 3.1.1 version of MQTT.

The messages published will be json objects where data values , time (UTC) and device name will be published. I used [HiveMQ](http://www.hivemq.com/demos/websocket-client/) as  public MQTT broker.

## Stresstesting NodeMcu

### Limit Bauderate for communication

Bauderate is bits/sec , in serial communication we send 10 bits words at a time.
After testing different bauderates for the communication we can verify that the maximum bauderate regarding nodemcu is 9600 when using 10 ms delay (960 characters/s), 115200 when using 1 ms (11520 characters/s) delay and 250000 when using $40 \mu s$ delay (25000 characters/s).

When trying to send faster or more characters , the serial buffer will get filled as it can hold up to 63 characters.

### Realibility of OTA

Over the air update uses 50% of the space available in NodeMcu as it needs to have the old firmware available in the memory for a future update.
NodeMcu maximum program storage space is 1MB ( with 3MB SPIFFS ), so we will be able only to use 50% if we proceed to OTA.

Some problems are encountered when using sketches of 49% or 50% , it’s preferable if the sketch doesn’t exceed 48% of maximum program size when using OTA.

## Conclusion

In this project we created an IOT node that can read tempeture , humidity , occupancy and light intensity values using NodeMcu device and multiple sensors.

The device NodeMcu is enabled for OTA programming through web server which simplifies the update of the firmware.

The data were transferred to the database via Json over MQTT using **Hivemq** public broker where a python script subscribes to the topics.

## Perspectives

* Adding more sensors to the node : wind speed, air quality, sound level..

* Creating a mother board containing the microprocessor and all the sensors.

* Creating a protecting package for the node.

* Using a private MQTT broker and securing the data.

* Using different technology (ie. LORA )
