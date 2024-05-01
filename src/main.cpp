#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "config.h"
#include "AutoGrowBufferStream.h"
#include "MqttParsingUtils.h"

WiFiClientSecure wifiSecure;
PubSubClient mqttClient(wifiSecure);
IPAddress ipAddress(0, 0, 0, 0);
AutoGrowBufferStream stream;

// Wifi
int connectionAttempts = 1;
int wifimode = 0;
char ssid[32] = "xxxxxxxxxx";
char password[64] = "xxxxxxxxxx";

// Printer
char printerIP[64] = "xx.xx.x.xx";
char topic[64] = "device/+/report";
char mqtt_username[32] = "bblp";
char accessCode[9] = "xxxxx";
char serialNumber[16] = "xxxxxxxxxx";

// MQTT
String espDeviceName;
int stage = 0;
unsigned long lastMQTTupdate = millis();
unsigned long mqttattempt = (millis() - 3000);
String device_topic;
String report_topic;
bool online = false;
unsigned long disconnectMQTTms = 0;
// Initialised to Finish so the logic doesn't assume a Print has just finished and needs to wait for a door interaction to continue
String gcodeState = "FINISH";
float bed_temp = 0;

// Bento
int currentBedTargetTemp = 0;
int currentBedTemp = 0;
int bentoGraceTime = 600000;
int inactivityTimeOut = 3600000;

const char *wl_status_to_string(wl_status_t status)
{
  switch (status)
  {
  case WL_NO_SHIELD:
    return "WL_NO_SHIELD";
  case WL_IDLE_STATUS:
    return "WL_IDLE_STATUS";
  case WL_NO_SSID_AVAIL:
    return "WL_NO_SSID_AVAIL";
  case WL_SCAN_COMPLETED:
    return "WL_SCAN_COMPLETED";
  case WL_CONNECTED:
    return "WL_CONNECTED";
  case WL_CONNECT_FAILED:
    return "WL_CONNECT_FAILED";
  case WL_CONNECTION_LOST:
    return "WL_CONNECTION_LOST";
  case WL_DISCONNECTED:
    return "WL_DISCONNECTED";
  }
  return "UNKNOWN";
}

void ParseCallback(char *topic, byte *payload, unsigned int length)
{
  delay(500);
  JsonDocument json;
  JsonDocument filter;

  filter["print"]["bed_target_temper"] = true;
  filter["print"]["bed_temper"] = true;
  filter["print"]["gcode_state"] = true;

  auto deserializeError = deserializeJson(json, payload, length, DeserializationOption::Filter(filter));
  if (!deserializeError)
  {
    // Serial.println(F("Mqtt message received."));
    // Serial.print(F("FreeHeap: "));
    // Serial.println(ESP.getFreeHeap());

    bool changed = false;

    if (json["print"].containsKey("command"))
    {
      if (json["print"]["command"] == "gcode_line"           // gcode_line used a lot during print initialisations - Skip these
          || json["print"]["command"] == "project_prepare"   // 1 message per print
          || json["print"]["command"] == "project_file"      // 1 message per print
          || json["print"]["command"] == "clean_print_error" // During error (no info)
          || json["print"]["command"] == "resume"            // After error or pause
          || json["print"]["command"] == "get_accessories"   // After error or pause
          || json["print"]["command"] == "prepare")
      { // 1 message per print
        Serial.println(F("This message contains print/command, skipping..."));
        return;
      }
    }

    if (json.size() == 0)
    {
      // Null or Filtered message that are not 'Print' or 'System' payload - Ignore these
      Serial.println(F("This message contains nothing, skipping..."));
      return;
    }

    // Output Filtered MQTT message
    // Serial.print(F("(Filtered) MQTT payload, ["));
    // Serial.print(millis());
    // Serial.print(F("], "));
    // serializeJson(json, Serial);
    // Serial.println();

    // Check gcode state
    if (json["print"].containsKey("gcode_state") && ((millis() - lastMQTTupdate) > 3000))
    {
      String mqttgcodeState = json["print"]["gcode_state"].as<String>();
      // Serial.print(F("Received MQTT gcode_state: "));
      // Serial.println(mqttgcodeState);

      // Onchange of gcodeState...
      if (gcodeState != mqttgcodeState)
      {
        gcodeState = mqttgcodeState;

        Serial.print(F("MQTT update - gcode_state now: "));
        Serial.println(gcodeState);

        changed = true;
      }
    }

    // check bed target temp
    if (json["print"].containsKey("bed_target_temper") && ((millis() - lastMQTTupdate) > 3000))
    {
      const auto targetBedTemp = json["print"]["bed_target_temper"].as<int>();
      // Serial.print(F("Received MQTT bed_target_temper: "));
      // Serial.println(temp);

      if (targetBedTemp != currentBedTargetTemp)
      {
        if (currentBedTargetTemp != targetBedTemp)
        {
          currentBedTargetTemp = targetBedTemp;
          changed = true;
        }
      }
    }

    // bed temp changes
    if (json["print"].containsKey("bed_temper") && ((millis() - lastMQTTupdate) > 3000))
    {
      const auto bedTemp = json["print"]["bed_temper"].as<int>();
      // Serial.print(F("Received MQTT bed_target_temper: "));
      // Serial.println(temp);

      if (bedTemp != currentBedTemp)
      {
        currentBedTemp = bedTemp;
        changed = true;
      }
    }

    // perform actions only if something changed
    if (changed)
    {

      Serial.println(F("*Change from mqtt*"));

      // Turn on when target bed temp >= 90
      if (currentBedTargetTemp >= 90)
      {
        Serial.println(F("writing to pin 5: turn on bentobox"));
        digitalWrite(5, HIGH);
      }

      // Turn off when state is finished or idle and targettemp is 0 and bedtemp is below 60
      if ((gcodeState == "FINISH" || gcodeState == "IDLE" || gcodeState == "FAILED") && currentBedTargetTemp == 0 && currentBedTemp <= 60)
      {
        Serial.println(F("writing to pin 5: turn off bentobox"));
        digitalWrite(5, LOW);
      }

    }
  }
  else
  {
    Serial.println(F("Deserialize error while parsing mqtt"));
    return;
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  ParseCallback(topic, (byte *)stream.get_buffer(), stream.current_length());
  stream.flush();
}

void connectMqtt()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    // Abort MQTT connection attempt when no Wifi
    return;
  }
  if (!mqttClient.connected() && (millis() - mqttattempt) >= 3000)
  {
    espDeviceName = "ESP32MQTT-" + String(random(0xffff), HEX);
    Serial.println(F("Connecting to mqtt..."));
    if (mqttClient.connect(espDeviceName.c_str(), "bblp", accessCode))
    {
      Serial.print(F("MQTT connected, subscribing to MQTT Topic:  "));
      Serial.println(report_topic);
      mqttClient.subscribe(report_topic.c_str());
      online = true;
      disconnectMQTTms = 0;
    }
    else
    {
      Serial.println(F("Failed to connect with error code: "));
      Serial.print(mqttClient.state());
      Serial.print(F("  "));
      ParseMQTTState(mqttClient.state());
      if (mqttClient.state() == 5)
      {
        Serial.println(F("Restarting Device"));
        delay(1000);
        ESP.restart();
      }
    }
  }
}

void setupMqtt()
{
  Serial.print(F("Setting up MQTT with Bambu Lab Printer IP address: "));
  Serial.println(printerIP);

  device_topic = String("device/") + serialNumber;
  report_topic = device_topic + String("/report");

  mqttClient.setBufferSize(1024);
  wifiSecure.setInsecure();
  mqttClient.setServer(ipAddress, 8883);
  mqttClient.setStream(stream);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setSocketTimeout(20);

  Serial.println(F("Finished setting up MQTT"));

  connectMqtt();
}

void mqttloop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    // Abort MQTT connection attempt when no Wifi
    return;
  }
  if (!mqttClient.connected())
  {
    online = false;
    // Only sent the timer from the first instance of a MQTT disconnect
    if (disconnectMQTTms == 0)
    {
      disconnectMQTTms = millis();
      // Record last time MQTT dropped connection
      Serial.println(F("MQTT dropped during mqttloop"));
      ParseMQTTState(mqttClient.state());
    }
    delay(500);
    connectMqtt();
    delay(32);
    return;
  }
  else
  {
    disconnectMQTTms = 0;
  }
  mqttClient.loop();
  delay(10);
}

bool connectToWifi()
{
  Serial.println(F("-------------------------------------"));
  WiFi.mode(WIFI_STA); // ESP32 connects to an access point
  delay(10);
  WiFi.disconnect();

  wl_status_t status = WiFi.status();
  while (status != WL_CONNECTED)
  {
    // Direct connect with SSID
    if (wifimode == 0)
    {
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      connectionAttempts = 1;
      wifimode = 1;
      Serial.println(F("Attempting to connect without specific BSSI"));
    }

    Serial.print(F("Connecting to WIFI.. Status check #"));
    Serial.print(connectionAttempts);
    Serial.print(F(" / 10      SSID: "));
    Serial.print(ssid);
    Serial.println();

    if (status != WiFi.status())
    {
      Serial.println(status);

      status = WiFi.status();
      switch (status)
      {
      case WL_CONNECTED:
      case WL_IDLE_STATUS:
      case WL_CONNECT_FAILED:
        Serial.print(F("Wifi Status: "));
        Serial.println(wl_status_to_string(status));
        break;
      case WL_NO_SSID_AVAIL:
        Serial.print(F("Wifi Status: "));
        Serial.println(wl_status_to_string(status));
        Serial.println(F("Bad WiFi credentials"));
        return false;
      case WL_DISCONNECTED:
        Serial.print(F("Wifi Status: "));
        Serial.println(wl_status_to_string(status));
        Serial.println(F("Disconnected. (Check low RSSI)"));
        return false;
      default:
        Serial.print(F("Uncaught Status - Wifi Status: "));
        Serial.println(wl_status_to_string(status));
        break;
      }
    }
    delay(2000); // Giving time to connect
    connectionAttempts++;
  }

#ifdef ARDUINO_ARCH_ESP32
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // https://github.com/G6EJD/ESP32-8266-Adjust-WiFi-RF-Power-Output/blob/main/README.md
#endif

#ifdef ESP32
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // https://github.com/G6EJD/ESP32-8266-Adjust-WiFi-RF-Power-Output/blob/main/README.md
#endif
  return true;
};

void setup()
{
  Serial.begin(115200);

  delay(100);

  Serial.println(F("Initializing"));
  Serial.println(ESP.getFreeHeap());

  // Setup Pin as output
  pinMode(5, OUTPUT);

  if (!connectToWifi())
  {
    return;
  }

  delay(500);

  Serial.print(F("Setting up MQTT...\n"));
  setupMqtt();

  digitalWrite(5, LOW);
}

void loop()
{
  mqttloop();

  if (WiFi.status() != WL_CONNECTED)
  {
    if (WiFi.status() == WL_DISCONNECTED)
      Serial.print(F("Wifi connection Disconnected.  "));

    Serial.println(F("Attempting to reconnect to WiFi..."));
    WiFi.disconnect();
    delay(10);
    WiFi.reconnect();
  }

  mqttClient.loop();
  delay(10);
}