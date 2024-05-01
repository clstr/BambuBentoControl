#include <Arduino.h>

#ifndef _MQTTPARSERUTILITY
#define _MQTTPARSERUTILITY


void ParseMQTTState(int code){
    switch (code)
    {
    case -4: // MQTT_CONNECTION_TIMEOUT
        Serial.println(F("MQTT TIMEOUT"));
        break;
    case -3: // MQTT_CONNECTION_LOST
        Serial.println(F("MQTT CONNECTION_LOST"));
        break;
    case -2: // MQTT_CONNECT_FAILED
        Serial.println(F("MQTT CONNECT_FAILED"));
        break;
    case -1: // MQTT_DISCONNECTED
        Serial.println(F("MQTT DISCONNECTED"));
        break;
    case 0:  // MQTT_CONNECTED
        Serial.println(F("MQTT CONNECTED"));
        break;
    case 1:  // MQTT_CONNECT_BAD_PROTOCOL
        Serial.println(F("MQTT BAD PROTOCOL"));
        break;
    case 2:  // MQTT_CONNECT_BAD_CLIENT_ID
        Serial.println(F("MQTT BAD CLIENT ID"));
        break;
    case 3:  // MQTT_CONNECT_UNAVAILABLE
        Serial.println(F("MQTT UNAVAILABLE"));
        break;
    case 4:  // MQTT_CONNECT_BAD_CREDENTIALS
        Serial.println(F("MQTT BAD CREDENTIALS"));
        break;
    case 5: // MQTT UNAUTHORIZED
        Serial.println(F("MQTT UNAUTHORIZED"));
        break;
    }
}


#endif