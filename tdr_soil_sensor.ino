// This code is a modification of the code found here: https://scienceinhydroponics.com/2023/01/connecting-a-low-cost-tdr-moisture-content-ec-temp-sensor-to-a-nodemcuv3.html
// Which was based off of: https://github.com/kromadg/soil-sensor
// Written for NodeMCU v3 (ESP8266) using ESP Software Serial Library.
// This TDR sensor measures moisture content, EC, and temperature for Rockwool Substrates and sends the data to a MQTT server by Ehsan Khalid.

#include <SoftwareSerial.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Replace with your WiFi and MQTT credentials
const char* ssid = "your_ssid";
const char* password = "your_password";
const char* mqtt_server = "your_mqtt_server";
const char* mqtt_user = "your_mqtt_user";
const char* mqtt_password = "your_mqtt_password";
const char* mqtt_topic = "mycodo/sensors";

const float TEMP_CORRECTION = 0.5;
const float HUM_CORRECTION = 0.8;
const float EC_SLOPE = 1.93;
const float EC_INTERCEPT = -270.8;
const float EC_TEMP_COEFF = 0.019;

#define RE D2
#define DE D3

const byte hum_temp_ec[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x05, 0xCB};
byte sensorResponse[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte sensor_values[11];

SoftwareSerial mod(D6, D5); // RX, TX
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;
const unsigned long interval = 5000;

// Create Instance of WiFi and MQTT client
void connectWiFi() {
  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }

  Serial.println(F(""));
  Serial.println(F("WiFi connected"));
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  client.setServer(mqtt_server, 1883);

  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));

    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println(F("connected"));
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      delay(5000);
    }
  }
}

void setup() {
    Serial.begin(115200);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE, LOW);
    digitalWrite(DE, LOW);
    delay(1000);
    mod.begin(4800);
    delay(100);

    // Connect to WiFi and MQTT Server
    connectWiFi();
    connectMQTT();
}

// Soil EC Reading
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    memset(sensor_values, 0, sizeof(sensor_values));
    delay(100);
    
    if (mod.write(hum_temp_ec, sizeof(hum_temp_ec)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for (byte i = 0; i < 12; i++) {
            sensorResponse[i] = mod.read();
            yield();
        }
    }
  }


    delay(250);

    // Sensor Response Data
    float soil_hum = 0.1 * int(sensorResponse[3] << 8 | sensorResponse[4]);
    float soil_temp = 0.1 * int(sensorResponse[5] << 8 | sensorResponse[6]);
    int soil_ec = int(sensorResponse[7] << 8 | sensorResponse[8]);

    // Soil EC Equations obtained from calibration using distilled water and a 1.1178 mS/cm solution
    soil_ec = EC_SLOPE * soil_ec + EC_INTERCEPT;
    soil_ec = soil_ec / (1.0 + EC_TEMP_COEFF * (soil_temp - 25));

    // Study by Shang et al. 2020 showed that the temperature readings of THC-S Sensor were consistently lower than those of Teros 11 by 0.5 degrees Celsius
    soil_temp = soil_temp + TEMP_CORRECTION;
    
    // 80% soil humidity correction for rockwool
    soil_hum = soil_hum * HUM_CORRECTION;

    // Updated Formula for calculating the apparent dielectric constant of soil based on Briciu-Burghina 2022. Corrected for Rockwool
    float soil_apparent_dieletric_constant = 1.68 + 0.267 * soil_hum - 0.00476 * soil_temp + 0.000101 * soil_temp * soil_temp;
    float soil_bulk_permittivity = soil_apparent_dieletric_constant;
    float soil_pore_permittivity = 80.3 / (1.0 + 0.06 * (soil_temp - 25.0));

    // Convert Bulk EC to Pore Water EC using Briciu-Burghina 2022
    float soil_pore_water_ec;
    if (soil_bulk_permittivity > 5.5)
        soil_pore_water_ec = (soil_ec * (soil_pore_permittivity - 5.5)) / (soil_bulk_permittivity - 5.5) / 1000.0;
    else
        soil_pore_water_ec = 0.0;

    Serial.print(F("Humidity:"));
    Serial.print(soil_hum);
    Serial.print(F(",Temperature:"));
    Serial.print(soil_temp);
    Serial.print(F(",EC:"));
    Serial.print(soil_ec);
    Serial.print(F(",pwEC:"));
    Serial.print(soil_pore_water_ec);
    Serial.print(F(",soil_bulk_permittivity:"));
    Serial.println(soil_bulk_permittivity);
    
    // Reconnect to MQTT if connection is lost
    if (!client.connected()) {
        connectMQTT();
    }

    // Publish sensor values to the MQTT server
    String payload = String("Humidity:") + soil_hum + ",Temperature:" + soil_temp + ",EC:" + soil_ec + ",pwEC:" + soil_pore_water_ec + ",soil_bulk_permittivity:" + soil_bulk_permittivity;
    client.publish(mqtt_topic, payload.c_str());
  }