#include <SoftwareSerial.h>
#include <Wire.h>

// This code is a modification of the code found here (https://github.com/kromadg/soil-sensor)

#define RE D2
#define DE D3

const byte hum_temp_ec[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x05, 0xCB};
byte sensorResponse[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte sensor_values[11];

SoftwareSerial mod(D6, D5); // RX, TX

void setup() {
    Serial.begin(115200);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE, LOW);
    digitalWrite(DE, LOW);
    delay(1000);
    mod.begin(4800);
    delay(100);
}

void loop() {
    /************** Soil EC Reading *******************/
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

    delay(250);

    // get sensor response data
    float soil_hum = 0.1 * int(sensorResponse[3] << 8 | sensorResponse[4]);
    float soil_temp = 0.1 * int(sensorResponse[5] << 8 | sensorResponse[6]);
    int soil_ec = int(sensorResponse[7] << 8 | sensorResponse[8]);

    /************* Calculations and sensor corrections *************/

    float as_read_ec = soil_ec;

    // This equation was obtained from calibration using distilled water and a 1.1178mS/cm solution.
    soil_ec = 1.93*soil_ec - 270.8;
    soil_ec = soil_ec/(1.0+0.019*(soil_temp-25));

    // soil_temp was left the same because the Teros and chinese sensor values are similar

    // quadratic aproximation
    // the teros bulk_permittivity was calculated from the teros temperature, teros bulk ec and teros pwec by Hilhorst 2000 model
    float soil_apparent_dieletric_constant = 1.3088 + 0.1439 * soil_hum + 0.0076 * soil_hum * soil_hum;

    float soil_bulk_permittivity = soil_apparent_dieletric_constant;  /// Hammed 2015 (apparent_dieletric_constant is the real part of permittivity)
    float soil_pore_permittivity = 80.3 - 0.37 * (soil_temp - 20); /// same as water 80.3 and corrected for temperature

    // converting bulk EC to pore water EC
    float soil_pw_ec;
    if (soil_bulk_permittivity > 4.1)
        soil_pw_ec = ((soil_pore_permittivity * soil_ec) / (soil_bulk_permittivity - 4.1) / 1000); /// from Hilhorst 2000.
    else
        soil_pw_ec = 0;

    Serial.print("Humidity:");
    Serial.print(soil_hum);
    Serial.print(",");
    Serial.print("Temperature:");
    Serial.print(soil_temp);
    Serial.print(",");
    Serial.print("EC:");
    Serial.print(soil_ec);
    Serial.print(",");
    Serial.print("READEC:");
    Serial.print(as_read_ec);
    Serial.print(",");
    Serial.print("pwEC:");
    Serial.print(soil_pw_ec);
    Serial.print(",");
    Serial.print("soil_bulk_permittivity:");
    Serial.println(soil_bulk_permittivity);
    delay(5000);
}