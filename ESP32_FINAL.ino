#include <TinyGPSPlus.h>

TinyGPSPlus gps;


HardwareSerial gpsSerial(1);

HardwareSerial arduinoSerial(2);

void setup() {
  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);   
  arduinoSerial.begin(9600, SERIAL_8N1, 4, 2);  

  Serial.println("ESP32 GPS iniciado");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    String json = "{";
    json += "\"gps\":{";
    json += "\"lat\":" + String(gps.location.lat(), 6) + ",";
    json += "\"lon\":" + String(gps.location.lng(), 6) + ",";
    json += "\"alt\":" + String(gps.altitude.meters());
    json += "}}";

    arduinoSerial.println(json);
    Serial.println(json);
  }

  delay(500);
}
