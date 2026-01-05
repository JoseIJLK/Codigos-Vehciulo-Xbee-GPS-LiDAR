#include <RPLidar.h>
#include <ArduinoJson.h>

RPLidar lidar;

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

#define LIDAR_FRENTE 12
#define LIDAR_LADO   12

#define DIST_FRENTE_CRITICA 25
#define DIST_FRENTE        38
#define DIST_LADO          30

#define VELOCIDAD 135
#define GIRO_VELOCIDAD 110

#define T_PARADA     120
#define T_RETROCESO  550
#define T_GIRO       650
#define T_SALIDA     500

bool modo_auto = false;
char comando_manual = 'S';

String estado_auto = "detenido";


enum EstadoAuto {
  AVANZAR,
  PARAR,
  RETROCEDER,
  GIRAR_SITIO,
  SALIDA_LIMPIA
};

EstadoAuto estado = AVANZAR;
unsigned long t_estado = 0;
bool giro_izq = true;

float dF = 9999, dL = 9999, dR = 9999;
unsigned long t_lidar = 0;

void setup() {
  Serial.begin(9600);     
  Serial1.begin(115200);  
  Serial2.begin(9600);    

  lidar.begin(Serial1);
  lidar.stop();
  delay(200);
  lidar.startScan();

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  detener();
}

void loop() {
  recibir_comandos();
  reenviar_gps();
  procesar_lidar();

  if (modo_auto)
    autonomo();
  else
    manual();

  enviar_estado_auto();
}


void recibir_comandos() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'A') modo_auto = true;
    else if (c == 'M') modo_auto = false;
    else comando_manual = c;
  }
}

void reenviar_gps() {
  if (Serial2.available()) {
    Serial.println(Serial2.readStringUntil('\n'));
  }
}


void procesar_lidar() {

  if (millis() - t_lidar > 100) {
    dF = dL = dR = 9999;
    t_lidar = millis();
  }

  if (IS_OK(lidar.waitPoint())) {

    float d = lidar.getCurrentPoint().distance / 10.0; 
    float a = lidar.getCurrentPoint().angle;

    if (d > 0 && d < 200) {

      if (a > 55 && a < 125) dF = min(dF, d);
      if (a >= 125 && a <= 210) dL = min(dL, d);
      if (a >= 0 && a <= 55) dR = min(dR, d);


      float x = -d * sin(a * DEG_TO_RAD);
      float y = -d * cos(a * DEG_TO_RAD);

      StaticJsonDocument<128> doc;
      JsonArray arr = doc.createNestedArray("lidar");
      JsonObject p = arr.createNestedObject();
      p["x"] = x;
      p["y"] = y;
      serializeJson(doc, Serial);
      Serial.println();
    }
  }
}


void autonomo() {

  switch (estado) {

    case AVANZAR:
      if (dF < DIST_FRENTE_CRITICA) {
        estado = PARAR;
        t_estado = millis();
        detener();
      }
      else if (dF < DIST_FRENTE) {
        estado = RETROCEDER;
        t_estado = millis();
        giro_izq = (dL > dR);
      }
      else if (dL < DIST_LADO) {
        girar_derecha_suave();
      }
      else if (dR < DIST_LADO) {
        girar_izquierda_suave();
      }
      else {
        avanzar();
      }
      break;

    case PARAR:
      if (millis() - t_estado > T_PARADA) {
        estado = RETROCEDER;
        t_estado = millis();
      }
      break;

    case RETROCEDER:
      retroceder();
      if (millis() - t_estado > T_RETROCESO) {
        estado = GIRAR_SITIO;
        t_estado = millis();
      }
      break;

    case GIRAR_SITIO:
      if (giro_izq) girar_izquierda();
      else girar_derecha();

      if (millis() - t_estado > T_GIRO) {
        estado = SALIDA_LIMPIA;
        t_estado = millis();
      }
      break;

    case SALIDA_LIMPIA:
      avanzar();
      if (millis() - t_estado > T_SALIDA) {
        estado = AVANZAR;
      }
      break;
  }
}

void manual() {
  switch (comando_manual) {
    case 'F': avanzar(); break;
    case 'B': retroceder(); break;
    case 'L': girar_izquierda(); break;
    case 'R': girar_derecha(); break;
    default: detener();
  }
}

void avanzar() {
  estado_auto = "adelante";
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void retroceder() {
  estado_auto = "atras";
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void girar_derecha() {
  estado_auto = "derecha";
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void girar_izquierda() {
  estado_auto = "izquierda";
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void girar_derecha_suave() {
  estado_auto = "derecha";
  analogWrite(ENA, GIRO_VELOCIDAD);
  analogWrite(ENB, VELOCIDAD);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void girar_izquierda_suave() {
  estado_auto = "izquierda";
  analogWrite(ENA, VELOCIDAD);
  analogWrite(ENB, GIRO_VELOCIDAD);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void detener() {
  estado_auto = "detenido";
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void enviar_estado_auto() {
  static unsigned long t = 0;
  if (millis() - t > 300) {
    StaticJsonDocument<64> doc;
    doc["auto_dir"] = estado_auto;
    doc["modo"] = modo_auto ? "auto" : "manual";
    serializeJson(doc, Serial);
    Serial.println();
    t = millis();
  }
}
