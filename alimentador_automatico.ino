// ============================================================================
// SISTEMA DE CONTROL TERMICO Y ALIMENTACION PORCINA CON UBIDOTS
// ESP32 + Ubidots MQTT + OLED SH1106 + DS18B20 + DFPlayer Mini
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <UbidotsEsp32Mqtt.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobotDFPlayerMini.h>

// ============================================================================
// CONFIGURACION
// ============================================================================
// Dejar en false cuando se conecte el DS18B20 real.
#define SIMULAR_SENSOR false

// No guardes credenciales reales en el repositorio. Usa un archivo local o
// reemplaza estos valores solo en tu copia antes de cargar el programa.
const char *UBIDOTS_TOKEN = "TU_TOKEN_UBIDOTS";
const char *WIFI_SSID = "TU_RED_WIFI";
const char *WIFI_PASS = "TU_CONTRASENA_WIFI";
const char *DEVICE_LABEL = "control-termico-esp32";

Ubidots ubidots(UBIDOTS_TOKEN);

// Relés activos en LOW.
constexpr uint8_t RELE_ON = LOW;
constexpr uint8_t RELE_OFF = HIGH;

// Pines.
constexpr uint8_t ONE_WIRE_BUS = 4;
constexpr uint8_t RELE_VENT = 26;
constexpr uint8_t RELE_CALOR = 25;
constexpr uint8_t NIVEL_PIN = 32;
constexpr uint8_t DFPLAYER_RX = 18;
constexpr uint8_t DFPLAYER_TX = 19;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
HardwareSerial mp3Serial(2);
DFRobotDFPlayerMini mp3;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ============================================================================
// CONTROL TERMICO
// ============================================================================
constexpr float SETPOINT = 38.5f;
constexpr float BANDA_PID = 3.0f;
constexpr float HIST_TEMPERATURA = 0.5f;
constexpr float KP = 40.0f;
constexpr float KI = 0.2f;
constexpr float KD = 8.0f;
constexpr float INTEGRAL_MIN = -50.0f;
constexpr float INTEGRAL_MAX = 50.0f;

float tempActual = 0.0f;
float errorAnterior = 0.0f;
float integral = 0.0f;
float pidOutput = 0.0f;

// ============================================================================
// INTERVALOS
// ============================================================================
constexpr unsigned long VENTANA_PWM = 5000UL;
constexpr unsigned long INTERVALO_TEMP = 800UL;
constexpr unsigned long REPETICION_LLAMADO = 60000UL;
constexpr unsigned long DURACION_AUDIO = 30000UL;
constexpr unsigned long INTERVALO_OLED = 250UL;
constexpr unsigned long INTERVALO_UBIDOTS = 5000UL;
constexpr unsigned long TIMEOUT_RECONEXION = 10000UL;

unsigned long ahora = 0;
unsigned long ultimaTemperatura = 0;
unsigned long inicioPWM = 0;
unsigned long ultimoLlamado = 0;
unsigned long inicioAudio = 0;
unsigned long ultimaOLED = 0;
unsigned long ultimoUbidots = 0;
unsigned long ultimoIntentoWiFi = 0;
unsigned long ultimoPID = 0;

bool metaAlcanzada = false;
bool reproduciendoAudio = false;
bool modoPIDActivo = false;
bool temperaturaValida = false;
bool mp3Disponible = false;

void mostrarOLED();
void gestionarAudio();
void calcularPID();
void reiniciarPID();
void actualizarTemperatura();
void apagarActuadores();
void procesarSeguridad();
void enviarDatosUbidots();
void actualizarComunicaciones();
void detenerAudio();

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin(21, 22);

  pinMode(RELE_VENT, OUTPUT);
  pinMode(RELE_CALOR, OUTPUT);
  pinMode(NIVEL_PIN, INPUT_PULLUP);
  apagarActuadores();

  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, "Control Termico");
  u8g2.drawStr(0, 35, "Iniciando...");
  u8g2.sendBuffer();

#if !SIMULAR_SENSOR
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
#endif

  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  mp3Disponible = mp3.begin(mp3Serial, true, false);
  if (mp3Disponible) {
    mp3.volume(25);
    mp3.EQ(DFPLAYER_EQ_NORMAL);
  } else {
    Serial.println("DFPlayer no disponible; se continua sin audio.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  ubidots.setup();

  ahora = millis();
  ultimaTemperatura = ahora;
  ultimoPID = ahora;
  inicioPWM = ahora;
}

void loop() {
  ahora = millis();
  actualizarComunicaciones();

  // Las protecciones siempre se ejecutan localmente, aunque no haya WiFi.
  if (digitalRead(NIVEL_PIN) != LOW) {
    procesarSeguridad();
    return;
  }

  actualizarTemperatura();
  if (!temperaturaValida) {
    procesarSeguridad();
    return;
  }

  // Histéresis para evitar conmutaciones alrededor del setpoint.
  if (!metaAlcanzada && tempActual >= SETPOINT) {
    metaAlcanzada = true;
  } else if (metaAlcanzada && tempActual <= SETPOINT - HIST_TEMPERATURA) {
    metaAlcanzada = false;
  }

  // El ventilador/agitador funciona durante el calentamiento.
  digitalWrite(RELE_VENT, metaAlcanzada ? RELE_OFF : RELE_ON);
  gestionarAudio();

  if (metaAlcanzada) {
    digitalWrite(RELE_CALOR, RELE_OFF);
    reiniciarPID();
  } else if (!modoPIDActivo) {
    // Calentamiento completo mientras se esta lejos del setpoint.
    digitalWrite(RELE_CALOR, RELE_ON);
  } else {
    if (ahora - inicioPWM >= VENTANA_PWM) {
      inicioPWM = ahora;
    }
    const bool calefactorON = pidOutput > (ahora - inicioPWM);
    digitalWrite(RELE_CALOR, calefactorON ? RELE_ON : RELE_OFF);
  }

  if (ahora - ultimoUbidots >= INTERVALO_UBIDOTS) {
    ultimoUbidots = ahora;
    enviarDatosUbidots();
  }
  if (ahora - ultimaOLED >= INTERVALO_OLED) {
    ultimaOLED = ahora;
    mostrarOLED();
  }
}

void actualizarComunicaciones() {
  if (WiFi.status() != WL_CONNECTED) {
    if (ahora - ultimoIntentoWiFi >= TIMEOUT_RECONEXION) {
      ultimoIntentoWiFi = ahora;
      Serial.println("Intentando reconectar WiFi...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  if (!ubidots.connected()) {
    ubidots.reconnect();
  }
  ubidots.loop();
}

void actualizarTemperatura() {
  if (ahora - ultimaTemperatura < INTERVALO_TEMP) {
    return;
  }

#if SIMULAR_SENSOR
  temperaturaValida = true;
  // Simulacion simple para pruebas de pantalla y actuadores.
  tempActual += metaAlcanzada ? -0.1f : 0.4f;
  if (tempActual > 42.0f) {
    tempActual = 20.0f;
    metaAlcanzada = false;
  }
#else
  const float leida = sensors.getTempCByIndex(0);
  temperaturaValida = leida != DEVICE_DISCONNECTED_C &&
                      leida > -10.0f && leida < 85.0f;
  if (temperaturaValida) {
    tempActual = leida;
  }
  sensors.requestTemperatures();
#endif

  if (temperaturaValida) {
    if (tempActual >= SETPOINT - BANDA_PID) {
      modoPIDActivo = true;
      calcularPID();
    } else {
      modoPIDActivo = false;
      reiniciarPID();
    }
  } else {
    modoPIDActivo = false;
    reiniciarPID();
    Serial.println("Lectura DS18B20 invalida.");
  }
  ultimaTemperatura = ahora;
}

void calcularPID() {
  float dt = (ahora - ultimoPID) / 1000.0f;
  if (dt <= 0.0f || dt > 10.0f) {
    dt = INTERVALO_TEMP / 1000.0f;
  }

  const float error = SETPOINT - tempActual;
  integral = constrain(integral + error * dt, INTEGRAL_MIN, INTEGRAL_MAX);
  const float derivada = (error - errorAnterior) / dt;
  const float salida = KP * error + KI * integral + KD * derivada;

  pidOutput = constrain(salida, 0.0f, static_cast<float>(VENTANA_PWM));
  errorAnterior = error;
  ultimoPID = ahora;
}

void reiniciarPID() {
  integral = 0.0f;
  errorAnterior = 0.0f;
  pidOutput = 0.0f;
  ultimoPID = ahora;
}

void apagarActuadores() {
  digitalWrite(RELE_CALOR, RELE_OFF);
  digitalWrite(RELE_VENT, RELE_OFF);
}

void procesarSeguridad() {
  apagarActuadores();
  metaAlcanzada = false;
  modoPIDActivo = false;
  if (digitalRead(NIVEL_PIN) != LOW) {
    temperaturaValida = false;
  }
  reiniciarPID();
  detenerAudio();

  if (ahora - ultimoUbidots >= INTERVALO_UBIDOTS) {
    ultimoUbidots = ahora;
    enviarDatosUbidots();
  }
  if (ahora - ultimaOLED >= INTERVALO_OLED) {
    ultimaOLED = ahora;
    mostrarOLED();
  }
}

void detenerAudio() {
  if (reproduciendoAudio && mp3Disponible) {
    mp3.stop();
  }
  reproduciendoAudio = false;
}

void gestionarAudio() {
  if (metaAlcanzada && !reproduciendoAudio &&
      ahora - ultimoLlamado >= REPETICION_LLAMADO) {
    ultimoLlamado = ahora;
    inicioAudio = ahora;
    reproduciendoAudio = true;
    if (mp3Disponible) {
      mp3.play(1);
    }
  }

  if (reproduciendoAudio && ahora - inicioAudio >= DURACION_AUDIO) {
    detenerAudio();
  }
}

void enviarDatosUbidots() {
  if (WiFi.status() != WL_CONNECTED || !ubidots.connected()) {
    return;
  }

  const int nivel = digitalRead(NIVEL_PIN) == LOW ? 1 : 0;
  const int estadoCalor = digitalRead(RELE_CALOR) == RELE_ON ? 1 : 0;
  const int estadoVent = digitalRead(RELE_VENT) == RELE_ON ? 1 : 0;
  int estado = 0;

  if (nivel) {
    if (!temperaturaValida) estado = 5;       // Sensor invalido
    else if (reproduciendoAudio) estado = 4; // Llamando
    else if (metaAlcanzada) estado = 3;      // Temperatura OK
    else if (modoPIDActivo) estado = 2;      // Control PID
    else estado = 1;                         // Calentando
  }

  // Un solo publish reduce trafico y mantiene las variables sincronizadas.
  ubidots.add("temperatura", tempActual);
  ubidots.add("setpoint", SETPOINT);
  ubidots.add("rele-calor", estadoCalor);
  ubidots.add("rele-ventilador", estadoVent);
  ubidots.add("nivel-agua", nivel);
  ubidots.add("temperatura-valida", temperaturaValida ? 1 : 0);
  ubidots.add("audio", reproduciendoAudio ? 1 : 0);
  ubidots.add("estado", estado);
  ubidots.publish(DEVICE_LABEL);
}

void mostrarOLED() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);

  const bool nivelOK = digitalRead(NIVEL_PIN) == LOW;
  const bool wifiOK = WiFi.status() == WL_CONNECTED;
  const bool ventON = digitalRead(RELE_VENT) == RELE_ON;
  const bool calorON = digitalRead(RELE_CALOR) == RELE_ON;
  char buf[35];

  if (!nivelOK) u8g2.drawStr(0, 8, "ESTADO: NIVEL BAJO");
  else if (!temperaturaValida) u8g2.drawStr(0, 8, "ESTADO: SENSOR ERROR");
  else if (reproduciendoAudio) u8g2.drawStr(0, 8, "ESTADO: LLAMANDO");
  else if (metaAlcanzada) u8g2.drawStr(0, 8, "ESTADO: TEMP OK");
  else if (modoPIDActivo) u8g2.drawStr(0, 8, "ESTADO: CONTROL PID");
  else u8g2.drawStr(0, 8, "ESTADO: CALENTANDO");

  u8g2.drawHLine(0, 11, 128);
  u8g2.drawStr(0, 20, "--- TEMPERATURA ---");
  if (temperaturaValida) {
    snprintf(buf, sizeof(buf), "SET: %.1fC REAL: %.1fC", SETPOINT, tempActual);
  } else {
    snprintf(buf, sizeof(buf), "SET: %.1fC REAL: --.-C", SETPOINT);
  }
  u8g2.drawStr(0, 30, buf);
  u8g2.drawHLine(0, 33, 128);

  snprintf(buf, sizeof(buf), "NIVEL: %s", nivelOK ? "OK" : "BAJO");
  u8g2.drawStr(0, 42, buf);
  snprintf(buf, sizeof(buf), "CALOR:%s VENT:%s", calorON ? "ON" : "OFF", ventON ? "ON" : "OFF");
  u8g2.drawStr(0, 52, buf);
  snprintf(buf, sizeof(buf), "WIFI: %s", wifiOK ? "OK" : "OFF");
  u8g2.drawStr(0, 62, buf);
  u8g2.sendBuffer();
}
