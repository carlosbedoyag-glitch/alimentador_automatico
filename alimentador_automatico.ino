// ============================================================================
// SISTEMA DE CONTROL TERMICO Y ALIMENTACION PORCINA CON UBIDOTS
// ESP32 + Ubidots MQTT + OLED SH1106 + DS18B20 + DFPlayer Mini
// ============================================================================

// ============================================================================
// BLOQUE 1: LIBRERIAS Y DEPENDENCIAS
// -----------------------------------------------------------------------------
// Incluye todas las bibliotecas necesarias para WiFi, MQTT, sensores,
// display OLED, DS18B20 y reproductor MP3.
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
// BLOQUE 2: CONFIGURACION GENERAL DEL SISTEMA
// -----------------------------------------------------------------------------
// Aquí se definen los parámetros de red, nombre del dispositivo y el modo de
// operación del sensor.
// ============================================================================

// Dejar en false cuando se conecte el DS18B20 real.
#define SIMULAR_SENSOR false

// Parámetros de conexión WiFi y Ubidots.
const char *UBIDOTS_TOKEN = "TU_TOKEN_UBIDOTS";
const char *WIFI_SSID = "TU_RED_WIFI";
const char *WIFI_PASS = "TU_CONTRASENA_WIFI";
const char *DEVICE_LABEL = "control-termico-esp32";

Ubidots ubidots(UBIDOTS_TOKEN);

// Relés activos en LOW.
constexpr uint8_t RELE_ON = LOW;
constexpr uint8_t RELE_OFF = HIGH;

// ============================================================================
// BLOQUE 3: DEFINICION DE PINES Y COMPONENTES
// -----------------------------------------------------------------------------
// Se asignan los pines GPIO usados por los relés, sensor, display OLED y
// DFPlayer. Esto centraliza la configuración del hardware.
// ============================================================================

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
// BLOQUE 4: PARAMETROS DEL CONTROL TERMICO
// -----------------------------------------------------------------------------
// Aquí se configuran los valores del setpoint, rango de histéresis y
// constantes del controlador PID.
// ============================================================================
constexpr float SETPOINT = 38.5f;
constexpr float BANDA_PID = 3.0f;
constexpr float HIST_TEMPERATURA = 0.5f;
constexpr float KP = 40.0f;
constexpr float KI = 0.2f;
constexpr float KD = 8.0f;
constexpr float INTEGRAL_MIN = -50.0f;
constexpr float INTEGRAL_MAX = 50.0f;

// ============================================================================
// BLOQUE 5: VARIABLES DE ESTADO Y CONTROL
// -----------------------------------------------------------------------------
// Se almacenan aquí los valores actuales del sistema: temperatura, errores,
// salida PID, tiempos y banderas de estado.
// ============================================================================
float tempActual = 0.0f;
float errorAnterior = 0.0f;
float integral = 0.0f;
float pidOutput = 0.0f;

// ============================================================================
// BLOQUE 6: INTERVALOS DE TIMING Y DELAY
// -----------------------------------------------------------------------------
// Define cada cuánto tiempo se toman lecturas, se actualiza la pantalla,
// se transmite a Ubidots y se activa la alarma sonora.
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

// ============================================================================
// BLOQUE 7: PROTOTIPOS DE FUNCIONES
// -----------------------------------------------------------------------------
// Declara las funciones que se usan en setup() y loop() para mantener
// una organización clara del código.
// ============================================================================
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

// ============================================================================
// BLOQUE 8: SETUP
// -----------------------------------------------------------------------------
// Inicializa todos los periféricos: serial, comunicación I2C, relés,
// sensor, OLED, DFPlayer y conexión WiFi.
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin(21, 22);

  // Configuración de los relés como salidas y apagado inicial.
  pinMode(RELE_VENT, OUTPUT);
  pinMode(RELE_CALOR, OUTPUT);
  pinMode(NIVEL_PIN, INPUT_PULLUP);
  apagarActuadores();

  // Inicializa la pantalla OLED con un mensaje de arranque.
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, "Control Termico");
  u8g2.drawStr(0, 35, "Iniciando...");
  u8g2.sendBuffer();

  // Inicializa el sensor DS18B20 si no se está simulando.
#if !SIMULAR_SENSOR
  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
#endif

  // Inicializa el DFPlayer Mini.
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  mp3Disponible = mp3.begin(mp3Serial, true, false);
  if (mp3Disponible) {
    mp3.volume(25);
    mp3.EQ(DFPLAYER_EQ_NORMAL);
  } else {
    Serial.println("DFPlayer no disponible; se continua sin audio.");
  }

  // Configura WiFi y la conexión de Ubidots.
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  ubidots.setup();

  // Inicializa tiempos de control.
  ahora = millis();
  ultimaTemperatura = ahora;
  ultimoPID = ahora;
  inicioPWM = ahora;
}

// ============================================================================
// BLOQUE 9: LOOP PRINCIPAL
// -----------------------------------------------------------------------------
// Es el ciclo continuo del programa. Aquí se actualizan comunicaciones,
// se revisa el nivel del agua, se lee la temperatura, se activa la lógica
// térmica y se publica información.
// ============================================================================
void loop() {
  ahora = millis();

  // Mantiene activas la conexión WiFi y MQTT.
  actualizarComunicaciones();

  // Si el nivel de agua está bajo, se bloquea el funcionamiento normal.
  if (digitalRead(NIVEL_PIN) != LOW) {
    procesarSeguridad();
    return;
  }

  // Lee y valida la temperatura.
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

  // Lógica de audio de alerta cuando se alcanza la temperatura objetivo.
  gestionarAudio();

  // Control de calentador según estado de temperatura y modo PID.
  if (metaAlcanzada) {
    digitalWrite(RELE_CALOR, RELE_OFF);
    reiniciarPID();
  } else if (!modoPIDActivo) {
    // Calentamiento completo mientras se está lejos del setpoint.
    digitalWrite(RELE_CALOR, RELE_ON);
  } else {
    if (ahora - inicioPWM >= VENTANA_PWM) {
      inicioPWM = ahora;
    }
    const bool calefactorON = pidOutput > (ahora - inicioPWM);
    digitalWrite(RELE_CALOR, calefactorON ? RELE_ON : RELE_OFF);
  }

  // Publicación periódica de datos a Ubidots.
  if (ahora - ultimoUbidots >= INTERVALO_UBIDOTS) {
    ultimoUbidots = ahora;
    enviarDatosUbidots();
  }

  // Actualización periódica del display OLED.
  if (ahora - ultimaOLED >= INTERVALO_OLED) {
    ultimaOLED = ahora;
    mostrarOLED();
  }
}

// ============================================================================
// BLOQUE 10: ACTUALIZACION DE COMUNICACIONES
// -----------------------------------------------------------------------------
// Reintenta la conexión WiFi cuando se pierde, y mantiene la sesión MQTT
// de Ubidots viva cuando la red está disponible.
// ============================================================================
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

// ============================================================================
// BLOQUE 11: ACTUALIZACION DE TEMPERATURA
// -----------------------------------------------------------------------------
// Lee el sensor DS18B20, valida la lectura y decide si el sistema debe
// entrar en modo de control PID o calentamiento manual.
// ============================================================================
void actualizarTemperatura() {
  if (ahora - ultimaTemperatura < INTERVALO_TEMP) {
    return;
  }

#if SIMULAR_SENSOR
  temperaturaValida = true;
  // Simulación simple para pruebas de pantalla y actuadores.
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

// ============================================================================
// BLOQUE 12: CONTROLADOR PID
// -----------------------------------------------------------------------------
// Calcula la salida del controlador en base al error y la derivada,
// usando un filtro de integral y limitando el rango de salida.
// ============================================================================
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

// ============================================================================
// BLOQUE 13: CONTROL DE ACTUADORES
// -----------------------------------------------------------------------------
// Apaga ambos relés en estado inicial o de emergencia, evitando arranques
// accidentales al reiniciar el sistema.
// ============================================================================
void apagarActuadores() {
  digitalWrite(RELE_CALOR, RELE_OFF);
  digitalWrite(RELE_VENT, RELE_OFF);
}

// ============================================================================
// BLOQUE 14: SEGURIDAD DEL SISTEMA
// -----------------------------------------------------------------------------
// Si el nivel de agua es insuficiente o el sensor falla, se apagan los
// actuadores y se detiene el audio para evitar daños.
// ============================================================================
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

// ============================================================================
// BLOQUE 15: CONTROL DE AUDIO
// -----------------------------------------------------------------------------
// Reproduce una alerta sonora cuando la temperatura alcanza el objetivo y
// detiene la reproducción después de cierto tiempo.
// ============================================================================
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

// ============================================================================
// BLOQUE 16: PUBLICACION A UBIDOTS
// -----------------------------------------------------------------------------
// Envía el estado actual del sistema a Ubidots en un único lote para
// reducir tráfico y mantener variables sincronizadas.
// ============================================================================
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

// ============================================================================
// BLOQUE 17: VISUALIZACION EN OLED
// -----------------------------------------------------------------------------
// Muestra de forma legible el estado del sistema, temperatura, nivel de agua,
// relés y WiFi en la pantalla pequeña.
// ============================================================================
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
