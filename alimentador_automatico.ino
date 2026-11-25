// ============================================================================
// SISTEMA DE CONTROL TERMICO Y ALIMENTACION PORCINA CON UBIDOTS
// ESP32 + Ubidots MQTT + OLED SH1106 + DS18B20 + DFPlayer Mini
// -----------------------------------------------------------------------------
// Este firmware mantiene la temperatura de un sistema de crianza porcina en un
// rango seguro y configurable, usando un calentador, un ventilador/agitador,
// lecturas de sensor, alarma sonora y telemetría remota.
// -----------------------------------------------------------------------------
// El objetivo principal es controlar la temperatura del agua/caldera sin dejar
// de monitorear nivel, sensor, estado de red y seguridad del sistema.
// ============================================================================

// ============================================================================
// BLOQUE 1: LIBRERIAS Y DEPENDENCIAS
// -----------------------------------------------------------------------------
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
TU_CONTRASENA_WIFI
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
constexpr float SETPOINT = 38.0f;
constexpr float BANDA_PID = 0.5f;
constexpr float HIST_TEMPERATURA = 2.0f;
constexpr float TOLERANCIA_META = 0.1f;
constexpr float KP = 40.0f;
constexpr float KI = 0.2f;
constexpr float KD = 8.0f;
constexpr float INTEGRAL_MIN = -50.0f;
constexpr float INTEGRAL_MAX = 50.0f;

// ============================================================================
// BLOQUE 5: VARIABLES DE ESTADO Y CONTROL
// -----------------------------------------------------------------------------
float tempActual = 0.0f;
float errorAnterior = 0.0f;
float integral = 0.0f;
float pidOutput = 0.0f;

bool metaAlcanzada = false;
bool reproduciendoAudio = false;
bool modoPIDActivo = false;
bool temperaturaValida = false;
bool mp3Disponible = false;
bool audioCicloIniciado = false;
bool audioBloqueadoSobreTemp = false;

// Contador para evitar que un pico de ruido eléctrico del relé dispare
// inmediatamente el estado de seguridad por una sola lectura fallida.
uint8_t fallosSensorConsecutivos = 0;

// ============================================================================
// BLOQUE 6: INTERVALOS DE TIMING Y DELAY
// -----------------------------------------------------------------------------
constexpr unsigned long VENTANA_PWM = 5000UL;
constexpr unsigned long INTERVALO_TEMP = 800UL;
constexpr unsigned long REPETICION_LLAMADO = 60000UL;
constexpr unsigned long DURACION_AUDIO = 30000UL;
constexpr unsigned long INTERVALO_OLED = 250UL;
constexpr unsigned long INTERVALO_UBIDOTS = 5000UL;
constexpr unsigned long TIMEOUT_RECONEXION = 10000UL;
constexpr unsigned long TIEMPO_FILTRO_NIVEL = 500UL;

constexpr unsigned long VENTILADOR_ON = 20000UL;
constexpr unsigned long VENTILADOR_OFF = 40000UL;

unsigned long ahora = 0;
unsigned long ultimaTemperatura = 0;
unsigned long inicioPWM = 0;
unsigned long ultimoLlamado = 0;
unsigned long inicioAudio = 0;
unsigned long ultimaOLED = 0;
unsigned long ultimoUbidots = 0;
unsigned long ultimoIntentoWiFi = 0;
unsigned long ultimoPID = 0;
unsigned long inicioVentilador = 0;

// NUEVO: variable para identificar el estado anterior de la meta.
// Permite reiniciar el ciclo del ventilador cuando se alcanzan los 38 °C
// o cuando la temperatura desciende hasta los 36 °C.
bool estadoAnteriorMeta = false;

// ============================================================================
// BLOQUE 7: PROTOTIPOS DE FUNCIONES
// -----------------------------------------------------------------------------
void mostrarOLED();
void gestionarAudio();
void gestionarVentilador();
void calcularPID();
void reiniciarPID();
void actualizarTemperatura();
void apagarActuadores();
void procesarSeguridad();
void enviarDatosUbidots();
void actualizarComunicaciones();
void detenerAudio();
bool nivelAguaOK();

// ============================================================================
// BLOQUE 8: SETUP
// -----------------------------------------------------------------------------
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

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  // DFPlayer Mini sin ACK bloqueante.
  // La opción false,false evita cierres largos si no responde el módulo.
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  mp3Disponible = mp3.begin(mp3Serial, false, false);
  if (mp3Disponible) {
    mp3.volume(30);
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
  inicioVentilador = ahora;
}

// ============================================================================
// BLOQUE 9: LOOP PRINCIPAL
// -----------------------------------------------------------------------------
void loop() {
  ahora = millis();
  actualizarComunicaciones();

  // Se actualiza SIEMPRE la temperatura antes de tomar decisiones.
  actualizarTemperatura();

  // Verificación de seguridad de nivel de agua o error en el sensor.
  if (!nivelAguaOK() || !temperaturaValida) {
    procesarSeguridad();
    return;
  }

  // Histéresis: se activa al llegar a 38.0°C y permanece activa hasta bajar a
  // 36.0°C. Esto evita que el sistema oscile alrededor del punto de ajuste.
  if (!metaAlcanzada && tempActual >= (SETPOINT - TOLERANCIA_META)) {
    metaAlcanzada = true;
    inicioVentilador = ahora;
  } else if (metaAlcanzada && tempActual <= (SETPOINT - HIST_TEMPERATURA)) {
    metaAlcanzada = false;
    inicioVentilador = ahora;
  }

  gestionarVentilador();
  gestionarAudio();

  // Control del elemento calefactor.
  // Cuando la meta está alcanzada, se apaga.
  // Cuando se encuentra alejado del objetivo, se calienta a máxima potencia.
  // En la zona cercana al setpoint, se usa el PID para regular con PWM.
  if (metaAlcanzada) {
    digitalWrite(RELE_CALOR, RELE_OFF);
    reiniciarPID();
  } else if (!modoPIDActivo) {
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

// ============================================================================
// BLOQUE 10: ACTUALIZACION DE COMUNICACIONES
// -----------------------------------------------------------------------------
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

  static unsigned long ultimoIntentoMQTT = 0;

  if (!ubidots.connected()) {
    if (ahora - ultimoIntentoMQTT >= TIMEOUT_RECONEXION) {
      ultimoIntentoMQTT = ahora;
      Serial.println("Reconectando a Ubidots MQTT...");
      ubidots.reconnect();
    }
    return;
  }
  ubidots.loop();
}

// ============================================================================
// BLOQUE 11: ACTUALIZACION DE TEMPERATURA
// -----------------------------------------------------------------------------
void actualizarTemperatura() {
  if (ahora - ultimaTemperatura < INTERVALO_TEMP) {
    return;
  }

  const float leida = sensors.getTempCByIndex(0);
  // Se exige que la lectura sea distinta de -127°C y 85°C (error de conversión
  // por caída de voltaje al activar relés) y se toleran hasta 2 fallos seguidos.
  const bool lecturaActualOK = (leida != DEVICE_DISCONNECTED_C) &&
                               (leida != 85.0f) &&
                               (leida > -10.0f) && (leida < 85.0f);

  if (lecturaActualOK) {
    fallosSensorConsecutivos = 0;
    temperaturaValida = true;
    tempActual = leida;
  } else {
    if (fallosSensorConsecutivos < 255) {
      fallosSensorConsecutivos++;
    }
    if (fallosSensorConsecutivos >= 3) {
      temperaturaValida = false;
    }
    Serial.printf("Advertencia DS18B20 (%.1f C) - fallo %u/3\n",
                  leida, fallosSensorConsecutivos);
  }
  sensors.requestTemperatures();

  if (temperaturaValida) {
    // El PID interviene solamente durante el calentamiento, entre 37.5 y 38.0 °C.
    // Por debajo de 37.5 °C se conserva el calentamiento a maxima potencia.
    // Al llegar a 38.0 °C, la histeresis apaga el calentador hasta 36.0 °C.
    if (!metaAlcanzada && tempActual >= (SETPOINT - BANDA_PID) &&
        tempActual < (SETPOINT - TOLERANCIA_META)) {
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
// BLOQUE 13: APAGADO DE ACTUADORES
// -----------------------------------------------------------------------------
void apagarActuadores() {
  digitalWrite(RELE_CALOR, RELE_OFF);
  digitalWrite(RELE_VENT, RELE_OFF);
}

// ============================================================================
// BLOQUE 14: ESTADO DE SEGURIDAD
// -----------------------------------------------------------------------------
bool nivelAguaOK() {
  static unsigned long tiempoNivelAlto = 0;

  if (digitalRead(NIVEL_PIN) == LOW) {
    tiempoNivelAlto = 0;
    return true;
  }

  if (tiempoNivelAlto == 0) {
    tiempoNivelAlto = ahora;
  }

  // Solo reporta nivel bajo si la señal se mantiene en HIGH más de 500 ms,
  // evitando falsos bloqueos por oleaje o ruido electromagnético del ventilador.
  return (ahora - tiempoNivelAlto) < TIEMPO_FILTRO_NIVEL;
}

void procesarSeguridad() {
  apagarActuadores();
  modoPIDActivo = false;
  reiniciarPID();
  detenerAudio();

  // Se sigue enviando telemetría y actualizando la pantalla para que el estado
  // de fallo siga siendo visible.
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
// BLOQUE 15: ALERTA SONORA
// -----------------------------------------------------------------------------
void detenerAudio() {
  if (reproduciendoAudio && mp3Disponible) {
    mp3.stop();
  }
  reproduciendoAudio = false;
}

void gestionarAudio() {
  // Si supera 39 °C, se detiene el audio y queda bloqueado hasta bajar a 36 °C.
  if (tempActual > 39.0f) {
    audioBloqueadoSobreTemp = true;
    detenerAudio();
    audioCicloIniciado = false;
    return;
  }

  // Al bajar a 36 °C termina el ciclo de audio y se habilita el siguiente.
  if (tempActual <= (SETPOINT - HIST_TEMPERATURA)) {
    detenerAudio();
    audioCicloIniciado = false;
    audioBloqueadoSobreTemp = false;
    return;
  }

  // El audio solamente funciona durante la etapa de histeresis (38 a 36 °C).
  if (!metaAlcanzada || audioBloqueadoSobreTemp) {
    detenerAudio();
    return;
  }

  // Al alcanzar 38 °C comienza inmediatamente un ciclo: 30 s con audio
  // y 30 s en silencio. El ciclo se repite mientras siga entre 36 y 39 °C.
  if (!audioCicloIniciado) {
    inicioAudio = ahora;
    audioCicloIniciado = true;
  }

  const unsigned long faseAudio = (ahora - inicioAudio) % REPETICION_LLAMADO;
  const bool debeSonar = faseAudio < DURACION_AUDIO;

  if (debeSonar && !reproduciendoAudio) {
    reproduciendoAudio = true;
    if (mp3Disponible) {
      mp3.loop(1);
    }
  } else if (!debeSonar && reproduciendoAudio) {
    detenerAudio();
  }
}

// ============================================================================
// BLOQUE 16: VENTILADOR / AGITADOR
// -----------------------------------------------------------------------------
// El ventilador tiene dos modos principales:
// - Cuando la temperatura está por debajo de 38°C: permanece encendido.
// - Al alcanzar 38°C: inicia ciclos de 20 segundos encendido y 40 apagado.
// - Mientras la temperatura esté entre 38°C y 36°C: continúa con los ciclos.
// - Al llegar a 36°C: queda encendido constantemente hasta llegar a 38°C.
// -----------------------------------------------------------------------------
void gestionarVentilador() {

  // Verificación de seguridad: si falta agua o falla el sensor,
  // el ventilador debe permanecer apagado.
  if (!temperaturaValida || !nivelAguaOK()) {
    digitalWrite(RELE_VENT, RELE_OFF);
    estadoAnteriorMeta = false;
    return;
  }

  // MODO 1: Temperatura inferior a 38°C.
  // El ventilador permanece encendido constantemente.
  if (!metaAlcanzada) {
    digitalWrite(RELE_VENT, RELE_ON);

    // Se mantiene actualizado el temporizador para iniciar el ciclo
    // desde cero cuando se alcancen nuevamente los 38°C.
    inicioVentilador = ahora;

    estadoAnteriorMeta = false;
    return;
  }

  // MODO 2: Temperatura entre 38°C y 36°C.
  // Al alcanzar los 38°C comienza el ciclo de 20 segundos ON y 40 segundos OFF.
  // El ciclo se repite continuamente mientras la meta permanezca alcanzada.

  // Se detecta el cambio de temperatura al estado de meta alcanzada.
  // Se reinicia el temporizador para comenzar con 20 segundos encendido.
  if (!estadoAnteriorMeta) {
    inicioVentilador = ahora;
    estadoAnteriorMeta = true;
  }

  const unsigned long cicloTotal = VENTILADOR_ON + VENTILADOR_OFF;

  // Posición dentro del ciclo de 60 s. Con módulo, una pausa del loop no
  // reinicia el ciclo ni se pierde la ventana de encendido.
  const unsigned long tiempoCiclo = (ahora - inicioVentilador) % cicloTotal;

  // Primeros 20 segundos: ventilador encendido.
  if (tiempoCiclo < VENTILADOR_ON) {
    digitalWrite(RELE_VENT, RELE_ON);
  }
  // Siguientes 40 segundos: ventilador apagado.
  else {
    digitalWrite(RELE_VENT, RELE_OFF);
  }
}

// ============================================================================
// BLOQUE 17: PUBLICACION DE DATOS A UBIDOTS
// -----------------------------------------------------------------------------
void enviarDatosUbidots() {
  if (WiFi.status() != WL_CONNECTED || !ubidots.connected()) {
    return;
  }

  const int nivel = nivelAguaOK() ? 1 : 0;
  const int estadoCalor = (digitalRead(RELE_CALOR) == RELE_ON) ? 1 : 0;
  const int estadoVent = (digitalRead(RELE_VENT) == RELE_ON) ? 1 : 0;
  int estado = 0;

  if (nivel) {
    if (!temperaturaValida) estado = 5;
    else if (reproduciendoAudio) estado = 4;
    else if (metaAlcanzada) estado = 3;
    else if (modoPIDActivo) estado = 2;
    else estado = 1;
  }

  // Se envía primero el estado de temperatura y relés y luego el estado de
  // seguridad y variables auxiliares. Esta separación ayudó a mantener la
  // versión funcional y estable del sistema.
  ubidots.add("temperatura", tempActual);
  ubidots.add("setpoint", SETPOINT);
  ubidots.add("rele-calor", estadoCalor);
  ubidots.add("rele-ventilador", estadoVent);
  ubidots.publish(DEVICE_LABEL);

  delay(100);

  ubidots.add("nivel-agua", nivel);
  ubidots.add("temperatura-valida", temperaturaValida ? 1 : 0);
  ubidots.add("audio", reproduciendoAudio ? 1 : 0);
  ubidots.add("estado", estado);
  ubidots.publish(DEVICE_LABEL);
}

// ============================================================================
// BLOQUE 18: VISUALIZACION EN OLED
// -----------------------------------------------------------------------------
void mostrarOLED() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tf);

  const bool nivelOK = nivelAguaOK();
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
  snprintf(buf, sizeof(buf), "CALOR:%s VENT:%s",
           calorON ? "ON" : "OFF", ventON ? "ON" : "OFF");
  u8g2.drawStr(0, 52, buf);
  snprintf(buf, sizeof(buf), "WIFI: %s", wifiOK ? "OK" : "OFF");
  u8g2.drawStr(0, 62, buf);
  u8g2.sendBuffer();
}
