// ============================================================================
// SISTEMA DE CONTROL TERMICO Y ALIMENTACION PORCINA CON UBIDOTS
// ============================================================================

#include <Arduino.h>
#include <UbidotsEsp32Mqtt.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobotDFPlayerMini.h>

// ================= CREDENCIALES =============================================
// Reemplaza estos valores localmente. No publiques credenciales reales.
const char *UBIDOTS_TOKEN = "TU_TOKEN_UBIDOTS";
const char *WIFI_SSID = "TU_RED_WIFI";
const char *WIFI_PASS = "TU_CONTRASENA_WIFI";
const char *DEVICE_LABEL = "control-termico-esp32";

Ubidots ubidots(UBIDOTS_TOKEN);

// ================= RELÉS (LÓGICA INVERSA) ====================================
#define RELE_ON LOW
#define RELE_OFF HIGH

// ================= OLED SH1106 ===============================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ================= PINES =====================================================
#define ONE_WIRE_BUS 4
#define RELE_VENT 26
#define RELE_CALOR 25
#define NIVEL_PIN 32
#define DFPLAYER_RX 18
#define DFPLAYER_TX 19

// ================= HARDWARE ==================================================
HardwareSerial mp3Serial(2);
DFRobotDFPlayerMini mp3;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ================= CONFIGURACIÓN TÉRMICA =====================================
float tempActual = 0.0f;
const float SETPOINT = 38.5f;
const float BANDA_PID = 3.0f;
const float HIST_TEMPERATURA = 0.5f;

// Kp: ms/°C, Ki: ms/(°C*s), Kd: ms/(°C/s). Ajustar con pruebas reales.
const float Kp = 40.0f;
const float Ki = 0.2f;
const float Kd = 8.0f;

float errorAnterior = 0.0f;
float integral = 0.0f;
float pidOutput = 0.0f;
unsigned long tiempoPIDAnterior = 0;

// ================= INTERVALOS =================================================
const unsigned long VENTANA_PWM = 5000UL;
const unsigned long INTERVALO_TEMP = 1000UL;
const unsigned long REPETICION_LLAMADO = 60000UL;
const unsigned long DURACION_AUDIO = 30000UL;
const unsigned long INTERVALO_OLED = 250UL;
const unsigned long TIEMPO_VENT_ON = 60000UL;
const unsigned long TIEMPO_VENT_CICLO = 300000UL;
const unsigned long INTERVALO_UBIDOTS = 5000UL;

// ================= ESTADO ====================================================
unsigned long tiempoActual = 0;
unsigned long tiempoUltimaTemp = 0;
unsigned long tiempoInicioPWM = 0;
unsigned long tiempoUltimoLlamado = 0;
unsigned long tiempoInicioAudio = 0;
unsigned long tiempoUltimoOLED = 0;
unsigned long tiempoInicioVentilador = 0;
unsigned long tiempoUltimoUbidots = 0;

bool metaAlcanzada = false;
bool reproduciendoAudio = false;
bool modoPIDActivo = false;
bool temperaturaValida = false;
bool mp3Disponible = false;

void mostrarOLED(const char *estadoStr);
void gestionarAudio();
void calcularPID();
void reiniciarPID();
void actualizarTemperatura();
void apagarActuadores();
void enviarDatosUbidots();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(RELE_VENT, OUTPUT);
  pinMode(RELE_CALOR, OUTPUT);
  digitalWrite(RELE_VENT, RELE_OFF);
  digitalWrite(RELE_CALOR, RELE_OFF);

  pinMode(NIVEL_PIN, INPUT_PULLUP);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 15, "Control Termico");
  u8g2.drawStr(0, 35, "Iniciando...");
  u8g2.sendBuffer();

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  tiempoUltimaTemp = millis();

  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  mp3Disponible = mp3.begin(mp3Serial);
  if (mp3Disponible) {
    mp3.volume(25);
    mp3.EQ(DFPLAYER_EQ_NORMAL);
  }

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setup();

  tiempoInicioVentilador = millis();
  tiempoPIDAnterior = millis();
}

void loop() {
  tiempoActual = millis();

  // La conectividad es secundaria; las protecciones se evalúan localmente.
  if (!ubidots.connected()) {
    ubidots.reconnect();
  }
  ubidots.loop();

  const bool nivelOK = digitalRead(NIVEL_PIN) == LOW;
  if (!nivelOK) {
    apagarActuadores();
    metaAlcanzada = false;
    temperaturaValida = false;
    reiniciarPID();

    if (reproduciendoAudio && mp3Disponible) {
      mp3.stop();
    }
    reproduciendoAudio = false;

    if (tiempoActual - tiempoUltimoOLED >= INTERVALO_OLED) {
      tiempoUltimoOLED = tiempoActual;
      mostrarOLED("NIVEL BAJO");
    }
    if (tiempoActual - tiempoUltimoUbidots >= INTERVALO_UBIDOTS) {
      tiempoUltimoUbidots = tiempoActual;
      enviarDatosUbidots();
    }
    return;
  }

  actualizarTemperatura();

  // Nunca se calienta sin una lectura válida del sensor.
  if (!temperaturaValida) {
    apagarActuadores();
    metaAlcanzada = false;
    reiniciarPID();

    if (tiempoActual - tiempoUltimoOLED >= INTERVALO_OLED) {
      tiempoUltimoOLED = tiempoActual;
      mostrarOLED("ERROR SENSOR");
    }
    if (tiempoActual - tiempoUltimoUbidots >= INTERVALO_UBIDOTS) {
      tiempoUltimoUbidots = tiempoActual;
      enviarDatosUbidots();
    }
    return;
  }

  // Agitador cíclico.
  if (tiempoActual - tiempoInicioVentilador >= TIEMPO_VENT_CICLO) {
    tiempoInicioVentilador = tiempoActual;
  }
  const bool estadoVentilador =
      tiempoActual - tiempoInicioVentilador < TIEMPO_VENT_ON;
  digitalWrite(RELE_VENT, estadoVentilador ? RELE_ON : RELE_OFF);

  // Histéresis: se alcanza el objetivo en SETPOINT y se abandona por debajo.
  if (!metaAlcanzada && tempActual >= SETPOINT) {
    metaAlcanzada = true;
  } else if (metaAlcanzada && tempActual <= SETPOINT - HIST_TEMPERATURA) {
    metaAlcanzada = false;
  }

  gestionarAudio();

  if (metaAlcanzada) {
    digitalWrite(RELE_CALOR, RELE_OFF);
    reiniciarPID();
  } else if (!modoPIDActivo) {
    digitalWrite(RELE_CALOR, RELE_ON);
  } else {
    if (tiempoActual - tiempoInicioPWM >= VENTANA_PWM) {
      tiempoInicioPWM = tiempoActual;
    }
    const bool calefactorON = pidOutput > (tiempoActual - tiempoInicioPWM);
    digitalWrite(RELE_CALOR, calefactorON ? RELE_ON : RELE_OFF);
  }

  if (tiempoActual - tiempoUltimoUbidots >= INTERVALO_UBIDOTS) {
    tiempoUltimoUbidots = tiempoActual;
    enviarDatosUbidots();
  }

  if (tiempoActual - tiempoUltimoOLED >= INTERVALO_OLED) {
    tiempoUltimoOLED = tiempoActual;
    if (reproduciendoAudio) {
      mostrarOLED("LLAMANDO LECHONES");
    } else if (metaAlcanzada) {
      mostrarOLED("TEMP OK");
    } else if (modoPIDActivo) {
      mostrarOLED("CONTROL PID");
    } else {
      mostrarOLED("CALENTANDO");
    }
  }
}

void actualizarTemperatura() {
  if (tiempoActual - tiempoUltimaTemp < INTERVALO_TEMP) {
    return;
  }

  const float tempLeida = sensors.getTempCByIndex(0);
  temperaturaValida = tempLeida != DEVICE_DISCONNECTED_C &&
                      tempLeida > -10.0f && tempLeida < 85.0f;

  if (temperaturaValida) {
    tempActual = tempLeida;
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
  }

  sensors.requestTemperatures();
  tiempoUltimaTemp = tiempoActual;
}

void calcularPID() {
  const unsigned long ahora = tiempoActual;
  float dt = (ahora - tiempoPIDAnterior) / 1000.0f;
  if (dt <= 0.0f || dt > 10.0f) {
    dt = INTERVALO_TEMP / 1000.0f;
  }

  const float error = SETPOINT - tempActual;
  const float integralPropuesta = integral + error * dt;

  // Anti-windup en unidades de error-segundo.
  integral = constrain(integralPropuesta, -50.0f, 50.0f);
  const float derivada = (error - errorAnterior) / dt;
  const float salida = Kp * error + Ki * integral + Kd * derivada;

  pidOutput = constrain(salida, 0.0f, static_cast<float>(VENTANA_PWM));
  errorAnterior = error;
  tiempoPIDAnterior = ahora;
}

void reiniciarPID() {
  integral = 0.0f;
  errorAnterior = 0.0f;
  pidOutput = 0.0f;
  tiempoPIDAnterior = tiempoActual;
}

void apagarActuadores() {
  digitalWrite(RELE_CALOR, RELE_OFF);
  digitalWrite(RELE_VENT, RELE_OFF);
}

void gestionarAudio() {
  if (metaAlcanzada && !reproduciendoAudio &&
      tiempoActual - tiempoUltimoLlamado >= REPETICION_LLAMADO) {
    tiempoUltimoLlamado = tiempoActual;
    tiempoInicioAudio = tiempoActual;
    reproduciendoAudio = true;
    if (mp3Disponible) {
      mp3.play(1);
    }
  }

  if (reproduciendoAudio &&
      tiempoActual - tiempoInicioAudio >= DURACION_AUDIO) {
    if (mp3Disponible) {
      mp3.stop();
    }
    reproduciendoAudio = false;
  }
}

void enviarDatosUbidots() {
  const int estadoCalor = digitalRead(RELE_CALOR) == RELE_ON ? 1 : 0;
  const int estadoVent = digitalRead(RELE_VENT) == RELE_ON ? 1 : 0;
  const int estadoNivel = digitalRead(NIVEL_PIN) == LOW ? 1 : 0;
  int estadoGeneral = 0;

  if (estadoNivel) {
    if (!temperaturaValida) {
      estadoGeneral = 5; // ERROR SENSOR
    } else if (reproduciendoAudio) {
      estadoGeneral = 4;
    } else if (metaAlcanzada) {
      estadoGeneral = 3;
    } else if (modoPIDActivo) {
      estadoGeneral = 2;
    } else {
      estadoGeneral = 1;
    }
  }

  ubidots.add("temperatura", tempActual);
  ubidots.add("setpoint", SETPOINT);
  ubidots.add("rele-calor", estadoCalor);
  ubidots.add("rele-ventilador", estadoVent);
  ubidots.add("nivel-agua", estadoNivel);
  ubidots.add("temperatura-valida", temperaturaValida ? 1 : 0);
  ubidots.add("audio", reproduciendoAudio ? 1 : 0);
  ubidots.add("estado", estadoGeneral);
  ubidots.publish(DEVICE_LABEL);
}

void mostrarOLED(const char *estadoStr) {
  char buf[40];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  snprintf(buf, sizeof(buf), "Temp: %.1f / %.1f C", tempActual, SETPOINT);
  u8g2.drawStr(0, 12, buf);
  snprintf(buf, sizeof(buf), "Estado: %s", estadoStr);
  u8g2.drawStr(0, 28, buf);
  snprintf(buf, sizeof(buf), "Calor: %s | Vent: %s",
           digitalRead(RELE_CALOR) == RELE_ON ? "ON" : "OFF",
           digitalRead(RELE_VENT) == RELE_ON ? "ON" : "OFF");
  u8g2.drawStr(0, 44, buf);
  snprintf(buf, sizeof(buf), "Nivel: %s",
           digitalRead(NIVEL_PIN) == LOW ? "OK" : "BAJO!");
  u8g2.drawStr(0, 60, buf);
  u8g2.sendBuffer();
}
