// ============================================================================
// SISTEMA DE CONTROL TÉRMICO Y ALIMENTACIÓN PORCINA CON INTEGRACIÓN UBIDOTS
// ============================================================================

#include <UbidotsEsp32Mqtt.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobotDFPlayerMini.h>

// ========= CREDENCIALES Y CONFIGURACIÓN UBIDOTS =================
const char *UBIDOTS_TOKEN = "Token"; // Token de autenticación Ubidots
const char *WIFI_SSID = "Red";                                     // SSID de la red Wi-Fi
const char *WIFI_PASS = "Wi Fi";                                 // Contraseña de la red Wi-Fi
const char *DEVICE_LABEL = "control-termico-esp32";                 // Etiqueta del dispositivo en la nube

Ubidots ubidots(UBIDOTS_TOKEN);

// ================= RELÉS (Lógica Inversa) =================
#define RELE_ON LOW   // Relés activos con nivel bajo
#define RELE_OFF HIGH // Relés desactivados con nivel alto

// ================= OLED SH1106 (Pantalla I2C) =================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ================= ASIGNACIÓN DE PINES =================
#define ONE_WIRE_BUS 4  // Bus OneWire para sonda DS18B20
#define RELE_VENT 26    // Control del agitador/ventilador de 12V
#define RELE_CALOR 25   // Control del calefactor de 110V
#define NIVEL_PIN 32    // Sensor de nivel tipo flotador (Switch con Pull-Up)

#define DFPLAYER_RX 18  // TX del DFPlayer -> RX del ESP32
#define DFPLAYER_TX 19  // RX del DFPlayer -> TX del ESP32 / Salida de Audio/Buzzer

// ================= OBJETOS DE HARDWARE =================
HardwareSerial mp3Serial(2);         // Serial2 para comunicación con módulo de audio
DFRobotDFPlayerMini mp3;             // Objeto reproductor MP3

OneWire oneWire(ONE_WIRE_BUS);        // Instancia bus OneWire
DallasTemperature sensors(&oneWire);  // Control del sensor térmico DS18B20

// ================= VARIABLES DEL SISTEMA =================
float tempActual = 0.0;
float setpoint = 30.0;                 // Temperatura objetivo programada en °C

const float BANDA_PID = 3.0;           // Rango de acción para iniciar PID (3°C antes del setpoint)

// Parámetros de sintonización PID
float Kp = 40.0;
float Ki = 0.2;
float Kd = 8.0;

float errorAnterior = 0.0;
float integral = 0.0;
float pidOutput = 0.0;

// ================= TIEMPOS Y CONSTANTES (ms) =================
const unsigned long VENTANA_PWM = 5000;         // Ventana del ciclo de trabajo PWM (5 segundos)
const unsigned long REPETICION_LLAMADO = 60000;   // Intervalo entre llamadas de audio (1 minuto)
const unsigned long DURACION_AUDIO = 30000;       // Duración de la reproducción de audio (30 segundos)
const unsigned long INTERVALO_OLED = 250;        // Frecuencia de refresco de pantalla (250 ms)

const unsigned long TIEMPO_VENT_ON = 60000;      // Tiempo encendido del agitador (1 minuto)
const unsigned long TIEMPO_VENT_CICLO = 300000;  // Período total del ciclo del agitador (5 minutos)

const unsigned long INTERVALO_UBIDOTS = 5000;    // Frecuencia de transmisión a la nube (5 segundos)

// ================= MARCAS DE TIEMPO (millis) =================
unsigned long tiempoActual = 0;
unsigned long tiempoUltimaTemp = 0;
unsigned long tiempoInicioPWM = 0;
unsigned long tiempoUltimoLlamado = 0;
unsigned long tiempoInicioAudio = 0;
unsigned long tiempoUltimoOLED = 0;
unsigned long tiempoInicioVentilador = 0;
unsigned long tiempoUltimoUbidots = 0;

// ================= BANDERAS DE ESTADO =================
bool metaAlcanzada = false;
bool reproduciendoAudio = false;
bool modoPIDActivo = false;

// ================= DECLARACIÓN DE FUNCIONES =================
void mostrarOLED(const char* estadoStr);
void gestionarAudio();
void calcularPID();
void enviarDatosUbidots();

// ======================================================
// CONFIGURACIÓN INICIAL (SETUP)
// ======================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // Inicializar I2C para pantalla (SDA=21, SCL=22)

  // Configurar pines de relés y apagarlos por seguridad al arrancar
  pinMode(RELE_VENT, OUTPUT);
  pinMode(RELE_CALOR, OUTPUT);
  digitalWrite(RELE_VENT, RELE_OFF);
  digitalWrite(RELE_CALOR, RELE_OFF);

  // Configurar entrada con pull-up para el flotador de nivel
  pinMode(NIVEL_PIN, INPUT_PULLUP);

  // Inicialización de la pantalla OLED
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "Control Termico PID");
  u8g2.drawStr(0, 35, "Conectando WiFi...");
  u8g2.sendBuffer();

  // Conexión al servicio MQTT de Ubidots vía Wi-Fi
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setup();

  // Inicializar sensor de temperatura
  sensors.begin();
  sensors.setWaitForConversion(false); // Lectura no bloqueante
  sensors.requestTemperatures();

  // Inicializar comunicación con DFPlayer Mini
  mp3Serial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  if (mp3.begin(mp3Serial)) {
    mp3.volume(25);
    mp3.EQ(DFPLAYER_EQ_NORMAL);
  }

  tiempoInicioVentilador = millis(); // Registrar marca inicial del temporizador
}

// ======================================================
// BUCLE PRINCIPAL (LOOP)
// ======================================================
void loop() {
  tiempoActual = millis();

  // Mantener la conexión con Ubidots activa
  if (!ubidots.connected()) {
    ubidots.reconnect();
  }
  ubidots.loop();

  // ----------------------------------------------------
  // 1. EVALUACIÓN DEL SENSOR DE NIVEL (Seguridad)
  // ----------------------------------------------------
  bool nivelOK = (digitalRead(NIVEL_PIN) == LOW); // LOW indica tanque con fluido

  if (!nivelOK) {
    // Apagar potencia si el nivel es bajo
    digitalWrite(RELE_CALOR, RELE_OFF);
    digitalWrite(RELE_VENT, RELE_OFF);

    if (reproduciendoAudio) {
      mp3.stop();
      reproduciendoAudio = false;
    }

    metaAlcanzada = false;
    modoPIDActivo = false;
    integral = 0; // Limpiar acumulador integral PID

    if (tiempoActual - tiempoUltimoOLED >= INTERVALO_OLED) {
      tiempoUltimoOLED = tiempoActual;
      mostrarOLED("NIVEL BAJO");
    }

    if (tiempoActual - tiempoUltimoUbidots >= INTERVALO_UBIDOTS) {
      enviarDatosUbidots();
      tiempoUltimoUbidots = tiempoActual;
    }

    return; // Interrumpe la ejecución del bucle si no hay fluido suficiente
  }

  // ----------------------------------------------------
  // 2. CONTROL CÍCLICO DEL AGITADOR/VENTILADOR
  // ----------------------------------------------------
  if (tiempoActual - tiempoInicioVentilador >= TIEMPO_VENT_CICLO) {
    tiempoInicioVentilador = tiempoActual; // Reinicia ventana de 5 min
  }

  bool estadoVentilador = (tiempoActual - tiempoInicioVentilador < TIEMPO_VENT_ON);
  digitalWrite(RELE_VENT, estadoVentilador ? RELE_ON : RELE_OFF);

  // ----------------------------------------------------
  // 3. LECTURA DE TEMPERATURA Y CÁLCULO PID
  // ----------------------------------------------------
  if (tiempoActual - tiempoUltimaTemp >= 1000) { // Cada 1 segundo
    float tempLeida = sensors.getTempCByIndex(0);

    if (tempLeida > -10.0 && tempLeida < 85.0) { // Filtro de errores de bus
      tempActual = tempLeida;
    }

    sensors.requestTemperatures(); // Solicitar próxima lectura
    tiempoUltimaTemp = tiempoActual;

    // Activar PID solo si está dentro del rango seguro (Setpoint - BANDA_PID)
    if (tempActual >= (setpoint - BANDA_PID)) {
      modoPIDActivo = true;
      calcularPID();
    } else {
      modoPIDActivo = false;
      integral = 0; // Desactivar PID y calentar a máxima potencia
    }
  }

  // ----------------------------------------------------
  // 4. LÓGICA DE CONTROL DE META TÉRMICA
  // ----------------------------------------------------
  if (tempActual >= setpoint) {
    metaAlcanzada = true;
  } else {
    metaAlcanzada = false;
  }

  // ----------------------------------------------------
  // 5. GESTIÓN DE AUDIO DE LLAMADO
  // ----------------------------------------------------
  gestionarAudio();

  // ----------------------------------------------------
  // 6. CONTROL PWM DE LA RESISTENCIA CALEFACTORA
  // ----------------------------------------------------
  if (metaAlcanzada) {
    digitalWrite(RELE_CALOR, RELE_OFF); // Apagar si llegó a la temperatura final
  } else if (!modoPIDActivo) {
    digitalWrite(RELE_CALOR, RELE_ON);  // Encendido continuo si está lejos de la meta
  } else {
    // Control PWM continuo en ventana de tiempo suave
    if (tiempoActual - tiempoInicioPWM >= VENTANA_PWM) {
      tiempoInicioPWM = tiempoActual;
    }

    bool estadoSSR = (pidOutput > (tiempoActual - tiempoInicioPWM));
    digitalWrite(RELE_CALOR, estadoSSR ? RELE_ON : RELE_OFF);
  }

  // ----------------------------------------------------
  // 7. ENVÍO PERIÓDICO A UBIDOTS
  // ----------------------------------------------------
  if (tiempoActual - tiempoUltimoUbidots >= INTERVALO_UBIDOTS) {
    enviarDatosUbidots();
    tiempoUltimoUbidots = tiempoActual;
  }

  // ----------------------------------------------------
  // 8. ACTUALIZACIÓN DE PANTALLA OLED
  // ----------------------------------------------------
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

// ======================================================
// TRANSMISIÓN DE DATOS A LA NUBE (UBIDOTS)
// ======================================================
void enviarDatosUbidots() {
  int estadoCalor = (digitalRead(RELE_CALOR) == RELE_ON) ? 1 : 0;
  int estadoVent = (digitalRead(RELE_VENT) == RELE_ON) ? 1 : 0;
  int estadoNivel = (digitalRead(NIVEL_PIN) == LOW) ? 1 : 0;

  // Codificación del estado global del sistema:
  // 0 = NIVEL BAJO, 1 = CALENTANDO, 2 = CONTROL PID, 3 = TEMP OK, 4 = LLAMANDO LECHONES
  int estadoGeneral;

  if (!estadoNivel) {
    estadoGeneral = 0;
  } else if (reproduciendoAudio) {
    estadoGeneral = 4;
  } else if (metaAlcanzada) {
    estadoGeneral = 3;
  } else if (modoPIDActivo) {
    estadoGeneral = 2;
  } else {
    estadoGeneral = 1;
  }

  // Cargar variables a la pila de publicación MQTT
  ubidots.add("temperatura", tempActual);
  ubidots.add("setpoint", setpoint);
  ubidots.add("rele-calor", estadoCalor);
  ubidots.add("rele-ventilador", estadoVent);
  ubidots.add("nivel-agua", estadoNivel);
  ubidots.add("audio", reproduciendoAudio);
  ubidots.add("estado", estadoGeneral);

  ubidots.publish(DEVICE_LABEL); // Publicar al broker MQTT
}

// ======================================================
// REPRODUCCIÓN AUTOMÁTICA DE AUDIO
// ======================================================
void gestionarAudio() {
  if (metaAlcanzada && !reproduciendoAudio && (tiempoActual - tiempoUltimoLlamado >= REPETICION_LLAMADO)) {
    tiempoUltimoLlamado = tiempoActual;
    tiempoInicioAudio = tiempoActual;
    reproduciendoAudio = true;

    mp3.play(1); // Reproducir pista 0001.mp3 en la tarjeta MicroSD
  }

  if (reproduciendoAudio && (tiempoActual - tiempoInicioAudio >= DURACION_AUDIO)) {
    mp3.stop(); // Detener reproducción tras 30 segundos
    reproduciendoAudio = false;
  }
}

// ======================================================
// INTERFAZ GRÁFICA EN PANTALLA OLED
// ======================================================
void mostrarOLED(const char* estadoStr) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  char buf[40];
  snprintf(buf, sizeof(buf), "Temp: %.1f / %.1f C", tempActual, setpoint);
  u8g2.drawStr(0, 12, buf);

  snprintf(buf, sizeof(buf), "Estado: %s", estadoStr);
  u8g2.drawStr(0, 28, buf);

  snprintf(buf, sizeof(buf), "Calor: %s | Vent: %s",
           (digitalRead(RELE_CALOR) == RELE_ON) ? "ON" : "OFF",
           (digitalRead(RELE_VENT) == RELE_ON) ? "ON" : "OFF");
  u8g2.drawStr(0, 44, buf);

  snprintf(buf, sizeof(buf), "Nivel: %s", (digitalRead(NIVEL_PIN) == LOW) ? "OK" : "BAJO!");
  u8g2.drawStr(0, 60, buf);

  u8g2.sendBuffer();
}

// ======================================================
// CÁLCULO DEL ALGORITMO PID
// ======================================================
void calcularPID() {
  float error = setpoint - tempActual;

  // Término Proporcional
  float P = Kp * error;

  // Término Integral con limitador Anti-Windup (+/- 50)
  integral += error;
  if (integral > 50) integral = 50;
  if (integral < -50) integral = -50;
  float I = Ki * integral;

  // Término Derivativo
  float D = Kd * (error - errorAnterior);
  errorAnterior = error;

  // Suma total y acotamiento del valor dentro del tiempo de la ventana PWM
  float salida = P + I + D;
  if (salida > VENTANA_PWM) salida = VENTANA_PWM;
  if (salida < 0) salida = 0;

  pidOutput = salida;
}
