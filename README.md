# Alimentador Automático

Sistema de control térmico y alimentación porcina con ESP32, sensor DS18B20, relés, pantalla OLED SH1106 y telemetría a Ubidots.

## Descripción

Este proyecto automatiza la temperatura de un sistema de crianza porcina mediante un calentador y un ventilador. El sistema:

- mantiene una temperatura objetivo de **38.5 °C**,
- activa control PID para estabilizar la temperatura,
- controla un ventilador con ciclo periódico,
- monitorea el nivel del agua,
- reproduce una alerta sonora con DFPlayer Mini cuando se alcanza la temperatura objetivo,
- envía variables a Ubidots para supervisión remota,
- muestra el estado del sistema en una pantalla OLED.

## Archivos

- `alimentador_automatico.ino`: código principal del proyecto.
- `README.md`: documentación del sistema.

## Hardware principal

- ESP32
- Sensor de temperatura DS18B20
- Relé para calentador
- Relé para ventilador
- Sensor de nivel de agua
- Pantalla OLED SH1106
- DFPlayer Mini
- WiFi + Ubidots

## Pines utilizados

| Función | GPIO |
|---|---:|
| DS18B20 | 4 |
| Relé ventilador | 26 |
| Relé calefactor | 25 |
| Sensor de nivel | 32 |
| DFPlayer RX | 18 |
| DFPlayer TX | 19 |
| OLED SDA | 21 |
| OLED SCL | 22 |

## Variables clave

```cpp
const float SETPOINT = 38.5f;
const float BANDA_PID = 3.0f;
const float HIST_TEMPERATURA = 0.5f;

const float Kp = 40.0f;
const float Ki = 0.2f;
const float Kd = 8.0f;
```

El sistema alcanza la temperatura objetivo cuando el sensor registra **38.5 °C** o más. Después, el calentador se apaga y vuelve a activarse cuando la temperatura desciende hasta **38.0 °C**, debido a la histéresis configurada de 0.5 °C.

## Lógica del sistema

### 1. Protección por nivel

Si el sensor de nivel indica que el agua está baja:

- se apagan el calentador y el ventilador,
- se reinicia el PID,
- si se está reproduciendo audio, se detiene,
- se muestra el estado `NIVEL BAJO` en la pantalla,
- se publica el estado en Ubidots.

### 2. Error del sensor

Si la temperatura es inválida o el sensor no responde:

- se apaga el calentador,
- se reinicia el control PID,
- se muestra `ERROR SENSOR` en OLED,
- se envía un estado de error a Ubidots.

### 3. Control del ventilador

El ventilador funciona en un ciclo:

- se activa durante `TIEMPO_VENT_ON = 60000 ms`,
- se desactiva durante el resto del ciclo `TIEMPO_VENT_CICLO = 300000 ms`.

Esto evita un uso continuo excesivo y mejora la circulación del aire.

### 4. Control de temperatura

Cuando la temperatura alcanza el setpoint de **38.5 °C**:

- `metaAlcanzada = true`,
- el calentador se apaga,
- el PID se reinicia,
- la alarma de audio puede activarse cada cierto intervalo.

Cuando la temperatura baja hasta **38.0 °C** o menos, debido a la histéresis de 0.5 °C:

- `metaAlcanzada = false`,
- el sistema vuelve a calentar.

### 5. Control PID

El calentador usa un controlador PID para ajustar la potencia de calentamiento:

- `Kp`: corrección proporcional,
- `Ki`: corrección integral,
- `Kd`: corrección derivativa.

El control PID comienza cuando la temperatura está dentro de `BANDA_PID = 3.0 °C` del setpoint, es decir, desde **35.5 °C**. Se aplica anti-windup para limitar la integral y evitar excesos de calentamiento.

### 6. Alarma de audio

Cuando la temperatura objetivo se alcanza y no se está reproduciendo ningún audio:

- se dispara el DFPlayer,
- se reproduce el archivo 1,
- la reproducción se detiene después del tiempo configurado.

## Pseudocódigo

```text
INICIO

  Configurar ESP32
  Configurar OLED
  Configurar relés
  Configurar sensor de temperatura
  Configurar sensor de nivel
  Configurar DFPlayer
  Conectar WiFi y Ubidots

  Apagar calentador
  Apagar ventilador

  SETPOINT = 38.5 °C
  HISTÉRESIS = 0.5 °C

  REPETIR SIEMPRE:

    Leer nivel
    SI nivel es bajo:
      apagar calentador
      apagar ventilador
      detener audio
      reiniciar PID
      mostrar "NIVEL BAJO"
      enviar datos a Ubidots
      continuar
    FIN SI

    Leer temperatura
    SI temperatura no es válida:
      apagar calentador
      reiniciar PID
      mostrar "ERROR SENSOR"
      enviar datos a Ubidots
      continuar
    FIN SI

    SI pasó el tiempo del ciclo del ventilador:
      reiniciar ciclo
    FIN SI

    SI está dentro del tiempo de encendido del ventilador:
      encender ventilador
    SINO:
      apagar ventilador
    FIN SI

    SI temperatura >= SETPOINT:
      metaAlcanzada = VERDADERO
    SINO SI metaAlcanzada == VERDADERO Y temperatura <= SETPOINT - HISTÉRESIS:
      metaAlcanzada = FALSO
    FIN SI

    SI metaAlcanzada == VERDADERO:
      apagar calentador
      reiniciar PID
    SINO SI temperatura < SETPOINT - BANDA_PID:
      encender calentador
    SINO:
      activar PID
      calcular salida PID
      SI salidaPID indica encender:
        encender calentador
      SINO:
        apagar calentador
      FIN SI
    FIN SI

    SI metaAlcanzada == VERDADERO Y no se está reproduciendo audio:
      reproducir audio cada 60 s
    FIN SI

    SI se está reproduciendo audio por más de 30 s:
      detener audio
    FIN SI

    SI han pasado 250 ms:
      mostrar el estado en OLED
    FIN SI

    SI han pasado 5 s:
      enviar datos a Ubidots
    FIN SI

  FIN REPETIR

FIN
```

## Estados del sistema

```text
0 = Sistema detenido o nivel bajo
1 = Calentando
2 = Control PID
3 = Temperatura objetivo alcanzada
4 = Reproduciendo llamado
5 = Error del sensor
```

## Configuración inicial

Antes de cargar el programa, actualiza estas variables con tus credenciales reales:

```cpp
const char *UBIDOTS_TOKEN = "TU_TOKEN_UBIDOTS";
const char *WIFI_SSID = "TU_RED_WIFI";
const char *WIFI_PASS = "TU_CONTRASENA_WIFI";
const char *DEVICE_LABEL = "control-termico-esp32";
```

No publiques tokens, contraseñas ni secretos reales en GitHub.

## Bibliotecas requeridas

- `Arduino.h`
- `UbidotsEsp32Mqtt.h`
- `U8g2lib.h`
- `Wire.h`
- `OneWire.h`
- `DallasTemperature.h`
- `DFRobotDFPlayerMini.h`

## Seguridad

Este proyecto controla cargas eléctricas. Para una instalación real, se recomienda usar:

- fusibles adecuados,
- protección térmica,
- relés y contacto apropiados para la carga,
- conexiones correctamente aisladas,
- revisión técnica antes de operar en producción.

## Licencia

Proyecto desarrollado para uso técnico y educativo en aplicaciones de automatización y monitoreo.
