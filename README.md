# Alimentador automático con control térmico

Proyecto para ESP32 con control de temperatura mediante DS18B20, calefactor, agitador, sensor de nivel, DFPlayer Mini, pantalla OLED SH1106 y telemetría MQTT hacia Ubidots.

## Archivos

- `alimentador_automatico.ino`: código fuente para Arduino IDE.

## Configuración

Antes de cargar el programa, edita en el archivo `.ino` estas constantes con tus propios datos:

```cpp
const char *UBIDOTS_TOKEN = "TU_TOKEN_UBIDOTS";
const char *WIFI_SSID = "TU_RED_WIFI";
const char *WIFI_PASS = "TU_CONTRASENA_WIFI";
```

No publiques tokens reales, contraseñas Wi-Fi ni otros secretos en el repositorio.

## Bibliotecas requeridas

- Ubidots ESP32 MQTT
- Arduino-ESP32
- U8g2
- OneWire
- DallasTemperature
- DFRobotDFPlayerMini

## Pines utilizados

| Función | GPIO |
|---|---:|
| DS18B20 | 4 |
| Relé agitador/ventilador | 26 |
| Relé calefactor | 25 |
| Sensor de nivel | 32 |
| DFPlayer RX | 18 |
| DFPlayer TX | 19 |
| OLED SDA | 21 |
| OLED SCL | 22 |

Los relés están configurados como activos en nivel bajo. El sensor de nivel debe llevar el tanque a estado seguro cuando la lectura sea distinta de LOW.

## Protecciones implementadas

- El calefactor y el agitador arrancan apagados.
- El calefactor se apaga si el nivel es bajo.
- El calefactor se apaga si el DS18B20 está desconectado o entrega una lectura inválida.
- El PID utiliza el tiempo real entre mediciones y límite anti-windup.
- Se utiliza histéresis para evitar conmutaciones constantes cerca del setpoint.
- El estado interno del PID se reinicia al abandonar el modo PID.

## Seguridad eléctrica

Este proyecto controla cargas de 110 V. El software no sustituye una protección física: usa fusible, termostato independiente, protección contra sobretemperatura, componentes correctamente dimensionados, aislamiento y puesta a tierra cuando corresponda. Prueba primero con cargas de baja tensión y verifica el circuito con un técnico calificado.
