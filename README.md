# Alimentador Automático

Firmware para ESP32 que controla un sistema térmico con calentador y ventilador/agitador, mide la temperatura mediante un DS18B20 y muestra/publica el estado local y remotamente. También incluye lectura de nivel, una pantalla OLED SH1106, un DFPlayer Mini y conexión WiFi/MQTT con Ubidots.

> La documentación describe el firmware que está en `alimentador_automatico.ino`, incluidos sus valores actuales. El control de cargas térmicas debe probarse y validarse con supervisión técnica antes de usarlo en una instalación real.

## Archivos

- `alimentador_automatico.ino`: firmware comentado del ESP32.
- `README.md`: conexiones, configuración y descripción de la lógica.

## Componentes

- ESP32.
- DS18B20 para medir temperatura.
- Módulo de relé para calentador.
- Módulo de relé para ventilador/agitador.
- Sensor de nivel de agua.
- Pantalla OLED SH1106 I2C de 128 × 64.
- DFPlayer Mini, tarjeta microSD y altavoz.
- Red WiFi y dispositivo Ubidots para telemetría.

## Conexiones

| Componente / señal | GPIO ESP32 | Nota |
|---|---:|---|
| DS18B20 DATA | 4 | Usar resistencia pull-up de 4.7 kΩ entre DATA y 3.3 V. |
| Relé del ventilador | 26 | El firmware asume relé activo en LOW. |
| Relé del calentador | 25 | El firmware asume relé activo en LOW. |
| Sensor de nivel | 32 | Configurado como `INPUT_PULLUP`; el nivel correcto corresponde a LOW. |
| DFPlayer RX del ESP32 | 18 | Conectar con TX del DFPlayer. |
| DFPlayer TX del ESP32 | 19 | Conectar con RX del DFPlayer. |
| OLED SDA | 21 | Bus I2C. |
| OLED SCL | 22 | Bus I2C. |

Verifica el pinout y los niveles eléctricos de cada módulo. Los módulos de señal deben compartir una referencia GND apropiada. No alimentes cargas de potencia directamente desde los GPIO del ESP32.

## Bibliotecas y entorno

Instala el soporte para placas ESP32 y estas bibliotecas mediante el gestor de bibliotecas de Arduino:

- Ubidots ESP32 MQTT (`UbidotsEsp32Mqtt`).
- U8g2.
- OneWire.
- DallasTemperature.
- DFRobotDFPlayerMini.

`Arduino.h`, `WiFi.h` y `Wire.h` se incluyen con el entorno de Arduino y el paquete de placa ESP32.

## Configuración y carga

1. Abre `alimentador_automatico.ino` en Arduino IDE y selecciona la placa ESP32 y el puerto correctos.
2. Reemplaza los valores de ejemplo por el token de Ubidots y las credenciales de tu red:

   ```cpp
   const char *UBIDOTS_TOKEN = "TU_TOKEN_UBIDOTS";
   const char *WIFI_SSID = "TU_RED_WIFI";
   const char *WIFI_PASS = "TU_CONTRASENA_WIFI";
   const char *DEVICE_LABEL = "control-termico-esp32";
   ```

3. Compila y carga el programa.
4. Comprueba en Ubidots el dispositivo `control-termico-esp32` y las variables publicadas.
5. Antes de conectar el calentador y el ventilador, prueba el sentido activo de los relés y las señales del sensor de nivel.

No subas tokens ni contraseñas reales al repositorio. Usa credenciales locales y evita compartirlas en capturas, documentación pública o commits.

## Parámetros configurados

| Constante | Valor | Uso en el firmware |
|---|---:|---|
| `SETPOINT` | 38.0 °C | Temperatura objetivo. La bandera de meta se activa al alcanzar este valor. |
| `BANDA_PID` | 0.3 °C | El modo PID se marca activo entre 37.7 °C y 38.3 °C, si la meta todavía no está alcanzada. Como la meta se activa al llegar a 38.0 °C, el calentador efectivamente usa PID antes de ese umbral. |
| `HIST_TEMPERATURA` | 2.0 °C | La meta alcanzada se desactiva cuando la temperatura baja hasta 36.0 °C. |
| `KP`, `KI`, `KD` | 40.0, 0.2, 8.0 | Ganancias proporcional, integral y derivativa del cálculo PID. |
| `VENTANA_PWM` | 5 s | Ventana temporal para comparar la salida PID y conmutar el relé del calentador. |
| `INTERVALO_TEMP` | 800 ms | Intervalo mínimo de actualización de la lectura de temperatura. |
| `VENTILADOR_ON` | 20 s | Fase encendida del ventilador después de alcanzar la meta. |
| `VENTILADOR_OFF` | 40 s | Fase apagada del ventilador después de alcanzar la meta. |
| `REPETICION_LLAMADO` | 60 s | Tiempo mínimo entre inicios de alerta sonora. |
| `DURACION_AUDIO` | 30 s | Tiempo tras el cual el firmware detiene el audio y limpia su estado. |
| `INTERVALO_OLED` | 250 ms | Periodo mínimo entre actualizaciones de pantalla. |
| `INTERVALO_UBIDOTS` | 5 s | Periodo mínimo entre publicaciones de telemetría. |
| `TIMEOUT_RECONEXION` | 10 s | Intervalo de reintento de conexión WiFi cuando se encuentra desconectado. |

## Cómo funciona

### Arranque

`setup()` inicializa el puerto serial, I2C, pines, pantalla, sensor DS18B20, DFPlayer, WiFi y Ubidots. Los relés se apagan al inicio. El DS18B20 trabaja sin esperar la conversión de forma bloqueante; el firmware solicita una conversión y obtiene la lectura en una actualización posterior.

### Lectura y validación de temperatura

Cada vez que vence `INTERVALO_TEMP`, se consulta el DS18B20. Se acepta una lectura cuando no es `DEVICE_DISCONNECTED_C` y está entre -10 °C y 85 °C. Si la lectura es válida, se actualiza `tempActual`; si no, se reinicia el PID y se registra el error por Serial. En ambos casos se solicita la siguiente conversión.

### Control del calentador

- Antes de la zona PID, el calentador permanece encendido mientras la meta no haya sido alcanzada.
- Cerca del setpoint, el firmware calcula una salida PID limitada entre 0 y `VENTANA_PWM`. Esa salida se compara con el tiempo transcurrido de la ventana para encender o apagar el relé.
- Al alcanzar 38.0 °C, se marca `metaAlcanzada`, se apaga el calentador y se reinicia el PID.
- La meta permanece activa hasta que la lectura baja a 36.0 °C; entonces el sistema vuelve al modo de calentamiento.

La ventana `BANDA_PID` se comprueba como rango de temperatura, pero la bandera `metaAlcanzada` toma prioridad a partir de 38.0 °C. Por ello, no se debe interpretar el tramo de 38.0 °C a 38.3 °C como un tramo efectivo de regulación PID del calentador.

### Ventilador / agitador

- Si la temperatura es válida, el nivel es correcto y todavía no se alcanzó la meta, el ventilador permanece encendido.
- Después de alcanzar la meta, trabaja en ciclos de 20 segundos encendido y 40 segundos apagado.
- Si el sensor no es válido o el nivel no está en LOW, la función del ventilador lo apaga.

### Seguridad

Si el pin de nivel no está en LOW o la temperatura no es válida, el bucle llama a `procesarSeguridad()`. Esta función apaga ambos relés, borra el estado de meta y el modo PID, reinicia el PID y detiene el audio. La pantalla y la telemetría siguen actualizándose según sus intervalos.

El código asume la lógica eléctrica descrita arriba: relés activos en LOW y nivel correcto en LOW. Confirma estas condiciones con el hardware real; una polaridad distinta puede cambiar el comportamiento de seguridad.

### Alerta sonora

Cuando se alcanza la meta, `gestionarAudio()` inicia la pista 1 del DFPlayer si han transcurrido al menos 60 segundos desde el aviso anterior. El estado de reproducción se limpia a los 30 segundos y se llama a `stop()`. Si el DFPlayer no se detecta, el firmware continúa sin enviar comandos al módulo; la bandera de audio igualmente representa el intervalo de aviso.

### WiFi, Ubidots y pantalla

- Mientras WiFi está desconectado, se intenta reconectar cada 10 segundos.
- Con WiFi conectado, se mantiene el ciclo MQTT de Ubidots mediante `ubidots.loop()` y se solicita reconexión si MQTT no está conectado.
- La OLED muestra estado, temperatura objetivo y real, nivel, relés y estado WiFi.
- La telemetría se publica en dos lotes, con una pausa de 100 ms entre publicaciones.

## Variables de Ubidots

| Variable | Significado |
|---|---|
| `temperatura` | Temperatura leída en °C. |
| `setpoint` | Temperatura objetivo en °C. |
| `rele-calor` | Estado del relé del calentador: 1 encendido, 0 apagado. |
| `rele-ventilador` | Estado del relé del ventilador: 1 encendido, 0 apagado. |
| `nivel-agua` | 1 si el pin de nivel está en LOW; 0 en caso contrario. |
| `temperatura-valida` | 1 si la lectura DS18B20 pasa la validación; 0 si no. |
| `audio` | 1 mientras el firmware mantiene activa la ventana de aviso; 0 en caso contrario. |
| `estado` | Código de estado calculado por el firmware. |

### Códigos de estado

| Código | Significado según la lógica de publicación |
|---:|---|
| 0 | El nivel no está correcto. |
| 1 | Temperatura válida; calentando fuera del modo PID. |
| 2 | Modo PID activo. |
| 3 | Meta de temperatura alcanzada. |
| 4 | Ventana de aviso de audio activa. |
| 5 | Sensor de temperatura inválido con nivel correcto. |

## Seguridad y puesta en marcha

Este firmware no reemplaza un termostato independiente, un limitador de sobretemperatura ni protecciones eléctricas certificadas. Antes de operar con cargas reales:

- utiliza fusibles, aislamiento y protecciones térmicas adecuados;
- selecciona relés o contactores dimensionados para la carga y el tipo de conmutación;
- verifica la polaridad y funcionamiento de sensores y relés;
- prueba inicialmente con las cargas de potencia desconectadas;
- valida el control con personal calificado y supervisión apropiada.

El código se comparte con fines técnicos y educativos; la integración final debe evaluarse para el equipo y las condiciones de uso concretas.
