# Pseudocódigo del alimentador automático

```text
INICIO

  Inicializar ESP32
  Configurar comunicación Serial e I2C
  Configurar pines de relés y sensor de nivel
  Apagar calentador y ventilador
  Inicializar pantalla OLED
  Inicializar sensor DS18B20
  Inicializar DFPlayer Mini
  Iniciar conexión WiFi y configurar Ubidots

  Definir temperatura objetivo: 38 °C
  Definir banda PID: 0.3 °C
  Definir histéresis: 2 °C

  REPETIR continuamente:

    Actualizar hora del sistema
    Revisar conexión WiFi y Ubidots
    Leer temperatura cuando corresponda

    SI el nivel de agua es bajo O la temperatura no es válida:
      Apagar calentador
      Apagar ventilador
      Detener audio
      Reiniciar PID
      Mostrar alerta en pantalla
      Enviar estado a Ubidots cuando corresponda
      Volver al inicio del ciclo
    FIN SI

    SI la temperatura alcanza 38 °C:
      Marcar temperatura objetivo alcanzada
      Reiniciar temporizador del ventilador
    FIN SI

    SI la temperatura baja hasta 36 °C:
      Marcar temperatura objetivo no alcanzada
    FIN SI

    SI la temperatura objetivo no se ha alcanzado:
      Mantener ventilador encendido
    SINO:
      Controlar el ventilador según su ciclo programado
    FIN SI

    SI la temperatura objetivo se alcanzó Y corresponde emitir el aviso:
      Reproducir audio
    FIN SI

    SI el audio lleva 30 segundos reproduciéndose:
      Detener audio
    FIN SI

    SI la temperatura objetivo se alcanzó:
      Apagar calentador
      Reiniciar PID
    SINO SI el control PID está activo:
      Calcular salida PID
      Encender o apagar calentador según la salida PID
    SINO:
      Encender calentador
    FIN SI

    Actualizar pantalla OLED cada 250 ms
    Enviar datos a Ubidots cada 5 segundos

  FIN REPETIR

FIN
```
