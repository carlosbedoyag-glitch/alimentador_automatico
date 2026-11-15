# Pseudocódigo del alimentador automático

Este pseudocódigo corresponde al firmware de `alimentador_automatico.ino`.

```text
INICIO

  Inicializar ESP32, Serial e I2C
  Configurar relés y sensor de nivel
  Apagar calentador y ventilador
  Inicializar OLED y DS18B20
  Inicializar DFPlayer
  Iniciar WiFi y configurar Ubidots

  SETPOINT = 38.0 °C
  BANDA_PID = 0.3 °C
  HIST_TEMPERATURA = 2.0 °C
  VENTANA_PWM = 5 s

  REPETIR SIEMPRE:

    Actualizar tiempo actual
    Gestionar conexión WiFi y MQTT
    Actualizar temperatura cuando corresponda

    SI el nivel no es correcto O la temperatura no es válida:
      Apagar calentador y ventilador
      Borrar estado de meta y modo PID
      Reiniciar PID
      Detener audio
      Actualizar pantalla y telemetría cuando corresponda
      CONTINUAR
    FIN SI

    SI metaAlcanzada es FALSO Y temperatura >= SETPOINT:
      metaAlcanzada = VERDADERO
      Reiniciar temporizador del ventilador
    SINO SI metaAlcanzada es VERDADERO Y
            temperatura <= SETPOINT - HIST_TEMPERATURA:
      metaAlcanzada = FALSO
    FIN SI

    SI metaAlcanzada es FALSO:
      Mantener ventilador encendido
    SINO:
      Ejecutar ciclo del ventilador: 20 s encendido dentro de un ciclo de 60 s
    FIN SI

    SI se alcanzó la meta Y han pasado al menos 60 s desde el último llamado:
      Reproducir pista 1 del DFPlayer
      Marcar audio activo e iniciar temporizador
    FIN SI
    SI audio activo durante 30 s:
      Detener audio
    FIN SI

    SI metaAlcanzada:
      Apagar calentador
      Reiniciar PID
    SINO SI modoPIDActivo:
      Comparar salida PID con el tiempo transcurrido de la ventana PWM
      Encender o apagar calentador según el resultado
    SINO:
      Mantener calentador encendido
    FIN SI

    Actualizar OLED cada 250 ms cuando corresponda
    Publicar telemetría en Ubidots cada 5 s cuando corresponda

  FIN REPETIR

FIN
```

> El ventilador se apaga durante la parte restante de su ciclo de 60 segundos; no se usa un nombre de carpeta derivado de esa descripción.
