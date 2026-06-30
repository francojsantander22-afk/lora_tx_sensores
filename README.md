HAPS Telemetry Node - LoRa Transmitter (STM32WL55)
Este repositorio contiene el firmware del nodo transmisor de telemetría para el proyecto HAPS (High Altitude Platform Station). El sistema está diseñado para adquirir datos ambientales e inerciales en vuelo, empaquetarlos eficientemente y transmitirlos a una estación terrena utilizando modulación LoRa.

El código está optimizado para microcontroladores de la familia STM32WL55, implementando una arquitectura no bloqueante basada en eventos (mediante el Secuenciador de ST) para garantizar la integridad de los datos y evitar colapsos del procesador durante las ventanas de transmisión/recepción.

- Hardware Utilizado
El sistema integra diversos sensores para monitorear las condiciones de la carga útil y la atmósfera:

Microcontrolador: STM32WL55C (SoC con radio Sub-GHz integrada).

Posicionamiento (UART): GPS ATG336H (Lectura de tramas NMEA, $GPGGA, $GPVTG).

Inercial (I2C): BMI270 (Acelerómetro y Giroscopio de 6 ejes).

Atmosféricos (I2C): * SHT21 (Temperatura y Humedad relativa).

BMP280 (Presión barométrica y Temperatura).

- Configuración de Radio (LoRa)
El middleware SubGHz_Phy de ST está configurado por defecto con los siguientes parámetros, optimizados para enlaces de largo alcance:

Frecuencia base: 917.3 MHz (Ajustable a 915 MHz).

Spreading Factor (SF): 7 (Configurable hasta SF12).

Bandwidth (BW): 125 kHz.

Potencia de Salida: TX_OUTPUT_POWER (Máxima permitida por hardware).

Timeouts: Ventana de escucha de ACK configurada en 3000 ms.

- Arquitectura del Firmware
El firmware utiliza un modelo de Productor-Consumidor sincronizado para evitar condiciones de carrera (Race Conditions) y HardFaults en el bus SPI interno de la radio.

Adquisición de Datos (main.c):

El GPS se lee constantemente en segundo plano usando interrupciones circulares (HAL_UART_Receive_IT) para no perder caracteres.

El muestreo de sensores I2C está gobernado por temporizadores no bloqueantes (HAL_GetTick()) y un "watchdog" lógico que asegura el envío de datos incluso si se pierde el fix satelital.

Sincronización (Flag lora_busy):

Se implementó un semáforo lógico (lora_busy). Mientras la radio está ocupada transmitiendo o esperando un ACK, el bucle principal congela la lectura del I2C y la actualización del buffer de transmisión, protegiendo la integridad de la trama en curso.

Máquina de Estados de Radio (subghz_phy_app.c):

Se ejecuta exclusivamente a través de la API UTIL_SEQ (Secuenciador de tareas de ST).

Las interrupciones de hardware (OnTxDone, OnRxTimeout) son extremadamente cortas: solo cambian el estado lógico y ceden el control al Secuenciador, manteniendo el procesador libre y el bus SPI seguro.

- Protocolo de Empaquetado (TLV)
Para maximizar la eficiencia del ancho de banda y reducir el tiempo en el aire (Time-on-Air), los datos no se envían en texto plano (ASCII). Se empaquetan en un formato binario TLV (Type-Length-Value).

El payload generado es dinámico (promedio 30-53 bytes) y contiene tags específicos para cada tipo de dato:

0x01: Tiempo (Hora, minuto, segundo).

0x02: Coordenadas GPS (Latitud y Longitud en grados x100000).

0x03: Altitud.

0x04: Aceleración (X, Y, Z).

0x05: Temperatura.

0x06: Giroscopio (X, Y, Z).

- Flujo de Ejecución (Ciclo de Vida)
El sistema adquiere datos de los sensores e interroga al GPS.

Si la radio está libre (lora_busy == 0), se construye el paquete TLV y se imprime un log ASCII de la telemetría por la UART de debug.

Se cierra el candado lógico y se encola la tarea PingPong_Process en el Secuenciador.

La radio emite el paquete y pasa automáticamente a modo RX para escuchar el ACK de la estación terrena.

Si se recibe el ACK: Se libera el candado y se espera al siguiente ciclo de sensores.

Si ocurre un Timeout: El paquete se descarta, se alerta por consola ("Master Alerta: Sin respuesta"), se libera el candado y se toman datos frescos del GPS/IMU para un nuevo intento (evitando enviar telemetría desactualizada).

- Compilación y Dependencias
IDE: STM32CubeIDE.

Paquete MCU: STM32Cube MCU Package for STM32WL series.

Middleware: SubGHz_Phy (proporcionado por STMicroelectronics).

Nota: Este nodo forma parte del segmento de vuelo. Para ver el comportamiento del receptor, dirígete al repositorio de la Estación Terrena.
