# STM32F1 · Plataforma de Sensores Embebidos

<div align="center">

![Platform](https://img.shields.io/badge/Plataforma-STM32F103-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![Language](https://img.shields.io/badge/Lenguaje-C-555?style=for-the-badge&logo=c&logoColor=white)
![HAL](https://img.shields.io/badge/HAL-STM32CubeF1-03234B?style=for-the-badge)

**Firmware embebido completo sobre STM32F103: fusión de sensores en tiempo real, comunicación WiFi UDP, control de motores y visualización en OLED — sin floats, sin RTOS, sin compromisos.**

</div>

---

## ¿Qué hace este proyecto?

Es un sistema embebido que corre sobre un microcontrolador STM32F103 a 72 MHz y hace varias cosas al mismo tiempo dentro de un loop determinístico de 10 ms:

- **Lee el IMU MPU6050** (acelerómetro + giroscopio) de forma no bloqueante usando DMA sobre I²C, convierte los valores crudos a m/s² y °/s mediante aritmética entera de punto fijo, y aplica calibración automática al arranque.
- **Calcula la inclinación** (roll y pitch) con un algoritmo CORDIC propio — sin usar `<math.h>` ni números flotantes.
- **Filtra las señales** con un filtro EMA (media móvil exponencial) para el IMU y una ventana deslizante de 40 muestras para 8 canales de ADC.
- **Muestra todo en un display OLED SSD1306** de 128×64: valores del IMU en la mitad izquierda y barras de los ADC en la derecha, actualizando de forma asíncrona por DMA.
- **Comunica los datos por WiFi UDP** usando un módulo ESP-01 controlado mediante una máquina de estados AT completa (reset → configuración → conexión → socket UDP → reintento automático).
- **Controla dos motores DC** con PWM de cuatro canales (TIM2), con API de velocidad firmada (–100 a +100%).
- **Envía telemetría** por USB-CDC o UDP usando el protocolo binario UNER, con un mini-printf propio sin `vsnprintf`.

---

## Arquitectura

```
Loop principal — 10 ms
│
├── MPU6050  →  DMA I²C  →  ISR  →  ProcessDMA()  →  EMA filter  →  UNER
├── ESP01    →  UART IT  →  Máquina de estados AT  →  UDP socket
├── SSD1306  →  DMA I²C  →  Dashboard OLED (compartido con MPU6050)
├── ADC x8   →  DMA + TIM3  →  Moving average  →  barras OLED
└── Motores  →  TIM2 PWM  →  MotorControl(derecho, izquierdo)
```

El bus I²C es compartido entre el MPU6050 y el SSD1306. La arbitración se maneja con un flag de ocupado (`i2c1_tx_busy`) chequeado antes de cada transferencia DMA.

---

## Lo más interesante del código

**Adquisición no bloqueante encadenada** — Al terminar la lectura del acelerómetro por DMA, la propia ISR lanza automáticamente la lectura del giroscopio. El loop principal nunca espera al sensor.

**Punto fijo sin floats** — Todos los valores físicos se expresan con 2 decimales implícitos en `int16_t`. Por ejemplo, 9.81 m/s² se representa como `981`. Las conversiones usan shifts y divisiones enteras, sin perder precisión relevante.

**CORDIC entero** — El ángulo de inclinación se calcula con 16 iteraciones de rotación vectorial en enteros de 32 bits. Sin FPU, sin librerías matemáticas.

**Abstracción de plataforma** — Tanto el MPU6050 como el SSD1306 usan una estructura de punteros a funciones (vtable) inyectada en tiempo de ejecución. Esto hace los drivers completamente portables a cualquier MCU.

**ESP01 con reintento automático** — La máquina de estados maneja reset por hardware (CH_PD), reconexión WiFi, reconexión UDP y reenvío de tramas sin intervención del usuario.

---

## Hardware

| Componente | Detalle |
|------------|---------|
| MCU | STM32F103C8T6 @ 72 MHz |
| IMU | MPU6050 — ±2 g / ±250 °/s |
| Display | SSD1306 OLED 128×64 |
| WiFi | ESP-01 (ESP8266), UART @ 115200 |
| Motores | 2× DC con L298N, PWM 1 kHz |
| ADC | 8 canales, trigger por TIM3 cada 250 µs |
| Comunicación | USB-CDC virtual (debug + protocolo UNER) |

---

## Estructura del proyecto

```
Core/Src/
├── main.c       — Loop principal, inicialización, lógica de aplicación
├── MPU6050.c    — Driver IMU: DMA, calibración, punto fijo
├── ESP01.c      — Máquina de estados WiFi AT (UDP/TCP)
├── UNER.c       — Protocolo binario de telemetría
└── ssd1306.c    — Driver OLED con escritura asíncrona
```

---

## Autor

**Tadeo Mendelevich** — Estudiante de Ingeniería en Sistemas, UNER  

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Conectar-0A66C2?style=flat&logo=linkedin)]([https://linkedin.com/in/tu-perfil](https://www.linkedin.com/in/tadeo-mendelevich/))
[![GitHub](https://img.shields.io/badge/GitHub-Seguir-181717?style=flat&logo=github)]([https://github.com/tu-usuario](https://github.com/tadeomendelevich))
