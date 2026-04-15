# STM32F1 · Plataforma de Sensores Embebidos

<div align="center">

![Platform](https://img.shields.io/badge/Plataforma-STM32F103-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![Language](https://img.shields.io/badge/Lenguaje-C-A8B9CC?style=for-the-badge&logo=c&logoColor=black)
![HAL](https://img.shields.io/badge/HAL-STM32CubeF1-1572B6?style=for-the-badge)
![Status](https://img.shields.io/badge/Estado-Funcional-brightgreen?style=for-the-badge)

<br/>

> **Firmware embebido completo sobre STM32F103:**
> fusión de sensores en tiempo real, comunicación WiFi UDP,
> control de motores y visualización en OLED —
> **sin floats · sin RTOS · sin compromisos.**

<br/>

[![LinkedIn](https://img.shields.io/badge/Tadeo_Mendelevich-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/tadeo-mendelevich/)
[![GitHub](https://img.shields.io/badge/tadeomendelevich-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/tadeomendelevich)

</div>

---

## ¿Qué hace este proyecto?

Sistema embebido corriendo sobre un STM32F103 a 72 MHz. Todo ocurre dentro de un **loop determinístico de 10 ms**, sin sistema operativo y sin bloqueos:

| Módulo | Descripción |
|--------|-------------|
| 🔵 **MPU6050** | Lee acelerómetro + giroscopio por DMA no bloqueante, convierte a unidades físicas con punto fijo, y se auto-calibra con 500 muestras al arranque |
| 📐 **Inclinación** | Calcula roll y pitch con CORDIC propio en enteros de 32 bits — sin `<math.h>`, sin FPU |
| 📉 **Filtros** | EMA sobre el IMU + ventana deslizante de 40 muestras sobre 8 canales de ADC |
| 🖥️ **SSD1306 OLED** | Dashboard en tiempo real: valores del IMU a la izquierda, barras de ADC a la derecha, escritura asíncrona por DMA |
| 📡 **ESP-01 WiFi** | Máquina de estados AT completa: reset → modo → DHCP → join → socket UDP → reintento automático |
| ⚙️ **Motores DC** | Control PWM de 4 canales (TIM2), API firmada de −100 a +100% por motor |
| 🔌 **Telemetría** | Protocolo binario UNER sobre USB-CDC o UDP, con mini-printf propio sin `vsnprintf` |

---

## Arquitectura

```
Loop principal — cada 10 ms
│
├── MPU6050 ──► DMA I²C ──► ISR ──► ProcessDMA() ──► Filtro EMA ──► UNER
│
├── ESP-01 ───► UART IT ──► Máquina de estados AT ──────────────► UDP socket
│
├── SSD1306 ──► DMA I²C ──► Dashboard OLED     (bus compartido con MPU6050)
│
├── ADC ×8 ───► DMA + TIM3 ──► Moving average ─────────────────► barras OLED
│
└── Motores ──► TIM2 PWM ──► MotorControl(derecho, izquierdo)
```

> El bus I²C es compartido entre el MPU6050 y el SSD1306.
> La arbitración se resuelve con un flag `i2c1_tx_busy` chequeado antes de cada transferencia DMA.

---

## Lo más interesante del código

#### ⚡ Adquisición no bloqueante encadenada
Al terminar la lectura del acelerómetro por DMA, la propia ISR lanza automáticamente la lectura del giroscopio. El loop principal nunca espera al sensor — la CPU queda libre para todo lo demás.

#### 🔢 Punto fijo sin floats
Todos los valores físicos se expresan con 2 decimales implícitos en `int16_t`. Por ejemplo, 9.81 m/s² se representa como `981`. Las conversiones usan shifts y divisiones enteras, sin perder precisión relevante.

#### 📐 CORDIC entero
El ángulo de inclinación se calcula con 16 iteraciones de rotación vectorial en enteros de 32 bits. Sin FPU, sin librerías matemáticas externas, ejecutable en cualquier Cortex-M.

#### 🔌 Abstracción de plataforma (vtable)
El MPU6050 y el SSD1306 reciben sus funciones de hardware como punteros en una estructura inyectada al inicializar. Los drivers son completamente portables — cambiar el I²C o el MCU no toca el driver.

#### 🔄 ESP-01 con reintento automático
La máquina de estados maneja reset por hardware (CH_PD), reconexión WiFi, reconexión UDP y reenvío de tramas perdidas sin ninguna intervención del usuario.

---

## Hardware

| Componente | Detalle |
|------------|---------|
| MCU | STM32F103C8T6 @ 72 MHz (PLL ×9, HSE 8 MHz) |
| IMU | MPU6050 — ±2 g / ±250 °/s, I²C fast-mode 400 kHz |
| Display | SSD1306 OLED 128×64, bus I²C compartido |
| WiFi | ESP-01 (ESP8266), UART @ 115200 bps |
| Motores | 2× DC con L298N, PWM 1 kHz — 4 canales TIM2 |
| ADC | 8 canales, trigger automático por TIM3 cada 250 µs |
| Debug | USB-CDC virtual (mini-printf + protocolo UNER) |

---

## Estructura del proyecto

```
Core/Src/
├── main.c       — Loop principal, inicialización, lógica de aplicación
├── MPU6050.c    — Driver IMU: DMA, calibración, punto fijo
├── ESP01.c      — Máquina de estados WiFi AT (UDP/TCP)
├── UNER.c       — Protocolo binario de telemetría
└── ssd1306.c    — Driver OLED con escritura asíncrona por DMA
```

---

<div align="center">

**Tadeo Mendelevich** · Ingeniería en Sistemas · UNER — Concordia, Entre Ríos

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Conectar-0A66C2?style=flat&logo=linkedin)](https://www.linkedin.com/in/tadeo-mendelevich/)
[![GitHub](https://img.shields.io/badge/GitHub-tadeomendelevich-181717?style=flat&logo=github)](https://github.com/tadeomendelevich)

</div>
