/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention TADEO MENDELEVICH
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include "ESP01.h"
#include "UNER.h"
#include "usbd_cdc_if.h"
#include "fonts.h"
#include "ssd1306.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CH_PD_Pin        GPIO_PIN_11
#define CH_PD_GPIO_Port  GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t BufUSBTx[256], nBytesTx;

extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t is250us, tmo100ms, is10ms;

uint8_t dataTx, dataRx, ESPSend;

static uint8_t unerRxBuffer[RXBUFSIZE];
static uint8_t unerTxBuffer[TXBUFSIZE];
static _sRx    unerRx;
static _sTx    unerTx;

char usbBuffer[128];

uint16_t adcValues[8];
uint8_t flag_adc = 0;
uint8_t adcCounter = 0;

static uint8_t esp01RxBuf[ESP01RXBUFAT];
static uint16_t esp01IwRx = 0;

static uint16_t esp01IrRx = 0;		/* Índice de lectura para el buffer UDP entrante */


uint8_t  espUSBBuf[ESP_USB_BUF_SIZE];
volatile uint16_t espUSBBufIw, espUSBBufIr;

uint8_t counter;

int x = 1;

uint8_t mpu6050Counter = 0;

uint8_t i2c1_tx_busy = 0;

uint8_t showMpuData = 0;

static const char HEX_DIGITS[] = "0123456789ABCDEF";	// Tabla de dígitos hex

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void USBRxData(uint8_t *buf, int len);
static int  uart_send_byte(uint8_t byte);
static void esp01_chpd(uint8_t val);
void onESP01StateChange(_eESP01STATUS state);
void ESP01_USB_DbgStr(const char *dbgStr);
void USB_BufferPush(uint8_t b);


void my_ssd1306_init(void *ctx);
int my_ssd1306_write_cmd(void *ctx, uint8_t cmd);
int my_ssd1306_write_data(void *ctx, const uint8_t *data, uint16_t len);
int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len);
uint8_t my_ssd1306_is_busy(void *ctx);
void my_ssd1306_errorCb(void *ctx, int err);
void my_ssd1306_delay_ms(void *ctx, uint32_t ms);

int  mpu_writeReg(void   *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length);
int  mpu_readReg(void   *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length);
int  mpu_readRegDMA(void   *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length);
void mpu_delayMs(void    *ctx, uint32_t ms);

void mpu_errorCb(void *ctx, int err);
void MPU6050_RegisterPlatform(MPU6050_Platform_t *plat);
void MPU6050_ProcessDMA(void);

void USB_DebugSend(const uint8_t *data, uint16_t len);
void USB_DebugStr(const char *s);	// Envía una cadena C terminada en '\0' por USB.
void USB_DebugHex(uint8_t b);	// Envía un byte representado en dos dígitos hexadecimales ASCII.
void USB_DebugUInt(unsigned int v);		// Envía un entero sin signo en formato decimal ASCII.
void USB_Debug(const char *fmt, ...);	// Mini-printf para debug: soporta %s, %c, %d/%u, %X y %%.
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
SSD1306_Ctx_t ssd_ctx = {
  .hi2c      = &hi2c1,
  .busy_flag = &i2c1_tx_busy
};

static SSD1306_Platform_t SSD1306_plat = {
  .ctx              = &ssd_ctx,
  .init             = my_ssd1306_init,
  .write_cmd        = my_ssd1306_write_cmd,
  .write_data       = my_ssd1306_write_data,
  .write_data_async = my_ssd1306_write_data_async,
  .is_busy          = my_ssd1306_is_busy,
  .delay_ms         = my_ssd1306_delay_ms,
  .onError          = my_ssd1306_errorCb
};

MPU6050_Platform_t mpuPlat = {
  .ctx         = &hi2c1,
  .writeReg    = mpu_writeReg,
  .readReg     = mpu_readReg,
  .readRegDMA  = mpu_readRegDMA,
  .delayMs     = mpu_delayMs,
  .onError    = mpu_errorCb
};

void my_ssd1306_init(void *ctx) {
    MX_I2C1_Init();
}

// Callback comando
int my_ssd1306_write_cmd(void *ctx, uint8_t cmd) {
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, (uint8_t[]){0x00, cmd}, 2, HAL_MAX_DELAY);

    if (st != HAL_OK) {
        SSD1306_plat.onError(ctx, (int)st);
        return -1;
    }
    return 0;
}


// Callback datos bloqueante
int my_ssd1306_write_data(void *ctx, const uint8_t *data, uint16_t len) {
    // Desempaquetar el contexto
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)ctx;
    // Preparo buffer con control byte + datos
    uint8_t buf[1 + SSD1306_WIDTH];
    buf[0] = 0x40;
    memcpy(&buf[1], data, len);
    // Transmisión bloqueante
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
        c->hi2c,
        SSD1306_I2C_ADDR,
        buf,
        len + 1,
        HAL_MAX_DELAY
    );
    if (st != HAL_OK) {
        // Notifico el fallo (p.ej. por USB y LED)
        SSD1306_plat.onError(ctx, (int)st);
        return -1;
    }
    return 0;
}

// Callback datos no bloqueante (DMA)
int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len) {
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)ctx;
    // 1) Si el bus está ocupado, lanza error
    if (*c->busy_flag) {
        SSD1306_plat.onError(ctx, -1);     // -1 = BUSY_ERROR
        return -1;
    }

    // 2) Preparo buffer (control byte + datos)
    static uint8_t dmaBuf[1 + SSD1306_WIDTH];
    dmaBuf[0] = 0x40;
    memcpy(&dmaBuf[1], data, len);

    // 3) Marco el bus como ocupado
    *c->busy_flag = 1;

    // 4) Lanzo la DMA
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit_DMA(c->hi2c, SSD1306_I2C_ADDR, dmaBuf,len + 1);
    if (st != HAL_OK) {
        // si falla al arrancar, limpio flag y notifico
        *c->busy_flag = 0;
        SSD1306_plat.onError(ctx, (int)st);
        return -1;
    }
    return 0;
}


// Callback busy-check
uint8_t my_ssd1306_is_busy(void *ctx) {
    return i2c1_tx_busy ? 1 : 0;
}

void my_ssd1306_errorCb(void *ctx, int err) {
    USB_Debug("ERROR SSD1306: 0x%02X\r\n", err);
    i2c1_tx_busy = 0;
	SSD1306_ResetUpdateState();
}

// Callback delay
void my_ssd1306_delay_ms(void *ctx, uint32_t ms) {
    HAL_Delay(ms);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
        i2c1_tx_busy = 0;
    }
}

int mpu_writeReg(void *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write((I2C_HandleTypeDef*)ctx, devAddr, regAddr,
    					I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        mpuPlat.onError(mpuPlat.ctx, st);
        return -1;
    }
    return 0;
}

int mpu_readReg(void *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read((I2C_HandleTypeDef*)ctx, devAddr, regAddr,
    					I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        mpuPlat.onError(mpuPlat.ctx, st);
        return -1;
    }
    return 0;
}

int mpu_readRegDMA(void *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA((I2C_HandleTypeDef*)ctx, devAddr, regAddr,
    					I2C_MEMADD_SIZE_8BIT, data, len);
    if (st != HAL_OK) {
        mpuPlat.onError(mpuPlat.ctx, st);
        return -1;
    }
    return 0;
}

void mpu_delayMs(void *ctx, uint32_t ms) {
    HAL_Delay(ms);
}

void mpu_errorCb(void *ctx, int err) {	// En caso de que exista un error, lo comunico por USB
    USB_Debug("ERROR MPU6050: 0x%02X\r\n", err);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) return;
    MPU6050_ProcessDMA();
}

void USB_BufferPush(uint8_t b){
    espUSBBuf[espUSBBufIw++] = b;
    espUSBBufIw &= (ESP_USB_BUF_SIZE - 1);
}

void USBRxData(uint8_t *buf, int len) {
	BufUSBTx[0] = 'U';
	BufUSBTx[1] = 'S';
	BufUSBTx[2] = 'B';
	BufUSBTx[3] = ' ';

	for(nBytesTx = 0; nBytesTx < len; nBytesTx++) {
		BufUSBTx[nBytesTx+4] = buf[nBytesTx];
	}

	nBytesTx += 4;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM1){		// 250us
		is250us = 1;
		adcCounter++;
	}

	if (htim->Instance == TIM2) {	// 10ms
		is10ms = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        ESP01_WriteRX(dataRx);      // Sube byte al buffer AT
        //ESP01_Task();    // dentro hace ESP01ATDecode()
        HAL_UART_Receive_IT(&huart1, &dataRx, 1);
    }
}

static void esp01_chpd(uint8_t val) {
    // CH_PD_GPIO_Port y CH_PD_Pin vienen de MX_GPIO_Init()
    HAL_GPIO_WritePin(CH_PD_GPIO_Port, CH_PD_Pin,
                      val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int uart_send_byte(uint8_t byte) {
    return (HAL_UART_Transmit(&huart1, &byte, 1, 100) == HAL_OK) ? 1 : 0;
}

void onESP01StateChange(_eESP01STATUS state) {
    switch(state) {
        case ESP01_WIFI_CONNECTED:
            ESP01_USB_DbgStr("\r\n>>> ESP01: WIFI_CONNECTED (got IP) <<<\r\n");
            ESP01_StartUDP("192.168.100.5", 30010, 30000);
            break;
        case ESP01_WIFI_NEW_IP: {
        	USB_DebugStr("\r\n>>> ESP01: New IP Address = ");
			USB_DebugStr(ESP01_GetLocalIP());
			USB_DebugStr(" <<<\r\n");
            break;
        }
        case ESP01_WIFI_DISCONNECTED:
            ESP01_USB_DbgStr("\r\nxxx ESP01: WIFI_DISCONNECTED xxx\r\n");
            break;
        case ESP01_UDPTCP_CONNECTED:
            ESP01_USB_DbgStr("\r\n+++ ESP01: UDP Connection Established +++\r\n");
            if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED) {
				char msg[] = "ALIVE\r\n";
				//ESP01_USB_DbgStr("\r\n>>> ESP01: Sending periodic ping <<<\r\n");
				ESP01_Send((uint8_t*)msg, 0, sizeof(msg)-1, sizeof(msg)-1);
			  }
            break;
        case ESP01_SEND_OK:
            ESP01_USB_DbgStr("\r\n>>> ESP01: Data Sent OK <<<\r\n");
            break;
        default:
            break;
    }
}


void ESP01_USB_DbgStr(const char *dbgStr) {
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        CDC_Transmit_FS((uint8_t*)dbgStr, strlen(dbgStr));
    }
}

// Envío “atómico” por USB (fragmenta en trozos de ≤64 bytes)
void USB_DebugSend(const uint8_t *data, uint16_t len) {
    extern USBD_HandleTypeDef hUsbDeviceFS;
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;

    while (len) {
        uint16_t chunk = (len > 64 ? 64 : len);
        if (CDC_Transmit_FS((uint8_t*)data, chunk) != USBD_BUSY) {
            data  += chunk;
            len   -= chunk;
        }
        // si está ocupado, lo reintentamos en el siguiente bucle
    }
}

// Envía una cadena literal
void USB_DebugStr(const char *s) {
    USB_DebugSend((uint8_t*)s, (uint16_t)strlen(s));
}

// Envía un byte como dos dígitos hex ASCII
void USB_DebugHex(uint8_t b) {
    char h[2] = { HEX_DIGITS[b >> 4], HEX_DIGITS[b & 0xF] };
    USB_DebugSend((uint8_t*)h, 2);
}

// Envía un entero sin signo en decimal
void USB_DebugUInt(unsigned int v) {
    char buf[10];
    int  pos = 0;
    if (v == 0) {
        USB_DebugSend((uint8_t*)"0", 1);
        return;
    }
    while (v) {
        buf[pos++] = '0' + (v % 10);
        v /= 10;
    }
    // buf[] está al revés
    for (int i = pos - 1; i >= 0; i--) {
        USB_DebugSend((uint8_t*)&buf[i], 1);
    }
}

// La nueva USB_Debug sin vsnprintf:
void USB_Debug(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            switch (*fmt) {
                case 's': { // cadena
                    char *s = va_arg(ap, char*);
                    USB_DebugStr(s);
                    break;
                }
                case 'c': { // caracter
                    char c = (char)va_arg(ap, int);
                    USB_DebugSend((uint8_t*)&c, 1);
                    break;
                }
                case 'u': // entero sin signo
                case 'd': { // entero con signo (le damos igual)
                    unsigned int v = va_arg(ap, unsigned int);
                    USB_DebugUInt(v);
                    break;
                }
                case 'X': { // hex byte
                    uint8_t b = (uint8_t)va_arg(ap, unsigned int);
                    USB_DebugHex(b);
                    break;
                }
                case '%': { // literal ‘%’
                    USB_DebugSend((uint8_t*)"%", 1);
                    break;
                }
                default: // desconocido, lo imprimimos tal cual
                    USB_DebugSend((uint8_t*)fmt, 1);
            }
        } else {
            // carácter normal
            USB_DebugSend((uint8_t*)fmt, 1);
        }
        fmt++;
    }

    va_end(ap);
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//    if (hadc->Instance == ADC1) {	// Interrupcion de conversion de los ADC listos
//        flag_adc = 1;
//        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // Blink LED
//    }
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
	  Error_Handler();
  }

  CDC_Attach_Rx(USBRxData);
  nBytesTx = 0;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, &dataRx, 1);

  static _sESP01Handle esp01Handle = {
      .aDoCHPD         = esp01_chpd,                  // Controla CH_PD
      .aWriteUSARTByte = uart_send_byte,          // Envía un byte por UART
      .bufRX           = esp01RxBuf,              // Buffer de recepción
      .iwRX            = &esp01IwRx,              // Índice de escritura
      .sizeBufferRX    = sizeof(esp01RxBuf)       // Tamaño del buffer
  };
  ESP01_Init(&esp01Handle);                        // Copia el handle interno :contentReference[oaicite:1]{index=1}
  esp01_chpd(1);  // Pone CH_PD a nivel alto para sacar al módulo de reset
  HAL_Delay(100);

  //ESP01_AttachDebugStr((void (*)(const char *))printf);
  ESP01_AttachDebugStr(ESP01_USB_DbgStr);
  ESP01_AttachChangeState(onESP01StateChange);

  ESP01_SetWIFI("MEGACABLE FIBRA-2.4G-ckd0", "djg19dlk");
  //ESP01_SetWIFI("FCAL", "fcalconcordia.06-2019");


  //if (ESP01_StateWIFI() == ESP01_WIFI_CONNECTED) {
  //    ESP01_StartTCPServer(5000);  // crea servidor TCP en puerto 5000
  //}

  //ESP01_StartUDP("172.23.205.98", 30000, 30010);
  //ESP01_StartUDP("192.168.100.5", 30000, 30010);		// Inicio conexion UDP

  unerRx.buff = unerRxBuffer;
  unerRx.mask = RXBUFSIZE - 1;
  unerTx.buff = unerTxBuffer;
  unerTx.mask = TXBUFSIZE - 1;
  UNER_Init(&unerRx, &unerTx);

  HAL_Delay(500);
  SSD1306_RegisterPlatform(&SSD1306_plat);
  SSD1306_Init();

  SSD1306_GotoXY(35, 5);
  SSD1306_Puts("TADEO",   &Font_7x10, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY(5, 35);
  SSD1306_Puts("MENDELEVICH",&Font_7x10, SSD1306_COLOR_WHITE);

  SSD1306_UpdateScreen_Blocking();
  HAL_Delay(3000);
  SSD1306_Fill(SSD1306_COLOR_BLACK);
  SSD1306_UpdateScreen_Blocking();

  MPU6050_RegisterPlatform(&mpuPlat);
  int status = MPU6050_Init();
  if (status != MPU6050_OK) {
      USB_Debug("ERROR MPU6050: NO SE HA PODIDO INICIALIZAR EL MODULO MPU6050\r\n");
  }
  MPU6050_StartRead_Accel_DMA();

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  tmo100ms = 10;
  is250us = 0;
  is10ms = 0;
  ESPSend = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (is250us) {
	      is250us = 0;
	  }

	  if(is10ms) {
		  is10ms = 0;

		  /*ESPCounter++;
		  if (ESPCounter >= 100) {
			  ESPCounter = 0;
			  // Solo si ya estamos conectados UDP
			  if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED) {
				  const char msg[] = "Ping desde ESP01\r\n";
				  //ESP01_USB_DbgStr("\r\n>>> ESP01: Sending periodic ping <<<\r\n");
				  ESP01_Send((uint8_t*)msg, 0, sizeof(msg)-1, sizeof(msg)-1);
			  }
		  }*/

		  ESP01_Timeout10ms();  // Requerido por la librería ESP01
		  ESP01_Task(); // Procesa tramas ESP01 recibidas

		  if (espUSBBufIr != espUSBBufIw) {
		      uint8_t tmp[64];
		      uint16_t cnt = 0;
		      while (espUSBBufIr != espUSBBufIw && cnt < sizeof(tmp)) {
		          tmp[cnt++] = espUSBBuf[espUSBBufIr++];
		          espUSBBufIr &= (ESP_USB_BUF_SIZE - 1);
		      }
		      CDC_Transmit_FS(tmp, cnt);
		  }

		  while (esp01IrRx != esp01IwRx) {
			  uint8_t b = esp01RxBuf[esp01IrRx++];
			  esp01IrRx &= (ESP01RXBUFAT - 1);   // wrap-around
			  UNER_PushByte(b);                  // turn5file2: UNER_PushByte guarda en unerRx->buff[]
		  }

		  UNER_Task(); // Procesa tramas UNER recibidas

		  mpu6050Counter++;
		  if (mpu6050Counter >= 2) {
			mpu6050Counter = 0;
			if (!i2c1_tx_busy) {
			  MPU6050_StartRead_Accel_DMA();   // Lanzo la lectura de aceleracion y luego se lanza sola la de giroscopio dentro de su funcion
			}
		  }

		  if (MPU6050_IsAccelReady() && !i2c1_tx_busy) {
			  MPU6050_ClearAccelReady();
		  }

		  if (MPU6050_IsGyroReady() && !i2c1_tx_busy) {
			  MPU6050_ClearGyroReady();
			  showMpuData    = 1;
		  }

		  tmo100ms--;
		  if (tmo100ms == 0) {
			  tmo100ms = 10;
			  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // Blink LED
		  }

	  }

	  /*if(adcCounter >= 1) {		// Envio y acero valores ADC

		  int len = sprintf(usbBuffer, "ADC: %d\r\n", adcValues[0]);
//			  int len = sprintf(usbBuffer, "ADC: %d,%d,%d,%d,%d,%d,%d,%d\r\n",
//								adcValues[0], adcValues[1], adcValues[2], adcValues[3],
//								adcValues[4], adcValues[5], adcValues[6], adcValues[7]);

		  CDC_Transmit_FS((uint8_t*)usbBuffer, len);

		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);	// Reinicio lectura ADC's

		  adcCounter = 0;

	  }*/

	  if (showMpuData && SSD1306_IsUpdateDone()) {
		  showMpuData = 0;

		  MPU6050_GetAccel(&ax, &ay, &az);
		  MPU6050_GetGyro(&gx, &gy, &gz);

		  SSD1306_Fill(SSD1306_COLOR_BLACK);

		  SSD1306_GotoXY(0, 0);
		  SSD1306_Puts("Valores MPU6050:", &Font_7x10, SSD1306_COLOR_WHITE);

		  SSD1306_GotoXY(0, 20);
		  SSD1306_Puts("AX:", &Font_7x10, SSD1306_COLOR_WHITE);
		  char num[7];
		  itoa(ax, num, 10);
		  SSD1306_Puts(num, &Font_7x10, SSD1306_COLOR_WHITE);
		  SSD1306_Puts(" AY:", &Font_7x10, SSD1306_COLOR_WHITE);
		  itoa(ay, num, 10);
		  SSD1306_Puts(num, &Font_7x10, SSD1306_COLOR_WHITE);

		  // Fila 2: AZ
		  SSD1306_GotoXY(0, 30);
		  SSD1306_Puts("AZ:", &Font_7x10, SSD1306_COLOR_WHITE);
		  itoa(az, num, 10);
		  SSD1306_Puts(num, &Font_7x10, SSD1306_COLOR_WHITE);

		  // Fila 3: GX, GY
		  SSD1306_GotoXY(0, 40);
		  SSD1306_Puts("GX:", &Font_7x10, SSD1306_COLOR_WHITE);
		  itoa(gx, num, 10);
		  SSD1306_Puts(num, &Font_7x10, SSD1306_COLOR_WHITE);
		  SSD1306_Puts(" GY:", &Font_7x10, SSD1306_COLOR_WHITE);
		  itoa(gy, num, 10);
		  SSD1306_Puts(num, &Font_7x10, SSD1306_COLOR_WHITE);

		  // Fila 4: GZ
		  SSD1306_GotoXY(0, 50);
		  SSD1306_Puts("GZ:", &Font_7x10, SSD1306_COLOR_WHITE);
		  itoa(gz, num, 10);
		  SSD1306_Puts(num, &Font_7x10, SSD1306_COLOR_WHITE);

		  SSD1306_RequestUpdate();
	  }

	  SSD1306_UpdateScreen();


	  if (unerTx.indexR != unerTx.indexW && ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED) {
	      uint8_t dataToSend[TXBUFSIZE];
	      uint16_t i = 0;

	      while (unerTx.indexR != unerTx.indexW && i < sizeof(dataToSend)) {
	          dataToSend[i++] = unerTx.buff[unerTx.indexR++];
	          unerTx.indexR &= unerTx.mask;
	      }

	      if (i > 0) {
	          ESP01_Send(dataToSend, 0, i, i);
	      }
	  }


	  if (dataTx) {
	      HAL_UART_Transmit(&huart1, &dataTx, 1, 100);
	      dataTx = 0;
	  }

	  if(CDC_Transmit_FS(BufUSBTx, nBytesTx) == USBD_OK)
		  nBytesTx = 0;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CH_PD_GPIO_Port, CH_PD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CH_PD_Pin */
  GPIO_InitStruct.Pin = CH_PD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH_PD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
