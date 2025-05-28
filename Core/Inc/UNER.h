/**
  ******************************************************************************
  * @file    ESP01.h
  * @author  Tadeo Mendelevich
  * @brief   Header file containing functions prototypes of ESP01 library.
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2023 HGE.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Version: 01b05 - 04/08/2024
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UNER_H_
#define UNER_H_

#include <stdint.h>
#include <stdbool.h>

/* buffers circulares para protocolo UNER */
#define RXBUFSIZE  256
#define TXBUFSIZE  256

extern uint16_t globalIndex;

// Estructura para la recepción de datos
typedef struct {
    volatile uint8_t *buff;
    uint8_t indexR;
    uint8_t indexW;
    uint8_t indexData;
    uint8_t nBytes;
    uint8_t header;
    uint8_t chk;
    uint8_t mask;
    bool isComannd;
    uint8_t timeOut;
} _sRx;

// Estructura para la transmisión de datos
typedef struct {
    uint8_t *buff;
    uint8_t indexR;
    uint8_t indexW;
    uint8_t mask;
    uint8_t chk;
} _sTx;

// Enums para el estado de parsing
enum { HEADER_U, HEADER_N, HEADER_E, HEADER_R, NBYTES, TOKEN, PAYLOAD };

// Comandos UNER (usar los mismos ID que en tu implementación Visual Studio)
typedef enum {
    ALIVE = 0xA0,
    FIRMWARE,
    LEDSTATUS,
    BUTTONSTATUS,
    ANALOGSENSORS,
    SETBLACKCOLOR,
    SETWHITECOLOR,
    SETLINESPEED,
    MOTORTEST,
    SERVOANGLE,
    CONFIGSERVO,
    GETDISTANCE,
    GETSPEED,
    SENDALLSENSORS,
    RADAR,
    SW0,
    UNKNOWN = 0xFE,
    ACK = 0xF0,
    SERVOFINISHMOVE = 0xF1
} _eCmd;

void UNER_Init(_sRx *rx, _sTx *tx);

void UNER_PushByte(uint8_t byte);

void UNER_Task(void);

void UNER_Send(uint8_t cmd, const uint8_t *payload, uint8_t length);

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte);

uint8_t putStrOntx(_sTx *dataTx, const char *str);

void decodeCommand(_sRx *dataRx, _sTx *dataTx);

#endif /* ESP01_H_ */
