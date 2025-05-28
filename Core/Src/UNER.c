/*
 * UNER.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Tadeo Mendelevich
 */

#include "UNER.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

const char *firmware = "UNER V1.0";

uint16_t globalIndex = 0;

static _sRx *unerRx;
static _sTx *unerTx;

void UNER_Init(_sRx *rx, _sTx *tx) {
    unerRx = rx;
    unerTx = tx;
    unerRx->indexR = 0;
    unerRx->indexW = 0;
    unerRx->header = HEADER_U;
    unerRx->mask = RXBUFSIZE - 1;
    unerTx->indexR = 0;
    unerTx->indexW = 0;
    unerTx->mask = TXBUFSIZE - 1;
    unerRx->indexData = 0;
    unerRx->nBytes    = 0;
    unerRx->chk       = 0;
    unerRx->timeOut   = 0;
    unerRx->isComannd = false;
    unerTx->chk       = 0;
}

void UNER_PushByte(uint8_t byte) {
    unerRx->buff[unerRx->indexW++] = byte;
    unerRx->indexW &= unerRx->mask;
}

void UNER_Task(void) {
    uint8_t auxIndex = unerRx->indexW;
    while (unerRx->indexR != auxIndex) {
        switch (unerRx->header) {
            case HEADER_U:
                if (unerRx->buff[unerRx->indexR] == 'U') {
                    unerRx->header = HEADER_N;
                    unerRx->timeOut = 5;
                }
                break;
            case HEADER_N:
                if (unerRx->buff[unerRx->indexR] == 'N') {
                    unerRx->header = HEADER_E;
                } else {
                    if (unerRx->buff[unerRx->indexR] != 'U') {
                        unerRx->header = HEADER_U;
                        unerRx->indexR--;
                    }
                }
                break;
            case HEADER_E:
                if (unerRx->buff[unerRx->indexR] == 'E') {
                    unerRx->header = HEADER_R;
                } else {
                    unerRx->header = HEADER_U;
                    unerRx->indexR--;
                }
                break;
            case HEADER_R:
                if (unerRx->buff[unerRx->indexR] == 'R') {
                    unerRx->header = NBYTES;
                } else {
                    unerRx->header = HEADER_U;
                    unerRx->indexR--;
                }
                break;
            case NBYTES:
                unerRx->nBytes = unerRx->buff[unerRx->indexR];
                unerRx->header = TOKEN;
                break;
            case TOKEN:
                if (unerRx->buff[unerRx->indexR] == ':') {
                    unerRx->header = PAYLOAD;
                    unerRx->indexData = unerRx->indexR + 1;
                    unerRx->indexData &= unerRx->mask;
                    unerRx->chk = 'U' ^ 'N' ^ 'E' ^ 'R' ^ unerRx->nBytes ^ ':';
                } else {
                    unerRx->header = HEADER_U;
                    unerRx->indexR--;
                }
                break;
            case PAYLOAD:
                unerRx->nBytes--;
                if (unerRx->nBytes > 0) {
                    unerRx->chk ^= unerRx->buff[unerRx->indexR];
                } else {
                    unerRx->header = HEADER_U;
                    if (unerRx->buff[unerRx->indexR] == unerRx->chk) {
                        unerRx->isComannd = true;
                        decodeCommand(unerRx, unerTx);
                    }
                }
                break;
            default:
                unerRx->header = HEADER_U;
                break;
        }
        unerRx->indexR++;
        unerRx->indexR &= unerRx->mask;
    }
}

void UNER_Send(uint8_t cmd, const uint8_t *payload, uint8_t length) {
    uint8_t chk = 0;
    const char header[] = { 'U', 'N', 'E', 'R' };

    for (int i = 0; i < 4; i++) {
        unerTx->buff[unerTx->indexW++] = header[i];
        unerTx->indexW &= unerTx->mask;
    }

    uint8_t len = length + 1; // incluye el cmd
    unerTx->buff[unerTx->indexW++] = len;
    unerTx->indexW &= unerTx->mask;
    unerTx->buff[unerTx->indexW++] = ':';
    unerTx->indexW &= unerTx->mask;
    unerTx->buff[unerTx->indexW++] = cmd;
    unerTx->indexW &= unerTx->mask;

    chk ^= ('U' ^ 'N' ^ 'E' ^ 'R' ^ len ^ ':' ^ cmd);
    for (uint8_t i = 0; i < length; i++) {
        unerTx->buff[unerTx->indexW++] = payload[i];
        unerTx->indexW &= unerTx->mask;
        chk ^= payload[i];
    }

    unerTx->buff[unerTx->indexW++] = chk;
    unerTx->indexW &= unerTx->mask;
}

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength)
{
    dataTx->chk = 0;
    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength+1;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= (frameLength+1);
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
    return  dataTx->chk;
}

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte)
{
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
    globalIndex=0;
    while(str[globalIndex]){
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk ;
}

void decodeCommand(_sRx *dataRx, _sTx *dataTx)
{
    switch(dataRx->buff[dataRx->indexData]){
        case ALIVE:
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case FIRMWARE:
            putHeaderOnTx(dataTx, FIRMWARE, 12);
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        default:
            putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
            putByteOnTx(dataTx,UNKNOWN );
            putByteOnTx(dataTx, dataTx->chk);
        break;
    }
}

/* END Private Functions*/
