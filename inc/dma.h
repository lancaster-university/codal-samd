#pragma once

#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_RX 1
#define DMA_TX 2
#define DMA_TIM_CH1 3
#define DMA_TIM_CH2 4
#define DMA_TIM_CH3 5
#define DMA_TIM_CH4 6

typedef struct
{
    uint32_t peripheral; // SPI1_BASE etc
    uint8_t rxdx;        // 1 rx, 2 tx
    uint8_t dma;         // 1 or 2
    uint8_t stream;
    uint8_t channel;
} DmaMap;

#define DMA_FLAG_1BYTE 0
#define DMA_FLAG_2BYTE 1
#define DMA_FLAG_4BYTE 2

int dma_init(uint32_t peripheral, uint8_t rxdx, DMA_HandleTypeDef *obj, int flags);

extern const DmaMap TheDmaMap[];

#ifdef __cplusplus
}
#endif
