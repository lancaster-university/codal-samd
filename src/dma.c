#include "dma.h"
#include "CodalDmesg.h"

#define NUM_STREAMS 8
#define NUM_DMA 2

//#define LOG DMESG
#define LOG(...) ((void)0)

typedef struct
{
    DMA_Stream_TypeDef *instance;
    uint8_t irqn;
} DmaStream;

const DmaStream streams[] = //
    {{DMA1_Stream0, DMA1_Stream0_IRQn},
     {DMA1_Stream1, DMA1_Stream1_IRQn},
     {DMA1_Stream2, DMA1_Stream2_IRQn},
     {DMA1_Stream3, DMA1_Stream3_IRQn},
     {DMA1_Stream4, DMA1_Stream4_IRQn},
     {DMA1_Stream5, DMA1_Stream5_IRQn},
     {DMA1_Stream6, DMA1_Stream6_IRQn},
     {DMA1_Stream7, DMA1_Stream7_IRQn},
     {DMA2_Stream0, DMA2_Stream0_IRQn},
     {DMA2_Stream1, DMA2_Stream1_IRQn},
     {DMA2_Stream2, DMA2_Stream2_IRQn},
     {DMA2_Stream3, DMA2_Stream3_IRQn},
     {DMA2_Stream4, DMA2_Stream4_IRQn},
     {DMA2_Stream5, DMA2_Stream5_IRQn},
     {DMA2_Stream6, DMA2_Stream6_IRQn},
     {DMA2_Stream7, DMA2_Stream7_IRQn},
     {0, 0}};

const uint32_t channels[] = {
    DMA_CHANNEL_0, DMA_CHANNEL_1, DMA_CHANNEL_2, DMA_CHANNEL_3,
    DMA_CHANNEL_4, DMA_CHANNEL_5, DMA_CHANNEL_6, DMA_CHANNEL_7,
};

#if !defined(STM32F4)
#error "check the DMA mapping table below"
#endif

MBED_WEAK const DmaMap TheDmaMap[] = //
    {
        // SPI1
        {SPI1_BASE, DMA_RX, 2, 0, 3},
        {SPI1_BASE, DMA_RX, 2, 2, 3},
        {SPI1_BASE, DMA_TX, 2, 3, 3},
        {SPI1_BASE, DMA_TX, 2, 5, 3},

        // SPI2
        {SPI2_BASE, DMA_RX, 1, 3, 0},
        {SPI2_BASE, DMA_TX, 1, 4, 0},

        // SPI3
        {SPI3_BASE, DMA_RX, 1, 0, 0},
        {SPI3_BASE, DMA_RX, 1, 2, 0},
        {SPI3_BASE, DMA_TX, 1, 5, 0},
        {SPI3_BASE, DMA_TX, 1, 7, 0},

        //
        {USART1_BASE, DMA_RX, 2, 2, 4},
        {USART1_BASE, DMA_RX, 2, 5, 4},
        {USART1_BASE, DMA_TX, 2, 7, 4},

        //
        {USART2_BASE, DMA_RX, 1, 5, 4},
        {USART2_BASE, DMA_TX, 1, 6, 4},

        //
        {USART6_BASE, DMA_RX, 2, 1, 5},
        {USART6_BASE, DMA_RX, 2, 2, 5},
        {USART6_BASE, DMA_TX, 2, 6, 5},
        {USART6_BASE, DMA_TX, 2, 7, 5},

        // 
        {TIM1_BASE, DMA_TIM_CH1, 2, 1, 6},
        {TIM1_BASE, DMA_TIM_CH2, 2, 2, 6},
        {TIM1_BASE, DMA_TIM_CH1, 2, 3, 6},
        {TIM1_BASE, DMA_TIM_CH4, 2, 4, 6},
        {TIM1_BASE, DMA_TIM_CH3, 2, 6, 6},

        //
        {TIM2_BASE, DMA_TIM_CH3, 1, 1, 3},
        {TIM2_BASE, DMA_TIM_CH1, 1, 5, 3},
        {TIM2_BASE, DMA_TIM_CH2, 1, 6, 3},
        //{TIM2_BASE, DMA_TIM_CH4, 1, 6, 3}, // duplicate TIM channels on a single DMA stream
        {TIM2_BASE, DMA_TIM_CH4, 1, 7, 3},

        // 
        {TIM3_BASE, DMA_TIM_CH4, 1, 2, 5},
        {TIM3_BASE, DMA_TIM_CH1, 1, 4, 5},
        {TIM3_BASE, DMA_TIM_CH2, 1, 5, 5},
        {TIM3_BASE, DMA_TIM_CH3, 1, 7, 5},

        //
        {TIM4_BASE, DMA_TIM_CH1, 1, 0, 2},
        {TIM4_BASE, DMA_TIM_CH2, 1, 3, 2},
        {TIM4_BASE, DMA_TIM_CH3, 1, 7, 2},

        {USART6_BASE, DMA_RX, 2, 2, 5},
        {USART6_BASE, DMA_TX, 2, 6, 5},
        {USART6_BASE, DMA_TX, 2, 7, 5},

        // The end
        {0, 0, 0, 0, 0}};

static DMA_HandleTypeDef *handles[NUM_STREAMS * NUM_DMA];

static void irq_callback(int id)
{
    LOG("DMA irq %d", id);
    if (handles[id])
        HAL_DMA_IRQHandler(handles[id]);
}

#define DEFIRQ(nm, id)                                                                             \
    void nm() { irq_callback(id); }

DEFIRQ(DMA1_Stream0_IRQHandler, 0)
DEFIRQ(DMA1_Stream1_IRQHandler, 1)
DEFIRQ(DMA1_Stream2_IRQHandler, 2)
DEFIRQ(DMA1_Stream3_IRQHandler, 3)
DEFIRQ(DMA1_Stream4_IRQHandler, 4)
DEFIRQ(DMA1_Stream5_IRQHandler, 5)
DEFIRQ(DMA1_Stream6_IRQHandler, 6)
DEFIRQ(DMA1_Stream7_IRQHandler, 7)
DEFIRQ(DMA2_Stream0_IRQHandler, NUM_STREAMS + 0)
DEFIRQ(DMA2_Stream1_IRQHandler, NUM_STREAMS + 1)
DEFIRQ(DMA2_Stream2_IRQHandler, NUM_STREAMS + 2)
DEFIRQ(DMA2_Stream3_IRQHandler, NUM_STREAMS + 3)
DEFIRQ(DMA2_Stream4_IRQHandler, NUM_STREAMS + 4)
DEFIRQ(DMA2_Stream5_IRQHandler, NUM_STREAMS + 5)
DEFIRQ(DMA2_Stream6_IRQHandler, NUM_STREAMS + 6)
DEFIRQ(DMA2_Stream7_IRQHandler, NUM_STREAMS + 7)

int dma_init(uint32_t peripheral, uint8_t rxdx, DMA_HandleTypeDef *obj, int flags)
{
    memset(obj, 0, sizeof(*obj));

    int id;
    const DmaMap *map;

    for (map = TheDmaMap; map->peripheral; map++)
    {
        if (map->peripheral == peripheral && map->rxdx == rxdx)
        {
            CODAL_ASSERT(map->dma <= NUM_DMA);
            CODAL_ASSERT(map->stream < NUM_STREAMS);
            id = (map->dma - 1) * NUM_STREAMS + map->stream;
            if (handles[id] == NULL)
            {
                handles[id] = obj;
                break;
            }
        }
    }

    CODAL_ASSERT(map->peripheral);

    obj->Instance = streams[id].instance;

    obj->Init.Channel = channels[map->channel];
    obj->Init.Direction = rxdx == DMA_RX ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH;
    obj->Init.PeriphInc = DMA_PINC_DISABLE;
    obj->Init.MemInc = DMA_MINC_ENABLE;
    if (flags & DMA_FLAG_2BYTE)
    {
        obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        obj->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    }
    else if (flags & DMA_FLAG_4BYTE)
    {
        obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        obj->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    }
    else
    {
        obj->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        obj->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    }
    obj->Init.Mode = DMA_NORMAL;
    obj->Init.Priority = rxdx == DMA_RX ? DMA_PRIORITY_HIGH : DMA_PRIORITY_LOW;

    if (map->dma == 1)
        __HAL_RCC_DMA1_CLK_ENABLE();
    else if (map->dma == 2)
        __HAL_RCC_DMA2_CLK_ENABLE();

    int res = HAL_DMA_Init(obj);
    CODAL_ASSERT(res == HAL_OK);

    LOG("DMA init %p irq=%d ch=%d str=%d", obj->Instance, streams[id].irqn, map->channel,
        map->stream);

    NVIC_EnableIRQ(streams[id].irqn);

    return 0;
}
