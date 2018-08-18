#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"
#include "dma.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "CodalFiber.h"

using namespace codal;

#define TX_CONFIGURED       0x02
#define RX_CONFIGURED       0x04

uint8_t buffer[1024] = {0};
uint16_t buffer_head = 0;
uint16_t buffer_tail = 0;
uint8_t uart_status = 0;

static ZSingleWireSerial *instances[4];

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define UART_ON (uart.Instance->CR1 & USART_CR1_UE)

#define LOG DMESG

#define ZERO(f) memset(&f, 0, sizeof(f))

static int enable_clock(uint32_t instance)
{
    switch (instance)
    {
    case USART1_BASE:
        __HAL_RCC_USART1_CLK_ENABLE();
        NVIC_EnableIRQ(USART1_IRQn);
        return HAL_RCC_GetPCLK2Freq();
    case USART2_BASE:
        __HAL_RCC_USART2_CLK_ENABLE();
        NVIC_EnableIRQ(USART2_IRQn);
        return HAL_RCC_GetPCLK1Freq();
#ifdef USART6_BASE
    case USART6_BASE:
        __HAL_RCC_USART6_CLK_ENABLE();
        NVIC_EnableIRQ(USART6_IRQn);
        return HAL_RCC_GetPCLK2Freq();
#endif
    default:
        CODAL_ASSERT(0);
        return 0;
    }
    return 0;
}

void ZSingleWireSerial::_complete(uint32_t instance, uint32_t mode)
{
    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] && (uint32_t)instances[i]->uart.Instance == instance)
        {
            if (mode == SWS_EVT_ERROR)
            {
                uint8_t err = HAL_UART_GetError(&instances[i]->uart);
                codal_dmesg("ERROR %d", HAL_UART_GetError(&instances[i]->uart));
                if (err == HAL_UART_ERROR_FE)
                {
                    // a uart error disable any previously configured DMA transfers, we will always get a framing error...
                    // quietly restart...
                    HAL_UART_Receive_DMA(&instances[i]->uart, instances[i]->buf, instances[i]->bufLen);
                    return;
                }
                else
                    HAL_UART_Abort(&instances[i]->uart);
            }

            if (mode == 0)
                HAL_UART_IRQHandler(&instances[i]->uart);
            else
            {
                Event evt(instances[i]->id, mode, CREATE_ONLY);

                if (instances[i]->cb)
                    instances[i]->cb->fire(evt);
            }

            break;
        }
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hspi)
{
    ZSingleWireSerial::_complete((uint32_t)hspi->Instance, SWS_EVT_DATA_SENT);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hspi)
{
    ZSingleWireSerial::_complete((uint32_t)hspi->Instance, SWS_EVT_DATA_RECEIVED);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *hspi)
{
    ZSingleWireSerial::_complete((uint32_t)hspi->Instance, SWS_EVT_ERROR);
}

#define DEFIRQ(nm, id)                                                                             \
    extern "C" void nm() { ZSingleWireSerial::_complete(id, 0); }

DEFIRQ(USART1_IRQHandler, USART1_BASE)
DEFIRQ(USART2_IRQHandler, USART2_BASE)
#ifdef USART6_BASE
DEFIRQ(USART6_IRQHandler, USART6_BASE)
#endif


void ZSingleWireSerial::configureRxInterrupt(int enable)
{
}

ZSingleWireSerial::ZSingleWireSerial(Pin& p) : DMASingleWireSerial(p)
{
    ZERO(uart);
    ZERO(hdma_tx);
    ZERO(hdma_rx);

    // only the TX pin is operable in half-duplex mode
    uart.Instance = (USART_TypeDef *)pinmap_peripheral(p.name, PinMap_UART_TX);

    enable_clock((uint32_t)uart.Instance);

    dma_init((uint32_t)uart.Instance, DMA_RX, &hdma_rx, 0);
    __HAL_LINKDMA(&uart, hdmarx, hdma_rx);

    dma_init((uint32_t)uart.Instance, DMA_TX, &hdma_tx, 0);
    __HAL_LINKDMA(&uart, hdmatx, hdma_tx);

    // set some reasonable defaults
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.Mode = UART_MODE_RX;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;

    for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    {
        if (instances[i] == NULL)
        {
            instances[i] = this;
            break;
        }
    }

    status = 0;
}

int ZSingleWireSerial::setBaud(uint32_t baud)
{
    uart.Init.BaudRate = baud;
    return DEVICE_OK;
}

uint32_t ZSingleWireSerial::getBaud()
{
    return uart.Init.BaudRate;
}

int ZSingleWireSerial::putc(char c)
{
    return send((uint8_t*)&c, 1);
}

int ZSingleWireSerial::getc()
{
    char c = 0;
    int res = receive((uint8_t*)&c, 1);

    if (res == DEVICE_OK)
        return c;

    return res;
}

int ZSingleWireSerial::configureTx(int enable)
{
    if (enable && !(status & TX_CONFIGURED))
    {
        uint8_t pin = (uint8_t)p.name;
        pin_mode(pin, PullNone);
        pin_function(pin, pinmap_function(pin, PinMap_UART_TX));
        uart.Init.Mode = UART_MODE_TX;
        HAL_HalfDuplex_Init(&uart);
        status |= TX_CONFIGURED;
    }
    else if (status & TX_CONFIGURED)
    {
        HAL_UART_DeInit(&uart);
        status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

int ZSingleWireSerial::configureRx(int enable)
{
    if (enable && !(status & RX_CONFIGURED))
    {
        uint8_t pin = (uint8_t)p.name;
        pin_function(pin, pinmap_function(pin, PinMap_UART_TX));
        pin_mode(pin, PullNone);
        uart.Init.Mode = UART_MODE_RX;
        HAL_HalfDuplex_Init(&uart);
        status |= RX_CONFIGURED;
    }
    else if (status & RX_CONFIGURED)
    {
        HAL_UART_DeInit(&uart);
        status &= ~RX_CONFIGURED;
    }

    return DEVICE_OK;
}

int ZSingleWireSerial::setMode(SingleWireMode sw)
{
    if (sw == SingleWireRx)
    {
        configureTx(0);
        configureRx(1);
    }
    else if (sw == SingleWireTx)
    {
        configureRx(0);
        configureTx(1);
    }
    else
    {
        configureTx(0);
        configureRx(0);
    }

    return DEVICE_OK;
}

int ZSingleWireSerial::send(uint8_t* data, int len)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    int res = HAL_UART_Transmit(&uart, data, len, 3);

    if (res == HAL_OK)
        return DEVICE_OK;

    return DEVICE_CANCELLED;
}

int ZSingleWireSerial::receive(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    int res = HAL_UART_Receive(&uart, data, len, 3);

    if (res == HAL_OK)
        return DEVICE_OK;

    return DEVICE_CANCELLED;
}

int ZSingleWireSerial::sendDMA(uint8_t* data, int len)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    this->buf = data;
    this->bufLen = len;

    int res = HAL_UART_Transmit_DMA(&uart, data, len);

    CODAL_ASSERT(res == HAL_OK);

    return DEVICE_OK;
}

int ZSingleWireSerial::receiveDMA(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    this->buf = data;
    this->bufLen = len;

    int res = HAL_UART_Receive_DMA(&uart, data, len);

    CODAL_ASSERT(res == HAL_OK);

    return DEVICE_OK;
}

int ZSingleWireSerial::abortDMA()
{
    if (!(status & (RX_CONFIGURED | TX_CONFIGURED)))
        return DEVICE_INVALID_PARAMETER;

    HAL_UART_Abort(&uart);
    return DEVICE_OK;
}

int ZSingleWireSerial::sendBreak()
{
    if (!(status & TX_CONFIGURED))
        return DEVICE_INVALID_PARAMETER;

    HAL_LIN_SendBreak(&uart);
    return DEVICE_OK;
}