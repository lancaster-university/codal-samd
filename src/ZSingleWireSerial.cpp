#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"
#include "dma.h"
#include "CodalFiber.h"

#include "driver_init.h"
#include "hal_usart_async.h"
#include "peripheral_clk_config.h"

using namespace codal;

#define TX_CONFIGURED       0x02
#define RX_CONFIGURED       0x04

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#define UART_ON (uart.Instance->CR1 & USART_CR1_UE)

#define LOG DMESG

#define ZERO(f) memset(&f, 0, sizeof(f))

struct _usart_async_device USART_INSTANCE;

void ZSingleWireSerial::dmaTransferComplete()
{

}

void ZSingleWireSerial::_complete(uint32_t instance, uint32_t mode)
{
    // for (unsigned i = 0; i < ARRAY_SIZE(instances); ++i)
    // {
    //     if (instances[i] && (uint32_t)instances[i]->uart.Instance == instance)
    //     {
    //         if (mode == SWS_EVT_ERROR)
    //         {
    //             uint8_t err = HAL_UART_GetError(&instances[i]->uart);
    //             codal_dmesg("ERROR %d", HAL_UART_GetError(&instances[i]->uart));
    //             if (err == HAL_UART_ERROR_FE)
    //             {
    //                 // a uart error disable any previously configured DMA transfers, we will always get a framing error...
    //                 // quietly restart...
    //                 HAL_UART_Receive_DMA(&instances[i]->uart, instances[i]->buf, instances[i]->bufLen);
    //                 return;
    //             }
    //             else
    //                 HAL_UART_Abort(&instances[i]->uart);
    //         }

    //         if (mode == 0)
    //             HAL_UART_IRQHandler(&instances[i]->uart);
    //         else
    //         {
    //             Event evt(instances[i]->id, mode, CREATE_ONLY);

    //             if (instances[i]->cb)
    //                 instances[i]->cb->fire(evt);
    //         }

    //         break;
    //     }
    // }
}

void ZSingleWireSerial::configureRxInterrupt(int enable)
{
}


ZSingleWireSerial::ZSingleWireSerial(Pin& p) : DMASingleWireSerial(p)
{
    // usart_dma.disable();
    // dmaChannel = usart_dma.allocateChannel();
    // usart_dma.onTransferComplete(dmaChannel, this);
    hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBAMASK_SERCOM0_bit(MCLK);

    _usart_async_init(&USART_INSTANCE, SERCOM0);

    // baud = 65536 * (1 - 16 * (115200/120000000))

    setBaud(115200);
}

int ZSingleWireSerial::setBaud(uint32_t baud)
{
    uint32_t val = _usart_async_calculate_baud_rate(baud, CONF_GCLK_SERCOM0_CORE_FREQUENCY, 16, USART_BAUDRATE_ASYNCH_ARITHMETIC, 0);
    _usart_async_set_baud_rate(&USART_INSTANCE, val);
    this->baud = val;
    return DEVICE_OK;
}

uint32_t ZSingleWireSerial::getBaud()
{
    return this->baud;
}
extern void set_gpio(int);
int ZSingleWireSerial::putc(char c)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    SERCOM0->USART.DATA.reg = c;
    while(!(SERCOM0->USART.INTFLAG.bit.DRE));
}

int ZSingleWireSerial::getc()
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    char c = 0;

    while(!(SERCOM0->USART.INTFLAG.bit.RXC));
    c = SERCOM0->USART.DATA.reg;

    return c;
}

int ZSingleWireSerial::configureTx(int enable)
{
    if (enable && !(status & TX_CONFIGURED))
    {
        gpio_set_pin_function(p.name, PINMUX_PA04D_SERCOM0_PAD0);

        SERCOM0->USART.CTRLA.bit.ENABLE = 0;
        while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

        SERCOM0->USART.CTRLA.bit.SAMPR = 0;
        SERCOM0->USART.CTRLA.bit.TXPO = 0;
        SERCOM0->USART.CTRLB.bit.CHSIZE = 0;

        SERCOM0->USART.CTRLA.bit.ENABLE = 1;
        while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

        SERCOM0->USART.CTRLB.bit.TXEN = 1;
        while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
        status |= TX_CONFIGURED;
    }
    else if (status & TX_CONFIGURED && !enable)
    {
        SERCOM0->USART.CTRLB.bit.TXEN = 0;
        while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
        //
        SERCOM0->USART.CTRLA.bit.ENABLE = 0;
        while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

        gpio_set_pin_function(p.name, GPIO_PIN_FUNCTION_OFF);
        status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

int ZSingleWireSerial::configureRx(int enable)
{
    if (enable && !(status & RX_CONFIGURED))
    {
        gpio_set_pin_function(p.name, PINMUX_PA04D_SERCOM0_PAD0);

        SERCOM0->USART.CTRLA.bit.ENABLE = 0;
        while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

        SERCOM0->USART.CTRLA.bit.SAMPR = 0;
        SERCOM0->USART.CTRLA.bit.RXPO = 0; // PAD 0
        SERCOM0->USART.CTRLB.bit.CHSIZE = 0; // 8 BIT

        SERCOM0->USART.CTRLA.bit.ENABLE = 1;
        while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

        SERCOM0->USART.CTRLB.bit.RXEN = 1;
        while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);

        status |= RX_CONFIGURED;
    }
    else if (status & RX_CONFIGURED)
    {
        SERCOM0->USART.CTRLB.bit.RXEN = 0;
        while(SERCOM0->USART.SYNCBUSY.bit.CTRLB);
        //
        SERCOM0->USART.CTRLA.bit.ENABLE = 0;
        while(SERCOM0->USART.SYNCBUSY.bit.ENABLE);

        gpio_set_pin_function(p.name, GPIO_PIN_FUNCTION_OFF);
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

    for (int i = 0; i < len; i++)
        putc(data[i]);

    return DEVICE_OK;
}

int ZSingleWireSerial::receive(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    for (int i = 0; i < len; i++)
        data[i] = getc();

    return DEVICE_OK;
}

int ZSingleWireSerial::sendDMA(uint8_t* data, int len)
{
    // usart_dma.configureChannel(dmaChannel, DMA_TRIGGER_ACTION_BEAT, DMA_BEAT_SIZE_BYTE, )
    return send(data,len);
}

int ZSingleWireSerial::receiveDMA(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    return receive(data,len);
}

int ZSingleWireSerial::abortDMA()
{
    if (!(status & (RX_CONFIGURED | TX_CONFIGURED)))
        return DEVICE_INVALID_PARAMETER;

    // abort dma transfer
    return DEVICE_OK;
}

int ZSingleWireSerial::sendBreak()
{
    if (!(status & TX_CONFIGURED))
        return DEVICE_INVALID_PARAMETER;

    // line break
    return DEVICE_OK;
}