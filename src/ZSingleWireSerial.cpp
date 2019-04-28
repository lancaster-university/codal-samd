#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"
#include "dma.h"
#include "CodalFiber.h"

#include "driver_init.h"
#include "peripheral_clk_config.h"
extern "C"
{
    #include "pins.h"
    #include "sercom.h"
}

using namespace codal;

#define TX_CONFIGURED       0x02
#define RX_CONFIGURED       0x04

#define LOG DMESG

#define CURRENT_USART ((Sercom*)(USART_INSTANCE.hw))

static ZSingleWireSerial* sws_instance = NULL;

static void error_callback(struct _usart_async_device *device)
{
    // flag any error to the dma handler.
    if (sws_instance)
        sws_instance->dmaTransferComplete(DMA_ERROR);
}

static void tx_callback(struct _usart_async_device *)
{
}


static void rx_callback(struct _usart_async_device *, uint8_t)
{
}

void ZSingleWireSerial::dmaTransferComplete(DmaCode errCode)
{
    uint16_t mode = 0;

    if (errCode == DMA_COMPLETE)
    {
        if (status & TX_CONFIGURED)
            mode = SWS_EVT_DATA_SENT;

        if (status & RX_CONFIGURED)
            mode = SWS_EVT_DATA_RECEIVED;
    }
    else
        mode = SWS_EVT_ERROR;

    Event evt(this->id, mode, CREATE_ONLY);

    // if we have a cb member function, we invoke
    // otherwise fire the event for any listeners.
    if (this->cb)
        this->cb->fire(evt);
    else
        evt.fire();
}

void ZSingleWireSerial::configureRxInterrupt(int enable)
{
}

ZSingleWireSerial::ZSingleWireSerial(Pin& p) : DMASingleWireSerial(p)
{
    const mcu_pin_obj_t* single_wire_pin = samd_peripherals_get_pin(p.name);

    if (single_wire_pin->sercom[0].index != 0x3f &&
        (
            single_wire_pin->sercom[0].pad == 0
#ifdef SAMD21
            || single_wire_pin->sercom[0].pad == 2
#endif
        )
    )
    {
        this->pad = (single_wire_pin->sercom[0].pad == 2) ? 0x01 : 0x0;
        this->pinmux = MUX_C; // c
        this->instance_number = single_wire_pin->sercom[0].index;
    }
    else if (single_wire_pin->sercom[1].index != 0x3f &&
        (
            single_wire_pin->sercom[1].pad == 0
#ifdef SAMD21
            || single_wire_pin->sercom[1].pad == 2
#endif
        )
    )
    {
        this->pad = (single_wire_pin->sercom[1].pad == 2) ? 0x01 : 0x0;
        this->pinmux = MUX_D; // d
        this->instance_number = single_wire_pin->sercom[1].index;
    }
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    Sercom* instance = sercom_insts[this->instance_number];
    DMESG("SWS pad %d, idx %d, fn: %d", 0, this->instance_number, this->pinmux);

    this->id = DEVICE_ID_SERIAL;
    sws_instance = this;

    memset(&USART_INSTANCE, 0, sizeof(_usart_async_device));

    samd_peripherals_sercom_clock_init(instance, instance_number);
    _usart_async_init(&USART_INSTANCE, instance);

    // enable error callback to abort dma when an error is detected (error bit is not linked to dma unfortunately).
    USART_INSTANCE.usart_cb.error_cb = error_callback;
    USART_INSTANCE.usart_cb.tx_done_cb = tx_callback;
    USART_INSTANCE.usart_cb.rx_done_cb = rx_callback;
    _usart_async_set_irq_state(&USART_INSTANCE, USART_ASYNC_ERROR, true);

    DmaFactory factory;
    usart_tx_dma = factory.allocate();
    usart_rx_dma = factory.allocate();
    CODAL_ASSERT(usart_tx_dma != NULL, DEVICE_HARDWARE_CONFIGURATION_ERROR);
    CODAL_ASSERT(usart_rx_dma != NULL, DEVICE_HARDWARE_CONFIGURATION_ERROR);

    usart_tx_dma->onTransferComplete(this);
    usart_rx_dma->onTransferComplete(this);

    usart_tx_dma->configure(sercom_trigger_src(this->instance_number, true), BeatByte, NULL, (volatile void*)&CURRENT_USART->USART.DATA.reg);
    usart_rx_dma->configure(sercom_trigger_src(this->instance_number, false), BeatByte, (volatile void*)&CURRENT_USART->USART.DATA.reg, NULL);

    setBaud(115200);
}

int ZSingleWireSerial::setBaud(uint32_t baud)
{
    CURRENT_USART->USART.BAUD.reg = 65536 - ((uint64_t)65536 * 16 * baud) / CONF_GCLK_SERCOM0_CORE_FREQUENCY;
    this->baud = baud;
    return DEVICE_OK;
}

uint32_t ZSingleWireSerial::getBaud()
{
    return this->baud;
}

int ZSingleWireSerial::putc(char c)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    CURRENT_USART->USART.DATA.reg = c;
    while(!(CURRENT_USART->USART.INTFLAG.bit.DRE));

    return DEVICE_OK;
}

int ZSingleWireSerial::getc()
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    char c = 0;

    while(!(CURRENT_USART->USART.INTFLAG.bit.RXC));
    c = CURRENT_USART->USART.DATA.reg;

    return c;
}

int ZSingleWireSerial::configureTx(int enable)
{
    if (enable && !(status & TX_CONFIGURED))
    {
        gpio_set_pin_function(p.name, this->pinmux);

        CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        CURRENT_USART->USART.CTRLA.bit.SAMPR = 0;
        CURRENT_USART->USART.CTRLA.bit.TXPO = this->pad;
        CURRENT_USART->USART.CTRLB.bit.CHSIZE = 0;

        CURRENT_USART->USART.CTRLA.bit.ENABLE = 1;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        CURRENT_USART->USART.CTRLB.bit.TXEN = 1;
        while(CURRENT_USART->USART.SYNCBUSY.bit.CTRLB);
        status |= TX_CONFIGURED;
    }
    else if (status & TX_CONFIGURED && !enable)
    {
        CURRENT_USART->USART.CTRLB.bit.TXEN = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.CTRLB);
        //
        CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        gpio_set_pin_function(p.name, GPIO_PIN_FUNCTION_OFF);
        status &= ~TX_CONFIGURED;
    }

    return DEVICE_OK;
}

int ZSingleWireSerial::configureRx(int enable)
{
    if (enable && !(status & RX_CONFIGURED))
    {
        gpio_set_pin_function(p.name, this->pinmux);

        CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        CURRENT_USART->USART.CTRLA.bit.SAMPR = 0;
        CURRENT_USART->USART.CTRLA.bit.RXPO = this->pad;
        CURRENT_USART->USART.CTRLB.bit.CHSIZE = 0; // 8 BIT

        CURRENT_USART->USART.CTRLA.bit.ENABLE = 1;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        CURRENT_USART->USART.CTRLB.bit.RXEN = 1;
        while(CURRENT_USART->USART.SYNCBUSY.bit.CTRLB);

        status |= RX_CONFIGURED;
    }
    else if (status & RX_CONFIGURED)
    {
        CURRENT_USART->USART.CTRLB.bit.RXEN = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.CTRLB);
        //
        CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

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
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireTx);

    usart_tx_dma->transfer((const void*)data, NULL, len);
    return DEVICE_OK;
}

int ZSingleWireSerial::receiveDMA(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    usart_rx_dma->transfer(NULL, data, len);

    return DEVICE_OK;
}

int ZSingleWireSerial::getBytesReceived()
{
    if (!(status & RX_CONFIGURED))
        return DEVICE_INVALID_STATE;

    return usart_rx_dma->getBytesTransferred();
}

int ZSingleWireSerial::getBytesTransmitted()
{
    if (!(status & TX_CONFIGURED))
        return DEVICE_INVALID_STATE;

    return usart_tx_dma->getBytesTransferred();
}

int ZSingleWireSerial::abortDMA()
{
    if (!(status & (RX_CONFIGURED | TX_CONFIGURED)))
        return DEVICE_INVALID_PARAMETER;

    usart_tx_dma->abort();
    usart_rx_dma->abort();

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