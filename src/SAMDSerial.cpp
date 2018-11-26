#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "SAMDSerial.h"
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
#if 0
void SAMDSerial::dmaTransferComplete(DmaCode errCode)
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

void SAMDSerial::configureRxInterrupt(int enable)
{
}

SAMDSerial::SAMDSerial(Pin& tx, Pin& rx) : Serial(tx, rx)
{
    const mcu_pin_obj_t* tx_pin = samd_peripherals_get_pin(tx.name);
    const mcu_pin_obj_t* rx_pin = samd_peripherals_get_pin(rx.name);

    if (tx_pin->sercom[0].index != 0x3f && tx_pin->sercom[0].pad == 0)
    {
        this->tx_pinmux = MUX_C; // c
        this->instance_number = tx_pin->sercom[0].index;
    }
    else if (tx_pin->sercom[1].index != 0x3f && tx_pin->sercom[1].pad == 0)
    {
        this->tx_pinmux = MUX_D; // d
        this->instance_number = tx_pin->sercom[1].index;
    }
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    if (rx_pin->sercom[0].index == this->instance_number)
        this->rx_pinmux = MUX_C; // c
    else if (rx_pin->sercom[1].index == this->instance_number)
        this->rx_pinmux = MUX_D; // d
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    Sercom* instance = sercom_insts[this->instance_number];
    DMESG("SWS pad %d, idx %d, fn: %d", 0, this->instance_number, this->pinmux);

    this->id = DEVICE_ID_SERIAL;

    samd_peripherals_sercom_clock_init(instance, instance_number);
    _usart_async_init(&USART_INSTANCE, instance);

    // DmaFactory factory;
    // usart_tx_dma = factory.allocate();
    // usart_rx_dma = factory.allocate();
    // CODAL_ASSERT(usart_tx_dma != NULL);
    // CODAL_ASSERT(usart_rx_dma != NULL);

    // usart_tx_dma->onTransferComplete(this);
    // usart_rx_dma->onTransferComplete(this);

    // usart_tx_dma->configure(sercom_trigger_src(this->instance_number, true), BeatByte, NULL, (volatile void*)&CURRENT_USART->USART.DATA.reg);
    // usart_rx_dma->configure(sercom_trigger_src(this->instance_number, false), BeatByte, (volatile void*)&CURRENT_USART->USART.DATA.reg, NULL);

    setBaud(115200);
}

int SAMDSerial::setBaud(uint32_t baud)
{
    uint32_t val = _usart_async_calculate_baud_rate(baud, CONF_GCLK_SERCOM0_CORE_FREQUENCY, 16, USART_BAUDRATE_ASYNCH_ARITHMETIC, 0);
    _usart_async_set_baud_rate(&USART_INSTANCE, val);
    this->baud = baud;
    return DEVICE_OK;
}

uint32_t SAMDSerial::getBaud()
{
    return this->baud;
}

int SAMDSerial::putc(char c)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    CURRENT_USART->USART.DATA.reg = c;
    while(!(CURRENT_USART->USART.INTFLAG.bit.DRE));

    return DEVICE_OK;
}

int SAMDSerial::getc()
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    char c = 0;

    while(!(CURRENT_USART->USART.INTFLAG.bit.RXC));
    c = CURRENT_USART->USART.DATA.reg;

    return c;
}

int SAMDSerial::configureTx(int enable)
{
    if (enable && !(status & TX_CONFIGURED))
    {
        gpio_set_pin_function(p.name, this->pinmux);

        CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        CURRENT_USART->USART.CTRLA.bit.SAMPR = 0;
        CURRENT_USART->USART.CTRLA.bit.TXPO = 0;
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

int SAMDSerial::configureRx(int enable)
{
    if (enable && !(status & RX_CONFIGURED))
    {
        gpio_set_pin_function(p.name, this->pinmux);

        CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
        while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

        CURRENT_USART->USART.CTRLA.bit.SAMPR = 0;
        CURRENT_USART->USART.CTRLA.bit.RXPO = 0;
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

int SAMDSerial::setMode(SingleWireMode sw)
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

int SAMDSerial::send(uint8_t* data, int len)
{
    if (!(status & TX_CONFIGURED))
        setMode(SingleWireTx);

    for (int i = 0; i < len; i++)
        putc(data[i]);

    return DEVICE_OK;
}

int SAMDSerial::receive(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    for (int i = 0; i < len; i++)
        data[i] = getc();

    return DEVICE_OK;
}

int SAMDSerial::sendDMA(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireTx);

    usart_tx_dma->transfer((const void*)data, NULL, len);
    return DEVICE_OK;
}

int SAMDSerial::receiveDMA(uint8_t* data, int len)
{
    if (!(status & RX_CONFIGURED))
        setMode(SingleWireRx);

    usart_rx_dma->transfer(NULL, data, len);

    return DEVICE_OK;
}

int SAMDSerial::abortDMA()
{
    if (!(status & (RX_CONFIGURED | TX_CONFIGURED)))
        return DEVICE_INVALID_PARAMETER;

    usart_tx_dma->abort();
    usart_rx_dma->abort();

    // abort dma transfer
    return DEVICE_OK;
}

int SAMDSerial::sendBreak()
{
    if (!(status & TX_CONFIGURED))
        return DEVICE_INVALID_PARAMETER;

    // line break
    return DEVICE_OK;
}
#endif