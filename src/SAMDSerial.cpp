#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "SAMDSerial.h"
#include "Event.h"
#include "CodalFiber.h"
#include "ZPin.h"

#include "driver_init.h"
#include "peripheral_clk_config.h"
extern "C"
{
    #include "pins.h"
    #include "sercom.h"
}

using namespace codal;

#define LOG DMESG

#define CURRENT_USART ((Sercom*)(USART_INSTANCE.hw))

static SAMDSerial* instances[SERCOM_INST_NUM] = { 0 };

static void tx_callback(struct ::_usart_async_device* dev)
{
    // DMESG("TXD");
    for (int i = 0; i < SERCOM_INST_NUM; i++)
    {
        if (instances[i] && dev == &instances[i]->USART_INSTANCE)
        {
            instances[i]->dataTransmitted();
            return;
        }
    }
}

static void rx_callback(struct ::_usart_async_device* dev, uint8_t data)
{
    // DMESG("RXD %d",data);
    for (int i = 0; i < SERCOM_INST_NUM; i++)
    {
        if (instances[i] && dev == &instances[i]->USART_INSTANCE)
            instances[i]->dataReceived(data);
    }
}

int SAMDSerial::putc(char c)
{
    while(!(CURRENT_USART->USART.INTFLAG.bit.DRE));
    CURRENT_USART->USART.DATA.reg = c;
    return DEVICE_OK;
}

int SAMDSerial::getc()
{
    int c = 0;

    while(!(CURRENT_USART->USART.INTFLAG.bit.RXC));
    c = CURRENT_USART->USART.DATA.reg;

    return c;
}

int SAMDSerial::enableInterrupt(SerialInterruptType t)
{
    // DMESG("INT EN: %d",t);
    if (t == RxInterrupt)
        _usart_async_set_irq_state(&USART_INSTANCE, USART_ASYNC_RX_DONE, true);

    if (t == TxInterrupt)
        _usart_async_set_irq_state(&USART_INSTANCE, USART_ASYNC_BYTE_SENT, true);

    return DEVICE_OK;
}

int SAMDSerial::disableInterrupt(SerialInterruptType t)
{
    // DMESG("INT DIS: %d", t);
    if (t == RxInterrupt)
        _usart_async_set_irq_state(&USART_INSTANCE, USART_ASYNC_RX_DONE, false);

    if (t == TxInterrupt)
        _usart_async_set_irq_state(&USART_INSTANCE, USART_ASYNC_BYTE_SENT, false);

    return DEVICE_OK;
}

#ifdef SERCOM_100MHZ_CLOCK
#define FREQ 100000000
#else
#define FREQ CONF_GCLK_SERCOM0_CORE_FREQUENCY
#endif

int SAMDSerial::setBaudrate(uint32_t baudrate)
{
    uint32_t val = _usart_async_calculate_baud_rate(baudrate, FREQ, 16, USART_BAUDRATE_ASYNCH_ARITHMETIC, 0);
    _usart_async_set_baud_rate(&USART_INSTANCE, val);
    this->baudrate = baudrate;
    return DEVICE_OK;
}

void SAMDSerial::setSercomInstanceValues(Pin& tx, Pin& rx)
{
    const mcu_pin_obj_t* tx_pin = samd_peripherals_get_pin(tx.name);
    const mcu_pin_obj_t* rx_pin = samd_peripherals_get_pin(rx.name);

    // UART can run on many pads, not just zero. This will need to change in the future.
    if (tx_pin->sercom[0].index != 0x3f &&
        (
            tx_pin->sercom[0].pad == 0
#ifdef SAMD21
            || tx_pin->sercom[0].pad == 2
#endif
        )
    )
    {
        this->tx_pad = (tx_pin->sercom[0].pad == 2) ? 0x01 : 0x0;
        this->tx_pinmux = MUX_C; // c
        this->instance_number = tx_pin->sercom[0].index;
    }
    else if (tx_pin->sercom[1].index != 0x3f &&
        (
            tx_pin->sercom[1].pad == 0
#ifdef SAMD21
            || tx_pin->sercom[1].pad == 2
#endif
        )
    )
    {
        this->tx_pad = (tx_pin->sercom[1].pad == 2) ? 0x01 : 0x0;
        this->tx_pinmux = MUX_D; // d
        this->instance_number = tx_pin->sercom[1].index;
    }
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    DMESG("TX pinmux: %d pad: %d inst: %d", this->tx_pinmux, this->tx_pad, this->instance_number);

    if (rx_pin->sercom[0].index == this->instance_number)
    {
        this->rx_pad = rx_pin->sercom[0].pad;
        this->rx_pinmux = MUX_C; // c
    }

    else if (rx_pin->sercom[1].index == this->instance_number)
    {
        this->rx_pad = rx_pin->sercom[1].pad;
        this->rx_pinmux = MUX_D; // d
    }
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    DMESG("RX pinmux: %d pad: %d inst: %d", this->rx_pinmux, this->rx_pad, this->instance_number);
}

void SAMDSerial::enablePins(Pin& tx, Pin& rx)
{
    ZPin* tx_zPin = (ZPin*)&tx;
    ZPin* rx_zPin = (ZPin*)&rx;
    // tx confg
    tx_zPin->_setMux(tx_pinmux);

    CURRENT_USART->USART.CTRLA.bit.ENABLE = 0;
    while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

    CURRENT_USART->USART.CTRLA.bit.SAMPR = 0;
    CURRENT_USART->USART.CTRLA.bit.TXPO = tx_pad;
    CURRENT_USART->USART.CTRLB.bit.CHSIZE = 0;

    // RX confg
    rx_zPin->_setMux(rx_pinmux);

    CURRENT_USART->USART.CTRLA.bit.SAMPR = 0;
    CURRENT_USART->USART.CTRLA.bit.RXPO = rx_pad;
    CURRENT_USART->USART.CTRLB.bit.CHSIZE = 0; // 8 BIT

    // re-enable
    CURRENT_USART->USART.CTRLA.bit.ENABLE = 1;
    while(CURRENT_USART->USART.SYNCBUSY.bit.ENABLE);

    CURRENT_USART->USART.CTRLB.bit.TXEN = 1;
    while(CURRENT_USART->USART.SYNCBUSY.bit.CTRLB);

    CURRENT_USART->USART.CTRLB.bit.RXEN = 1;
    while(CURRENT_USART->USART.SYNCBUSY.bit.CTRLB);

    // DMESG("INST: %d TX: %s %d %d, RX: %d %d %d", this->instance_number, tx.name, tx_pinmux, tx_pad, rx.name, rx_pinmux, rx_pad);
}

int SAMDSerial::configurePins(Pin& tx, Pin& rx)
{
    bool differentSercom = false;
    int oldInstanceNumber = this->instance_number;
    setSercomInstanceValues(tx, rx);

    if (oldInstanceNumber != this->instance_number)
        differentSercom = true;

    if (differentSercom)
    {
        Sercom* instance = sercom_insts[this->instance_number];
        samd_peripherals_sercom_clock_init(instance, instance_number);

        // come from ctor, don't deinit
        if (oldInstanceNumber != 255)
        {
            instances[oldInstanceNumber] = NULL;
            disableInterrupt(RxInterrupt);
            disableInterrupt(TxInterrupt);
            _usart_async_deinit(&USART_INSTANCE);
        }

        _usart_async_init(&USART_INSTANCE, instance);

        USART_INSTANCE.usart_cb.rx_done_cb = rx_callback;
        USART_INSTANCE.usart_cb.tx_byte_sent = tx_callback;
        USART_INSTANCE.usart_cb.tx_done_cb = tx_callback;

        instances[instance_number] = this;
    }

    enablePins(tx, rx);
    this->setBaud(this->baudrate);

    return DEVICE_OK;
}

/**
 * Constructor
 *
 * @param tx the pin instance to use for transmission
 *
 * @param rx the pin instance to use for reception
 *
 **/
SAMDSerial::SAMDSerial(Pin& tx, Pin& rx) : Serial(tx, rx)
{
    // set it to a bizarre instance number initially to trigger clock re-init in confg pins
    this->instance_number = 255;
    this->baudrate = CODAL_SERIAL_DEFAULT_BAUD_RATE;
    memset(&USART_INSTANCE, 0, sizeof(USART_INSTANCE));
    configurePins(tx, rx);
}

SAMDSerial::~SAMDSerial()
{

}