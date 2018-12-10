#ifndef SAMD_SERIAL_H
#define SAMD_SERIAL_H

#include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "SingleWireSerial.h"
#include "JACDAC.h"
#include "pinmap.h"
#include "MemberFunctionCallback.h"
#include "SAMDDMAC.h"
#include "Serial.h"

#include "hal_usart_async.h"
extern "C"
{
#include "sercom.h"
}

namespace codal
{
    class SAMDSerial : public Serial
    {
        uint8_t instance_number;
        uint8_t tx_pinmux;
        uint8_t tx_pad;

        uint8_t rx_pinmux;
        uint8_t rx_pad;

        void setSercomInstanceValues(Pin& tx, Pin& rx);
        void enablePins(Pin& tx, Pin& rx);

        protected:
        virtual int enableInterrupt(SerialInterruptType t);
        virtual int disableInterrupt(SerialInterruptType t);
        virtual int setBaudrate(uint32_t baudrate);
        virtual int configurePins(Pin& tx, Pin& rx);

        public:

        struct ::_usart_async_device USART_INSTANCE;

        virtual int putc(char);
        virtual int getc();

        /**
         * Constructor
         *
         * @param tx the pin instance to use for transmission
         *
         * @param rx the pin instance to use for reception
         *
         **/
        SAMDSerial(Pin& tx, Pin& rx);

        ~SAMDSerial();
    };
}

#endif