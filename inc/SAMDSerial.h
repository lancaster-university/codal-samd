#ifndef ZSINGLE_WIRE_SERIAL_H
#define ZSINGLE_WIRE_SERIAL_H

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
        uint32_t baud;
        struct ::_usart_async_device USART_INSTANCE;
        DmaInstance* usart_tx_dma;
        DmaInstance* usart_rx_dma;
        uint32_t pinmux;
        uint8_t instance_number;

        protected:
        virtual void configureRxInterrupt(int enable);

        virtual int configureTx(int);

        virtual int configureRx(int);

        public:

        /**
         * Constructor
         *
         * @param tx the pin instance to use for transmission
         *
         * @param rx the pin instance to use for reception
         *
         **/
        SAMDSerial(Pin& tx, Pin& rx);

        virtual int putc(char c);
        virtual int getc();

        virtual int send(uint8_t* data, int len);
        virtual int receive(uint8_t* data, int len);

        virtual int sendDMA(uint8_t* data, int len);
        virtual int receiveDMA(uint8_t* data, int len);
        virtual int abortDMA();

        virtual int setBaud(uint32_t baud);
        virtual uint32_t getBaud();

        virtual int setMode(SingleWireMode sw);

        virtual int sendBreak();

        void dmaTransferComplete(DmaCode c);
    };
}

#endif