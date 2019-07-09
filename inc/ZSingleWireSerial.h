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

#include "hal_usart_async.h"
extern "C"
{
#include "sercom.h"
}

namespace codal
{
    class ZSingleWireSerial : public DMASingleWireSerial, public DmaComponent
    {
        uint32_t baud;
        struct ::_usart_async_device USART_INSTANCE;
        DmaInstance* usart_tx_dma;
        DmaInstance* usart_rx_dma;
        Pin *rx;
        uint8_t pinmux;
        uint8_t instance_number;
        uint8_t pad;
        uint8_t rx_pinmux;
        uint8_t rx_pad;

        protected:
        virtual void configureRxInterrupt(int enable);

        virtual int configureTx(int);

        virtual int configureRx(int);

        public:

        /**
         * Create a new instance of single wire serial.
         *
         * @param p the pin instance to use for output (and input if no rx given)
         *
         * @param rx the pin instance to use for input
         * 
         **/
        ZSingleWireSerial(Pin& p, Pin *rx = NULL);

        virtual int putc(char c);
        virtual int getc();

        virtual int send(uint8_t* data, int len);
        virtual int receive(uint8_t* data, int len);

        virtual int sendDMA(uint8_t* data, int len);
        virtual int receiveDMA(uint8_t* data, int len);
        virtual int abortDMA();

        virtual int setBaud(uint32_t baud);
        virtual uint32_t getBaud();

        int getBytesReceived() override;
        int getBytesTransmitted() override;

        virtual int setMode(SingleWireMode sw);

        virtual int sendBreak();

        void dmaTransferComplete(DmaInstance *dma, DmaCode c) override;
    };
}

#endif