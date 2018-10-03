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

namespace codal
{

    class ZSingleWireSerial : public DMASingleWireSerial, public DmaComponent
    {
        uint32_t baud;

        DmaInstance* usart_dma;
        int dmaChannel;

        protected:
        virtual void configureRxInterrupt(int enable);

        virtual int configureTx(int);

        virtual int configureRx(int);

        public:

        ZSingleWireSerial(Pin& p);

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