#ifndef ZSINGLE_WIRE_SERIAL_H
#define ZSINGLE_WIRE_SERIAL_H

#include "Pin.h"
#include "CodalComponent.h"
#include "CodalConfig.h"
#include "SingleWireSerial.h"
#include "PktSerial.h"
#include "pinmap.h"
#include "MemberFunctionCallback.h"

namespace codal
{

    class ZSingleWireSerial : public DMASingleWireSerial
    {
        UART_HandleTypeDef uart;
        DMA_HandleTypeDef hdma_tx;
        DMA_HandleTypeDef hdma_rx;

        uint8_t* buf;
        uint16_t bufLen;

        protected:
        virtual void configureRxInterrupt(int enable);

        virtual int configureTx(int);

        virtual int configureRx(int);

        public:

        static void _complete(uint32_t instance, uint32_t mode);

        PktSerialPkt* currentBuffer;
        uint32_t currentBufferIndex;

        // only works with a TX uart pin on STM.
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
    };
}

#endif