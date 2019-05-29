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
        uint32_t pinmux;
        uint8_t instance_number;
        uint8_t pad;

        protected:
        virtual void configureRxInterrupt(int enable);

        virtual int configureTx(int);

        virtual int configureRx(int);

        public:

        /**
         * This constructor is really ugly, but there isn't currently a nice representation of a model of the device
         * i.e. a resource manager?
         *
         * @param p the pin instance to use for output
         *
         * @param instance the sercom instance that is compatible with p.
         *
         * @param instance_number the index into the sercom array (SERCOM0 == index 0)
         *
         * @param pinmux the pinmux settings for p, i.e. PA08 has pinmux settings PINMUX_PB08D_SERCOM4_PAD0 for sercom four
         *
         * @param pad the pad that the pin is routed through i.e. PA08 uses PAD0 of SERCOM4 (see data sheet).
         **/
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

        int getBytesReceived() override;
        int getBytesTransmitted() override;

        virtual int setMode(SingleWireMode sw);

        virtual int sendBreak();

        void dmaTransferComplete(DmaCode c) override;
    };
}

#endif