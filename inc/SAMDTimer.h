#ifndef SAMDTIMER_H
#define SAMDTIMER_H

#include "Timer.h"
#include "sam.h"

namespace codal
{
    class SAMDTimer : public codal::Timer
    {
        uint8_t irqN;
        uint32_t period;

        public:
        uint32_t sigma;
        Tc* tc;

        SAMDTimer(Tc* tc, uint8_t irqn);

#ifdef SAMD51
        SAMDTimer() : SAMDTimer(TC0, TC0_IRQn) {}
#endif
#ifdef SAMD21
        SAMDTimer() : SAMDTimer(TC4, TC4_IRQn) {}
#endif

        static SAMDTimer *instance;

        virtual void triggerIn(CODAL_TIMESTAMP t);

        virtual void syncRequest();
    };
}

#endif