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

        static SAMDTimer *instance;

        virtual void triggerIn(CODAL_TIMESTAMP t);

        virtual void syncRequest();

        void enable();
    };
}

#endif