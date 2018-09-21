#ifndef CODAL_Z_TIMER_H
#define CODAL_Z_TIMER_H

#include "codal-core/inc/types/Event.h"
#include "codal-core/inc/driver-models/Timer.h"

namespace codal
{
class ZTimer : public codal::Timer
{
public:
    ZTimer();
    static ZTimer *instance;
    virtual void triggerIn(CODAL_TIMESTAMP t);
    virtual void syncRequest();
    void init();
};
} // namespace codal

#endif
