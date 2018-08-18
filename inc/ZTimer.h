#ifndef CODAL_Z_TIMER_H
#define CODAL_Z_TIMER_H

#include "codal-core/inc/types/Event.h"
#include "codal-core/inc/driver-models/Timer.h"

namespace codal
{
class ZTimer : public codal::Timer
{
    uint32_t prev;
public:
    TIM_HandleTypeDef TimHandle;
    
    ZTimer();
    static ZTimer *instance;
    void init();
    virtual void triggerIn(CODAL_TIMESTAMP t);
    virtual void syncRequest();
};
} // namespace codal

#endif
