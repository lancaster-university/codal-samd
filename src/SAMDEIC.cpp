#include "SAMDEIC.h"
#include "codal_target_hal.h"
#include "CodalDmesg.h"

extern "C"
{
#include "external_interrupts.h"
}

extern void string_dbg_const(const char *str);

using namespace codal;

EICFactory* EICFactory::instance = NULL;
EICChannel* EICFactory::instances[EIC_CHANNEL_COUNT];

void EICInterface::pinEventDetected() {}

static void eic_handler(uint8_t channel)
{
    if (EICFactory::instance && EICFactory::instance->instances[channel])
        EICFactory::instance->instances[channel]->trigger();
}


EICChannel::EICChannel(int channel)
{
    DMESG("CHAN: %d ADDR: %x", channel, this);
    this->cb = NULL;
    turn_on_cpu_interrupt(channel);
    this->channel_number = channel;
}

void EICChannel::trigger()
{
    DMESG("TRIG CHAN %d", this->channel_number);

    if (this->cb)
        this->cb->pinEventDetected();
}

void EICChannel::configure(EICEventType t)
{
    this->configuration = t;
    configure_eic_channel(this->channel_number, t);
}

EICEventType EICChannel::getConfiguration()
{
    return this->configuration;
}

void EICChannel::disable()
{
    configure(EICEventsNone);
    turn_off_eic_channel(this->channel_number);
}

void EICChannel::setChangeCallback(EICInterface* interface)
{
    target_disable_irq();
    this->cb = interface;
    target_enable_irq();
}

void EICChannel::enable(EICEventType t)
{
    configure(t);
    turn_on_eic_channel(this->channel_number, t);
}

EICChannel::~EICChannel()
{
    disable();
    EICFactory::instance->free(this->channel_number);
}


EICFactory::EICFactory()
{
    if (instance)
        return;

    instance = this;

    memset(instances, 0, sizeof(EICChannel*) * EIC_CHANNEL_COUNT);

    turn_on_external_interrupt_controller();

    set_eic_irq_handler(eic_handler);

    eic_reset();

    enable();
}

void EICFactory::disable()
{
    eic_set_enable(false);
}

void EICFactory::enable()
{
    eic_set_enable(true);
}

EICChannel* EICFactory::getInstance(int channel)
{
    if (instance->instances[channel] == NULL)
    {
        target_disable_irq();
        EICFactory::instances[channel] = new EICChannel(channel);
        target_enable_irq();
        return EICFactory::instances[channel];
    }

    return NULL;
}

void EICFactory::free(int channel)
{
    EICFactory::instances[channel] = NULL;
}