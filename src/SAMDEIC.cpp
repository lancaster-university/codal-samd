#include "SAMDEIC.h"

extern "C"
{
#include "external_interrupts.h"
}

using namespace codal;

EICFactory* EICFactory::instance = NULL;

static void eic_handler(uint8_t channel)
{
    if (EICFactory::instance && EICFactory::instance->instances[channel])
        EICFactory::instance->instances[channel]->trigger(0);
}


EICChannel::EICChannel(int channel)
{
    turn_on_cpu_interrupt(channel);
    this->channel_number = channel;
}

void EICChannel::trigger(uint8_t type)
{
    if (cb)
        cb->onEICEvent((EICEventType)type);
}

void EICChannel::configure(EICEventType t)
{
    configure_eic_channel(this->channel_number, t);
}

void EICChannel::disable()
{
    turn_on_eic_channel(this->channel_number, EICEventsNone);
    turn_off_eic_channel(this->channel_number);
}

void EICChannel::onEICEvent(EICInterface* interface)
{
    this->cb = interface;
}

void EICChannel::enable(EICEventType t)
{
    turn_on_eic_channel(this->channel_number, t);
}

EICChannel::~EICChannel()
{
    EICFactory::instance->free(this);
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
}

/**
 * Disables all confgures DMA activity.
 * Typically required before configuring DMA descriptors and DMA channels.
 */
void EICFactory::disable()
{
    eic_set_enable(false);
}

/**
 * Enables all confgures DMA activity
 */
void EICFactory::enable()
{
    eic_set_enable(true);
}

EICChannel* EICFactory::getInstance(int channel)
{
    if (instances[channel] == NULL)
    {
        instances[channel] = new EICChannel(channel);
        return instances[channel];
    }

    return NULL;
}

void EICFactory::free(EICChannel* instance)
{
    for (int i = 0; i < EIC_CHANNEL_COUNT; i++)
    {
        if (instances[i] == instance)
            instances[i] = NULL;
    }
}
