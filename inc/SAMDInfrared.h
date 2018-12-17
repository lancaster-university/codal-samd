#ifndef SAMD_INFRARED_H
#define SAMD_INFRARED_H

#include "CodalConfig.h"
#include "LowLevelTimer.h"
#include "CodalComponent.h"
#include "BitVector.h"

#define PULSE_MAX_MSG_SIZE 34
#define PULSE_PACKET_END_EVENT 0x1
#define PULSE_PACKET_EVENT 0x2
#define PULSE_PACKET_ERROR_EVENT 0x3
#define PULSE_MAX_PULSES (PULSE_MAX_MSG_SIZE * 14 + 10)
#define PULSE_PULSE_LEN 250

#define PULSE_IR_COMPONENT_ID 0x2042
#define PULSE_CABLE_COMPONENT_ID 0x2043

#define PULSE_DEBUG 0

#if PULSE_DEBUG
#define PULSE_DMESG DMESG
#else
#define PULSE_DMESG(...)                                                                           \
    do {                                                                                           \
    } while (0)
#endif

namespace codal
{
enum PulseRecvState : uint8_t {
    PULSE_RECV_ERROR,
    PULSE_WAIT_START_GAP,
    PULSE_WAIT_DATA,
};

class PulseBase : public CodalComponent
{
  protected:
    Pin& pinOut;
    Pin& pinIn;
    LowLevelTimer& timer;

    BitVector encodedMsg;
    uint16_t sendPtr;
    int8_t pwmstate;
    bool sending;
    uint16_t id;
    uint64_t startTime;
    uint64_t sendStartTime;
    uint64_t lastMarkTime;
    uint64_t lastSendTime;

    int16_t pulses[PULSE_MAX_PULSES + 1];
    uint16_t pulsePtr;

    PulseRecvState recvState;
    Buffer outBuffer;

    DbgBuffer dbg;

  public:
    PulseBase(Pin& pinOut, Pin& pinIn, LowLevelTimer& timer, int16_t id);

    virtual void setupGapEvents();
    virtual void listen();
    virtual void setupPWM();
    virtual void setPWM(int enabled);
    virtual void finishPWM();
    void send(Buffer d);
    void finish(int code);
    void addPulse(int v);
    int adjustShift();
    void pulseGap(Event ev);
    int errorRate(int start, BitVector &bits);
    void packetEnd(Event);
    void pulseMark(Event ev);
    Buffer getBuffer();
    bool isReciving();
    void process();
};
}
#endif