/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "CodalUSB.h"

#if CONFIG_ENABLED(DEVICE_USB)
#include "pinmap.h"
#include "PeripheralPins.h"
#include "CodalDmesg.h"
#include "Timer.h"

#define LOG DMESG
#define DBG DMESG

#if defined(STM32F1) || defined(STM32F2)
#define NUM_EP 3
#elif defined(STM32F4)
#if defined(STM32F446) || defined(STM32F469) || defined(STM32F479)
#define NUM_EP 5
#else
#define NUM_EP 3
#endif
#elif defined(STM32F7) || defined(STM32L4)
#define NUM_EP 5
#elif defined(STM32H7)
#define NUM_EP 8
#else
#error "Invalid chip"
#endif

#define NUM_IN_EP (NUM_EP + 1)
#define NUM_OUT_EP (NUM_EP + 1)
#define NUM_BIDIR_EP (NUM_EP + 1)

// some devices have USB_OTG_FS, others just USB
#ifdef USB
#define USBx USB
#else /* USB_OTG_FS */
//#define NUM_EP (USB_OTG_FS_MAX_IN_ENDPOINTS - 1)
#define FIFO_NUM (1 + NUM_IN_EP)
#define FIFO_WORDS (USB_OTG_FS_TOTAL_FIFO_SIZE / 4)
#define FIFO_EP_WORDS (FIFO_WORDS / FIFO_NUM)
#define USBx USB_OTG_FS
#endif /* USB */

static PCD_HandleTypeDef pcd;
static IRQn_Type irqn;

#ifdef USB
static uint16_t pmaOffset;
#endif

#define CHK(call)                                                                                  \
    do                                                                                             \
    {                                                                                              \
        auto status = call;                                                                        \
        usb_assert(status == HAL_OK);                                                              \
    } while (0)

void usb_configure(uint8_t numEndpoints)
{
    usb_assert(pcd.Instance == 0);

    for (int i = 0; PinMap_USB[i].pin != NC; ++i)
        pinmap_pinout(PinMap_USB[i].pin, PinMap_USB);

#ifdef USB
    __HAL_RCC_USB_CLK_ENABLE();
    irqn = USB_IRQn;
    pcd.Init.speed = PCD_SPEED_FULL;
#else  /* USB_OTG_FS */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    irqn = OTG_FS_IRQn;
    pcd.Init.speed = USB_OTG_SPEED_FULL;
#endif /* USB */

    pcd.Instance = USBx;
    pcd.Init.phy_itface = PCD_PHY_EMBEDDED;
    pcd.Init.ep0_mps = USB_MAX_PKT_SIZE;
    pcd.Init.dev_endpoints = NUM_BIDIR_EP;

    CHK(HAL_PCD_Init(&pcd));
    CHK(HAL_PCD_Start(&pcd));

#ifdef USB
    pmaOffset = 8 * NUM_IN_EP;
#else  /* USB_OTG_FS */
    HAL_PCDEx_SetRxFiFo(&pcd, FIFO_EP_WORDS);
    for (int i = 0; i < NUM_IN_EP; i++)
        HAL_PCDEx_SetTxFiFo(&pcd, i, FIFO_EP_WORDS);
#endif /* USB */

    NVIC_SetPriority(irqn, 5);
    NVIC_EnableIRQ(irqn);
}

#if 0
static UsbEndpointIn *findInEp(int ep)
{
    auto cusb = CodalUSB::usbInstance;
    ep &= 0x7f;
    if (ep == 0)
        return cusb->ctrlIn;
    for (auto iface = cusb->interfaces; iface; iface = iface->next)
    {
        if (iface->in && iface->in->ep == ep)
            return iface->in;
    }
    return NULL;
}
#endif

static UsbEndpointOut *findOutEp(int ep)
{
    auto cusb = CodalUSB::usbInstance;
    if (ep == 0)
        return cusb->ctrlOut;
    for (auto iface = cusb->interfaces; iface; iface = iface->next)
    {
        if (iface->out && iface->out->ep == ep)
            return iface->out;
    }
    return NULL;
}

#define NO_IRQ 0x8000
#define SIZE_MASK 0xfff

extern "C"
{

    void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&pcd); }

    /* Callbacks from the STM32 Cube HAL code */

    void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
    {
        LOG("USB end of reset");
        CodalUSB::usbInstance->initEndpoints();
    }

    void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
    {
        LOG("USB setup");
        USBSetup stp;
        memcpy(&stp, &hpcd->Setup, sizeof(stp));
        // USB_EP0_OutStart(pcd.Instance, pcd.Init.dma_enable, (uint8_t *)pcd.Setup);
        CodalUSB::usbInstance->setupRequest(stp);
        // CodalUSB::usbInstance->ctrlOut->startRead();
    }

    void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
    {
        auto ep = findOutEp(epnum);
        ep->userdata = (ep->userdata & ~SIZE_MASK) | HAL_PCD_EP_GetRxCount(&pcd, epnum);
        if (!(ep->userdata & NO_IRQ))
            CodalUSB::usbInstance->interruptHandler();
    }

    void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
    {
        // we shouldn't be getting this
    }

    void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) { LOG("USB connect"); }

    void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) { LOG("USB disconnect"); }

    void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) { LOG("USB suspend"); }

    void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) { LOG("USB resume"); }
}

void usb_set_address(uint16_t wValue) {}

void usb_set_address_pre(uint16_t wValue)
{

    DBG("ctl=%p", USBx_OUTEP(0)->DOEPCTL);

    LOG("set address %d", wValue);
    CHK(HAL_PCD_SetAddress(&pcd, wValue));

    DBG("ctl2=%p", USBx_OUTEP(0)->DOEPCTL);
    auto cusb = CodalUSB::usbInstance;
    cusb->ctrlOut->startRead();
    for (auto iface = cusb->interfaces; iface; iface = iface->next)
        if (iface->out)
            iface->out->startRead();

    DBG("ctl3=%p", USBx_OUTEP(0)->DOEPCTL);
}

int UsbEndpointIn::clearStall()
{
    LOG("clear stall IN %d", ep);
    CHK(HAL_PCD_EP_ClrStall(&pcd, ep | 0x80));
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::reset()
{
    LOG("reset IN %d", ep);
    // TODO?
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointIn::stall()
{
    LOG("stall IN %d", ep);
    CHK(HAL_PCD_EP_SetStall(&pcd, ep | 0x80));
    wLength = 0;
    return DEVICE_OK;
}

int UsbEndpointOut::clearStall()
{
    LOG("clear stall OUT %d", ep);
    CHK(HAL_PCD_EP_ClrStall(&pcd, ep));
    return DEVICE_OK;
}

int UsbEndpointOut::reset()
{
    LOG("reset OUT %d", ep);
    // TODO
    return DEVICE_OK;
}

int UsbEndpointOut::stall()
{
    LOG("stall OUT %d", ep);
    CHK(HAL_PCD_EP_SetStall(&pcd, ep));
    return DEVICE_OK;
}

UsbEndpointIn::UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;
    flags = 0;
    userdata = 0;

    if (type == USB_EP_TYPE_INTERRUPT)
        flags = USB_EP_FLAG_NO_AUTO_ZLP;

#ifdef USB
    HAL_PCDEx_PMAConfig(&pcd, ep | 0x80, PCD_SNG_BUF, pmaOffset);
    pmaOffset += size;
#endif

    CHK(HAL_PCD_EP_Open(&pcd, ep | 0x80, size, type));
}

UsbEndpointOut::UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size)
{
    usb_assert(size == 64);
    usb_assert(type <= USB_EP_TYPE_INTERRUPT);
    ep = idx;
    userdata = 0;

#ifdef USB
    HAL_PCDEx_PMAConfig(&pcd, ep, PCD_SNG_BUF, pmaOffset);
    pmaOffset += size;
#endif

    CHK(HAL_PCD_EP_Open(&pcd, ep, size, type));

    enableIRQ();
}

int UsbEndpointOut::disableIRQ()
{
    userdata |= 0x8000;
    return DEVICE_OK;
}

int UsbEndpointOut::enableIRQ()
{
    userdata &= ~0x8000;
    return DEVICE_OK;
}

void UsbEndpointOut::startRead()
{
    DBG("OUT %d start read", ep);
    CHK(HAL_PCD_EP_Receive(&pcd, ep, buf, USB_MAX_PKT_SIZE));
}

int UsbEndpointOut::read(void *dst, int maxlen)
{
    usb_assert(this != NULL);

    int packetSize = userdata & 0xff;

    if (packetSize)
    {
        // LOG("USBRead(%d) => %d bytes", ep, packetSize);
        userdata -= packetSize;
        // Note that we shall discard any excessive data
        if (packetSize > maxlen)
            packetSize = maxlen;
        memcpy(dst, buf, packetSize);
        startRead();
    }

    return packetSize;
}

static void writeEP(uint8_t *data, uint8_t ep, int len)
{
    usb_assert(len <= USB_MAX_PKT_SIZE);
    uint32_t len32b = (len + 3) / 4;

    DBG("%d write space: %d words", (int)codal::system_timer_current_time_us(),
        USBx_INEP(ep)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV);

    while ((USBx_INEP(ep)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) < len32b)
        ;

    DBG("write: %p len=%d at IN %d", data, len, ep);
    CHK(HAL_PCD_EP_Transmit(&pcd, ep | 0x80, data, len));

    if (len)
        CHK(USB_WritePacket(pcd.Instance, data, ep, len, 0));

    USBx_DEVICE->DIEPEMPMSK &= ~0x1U << ep;

#if 0
    while (!(USB_ReadDevInEPInterrupt(pcd.Instance, ep) & USB_OTG_DIEPINT_XFRC))
        ;
    CLEAR_IN_EP_INTR(ep, USB_OTG_DIEPINT_XFRC);
#endif

    DBG("%d write done, ctl=%p", (int)codal::system_timer_current_time_us(),
        USBx_OUTEP(0)->DOEPCTL);

    // there is a ZLP from the host coming
    if (len && ep == 0)
        CodalUSB::usbInstance->ctrlOut->startRead();
}

int UsbEndpointIn::write(const void *src, int len)
{
    // this happens when someone tries to write before USB is initialized
    usb_assert(this != NULL);

    int zlp = !(flags & USB_EP_FLAG_NO_AUTO_ZLP);

    if (wLength)
    {
        if (len >= wLength)
        {
            len = wLength;
            // see
            // https://stackoverflow.com/questions/3739901/when-do-usb-hosts-require-a-zero-length-in-packet-at-the-end-of-a-control-read-t
            zlp = 0;
        }
        wLength = 0;
    }

    NVIC_DisableIRQ(irqn);

    for (;;)
    {
        int n = len;
        if (n > USB_MAX_PKT_SIZE)
            n = USB_MAX_PKT_SIZE;
        memcpy(buf, src, len);
        writeEP(buf, ep, len);
        len -= n;
        src = (const uint8_t *)src + n;
        if (!len)
            break;
    }

    // send ZLP manually if needed.
    if (zlp && len && (len & (USB_MAX_PKT_SIZE - 1)) == 0)
    {
        writeEP(buf, ep, 0);
    }

    NVIC_EnableIRQ(irqn);

    return DEVICE_OK;
}

#endif