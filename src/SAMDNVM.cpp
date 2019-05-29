#include "SAMDNVM.h"
#include "ErrorNo.h"
#include "CodalDmesg.h"

#ifdef SAMD21
using namespace codal;

// this actually generates less code than a function
#define wait_ready()                                                                               \
    while (NVMCTRL->INTFLAG.bit.READY == 0)                                                        \
        ;

static const uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
uint32_t flash_page_size()
{
    return pageSizes[NVMCTRL->PARAM.bit.PSZ];
}

uint32_t flash_eep_size()
{
    return 0;
#if 0
    return NVMCTRL->PARAM.bit.RWWEEP * flash_page_size();
#endif
}

uint32_t flash_row_size()
{
    return NVMCTRL->PARAM.bit.PSZ * 4;
}

uint32_t flash_page_count()
{
    return NVMCTRL->PARAM.bit.NVMP;
}

void flash_erase_row(uint32_t *dst) {
    wait_ready();
    NVMCTRL->STATUS.reg = NVMCTRL_STATUS_MASK;

    // Execute "ER" Erase Row
    NVMCTRL->ADDR.reg = (uint32_t)dst / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    wait_ready();
}

void copy_words(uint32_t *dst, uint32_t *src, uint32_t n_words) {
    while (n_words--)
        *dst++ = *src++;
}

void flash_write_words(uint32_t *dst, uint32_t *src, uint32_t n_words) {
    // Set automatic page write
    NVMCTRL->CTRLB.bit.MANW = 0;

    while (n_words > 0) {
        uint32_t len = (FLASH_PAGE_SIZE >> 2) < n_words ? (FLASH_PAGE_SIZE >> 2) : n_words;
        n_words -= len;

        // Execute "PBC" Page Buffer Clear
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
        wait_ready();

        // make sure there are no other memory writes here
        // otherwise we get lock-ups

        while (len--)
            *dst++ = *src++;

        // Execute "WP" Write Page
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
        wait_ready();
    }
}

// Skip writing blocks that are identical to the existing block.
// only disable for debugging/timing
#define QUICK_FLASH 1

void flash_write_row(uint32_t *dst, uint32_t *src) {
#if QUICK_FLASH
    bool src_different = false;
    for (uint32_t i = 0; i < flash_row_size() / 4; ++i) {
        if (src[i] != dst[i]) {
            src_different = true;
            break;
        }
    }

    if (!src_different) {
        return;
    }
#endif

    flash_erase_row(dst);
    flash_write_words(dst, src, flash_row_size() / 4);
}

SAMDNVM::SAMDNVM()
{

}

uint32_t* SAMDNVM::getFlashStart()
{
    return (uint32_t*)0;
}

uint32_t* SAMDNVM::getFlashEnd()
{
    return (uint32_t*)(((uint8_t*)getFlashStart()) + (getFlashSize() - flash_eep_size()));
}

uint32_t SAMDNVM::getPageSize()
{
    return flash_page_size();
}


uint32_t SAMDNVM::getFlashSize()
{
    return flash_page_count() * flash_page_size();
}

int SAMDNVM::copy(uint32_t* dest, uint32_t* source, uint32_t size)
{
    copy_words(dest, source, size);
    return DEVICE_OK;
}

int SAMDNVM::erase(uint32_t* page)
{
    flash_erase_row(page);
    return DEVICE_OK;
}

int SAMDNVM::write(uint32_t* dst, uint32_t* source, uint32_t size)
{
    flash_write_words(dst, source, size);
    return DEVICE_OK;
}
#endif