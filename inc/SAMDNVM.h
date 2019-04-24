#ifndef SAMD_NVM_H
#define SAMD_NVM_H

#include "CodalConfig.h"
#include "NVMController.h"

namespace codal
{
    class SAMDNVM : public NVMController
    {
        public:

        SAMDNVM();

        virtual uint32_t* getFlashEnd() override;

        virtual uint32_t* getFlashStart() override;

        virtual uint32_t getPageSize() override;

        virtual uint32_t getFlashSize() override;

        virtual int copy(uint32_t* dest, uint32_t* source, uint32_t size) override;

        virtual int erase(uint32_t* page) override;

        virtual int write(uint32_t* dst, uint32_t* source, uint32_t size) override;
    };
}



#endif