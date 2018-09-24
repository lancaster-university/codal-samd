#include "pinmap.h"

uint8_t used_sercoms[SERCOM_INST_NUM];

const mcu_pin_obj_t *find_mcu_pin(uint8_t pinId)
{
    int i = 0;
    while (samd_pins[i].number != PIN_NONE)
    {
        if (samd_pins[i].number == pinId)
            return &samd_pins[i];
        i++;
    }
    return NULL;
}

int find_sercom(const mcu_pin_obj_t *pin, int sercomIdx)
{
    if (!pin)
        return -2;
    for (int j = 0; j < NUM_SERCOMS_PER_PIN; j++)
    {
        if (pin->sercom[j].index == sercomIdx)
        {
            return j;
        }
    }
    return -1;
}
