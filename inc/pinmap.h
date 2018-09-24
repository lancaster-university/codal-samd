
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_PINMAP_H
#define MBED_PINMAP_H

#include "platform_includes.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "pins.h"

extern uint8_t used_sercoms[SERCOM_INST_NUM];
const mcu_pin_obj_t *find_mcu_pin(uint8_t pinId);
int find_sercom(const mcu_pin_obj_t *pin, int sercomIdx);

#ifdef __cplusplus
}
#endif

#endif

/** @}*/
