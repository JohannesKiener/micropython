/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_WATCHDOG_H
#define MICROPY_INCLUDED_WATCHDOG_H

#include "py/obj.h"
#define WDT0 (0)
#define WDT1 (1) 


typedef struct _machine_watchdog_obj_t {
    mp_obj_base_t base;

    uint32_t watchdog_base;              // base address of watchdog module
    uint32_t periph;                    // address needed for tivaware sysctl functions
    uint32_t wdt_id;
    bool is_enabled;
    uint32_t timeout;
    // TODO WDOG: add Attributes 

    mp_obj_t call_back_fun;             // Callbackfuntion for time = 0

} machine_watchdog_obj_t;

extern const mp_obj_type_t machine_watchdog_type;
void watchdog_deinit();

void WATCHDOGGenericIntHandler(uint32_t base);

#endif // MICROPY_INCLUDED_WATCHDOG_H