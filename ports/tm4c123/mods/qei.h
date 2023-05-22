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
#ifndef MICROPY_INCLUDED_QUADENCODER_H
#define MICROPY_INCLUDED_QUADENCODER_H

#include "py/obj.h"
#define QEI0 (0)
#define QEI1 (1) 


typedef struct _machine_qei_obj_t {
    mp_obj_base_t base;
    uint32_t qei_base;                  // base address of quadencoder module
    uint32_t periph;                    // address needed for tivaware sysctl functions
    uint8_t qei_id;  
    uint32_t filter; 
    uint32_t timeout;
    uint32_t vel_div;                 
    bool is_enabled;
    
    bool max_pos;
    bool both_phases;
    bool mode;
    bool idx_reset;
    bool swap;
    mp_obj_t call_back_fun;

} machine_qei_obj_t;


extern const mp_obj_type_t machine_qei_type;
void qei_deinit();

void QEIGenericIntHandler(uint32_t base);

#endif // MICROPY_INCLUDED_QUADENCODER_H