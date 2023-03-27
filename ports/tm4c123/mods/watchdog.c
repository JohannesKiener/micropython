/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2019 Damien P. George
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

#include <stdio.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/builtin.h"
#include "py/objarray.h" 
#include "mpconfigboard.h"
#include "watchdog.h"
#include "mphalport.h"
#include "handlers.h"
#include "mpirq.h"

#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"
#include "inc/hw_ints.h"

#define WATCHDOG_DEFAULT_TIMEOUT 5000 // in ms
//#define WATCHDOG_MAX_TIMEOUT #TODO

// Holds the global pointers for the 2 Wachtdogtimer modules that are needed in the IRQ Handler
// TODO Maybe not needed as global
STATIC machine_watchdog_obj_t *glob_wdt_self[2];    


// Works out the Port ID
STATIC int wdt_find(mp_obj_t id) {
    if (MP_OBJ_IS_STR(id)) {
        // given a string id
        const char *port = mp_obj_str_get_str(id);
        if (0) {
            #ifdef MICROPY_HW_WATCHDOG0_NAME
        } else if (strcmp(port, MICROPY_HW_WATCHDOG0_NAME) == 0) {
            return WDT0;
            #endif
            #ifdef MICROPY_HW_WATCHDOG1_NAME
        } else if (strcmp(port, MICROPY_HW_WATCHDOG1_NAME) == 0) {
            return WDT1;
            #endif
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("Wachtdog Timer(%s) doesn't exist"), port));
    } else {
        // given an integer id
        int wdt_id = mp_obj_get_int(id);
        if (wdt_id >= 0 && wdt_id < MP_ARRAY_SIZE(MP_STATE_PORT(machine_watchdog_obj_all))) {
            return wdt_id;
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("Wachtdog Timer(%d) doesn't exist"), wdt_id));
    }
}


/* ----------- Interrupt functions -------------- */
/* ---------------------------------------------- */


/* ------------ init/deinit Helper--------------- */
/* ---------------------------------------------- */

// Initializes the CAN Module und it's Pins
void watchdog_init(machine_watchdog_obj_t *self){

    // TODO
    uint32_t a=SysCtlClockGet();
    printf("clk:%lu\n",a);

    if(self->wdt_id==WDT0){
        self->watchdog_base=WATCHDOG0_BASE;
        self->periph=SYSCTL_PERIPH_WDOG0;

    } else if(self->wdt_id==WDT1){
        self->watchdog_base=WATCHDOG1_BASE;
        self->periph=SYSCTL_PERIPH_WDOG1;

    } else{
         // CAN does not exist for this board (shouldn't get here, should be checked by caller)
        return;
    }

    // disable WDT module 
    SysCtlPeripheralDisable(self->periph);

    // reset module
    SysCtlPeripheralReset(self->periph);

    // enable modul
    SysCtlPeripheralEnable(self->periph);

    // Wait for the CAN0 module to be ready.
    while(!SysCtlPeripheralReady(self->periph)){}

    // Check to see if the registers are locked, and if so, unlock them.
    if(WatchdogLockState(self->watchdog_base) == true){
        WatchdogUnlock(self->watchdog_base);
    }

    // Initialize the watchdog timer.
    WatchdogReloadSet(self->watchdog_base, self->timeout);
    // Enable the reset.
    WatchdogResetEnable(self->watchdog_base);
    // Enable the watchdog timer.
    WatchdogEnable(self->watchdog_base);

    self->is_enabled=true;
}

STATIC void machine_watchdog_init_helper(machine_watchdog_obj_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum {ARG_timeout};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_timeout,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = WATCHDOG_DEFAULT_TIMEOUT} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_watchdog_obj_t *self = (machine_watchdog_obj_t *)self_in;

    // --Check the ranges of values-- 
    // TODO check to big/small values 
    // if(args[ARG_timeout].u_int>64){
    //     mp_raise_TypeError(MP_ERROR_TEXT("prescaler must be within [1,...,64]"));
    // }

    // --Saving the Settings--
    self->timeout = args[ARG_timeout].u_int;
       
    // calling CAN-Init Function 
    watchdog_init(self);
}

/* --------- Binding for Micropython ------------ */
/* ---------------------------------------------- */

STATIC mp_obj_t machine_watchdog_deinit(){
    //TODO
    return mp_const_none;
}

// WDT(id,*,timeout)
STATIC mp_obj_t machine_watchdog_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, 1, true);

    printf("nargs: %u, kw_args %u\n",n_args,n_kw);
    // find WDT Port id
    mp_uint_t wdt_idx=wdt_find(args[0]);

    // get/create wdt object
    if (MP_STATE_PORT(machine_watchdog_obj_all)[wdt_idx] == NULL) {
        // create new can object
        glob_wdt_self[wdt_idx] = m_new_obj(machine_watchdog_obj_t);
        glob_wdt_self[wdt_idx]->base.type = &machine_watchdog_type;
        glob_wdt_self[wdt_idx]->wdt_id = wdt_idx;
        glob_wdt_self[wdt_idx]->is_enabled = false;
        MP_STATE_PORT(machine_watchdog_obj_all)[wdt_idx] = glob_wdt_self[wdt_idx];
    } else {
        // reference existing wdt object
        glob_wdt_self[wdt_idx] = MP_STATE_PORT(machine_watchdog_obj_all)[wdt_idx];
    }

    // The caller is requesting a reconfiguration of the hardware
    // this can only be done if the hardware is in init mode
    if (glob_wdt_self[wdt_idx]->is_enabled) {
        machine_watchdog_deinit((mp_obj_t)glob_wdt_self[wdt_idx]);
    }
    
    
    if (n_args > 0 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        machine_watchdog_init_helper(glob_wdt_self[wdt_idx], n_args - 1, args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(args[0]);
}

// prints importent information about the WDT obj
STATIC void machine_watchdog_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    printf("print called\n");
}


STATIC const mp_rom_map_elem_t machine_watchdog_locals_dict_table[] = {

//    { MP_ROM_QSTR(MP_QSTR_send),            MP_ROM_PTR(&mp_machine_hard_can_send_obj) },
//    { MP_ROM_QSTR(MP_QSTR_receive),         MP_ROM_PTR(&mp_machine_hard_can_receive_obj) },
};

STATIC MP_DEFINE_CONST_DICT(machine_watchdog_locals_dict, machine_watchdog_locals_dict_table);

const mp_obj_type_t machine_watchdog_type = {
    { &mp_type_type },
    .name = MP_QSTR_WDT,
    .print = machine_watchdog_print,
    .make_new = machine_watchdog_make_new,
    .locals_dict = (mp_obj_dict_t *)&machine_watchdog_locals_dict,
};

// TODO Filter