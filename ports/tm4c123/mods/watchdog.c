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

#define WATCHDOG_DEFAULT_TIMEOUT_US 5000 // in us

// Holds the global pointers for the 2 Wachtdogtimer modules that are needed in the IRQ Handler
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

// set/resets the watchdog timer, with the right value so the timeout irq/reset start at the needed time
STATIC void reset_watchdog(machine_watchdog_obj_t *self){
    // Check to see if the registers are locked, and if so, unlock them.
    if(WatchdogLockState(self->watchdog_base) == true){
        WatchdogUnlock(self->watchdog_base);
    }
    
    // no handler registered, only reset requested
    if(self->call_back_fun==MP_OBJ_NULL){
        // here Timer value halfed, because tiva only resets when the irq is not cleared 
        // and the timer reaches zero a second time
        WatchdogIntClear(self->watchdog_base);                  // clears irq, so next time again-> irq first and reset second
        WatchdogReloadSet(self->watchdog_base, self->timeout/2);    
        // Here no handler is registered and the irq does not reach nvic, so no irq in cpu
    } else {
        // handler registered, first timeout calles handler -> if stuck in hander to long -> second timeout reset
        WatchdogReloadSet(self->watchdog_base, self->timeout); 
    }
}

/* ----------- Interrupt functions -------------- */
/* ---------------------------------------------- */

STATIC void watchdog_set_handler(machine_watchdog_obj_t * self,mp_obj_t handler){
    // check if funktion is callable
    if(!mp_obj_is_callable(handler)){
        mp_raise_TypeError(MP_ERROR_TEXT("Callback handler is not callable"));
    }
    // Disable IRQ with mask in NVIC
    WatchdogIntUnregister(self->watchdog_base);

    if(WATCHDOG0_BASE==self->watchdog_base){
        WatchdogIntRegister(self->watchdog_base, &WATCHDOG0_IRQHandler);
    }else{
        WatchdogIntRegister(self->watchdog_base, &WATCHDOG1_IRQHandler);
    }
}

// Interrupthandler
void WATCHDOGGenericIntHandler(uint32_t base){
    machine_watchdog_obj_t *self=glob_wdt_self[WATCHDOG0_BASE==base? 0 : 1];

    if (self->call_back_fun != mp_const_none) {
        // no allocation allowed
        gc_lock();
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            // bus_off irq (idx_msg_obj = 33)   -> def callbackfun(self): ...
            mp_call_function_1(self->call_back_fun,MP_OBJ_FROM_PTR(self));
            nlr_pop();
        } else {
            // Uncaught exception; leads to a restart
            mp_printf(MICROPY_ERROR_PRINTER, "uncaught exception in watchdogtimer(%lu) handler\n", self->wdt_id);
            mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
            WatchdogReloadSet(self->watchdog_base,0);    // immediate reset
        }
        gc_unlock();
        WatchdogIntClear(base);
    }
}

/* ------------ init/deinit Helper--------------- */
/* ---------------------------------------------- */

// Initializes and starts the WDT 
STATIC void watchdog_init(machine_watchdog_obj_t *self){

    // disable WDT module 
    SysCtlPeripheralDisable(self->periph);

    // reset module
    SysCtlPeripheralReset(self->periph);

    // enable modul
    SysCtlPeripheralEnable(self->periph);

    // Wait for the WDT module to be ready.
    while(!SysCtlPeripheralReady(self->periph)){}

    // Check to see if the registers are locked, and if so, unlock them.
    if(WatchdogLockState(self->watchdog_base) == true){
        WatchdogUnlock(self->watchdog_base);
    }

    if(self->call_back_fun!=MP_OBJ_NULL){
        watchdog_set_handler(self,self->call_back_fun);
    }

    // Initialize the watchdog timer.
    reset_watchdog(self);
    // Enable the reset.
    WatchdogResetEnable(self->watchdog_base);
    // Enable the watchdog timer.
    WatchdogEnable(self->watchdog_base);
}

STATIC void watchdog_init_helper(machine_watchdog_obj_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum {ARG_timeout,ARG_handler};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_timeout,          MP_ARG_KW_ONLY | MP_ARG_INT,    {.u_int = WATCHDOG_DEFAULT_TIMEOUT_US} },
        { MP_QSTR_handler,          MP_ARG_KW_ONLY | MP_ARG_OBJ,    {.u_obj = MP_OBJ_NULL} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    machine_watchdog_obj_t *self = (machine_watchdog_obj_t *)self_in;
    
    // --Check the ranges of values-- 
    if(args[ARG_timeout].u_int<0){
        mp_raise_ValueError(MP_ERROR_TEXT("timeout can not be negative"));
    }

    uint32_t clk;

    // id
    if(self->wdt_id==WDT0){
        self->watchdog_base=WATCHDOG0_BASE;
        self->periph=SYSCTL_PERIPH_WDOG0;
        clk=SysCtlClockGet();

    } else if(self->wdt_id==WDT1){
        self->watchdog_base=WATCHDOG1_BASE;
        self->periph=SYSCTL_PERIPH_WDOG1;
        
        // TODO maybe remove
        // long int reg=HWREG(SYSCTL_BASE+0x154);
        // clk=reg & 0x0000007F;
        // printf("%lu",clk);
        clk=16000000;       // PISCO 16 Mhz clk      
    } else{
        mp_raise_ValueError(MP_ERROR_TEXT("Watchdogtimer does not exist"));
    }

    uint32_t timeout_us=args[ARG_timeout].u_int;
    uint32_t timeout_ticks=(uint32_t) (timeout_us*(clk/1000000.0));  // timeout given in us
    uint32_t max_timeout_us=(uint32_t) (0xFFFFFFFF)*(1000000.0/clk);
    if(timeout_us>max_timeout_us){
        mp_printf(MICROPY_ERROR_PRINTER, "Max timeout with current clk -> %u us\n", max_timeout_us);
        mp_raise_ValueError(MP_ERROR_TEXT("timeout value is to big"));
    }

    // --Saving the Settings--
    self->timeout = timeout_ticks;

    self->call_back_fun = args[ARG_handler].u_obj;

    // calling watchdog-Init Function 
    watchdog_init(self);
}

/* --------- Binding for Micropython ------------ */
/* ---------------------------------------------- */
STATIC mp_obj_t machine_watchdog_feed(mp_obj_t self_in){
    machine_watchdog_obj_t *self = MP_OBJ_TO_PTR(self_in);
    reset_watchdog(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_watchdog_feed_obj,machine_watchdog_feed);

// SEE if deinit even makes sense
STATIC mp_obj_t machine_watchdog_deinit(machine_watchdog_obj_t * self){
    // Disable IRQ with mask in NVIC
    WatchdogIntUnregister(self->watchdog_base);
    self->is_enabled=false;
    self->call_back_fun=mp_const_none;
    return mp_const_none;
}


//TODO see what happens on reconfiguring 

// WDT(id,*,timeout)
STATIC mp_obj_t machine_watchdog_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, 2, true);

    // find WDT Port id
    mp_uint_t wdt_idx=wdt_find(args[0]);

    // get/create wdt object
    if (MP_STATE_PORT(machine_watchdog_obj_all)[wdt_idx] == NULL) {
        // create new wdt object
        glob_wdt_self[wdt_idx] = m_new_obj(machine_watchdog_obj_t);
        glob_wdt_self[wdt_idx]->base.type = &machine_watchdog_type;
        glob_wdt_self[wdt_idx]->wdt_id = wdt_idx;
        glob_wdt_self[wdt_idx]->is_enabled = false;
        glob_wdt_self[wdt_idx]->call_back_fun=mp_const_none;
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
        watchdog_init_helper(glob_wdt_self[wdt_idx], n_args - 1, args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(glob_wdt_self[wdt_idx]);
}

// prints importent information about the WDT obj
STATIC void machine_watchdog_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    printf("print called\n");
}


STATIC const mp_rom_map_elem_t machine_watchdog_locals_dict_table[] = {

    { MP_ROM_QSTR(MP_QSTR_feed),            MP_ROM_PTR(&machine_watchdog_feed_obj) },
};

STATIC MP_DEFINE_CONST_DICT(machine_watchdog_locals_dict, machine_watchdog_locals_dict_table);

const mp_obj_type_t machine_watchdog_type = {
    { &mp_type_type },
    .name = MP_QSTR_WDT,
    .print = machine_watchdog_print,
    .make_new = machine_watchdog_make_new,
    .locals_dict = (mp_obj_dict_t *)&machine_watchdog_locals_dict,
};