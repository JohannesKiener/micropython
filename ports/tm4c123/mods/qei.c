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
#include "qei.h"
#include "mphalport.h"
#include "handlers.h"
#include "mpirq.h"

#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/qei.h"
#include "inc/hw_ints.h" 



// Works out the Port ID
STATIC int qei_find(mp_obj_t id) {
    if (MP_OBJ_IS_STR(id)) {
        // given a string id
        const char *port = mp_obj_str_get_str(id);
        if (0) {
            #ifdef MICROPY_HW_QEI0_NAME
        } else if (strcmp(port, MICROPY_HW_QEI0_NAME) == 0) {
            return QEI0;
            #endif
            #ifdef MICROPY_HW_QEI1_NAME
        } else if (strcmp(port, MICROPY_HW_QEI1_NAME) == 0) {
            return QEI1;
            #endif
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("Quadratur encoder (%s) doesn't exist"), port));
    } else {
        // given an integer id
        int qei_id = mp_obj_get_int(id);
        if (qei_id >= 0 && qei_id < MP_ARRAY_SIZE(MP_STATE_PORT(machine_qei_obj_all))) {
            return qei_id;
        }
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
            MP_ERROR_TEXT("Quadratur encoder (%s) doesn't exist"), qei_id));
    }
}


/* ------------ init/deinit Helper--------------- */
/* ---------------------------------------------- */

// Initializes and starts the WDT 
STATIC void qei_init(machine_qei_obj_t *self){
    const pin_obj_t *pins[3]={NULL,NULL,NULL};

    if(0){

    #if defined(MICROPY_HW_QEI0_PHA)
    } else if(self->qei_id==QEI0){
    
        self->qei_base=QEI0_BASE;
        self->periph=SYSCTL_PERIPH_QEI0;
        
        pins[0]=MICROPY_HW_QEI0_PHA;
        #if defined(MICROPY_HW_QEI0_PHB)
        pins[1]=MICROPY_HW_QEI0_PHB;
        // pin PD7 is locked
        mp_hal_unlock_special_pin(pins[1]);
        #endif
        #if defined (MICROPY_HW_QEI0_IDX)
        pins[2]=MICROPY_HW_QEI0_IDX;
        #endif
    #endif

    #if defined(MICROPY_HW_QEI1_PHA)
    } else if(self->qei_id==QEI1){
    
        self->qei_base=QEI1_BASE;
        self->periph=SYSCTL_PERIPH_QEI1;
        
        pins[0]=MICROPY_HW_QEI1_PHA;
        #if defined(MICROPY_HW_QEI1_PHB)
        pins[1]=MICROPY_HW_QEI1_PHB;
        #endif
        #if defined (MICROPY_HW_QEI1_IDX)
        pins[2]=MICROPY_HW_QEI1_IDX;
        #endif
    #endif
    } else{
        return;
    }
    // CONFIG PINS  
    bool error0=mp_hal_pin_config_alt(pins[0], PIN_FN_QEI, -1);
    bool error1=mp_hal_pin_config_alt(pins[1], PIN_FN_QEI, -1);
    bool error2=mp_hal_pin_config_alt(pins[2], PIN_FN_QEI, -1);

    // pin error
    if (!(error0&&error1&&error2)){
        // TODO display pins of given qei
        mp_raise_ValueError(MP_ERROR_TEXT("Pin Error"));   
    }

    // disable QEI module 
    SysCtlPeripheralDisable(self->periph);

    // reset module
    SysCtlPeripheralReset(self->periph);

    // enable QEI module 
    SysCtlPeripheralEnable(self->periph);

    // Wait for the QEI module to be ready.
    while(!SysCtlPeripheralReady(self->periph))
    {
    }
    // -----Configure QEI Module
    uint32_t flags=0;

    // Operation mode
    flags|=(self->mode? QEI_CONFIG_CLOCK_DIR:QEI_CONFIG_QUADRATURE);
    flags|=(self->both_edges? QEI_CONFIG_CAPTURE_A_B:QEI_CONFIG_CAPTURE_A);
    flags|=(self->idx_reset? QEI_CONFIG_RESET_IDX:QEI_CONFIG_NO_RESET);
    flags|=(self->swap? QEI_CONFIG_SWAP:QEI_CONFIG_NO_SWAP);

    QEIConfigure(self->qei_base, flags,self->max_pos-1);
    self->is_enabled=true;

    // Filter config [2-17] (pulses) -> [0-15] (Bits)
    QEIFilterConfigure(self->qei_base,(self->filter-2)<<16);  //TODO catch filter [2-17]

    if(self->filter!=0){
        QEIFilterEnable(self->qei_base);
    }else{
        QEIFilterDisable(self->qei_base);
    }

    QEIEnable(self->qei_base);
}
// (maxpos,*,mode=False, both_edges=true, idx_reset=false, swap=False, filter=0)
STATIC void qei_init_helper(machine_qei_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum {ARG_maxpos,ARG_mode, ARG_both_edges,ARG_idx_reset,ARG_swap,ARG_filter};

    // TODO maybe add REQUIRED
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_max_pos,      MP_ARG_INT,                      {.u_int  = 0} },
        { MP_QSTR_mode,         MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },           // TODO changes mode
        { MP_QSTR_both_edges,   MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },            // TODO see if both edegs make sense in dir mode
        { MP_QSTR_idx_reset,    MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },
        { MP_QSTR_swap,         MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },
        { MP_QSTR_filter,       MP_ARG_KW_ONLY | MP_ARG_INT,     {.u_int = 0} },

    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    // // TODO --Check the ranges of values-- 
    // if(args[ARG_timeout].u_int<0){
    //     mp_raise_ValueError(MP_ERROR_TEXT("timeout can not be negative"));
    // }


    // --Saving the Settings--
    self->max_pos = args[ARG_maxpos].u_bool;
    self->both_edges = args[ARG_both_edges].u_bool;
    // TODO Test dir mode and idx_reset
    self->mode = args[ARG_mode].u_bool;
    self->idx_reset=args[ARG_mode].u_bool;
    self->swap = args[ARG_swap].u_bool;

    // TODO maybe add time instead of cycles
    if (args[ARG_filter].u_int == 0 || args[ARG_filter].u_int >=2 || args[ARG_filter].u_int <=17){
        self->filter = args[ARG_filter].u_int;
    }else{
        mp_raise_ValueError(MP_ERROR_TEXT("Filter can only be between [2,17]"));
    }

    // calling watchdog-Init Function 
    qei_init(self);
}

void qei_deinit(machine_qei_obj_t * self){
    // Disable IRQ with mask in NVIC
    printf("Deinit\n");
    // TODO
}


/* --------- Binding for Micropython ------------ */
/* ---------------------------------------------- */

STATIC mp_obj_t machine_qei_set_pos(mp_obj_t self_in,mp_obj_t pos_in){
    
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t pos= mp_obj_get_int(pos_in);
    if(pos>=40){
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
        MP_ERROR_TEXT("Position needs to be smaller than (%u)"), self->max_pos));
    }

    QEIPositionSet(self->qei_base,pos);

    return mp_const_none;
}
  
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_qei_set_pos_obj, machine_qei_set_pos);

STATIC mp_obj_t machine_qei_get_pos(mp_obj_t self_in){
    
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t pos=QEIPositionGet(self->qei_base);

    return mp_obj_new_int_from_uint(pos);
}
  
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_qei_get_pos_obj, machine_qei_get_pos);

// QEI(id,max_pos,*,mode=0, both_edges=true, idx_reset=false, swap=False) TODO
STATIC mp_obj_t machine_qei_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 2, 6, true);

    // find QEI Port id
    uint8_t qei_idx=qei_find(args[0]);

    // create new qei object
    machine_qei_obj_t *self;

    // get/create qei object
    if (MP_STATE_PORT(machine_qei_obj_all)[qei_idx] == NULL) {
        self = m_new_obj(machine_qei_obj_t);
        self ->base.type = &machine_qei_type;
        self -> qei_id = qei_idx;
        self -> is_enabled = false;

        MP_STATE_PORT(machine_qei_obj_all)[qei_idx] = self;
    } else {
        // reference existing qei object
        self = MP_STATE_PORT(machine_qei_obj_all)[qei_idx];
    }

    // The caller is requesting a reconfiguration of the hardware
    // this can only be done if the hardware is in init mode
    if (self->is_enabled) {
        qei_deinit(self);
    }
    
    // TODO 
    if (n_args > 0 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        qei_init_helper(self, n_args - 1, args + 1, &kw_args);
    }

    return MP_OBJ_FROM_PTR(self);
}

// prints importent information about the QEI obj
STATIC void machine_qei_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    printf("Print qei obj info\n");
    // TODO
}


STATIC const mp_rom_map_elem_t machine_qei_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get_pos),            MP_ROM_PTR(&machine_qei_get_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos),            MP_ROM_PTR(&machine_qei_set_pos_obj) },
};

STATIC MP_DEFINE_CONST_DICT(machine_qei_locals_dict, machine_qei_locals_dict_table);

const mp_obj_type_t machine_qei_type = {
    { &mp_type_type },
    .name = MP_QSTR_QEI,
    .print = machine_qei_print,
    .make_new = machine_qei_make_new,
    .locals_dict = (mp_obj_dict_t *)&machine_qei_locals_dict,
};