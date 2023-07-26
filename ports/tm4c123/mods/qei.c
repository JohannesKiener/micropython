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
#include <math.h>

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

// Holds the global pointers for the 2 QEI modules that are needed in the IRQ Handler
STATIC machine_qei_obj_t *glob_qei_self[2];

uint32_t allowed_vel_divs[8]=  {QEI_VELDIV_1,QEI_VELDIV_2,QEI_VELDIV_4,QEI_VELDIV_8,
                                 QEI_VELDIV_16, QEI_VELDIV_32,QEI_VELDIV_64,QEI_VELDIV_128};

typedef struct _mp_obj_float_t {
    mp_obj_base_t base;
    mp_float_t value;
} mp_obj_float_t;

mp_obj_float_t global_float_buffer;

// own float to mp_obj_t function with no allocation, allocation in irq handler disabled
mp_obj_t qei_irq_mp_obj_new_float(mp_float_t value){
    //mp_obj_float_t *o = m_new(mp_obj_float_t, 1); // this allocation happens beforehand, see global_float_buffer 
    global_float_buffer.base.type = &mp_type_float;
    global_float_buffer.value = value;
    mp_obj_float_t * ptr=&global_float_buffer;
    return MP_OBJ_FROM_PTR(ptr);
}

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

STATIC float get_rpm(machine_qei_obj_t * self){
    // calculate rpm from vel_counter
    uint32_t vel_counter=QEIVelocityGet(self->qei_base);    // number of edges (ris+fal) in timer period
    uint32_t clk=SysCtlClockGet();                          
    uint8_t veldiv=self->vel_div>>6;
    uint8_t edges= self->both_phases? 4 : 2;
    float rpm;
    rpm=(clk*(double)(0x1<<veldiv)*vel_counter*60)/(self->timeout*self->max_pos*edges);
    return rpm;
}
    

STATIC void qei_irq_configure(machine_qei_obj_t * self, uint32_t source, mp_obj_t handler){
    //source =0->Index irq,=1-> Vel Timer irq,=2-> Direction change irq,=3-> QEI Error irq 
    if(source>3){
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid irq source. Use QEI.IRQSRCX"));
    }
    // Vel timer needs to be activated (timeout>0) for a timer irq 
    if(source==1 && self->timeout<=0){
        MP_ERROR_TEXT("Velocity capture disable, provide timeout > 0 for a Velocity timer irq");
    }
    if(!mp_obj_is_callable(handler)){
        mp_raise_TypeError(MP_ERROR_TEXT("Callback handler is not callable"));
    }
    self->call_back_fun=handler;
    QEIIntDisable(self->qei_base,0x0f); //disables all prev IRQs
    QEIIntUnregister(self->qei_base);

    if(QEI0_BASE==self->qei_base){
        QEIIntRegister(self->qei_base, &QEI0_IRQHandler);
    }else{
        QEIIntRegister(self->qei_base, &QEI1_IRQHandler);
    }
    QEIIntEnable(self->qei_base, 0x1<<source);
}

// Interrupthandler
void QEIGenericIntHandler(uint32_t base){
    machine_qei_obj_t *self=glob_qei_self[QEI0_BASE==base? 0 : 1];
    QEIIntClear(base,0x0f);
    if (self->call_back_fun != mp_const_none) {
        // no allocation allowed
        gc_lock();
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_call_function_1(self->call_back_fun,MP_OBJ_FROM_PTR(self));
            nlr_pop();
        } else {
            // Uncaught exception; leads to a restart
            mp_printf(MICROPY_ERROR_PRINTER, "uncaught exception in qei(%lu) handler\n", self->qei_id);
            mp_obj_print_exception(&mp_plat_print, MP_OBJ_FROM_PTR(nlr.ret_val));
        }
        gc_unlock();
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

        //pin PD0 is locked
        if(mp_hal_pin_needs_unlocking(pins[0])){
            mp_hal_unlock_special_pin(pins[0]);
        }

        #if defined(MICROPY_HW_QEI0_PHB)
        pins[1]=MICROPY_HW_QEI0_PHB;
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
    flags|=(self->clk_dir? QEI_CONFIG_CLOCK_DIR:QEI_CONFIG_QUADRATURE);
    flags|=(self->both_phases? QEI_CONFIG_CAPTURE_A_B:QEI_CONFIG_CAPTURE_A);
    flags|=(self->idx_reset? QEI_CONFIG_RESET_IDX:QEI_CONFIG_NO_RESET);
    flags|=(self->swap? QEI_CONFIG_SWAP:QEI_CONFIG_NO_SWAP);

    QEIConfigure(self->qei_base, flags,self->max_pos-1);

    // Filter config [2-17] (pulses) -> [0-15] (Bits)
    QEIFilterConfigure(self->qei_base,(self->filter-2)<<16);

    if(self->filter!=0){
        QEIFilterEnable(self->qei_base);
    }else{
        QEIFilterDisable(self->qei_base);
    }

    // only activate velo calculation, when timeout > 0 is given
    if(self->timeout > 0){
        QEIVelocityDisable(self->qei_base);
        QEIVelocityConfigure(self->qei_base,self->vel_div,self->timeout);
        QEIVelocityEnable(self->qei_base);
    }

    self->is_enabled=true;
    QEIEnable(self->qei_base);
}

// (max_pos,*,clk_dir=false, both_phases=flase, idx_reset=false, swap=false, filter=0, timeout=0, vel_div=QEI_VELDIV_1,)
STATIC void qei_init_helper(machine_qei_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
    enum {ARG_maxpos,ARG_clk_dir, ARG_both_phases,ARG_idx_reset,ARG_swap,ARG_filter,ARG_timeout,ARG_vel_div};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_max_pos,      MP_ARG_REQUIRED| MP_ARG_INT,     {.u_int  = 0} },
        { MP_QSTR_clk_dir,     MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },           
        { MP_QSTR_both_phases,  MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },            
        { MP_QSTR_idx_reset,    MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },
        { MP_QSTR_swap,         MP_ARG_KW_ONLY | MP_ARG_BOOL,    {.u_bool = false} },
        { MP_QSTR_filter,       MP_ARG_KW_ONLY | MP_ARG_INT,     {.u_int = 0} },
        { MP_QSTR_timeout,      MP_ARG_KW_ONLY | MP_ARG_INT,     {.u_int = 0} },              // vel measurement off
        { MP_QSTR_vel_div,      MP_ARG_KW_ONLY | MP_ARG_INT,     {.u_int = QEI_VELDIV_1}},
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    
    if (args[ARG_filter].u_int == 0 || (args[ARG_filter].u_int >=2 && args[ARG_filter].u_int <=17)){
        self->filter = args[ARG_filter].u_int;
    }else{
        mp_raise_ValueError(MP_ERROR_TEXT("Filter can only be between [2,17]"));
    }

    // converts period in us into ticks and checks the range
    uint32_t clk=SysCtlClockGet();

    uint32_t timeout_us=args[ARG_timeout].u_int;
    uint32_t timeout_ticks=(uint32_t) (timeout_us*(clk/1000000.0));  // timeout given in us
    uint32_t max_timeout_us=(uint32_t) (0xFFFFFFFF)*(1000000.0/clk);
    if(timeout_us>max_timeout_us){
        mp_printf(MICROPY_ERROR_PRINTER, "Max timeout with current clk -> %u us\n", max_timeout_us);
        mp_raise_ValueError(MP_ERROR_TEXT("timeout value is to big"));
    }

    // checks if vel_div parameter is in allowed range
    int i;
    for(i=0; args[ARG_vel_div].u_int!=allowed_vel_divs[i]; i++){
        if( i>=sizeof(allowed_vel_divs)/sizeof(allowed_vel_divs[0])){
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid vel_div. Use QEI.VELDIVX"));
        }
    }

    self->timeout = timeout_ticks;
    self->vel_div = allowed_vel_divs[i];

    self->max_pos = args[ARG_maxpos].u_bool;
    self->both_phases = args[ARG_both_phases].u_bool;
    self->clk_dir = args[ARG_clk_dir].u_bool;
    self->idx_reset=args[ARG_idx_reset].u_bool;    
    self->swap = args[ARG_swap].u_bool;

    // calling qei-Init Function 
    qei_init(self);
}

void qei_deinit(machine_qei_obj_t * self){
    self->call_back_fun=mp_const_none;
    
    QEIIntUnregister(self->qei_base);
    QEIIntDisable(self->qei_base,0x0f);
    QEIDisable(self->qei_base);
    self->is_enabled=false;
    glob_qei_self[self->qei_id]=mp_const_none;
    self=mp_const_none;
}

/* --------- Binding for Micropython ------------ */
/* ---------------------------------------------- */
/// \method qei_irq(source, handler)
/// \brief activates irqs and registers a callback handler
/// \param source can be one of the following:
///  \b INT_INDEX irq when an index puls is detected, \b INT_TIMER int when vel timer reaches 0, 
///  \b INT_DIR int when dir is changed, \b INT_ERROR int when an error is detected.
/// \param handler callable micropython function, that is executed during an interrupt
STATIC mp_obj_t machine_qei_irq(mp_obj_t self_in, mp_obj_t source, mp_obj_t handler){
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t irq_src=mp_obj_get_int(source);
    qei_irq_configure(self,irq_src,handler);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(machine_qei_irq_obj, machine_qei_irq);

/// \method get_vel
/// \brief returns current rpm value. Raises error if timeout is not configured during the creation of the qei obj. See constructor.
/// \return float rpm value
STATIC mp_obj_t machine_qei_get_vel(mp_obj_t self_in){
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (self->timeout<=0)
        mp_raise_ValueError(MP_ERROR_TEXT("Velocity capture disabled, provide timeout > 0"));

    return qei_irq_mp_obj_new_float(get_rpm(self));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_qei_get_vel_obj, machine_qei_get_vel);

/// \method set_pos()
/// @brief sets the current postion of the QEI module
/// @param pos_in new current positon
/// @return None
STATIC mp_obj_t machine_qei_set_pos(mp_obj_t self_in,mp_obj_t pos_in){
    
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t pos= mp_obj_get_int(pos_in);
    if(pos>=self->max_pos){
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
        MP_ERROR_TEXT("Position needs to be smaller than (%u)"), self->max_pos));
    }

    QEIPositionSet(self->qei_base,pos);
    return mp_const_none;
}
  
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_qei_set_pos_obj, machine_qei_set_pos);

/// \method get_pos()
/// \brief returns the current position of the QEI modulemax_pos
/// \return current position
STATIC mp_obj_t machine_qei_get_pos(mp_obj_t self_in){
    
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t pos=QEIPositionGet(self->qei_base);

    return mp_obj_new_int_from_uint(pos);
}
  
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_qei_get_pos_obj, machine_qei_get_pos);

/// \classmethod @constructor QEI(id,max_pos,*,clk_dir=false, both_phases=true, idx_reset=false, swap=false, filter=0, vel_div=QEI_VELDIV_1, timeout=0 )
/// \brief              Creates a QEI Objekt
/// \param id           [0,1]: id of the QEI module 
/// \param max_pos      maximal number of position of the qudratur encoder before reseting the position back to 0
/// \param clk_dir      if True: activates clock direction mode (different codierung of input phases A and B )
/// \param both_phases  if True: The changes of phases A and B lead to an position change. (=False) only phase A is considered.  
/// \param idx_reset    if True: an puls received on the index channel leads to an reset of the positioncounter
/// \param swap         if True: swaps the input phases A and B
/// \param filter       int [2,17]: number of waiting clk cycles before the actual value of the phases is evaluated. Used for debouncing.  
/// \param vel_div      number of position changes needed for a change of the velocity register
/// \param timeout      periode of the velocity timeout. Activates velocity messurment if > 0.
/// \return qei object
STATIC mp_obj_t machine_qei_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 2, 8, true);

    // find QEI Port id
    uint8_t qei_idx=qei_find(args[0]);

    machine_qei_obj_t *self;
    // create qei object
    if (MP_STATE_PORT(machine_qei_obj_all)[qei_idx] == NULL) {
        self = m_new_obj(machine_qei_obj_t);
        self -> base.type = &machine_qei_type;
        self -> qei_id = qei_idx;

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
    
    if (n_args > 0 || n_kw > 0) {
        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        qei_init_helper(self, n_args - 1, args + 1, &kw_args);
    }

    glob_qei_self[self->qei_id]=self;   // neede for interrupts
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_qei_deinit(mp_obj_t self_in){
    machine_qei_obj_t *self = MP_OBJ_TO_PTR(self_in);
    qei_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_qei_deinit_obj, machine_qei_deinit);

STATIC const mp_rom_map_elem_t machine_qei_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get_pos),            MP_ROM_PTR(&machine_qei_get_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos),            MP_ROM_PTR(&machine_qei_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_vel),            MP_ROM_PTR(&machine_qei_get_vel_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),             MP_ROM_PTR(&machine_qei_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq),                MP_ROM_PTR(&machine_qei_irq_obj) },

    // class constants
    // Velocity predivider
    {MP_ROM_QSTR(MP_QSTR_VELDIV1),              MP_ROM_INT(QEI_VELDIV_1)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV2),              MP_ROM_INT(QEI_VELDIV_2)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV4),              MP_ROM_INT(QEI_VELDIV_4)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV8),              MP_ROM_INT(QEI_VELDIV_8)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV16),             MP_ROM_INT(QEI_VELDIV_16)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV32),             MP_ROM_INT(QEI_VELDIV_32)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV64),             MP_ROM_INT(QEI_VELDIV_64)},
    {MP_ROM_QSTR(MP_QSTR_VELDIV128),            MP_ROM_INT(QEI_VELDIV_128)},

    // IRQ Sources
    {MP_ROM_QSTR(MP_QSTR_INT_INDEX),              MP_ROM_INT(0)},
    {MP_ROM_QSTR(MP_QSTR_INT_TIMER),              MP_ROM_INT(1)},
    {MP_ROM_QSTR(MP_QSTR_INT_DIR),                MP_ROM_INT(2)},
    {MP_ROM_QSTR(MP_QSTR_INT_ERROR),              MP_ROM_INT(3)},
};

STATIC MP_DEFINE_CONST_DICT(machine_qei_locals_dict, machine_qei_locals_dict_table);

const mp_obj_type_t machine_qei_type = {
    { &mp_type_type },
    .name = MP_QSTR_QEI,
    .make_new = machine_qei_make_new,
    .locals_dict = (mp_obj_dict_t *)&machine_qei_locals_dict,
};