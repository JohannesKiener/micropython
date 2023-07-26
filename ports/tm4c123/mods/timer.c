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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "py/runtime.h"
#include "py/gc.h"
#include "timer.h"
#include "pin.h"
#include "py/obj.h"
#include "py/objlist.h"

#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "py/mphal.h"
#include "handlers.h"
#include "mpirq.h"
#include "py/mperrno.h"


/// \moduleref pyb
/// \class Timer - periodically call a function
///
/// Timers can be used for a great variety of tasks.  At the moment, only
/// the simplest case is implemented: that of calling a function periodically.
///
/// Each timer consists of a counter that counts up/down at a certain rate.  The rate
/// at which it counts is the peripheral clock frequency (in Hz).  
/// When the counter reaches the timer period/ zero it triggers an
/// event, and the counter resets back to zero.  By using the callback method,
/// the timer event can call a Python function.
///
/// Example usage to toggle an LED at a fixed frequency:
///     from umachine import Timer
///     tim=Timer(Timer.TIMER0,Timer.A,freq=1)      # create Timer obj and start timer with 1 Hz
/// 
/// Add a callback:
///     def callback_fun(self):
///         print("timeout")
///         #...
///
///     tim.irq(callback=callback_fun)              # activate timeout irq + register callback 
///
/// Further examples:
///     # Configure timer with exact register values
///     tim=Timer(Timer.TIMER0,Timer.A,ticks=100,prescaler=10)
///     tim.ticks()                     # get current value of timer reg
///     tim.ticks(200)                  # set reload value of timer reg
///     tim.frequency(2)                # set freq to 2 Hz (can also get)
///     tim.prescaler(2)                # set prescaler (can also get)
///     tim.callback(None)              # clear callback
///

/******************************************************************************
 DECLARE PRIVATE CONSTANTS
 ******************************************************************************/
#define MP_TIMER_CFG_CONCATENATED                      (0x00)
//#define TIMER_CFG_RTC                               (0x01) // TODO not yet implemented
#define MP_TIMER_CFG_SPLIT                             (0x04)

// supported irq sources
#define MP_IRQ_TIMEOUT                                  (TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT)

STATIC const mp_irq_methods_t machine_timer_irq_methods;

// Array with the Base addresses of the different timers
STATIC uint32_t timer_block_bases[MICROPY_HW_MAX_TIMER] = { TIMER0_BASE,TIMER1_BASE,TIMER2_BASE,
                                                            TIMER3_BASE,TIMER4_BASE,TIMER5_BASE,
                                                            WTIMER0_BASE,WTIMER1_BASE,WTIMER2_BASE,
                                                            WTIMER3_BASE,WTIMER4_BASE,WTIMER5_BASE};

// Array with the peripheral addresses of the different timers
STATIC uint32_t timer_block_sysctl[MICROPY_HW_MAX_TIMER] = {SYSCTL_PERIPH_TIMER0,SYSCTL_PERIPH_TIMER1,SYSCTL_PERIPH_TIMER2,
                                                            SYSCTL_PERIPH_TIMER3,SYSCTL_PERIPH_TIMER4,SYSCTL_PERIPH_TIMER5,
                                                            SYSCTL_PERIPH_WTIMER0,SYSCTL_PERIPH_WTIMER1,SYSCTL_PERIPH_WTIMER2,
                                                            SYSCTL_PERIPH_WTIMER3,SYSCTL_PERIPH_WTIMER4,SYSCTL_PERIPH_WTIMER5};

// Map of the given irq handlers 
STATIC const void* timer_irq_handles[2][MICROPY_HW_MAX_TIMER] = {{  &TIMER0AIntHandler,&TIMER1AIntHandler,&TIMER2AIntHandler,
                                                                    &TIMER3AIntHandler,&TIMER4AIntHandler,&TIMER5AIntHandler,
                                                                    &TIMER6AIntHandler,&TIMER7AIntHandler,&TIMER8AIntHandler,
                                                                    &TIMER9AIntHandler,&TIMER10AIntHandler,&TIMER11AIntHandler},
                                                                 {  &TIMER0BIntHandler,&TIMER1BIntHandler,&TIMER2BIntHandler,
                                                                    &TIMER3BIntHandler,&TIMER4BIntHandler,&TIMER5BIntHandler,
                                                                    &TIMER6BIntHandler,&TIMER7BIntHandler,&TIMER8BIntHandler,
                                                                    &TIMER9BIntHandler,&TIMER10BIntHandler,&TIMER11BIntHandler}};

// returns the max number of bits in the counting reg of a given timer obj
STATIC uint8_t get_timer_bits(machine_timer_obj_t * self){
    if (!self->timer_block->is_wide && self->timer_block->is_split){
        return 16;
    } else if (self->timer_block->is_wide && !self->timer_block->is_split){
        return 64;
    } else {
        return 32;
    }
}

// returns the max number of prescaler bits of a given timer obj
STATIC uint8_t get_prescaler_bits(machine_timer_obj_t * self){
    if (self->timer_block->is_wide && self->timer_block->is_split){
        // 32 bit wide Timer -> 16 Bit  prescalers
        return 16;
    } else if (!self->timer_block->is_wide && self->timer_block->is_split){
        // 16 bit Timer -> 8 Bit  prescaler
        return 8;
    } else {
        // TIMERA|TIMERB -> no prescaler, see datasheet
        return 0;
    }
}

// Returns the bitmask from a given length (<bit>).
// This mask also represents the max value that can be represented with a given number of bits.
// So this function can be used for max value overflow checks.
STATIC uint64_t bitmask(uint16_t bit){
    return (((uint64_t)1<<bit)-1);
}

STATIC void set_prescaler(machine_timer_obj_t * self, uint16_t prescaler){
    if(prescaler > bitmask(get_prescaler_bits(self))){
        mp_raise_ValueError(MP_ERROR_TEXT("Given <prescaler> invalid"));
    }
    self->prescaler=prescaler;
}

STATIC uint64_t get_current_ticks(machine_timer_obj_t * self){
    uint64_t value;
    if(self->timer_block->is_wide && !self->timer_block->is_split){
        // wide + concatenated needs special reg call
        value=TimerValueGet64(self->timer_block->timer_base);
    }else{
        value=TimerValueGet(self->timer_block->timer_base,self->channel);
    }
    return value;
}

STATIC void set_ticks(machine_timer_obj_t * self, uint64_t ticks){
    if(ticks > bitmask(get_timer_bits(self))){
        mp_raise_ValueError(MP_ERROR_TEXT("Given <ticks> invalid"));
    }
    self->ticks=ticks;
}

STATIC float get_freq(machine_timer_obj_t * self){
    uint32_t clk=SysCtlClockGet();
    return (float)clk/((self->ticks+1)*(self->prescaler+1));
}

// checks the ranges and calculates the ticks und the prescaler from a given frequency
// be careful with other modes than oneshot/periodic down, here prescaler not low but high bits
STATIC void set_prescaler_ticks_from_freq_down(machine_timer_obj_t * self, mp_obj_t freq){
    uint32_t clk=SysCtlClockGet();
    float ticks = (float)clk/mp_obj_get_float_to_f(freq);
    uint32_t prescaler=1;
    uint64_t ticks_int;

    if(ticks<=0){
        mp_raise_ValueError(MP_ERROR_TEXT("<freq> must to be positive"));
    }else if(ticks<1){
        mp_raise_ValueError(MP_ERROR_TEXT("freq is higher than current system clock"));
    }

    // max possible ticks (64-Bit)
    if(ticks > 0xffffffffffffffff){
        mp_raise_ValueError(MP_ERROR_TEXT("Given <freq> is to small"));
    }

    ticks_int=(uint64_t)ticks;

    //try to use prescaler to avoid the overflow
    while (ticks_int > bitmask(get_timer_bits(self))) {
    // Disassembles the ticks factorially, while trying to minimize the error
        // if we can divide exactly, do that first
        if (ticks_int % 5 == 0) {
            prescaler *= 5;
            ticks_int /= 5;
        } else if (ticks_int % 3 == 0) {
            prescaler *= 3;
            ticks_int /= 3;
        } else {
            // may not divide exactly, but loses minimal precision
            uint8_t period_lsb = ticks_int & 1;
            prescaler <<= 1;
            ticks_int >>= 1;
            if (ticks_int < prescaler) {
                // round division up
                prescaler |= period_lsb;
            }
        }
        // check for prescaler overflow
        if (prescaler>bitmask(get_prescaler_bits(self))){
            //overflow error
            if(!self->timer_block->is_wide){
                printf("Hint: Consider using Wide-Timers\n");
            }
            if(self->timer_block->is_split){
                printf("Hint: Consider using TimerA and TimerB combined\n");
            }
            mp_raise_ValueError(MP_ERROR_TEXT("Given <freq> is to small"));
        }   
    }

    self->ticks=(ticks_int - 1)&bitmask(get_timer_bits(self));
    self->prescaler=(prescaler - 1)&bitmask(get_prescaler_bits(self));
}

// -------- IRQ Functions -----------------

STATIC void machine_timer_irq_enable (mp_obj_t self_in) {
    machine_timer_obj_t *self = self_in;
    TimerIntClear(self->timer_block->timer_base, self->irq_trigger);
    TimerIntEnable(self->timer_block->timer_base, self->irq_trigger);
}

STATIC void machine_timer_irq_disable (mp_obj_t self_in) {
    machine_timer_obj_t *self = self_in;
    TimerIntClear(self->timer_block->timer_base, self->irq_trigger);
    TimerIntDisable(self->timer_block->timer_base, self->irq_trigger);
}

STATIC int machine_timer_irq_flags (mp_obj_t self_in) {
    machine_timer_obj_t *self = self_in;
    return self->irq_trigger;
}

// (De)Actives irqs 
STATIC void update_timer_irq(machine_timer_obj_t *self, mp_obj_t callback, uint32_t trigger){
    // check if trigger is valid
    if(trigger!=MP_IRQ_TIMEOUT){
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid <callback>, must be a function"));
    }

    // Deactivate irq
    if(callback==MP_OBJ_NULL){
        machine_timer_irq_disable(self);
        mp_irq_remove(self);
    // setup irq
    }else if (mp_obj_is_fun(callback)){
        machine_timer_irq_disable(self);
        // save trigger source. channel mask needed cause irq regs are shared between TimerA and TimerB
        self->irq_trigger=trigger&self->channel;
        uint32_t channel_idx = self->channel==TIMER_B? 1:0;

        // register the interrupt
        TimerIntRegister(self->timer_block->timer_base, self->channel, timer_irq_handles[channel_idx][self->timer_block->id]);

        // create the irq obj and handler that gets call on irq
        mp_irq_new(self, callback, &machine_timer_irq_methods);
        
        // enable the callback before returning
        machine_timer_irq_enable(self);

    }else{
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid <callback>, must be a function"));
    }
}

// irq Handler
void TIMERGenericIntHandler(uint32_t timer_idx, uint16_t channel) {
    machine_timer_obj_t *self;
    uint32_t status;
    // find the right timer_block
    machine_timer_block_obj_t *timblock=MP_STATE_PORT(machine_timer_block_obj_all)[timer_idx];
    // find the right timer pointer
    self=channel==TIMER_B? timblock->ch_B : timblock->ch_A; // (TIMER_A|TIMER_B) can be controlled with TIMER_A regs 

    status=TimerIntStatus(timblock->timer_base,true);
    status&=self->channel; // only clear irq from the given timer
    TimerIntClear(timblock->timer_base, status);
    irq_handler(mp_irq_find(self));
}


// -------- Init/Deint-Functions ---------- 

// deinits the timerblock if no other timer within the block is still used
STATIC void timer_block_deinit(machine_timer_block_obj_t *self){
    if(self!=mp_const_none){
        if(!self->ch_A->is_enabled && !self->ch_B->is_enabled){
            SysCtlPeripheralDisable(self->peripheral);
            self->is_enabled=false;
        }
    }
}

// deinits a single timer obj. Also disables irqs
STATIC void timer_deinit(machine_timer_obj_t *self){
    // only deinit if timer is allocated
    if(self!=mp_const_none){
        TimerDisable(self->timer_block->timer_base, self->channel);
        machine_timer_irq_disable(self);
        self->is_enabled=false;
    }
}

// deinits all the timers and timer_blocks that are active
// this also disables irqs
void timer_deinit_all(){
    for (int i = 0; i < MP_ARRAY_SIZE(MP_STATE_PORT(machine_timer_block_obj_all)); i++) {
        machine_timer_block_obj_t *timer_block = MP_STATE_PORT(machine_timer_block_obj_all)[i];
        if (timer_block != NULL) {
            timer_deinit(timer_block->ch_A);
            timer_deinit(timer_block->ch_B);
            timer_block_deinit(timer_block);
        }
    }
}

// init struct, when newly allocating a timer obj
STATIC void timer_struct_init0(machine_timer_obj_t *self){
    self->base.type=&machine_timer_type;
    self->timer_block=mp_const_none;
    self->channel=0;
    self->mode=0;
    self->prescaler=0;
    self->ticks=0;
    self->irq_trigger=0;
    self->is_enabled=false;
}

// init struct, when newly allocating a timer_block obj
STATIC void timer_block_struct_init0(machine_timer_block_obj_t *self){
    self->base.type=&machine_timer_block_type;
    self->timer_base=0;
    self->peripheral=0;
    self->regs=NULL;
    self->ch_A=mp_const_none;
    self->ch_B=mp_const_none;
    self->id=0;
    self->is_wide=false;
    self->is_split=false;
    self->is_enabled=false;
}

// configures the peripheral regs and starts the timer 
STATIC void timer_configure (machine_timer_obj_t *self) {
    SysCtlPeripheralEnable(self->timer_block->peripheral);
    while(!SysCtlPeripheralReady(self->timer_block->peripheral));


    // ----Configuring the Timer Regs---- 
    // here we dont use the driver lib, because it does not support our wanted functionality 
    // of configuring the timers in a block individually
    
    // changing the CFG REG only allowed when newly configuring Timer
    if(!self->timer_block->is_enabled){
        self->timer_block->regs->CFG=self->timer_block->is_split ? MP_TIMER_CFG_SPLIT : MP_TIMER_CFG_CONCATENATED;
    }

    // clears the enable bits of the timer for reconfigurations
    self->timer_block->regs->CTL&=~((TIMER_CTL_TAEN | TIMER_CTL_TBEN) & self->channel); 
    
    // sets the mode 
    if(self->channel==TIMER_B){
        self->timer_block->regs->TBMR=self->mode;
    }else{
        //TIMER_A and concatenated Timers (TIMER_A|TIMER_B)
        self->timer_block->regs->TAMR=self->mode;
    }
    
    // --- Timing Regs 
    TimerPrescaleSet(self->timer_block->timer_base, self->channel,self->prescaler);
    if(self->timer_block->is_wide && !self->timer_block->is_split){
        TimerLoadSet64(self->timer_block->timer_base, self->ticks);
    }else{
        TimerLoadSet(self->timer_block->timer_base, self->channel,self->ticks);
    }
    self->timer_block->is_enabled=true;
    self->is_enabled=true;
    TimerEnable(self->timer_block->timer_base, self->channel);
}

// Configures a given timer obj, checks all the user inputs and inits the timer obj
STATIC mp_obj_t machine_timer_init_helper(machine_timer_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_freq , ARG_prescaler, ARG_ticks};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,         MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int  = TIMER_CFG_PERIODIC}},
        { MP_QSTR_freq,         MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        { MP_QSTR_prescaler,    MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0x0} },
        { MP_QSTR_ticks,        MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0x0} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // ---check the mode
    uint32_t mode = args[ARG_mode].u_int;
    if (mode != TIMER_CFG_ONE_SHOT && mode != TIMER_CFG_PERIODIC){
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid mode"));
    }

    // ---Timing settings
    // freq given
    if (args[ARG_freq].u_obj != MP_OBJ_NULL) {
        set_prescaler_ticks_from_freq_down(self,args[ARG_freq].u_obj);
    // exact register values are given
    }else if(args[ARG_ticks].u_int != 0){
        set_ticks(self,args[ARG_ticks].u_int);
        set_prescaler(self,args[ARG_prescaler].u_int);
    }else{
        //none or both are given
        mp_raise_ValueError(MP_ERROR_TEXT("Specify <freq> or <ticks> + <prescaler>"));
    }

    // set mode
    self->mode=mode;
    timer_configure(self);
    return mp_const_none;
}

// Inits a given timer_block obj, with a given channel.
// Also creates right timer within the timer_block and returns it.
STATIC machine_timer_obj_t * machine_timer_block_init_helper(machine_timer_block_obj_t *self, uint32_t channel) {
    // check channel
    if (channel != TIMER_A && channel !=TIMER_B && channel != (TIMER_A|TIMER_B)){
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid channel"));
    }
    bool is_split=(channel!=(TIMER_A|TIMER_B));

    // check if already configured in other combination (split or concatenated)
    if(self->is_enabled && (self->is_split != is_split)){
        vstr_t vstr[2];
        // Strings for displaying the Type of the msg objs ind the obj RAM 
        vstr_init_len(&vstr[0], 12);
        memcpy(vstr[0].buf,"concatenated",13);
        vstr_init_len(&vstr[1], 5);
        memcpy(vstr[1].buf,"split",6);
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
        MP_ERROR_TEXT("Timer(%d) is already configured as %s"), self->id,self->is_split? vstr[1].buf:vstr[0].buf));
    }
    
    // should not change if timer is already enabled
    self->is_split=is_split;
    self->is_wide=self->id>5;

    //--- create/recreate timer obj---
    machine_timer_obj_t *timer;

    // get the rigth timer from the timer_block. (TimerA|TimerB (concatenated) also use TimerA regs)
    timer=channel==TIMER_B? self->ch_B:self->ch_A;

    // init new timer obj
    if(timer==mp_const_none){
        timer = m_new_obj(machine_timer_obj_t);
        timer_struct_init0(timer);
        timer -> base.type=&machine_timer_type;
        timer -> channel=channel;
        timer -> timer_block=self;
    }

    // deinit+del old timer obj if already initialised
    if(timer->is_enabled){
        timer_deinit(timer);
    }
    
    // save reafeerence of new timer obj to timer_block obj
    if(channel==TIMER_B){
        self->ch_B=timer;
    }else{
        self->ch_A=timer;
    }

    return timer;
}

// -------------Micropython bindings---------------------

/// \method prescaler(*,prescaler)
/// @brief Gets/Sets the value of the timer prescaler register.
/// @param prescaler when provided, sets the exact prescaler value of the timer 
/// @return reload value of prescaler
STATIC mp_obj_t machine_timer_prescaler(size_t n_args, const mp_obj_t* args){
    machine_timer_obj_t *self=args[0];
    if(n_args>1){
        set_prescaler(self,mp_obj_get_int(args[1]));
        timer_configure(self);
    }
    return mp_obj_new_int(self->prescaler);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_prescaler_obj,1,2,machine_timer_prescaler);

/// \method ticks(*,ticks)
/// @brief Sets the value of the timer reload register. Gets the current value of the timer register.
/// @param ticks when provided sets the exact ticks value of the timer 
/// @return current ticks of the timer obj
/// @note Get and set do not use the same timer registers. While setting only changes the relaod value of the timer obj
/// and thus the period, getting returns the current value of the timer. 
STATIC mp_obj_t machine_timer_ticks(size_t n_args, const mp_obj_t* args){
    machine_timer_obj_t *self=args[0];
    if(n_args>1){
        set_ticks(self,mp_obj_get_int(args[1]));
        timer_configure(self);
    }
    return mp_obj_new_int_from_ull(get_current_ticks(self));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_ticks_obj,1,2,machine_timer_ticks);

/// \method frequency(*,freq)
/// @brief Gets/sets the frequency of a timer obj.
/// @param freq when provided sets the timer to the specified frequency
/// @return current freqency of the timer obj
STATIC mp_obj_t machine_timer_frequency(size_t n_args, const mp_obj_t* args){
    machine_timer_obj_t *self=args[0];
    //set
    if(n_args>1){
        if (mp_obj_is_float(args[1])||mp_obj_is_int(args[1])){
            set_prescaler_ticks_from_freq_down(self,args[1]);
            timer_configure(self);
        }
    }
    //get
    return mp_obj_new_float(get_freq(self));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_frequency_obj, 1, 2, machine_timer_frequency);

/// \method irq(*,callback=MP_OBJ_NULL,trigger=MP_IRQ_TIMEROUT)
/// \brief (de)activates the timer irq of a timer obj and attaches a callback function to it.
/// \param callback is the callback function that gets called upon an interrupt.
/// When only calling callback() a previously actived irq gets cleared.
/// \param trigger is the IRQ type that is activated
STATIC mp_obj_t machine_timer_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_callback, ARG_trigger};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_callback,     MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        { MP_QSTR_trigger,      MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_int = MP_IRQ_TIMEOUT}},
    };

    // parse args
    machine_timer_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    update_timer_irq(self,args[ARG_callback].u_obj,args[ARG_trigger].u_int);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_irq_obj, 1, machine_timer_irq);

/// @method deinit()
/// @brief Deinitialises the Timer object. Also deinitialises the whole TimerBlock, if no other timer in the 
/// block is active.
/// @return None
STATIC mp_obj_t machine_timer_deinit(mp_obj_t self){
    machine_timer_obj_t *timer=MP_OBJ_TO_PTR(self);
    machine_timer_block_obj_t *timer_block=timer->timer_block;

    // deinits the timer
    timer_deinit(timer);
    // also deinits timer_block, if no other timer in this block is active
    timer_block_deinit(timer_block);
    self=(mp_obj_t)mp_const_none;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_deinit_obj, machine_timer_deinit);


/// @classmethod @constructor(TimerBlockID,timer,*,mode=PERIODC,freq=MP_OBJ_NULL,ticks=0,prescaler=0)
/// @brief Creates a Timer object in the given TimerBlock and with the given Timer. 
/// To configure the timing only <freq> can be used or for more precise configurations <ticks> + <prescaler> can be used.
/// @param TimerBlockID is the ID of the Normal or Wide-Timerblock. See:
/// \b TIMER0, \b TIMER1, \b TIMER2, \b TIMER3, \b TIMER4, \b TIMER5
/// , \b WTIMER0, \b WTIMER1, \b WTIMER2, \b WTIMER3, \b WTIMER4, \b WTIMER5,
/// @param timer specifies which timer to use within the TimerBlock. Can be one of the following:
/// \b A , \b B , \b A|B (concatenated) 
/// @param mode specifies the timer mode: \b ONE_SHOT , \b PERIODC
/// @param freq specifies the frequency of the timer.
/// @param ticks specifies the exact reload value of the timer reg
/// @param prescaler specifies the exact prescaler of the timer reg  
/// @return Timer object
STATIC mp_obj_t machine_timer_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 2, MP_OBJ_FUN_ARGS_MAX, true);

    // create/get timer_block_obj
    uint32_t timer_idx = mp_obj_get_int(args[0]);    
    if (timer_idx < 0 || timer_idx > (MICROPY_HW_MAX_TIMER - 1)) {
        mp_raise_ValueError(MP_ERROR_TEXT("Given <id> invalid. Use Timer.TIMERX or Timer.WTIMERX"));
    }

    machine_timer_block_obj_t *timer_block_obj;

    // create timer obj, if necessary
    if(MP_STATE_PORT(machine_timer_block_obj_all)[timer_idx]==NULL){
        timer_block_obj = m_new_obj(machine_timer_block_obj_t);
        timer_block_struct_init0(timer_block_obj);
        timer_block_obj->base.type = &machine_timer_block_type;
        timer_block_obj->id = timer_idx;
        timer_block_obj->timer_base=timer_block_bases[timer_idx];
        timer_block_obj->peripheral=timer_block_sysctl[timer_idx];
        timer_block_obj->regs = (periph_timer_t *) timer_block_obj->timer_base;

        MP_STATE_PORT(machine_timer_block_obj_all)[timer_idx] = timer_block_obj;
    // reference existing timer_block obj
    } else{
        timer_block_obj = MP_STATE_PORT(machine_timer_block_obj_all)[timer_idx];
    }

    machine_timer_obj_t *self;
    uint32_t channel=mp_obj_get_int(args[1]);
    self=machine_timer_block_init_helper(timer_block_obj,channel);

    
    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    machine_timer_init_helper(self, n_args - 2, args + 2, &kw_args);

    return (mp_obj_t)self;
}

/// \method __str__()
/// \brief Return a string describing the Timer object
STATIC void machine_timer_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_timer_obj_t *self=MP_OBJ_TO_PTR(self_in);

    // strings for displaying what timer is used and if its concatenated
    vstr_t vstr[3];
    uint8_t vstr_idx;
    vstr_init_len(&vstr[0], 6);
    memcpy(vstr[0].buf,"TimerA",7);
    vstr_init_len(&vstr[1], 6);
    memcpy(vstr[1].buf,"TimerB",7);
    vstr_init_len(&vstr[2], 13);
    memcpy(vstr[2].buf,"TimerA|TimerB",14);
    if(self->channel==0x00ff){
        vstr_idx=0;
    }else if(self->channel==0xff00){
        vstr_idx=1;
    }else{
        //self->channel==0xffff
        vstr_idx=2;
    }
    printf("<Timer Object>\n");
    printf("TimerBlock ID: %u, '%s'\n",self->timer_block->id,vstr[vstr_idx].buf);
    printf("max bitlengths: timer(%u) | prescaler(%u)\n",get_timer_bits(self),get_prescaler_bits(self));
    printf("reload values: ticks("); mp_obj_print(mp_obj_new_int_from_ull(self->ticks), PRINT_STR); printf(") | prescaler(%u)",self->prescaler);
}  


STATIC const mp_rom_map_elem_t machine_timer_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_prescaler),              MP_ROM_PTR(&machine_timer_prescaler_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks),                  MP_ROM_PTR(&machine_timer_ticks_obj) },
    { MP_ROM_QSTR(MP_QSTR_frequency),              MP_ROM_PTR(&machine_timer_frequency_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq),                    MP_ROM_PTR(&machine_timer_irq_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),                 MP_ROM_PTR(&machine_timer_deinit_obj) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_A),                       MP_ROM_INT(TIMER_A) },
    { MP_ROM_QSTR(MP_QSTR_B),                       MP_ROM_INT(TIMER_B) },
    { MP_ROM_QSTR(MP_QSTR_ONE_SHOT),                MP_ROM_INT(TIMER_CFG_ONE_SHOT) },
    { MP_ROM_QSTR(MP_QSTR_PERIODIC),                MP_ROM_INT(TIMER_CFG_PERIODIC) },
    { MP_ROM_QSTR(MP_QSTR_IRQ_TIMEOUT),             MP_ROM_INT(MP_IRQ_TIMEOUT) },
    { MP_ROM_QSTR(MP_QSTR_TIMER0),                  MP_ROM_INT(0)},
    { MP_ROM_QSTR(MP_QSTR_TIMER1),                  MP_ROM_INT(1)},
    { MP_ROM_QSTR(MP_QSTR_TIMER2),                  MP_ROM_INT(2)},
    { MP_ROM_QSTR(MP_QSTR_TIMER3),                  MP_ROM_INT(3)},
    { MP_ROM_QSTR(MP_QSTR_TIMER4),                  MP_ROM_INT(4)},
    { MP_ROM_QSTR(MP_QSTR_TIMER5),                  MP_ROM_INT(5)},
    { MP_ROM_QSTR(MP_QSTR_WTIMER0),                 MP_ROM_INT(6)},
    { MP_ROM_QSTR(MP_QSTR_WTIMER1),                 MP_ROM_INT(7)},
    { MP_ROM_QSTR(MP_QSTR_WTIMER2),                 MP_ROM_INT(8)},
    { MP_ROM_QSTR(MP_QSTR_WTIMER3),                 MP_ROM_INT(9)},
    { MP_ROM_QSTR(MP_QSTR_WTIMER4),                 MP_ROM_INT(10)},
    { MP_ROM_QSTR(MP_QSTR_WTIMER5),                 MP_ROM_INT(11)},
    };
STATIC MP_DEFINE_CONST_DICT(machine_timer_locals_dict, machine_timer_locals_dict_table);

// Timer class type 
const mp_obj_type_t machine_timer_type = {
    { &mp_type_type },
    .name = MP_QSTR_Timer,                                  // Name Timer
    .print = machine_timer_print,                           // __str__()            
    .make_new = machine_timer_make_new,                     // constructor __init__()
    .locals_dict = (mp_obj_t)&machine_timer_locals_dict,    // local variables and methods
};

// has no make new-> no instance can be created. Maybe add in the future with more modes.
const mp_obj_type_t machine_timer_block_type = {
    { &mp_type_type },
    .name = MP_QSTR_TimerBlock,
};

STATIC const mp_irq_methods_t machine_timer_irq_methods = {
    .init = machine_timer_irq,
    .enable = machine_timer_irq_enable,
    .disable = machine_timer_irq_disable,
    .flags = machine_timer_irq_flags,
};


