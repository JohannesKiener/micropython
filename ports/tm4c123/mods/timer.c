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
/// Each timer consists of a counter that counts up at a certain rate.  The rate
/// at which it counts is the peripheral clock frequency (in Hz) divided by the
/// timer prescaler.  When the counter reaches the timer period it triggers an
/// event, and the counter resets back to zero.  By using the callback method,
/// the timer event can call a Python function.
///
/// Example usage to toggle an LED at a fixed frequency:
///
///     tim = pyb.Timer(4)              # create a timer object using timer 4
///     tim.init(freq=2)                # trigger at 2Hz
///     tim.callback(lambda t:pyb.LED(1).toggle())
///
/// Further examples:
///
///     tim = pyb.Timer(4, freq=100)    # freq in Hz
///     tim = pyb.Timer(4, prescaler=0, period=99)
///     tim.counter()                   # get counter (can also set)
///     tim.prescaler(2)                # set prescaler (can also get)
///     tim.period(199)                 # set period (can also get)
///     tim.callback(lambda t: ...)     # set callback for update interrupt (t=tim instance)
///     tim.callback(None)              # clear callback
///

// The timers can be used by multiple drivers, and need a common point for
// the interrupts to be dispatched, so they are all collected here.
//

/******************************************************************************
 DECLARE PRIVATE CONSTANTS
 ******************************************************************************/
#define MP_TIMER_CFG_CONCATENATED                      (0x00)
//#define TIMER_CFG_RTC                               (0x01) // TODO not yet implemented
#define MP_TIMER_CFG_SPLIT                             (0x04)

/******************************************************************************
 DEFINE PRIVATE TYPES
 ******************************************************************************/


/*******************************************************************************
 *
 Create Timer 
 *****************************************************************************/

STATIC uint32_t timer_block_bases[MICROPY_HW_MAX_TIMER] = { TIMER0_BASE,TIMER1_BASE,TIMER2_BASE,
                                                            TIMER3_BASE,TIMER4_BASE,TIMER5_BASE,
                                                            WTIMER0_BASE,WTIMER1_BASE,WTIMER2_BASE,
                                                            WTIMER3_BASE,WTIMER4_BASE,WTIMER5_BASE};

STATIC uint32_t timer_block_sysctl[MICROPY_HW_MAX_TIMER] = {SYSCTL_PERIPH_TIMER0,SYSCTL_PERIPH_TIMER1,SYSCTL_PERIPH_TIMER2,
                                                            SYSCTL_PERIPH_TIMER3,SYSCTL_PERIPH_TIMER4,SYSCTL_PERIPH_TIMER5,
                                                            SYSCTL_PERIPH_WTIMER0,SYSCTL_PERIPH_WTIMER1,SYSCTL_PERIPH_WTIMER2,
                                                            SYSCTL_PERIPH_WTIMER3,SYSCTL_PERIPH_WTIMER4,SYSCTL_PERIPH_WTIMER5};


typedef struct _machine_timer_block_obj_t {
    mp_obj_base_t base;
    // Timer regs
    uint32_t timer_base;
    uint32_t peripheral;
    periph_timer_t *regs;
    
    // timer objs in the block
    machine_timer_obj_t *ch_A;
    machine_timer_obj_t *ch_B;
    
    // Configs
    uint8_t id;
    bool is_wide;
    bool is_split;
    bool is_enabled;
} machine_timer_block_obj_t;

typedef struct _machine_timer_obj_t {
    mp_obj_base_t base;
    machine_timer_block_obj_t *timer_block;

    uint32_t channel; 
    uint32_t mode;

    uint16_t prescaler;
    uint64_t ticks;
    uint16_t irq_trigger;
    uint16_t irq_flags;
} machine_timer_obj_t;

// returns the right mask for the Load Reg of the referenced Timer
// can also be used as the max value of the timer load register
STATIC uint64_t get_timer_mask(machine_timer_obj_t * self){
    if (!self->timer_block->is_wide && self->timer_block->is_split){
        // 16 Bit
        return 0x000000000000ffff;
    } else if (self->timer_block->is_wide && !self->timer_block->is_split){
        // 64 Bit
        return 0xffffffffffffffff;
    } else {
        // 32 Bit
        return 0x00000000ffffffff;
    }
}

// returns the right mask for the prescaler Reg of the referenced Timer
// can also be used as the max value of the prescaler register
STATIC uint16_t get_prescaler_mask(machine_timer_obj_t * self){
    if (self->timer_block->is_wide && self->timer_block->is_split){
        // 32 bit wide Timer -> 16 Bit  prescalers
        return 0xffff;
    } else if (!self->timer_block->is_wide && self->timer_block->is_split){
        // 16 bit Timer -> 8 Bit  prescaler
        return 0x00ff;
    } else {
        // TIMERA|TIMERB -> no prescaler, see datasheet
        return 0x0000;
    }
}

// TODO becarefull with oder modes than oneshot periodic, here prescaler not low but high bits
STATIC void calculate_prescaler_from_ticks_down(machine_timer_obj_t * self, float ticks){
    uint16_t prescaler=1;
    uint64_t ticks_int;

    if(ticks<=0){
        mp_raise_ValueError(MP_ERROR_TEXT("<freq> must to be positive"));
    }else if(ticks<1){// TODO change
        mp_raise_ValueError(MP_ERROR_TEXT("freq is higher than current system clock"));
    }

    // max possible ticks (64-Bit)
    if(ticks > 0xffffffffffffffff){
        mp_raise_ValueError(MP_ERROR_TEXT("Given <freq> is to small"));
    }

    ticks_int=(uint64_t)ticks;

    //try to use prescaler to avoid the overflow
    while (ticks_int > get_timer_mask(self)) {
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
        if (prescaler>get_prescaler_mask(self)){
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

    // See if right
    self->ticks=(ticks_int - 1)&get_timer_mask(self);
    self->prescaler=(prescaler - 1)&get_prescaler_mask(self);
}

// TODO deinit care with concat timers
// void timer_init0 (void) {
//     mp_obj_list_init(&MP_STATE_PORT(mp_timer_channel_obj_list), 0);
// }

// configures the peripheral regs 
STATIC void timer_init (machine_timer_obj_t *self) {
    SysCtlPeripheralEnable(self->timer_block->peripheral);
    //SysCtlPeripheralReset(tim->peripheral);           // TODO maybe change
    while(!SysCtlPeripheralReady(self->timer_block->peripheral));    // TODO maybe write before
    //TimerDisable(tim->timer,TIMER_BOTH);              // TODO change propably

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
    TimerEnable(self->timer_block->timer_base, self->channel);
}

// TODO check second init on different channel if same mode
// TODO maybe add upcounting timers
STATIC mp_obj_t machine_timer_init_helper(machine_timer_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_mode, ARG_freq , ARG_prescaler, ARG_ticks, ARG_callback};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode,         MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int  = TIMER_CFG_PERIODIC}},    // TODO maybe change
        { MP_QSTR_freq,         MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        { MP_QSTR_prescaler,    MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0x0} },
        { MP_QSTR_ticks,        MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0x0} },
        { MP_QSTR_callback,     MP_ARG_KW_ONLY  | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} }
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // check the mode
    uint32_t mode = args[ARG_mode].u_int;
    if (mode != TIMER_CFG_ONE_SHOT && mode != TIMER_CFG_PERIODIC){
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid mode"));
    }

    // Timing settings
    uint32_t clk=SysCtlClockGet();
    // freq given
    if (args[ARG_freq].u_obj != MP_OBJ_NULL) {
        float freq = mp_obj_get_float_to_f(args[ARG_freq].u_obj);
        calculate_prescaler_from_ticks_down(self,(float)clk/freq);

    // exact register values are given
    }else if(args[ARG_ticks].u_int != 0){
        uint64_t ticks=args[ARG_ticks].u_int;
        uint16_t prescaler=args[ARG_prescaler].u_int;
        if(prescaler > get_prescaler_mask(self)){
            mp_raise_ValueError(MP_ERROR_TEXT("Given <prescaler> invalid"));
        }
        if(ticks > get_timer_mask(self)){
            mp_raise_ValueError(MP_ERROR_TEXT("Given <ticks> invalid"));
        }
        self->ticks=ticks;
        self->prescaler=prescaler;
    }else{
        //none or both are given
        mp_raise_ValueError(MP_ERROR_TEXT("Specify <freq> or <ticks> + <prescaler>"));
    }

    // TODO add more savings here
    self->mode=mode;
    printf("Ticks:%lu %lu, pres:%u \n",(uint32_t)(self->ticks>>32),(uint32_t)self->ticks,self->prescaler);
    timer_init(self);
    return mp_const_none;
}


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
        // TODO see if easier
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError,
        MP_ERROR_TEXT("Timer(%d) is already configured as %s"), self->id,self->is_split? vstr[1].buf:vstr[0].buf));
    }
 
    self->is_split=is_split;
    self->is_wide=self->id>5;

    // get or create timer_obj from timer_block_obj
    if(channel==TIMER_B){
        if(self->ch_B==NULL){
            self->ch_B = m_new_obj(machine_timer_obj_t);
            self->ch_B -> base.type=&machine_timer_type;
            self->ch_B -> channel=channel;
            self->ch_B -> timer_block=self;
        }
        return self->ch_B;
    }else{
        // TIMER_A and TIMER_A|TIMER_B (concatanated)
        if(self->ch_A==NULL){
            self->ch_A = m_new_obj(machine_timer_obj_t);
            self->ch_A -> base.type=&machine_timer_type;
            self->ch_A -> channel=channel;
            self->ch_A -> timer_block=self;
        }
        return self->ch_A;
    }
}

// TODO add spilt or normal init
STATIC mp_obj_t machine_timer_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 2, MP_OBJ_FUN_ARGS_MAX, true);

    // create/get timer_block_obj
    uint32_t timer_idx = mp_obj_get_int(args[0]);    
    if (timer_idx < 0 || timer_idx > (MICROPY_HW_MAX_TIMER - 1)) {
        mp_raise_OSError(MP_ENODEV);    // TODO change to better error
    }

    machine_timer_block_obj_t *timer_block_obj;

    // create timer obj
    if (MP_STATE_PORT(machine_timer_block_obj_all)[timer_idx] == NULL) {
        timer_block_obj = m_new_obj(machine_timer_block_obj_t);
        timer_block_obj->base.type = &machine_timer_block_type;
        timer_block_obj->id = timer_idx;
        timer_block_obj->timer_base=timer_block_bases[timer_idx];
        timer_block_obj->peripheral=timer_block_sysctl[timer_idx];
        timer_block_obj->regs = (periph_timer_t *) timer_block_obj->timer_base;

        MP_STATE_PORT(machine_timer_block_obj_all)[timer_idx] = timer_block_obj;
    } else {
        // reference existing timer object
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


// STATIC mp_obj_t machine_timer_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
//     return machine_timer_init_helper(args[0], n_args - 1, args + 1, kw_args);
// }
// STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_init_obj, 1, machine_timer_init);

/******************************************************************************
 DEFINE PRIVATE DATA
 ******************************************************************************/

/******************************************************************************/

/******************************************************************************
 DEFINE PRIVATE FUNCTIONS
 ******************************************************************************/

// STATIC void machine_timer_channel_irq_enable (mp_obj_t self_in) {
//     machine_timer_channel_obj_t *self = self_in;
//     MAP_TimerIntClear(self->timer->timer, self->timer->irq_trigger & self->channel);
//     MAP_TimerIntEnable(self->timer->timer, self->timer->irq_trigger & self->channel);
// }

// STATIC void machine_timer_channel_irq_disable (mp_obj_t self_in) {
//     machine_timer_channel_obj_t *self = self_in;
//     MAP_TimerIntDisable(self->timer->timer, self->timer->irq_trigger & self->channel);
// }

// STATIC int machine_timer_channel_irq_flags (mp_obj_t self_in) {
//     machine_timer_channel_obj_t *self = self_in;
//     return self->timer->irq_flags;
// }

// STATIC machine_timer_channel_obj_t *machine_timer_channel_find (uint32_t timer, uint16_t channel_n) {
//     for (mp_uint_t i = 0; i < MP_STATE_PORT(mp_timer_channel_obj_list).len; i++) {
//         machine_timer_channel_obj_t *ch = ((machine_timer_channel_obj_t *)(MP_STATE_PORT(mp_timer_channel_obj_list).items[i]));
//         // any 32-bit timer must be matched by any of its 16-bit versions
//         if (ch->timer->timer == timer && ((ch->channel & TIMER_A) == channel_n || (ch->channel & TIMER_B) == channel_n)) {
//             return ch;
//         }
//     }
//     return MP_OBJ_NULL;
// }

// STATIC mp_obj_t machine_timer_channel_irq(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
//     mp_arg_val_t args[mp_irq_INIT_NUM_ARGS];
//     mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, mp_irq_INIT_NUM_ARGS, mp_irq_init_args, args);
//     machine_timer_channel_obj_t *ch = pos_args[0];

//     // convert the priority to the correct value
//     uint priority = args[1].u_int;

//     // // validate the power mode
//     // int pwrmode = (args[3].u_obj == mp_const_none) ? PYB_PWR_MODE_ACTIVE : mp_obj_get_int(args[3].u_obj);
//     // if (pwrmode != PYB_PWR_MODE_ACTIVE) {
//     //     goto invalid_args;
//     // }
//     // PWM

//     // get the trigger
//     uint trigger = mp_obj_get_int(args[0].u_obj);

//     // disable the callback first
//     machine_timer_channel_irq_disable(ch);

//     uint8_t shift = (ch->channel == TIMER_B) ? 8 : 0;
//     uint32_t _config = (ch->channel == TIMER_B) ? ((ch->timer->config & TIMER_B) >> 8) : (ch->timer->config & TIMER_A);
//     switch (_config) {
//     case TIMER_CFG_A_ONE_SHOT_UP:
//     case TIMER_CFG_A_PERIODIC_UP:
//         ch->timer->irq_trigger |= TIMER_TIMA_TIMEOUT << shift;
//         if (trigger != PYBTIMER_TIMEOUT_TRIGGER) {
//             goto invalid_args;
//         }
//         break;
//     case TIMER_CFG_A_PWM:
//         // special case for the PWM match interrupt
//         ch->timer->irq_trigger |= ((ch->channel & TIMER_A) == TIMER_A) ? TIMER_TIMA_MATCH : TIMER_TIMB_MATCH;
//         if (trigger != PYBTIMER_MATCH_TRIGGER) {
//             goto invalid_args;
//         }
//         break;
//     default:
//         break;
//     }
//     // special case for a 32-bit timer
//     if (ch->channel == (TIMER_A | TIMER_B)) {
//        ch->timer->irq_trigger |= (ch->timer->irq_trigger << 8);
//     }

//     void (*pfnHandler)(void);
//     uint32_t intregister;
//     switch (ch->timer->timer) {
//     case TIMER0_BASE:
//         if (ch->channel == TIMER_B) {
//             pfnHandler = &TIMER0BIntHandler;
//             intregister = INT_TIMER0B;
//         } else {
//             pfnHandler = &TIMER0AIntHandler;
//             intregister = INT_TIMER0A;
//         }
//         break;
//     case TIMER1_BASE:
//         if (ch->channel == TIMER_B) {
//             pfnHandler = &TIMER1BIntHandler;
//             intregister = INT_TIMER1B;
//         } else {
//             pfnHandler = &TIMER1AIntHandler;
//             intregister = INT_TIMER1A;
//         }
//         break;
//     case TIMER2_BASE:
//         if (ch->channel == TIMER_B) {
//             pfnHandler = &TIMER2BIntHandler;
//             intregister = INT_TIMER2B;
//         } else {
//             pfnHandler = &TIMER2AIntHandler;
//             intregister = INT_TIMER2A;
//         }
//         break;
//     case TIMER3_BASE:
//     if (ch->channel == TIMER_B) {
//         pfnHandler = &TIMER3BIntHandler;
//         intregister = INT_TIMER3B;
//     } else {
//         pfnHandler = &TIMER3AIntHandler;
//         intregister = INT_TIMER2A;
//     }
//     break;
//     case TIMER4_BASE:
//     if (ch->channel == TIMER_B) {
//         pfnHandler = &TIMER4BIntHandler;
//         intregister = INT_TIMER4B;
//     } else {
//         pfnHandler = &TIMER4AIntHandler;
//         intregister = INT_TIMER4A;
//     }
//     break;
//     default:
//         if (ch->channel == TIMER_B) {
//             pfnHandler = &TIMER5BIntHandler;
//             intregister = INT_TIMER5B;
//         } else {
//             pfnHandler = &TIMER3AIntHandler;
//             intregister = INT_TIMER3A;
//         }
//         break;
//     }

//     // register the interrupt and configure the priority
//     IntPrioritySet(intregister, priority);
//     TimerIntRegister(ch->timer->timer, ch->channel, pfnHandler);

//     // create the callback
//     mp_obj_t _irq = mp_irq_new (ch, args[2].u_obj, &machine_timer_channel_irq_methods);

//     // enable the callback before returning
//     machine_timer_channel_irq_enable(ch);

//     return _irq;

// invalid_args:
//     mp_raise_ValueError(MP_ERROR_TEXT("invalid argument(s) value"));
// }
// STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_channel_irq_obj, 1, machine_timer_channel_irq);

// STATIC void machine_timer_channel_remove (machine_timer_channel_obj_t *ch) {
//     machine_timer_channel_obj_t *channel;
//     if ((channel = machine_timer_channel_find(ch->timer->timer, ch->channel))) {
//         mp_obj_list_remove(&MP_STATE_PORT(mp_timer_channel_obj_list), channel);
//         // unregister it with the sleep module
//     }
// }

// STATIC void machine_timer_channel_add (machine_timer_channel_obj_t *ch) {
//     // remove it in case it already exists
//     machine_timer_channel_remove(ch);
//     mp_obj_list_append(&MP_STATE_PORT(mp_timer_channel_obj_list), ch);
//     // register it with the sleep module
// }

// // computes prescaler period and match value so timer triggers at freq-Hz
// STATIC uint32_t compute_prescaler_period_and_match_value(machine_timer_channel_obj_t *ch, uint32_t *period_out, uint32_t *match_out) {
//     uint32_t maxcount = (ch->channel == (TIMER_A | TIMER_B)) ? 0xFFFFFFFF : 0xFFFF;
//     uint32_t prescaler;
//     uint32_t sysclk = SysCtlClockGet();
//     uint32_t period_c = (ch->frequency > 0) ? sysclk / ch->frequency : ((sysclk) * ch->period);

//     period_c = MAX(1, period_c) - 1;
//     if (period_c == 0) {
//         goto error;
//     }
//     // //only 16 Bit now considered 
//     // TODO add 32 bit check maxcount == 0xFFFFFFF + ? (see datasheet)
    
//     prescaler = period_c >> 16;
//     *period_out = period_c;
//     if (prescaler > 0xFF && maxcount == 0xFFFF) {
//         goto error;
//     }
//     // check limit values for the duty cycle
//     if (ch->duty_cycle == 0) {
//         *match_out = period_c - 1;
//     } else {
//         if (period_c > 0xFFFF) {
//             uint32_t match = (period_c * 100) / 10000;
//             *match_out = period_c - ((match * ch->duty_cycle) / 100);

//         } else {
//             *match_out = period_c - ((period_c * ch->duty_cycle) / 10000);
//         }
//     }
//     return prescaler;

// error:
//     mp_raise_ValueError(MP_ERROR_TEXT("invalid argument(s) value"));
// }


// STATIC void m_timer_channel_init (machine_timer_channel_obj_t *ch) {
//     // calculate the period, the prescaler and the match value
//     uint32_t period_c;
//     uint32_t match;
//     uint32_t prescaler = compute_prescaler_period_and_match_value(ch, &period_c, &match);
//     // period_c = 0xFFFF;
    
//     // #### Debug stuff to get right source und period 
//     // uint32_t clksrc;
//     // uint32_t clk;
//     // clksrc = TimerClockSourceGet(ch->timer->timer);
//     // clk = SysCtlClockGet();
//     // if (clk != 0xF42400){
//     //     mp_hal_stdout_tx_str("Wrong Clock!\r\n"); 
//     // }


//     TimerClockSourceSet(ch->timer->timer,TIMER_CLOCK_SYSTEM);
    
//     // set the prescaler
//     TimerPrescaleSet(ch->timer->timer, ch->channel, (prescaler < 0xFF) ? prescaler : 0);
    

//     // set the load value
//     TimerLoadSet(ch->timer->timer, ch->channel, period_c);


//     // configure the pwm if we are in such mode
//     if ((ch->timer->config & 0x0F) == TIMER_CFG_A_PWM) {
//         // invert the timer output if required
//         TimerControlLevel(ch->timer->timer, ch->channel, (ch->polarity == PYBTIMER_POLARITY_NEG) ? true : false);
//         // set the match value (which is simply the duty cycle translated to ticks)
//         TimerMatchSet(ch->timer->timer, ch->channel, match);
//         TimerPrescaleMatchSet(ch->timer->timer, ch->channel, match >> 16);
//     }

// #ifdef DEBUG
//     // stall the timer when the processor is halted while debugging
//     MAP_TimerControlStall(ch->timer->timer, ch->channel, true);
// #endif

//     // now enable the timer channel
//     MAP_TimerEnable(ch->timer->timer, ch->channel);
// }

// STATIC void timer_disable (machine_timer_obj_t *tim) {
//     // disable all timers and it's interrupts
//     MAP_TimerDisable(tim->timer, TIMER_A | TIMER_B);
//     MAP_TimerIntDisable(tim->timer, tim->irq_trigger);
//     MAP_TimerIntClear(tim->timer, tim->irq_trigger);
//     // machine_timer_channel_obj_t *ch;
//     // disable its channels
//     // if ((ch = machine_timer_channel_find (tim->timer, TIMER_A))) {
//     //     pyb_sleep_remove(ch);
//     // }
//     // if ((ch = machine_timer_channel_find (tim->timer, TIMER_B))) {
//     //     pyb_sleep_remove(ch);
//     // }
//     SysCtlPeripheralDisable(tim->peripheral);
// }

// STATIC mp_obj_t machine_timer_channel(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
//     static const mp_arg_t allowed_args[] = {
//         { MP_QSTR_freq,                MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0} },
//         { MP_QSTR_period,              MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0} },
//         { MP_QSTR_polarity,            MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = PYBTIMER_POLARITY_POS} },
//         { MP_QSTR_duty_cycle,          MP_ARG_KW_ONLY  | MP_ARG_INT, {.u_int = 0} },
//     };

//     machine_timer_obj_t *tim = pos_args[0];
//     mp_int_t channel_n = mp_obj_get_int(pos_args[1]);

//     // throw an exception if both frequency and period are given
//     if (args[0].u_int != 0 && args[1].u_int != 0) {
//         goto error;
//     }
//     // TODO Periods bigger than 1 needed
//     // check that at least one of them has a valid value
//     if (args[0].u_int <= 0 && args[1].u_int <= 0) {
//         goto error;
//     }
//     // TODO why only TimerA, see datasheet maybe because PWM only soported on 32 Bit, both
//     // check that the polarity is not 'both' in pwm mode
//     if ((tim->config & TIMER_A) == TIMER_CFG_A_PWM && args[2].u_int == (PYBTIMER_POLARITY_POS | PYBTIMER_POLARITY_NEG)) {
//         goto error;
//     }

//     // allocate a new timer channel
//     machine_timer_channel_obj_t *ch = m_new_obj(machine_timer_channel_obj_t);
//     ch->base.type = &machine_timer_channel_type;
//     ch->timer = tim;
//     ch->channel = channel_n;

//     // get the frequency the polarity and the duty cycle
//     ch->frequency = args[0].u_int;
//     ch->period = args[1].u_int;
//     ch->polarity = args[2].u_int;
//     ch->duty_cycle = MIN(10000, MAX(0, args[3].u_int));

//     m_timer_channel_init(ch);

//     // assign the pin
//     if ((ch->timer->config & 0x0F) == TIMER_CFG_A_PWM) {
//         uint32_t ch_idx = (ch->channel == TIMER_A) ? 0 : 1;
//         // use the default pin if available
//         mp_obj_t pin_o = (mp_obj_t)machine_timer_pwm_pin[(ch->timer->id * 2) + ch_idx];

//         if (pin_o != MP_OBJ_NULL) {
//             const pin_obj_t *pin = pin_find(pin_o);
//             mp_hal_pin_config_alt(pin, PIN_FN_TIM, ch->timer->id);
           
//         }
//     }
//     // // add the timer to the list
//     machine_timer_channel_add(ch);

//     return ch;

// error:
//     mp_raise_ValueError(MP_ERROR_TEXT("invalid argument(s) value"));
// }
// STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_timer_channel_obj, 2, machine_timer_channel);

// STATIC mp_obj_t machine_timer_deinit(mp_obj_t self_in) {
//     machine_timer_obj_t *self = self_in;
//     timer_disable(self);
//     return mp_const_none;
// }

// STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_timer_deinit_obj, machine_timer_deinit);

// STATIC mp_obj_t machine_timer_channel_freq(size_t n_args, const mp_obj_t *args) {
//     machine_timer_channel_obj_t *ch = args[0];
//     if (n_args == 1) {
//         // get
//         return mp_obj_new_int(ch->frequency);
//     } else {
//         // set
//         int32_t _frequency = mp_obj_get_int(args[1]);
//         if (_frequency <= 0) {
//             mp_raise_ValueError(MP_ERROR_TEXT("invalid argument(s) value"));    //TODO see if works or maybe also add period
//         }
//         ch->frequency = _frequency;
//         ch->period = 1 / _frequency;
//         m_timer_channel_init(ch);
//         return mp_const_none;
//     }
// }
// STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_timer_channel_freq_obj, 1, 2, machine_timer_channel_freq);

// void TIMERGenericIntHandler(uint32_t timer, uint16_t channel) {
//     machine_timer_channel_obj_t *self;
//     uint32_t status;
//     if ((self = machine_timer_channel_find(timer, channel))) {
//         status = MAP_TimerIntStatus(self->timer->timer, true) & self->channel;
//         MAP_TimerIntClear(self->timer->timer, status);
//         irq_handler(mp_irq_find(self));
//     }
// }

STATIC const mp_rom_map_elem_t machine_timer_locals_dict_table[] = {
    // { MP_ROM_QSTR(MP_QSTR_init),                    MP_ROM_PTR(&machine_timer_init_obj) },
    // { MP_ROM_QSTR(MP_QSTR_deinit),                  MP_ROM_PTR(&machine_timer_deinit_obj) },
    // { MP_ROM_QSTR(MP_QSTR_channel),                 MP_ROM_PTR(&machine_timer_channel_obj) },

    // class constants
    { MP_ROM_QSTR(MP_QSTR_A),                       MP_ROM_INT(TIMER_A) },
    { MP_ROM_QSTR(MP_QSTR_B),                       MP_ROM_INT(TIMER_B) },
    { MP_ROM_QSTR(MP_QSTR_ONE_SHOT),                MP_ROM_INT(TIMER_CFG_ONE_SHOT) },
    { MP_ROM_QSTR(MP_QSTR_PERIODIC),                MP_ROM_INT(TIMER_CFG_PERIODIC) },
    };
STATIC MP_DEFINE_CONST_DICT(machine_timer_locals_dict, machine_timer_locals_dict_table);


const mp_obj_type_t machine_timer_type = {
    { &mp_type_type },
    .name = MP_QSTR_Timer,
    // .print = machine_timer_print,
    .make_new = machine_timer_make_new,
    .locals_dict = (mp_obj_t)&machine_timer_locals_dict,
};

// TODO see if needed
const mp_obj_type_t machine_timer_block_type = {
    { &mp_type_type },
    .name = MP_QSTR_TimerBlock,
    //.print = machine_timer_print,
    //.make_new = machine_timer_block_make_new,
    //.locals_dict = (mp_obj_t)&machine_timer_block_locals_dict,
};

// STATIC const mp_irq_methods_t machine_timer_channel_irq_methods = {
//     .init = machine_timer_channel_irq,
//     .enable = machine_timer_channel_irq_enable,
//     .disable = machine_timer_channel_irq_disable,
//     .flags = machine_timer_channel_irq_flags,
// };


