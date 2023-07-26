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
#ifndef MICROPY_INCLUDED_TM4C_TIMER_H
#define MICROPY_INCLUDED_TM4C_TIMER_H

#include <stdint.h>
#include "driverlib/timer.h"
#include "inc/hw_timer.h"

#include CMSIS_HEADER

/**
 *  Timer register struct for easy access 
 *  for register description, see datasheet
*/

typedef struct {                                    /*!< TIMER0 Structure                                                      */
  __IO uint32_t  CFG;                               /*!< GPTM Configuration                                                    */
  __IO uint32_t  TAMR;                              /*!< GPTM Timer A Mode                                                     */
  __IO uint32_t  TBMR;                              /*!< GPTM Timer B Mode                                                     */
  __IO uint32_t  CTL;                               /*!< GPTM Control                                                          */
  __IO uint32_t  SYNC;                              /*!< GPTM Synchronize                                                      */
  __I  uint32_t  RESERVED;
  __IO uint32_t  IMR;                               /*!< GPTM Interrupt Mask                                                   */
  __IO uint32_t  RIS;                               /*!< GPTM Raw Interrupt Status                                             */
  __IO uint32_t  MIS;                               /*!< GPTM Masked Interrupt Status                                          */
  __O  uint32_t  ICR;                               /*!< GPTM Interrupt Clear                                                  */
  __IO uint32_t  TAILR;                             /*!< GPTM Timer A Interval Load                                            */
  __IO uint32_t  TBILR;                             /*!< GPTM Timer B Interval Load                                            */
  __IO uint32_t  TAMATCHR;                          /*!< GPTM Timer A Match                                                    */
  __IO uint32_t  TBMATCHR;                          /*!< GPTM Timer B Match                                                    */
  __IO uint32_t  TAPR;                              /*!< GPTM Timer A Prescale                                                 */
  __IO uint32_t  TBPR;                              /*!< GPTM Timer B Prescale                                                 */
  __IO uint32_t  TAPMR;                             /*!< GPTM TimerA Prescale Match                                            */
  __IO uint32_t  TBPMR;                             /*!< GPTM TimerB Prescale Match                                            */
  __IO uint32_t  TAR;                               /*!< GPTM Timer A                                                          */
  __IO uint32_t  TBR;                               /*!< GPTM Timer B                                                          */
  __IO uint32_t  TAV;                               /*!< GPTM Timer A Value                                                    */
  __IO uint32_t  TBV;                               /*!< GPTM Timer B Value                                                    */
  __IO uint32_t  RTCPD;                             /*!< GPTM RTC Predivide                                                    */
  __IO uint32_t  TAPS;                              /*!< GPTM Timer A Prescale Snapshot                                        */
  __IO uint32_t  TBPS;                              /*!< GPTM Timer B Prescale Snapshot                                        */
  __IO uint32_t  TAPV;                              /*!< GPTM Timer A Prescale Value                                           */
  __IO uint32_t  TBPV;                              /*!< GPTM Timer B Prescale Value                                           */
  __I  uint32_t  RESERVED1[981];
  __IO uint32_t  PP;                                /*!< GPTM Peripheral Properties                                            */
} periph_timer_t;

typedef struct _machine_timer_obj_t machine_timer_obj_t;
typedef struct _machine_timer_block_obj_t machine_timer_block_obj_t;

// Struck for a timer block, that contains two timers (A and B)
typedef struct _machine_timer_block_obj_t {
    mp_obj_base_t base;                   // micropython obj base
    // Timer regs
    uint32_t timer_base;                  // base Adress of the Timer Block
    uint32_t peripheral;                  // address of the peripheral, needed for sysctl
    periph_timer_t *regs;                 // struct for easier reg access
    
    // timer objs in the block
    machine_timer_obj_t *ch_A;            // pointer to timer A (also used if timers a concatenated)
    machine_timer_obj_t *ch_B;            // pointer to timer B (not used when concatenated)

    // Configs
    uint8_t id;                           // id: 0-5 normal timers, 6-11 wide timers
    bool is_wide;                         
    bool is_split;                        
    bool is_enabled;
} machine_timer_block_obj_t;

// Struck for a Timer (Can either be A or B)
typedef struct _machine_timer_obj_t {
    mp_obj_base_t base;                       // micropython obj base
    machine_timer_block_obj_t *timer_block;   // pointer to parent timer_block

    uint32_t channel;                         // mask: A=0x00ff, B=0xff00, A|B concat 0xffff
                                              // Mask used for regs that are shared by Timer A and B
    uint32_t mode;                            // PERIODIC or ONESHOT
  
    uint16_t prescaler;                       // Value of prescalere reg
    uint64_t ticks;                           // Value of timer intervall load reg
    uint16_t irq_trigger;                     // irq trigger source
    bool is_enabled;                          // timer is running
} machine_timer_obj_t;

extern const mp_obj_type_t machine_timer_type;
extern const mp_obj_type_t machine_timer_block_type;

void TIMERGenericIntHandler(uint32_t timer_idx, uint16_t channel);

/******************************************************************************
 DECLARE PUBLIC FUNCTIONS
 ******************************************************************************/
void timer_deinit_all(void);

#endif // MICROPY_INCLUDED_TM4C_TIMER_H
