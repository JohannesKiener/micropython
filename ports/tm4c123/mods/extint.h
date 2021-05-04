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

#ifndef MICROPY_INCLUDED_TM4C_EXTINT_H
#define MICROPY_INCLUDED_TM4C_EXTINT_H

// Vectors 0-15 are for regular pins
// Vectors 16-22 are for internal sources.
//
// Use the following constants for the internal sources:

#define EXTI_PVD_OUTPUT         (16)
#define EXTI_RTC_ALARM          (17)
#define EXTI_USB_OTG_FS_WAKEUP  (18)
#define EXTI_ETH_WAKEUP         (19)
#define EXTI_USB_OTG_HS_WAKEUP  (20)
#define EXTI_RTC_TIMESTAMP      (21)
#define EXTI_RTC_WAKEUP         (22)
#if defined(STM32F7)
#define EXTI_LPTIM1_ASYNC_EVENT (23)
#endif

#define EXTI_NUM_VECTORS        (PYB_EXTI_NUM_VECTORS)

#define EXTI_MODE_INTERRUPT     (offsetof(EXTI_TypeDef, IMR))
#define EXTI_MODE_EVENT         (offsetof(EXTI_TypeDef, EMR))

#define EXTI_TRIGGER_RISING         (offsetof(EXTI_TypeDef, RTSR))
#define EXTI_TRIGGER_FALLING        (offsetof(EXTI_TypeDef, FTSR))
#define EXTI_TRIGGER_RISING_FALLING (EXTI_TRIGGER_RISING + EXTI_TRIGGER_FALLING)  // just different from RISING or FALLING


 enum GPIO_IntHndl {
    GPIOA_IntHndl,
    GPIOB_IntHndl,
    GPIOC_IntHndl,
    GPIOD_IntHndl,
    GPIOE_IntHndl,
    GPIOF_IntHndl,
    GPIOG_IntHndl,
    GPIOH_IntHndl,
    GPIOJ_IntHndl,
};

typedef struct {
    mp_obj_base_t base;
    mp_int_t line;
    uint32_t      pin_mask;
    uint32_t      gpio;   //Address of Port
} extint_obj_t;

void extint_init0(void);

uint extint_register(extint_obj_t *self_in, mp_obj_t pin_obj, uint32_t mode, uint32_t pull, mp_obj_t callback_obj, bool override_callback_obj);
void extint_register_pin(const pin_obj_t *pin, uint32_t mode, bool hard_irq, mp_obj_t callback_obj);

 void extint_enable(const uint32_t gpio, const uint32_t pin_mask);
 void extint_disable(const uint32_t gpio, const uint32_t pin_mask); 
void extint_swint(uint line);

void Handle_EXTI_Irq(uint32_t line);

extern const mp_obj_type_t extint_type;

#endif // MICROPY_INCLUDED_TM4C_EXTINT_H
