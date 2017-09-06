#ifndef __EXTI_V1_H
#define __EXTI_V1_H
#include "stm32f0xx_exti.h"
#include "deca_regs.h"
#include "deca_device_api.h"

/* DW1000 IRQ (EXTI9_5_IRQ) handler type. */
typedef void (*port_deca_isr_t)(void);
/* DW1000 IRQ handler declaration. */
extern port_deca_isr_t port_deca_isr;
extern void Delay_ms(__IO uint32_t nTime);
void dw1000IRQ_init(void);
void EXTI0_1_IRQHandler(void);
void port_set_deca_isr(port_deca_isr_t deca_isr);

#endif
