;/********************************** (C) COPYRIGHT *******************************
;* File Name          : startup_ch32v00X_app_probe.s
;* Description        : Minimal startup for app-slot probe. It skips SystemInit
;*                      so we can test bootloader handoff independently of RCC.
;*******************************************************************************/

	.section  .init, "ax", @progbits
	.globl  _start
	.align  2
_start:
	.option   norvc;
    j       handle_reset
    .word   0
    .word   NMI_Handler
    .word   HardFault_Handler
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   0
    .word   SysTick_Handler
    .word   0
    .word   SW_Handler
    .word   0
    .word   WWDG_IRQHandler
    .word   PVD_IRQHandler
    .word   FLASH_IRQHandler
    .word   RCC_IRQHandler
    .word   EXTI7_0_IRQHandler
    .word   AWU_IRQHandler
    .word   DMA1_Channel1_IRQHandler
    .word   DMA1_Channel2_IRQHandler
    .word   DMA1_Channel3_IRQHandler
    .word   DMA1_Channel4_IRQHandler
    .word   DMA1_Channel5_IRQHandler
    .word   DMA1_Channel6_IRQHandler
    .word   DMA1_Channel7_IRQHandler
    .word   ADC1_IRQHandler
    .word   I2C1_EV_IRQHandler
    .word   I2C1_ER_IRQHandler
    .word   USART1_IRQHandler
	.word   SPI1_IRQHandler
	.word   TIM1_BRK_IRQHandler
    .word   TIM1_UP_IRQHandler
    .word   TIM1_TRG_COM_IRQHandler
    .word   TIM1_CC_IRQHandler
    .word   TIM2_IRQHandler
    .word   USART2_IRQHandler
    .word   OPCM_IRQHandler

	.option rvc;
	.section  .text.vector_handler, "ax", @progbits
	.weak   NMI_Handler
	.weak   HardFault_Handler
	.weak   SysTick_Handler
	.weak   SW_Handler
	.weak   WWDG_IRQHandler
	.weak   PVD_IRQHandler
	.weak   FLASH_IRQHandler
	.weak   RCC_IRQHandler
	.weak   EXTI7_0_IRQHandler
	.weak   AWU_IRQHandler
	.weak   DMA1_Channel1_IRQHandler
	.weak   DMA1_Channel2_IRQHandler
	.weak   DMA1_Channel3_IRQHandler
	.weak   DMA1_Channel4_IRQHandler
	.weak   DMA1_Channel5_IRQHandler
	.weak   DMA1_Channel6_IRQHandler
	.weak   DMA1_Channel7_IRQHandler
	.weak   ADC1_IRQHandler
	.weak   I2C1_EV_IRQHandler
	.weak   I2C1_ER_IRQHandler
	.weak   USART1_IRQHandler
	.weak   SPI1_IRQHandler
	.weak   TIM1_BRK_IRQHandler
	.weak   TIM1_UP_IRQHandler
	.weak   TIM1_TRG_COM_IRQHandler
	.weak   TIM1_CC_IRQHandler
	.weak   TIM2_IRQHandler
	.weak   USART2_IRQHandler
	.weak   OPCM_IRQHandler

NMI_Handler:
HardFault_Handler:
SysTick_Handler:
SW_Handler:
WWDG_IRQHandler:
PVD_IRQHandler:
FLASH_IRQHandler:
RCC_IRQHandler:
EXTI7_0_IRQHandler:
AWU_IRQHandler:
DMA1_Channel1_IRQHandler:
DMA1_Channel2_IRQHandler:
DMA1_Channel3_IRQHandler:
DMA1_Channel4_IRQHandler:
DMA1_Channel5_IRQHandler:
DMA1_Channel6_IRQHandler:
DMA1_Channel7_IRQHandler:
ADC1_IRQHandler:
I2C1_EV_IRQHandler:
I2C1_ER_IRQHandler:
USART1_IRQHandler:
SPI1_IRQHandler:
TIM1_BRK_IRQHandler:
TIM1_UP_IRQHandler:
TIM1_TRG_COM_IRQHandler:
TIM1_CC_IRQHandler:
TIM2_IRQHandler:
USART2_IRQHandler:
OPCM_IRQHandler:
1:
	j 1b

	.section  .text.handle_reset, "ax", @progbits
	.weak     handle_reset
	.align    1
handle_reset:
.option push
.option norelax
	la gp, __global_pointer$
.option pop
1:
	la sp, _eusrstack

/* Ultra-early probe: enable GPIOC and drive PC7 high, PC6 low before C init. */
    li t0, 0x40021018
    lw t1, 0(t0)
    li t2, 0x10
    or t1, t1, t2
    sw t1, 0(t0)

    li t0, 0x40011000
    lw t1, 0(t0)
    li t2, 0x00FFFFFF
    and t1, t1, t2
    li t2, 0x33000000
    or t1, t1, t2
    sw t1, 0(t0)

    li t0, 0x40011010
    li t1, 0x00400080
    sw t1, 0(t0)

2:
	la a0, _data_lma
	la a1, _data_vma
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:
    la a0, _sbss
    la a1, _ebss
    bgeu a0, a1, 2f
1:
    sw zero, (a0)
    addi a0, a0, 4
    bltu a0, a1, 1b
2:
    li t0, 0x1880
    csrw mstatus, t0
    li t0, 0x3
    csrw 0x804, t0
    la t0, _start
    ori t0, t0, 3
    csrw mtvec, t0
    la t0, main
    csrw mepc, t0
    mret
