/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTORS */
#define PWM_MOTORS_INST                                                    TIMA0
#define PWM_MOTORS_INST_IRQHandler                              TIMA0_IRQHandler
#define PWM_MOTORS_INST_INT_IRQN                                (TIMA0_INT_IRQn)
#define PWM_MOTORS_INST_CLK_FREQ                                        32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTORS_C0_PORT                                            GPIOA
#define GPIO_PWM_MOTORS_C0_PIN                                     DL_GPIO_PIN_8
#define GPIO_PWM_MOTORS_C0_IOMUX                                 (IOMUX_PINCM19)
#define GPIO_PWM_MOTORS_C0_IOMUX_FUNC                IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_MOTORS_C0_IDX                               DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTORS_C1_PORT                                            GPIOB
#define GPIO_PWM_MOTORS_C1_PIN                                     DL_GPIO_PIN_9
#define GPIO_PWM_MOTORS_C1_IOMUX                                 (IOMUX_PINCM26)
#define GPIO_PWM_MOTORS_C1_IOMUX_FUNC                IOMUX_PINCM26_PF_TIMA0_CCP1
#define GPIO_PWM_MOTORS_C1_IDX                               DL_TIMER_CC_1_INDEX



/* Defines for UART_DEBUG */
#define UART_DEBUG_INST                                                    UART0
#define UART_DEBUG_INST_FREQUENCY                                       32000000
#define UART_DEBUG_INST_IRQHandler                              UART0_IRQHandler
#define UART_DEBUG_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_DEBUG_RX_PORT                                            GPIOA
#define GPIO_UART_DEBUG_TX_PORT                                            GPIOA
#define GPIO_UART_DEBUG_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_UART_DEBUG_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_UART_DEBUG_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_UART_DEBUG_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_UART_DEBUG_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_DEBUG_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define UART_DEBUG_BAUD_RATE                                            (115200)
#define UART_DEBUG_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_DEBUG_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group GPIO_MOTOR_A */
#define GPIO_MOTOR_A_PORT                                                (GPIOA)

/* Defines for AIN1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_MOTOR_A_AIN1_PIN                                   (DL_GPIO_PIN_25)
#define GPIO_MOTOR_A_AIN1_IOMUX                                  (IOMUX_PINCM55)
/* Defines for AIN2: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_MOTOR_A_AIN2_PIN                                   (DL_GPIO_PIN_31)
#define GPIO_MOTOR_A_AIN2_IOMUX                                   (IOMUX_PINCM6)
/* Defines for STBY: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_MOTOR_A_STBY_PIN                                   (DL_GPIO_PIN_27)
#define GPIO_MOTOR_A_STBY_IOMUX                                  (IOMUX_PINCM60)
/* Port definition for Pin Group GPIO_MOTOR_B */
#define GPIO_MOTOR_B_PORT                                                (GPIOB)

/* Defines for BIN1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_MOTOR_B_BIN1_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_MOTOR_B_BIN1_IOMUX                                  (IOMUX_PINCM33)
/* Defines for BIN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_MOTOR_B_BIN2_PIN                                   (DL_GPIO_PIN_13)
#define GPIO_MOTOR_B_BIN2_IOMUX                                  (IOMUX_PINCM30)
/* Port definition for Pin Group GPIO_STATUS_LED */
#define GPIO_STATUS_LED_PORT                                             (GPIOB)

/* Defines for PB22_LED: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_STATUS_LED_PB22_LED_PIN                            (DL_GPIO_PIN_22)
#define GPIO_STATUS_LED_PB22_LED_IOMUX                           (IOMUX_PINCM50)
/* Defines for PB26_LED: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_STATUS_LED_PB26_LED_PIN                            (DL_GPIO_PIN_26)
#define GPIO_STATUS_LED_PB26_LED_IOMUX                           (IOMUX_PINCM57)
/* Defines for PB27_LED: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_STATUS_LED_PB27_LED_PIN                            (DL_GPIO_PIN_27)
#define GPIO_STATUS_LED_PB27_LED_IOMUX                           (IOMUX_PINCM58)
/* Port definition for Pin Group GPIO_ENCODER_LEFT */
#define GPIO_ENCODER_LEFT_PORT                                           (GPIOB)

/* Defines for LEFT_C0: GPIOB.2 with pinCMx 15 on package pin 50 */
// groups represented: ["GPIO_ENCODER_RIGHT","GPIO_ENCODER_LEFT"]
// pins affected: ["RIGHT_C0","RIGHT_C1","LEFT_C0","LEFT_C1"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_LEFT_LEFT_C0_IIDX                       (DL_GPIO_IIDX_DIO2)
#define GPIO_ENCODER_LEFT_LEFT_C0_PIN                            (DL_GPIO_PIN_2)
#define GPIO_ENCODER_LEFT_LEFT_C0_IOMUX                          (IOMUX_PINCM15)
/* Defines for LEFT_C1: GPIOB.3 with pinCMx 16 on package pin 51 */
#define GPIO_ENCODER_LEFT_LEFT_C1_IIDX                       (DL_GPIO_IIDX_DIO3)
#define GPIO_ENCODER_LEFT_LEFT_C1_PIN                            (DL_GPIO_PIN_3)
#define GPIO_ENCODER_LEFT_LEFT_C1_IOMUX                          (IOMUX_PINCM16)
/* Port definition for Pin Group GPIO_ENCODER_RIGHT */
#define GPIO_ENCODER_RIGHT_PORT                                          (GPIOB)

/* Defines for RIGHT_C0: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_ENCODER_RIGHT_RIGHT_C0_IIDX                     (DL_GPIO_IIDX_DIO0)
#define GPIO_ENCODER_RIGHT_RIGHT_C0_PIN                          (DL_GPIO_PIN_0)
#define GPIO_ENCODER_RIGHT_RIGHT_C0_IOMUX                        (IOMUX_PINCM12)
/* Defines for RIGHT_C1: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_ENCODER_RIGHT_RIGHT_C1_IIDX                     (DL_GPIO_IIDX_DIO1)
#define GPIO_ENCODER_RIGHT_RIGHT_C1_PIN                          (DL_GPIO_PIN_1)
#define GPIO_ENCODER_RIGHT_RIGHT_C1_IOMUX                        (IOMUX_PINCM13)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTORS_init(void);
void SYSCFG_DL_UART_DEBUG_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
