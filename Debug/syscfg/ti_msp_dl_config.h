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



/* Defines for PWM_LEFT */
#define PWM_LEFT_INST                                                      TIMA0
#define PWM_LEFT_INST_IRQHandler                                TIMA0_IRQHandler
#define PWM_LEFT_INST_INT_IRQN                                  (TIMA0_INT_IRQn)
#define PWM_LEFT_INST_CLK_FREQ                                          32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_LEFT_C0_PORT                                              GPIOA
#define GPIO_PWM_LEFT_C0_PIN                                       DL_GPIO_PIN_8
#define GPIO_PWM_LEFT_C0_IOMUX                                   (IOMUX_PINCM19)
#define GPIO_PWM_LEFT_C0_IOMUX_FUNC                  IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_LEFT_C0_IDX                                 DL_TIMER_CC_0_INDEX

/* Defines for PWM_RIGHT */
#define PWM_RIGHT_INST                                                     TIMA1
#define PWM_RIGHT_INST_IRQHandler                               TIMA1_IRQHandler
#define PWM_RIGHT_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define PWM_RIGHT_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_RIGHT_C0_PORT                                             GPIOB
#define GPIO_PWM_RIGHT_C0_PIN                                      DL_GPIO_PIN_4
#define GPIO_PWM_RIGHT_C0_IOMUX                                  (IOMUX_PINCM17)
#define GPIO_PWM_RIGHT_C0_IOMUX_FUNC                 IOMUX_PINCM17_PF_TIMA1_CCP0
#define GPIO_PWM_RIGHT_C0_IDX                                DL_TIMER_CC_0_INDEX



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
/* Defines for UART_OPENMV */
#define UART_OPENMV_INST                                                   UART2
#define UART_OPENMV_INST_FREQUENCY                                      32000000
#define UART_OPENMV_INST_IRQHandler                             UART2_IRQHandler
#define UART_OPENMV_INST_INT_IRQN                                 UART2_INT_IRQn
#define GPIO_UART_OPENMV_RX_PORT                                           GPIOA
#define GPIO_UART_OPENMV_TX_PORT                                           GPIOA
#define GPIO_UART_OPENMV_RX_PIN                                   DL_GPIO_PIN_24
#define GPIO_UART_OPENMV_TX_PIN                                   DL_GPIO_PIN_23
#define GPIO_UART_OPENMV_IOMUX_RX                                (IOMUX_PINCM54)
#define GPIO_UART_OPENMV_IOMUX_TX                                (IOMUX_PINCM53)
#define GPIO_UART_OPENMV_IOMUX_RX_FUNC                 IOMUX_PINCM54_PF_UART2_RX
#define GPIO_UART_OPENMV_IOMUX_TX_FUNC                 IOMUX_PINCM53_PF_UART2_TX
#define UART_OPENMV_BAUD_RATE                                           (115200)
#define UART_OPENMV_IBRD_32_MHZ_115200_BAUD                                 (17)
#define UART_OPENMV_FBRD_32_MHZ_115200_BAUD                                 (23)
/* Defines for UART_JY61P */
#define UART_JY61P_INST                                                    UART1
#define UART_JY61P_INST_FREQUENCY                                       32000000
#define UART_JY61P_INST_IRQHandler                              UART1_IRQHandler
#define UART_JY61P_INST_INT_IRQN                                  UART1_INT_IRQn
#define GPIO_UART_JY61P_RX_PORT                                            GPIOB
#define GPIO_UART_JY61P_TX_PORT                                            GPIOB
#define GPIO_UART_JY61P_RX_PIN                                     DL_GPIO_PIN_7
#define GPIO_UART_JY61P_TX_PIN                                     DL_GPIO_PIN_6
#define GPIO_UART_JY61P_IOMUX_RX                                 (IOMUX_PINCM24)
#define GPIO_UART_JY61P_IOMUX_TX                                 (IOMUX_PINCM23)
#define GPIO_UART_JY61P_IOMUX_RX_FUNC                  IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_JY61P_IOMUX_TX_FUNC                  IOMUX_PINCM23_PF_UART1_TX
#define UART_JY61P_BAUD_RATE                                              (9600)
#define UART_JY61P_IBRD_32_MHZ_9600_BAUD                                   (208)
#define UART_JY61P_FBRD_32_MHZ_9600_BAUD                                    (21)




/* Defines for SPI_LCD */
#define SPI_LCD_INST                                                       SPI1
#define SPI_LCD_INST_IRQHandler                                 SPI1_IRQHandler
#define SPI_LCD_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_SPI_LCD_PICO_PORT                                            GPIOB
#define GPIO_SPI_LCD_PICO_PIN                                     DL_GPIO_PIN_8
#define GPIO_SPI_LCD_IOMUX_PICO                                 (IOMUX_PINCM25)
#define GPIO_SPI_LCD_IOMUX_PICO_FUNC                 IOMUX_PINCM25_PF_SPI1_PICO
/* GPIO configuration for SPI_LCD */
#define GPIO_SPI_LCD_SCLK_PORT                                            GPIOB
#define GPIO_SPI_LCD_SCLK_PIN                                     DL_GPIO_PIN_9
#define GPIO_SPI_LCD_IOMUX_SCLK                                 (IOMUX_PINCM26)
#define GPIO_SPI_LCD_IOMUX_SCLK_FUNC                 IOMUX_PINCM26_PF_SPI1_SCLK



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
/* Port definition for Pin Group GPIO_LCD */
#define GPIO_LCD_PORT                                                    (GPIOB)

/* Defines for PIN_RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_LCD_PIN_RES_PIN                                    (DL_GPIO_PIN_10)
#define GPIO_LCD_PIN_RES_IOMUX                                   (IOMUX_PINCM27)
/* Defines for PIN_DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_LCD_PIN_DC_PIN                                     (DL_GPIO_PIN_11)
#define GPIO_LCD_PIN_DC_IOMUX                                    (IOMUX_PINCM28)
/* Defines for PIN_CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_LCD_PIN_CS_PIN                                     (DL_GPIO_PIN_14)
#define GPIO_LCD_PIN_CS_IOMUX                                    (IOMUX_PINCM31)
/* Defines for PIN_BLK: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_LCD_PIN_BLK_PIN                                    (DL_GPIO_PIN_26)
#define GPIO_LCD_PIN_BLK_IOMUX                                   (IOMUX_PINCM57)
/* Port definition for Pin Group GPIO_W25Q64 */
#define GPIO_W25Q64_PORT                                                 (GPIOA)

/* Defines for W25_CS: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_W25Q64_W25_CS_PIN                                  (DL_GPIO_PIN_12)
#define GPIO_W25Q64_W25_CS_IOMUX                                 (IOMUX_PINCM34)
/* Defines for W25_SCLK: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_W25Q64_W25_SCLK_PIN                                (DL_GPIO_PIN_13)
#define GPIO_W25Q64_W25_SCLK_IOMUX                               (IOMUX_PINCM35)
/* Defines for W25_MOSI: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_W25Q64_W25_MOSI_PIN                                (DL_GPIO_PIN_14)
#define GPIO_W25Q64_W25_MOSI_IOMUX                               (IOMUX_PINCM36)
/* Defines for W25_MISO: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_W25Q64_W25_MISO_PIN                                (DL_GPIO_PIN_15)
#define GPIO_W25Q64_W25_MISO_IOMUX                               (IOMUX_PINCM37)
/* Port definition for Pin Group GPIO_MENU_KEYS */
#define GPIO_MENU_KEYS_PORT                                              (GPIOA)

/* Defines for KEY_UP: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_MENU_KEYS_KEY_UP_PIN                               (DL_GPIO_PIN_16)
#define GPIO_MENU_KEYS_KEY_UP_IOMUX                              (IOMUX_PINCM38)
/* Defines for KEY_DOWN: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_MENU_KEYS_KEY_DOWN_PIN                             (DL_GPIO_PIN_17)
#define GPIO_MENU_KEYS_KEY_DOWN_IOMUX                            (IOMUX_PINCM39)
/* Defines for KEY_BACK_OK: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_MENU_KEYS_KEY_BACK_OK_PIN                          (DL_GPIO_PIN_21)
#define GPIO_MENU_KEYS_KEY_BACK_OK_IOMUX                         (IOMUX_PINCM46)
/* Port definition for Pin Group GPIO_GRAY_TRACK */
#define GPIO_GRAY_TRACK_PORT                                             (GPIOA)

/* Defines for GRAY_LEFT_OUTER: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_GRAY_TRACK_GRAY_LEFT_OUTER_PIN                      (DL_GPIO_PIN_7)
#define GPIO_GRAY_TRACK_GRAY_LEFT_OUTER_IOMUX                    (IOMUX_PINCM14)
/* Defines for GRAY_LEFT_INNER: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GPIO_GRAY_TRACK_GRAY_LEFT_INNER_PIN                     (DL_GPIO_PIN_18)
#define GPIO_GRAY_TRACK_GRAY_LEFT_INNER_IOMUX                    (IOMUX_PINCM40)
/* Defines for GRAY_CENTER: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_GRAY_TRACK_GRAY_CENTER_PIN                         (DL_GPIO_PIN_26)
#define GPIO_GRAY_TRACK_GRAY_CENTER_IOMUX                        (IOMUX_PINCM59)
/* Port definition for Pin Group GPIO_GRAY_TRACK_B */
#define GPIO_GRAY_TRACK_B_PORT                                           (GPIOB)

/* Defines for GRAY_RIGHT_INNER: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_GRAY_TRACK_B_GRAY_RIGHT_INNER_PIN                  (DL_GPIO_PIN_24)
#define GPIO_GRAY_TRACK_B_GRAY_RIGHT_INNER_IOMUX                 (IOMUX_PINCM52)
/* Defines for GRAY_RIGHT_OUTER: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_GRAY_TRACK_B_GRAY_RIGHT_OUTER_PIN                  (DL_GPIO_PIN_25)
#define GPIO_GRAY_TRACK_B_GRAY_RIGHT_OUTER_IOMUX                 (IOMUX_PINCM56)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_LEFT_init(void);
void SYSCFG_DL_PWM_RIGHT_init(void);
void SYSCFG_DL_UART_DEBUG_init(void);
void SYSCFG_DL_UART_OPENMV_init(void);
void SYSCFG_DL_UART_JY61P_init(void);
void SYSCFG_DL_SPI_LCD_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
