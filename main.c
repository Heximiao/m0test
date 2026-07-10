/*
 * MSPM0 car controller entry point.
 *
 * The application is split like the EasyPidKit reference project:
 * hardware drivers own pins/peripherals, middle modules own reusable control
 * logic, and app_car_control owns the vehicle behavior.
 */

#include "app/app_car_control.h"
#include "app/app_image_store.h"
#include "app/app_menu.h"
#include "app/app_attitude.h"
#include "bsp/bsp_tb6612.h"
#include "bsp/jy61/bsp_jy61.h"
#include "hw/hw_encoder.h"
#include "hw/hw_lcd.h"
#include "hw/hw_openmv_uart.h"
#include "hw/hw_uart.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define CONTROL_PERIOD_MS (20U)
#define ATTITUDE_PERIOD_MS (20U)
#define TELEMETRY_PERIOD_MS (100U)
#define ODOMETRY_PERIOD_MS (500U)
#define HEARTBEAT_PERIOD_MS (30000U)
#define STATUS_LED_PERIOD_MS (500U)
#define MENU_PERIOD_MS (50U)

static volatile bool gControlUpdatePending;
static volatile bool gAttitudeUpdatePending;
static volatile bool gTelemetryUpdatePending;
static volatile bool gOdometryUpdatePending;
static volatile bool gHeartbeatUpdatePending;
static volatile bool gStatusLedUpdatePending;
static volatile bool gMenuUpdatePending;
static volatile uint32_t gSysTickMs;

int main(void)
{
    SYSCFG_DL_init();

    /*
     * Current wiring map.
     *
     * Wheel naming:
     *   Left/right are viewed from behind the car, looking toward the front.
     *
     * TB6612 motor driver:
     *   PA8  -> PWMD, left wheel PWM  (TIMA0_CCP0)
     *   PA25 -> DIN1, left wheel direction
     *   PA31 -> DIN2, left wheel direction
     *   PB4  -> PWMA, right wheel PWM (TIMA1_CCP0)
     *   PB16 -> AIN1, right wheel direction
     *   PB13 -> AIN2, right wheel direction
     *   PA27 -> STBY, TB6612 standby enable
     *
     * Encoder GPIO wiring:
     *   E4A -> PB2, left wheel encoder phase A, reported as telemetry L/LD
     *   E4B -> PB3, left wheel encoder phase B, reported as telemetry L/LD
     *   E1A -> PB0, right wheel encoder phase A, reported as telemetry R/RD
     *   E1B -> PB1, right wheel encoder phase B, reported as telemetry R/RD
     *
     * Encoder notes:
     *   Encoder polarity is set in hw/hw_encoder.c so that forward wheel
     *   rotation gives positive counts. Encoder GPIOs use rising-edge
     *   interrupts like EasyPidKit; the speed loop reads the accumulated
     *   counts every control period.
     *
     * UART debug/tuning port:
     *   PA10 -> UART0 TX, MCU sends telemetry to PC
     *   PA11 -> UART0 RX, MCU receives tuning commands from PC
     *   Baud rate: 115200, 8N1
     *
     * OpenMV / Raspberry Pi UART2:
     *   PA23 -> UART2 TX, MCU sends odometry to Raspberry Pi/OpenMV RX
     *   PA24 -> UART2 RX, MCU receives LINE/LTURN from Raspberry Pi/OpenMV TX
     *   Baud rate: 115200, 8N1
     *   OpenMV P4 TX -> PA24 RX
     *   OpenMV P5 RX <- PA23 TX
     *   树莓派 GPIO14 TXD -> PA24 RX
     *   树莓派 GPIO15 RXD <- PA23 TX
     *
     * LaunchPad status LED:
     *   PB22 -> status LED
     *
     * LCD:
     *   PB8  -> LCD MOSI
     *   PB9  -> LCD SCLK
     *   PB10 -> LCD RES
     *   PB11 -> LCD DC
     *   PB14 -> LCD CS
     *   PB26 -> LCD BLK/backlight
     *
     * JY61P attitude UART:
     *   PB6 -> UART1 TX, connect to JY61P RX
     *   PB7 -> UART1 RX, connect to JY61P TX
     *   Baud rate: 9600, 8N1
     *   Attitude telemetry: ATT PITCH/ROLL/YAW, degrees * 100
     *
     * W25Q64 software SPI:
     *   PA12 -> CS
     *   PA13 -> SCLK
     *   PA14 -> MOSI   DI
     *   PA15 -> MISO   DO
     *
     * LCD menu keys, active low with internal pull-up:
     *   PA16 -> menu up, key connects PA16 to GND when pressed
     *   PA17 -> menu down, key connects PA17 to GND when pressed
     *   PA21 -> short press back, long press confirm
     */
    encoder_init();
    uart_debug_init();
    uart_openmv_init();
    app_image_store_init();
    TB6612_Init();
    lcd_init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(20, 24, (const unsigned char *) "MSPM0 CAR", WHITE, BLACK,
        24, 0);
    LCD_ShowString(20, 58, (const unsigned char *) "LCD OK", GREEN, BLACK, 24,
        0);
    LCD_ShowString(20, 92, (const unsigned char *) "PWM R: PB4", YELLOW,
        BLACK, 16, 0);
    LCD_BLK_Set();
    app_car_control_init();
    app_menu_init(gSysTickMs);

    app_attitude_init();

    SysTick_Config(CPUCLK_FREQ / 1000U);

    while (1) {
        char command[UART_DEBUG_LINE_BUFFER_SIZE];

        if (app_image_store_is_receiving()) {
            app_image_store_service();
            lcd_service();
            uart_debug_service_tx();
            continue;
        }

        while (uart_debug_read_line(command, sizeof(command))) {
            if (app_image_store_process_command(command)) {
            } else if (!app_attitude_process_command(command)) {
                app_car_control_process_command(command);
            }
        }

        while (uart_openmv_read_line(command, sizeof(command))) {
            app_car_control_process_command(command);
        }

        app_attitude_poll();
        uart_debug_service_tx();

        if (gControlUpdatePending) {
            gControlUpdatePending = false;
            app_car_control_update(gSysTickMs);
        }

        if (gAttitudeUpdatePending) {
            gAttitudeUpdatePending = false;
            app_attitude_send(gSysTickMs);
        }

        if (gTelemetryUpdatePending) {
            gTelemetryUpdatePending = false;
            app_car_control_send_telemetry();
        }

        if (gOdometryUpdatePending) {
            gOdometryUpdatePending = false;
            app_car_control_send_odometry();
        }

        if (gHeartbeatUpdatePending) {
            gHeartbeatUpdatePending = false;
            app_car_control_send_heartbeat(gSysTickMs);
        }

        if (gStatusLedUpdatePending) {
            gStatusLedUpdatePending = false;
            app_car_control_toggle_status_led();
        }

        if (gMenuUpdatePending) {
            gMenuUpdatePending = false;
            app_menu_update(gSysTickMs);
        }

        uart_debug_service_tx();
        lcd_service();
        __WFI();
    }
}

void SysTick_Handler(void)
{
    gSysTickMs++;
    if ((gSysTickMs % CONTROL_PERIOD_MS) == 0U) {
        gControlUpdatePending = true;
    }
    if ((gSysTickMs % ATTITUDE_PERIOD_MS) == 0U) {
        gAttitudeUpdatePending = true;
    }
    if ((gSysTickMs % TELEMETRY_PERIOD_MS) == 0U) {
        gTelemetryUpdatePending = true;
    }
    if ((gSysTickMs % ODOMETRY_PERIOD_MS) == 0U) {
        gOdometryUpdatePending = true;
    }
    if ((gSysTickMs % HEARTBEAT_PERIOD_MS) == 0U) {
        gHeartbeatUpdatePending = true;
    }
    if ((gSysTickMs % STATUS_LED_PERIOD_MS) == 0U) {
        gStatusLedUpdatePending = true;
    }
    if ((gSysTickMs % MENU_PERIOD_MS) == 0U) {
        gMenuUpdatePending = true;
    }
}

void UART_DEBUG_INST_IRQHandler(void)
{
    uart_debug_handle_irq();
}

void UART_OPENMV_INST_IRQHandler(void)
{
    uart_openmv_handle_irq();
}

void UART_JY61P_INST_IRQHandler(void)
{
    jy61_handle_irq();
}

void GROUP1_IRQHandler(void)
{
    encoder_handle_gpio_irq();
}
