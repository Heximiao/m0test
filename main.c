/*
 * MSPM0 car controller entry point.
 *
 * The application is split like the EasyPidKit reference project:
 * hardware drivers own pins/peripherals, middle modules own reusable control
 * logic, and app_car_control owns the vehicle behavior.
 */

#include "app/app_car_control.h"
#include "app/app_mpu6050_attitude.h"
#include "bsp/bsp_tb6612.h"
#include "hw/hw_encoder.h"
#include "hw/hw_openmv_uart.h"
#include "hw/hw_uart.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

#define CONTROL_PERIOD_MS (20U)
#define ATTITUDE_PERIOD_MS (100U)
#define TELEMETRY_PERIOD_MS (100U)
#define HEARTBEAT_PERIOD_MS (30000U)
#define STATUS_LED_PERIOD_MS (500U)

static volatile bool gControlUpdatePending;
static volatile bool gAttitudeUpdatePending;
static volatile bool gTelemetryUpdatePending;
static volatile bool gHeartbeatUpdatePending;
static volatile bool gStatusLedUpdatePending;
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
     *   PB9  -> PWMA, right wheel PWM (TIMA0_CCP1)
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
     * OpenMV vision UART:
     *   PA23 -> UART2 TX, optional MCU-to-OpenMV line
     *   PA24 -> UART2 RX, connect to OpenMV TX       openmv的4和5脚
     *   Baud rate: 115200, 8N1
     *   OpenMV P4 TX -> PA24 RX
     *   OpenMV P5 RX <- PA23 TX
     *
     * LaunchPad status LED:
     *   PB22/PB26/PB27 -> LED2 blue/red/green
     *
     * MPU6050 software I2C:
     *   PA1 -> SCL
     *   PA0 -> SDA
     *   DMP attitude telemetry: ATT PITCH/ROLL/YAW, degrees * 100
     */
    encoder_init();
    uart_debug_init();
    uart_openmv_init();
    TB6612_Init();
    app_car_control_init();

    app_mpu6050_attitude_init();

    SysTick_Config(CPUCLK_FREQ / 1000U);

    while (1) {
        char command[UART_DEBUG_LINE_BUFFER_SIZE];

        while (uart_debug_read_line(command, sizeof(command))) {
            if (!app_mpu6050_attitude_process_command(command)) {
                app_car_control_process_command(command);
            }
        }

        while (uart_openmv_read_line(command, sizeof(command))) {
            app_car_control_process_command(command);
        }

        uart_debug_service_tx();

        if (gControlUpdatePending) {
            gControlUpdatePending = false;
            app_car_control_update(gSysTickMs);
        }

        if (gAttitudeUpdatePending) {
            gAttitudeUpdatePending = false;
            app_mpu6050_attitude_send();
        }

        if (gTelemetryUpdatePending) {
            gTelemetryUpdatePending = false;
            app_car_control_send_telemetry();
        }

        if (gHeartbeatUpdatePending) {
            gHeartbeatUpdatePending = false;
            app_car_control_send_heartbeat(gSysTickMs);
        }

        if (gStatusLedUpdatePending) {
            gStatusLedUpdatePending = false;
            app_car_control_toggle_status_led();
        }

        uart_debug_service_tx();
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
    if ((gSysTickMs % HEARTBEAT_PERIOD_MS) == 0U) {
        gHeartbeatUpdatePending = true;
    }
    if ((gSysTickMs % STATUS_LED_PERIOD_MS) == 0U) {
        gStatusLedUpdatePending = true;
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

void GROUP1_IRQHandler(void)
{
    encoder_handle_gpio_irq();
}
