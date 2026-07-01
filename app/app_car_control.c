#include "app_car_control.h"
#include "app_line_follow.h"
#include "app_motion_control.h"
#include "bsp/bsp_tb6612.h"
#include "hw/hw_encoder.h"
#include "hw/hw_uart.h"
#include "mid/mid_pid.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TELEMETRY_BUFFER_SIZE (256U) /* 串口遥测发送缓冲区字节数 */
#define FIRMWARE_VERSION "diag-20260630-refactor" /* 上电时打印的固件版本 */

#define MIN_DUTY_PERCENT (0.0f) /* 电机 PWM 占空比下限，单位：百分比 */
#define MIN_TARGET_SPEED_COUNTS (-60.0f) /* BASE/目标速度允许的最小编码器增量，单位：counts/20ms */
#define MIN_RUNNING_TARGET_SPEED_COUNTS (0.0f) /* 非零速度命令的最小绝对值；当前不强制起步速度 */
#define MAX_TARGET_SPEED_COUNTS (60.0f) /* BASE/目标速度允许的最大编码器增量，单位：counts/20ms */
#define SPEED_LOOP_START_DUTY_PERCENT (8.0f) /* 速度闭环前馈的起步占空比，用来克服静摩擦 */
#define SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT (0.70f) /* 每 1 count/20ms 目标速度对应的基础占空比 */
#define LEFT_SPEED_FEED_FORWARD_SCALE (1.00f) /* 左轮前馈比例，用来校准左右电机差异 */
#define RIGHT_SPEED_FEED_FORWARD_SCALE (0.96f) /* 右轮前馈比例；小于 1 表示右轮基础输出略降 */
#define MAX_DUTY_PERCENT (85.0f) /* 电机 PWM 占空比上限，避免满占空比失控 */
#define TARGET_SPEED_RAMP_STEP_COUNTS (0.25f) /* BASE 目标斜坡步进，单位：counts/20ms/控制周期 */
#define MANUAL_DUTY_RAMP_STEP_PERCENT (0.5f) /* PWM/MOTOR 手动模式占空比斜坡步进，单位：百分比/控制周期 */
#define MANUAL_MOTOR_DEFAULT_TIMEOUT_MS (2000U) /* MOTOR 命令默认运行时间，单位：ms */
#define MANUAL_MOTOR_MAX_TIMEOUT_MS (10000U) /* MOTOR/PWM 命令允许的最长运行时间，单位：ms */
#define PID_INTEGRAL_LIMIT (200.0f) /* PID 积分项限幅，防止积分累积过大 */
#define PID_OUTPUT_LIMIT (85.0f) /* 单个速度 PID 输出限幅，单位：占空比百分比 */
#define SPEED_FILTER_ALPHA (0.25f) /* 编码器速度一阶低通滤波系数，越大响应越快但越抖 */
#define LINE_LOST_SPEED_SCALE (0.35f) /* 巡线丢线时的速度缩放比例 */
#define LINE_FOLLOW_BASE_COUNTS (10.0f) /* 巡线自动前进基础速度，单位：counts/20ms */
#define LINE_FOLLOW_MIN_FORWARD_COUNTS (0.0f) /* 巡线时单轮最小前进速度，单位：counts/20ms */
#define LINE_FOLLOW_MAX_COUNTS (14.0f) /* 巡线自动前进速度上限，单位：counts/20ms */

static PID gLeftSpeedPid;
static PID gRightSpeedPid;
static float gSpeedTargetCounts;
static float gTargetSpeedCounts;
static float gManualLeftDutyPercent;
static float gManualRightDutyPercent;
static float gTargetManualLeftDutyPercent;
static float gTargetManualRightDutyPercent;
static uint32_t gLastUpdateMs;
static uint32_t gManualPulseStopMs;
static bool gTelemetryEnabled;
static bool gManualMotorMode;
static int32_t gPreviousLeftCount;
static int32_t gPreviousRightCount;
static int32_t gLastLeftDelta;
static int32_t gLastRightDelta;
static int32_t gLastPidError;
static float gLastLeftOutput;
static float gLastRightOutput;
static float gLastLeftTargetSpeed;
static float gLastRightTargetSpeed;
static float gFilteredLeftSpeed;
static float gFilteredRightSpeed;

static void set_motor_duty(float leftDutyPercent, float rightDutyPercent);
static void update_speed_target_ramp(void);
static void update_manual_motor_ramp(uint32_t nowMs);
static uint32_t duty_percent_to_speed(float dutyPercent);
static float clamp_speed_target_command(float speedCounts);
static float clamp_line_follow_speed(float speedCounts);
static float clamp_motor_test_duty_command(float dutyPercent);
static float apply_speed_loop_start_duty(float pidOutput, float targetSpeed,
    float feedForwardScale);
static float lowpass_filter(float previous, float current);
static float ramp_toward(float current, float target, float step);
static void send_status(void);
static void send_command_ack(const char *command, float firstValue,
    float secondValue);
static void send_pulse_ack(float dutyPercent, unsigned long durationMs);
static void send_motor_ack(const char *command, float firstValue,
    float secondValue, unsigned long durationMs);
static char *normalize_command(char *command);
static void send_unknown_command(const char *command);
static int32_t scale_float_100(float value);
static int32_t scale_float_1000(float value);
static float abs_float(float value);

void app_car_control_init(void)
{
    pid_init(&gLeftSpeedPid, 4.0f, 0.2f, 0.4f, PID_INTEGRAL_LIMIT,
        PID_OUTPUT_LIMIT);
    pid_init(&gRightSpeedPid, 4.0f, 0.2f, 0.4f, PID_INTEGRAL_LIMIT,
        PID_OUTPUT_LIMIT);
    line_follow_init();
    motion_control_init();

    DL_GPIO_setPins(GPIO_STATUS_LED_PORT,
        GPIO_STATUS_LED_PB22_LED_PIN | GPIO_STATUS_LED_PB26_LED_PIN |
            GPIO_STATUS_LED_PB27_LED_PIN);

    uart_debug_write_string("MSPM0 car PID " FIRMWARE_VERSION
        " ready, UART0 PA10 TX PA11 RX 115200 8N1\r\n");
}

void app_car_control_update(uint32_t nowMs)
{
    EncoderCounts counts = encoder_get_counts();
    int32_t leftDelta = counts.left_count - gPreviousLeftCount;
    int32_t rightDelta = counts.right_count - gPreviousRightCount;

    gLastUpdateMs = nowMs;
    gPreviousLeftCount = counts.left_count;
    gPreviousRightCount = counts.right_count;

    if (gManualMotorMode) {
        gLastLeftDelta = leftDelta;
        gLastRightDelta = rightDelta;
        update_manual_motor_ramp(nowMs);
        return;
    }

    update_speed_target_ramp();

    int32_t leftSpeed = leftDelta;
    int32_t rightSpeed = rightDelta;
    gFilteredLeftSpeed = lowpass_filter(gFilteredLeftSpeed, (float) leftSpeed);
    gFilteredRightSpeed = lowpass_filter(gFilteredRightSpeed, (float) rightSpeed);

    gLastLeftDelta = scale_float_100(gFilteredLeftSpeed);
    gLastRightDelta = scale_float_100(gFilteredRightSpeed);
    gLastPidError = gLastLeftDelta - gLastRightDelta;
    float leftTargetSpeed = gSpeedTargetCounts;
    float rightTargetSpeed = gSpeedTargetCounts;
    float lineTurnAdjust = line_follow_get_turn_adjust(nowMs);
    bool lineAutoDrive = false;

    motion_control_update(counts, &leftTargetSpeed, &rightTargetSpeed);

    if (line_follow_is_active(nowMs) && !motion_control_is_active() &&
        (leftTargetSpeed == 0.0f) && (rightTargetSpeed == 0.0f)) {
        leftTargetSpeed = LINE_FOLLOW_BASE_COUNTS;
        rightTargetSpeed = LINE_FOLLOW_BASE_COUNTS;
        lineAutoDrive = true;
    }

    if (line_follow_is_valid(nowMs)) {
        /*
         * OpenMV sends a positive line error when the target line is to the
         * right side of the image. Positive correction should speed up the
         * right wheel and slow down the left wheel.
         */
        leftTargetSpeed -= lineTurnAdjust;
        rightTargetSpeed += lineTurnAdjust;
        if (!motion_control_is_active()) {
            if (leftTargetSpeed < LINE_FOLLOW_MIN_FORWARD_COUNTS) {
                leftTargetSpeed = LINE_FOLLOW_MIN_FORWARD_COUNTS;
            }
            if (rightTargetSpeed < LINE_FOLLOW_MIN_FORWARD_COUNTS) {
                rightTargetSpeed = LINE_FOLLOW_MIN_FORWARD_COUNTS;
            }
        }
    } else if (line_follow_is_active(nowMs)) {
        leftTargetSpeed *= LINE_LOST_SPEED_SCALE;
        rightTargetSpeed *= LINE_LOST_SPEED_SCALE;
    }

    leftTargetSpeed = clamp_speed_target_command(leftTargetSpeed);
    rightTargetSpeed = clamp_speed_target_command(rightTargetSpeed);
    if (lineAutoDrive) {
        leftTargetSpeed = clamp_line_follow_speed(leftTargetSpeed);
        rightTargetSpeed = clamp_line_follow_speed(rightTargetSpeed);
    }
    gLastLeftTargetSpeed = leftTargetSpeed;
    gLastRightTargetSpeed = rightTargetSpeed;

    gLastLeftOutput = pid_calc(&gLeftSpeedPid, leftTargetSpeed,
        gFilteredLeftSpeed);
    gLastRightOutput = pid_calc(&gRightSpeedPid, rightTargetSpeed,
        gFilteredRightSpeed);
    gLastLeftOutput = apply_speed_loop_start_duty(gLastLeftOutput,
        leftTargetSpeed, LEFT_SPEED_FEED_FORWARD_SCALE);
    gLastRightOutput = apply_speed_loop_start_duty(gLastRightOutput,
        rightTargetSpeed, RIGHT_SPEED_FEED_FORWARD_SCALE);

    set_motor_duty(gLastLeftOutput, gLastRightOutput);
}

void app_car_control_process_command(char *command)
{
    command = normalize_command(command);

    if (line_follow_parse_command(command, gLastUpdateMs)) {
        return;
    } else if (motion_control_parse_command(command, encoder_get_counts())) {
        gTargetSpeedCounts = 0.0f;
        gSpeedTargetCounts = 0.0f;
        uart_debug_write_string("OK MOTION\r\n");
    } else if (strncmp(command, "PID ", 4U) == 0) {
        float kp;
        float ki;
        float kd;

        if (sscanf(command + 4, "%f %f %f", &kp, &ki, &kd) == 3) {
            gLeftSpeedPid.kp = kp;
            gLeftSpeedPid.ki = ki;
            gLeftSpeedPid.kd = kd;
            gRightSpeedPid.kp = kp;
            gRightSpeedPid.ki = ki;
            gRightSpeedPid.kd = kd;
            pid_reset(&gLeftSpeedPid);
            pid_reset(&gRightSpeedPid);
            uart_debug_write_string("OK PID\r\n");
        } else {
            uart_debug_write_string("ERR PID\r\n");
        }
    } else if (strncmp(command, "BASE ", 5U) == 0) {
        float base = strtof(command + 5, NULL);
        base = clamp_speed_target_command(base);
        gManualMotorMode = false;
        motion_control_init();
        gTargetSpeedCounts = base;
        if (base == 0.0f) {
            gSpeedTargetCounts = 0.0f;
            pid_reset(&gLeftSpeedPid);
            pid_reset(&gRightSpeedPid);
            set_motor_duty(0.0f, 0.0f);
        }
        send_command_ack("BASE", base, base);
    } else if (strncmp(command, "MOTOR ", 6U) == 0) {
        float duty;
        unsigned long durationMs = MANUAL_MOTOR_DEFAULT_TIMEOUT_MS;

        if (sscanf(command + 6, "%f %lu", &duty, &durationMs) < 1) {
            uart_debug_write_string("ERR MOTOR\r\n");
            return;
        }
        duty = clamp_motor_test_duty_command(duty);
        if (durationMs > MANUAL_MOTOR_MAX_TIMEOUT_MS) {
            durationMs = MANUAL_MOTOR_MAX_TIMEOUT_MS;
        }
        gManualMotorMode = true;
        motion_control_init();
        gTargetManualLeftDutyPercent = duty;
        gTargetManualRightDutyPercent = duty;
        gManualPulseStopMs = (durationMs == 0U) ? 0U :
            (gLastUpdateMs + (uint32_t) durationMs);
        pid_reset(&gLeftSpeedPid);
        pid_reset(&gRightSpeedPid);
        send_motor_ack("MOTOR", duty, duty, durationMs);
    } else if (strncmp(command, "PWM ", 4U) == 0) {
        float leftDuty;
        float rightDuty;
        unsigned long durationMs = MANUAL_MOTOR_DEFAULT_TIMEOUT_MS;

        if (sscanf(command + 4, "%f %f %lu", &leftDuty, &rightDuty,
                &durationMs) >= 2) {
            leftDuty = clamp_motor_test_duty_command(leftDuty);
            rightDuty = clamp_motor_test_duty_command(rightDuty);
            if (durationMs > MANUAL_MOTOR_MAX_TIMEOUT_MS) {
                durationMs = MANUAL_MOTOR_MAX_TIMEOUT_MS;
            }
            gManualMotorMode = true;
            motion_control_init();
            gTargetManualLeftDutyPercent = leftDuty;
            gTargetManualRightDutyPercent = rightDuty;
            gManualPulseStopMs = (durationMs == 0U) ? 0U :
                (gLastUpdateMs + (uint32_t) durationMs);
            pid_reset(&gLeftSpeedPid);
            pid_reset(&gRightSpeedPid);
            send_motor_ack("PWM", leftDuty, rightDuty, durationMs);
        } else {
            uart_debug_write_string("ERR PWM\r\n");
        }
    } else if (strncmp(command, "PULSE ", 6U) == 0) {
        float duty;
        unsigned long durationMs;

        if (sscanf(command + 6, "%f %lu", &duty, &durationMs) == 2) {
            duty = clamp_motor_test_duty_command(duty);
            if (durationMs > 2000U) {
                durationMs = 2000U;
            }
            gManualMotorMode = true;
            motion_control_init();
            gTargetManualLeftDutyPercent = duty;
            gTargetManualRightDutyPercent = duty;
            gManualPulseStopMs = gLastUpdateMs + (uint32_t) durationMs;
            pid_reset(&gLeftSpeedPid);
            pid_reset(&gRightSpeedPid);
            send_pulse_ack(duty, durationMs);
        } else {
            uart_debug_write_string("ERR PULSE\r\n");
        }
    } else if (strncmp(command, "QUIET ", 6U) == 0) {
        gTelemetryEnabled = (strtol(command + 6, NULL, 10) == 0);
        uart_debug_write_string(gTelemetryEnabled ? "OK QUIET 0\r\n" :
                                                    "OK QUIET 1\r\n");
    } else if (strncmp(command, "TELE ", 5U) == 0) {
        gTelemetryEnabled = (strtol(command + 5, NULL, 10) != 0);
        uart_debug_write_string(gTelemetryEnabled ? "OK TELE 1\r\n" :
                                                    "OK TELE 0\r\n");
    } else if (strcmp(command, "STOP") == 0) {
        gManualMotorMode = false;
        motion_control_init();
        gTargetSpeedCounts = 0.0f;
        gSpeedTargetCounts = 0.0f;
        gManualLeftDutyPercent = 0.0f;
        gManualRightDutyPercent = 0.0f;
        gTargetManualLeftDutyPercent = 0.0f;
        gTargetManualRightDutyPercent = 0.0f;
        gManualPulseStopMs = 0U;
        pid_reset(&gLeftSpeedPid);
        pid_reset(&gRightSpeedPid);
        set_motor_duty(0.0f, 0.0f);
        uart_debug_write_string("OK STOP\r\n");
    } else if (strcmp(command, "GET") == 0) {
        send_status();
    } else if (strcmp(command, "ENCZERO") == 0) {
        encoder_reset();
        gPreviousLeftCount = 0;
        gPreviousRightCount = 0;
        uart_debug_write_string("OK ENCZERO\r\n");
    } else {
        send_unknown_command(command);
    }
}

void app_car_control_send_telemetry(void)
{
    char message[TELEMETRY_BUFFER_SIZE];
    EncoderCounts counts;
    int length;

    if (!gTelemetryEnabled) {
        return;
    }

    counts = encoder_get_counts();
    length = snprintf(message, sizeof(message),
        "L=%ld R=%ld LD=%ld RD=%ld ERR=%ld OUT=%ld LO=%ld RO=%ld LT=%ld RT=%ld KP=%ld KI=%ld KD=%ld BASE=%ld LINE=%ld LV=%d MM=%ld DG=%ld MO=%ld MB=%d\r\n",
        (long) counts.left_count, (long) counts.right_count,
        (long) gLastLeftDelta, (long) gLastRightDelta, (long) gLastPidError,
        (long) scale_float_100(gLastRightOutput - gLastLeftOutput),
        (long) scale_float_100(gLastLeftOutput),
        (long) scale_float_100(gLastRightOutput),
        (long) scale_float_100(gLastLeftTargetSpeed),
        (long) scale_float_100(gLastRightTargetSpeed),
        (long) scale_float_1000(gLeftSpeedPid.kp),
        (long) scale_float_1000(gLeftSpeedPid.ki),
        (long) scale_float_1000(gLeftSpeedPid.kd),
        (long) scale_float_100(gSpeedTargetCounts),
        (long) line_follow_get_error(),
        line_follow_is_valid(gLastUpdateMs) ? 1 : 0,
        (long) motion_control_get_target_mm_s(),
        (long) motion_control_get_target_deg_s(),
        (long) motion_control_get_mode(),
        motion_control_is_busy() ? 1 : 0);

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    }
}

void app_car_control_send_heartbeat(uint32_t nowMs)
{
    char message[TELEMETRY_BUFFER_SIZE];
    EncoderCounts counts = encoder_get_counts();
    int length = snprintf(message, sizeof(message),
        "RUN ms=%lu L=%ld R=%ld RXDROP=%lu TXDROP=%lu LED=PB22/PB26/PB27\r\n",
        (unsigned long) nowMs, (long) counts.left_count,
        (long) counts.right_count,
        (unsigned long) uart_debug_get_rx_dropped_bytes(),
        (unsigned long) uart_debug_get_tx_dropped_bytes());

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    }
}

void app_car_control_toggle_status_led(void)
{
    DL_GPIO_togglePins(GPIO_STATUS_LED_PORT,
        GPIO_STATUS_LED_PB22_LED_PIN | GPIO_STATUS_LED_PB26_LED_PIN |
            GPIO_STATUS_LED_PB27_LED_PIN);
}

static void set_motor_duty(float leftDutyPercent, float rightDutyPercent)
{
    if ((leftDutyPercent == 0.0f) && (rightDutyPercent == 0.0f)) {
        TB6612_Motor_Stop();
        return;
    }

    /*
     * Board wiring: TB6612 channel A drives the physical left wheel, and
     * channel B drives the physical right wheel.
     */
    AO_Control((leftDutyPercent >= 0.0f) ? 1U : 0U,
        duty_percent_to_speed(abs_float(leftDutyPercent)));
    BO_Control((rightDutyPercent >= 0.0f) ? 1U : 0U,
        duty_percent_to_speed(abs_float(rightDutyPercent)));
}

static void update_speed_target_ramp(void)
{
    gSpeedTargetCounts = ramp_toward(gSpeedTargetCounts, gTargetSpeedCounts,
        TARGET_SPEED_RAMP_STEP_COUNTS);
}

static void update_manual_motor_ramp(uint32_t nowMs)
{
    if ((gManualPulseStopMs != 0U) &&
        ((int32_t) (nowMs - gManualPulseStopMs) >= 0)) {
        gTargetManualLeftDutyPercent = 0.0f;
        gTargetManualRightDutyPercent = 0.0f;
        gManualPulseStopMs = 0U;
    }

    gManualLeftDutyPercent = ramp_toward(gManualLeftDutyPercent,
        gTargetManualLeftDutyPercent, MANUAL_DUTY_RAMP_STEP_PERCENT);
    gManualRightDutyPercent = ramp_toward(gManualRightDutyPercent,
        gTargetManualRightDutyPercent, MANUAL_DUTY_RAMP_STEP_PERCENT);

    set_motor_duty(gManualLeftDutyPercent, gManualRightDutyPercent);

    if ((gTargetManualLeftDutyPercent == 0.0f) &&
        (gTargetManualRightDutyPercent == 0.0f) &&
        (gManualLeftDutyPercent == 0.0f) &&
        (gManualRightDutyPercent == 0.0f)) {
        gManualMotorMode = false;
    }
}

static uint32_t duty_percent_to_speed(float dutyPercent)
{
    if (dutyPercent < MIN_DUTY_PERCENT) {
        dutyPercent = MIN_DUTY_PERCENT;
    } else if (dutyPercent > MAX_DUTY_PERCENT) {
        dutyPercent = MAX_DUTY_PERCENT;
    }

    return (uint32_t) (((dutyPercent * (float) TB6612_SPEED_MAX) / 100.0f) +
                       0.5f);
}

static float clamp_speed_target_command(float speedCounts)
{
    if (speedCounts < MIN_TARGET_SPEED_COUNTS) {
        return MIN_TARGET_SPEED_COUNTS;
    }
    if ((speedCounts > 0.0f) && (speedCounts < MIN_RUNNING_TARGET_SPEED_COUNTS)) {
        return MIN_RUNNING_TARGET_SPEED_COUNTS;
    }
    if ((speedCounts < 0.0f) &&
        (speedCounts > -MIN_RUNNING_TARGET_SPEED_COUNTS)) {
        return -MIN_RUNNING_TARGET_SPEED_COUNTS;
    }
    if (speedCounts > MAX_TARGET_SPEED_COUNTS) {
        return MAX_TARGET_SPEED_COUNTS;
    }

    return speedCounts;
}

static float clamp_line_follow_speed(float speedCounts)
{
    if (speedCounts < 0.0f) {
        return 0.0f;
    }
    if (speedCounts > LINE_FOLLOW_MAX_COUNTS) {
        return LINE_FOLLOW_MAX_COUNTS;
    }

    return speedCounts;
}

static float clamp_motor_test_duty_command(float dutyPercent)
{
    if (dutyPercent < MIN_DUTY_PERCENT) {
        return MIN_DUTY_PERCENT;
    }
    if (dutyPercent > MAX_DUTY_PERCENT) {
        return MAX_DUTY_PERCENT;
    }

    return dutyPercent;
}

static float apply_speed_loop_start_duty(float pidOutput, float targetSpeed,
    float feedForwardScale)
{
    float feedForwardDuty;

    if (targetSpeed == 0.0f) {
        return 0.0f;
    }

    feedForwardDuty = SPEED_LOOP_START_DUTY_PERCENT +
        (abs_float(targetSpeed) * SPEED_LOOP_FEED_FORWARD_PERCENT_PER_COUNT);
    feedForwardDuty *= feedForwardScale;

    if (targetSpeed > 0.0f) {
        return feedForwardDuty + pidOutput;
    }

    return -feedForwardDuty + pidOutput;
}

static float lowpass_filter(float previous, float current)
{
    return previous + (SPEED_FILTER_ALPHA * (current - previous));
}

static float ramp_toward(float current, float target, float step)
{
    if (current < target) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }

    return current;
}

static void send_status(void)
{
    char message[TELEMETRY_BUFFER_SIZE];
    uint32_t standbyPins = DL_GPIO_readPins(GPIO_MOTOR_A_PORT,
        GPIO_MOTOR_A_STBY_PIN);
    int length = snprintf(message, sizeof(message),
        "STATUS FW=%s SPEED=%ld TARGET=%ld ML=%ld MR=%ld MANUAL=%d TELE=%d STBY=%d RXDROP=%lu TXDROP=%lu KP=%ld KI=%ld KD=%ld\r\n",
        FIRMWARE_VERSION, (long) scale_float_100(gSpeedTargetCounts),
        (long) scale_float_100(gTargetSpeedCounts),
        (long) scale_float_100(gManualLeftDutyPercent),
        (long) scale_float_100(gManualRightDutyPercent),
        gManualMotorMode ? 1 : 0,
        gTelemetryEnabled ? 1 : 0,
        ((standbyPins & GPIO_MOTOR_A_STBY_PIN) != 0U) ? 1 : 0,
        (unsigned long) uart_debug_get_rx_dropped_bytes(),
        (unsigned long) uart_debug_get_tx_dropped_bytes(),
        (long) scale_float_1000(gLeftSpeedPid.kp),
        (long) scale_float_1000(gLeftSpeedPid.ki),
        (long) scale_float_1000(gLeftSpeedPid.kd));

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void send_command_ack(const char *command, float firstValue,
    float secondValue)
{
    char message[TELEMETRY_BUFFER_SIZE];
    int length = snprintf(message, sizeof(message), "OK %s L=%ld R=%ld\r\n",
        command, (long) scale_float_100(firstValue),
        (long) scale_float_100(secondValue));

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void send_pulse_ack(float dutyPercent, unsigned long durationMs)
{
    char message[TELEMETRY_BUFFER_SIZE];
    int length = snprintf(message, sizeof(message), "OK PULSE DUTY=%ld MS=%lu\r\n",
        (long) scale_float_100(dutyPercent), durationMs);

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static void send_motor_ack(const char *command, float firstValue,
    float secondValue, unsigned long durationMs)
{
    char message[TELEMETRY_BUFFER_SIZE];
    int length = snprintf(message, sizeof(message),
        "OK %s L=%ld R=%ld MS=%lu\r\n", command,
        (long) scale_float_100(firstValue), (long) scale_float_100(secondValue),
        durationMs);

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    }
}

static char *normalize_command(char *command)
{
    while ((*command == ' ') || (*command == '\t') || (*command == '>')) {
        command++;
    }

    char *end = command + strlen(command);
    while (end > command) {
        char previous = *(end - 1);
        if ((previous != ' ') && (previous != '\t')) {
            break;
        }
        end--;
    }
    *end = '\0';

    for (char *cursor = command; *cursor != '\0'; cursor++) {
        if ((*cursor >= 'a') && (*cursor <= 'z')) {
            *cursor = (char) (*cursor - ('a' - 'A'));
        }
    }

    return command;
}

static void send_unknown_command(const char *command)
{
    char message[TELEMETRY_BUFFER_SIZE];
    int length = snprintf(message, sizeof(message), "ERR CMD RX=[%s]\r\n",
        command);

    if ((length > 0) && (length < (int) sizeof(message))) {
        uart_debug_write_string(message);
    } else {
        uart_debug_write_string("ERR CMD RX_TOO_LONG\r\n");
    }
}

static int32_t scale_float_100(float value)
{
    if (value >= 0.0f) {
        return (int32_t) ((value * 100.0f) + 0.5f);
    }

    return (int32_t) ((value * 100.0f) - 0.5f);
}

static int32_t scale_float_1000(float value)
{
    if (value >= 0.0f) {
        return (int32_t) ((value * 1000.0f) + 0.5f);
    }

    return (int32_t) ((value * 1000.0f) - 0.5f);
}

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}
