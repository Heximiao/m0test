#include "app_line_follow.h"
#include <stdio.h>
#include <string.h>

#define LINE_LOST_TIMEOUT_MS (500U) /* 超过这个时间没有 LINE 数据就认为巡线丢失，单位：ms */
#define LINE_ERROR_LIMIT_PIXELS (160) /* OpenMV 发送的横向误差限幅，单位：像素，右侧为正 */
#define LINE_TURN_KP (0.04f) /* 像素误差到左右轮目标速度差的比例 */
#define LINE_TURN_LIMIT_COUNTS (4.0f) /* 巡线转向修正限幅，单位：counts/20ms */

static bool gLineEnabled;
static bool gLineValid;
static int32_t gLineError;
static uint32_t gLastLineMs;

static int32_t clamp_line_error(int32_t error);
static float clamp_turn(float turn);

void line_follow_init(void)
{
    gLineEnabled = false;
    gLineValid = false;
    gLineError = 0;
    gLastLineMs = 0U;
}

bool line_follow_parse_command(const char *command, uint32_t nowMs)
{
    int valid;
    int error;

    if (strncmp(command, "LINE ", 5U) != 0) {
        return false;
    }

    if (sscanf(command + 5, "%d %d", &valid, &error) != 2) {
        return true;
    }

    gLineEnabled = true;
    gLineValid = (valid != 0);
    gLineError = clamp_line_error((int32_t) error);
    gLastLineMs = nowMs;
    return true;
}

float line_follow_get_turn_adjust(uint32_t nowMs)
{
    if (!line_follow_is_valid(nowMs)) {
        return 0.0f;
    }

    return clamp_turn((float) gLineError * LINE_TURN_KP);
}

bool line_follow_is_active(uint32_t nowMs)
{
    if (!gLineEnabled) {
        return false;
    }

    return ((uint32_t) (nowMs - gLastLineMs) <= LINE_LOST_TIMEOUT_MS);
}

bool line_follow_is_valid(uint32_t nowMs)
{
    return line_follow_is_active(nowMs) && gLineValid;
}

int32_t line_follow_get_error(void)
{
    return gLineError;
}

static int32_t clamp_line_error(int32_t error)
{
    if (error < -LINE_ERROR_LIMIT_PIXELS) {
        return -LINE_ERROR_LIMIT_PIXELS;
    }
    if (error > LINE_ERROR_LIMIT_PIXELS) {
        return LINE_ERROR_LIMIT_PIXELS;
    }

    return error;
}

static float clamp_turn(float turn)
{
    if (turn < -LINE_TURN_LIMIT_COUNTS) {
        return -LINE_TURN_LIMIT_COUNTS;
    }
    if (turn > LINE_TURN_LIMIT_COUNTS) {
        return LINE_TURN_LIMIT_COUNTS;
    }

    return turn;
}
