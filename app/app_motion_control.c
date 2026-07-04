#include "app_motion_control.h"
#include "app_util.h"
#include <stdio.h>
#include <string.h>

#define CONTROL_PERIOD_S (0.02f) /* 速度控制周期，单位：秒 */
#define WHEEL_DIAMETER_MM (48.0f) /* 实测车轮直径，用于 mm/s 和编码器 counts 换算 */
#define WHEEL_BASE_MM (129.0f) /* 左右后驱轮中心距，原地转向角度按这个轴距换算 */
#define MOTOR_ENCODER_PPR (11.0f) /* MG310 霍尔编码器电机轴每圈脉冲数 */
#define MOTOR_REDUCTION_RATIO (20.0f) /* MG310 减速箱减速比，用于换算到车轮轴 */
#define ENCODER_COUNT_MULTIPLIER (2.0f) /* A/B 两相边沿计数倍率 */
#define PI_F (3.1415926f) /* 圆周率 */
#define DEFAULT_ANGULAR_SPEED_DEG_S (90.0f) /* ANGLE/TURN 只给角度时使用的默认角速度 */
#define MAX_LINEAR_SPEED_MM_S (100.0f) /* 运动命令允许的最大直线速度，单位：mm/s */
#define MAX_ANGULAR_SPEED_DEG_S (120.0f) /* 运动命令允许的最大角速度，单位：deg/s */
#define DIST_DONE_TOLERANCE_COUNTS (8.0f) /* 距离运动到位容差，单位：编码器 counts */
#define TURN_DONE_TOLERANCE_COUNTS (8.0f) /* 转角运动到位容差，单位：编码器 counts */

typedef enum {
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_DRIVE = 1,
    MOTION_MODE_DISTANCE = 2,
    MOTION_MODE_TURN = 3,
} MotionMode;

static MotionMode gMode;
static float gLinearSpeedMmS;
static float gAngularSpeedDegS;
static float gTargetCounts;
static int32_t gStartLeftCount;
static int32_t gStartRightCount;
static bool gBusy;

static int32_t abs_i32_local(int32_t value);
static float counts_per_mm(void);
static float mm_s_to_counts_per_period(float speedMmS);
static void velocity_to_wheel_targets(float linearMmS, float angularDegS,
    float *leftTargetCounts, float *rightTargetCounts);
static void start_drive(float linearMmS, float angularDegS);
static void start_distance(float distanceMm, float speedMmS,
    EncoderCounts counts);
static void start_turn(float angleDeg, float angularSpeedDegS,
    EncoderCounts counts);
static void stop_motion(void);

void motion_control_init(void)
{
    stop_motion();
}

bool motion_control_parse_command(const char *command, EncoderCounts counts)
{
    float first;
    float second;
    int argCount;

    if (strncmp(command, "DRIVE ", 6U) == 0) {
        if (sscanf(command + 6, "%f %f", &first, &second) == 2) {
            start_drive(first, second);
        }
        return true;
    }
    if (strncmp(command, "SPEED ", 6U) == 0) {
        if (sscanf(command + 6, "%f", &first) == 1) {
            start_drive(first, 0.0f);
        }
        return true;
    }
    if (strncmp(command, "DIST ", 5U) == 0) {
        if (sscanf(command + 5, "%f %f", &first, &second) == 2) {
            start_distance(first, second, counts);
        }
        return true;
    }
    if ((strncmp(command, "TURN ", 5U) == 0) ||
        (strncmp(command, "ANGLE ", 6U) == 0)) {
        const char *args = (command[0] == 'T') ? (command + 5) : (command + 6);
        argCount = sscanf(args, "%f %f", &first, &second);
        if (argCount == 1) {
            second = DEFAULT_ANGULAR_SPEED_DEG_S;
            start_turn(first, second, counts);
        } else if (argCount == 2) {
            start_turn(first, second, counts);
        }
        return true;
    }
    if (strcmp(command, "MSTOP") == 0) {
        stop_motion();
        return true;
    }

    return false;
}

void motion_control_update(EncoderCounts counts, float *leftTargetCounts,
    float *rightTargetCounts)
{
    if (gMode == MOTION_MODE_IDLE) {
        return;
    }

    if (gMode == MOTION_MODE_DISTANCE) {
        float leftTravel = (float) (counts.left_count - gStartLeftCount);
        float rightTravel = (float) (counts.right_count - gStartRightCount);
        float travel = app_abs_float((leftTravel + rightTravel) * 0.5f);
        if (travel >= (gTargetCounts - DIST_DONE_TOLERANCE_COUNTS)) {
            stop_motion();
            *leftTargetCounts = 0.0f;
            *rightTargetCounts = 0.0f;
            return;
        }
    } else if (gMode == MOTION_MODE_TURN) {
        float leftTravel = (float) abs_i32_local(counts.left_count -
            gStartLeftCount);
        float rightTravel = (float) abs_i32_local(counts.right_count -
            gStartRightCount);
        float travel = (leftTravel + rightTravel) * 0.5f;
        if (travel >= (gTargetCounts - TURN_DONE_TOLERANCE_COUNTS)) {
            stop_motion();
            *leftTargetCounts = 0.0f;
            *rightTargetCounts = 0.0f;
            return;
        }
    }

    velocity_to_wheel_targets(gLinearSpeedMmS, gAngularSpeedDegS,
        leftTargetCounts, rightTargetCounts);
}

bool motion_control_is_active(void)
{
    return (gMode != MOTION_MODE_IDLE);
}

bool motion_control_is_busy(void)
{
    return gBusy;
}

int32_t motion_control_get_mode(void)
{
    return (int32_t) gMode;
}

int32_t motion_control_get_target_mm_s(void)
{
    return (int32_t) gLinearSpeedMmS;
}

int32_t motion_control_get_target_deg_s(void)
{
    return (int32_t) gAngularSpeedDegS;
}

static void start_drive(float linearMmS, float angularDegS)
{
    gMode = MOTION_MODE_DRIVE;
    gLinearSpeedMmS = app_clamp_float(linearMmS, -MAX_LINEAR_SPEED_MM_S,
        MAX_LINEAR_SPEED_MM_S);
    gAngularSpeedDegS = app_clamp_float(angularDegS, -MAX_ANGULAR_SPEED_DEG_S,
        MAX_ANGULAR_SPEED_DEG_S);
    gTargetCounts = 0.0f;
    gBusy = false;
}

static void start_distance(float distanceMm, float speedMmS,
    EncoderCounts counts)
{
    float direction = (distanceMm >= 0.0f) ? 1.0f : -1.0f;
    float speed = app_abs_float(speedMmS);

    gMode = MOTION_MODE_DISTANCE;
    gLinearSpeedMmS = direction * app_clamp_float(speed, 1.0f,
        MAX_LINEAR_SPEED_MM_S);
    gAngularSpeedDegS = 0.0f;
    gTargetCounts = (app_abs_float(distanceMm) * counts_per_mm()) +
        DIST_DONE_TOLERANCE_COUNTS;
    gStartLeftCount = counts.left_count;
    gStartRightCount = counts.right_count;
    gBusy = true;
}

static void start_turn(float angleDeg, float angularSpeedDegS,
    EncoderCounts counts)
{
    /* 角度正负号用于区分方向：正数左转，负数右转。 */
    float direction = (angleDeg >= 0.0f) ? 1.0f : -1.0f;
    float speed = app_abs_float(angularSpeedDegS);
    float wheelTravelMm = app_abs_float(angleDeg) * PI_F / 180.0f *
        (WHEEL_BASE_MM * 0.5f);

    gMode = MOTION_MODE_TURN;
    gLinearSpeedMmS = 0.0f;
    gAngularSpeedDegS = direction * app_clamp_float(speed, 1.0f,
        MAX_ANGULAR_SPEED_DEG_S);
    gTargetCounts = (wheelTravelMm * counts_per_mm()) +
        TURN_DONE_TOLERANCE_COUNTS;
    gStartLeftCount = counts.left_count;
    gStartRightCount = counts.right_count;
    gBusy = true;
}

static void stop_motion(void)
{
    gMode = MOTION_MODE_IDLE;
    gLinearSpeedMmS = 0.0f;
    gAngularSpeedDegS = 0.0f;
    gTargetCounts = 0.0f;
    gStartLeftCount = 0;
    gStartRightCount = 0;
    gBusy = false;
}

static void velocity_to_wheel_targets(float linearMmS, float angularDegS,
    float *leftTargetCounts, float *rightTargetCounts)
{
    float angularRadS = angularDegS * PI_F / 180.0f;
    float turnMmS = angularRadS * (WHEEL_BASE_MM * 0.5f);
    float leftMmS = linearMmS - turnMmS;
    float rightMmS = linearMmS + turnMmS;

    *leftTargetCounts = mm_s_to_counts_per_period(leftMmS);
    *rightTargetCounts = mm_s_to_counts_per_period(rightMmS);
}

static float mm_s_to_counts_per_period(float speedMmS)
{
    return speedMmS * counts_per_mm() * CONTROL_PERIOD_S;
}

static float counts_per_mm(void)
{
    float countsPerRev = MOTOR_ENCODER_PPR * MOTOR_REDUCTION_RATIO *
        ENCODER_COUNT_MULTIPLIER;
    float wheelCircumferenceMm = WHEEL_DIAMETER_MM * PI_F;

    return countsPerRev / wheelCircumferenceMm;
}

static int32_t abs_i32_local(int32_t value)
{
    return (value < 0) ? -value : value;
}
