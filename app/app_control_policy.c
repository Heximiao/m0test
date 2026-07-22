#include "app_control_policy.h"

static bool gLineFollowRequested;
static bool gGrayEnabled;
static bool gGyroEnabled;
static bool gNavigationActive;

void control_policy_init(void)
{
    gLineFollowRequested = false;
    gGrayEnabled = true;
    gGyroEnabled = true;
    gNavigationActive = false;
}

bool control_policy_start_line_follow(void)
{
    if (!gGrayEnabled) {
        return false;
    }
    gLineFollowRequested = true;
    return true;
}

void control_policy_stop_line_follow(void)
{
    gLineFollowRequested = false;
}

bool control_policy_is_line_follow_requested(void)
{
    return gLineFollowRequested;
}

bool control_policy_can_run_line_follow(void)
{
    return gLineFollowRequested && gGrayEnabled && !gNavigationActive;
}
void control_policy_set_gray_enabled(bool enabled)
{
    gGrayEnabled = enabled;
    if (!enabled) {
        gLineFollowRequested = false;
    }
}

bool control_policy_is_gray_enabled(void)
{
    return gGrayEnabled;
}

void control_policy_set_gyro_enabled(bool enabled)
{
    gGyroEnabled = enabled;
}

bool control_policy_is_gyro_enabled(void)
{
    return gGyroEnabled;
}

void control_policy_set_navigation_active(bool active)
{
    gNavigationActive = active;
}

bool control_policy_is_navigation_active(void)
{
    return gNavigationActive;
}
