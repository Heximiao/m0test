#include "app_menu.h"

#include "app/app_car_control.h"
#include "app/app_attitude.h"
#include "app/app_gray_track.h"
#include "app/app_image_store.h"
#include "app/app_motion_control.h"
#include "hw/hw_encoder.h"
#include "hw/hw_lcd.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MENU_LONG_PRESS_MS (800U)
#define MENU_KEY_DEBOUNCE_MS (100U)
#define MENU_STATUS_REFRESH_MS (500U)
#define MENU_LINE_HEIGHT (24U)
#define MENU_VISIBLE_ROWS (5U)

typedef enum {
    MENU_PAGE_ROOT = 0,
    MENU_PAGE_FILES,
    MENU_PAGE_STATUS,
    MENU_PAGE_CONTROL,
    MENU_PAGE_IMAGE_VIEW,
} MenuPage;

typedef struct {
    bool pressed;
    bool rawPressed;
    bool longSent;
    uint32_t rawChangedMs;
    uint32_t pressStartMs;
} MenuKeyState;

typedef struct {
    const char *label;
    const char *command;
} MenuControlAction;

static void handle_up(void);
static void handle_down(void);
static void handle_back(void);
static void handle_confirm(void);
static void render_menu(uint32_t nowMs);
static void render_menu_step(void);
static void draw_page_header(MenuPage page);
static void draw_page_row(MenuPage page, uint8_t row);
static void draw_page_footer(MenuPage page);
static void draw_header(const char *title);
static void draw_line(uint16_t row, const char *text, bool selected);
static void draw_footer(const char *text);
static void select_previous(uint8_t *selected, uint8_t count);
static void select_next(uint8_t *selected, uint8_t count);
static void run_control_action(const MenuControlAction *action);
static bool is_key_pressed(uint32_t pin);
static bool update_key(MenuKeyState *key, bool rawPressed, uint32_t nowMs,
    bool *pressedEvent, bool *releasedEvent);
static void copy_command(char *dest, size_t destSize, const char *source);

static const char *const gRootItems[] = {
    "File List",
    "Vehicle Status",
    "Vehicle Control",
};

static const MenuControlAction gControlActions[] = {
    {"Start Line Follow", "LINESTART"},
    {"Stop Line Follow", "LINESTOP"},
    {"Gray Input Use", "GRAY 1"},
    {"Gray Input Ignore", "GRAY 0"},
    {"Gyro Input Use", "GYRO 1"},
    {"Gyro Input Ignore", "GYRO 0"},
    {"Stop Motors", "STOP"},
    {"Encoder Zero", "ENCZERO"},
    {"Base Stop", "BASE 0"},
    {"Base Forward", "BASE 7"},
    {"Telemetry On", "TELE 1"},
    {"Telemetry Off", "TELE 0"},
};

static MenuPage gPage = MENU_PAGE_ROOT;
static MenuPage gReturnPage = MENU_PAGE_ROOT;
static uint8_t gRootSelected;
static uint8_t gFileSelected;
static uint8_t gControlSelected;
static bool gDirty = true;
static uint32_t gLastStatusRefreshMs;
static MenuKeyState gBackOkKey;
static MenuKeyState gUpKey;
static MenuKeyState gDownKey;
static char gMessage[32];
static MenuPage gRenderedPage = MENU_PAGE_IMAGE_VIEW;
static MenuPage gRenderPage = MENU_PAGE_ROOT;
static uint8_t gRenderStep;
static bool gRenderActive;

void app_menu_init(uint32_t nowMs)
{
    gPage = MENU_PAGE_ROOT;
    gReturnPage = MENU_PAGE_ROOT;
    gRootSelected = 0U;
    gFileSelected = 0U;
    gControlSelected = 0U;
    gLastStatusRefreshMs = nowMs;
    gBackOkKey.pressed = false;
    gBackOkKey.rawPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_BACK_OK_PIN);
    gBackOkKey.longSent = false;
    gBackOkKey.rawChangedMs = nowMs;
    gBackOkKey.pressStartMs = 0U;
    gUpKey = gBackOkKey;
    gUpKey.rawPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_UP_PIN);
    gDownKey = gBackOkKey;
    gDownKey.rawPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_DOWN_PIN);
    gRenderedPage = MENU_PAGE_IMAGE_VIEW;
    gRenderPage = MENU_PAGE_ROOT;
    gRenderStep = 0U;
    gRenderActive = false;
    gMessage[0] = '\0';
    gDirty = true;
    render_menu(nowMs);
}

void app_menu_update(uint32_t nowMs)
{
    bool upPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_UP_PIN);
    bool downPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_DOWN_PIN);
    bool backOkPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_BACK_OK_PIN);
    bool upEvent;
    bool downEvent;
    bool backPressedEvent;
    bool backReleasedEvent;
    bool ignoredEvent;

    update_key(&gUpKey, upPressed, nowMs, &upEvent, &ignoredEvent);
    update_key(&gDownKey, downPressed, nowMs, &downEvent, &ignoredEvent);
    update_key(&gBackOkKey, backOkPressed, nowMs, &backPressedEvent,
        &backReleasedEvent);

    if (upEvent) {
        handle_up();
    }
    if (downEvent) {
        handle_down();
    }
    if (backReleasedEvent && !gBackOkKey.longSent) {
        handle_back();
    }
    if (gBackOkKey.pressed && !gBackOkKey.longSent &&
        ((uint32_t) (nowMs - gBackOkKey.pressStartMs) >=
            MENU_LONG_PRESS_MS)) {
        gBackOkKey.longSent = true;
        handle_confirm();
    }

    if ((gPage == MENU_PAGE_STATUS) &&
        ((uint32_t) (nowMs - gLastStatusRefreshMs) >=
            MENU_STATUS_REFRESH_MS)) {
        gLastStatusRefreshMs = nowMs;
        gDirty = true;
    }

    if (gDirty || gRenderActive) {
        render_menu(nowMs);
    }
}

static void handle_up(void)
{
    if (gPage == MENU_PAGE_ROOT) {
        select_previous(&gRootSelected,
            (uint8_t) (sizeof(gRootItems) / sizeof(gRootItems[0])));
    } else if (gPage == MENU_PAGE_FILES) {
        select_previous(&gFileSelected, app_image_store_get_file_count());
    } else if (gPage == MENU_PAGE_CONTROL) {
        select_previous(&gControlSelected,
            (uint8_t) (sizeof(gControlActions) / sizeof(gControlActions[0])));
    }
}

static void handle_down(void)
{
    if (gPage == MENU_PAGE_ROOT) {
        select_next(&gRootSelected,
            (uint8_t) (sizeof(gRootItems) / sizeof(gRootItems[0])));
    } else if (gPage == MENU_PAGE_FILES) {
        select_next(&gFileSelected, app_image_store_get_file_count());
    } else if (gPage == MENU_PAGE_CONTROL) {
        select_next(&gControlSelected,
            (uint8_t) (sizeof(gControlActions) / sizeof(gControlActions[0])));
    }
}

static void handle_back(void)
{
    if (gPage == MENU_PAGE_IMAGE_VIEW) {
        gPage = gReturnPage;
    } else if (gPage != MENU_PAGE_ROOT) {
        gPage = MENU_PAGE_ROOT;
    }
    gMessage[0] = '\0';
    gDirty = true;
}

static void handle_confirm(void)
{
    if (gPage == MENU_PAGE_ROOT) {
        if (gRootSelected == 0U) {
            gPage = MENU_PAGE_FILES;
        } else if (gRootSelected == 1U) {
            gPage = MENU_PAGE_STATUS;
            gLastStatusRefreshMs = 0U;
        } else {
            gPage = MENU_PAGE_CONTROL;
        }
        gMessage[0] = '\0';
        gDirty = true;
    } else if (gPage == MENU_PAGE_FILES) {
        AppImageFileInfo info;

        if (app_image_store_get_file_by_index(gFileSelected, &info) &&
            app_image_store_show_image(info.id)) {
            gReturnPage = MENU_PAGE_FILES;
            gPage = MENU_PAGE_IMAGE_VIEW;
            gDirty = false;
        } else {
            copy_command(gMessage, sizeof(gMessage), "Image not found");
            gDirty = true;
        }
    } else if (gPage == MENU_PAGE_CONTROL) {
        run_control_action(&gControlActions[gControlSelected]);
    }
}

static void render_menu(uint32_t nowMs)
{
    (void) nowMs;

    if (gPage == MENU_PAGE_IMAGE_VIEW) {
        gRenderedPage = MENU_PAGE_IMAGE_VIEW;
        gRenderActive = false;
        gDirty = false;
        return;
    }

    if (gDirty) {
        gRenderPage = gPage;
        gRenderStep = 0U;
        gRenderActive = true;
        gDirty = false;
    }

    if (gRenderActive) {
        render_menu_step();
    }
}

static void render_menu_step(void)
{
    if (gRenderStep == 0U) {
        draw_page_header(gRenderPage);
    } else if (gRenderStep <= MENU_VISIBLE_ROWS) {
        draw_page_row(gRenderPage, (uint8_t) (gRenderStep - 1U));
    } else {
        draw_page_footer(gRenderPage);
    }

    gRenderStep++;
    if (gRenderStep > (MENU_VISIBLE_ROWS + 1U)) {
        gRenderedPage = gRenderPage;
        gRenderActive = false;
    }
}

static void draw_page_header(MenuPage page)
{
    if (page == MENU_PAGE_ROOT) {
        draw_header("CAR MENU");
    } else if (page == MENU_PAGE_FILES) {
        draw_header("FILES");
    } else if (page == MENU_PAGE_STATUS) {
        draw_header("STATUS");
    } else {
        draw_header("CONTROL");
    }
}

static void draw_page_row(MenuPage page, uint8_t row)
{
    char line[40];

    if (page == MENU_PAGE_ROOT) {
        if (row < (sizeof(gRootItems) / sizeof(gRootItems[0]))) {
            draw_line(row, gRootItems[row], row == gRootSelected);
        } else {
            draw_line(row, "", false);
        }
    } else if (page == MENU_PAGE_FILES) {
        uint8_t count = app_image_store_get_file_count();
        uint8_t first = 0U;

        if (count == 0U) {
            if (row == 0U) {
                draw_line(row, "No images", false);
            } else if (row == 1U) {
                draw_line(row, "Send with upper_pc", false);
            } else {
                draw_line(row, "", false);
            }
            return;
        }

        if (gFileSelected >= count) {
            gFileSelected = count - 1U;
        }
        if (gFileSelected >= MENU_VISIBLE_ROWS) {
            first = gFileSelected - MENU_VISIBLE_ROWS + 1U;
        }

        AppImageFileInfo info;
        uint8_t index = first + row;

        if ((index < count) &&
            app_image_store_get_file_by_index(index, &info)) {
            snprintf(line, sizeof(line), "%03u %-9s %ux%u", info.id,
                info.name, (unsigned int) info.width,
                (unsigned int) info.height);
            draw_line(row, line, index == gFileSelected);
        } else {
            draw_line(row, "", false);
        }
    } else if (page == MENU_PAGE_STATUS) {
        EncoderCounts counts = encoder_get_counts();

        if (row == 0U) {
            snprintf(line, sizeof(line), "L:%ld R:%ld",
                (long) counts.left_count, (long) counts.right_count);
            draw_line(row, line, false);
        } else if (row == 1U) {
            snprintf(line, sizeof(line), "Mode:%ld Busy:%d",
                (long) motion_control_get_mode(),
                motion_control_is_busy() ? 1 : 0);
            draw_line(row, line, false);
        } else if (row == 2U) {
            snprintf(line, sizeof(line), "MM/s:%ld Deg/s:%ld",
                (long) motion_control_get_target_mm_s(),
                (long) motion_control_get_target_deg_s());
            draw_line(row, line, false);
        } else if (row == 3U) {
            snprintf(line, sizeof(line), "Line:%s Nav:%s",
                app_car_control_is_line_follow_running() ? "RUN" : "STOP",
                app_car_control_is_navigation_active() ? "RUN" : "IDLE");
            draw_line(row, line, false);
        } else {
            snprintf(line, sizeof(line), "Gray:%s Gyro:%s",
                gray_track_is_enabled() ? "USE" : "IGNORE",
                app_attitude_is_enabled() ? "USE" : "IGNORE");
            draw_line(row, line, false);
        }
    } else {
        uint8_t count =
            (uint8_t) (sizeof(gControlActions) / sizeof(gControlActions[0]));
        uint8_t first = 0U;

        if (gControlSelected >= count) {
            gControlSelected = count - 1U;
        }
        if (gControlSelected >= MENU_VISIBLE_ROWS) {
            first = gControlSelected - MENU_VISIBLE_ROWS + 1U;
        }
        uint8_t index = first + row;

        if (index < count) {
            draw_line(row, gControlActions[index].label,
                index == gControlSelected);
        } else {
            draw_line(row, "", false);
        }
    }
}

static void draw_page_footer(MenuPage page)
{
    if (page == MENU_PAGE_ROOT) {
        draw_footer("UP/DOWN  HOLD=OK");
    } else if (page == MENU_PAGE_FILES) {
        draw_footer("BACK  HOLD=SHOW");
    } else if (page == MENU_PAGE_STATUS) {
        draw_footer("BACK");
    } else {
        draw_footer((gMessage[0] != '\0') ? gMessage : "BACK  HOLD=RUN");
    }
}

static void draw_header(const char *title)
{
    LCD_Fill(0U, 0U, LCD_W, 32U, DARKBLUE);
    LCD_ShowString(8U, 4U, (const unsigned char *) title, WHITE, DARKBLUE,
        16U, 0U);
}

static void draw_line(uint16_t row, const char *text, bool selected)
{
    uint16_t y = 32U + (row * MENU_LINE_HEIGHT);
    uint16_t bg = selected ? BLUE : BLACK;
    uint16_t fg = selected ? WHITE : LGRAY;

    LCD_Fill(0U, y, LCD_W, y + MENU_LINE_HEIGHT, bg);
    if ((text != NULL) && (text[0] != '\0')) {
        LCD_ShowString(12U, y + 4U, (const unsigned char *) text, fg, bg,
            16U, 0U);
    }
}

static void draw_footer(const char *text)
{
    LCD_Fill(0U, 152U, LCD_W, LCD_H, GRAY);
    LCD_ShowString(8U, 154U, (const unsigned char *) text, WHITE, GRAY, 16U,
        0U);
}

static void select_previous(uint8_t *selected, uint8_t count)
{
    if ((selected == NULL) || (count == 0U)) {
        return;
    }
    *selected = (*selected == 0U) ? (count - 1U) : (*selected - 1U);
    gDirty = true;
}

static void select_next(uint8_t *selected, uint8_t count)
{
    if ((selected == NULL) || (count == 0U)) {
        return;
    }
    *selected = (uint8_t) ((*selected + 1U) % count);
    gDirty = true;
}

static void run_control_action(const MenuControlAction *action)
{
    char command[24];

    if (action == NULL) {
        return;
    }

    copy_command(command, sizeof(command), action->command);
    app_car_control_process_command(command);
    snprintf(gMessage, sizeof(gMessage), "OK %s", action->label);
    gDirty = true;
}

static bool is_key_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(GPIO_MENU_KEYS_PORT, pin) & pin) == 0U;
}

static bool update_key(MenuKeyState *key, bool rawPressed, uint32_t nowMs,
    bool *pressedEvent, bool *releasedEvent)
{
    if ((key == NULL) || (pressedEvent == NULL) || (releasedEvent == NULL)) {
        return false;
    }
    *pressedEvent = false;
    *releasedEvent = false;

    if (rawPressed != key->rawPressed) {
        key->rawPressed = rawPressed;
        key->rawChangedMs = nowMs;
    }
    if ((rawPressed == key->pressed) ||
        ((uint32_t) (nowMs - key->rawChangedMs) < MENU_KEY_DEBOUNCE_MS)) {
        return false;
    }

    key->pressed = rawPressed;
    if (rawPressed) {
        key->longSent = false;
        key->pressStartMs = nowMs;
        *pressedEvent = true;
    } else {
        *releasedEvent = true;
    }
    return true;
}

static void copy_command(char *dest, size_t destSize, const char *source)
{
    if ((dest == NULL) || (destSize == 0U)) {
        return;
    }

    strncpy(dest, source, destSize - 1U);
    dest[destSize - 1U] = '\0';
}
