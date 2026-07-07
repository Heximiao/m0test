#include "app_menu.h"

#include "app/app_car_control.h"
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
    bool longSent;
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
static void render_root(void);
static void render_files(void);
static void render_status(void);
static void render_control(void);
static void draw_header(const char *title);
static void draw_line(uint16_t row, const char *text, bool selected);
static void draw_footer(const char *text);
static void select_previous(uint8_t *selected, uint8_t count);
static void select_next(uint8_t *selected, uint8_t count);
static void run_control_action(const MenuControlAction *action);
static bool is_key_pressed(uint32_t pin);
static void update_back_ok_key(MenuKeyState *key, bool pressed,
    uint32_t nowMs);
static void copy_command(char *dest, size_t destSize, const char *source);

static const char *const gRootItems[] = {
    "File List",
    "Vehicle Status",
    "Vehicle Control",
};

static const MenuControlAction gControlActions[] = {
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
static char gMessage[32];
static MenuPage gRenderedPage = MENU_PAGE_IMAGE_VIEW;

void app_menu_init(uint32_t nowMs)
{
    gPage = MENU_PAGE_ROOT;
    gReturnPage = MENU_PAGE_ROOT;
    gRootSelected = 0U;
    gFileSelected = 0U;
    gControlSelected = 0U;
    gLastStatusRefreshMs = nowMs;
    gBackOkKey.pressed = false;
    gBackOkKey.longSent = false;
    gBackOkKey.pressStartMs = 0U;
    gRenderedPage = MENU_PAGE_IMAGE_VIEW;
    gMessage[0] = '\0';
    gDirty = true;
    render_menu(nowMs);
}

void app_menu_update(uint32_t nowMs)
{
    bool upPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_UP_PIN);
    bool downPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_DOWN_PIN);
    bool backOkPressed = is_key_pressed(GPIO_MENU_KEYS_KEY_BACK_OK_PIN);
    static bool previousUpPressed;
    static bool previousDownPressed;

    if (upPressed && !previousUpPressed) {
        handle_up();
    }
    if (downPressed && !previousDownPressed) {
        handle_down();
    }
    previousUpPressed = upPressed;
    previousDownPressed = downPressed;

    update_back_ok_key(&gBackOkKey, backOkPressed, nowMs);

    if ((gPage == MENU_PAGE_STATUS) &&
        ((uint32_t) (nowMs - gLastStatusRefreshMs) >=
            MENU_STATUS_REFRESH_MS)) {
        gLastStatusRefreshMs = nowMs;
        gDirty = true;
    }

    if (gDirty) {
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
        gDirty = false;
        return;
    }

    if (gPage == MENU_PAGE_ROOT) {
        render_root();
    } else if (gPage == MENU_PAGE_FILES) {
        render_files();
    } else if (gPage == MENU_PAGE_STATUS) {
        render_status();
    } else {
        render_control();
    }

    gRenderedPage = gPage;
    gDirty = false;
}

static void render_root(void)
{
    draw_header("CAR MENU");
    for (uint8_t i = 0U; i < (sizeof(gRootItems) / sizeof(gRootItems[0]));
         i++) {
        draw_line(i, gRootItems[i], i == gRootSelected);
    }
    draw_footer("UP/DOWN  HOLD=OK");
}

static void render_files(void)
{
    uint8_t count = app_image_store_get_file_count();
    uint8_t first = 0U;
    char line[40];

    draw_header("FILES");

    if (count == 0U) {
        draw_line(0U, "No images", false);
        draw_line(1U, "Send with upper_pc", false);
        draw_footer("BACK");
        return;
    }

    if (gFileSelected >= count) {
        gFileSelected = count - 1U;
    }
    if (gFileSelected >= MENU_VISIBLE_ROWS) {
        first = gFileSelected - MENU_VISIBLE_ROWS + 1U;
    }

    for (uint8_t row = 0U; row < MENU_VISIBLE_ROWS; row++) {
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
    }

    draw_footer("BACK  HOLD=SHOW");
}

static void render_status(void)
{
    EncoderCounts counts = encoder_get_counts();
    char line[40];

    if (gRenderedPage != MENU_PAGE_STATUS) {
        draw_header("STATUS");
    }
    snprintf(line, sizeof(line), "L:%ld R:%ld", (long) counts.left_count,
        (long) counts.right_count);
    draw_line(0U, line, false);
    snprintf(line, sizeof(line), "Mode:%ld Busy:%d",
        (long) motion_control_get_mode(), motion_control_is_busy() ? 1 : 0);
    draw_line(1U, line, false);
    snprintf(line, sizeof(line), "MM/s:%ld Deg/s:%ld",
        (long) motion_control_get_target_mm_s(),
        (long) motion_control_get_target_deg_s());
    draw_line(2U, line, false);
    draw_line(3U, "PWM L:PA8 R:PB4", false);
    draw_line(4U, "LCD SCLK:PB9", false);
    draw_footer("BACK");
}

static void render_control(void)
{
    uint8_t count =
        (uint8_t) (sizeof(gControlActions) / sizeof(gControlActions[0]));
    uint8_t first = 0U;

    draw_header("CONTROL");
    if (gControlSelected >= count) {
        gControlSelected = count - 1U;
    }
    if (gControlSelected >= MENU_VISIBLE_ROWS) {
        first = gControlSelected - MENU_VISIBLE_ROWS + 1U;
    }
    for (uint8_t row = 0U; row < MENU_VISIBLE_ROWS; row++) {
        uint8_t index = first + row;

        if (index < count) {
            draw_line(row, gControlActions[index].label,
                index == gControlSelected);
        } else {
            draw_line(row, "", false);
        }
    }
    draw_footer((gMessage[0] != '\0') ? gMessage : "BACK  HOLD=RUN");
}

static void draw_header(const char *title)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, BLACK);
    LCD_Fill(0U, 0U, LCD_W, 26U, DARKBLUE);
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

static void update_back_ok_key(MenuKeyState *key, bool pressed,
    uint32_t nowMs)
{
    if ((key == NULL) || (pressed == key->pressed)) {
        if (pressed && !key->longSent &&
            ((uint32_t) (nowMs - key->pressStartMs) >=
                MENU_LONG_PRESS_MS)) {
            key->longSent = true;
            handle_confirm();
        }
        return;
    }

    if (pressed) {
        key->pressed = true;
        key->longSent = false;
        key->pressStartMs = nowMs;
        return;
    }

    if (!key->longSent) {
        handle_back();
    }
    key->pressed = false;
}

static void copy_command(char *dest, size_t destSize, const char *source)
{
    if ((dest == NULL) || (destSize == 0U)) {
        return;
    }

    strncpy(dest, source, destSize - 1U);
    dest[destSize - 1U] = '\0';
}
