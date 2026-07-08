#include "hw_lcd.h"
#include "hw_spi.h"
#include "lcdfont.h"
#include "ti/driverlib/m0p/dl_core.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define delay_ms(X) delay_cycles((80000 * (X)))

#define LCD_QUEUE_SIZE (48U)
#define LCD_TEXT_MAX (39U)
#define LCD_SERVICE_BYTE_BUDGET (512U)

typedef enum {
    LCD_TASK_FILL = 0,
    LCD_TASK_STRING,
} LcdTaskType;

typedef struct {
    LcdTaskType type;
    uint16_t x;
    uint16_t y;
    uint16_t x2;
    uint16_t y2;
    uint16_t fc;
    uint16_t bc;
    uint8_t sizey;
    uint8_t mode;
    char text[LCD_TEXT_MAX + 1U];
} LcdTask;

typedef struct {
    LcdTask task;
    bool active;
    bool addressSet;
    uint32_t pixelsRemaining;
    uint8_t textIndex;
    uint16_t charX;
    uint8_t charWidth;
    uint16_t charPixelsSent;
    uint16_t glyphByteIndex;
    uint8_t glyphBitIndex;
    uint8_t glyphByteCount;
    unsigned char currentChar;
} LcdDrawState;

static LcdTask gQueue[LCD_QUEUE_SIZE];
static volatile uint8_t gQueueHead;
static volatile uint8_t gQueueTail;
static LcdDrawState gDraw;

static void lcd_write_bus_sync(unsigned char dat);
static void lcd_write_reg_sync(unsigned char dat);
static void lcd_write_data8_sync(unsigned char dat);
static void lcd_write_data16_sync(unsigned int dat);
static void lcd_address_set_sync(unsigned int x1, unsigned int y1,
    unsigned int x2, unsigned int y2);
static bool lcd_enqueue(const LcdTask *task);
static bool lcd_dequeue(LcdTask *task);
static bool lcd_service_fill(uint16_t *budgetBytes);
static bool lcd_service_string(uint16_t *budgetBytes);
static bool lcd_start_next_char(void);
static uint8_t lcd_glyph_byte(unsigned char ch, uint8_t sizey,
    uint16_t glyphByteIndex);
static uint8_t lcd_glyph_byte_count(uint8_t sizey);
static uint8_t lcd_char_width(uint8_t sizey);
static void lcd_write_pixel_budgeted(uint16_t color, uint16_t *budgetBytes);

static void lcd_write_bus_sync(unsigned char dat)
{
    LCD_CS_Clr();
    spi_write_bus(dat);
    LCD_CS_Set();
}

void LCD_WR_DATA8(unsigned char dat)
{
    lcd_write_bus_sync(dat);
}

void LCD_WR_DATA(unsigned int dat)
{
    lcd_write_data16_sync(dat);
}

static void lcd_write_data8_sync(unsigned char dat)
{
    lcd_write_bus_sync(dat);
}

static void lcd_write_data16_sync(unsigned int dat)
{
    lcd_write_bus_sync((unsigned char) (dat >> 8));
    lcd_write_bus_sync((unsigned char) dat);
}

static void lcd_write_reg_sync(unsigned char dat)
{
    LCD_DC_Clr();
    lcd_write_bus_sync(dat);
    LCD_DC_Set();
}

void LCD_WR_REG(unsigned char dat)
{
    lcd_write_reg_sync(dat);
}

void LCD_Address_Set(unsigned int x1, unsigned int y1, unsigned int x2,
    unsigned int y2)
{
    lcd_address_set_sync(x1, y1, x2, y2);
}

static void lcd_address_set_sync(unsigned int x1, unsigned int y1,
    unsigned int x2, unsigned int y2)
{
    lcd_write_reg_sync(0x2a);
    lcd_write_data16_sync(x1);
    lcd_write_data16_sync(x2);
    lcd_write_reg_sync(0x2b);
    lcd_write_data16_sync(y1 + 35U);
    lcd_write_data16_sync(y2 + 35U);
    lcd_write_reg_sync(0x2c);
}

void lcd_init(void)
{
    gQueueHead = 0U;
    gQueueTail = 0U;
    memset(&gDraw, 0, sizeof(gDraw));

    LCD_RES_Clr();
    delay_ms(30);
    LCD_RES_Set();
    delay_ms(100);

    lcd_write_reg_sync(0x11);

    lcd_write_reg_sync(0x36);
    lcd_write_data8_sync(0x70);

    lcd_write_reg_sync(0x3A);
    lcd_write_data8_sync(0x05);

    lcd_write_reg_sync(0xB2);
    lcd_write_data8_sync(0x0C);
    lcd_write_data8_sync(0x0C);
    lcd_write_data8_sync(0x00);
    lcd_write_data8_sync(0x33);
    lcd_write_data8_sync(0x33);

    lcd_write_reg_sync(0xB7);
    lcd_write_data8_sync(0x35);

    lcd_write_reg_sync(0xBB);
    lcd_write_data8_sync(0x1A);

    lcd_write_reg_sync(0xC0);
    lcd_write_data8_sync(0x2C);

    lcd_write_reg_sync(0xC2);
    lcd_write_data8_sync(0x01);

    lcd_write_reg_sync(0xC3);
    lcd_write_data8_sync(0x0B);

    lcd_write_reg_sync(0xC4);
    lcd_write_data8_sync(0x20);

    lcd_write_reg_sync(0xC6);
    lcd_write_data8_sync(0x0F);

    lcd_write_reg_sync(0xD0);
    lcd_write_data8_sync(0xA4);
    lcd_write_data8_sync(0xA1);

    lcd_write_reg_sync(0x21);
    lcd_write_reg_sync(0xE0);
    lcd_write_data8_sync(0xF0);
    lcd_write_data8_sync(0x00);
    lcd_write_data8_sync(0x04);
    lcd_write_data8_sync(0x04);
    lcd_write_data8_sync(0x04);
    lcd_write_data8_sync(0x05);
    lcd_write_data8_sync(0x29);
    lcd_write_data8_sync(0x33);
    lcd_write_data8_sync(0x3E);
    lcd_write_data8_sync(0x38);
    lcd_write_data8_sync(0x12);
    lcd_write_data8_sync(0x12);
    lcd_write_data8_sync(0x28);
    lcd_write_data8_sync(0x30);

    lcd_write_reg_sync(0xE1);
    lcd_write_data8_sync(0xF0);
    lcd_write_data8_sync(0x07);
    lcd_write_data8_sync(0x0A);
    lcd_write_data8_sync(0x0D);
    lcd_write_data8_sync(0x0B);
    lcd_write_data8_sync(0x07);
    lcd_write_data8_sync(0x28);
    lcd_write_data8_sync(0x33);
    lcd_write_data8_sync(0x3E);
    lcd_write_data8_sync(0x36);
    lcd_write_data8_sync(0x14);
    lcd_write_data8_sync(0x14);
    lcd_write_data8_sync(0x29);
    lcd_write_data8_sync(0x32);

    lcd_write_reg_sync(0x11);
    delay_ms(120);
    lcd_write_reg_sync(0x29);
}

void lcd_service(void)
{
    uint16_t budgetBytes = LCD_SERVICE_BYTE_BUDGET;

    while (budgetBytes > 0U) {
        if (!gDraw.active) {
            if (!lcd_dequeue(&gDraw.task)) {
                return;
            }
            gDraw.active = true;
            gDraw.addressSet = false;
            gDraw.pixelsRemaining = 0U;
            gDraw.textIndex = 0U;
            gDraw.charX = gDraw.task.x;
            gDraw.charPixelsSent = 0U;
            gDraw.glyphByteIndex = 0U;
            gDraw.glyphBitIndex = 0U;
            gDraw.currentChar = '\0';
        }

        if (gDraw.task.type == LCD_TASK_FILL) {
            if (!lcd_service_fill(&budgetBytes)) {
                return;
            }
        } else {
            if (!lcd_service_string(&budgetBytes)) {
                return;
            }
        }
    }
}

void LCD_Fill(unsigned int xsta, unsigned int ysta, unsigned int xend,
    unsigned int yend, unsigned int color)
{
    LcdTask task;

    if ((xsta >= xend) || (ysta >= yend)) {
        return;
    }

    memset(&task, 0, sizeof(task));
    task.type = LCD_TASK_FILL;
    task.x = (uint16_t) xsta;
    task.y = (uint16_t) ysta;
    task.x2 = (uint16_t) (xend - 1U);
    task.y2 = (uint16_t) (yend - 1U);
    task.fc = (uint16_t) color;
    (void) lcd_enqueue(&task);
}

void LCD_ShowString(unsigned int x, unsigned int y, const unsigned char *p,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    LcdTask task;

    if ((p == NULL) || (*p == '\0')) {
        return;
    }

    memset(&task, 0, sizeof(task));
    task.type = LCD_TASK_STRING;
    task.x = (uint16_t) x;
    task.y = (uint16_t) y;
    task.fc = (uint16_t) fc;
    task.bc = (uint16_t) bc;
    task.sizey = sizey;
    task.mode = mode;
    strncpy(task.text, (const char *) p, LCD_TEXT_MAX);
    task.text[LCD_TEXT_MAX] = '\0';
    (void) lcd_enqueue(&task);
}

void LCD_ShowChar(unsigned int x, unsigned int y, unsigned char num,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    unsigned char text[2];

    text[0] = num;
    text[1] = '\0';
    LCD_ShowString(x, y, text, fc, bc, sizey, mode);
}

static bool lcd_enqueue(const LcdTask *task)
{
    uint8_t next = (uint8_t) ((gQueueHead + 1U) % LCD_QUEUE_SIZE);

    if (next == gQueueTail) {
        return false;
    }

    gQueue[gQueueHead] = *task;
    gQueueHead = next;
    return true;
}

static bool lcd_dequeue(LcdTask *task)
{
    if (gQueueTail == gQueueHead) {
        return false;
    }

    *task = gQueue[gQueueTail];
    gQueueTail = (uint8_t) ((gQueueTail + 1U) % LCD_QUEUE_SIZE);
    return true;
}

static bool lcd_service_fill(uint16_t *budgetBytes)
{
    if (!gDraw.addressSet) {
        uint32_t width = (uint32_t) gDraw.task.x2 - gDraw.task.x + 1U;
        uint32_t height = (uint32_t) gDraw.task.y2 - gDraw.task.y + 1U;

        lcd_address_set_sync(gDraw.task.x, gDraw.task.y, gDraw.task.x2,
            gDraw.task.y2);
        gDraw.pixelsRemaining = width * height;
        gDraw.addressSet = true;
    }

    while ((gDraw.pixelsRemaining > 0U) && (*budgetBytes >= 2U)) {
        lcd_write_pixel_budgeted(gDraw.task.fc, budgetBytes);
        gDraw.pixelsRemaining--;
    }

    if (gDraw.pixelsRemaining == 0U) {
        gDraw.active = false;
        return true;
    }

    return false;
}

static bool lcd_service_string(uint16_t *budgetBytes)
{
    while (*budgetBytes >= 2U) {
        if (gDraw.currentChar == '\0') {
            if (!lcd_start_next_char()) {
                gDraw.active = false;
                return true;
            }
        }

        uint8_t glyph = lcd_glyph_byte(gDraw.currentChar, gDraw.task.sizey,
            gDraw.glyphByteIndex);
        uint16_t color =
            ((glyph & (uint8_t) (0x01U << gDraw.glyphBitIndex)) != 0U) ?
            gDraw.task.fc : gDraw.task.bc;

        lcd_write_pixel_budgeted(color, budgetBytes);
        gDraw.charPixelsSent++;

        if ((gDraw.charPixelsSent % gDraw.charWidth) == 0U) {
            gDraw.glyphByteIndex++;
            gDraw.glyphBitIndex = 0U;
        } else {
            gDraw.glyphBitIndex++;
            if (gDraw.glyphBitIndex >= 8U) {
                gDraw.glyphBitIndex = 0U;
                gDraw.glyphByteIndex++;
            }
        }

        if (gDraw.glyphByteIndex >= gDraw.glyphByteCount) {
            gDraw.charX = (uint16_t) (gDraw.charX + gDraw.charWidth);
            gDraw.currentChar = '\0';
        }
    }

    return false;
}

static bool lcd_start_next_char(void)
{
    unsigned char ch = (unsigned char) gDraw.task.text[gDraw.textIndex];

    if (ch == '\0') {
        return false;
    }

    if ((ch < ' ') || (ch > '~')) {
        ch = ' ';
    }

    gDraw.currentChar = (unsigned char) (ch - ' ');
    gDraw.charWidth = lcd_char_width(gDraw.task.sizey);
    gDraw.glyphByteCount = lcd_glyph_byte_count(gDraw.task.sizey);
    gDraw.charPixelsSent = 0U;
    gDraw.glyphByteIndex = 0U;
    gDraw.glyphBitIndex = 0U;
    lcd_address_set_sync(gDraw.charX, gDraw.task.y,
        (uint16_t) (gDraw.charX + gDraw.charWidth - 1U),
        (uint16_t) (gDraw.task.y + gDraw.task.sizey - 1U));
    gDraw.textIndex++;
    return true;
}

static uint8_t lcd_glyph_byte(unsigned char ch, uint8_t sizey,
    uint16_t glyphByteIndex)
{
    if (sizey == 12U) {
        return ascii_1206[ch][glyphByteIndex];
    }
    if (sizey == 16U) {
        return ascii_1608[ch][glyphByteIndex];
    }
    if (sizey == 24U) {
        return ascii_2412[ch][glyphByteIndex];
    }
    if (sizey == 32U) {
        return ascii_3216[ch][glyphByteIndex];
    }

    return 0U;
}

static uint8_t lcd_glyph_byte_count(uint8_t sizey)
{
    uint8_t sizex = lcd_char_width(sizey);

    return (uint8_t) (((sizex / 8U) + ((sizex % 8U) ? 1U : 0U)) * sizey);
}

static uint8_t lcd_char_width(uint8_t sizey)
{
    if ((sizey == 12U) || (sizey == 16U) ||
        (sizey == 24U) || (sizey == 32U)) {
        return (uint8_t) (sizey / 2U);
    }

    return 8U;
}

static void lcd_write_pixel_budgeted(uint16_t color, uint16_t *budgetBytes)
{
    lcd_write_data16_sync(color);
    *budgetBytes = (uint16_t) (*budgetBytes - 2U);
}

void LCD_DrawPoint(unsigned int x, unsigned int y, unsigned int color)
{
    LCD_Fill(x, y, x + 1U, y + 1U, color);
}

void LCD_DrawLine(unsigned int x1, unsigned int y1, unsigned int x2,
    unsigned int y2, unsigned int color)
{
    int xerr = 0;
    int yerr = 0;
    int delta_x = (int) x2 - (int) x1;
    int delta_y = (int) y2 - (int) y1;
    int incx;
    int incy;
    int distance;
    int uRow = (int) x1;
    int uCol = (int) y1;

    if (delta_x > 0) {
        incx = 1;
    } else if (delta_x == 0) {
        incx = 0;
    } else {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0) {
        incy = 1;
    } else if (delta_y == 0) {
        incy = 0;
    } else {
        incy = -1;
        delta_y = -delta_y;
    }

    distance = (delta_x > delta_y) ? delta_x : delta_y;
    for (int t = 0; t < distance + 1; t++) {
        LCD_DrawPoint((unsigned int) uRow, (unsigned int) uCol, color);
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

void LCD_DrawRectangle(unsigned int x1, unsigned int y1, unsigned int x2,
    unsigned int y2, unsigned int color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

void LCD_DrawVerrticalLine(int x, int y1, int y2, unsigned int color)
{
    LCD_Fill((unsigned int) x, (unsigned int) y1, (unsigned int) (x + 1),
        (unsigned int) (y1 + y2 + 1), color);
}

void Draw_Circle(unsigned int x0, unsigned int y0, unsigned char r,
    unsigned int color)
{
    int a = 0;
    int b = r;

    while (a <= b) {
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 + b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 + b, color);
        LCD_DrawPoint(x0 - a, y0 - b, color);
        LCD_DrawPoint(x0 + b, y0 + a, color);
        LCD_DrawPoint(x0 + a, y0 - b, color);
        LCD_DrawPoint(x0 + a, y0 + b, color);
        LCD_DrawPoint(x0 - b, y0 + a, color);
        a++;
        if (((a * a) + (b * b)) > ((int) r * (int) r)) {
            b--;
        }
    }
}

void LCD_ArcRect(unsigned int xsta, unsigned int ysta, unsigned int xend,
    unsigned int yend, unsigned int color)
{
    LCD_DrawRectangle(xsta, ysta, xend, yend, color);
}

void LCD_ShowChinese(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    LCD_ShowString(x, y, s, fc, bc, sizey, mode);
}

void LCD_ShowChinese12x12(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    LCD_ShowString(x, y, s, fc, bc, sizey, mode);
}

void LCD_ShowChinese16x16(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    LCD_ShowString(x, y, s, fc, bc, sizey, mode);
}

void LCD_ShowChinese24x24(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    LCD_ShowString(x, y, s, fc, bc, sizey, mode);
}

void LCD_ShowChinese32x32(unsigned int x, unsigned int y, unsigned char *s,
    unsigned int fc, unsigned int bc, unsigned char sizey, unsigned char mode)
{
    LCD_ShowString(x, y, s, fc, bc, sizey, mode);
}

unsigned int mypow(unsigned char m, unsigned char n)
{
    unsigned int result = 1U;

    while (n--) {
        result *= m;
    }
    return result;
}

void LCD_ShowIntNum(unsigned int x, unsigned int y, unsigned int num,
    unsigned char len, unsigned int fc, unsigned int bc, unsigned char sizey)
{
    char text[12];

    (void) len;
    snprintf(text, sizeof(text), "%u", num);
    LCD_ShowString(x, y, (const unsigned char *) text, fc, bc, sizey, 0U);
}

void LCD_ShowFloatNum1(unsigned int x, unsigned int y, float num,
    unsigned char len, unsigned int fc, unsigned int bc, unsigned char sizey)
{
    char text[16];

    (void) len;
    snprintf(text, sizeof(text), "%ld", (long) (num * 100.0f));
    LCD_ShowString(x, y, (const unsigned char *) text, fc, bc, sizey, 0U);
}

void LCD_ShowPicture(unsigned int x, unsigned int y, unsigned int length,
    unsigned int width, const unsigned char pic[])
{
    uint32_t pixels = (uint32_t) length * (uint32_t) width;
    uint32_t k = 0U;

    lcd_address_set_sync(x, y, x + length - 1U, y + width - 1U);
    while (pixels-- > 0U) {
        lcd_write_data8_sync(pic[k++]);
        lcd_write_data8_sync(pic[k++]);
    }
}
