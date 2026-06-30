#include "hw_openmv_uart.h"
#include "ti_msp_dl_config.h"

#define UART_OPENMV_RX_BUFFER_SIZE (128U)

static uint8_t gOpenmvRxBuffer[UART_OPENMV_RX_BUFFER_SIZE];
static volatile uint16_t gOpenmvRxHead;
static volatile uint16_t gOpenmvRxTail;
static volatile uint32_t gOpenmvRxDroppedBytes;

static bool read_rx_byte(uint8_t *byte);
static void queue_rx_byte(uint8_t byte);

void uart_openmv_init(void)
{
    DL_UART_Main_clearInterruptStatus(UART_OPENMV_INST,
        DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_OPENMV_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_OPENMV_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_OPENMV_INST_INT_IRQN);
}

bool uart_openmv_read_line(char *line, size_t lineSize)
{
    uint8_t rxByte;
    static char rxLine[UART_OPENMV_LINE_BUFFER_SIZE];
    static uint32_t rxIndex;

    while (read_rx_byte(&rxByte)) {
        if ((rxByte == '\r') || (rxByte == '\n')) {
            if (rxIndex > 0U) {
                size_t copyLength = rxIndex;
                if (copyLength >= lineSize) {
                    copyLength = lineSize - 1U;
                }
                for (size_t i = 0U; i < copyLength; i++) {
                    line[i] = rxLine[i];
                }
                line[copyLength] = '\0';
                rxIndex = 0U;
                return true;
            }
        } else if (rxIndex < (UART_OPENMV_LINE_BUFFER_SIZE - 1U)) {
            rxLine[rxIndex] = (char) rxByte;
            rxIndex++;
        } else {
            rxIndex = 0U;
            gOpenmvRxDroppedBytes++;
        }
    }

    return false;
}

void uart_openmv_handle_irq(void)
{
    uint8_t rxByte;

    switch (DL_UART_Main_getPendingInterrupt(UART_OPENMV_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_receiveDataCheck(UART_OPENMV_INST, &rxByte)) {
                queue_rx_byte(rxByte);
            }
            break;
        default:
            break;
    }
}

uint32_t uart_openmv_get_rx_dropped_bytes(void)
{
    return gOpenmvRxDroppedBytes;
}

static bool read_rx_byte(uint8_t *byte)
{
    if (gOpenmvRxTail == gOpenmvRxHead) {
        return false;
    }

    *byte = gOpenmvRxBuffer[gOpenmvRxTail];
    gOpenmvRxTail =
        (uint16_t) ((gOpenmvRxTail + 1U) % UART_OPENMV_RX_BUFFER_SIZE);
    return true;
}

static void queue_rx_byte(uint8_t byte)
{
    uint16_t nextHead =
        (uint16_t) ((gOpenmvRxHead + 1U) % UART_OPENMV_RX_BUFFER_SIZE);
    if (nextHead == gOpenmvRxTail) {
        gOpenmvRxDroppedBytes++;
        return;
    }

    gOpenmvRxBuffer[gOpenmvRxHead] = byte;
    gOpenmvRxHead = nextHead;
}
