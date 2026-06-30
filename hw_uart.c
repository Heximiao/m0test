#include "hw_uart.h"
#include "ti_msp_dl_config.h"

#define UART_TX_BUFFER_SIZE (512U)
#define UART_RX_BUFFER_SIZE (128U)

static uint8_t gUartTxBuffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t gUartTxHead;
static volatile uint16_t gUartTxTail;
static volatile uint32_t gUartTxDroppedBytes;
static uint8_t gUartRxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t gUartRxHead;
static volatile uint16_t gUartRxTail;
static volatile uint32_t gUartRxDroppedBytes;

static bool queue_tx_byte(uint8_t byte);
static bool read_rx_byte(uint8_t *byte);
static void queue_rx_byte(uint8_t byte);

void uart_debug_init(void)
{
    DL_UART_Main_clearInterruptStatus(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enableInterrupt(UART_DEBUG_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);
}

void uart_debug_service_tx(void)
{
    while (gUartTxTail != gUartTxHead) {
        uint8_t byte = gUartTxBuffer[gUartTxTail];
        if (!DL_UART_transmitDataCheck(UART_DEBUG_INST, byte)) {
            return;
        }
        gUartTxTail = (uint16_t) ((gUartTxTail + 1U) % UART_TX_BUFFER_SIZE);
    }
}

void uart_debug_write_string(const char *text)
{
    while (*text != '\0') {
        if (!queue_tx_byte((uint8_t) *text)) {
            return;
        }
        text++;
    }
}

bool uart_debug_read_line(char *line, size_t lineSize)
{
    uint8_t rxByte;
    static char rxLine[UART_DEBUG_LINE_BUFFER_SIZE];
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
        } else if (rxIndex < (UART_DEBUG_LINE_BUFFER_SIZE - 1U)) {
            rxLine[rxIndex] = (char) rxByte;
            rxIndex++;
        } else {
            rxIndex = 0U;
            uart_debug_write_string("ERR RX_OVERFLOW\r\n");
        }
    }

    return false;
}

void uart_debug_handle_irq(void)
{
    uint8_t rxByte;

    switch (DL_UART_Main_getPendingInterrupt(UART_DEBUG_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_receiveDataCheck(UART_DEBUG_INST, &rxByte)) {
                queue_rx_byte(rxByte);
            }
            break;
        default:
            break;
    }
}

uint32_t uart_debug_get_rx_dropped_bytes(void)
{
    return gUartRxDroppedBytes;
}

uint32_t uart_debug_get_tx_dropped_bytes(void)
{
    return gUartTxDroppedBytes;
}

static bool queue_tx_byte(uint8_t byte)
{
    uint16_t nextHead = (uint16_t) ((gUartTxHead + 1U) % UART_TX_BUFFER_SIZE);
    if (nextHead == gUartTxTail) {
        gUartTxDroppedBytes++;
        return false;
    }

    gUartTxBuffer[gUartTxHead] = byte;
    gUartTxHead = nextHead;
    return true;
}

static bool read_rx_byte(uint8_t *byte)
{
    if (gUartRxTail == gUartRxHead) {
        return false;
    }

    *byte = gUartRxBuffer[gUartRxTail];
    gUartRxTail = (uint16_t) ((gUartRxTail + 1U) % UART_RX_BUFFER_SIZE);
    return true;
}

static void queue_rx_byte(uint8_t byte)
{
    uint16_t nextHead = (uint16_t) ((gUartRxHead + 1U) % UART_RX_BUFFER_SIZE);
    if (nextHead == gUartRxTail) {
        gUartRxDroppedBytes++;
        return;
    }

    gUartRxBuffer[gUartRxHead] = byte;
    gUartRxHead = nextHead;
}
