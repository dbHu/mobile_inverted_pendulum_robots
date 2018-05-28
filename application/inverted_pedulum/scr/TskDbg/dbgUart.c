#include "embARC.h"
#include "embARC_debug.h"

#include "dbgUart.h"
#include "Queue/Queue.h"

const int dbgTskPrio = 2;
const int dbgTskStkSize = 512;

QueueHandle_t dbgMbCmd;
Queue  QUartRx;

char msg[128];

static bool InitFlag = 0;
static DEV_UART *dbg_uart;
static uint8_t RxBuff;
static DEV_BUFFER UartRxBuff;
static DEV_BUFFER UartTxBuff;

int putStr(const char *format, ...)
{
    BaseType_t rtn;

    va_list arg;
    va_start(arg, format);

    // ARMCC microlib does not properly handle a size of 0.
    // As a workaround supply a dummy buffer with a size of 1.
    char dummy_buf[1];
    int len = vsnprintf(dummy_buf, sizeof(dummy_buf), format, arg);

    if (len < 128) {
        vsprintf(msg, format, arg);
        rtn=xQueueSend(dbgMbCmd, msg, portMAX_DELAY);
        configASSERT(rtn==pdPASS);
    } else {
        vsprintf(msg, "error\r\n", arg);
        rtn=xQueueSend(dbgMbCmd, msg, portMAX_DELAY);
        configASSERT(rtn==pdPASS);
    }

    va_end(arg);
    return len;
}

static void UartRxISR(void *ptr)
{
    int len;
    char c;
    dbg_uart->uart_control(UART_CMD_SET_RXINT, (void *)0);

    /** read uart char & enqueue */
    dbg_uart->uart_read((void *)&c, 1);
    q_en_char(&QUartRx, c);

    dbg_uart->uart_control(UART_CMD_SET_RXINT, (void *)1);  
}

void UartGetLine(char *line)
{
    
    char c;
    bool b;
    if(line == NULL)
        return;

    while(1)
    {
        /** string dequeue, end with '\r' */
        do{
            b = q_de_char(&QUartRx, &c);
            vTaskDelay(5);
        }while(!b);
        if((*line++ = c) == '\r')
            break;
    }
    /** aad '\0' to the end of string, contribute to strcmp*/
    *(line-1) = '\0';
    return;
}

int UartWrite(char *str)
{
    unsigned int uIdx,len;

    len = strlen(str);

    return (int)dbg_uart->uart_write((const void *)(str), len); 
    //DEV_BUFFER_INIT(&UartTxBuff, str, len);
    //dbg_uart->uart_control(UART_CMD_SET_TXINT_BUF, (void *)&UartTxBuff);
    //dbg_uart->uart_control(UART_CMD_SET_TXINT, (void *)1);

}

void UartInit(void)
{

    if (InitFlag) {
        return;
    }
    
    dbg_uart = uart_get_dev(DW_UART_0_ID);
    dbg_uart->uart_open(BOARD_CONSOLE_UART_BAUD);

    dbg_uart->uart_control(UART_CMD_SET_RXINT_BUF, (void *)NULL);
    dbg_uart->uart_control(UART_CMD_SET_RXCB, (void *)&UartRxISR);
    dbg_uart->uart_control(UART_CMD_SET_RXINT, (void *)1);

    putStr("DBG:%s, %s\r\n", __DATE__, __TIME__);
    #if defined(__GNU__)
    putStr("GNU,%s\r\n", __VERSION__);
    #else
    putStr("Metaware,%s\r\n", __VERSION__);
    #endif

    InitFlag = 1;
}

void dbgTask(void *pvParameters)
{
    BaseType_t rtn;
    while(true){
        
        if(xQueueReceive(dbgMbCmd, msg, ( TickType_t ) 0)){
            UartWrite(msg);
        }
    }
}

void dbgInit()
{
	BaseType_t rtn;

    q_init_char(&QUartRx, 128);

    dbgMbCmd = xQueueCreate(10, 128);
    configASSERT(dbgMbCmd);
    /*
     *  Initialize UART port 0
     */
    UartInit();

    // Create tasks
    rtn=xTaskCreate(dbgTask, "dbgTask", dbgTskStkSize, NULL, dbgTskPrio, NULL);
    configASSERT(rtn==pdPASS);
}