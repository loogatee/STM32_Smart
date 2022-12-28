

#include <stddef.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Ticker.h"
#include "Utils.h"
#include "Uart.h"
#include "Cmds.h"


static void InitHW_SetPins(void);
static void InitHW_Usart1(void);

RCC_ClocksTypeDef  rclocks;

uint32_t  dbg_counter;


int main(void)
{

    uint32_t  Ntime = 0;

    dbg_counter = 0;

    Ticker_Init();                                          // SW construct only, does not touch any HW or registers
    UartOut_Init();                                         // SW construct only, no HW
    UartIn_Init();                                          // SW construct only, no HW
    CMDS_Init();

    SystemCoreClockUpdate();                                // Update SystemCoreClock variable according to Clock Register Values.
    RCC_GetClocksFreq(&rclocks);                            // fills in HCLK_Frequency
    SysTick_Config(rclocks.HCLK_Frequency / 1000 );         // This provides for a 1ms ticker.  Verified with scope!

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   // To enable output LED on C13
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // To enable output Pin on B5, used for scope probe
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);  // to enable USART pins A9 (Tx) and A10 (Rx)

    InitHW_SetPins();     // hits HW registers.  Sets up C13,B5,A9,A10
    InitHW_Usart1();      // hits HW.  Sets up the Usart1 registers.

    CMDS_DisplayVersion();
    Uart_PrintSTR("\r\n>> ");

    while (1)
    {
        ++dbg_counter;

        UartOut_Process();
        UartIn_Process();
        CMDS_Process();

        if (Ticker_GetDelta(Ntime) >= 500 )                 // When ticker delta hits 500ms
        {
            Ntime       = Ticker_Get();                     // new base for the delta compare
            dbg_counter = 0;
            CMDS_LedBlinks();
        }
    }
}


static void InitHW_SetPins(void)
{
    GPIO_InitTypeDef   LED_C13;
    GPIO_InitTypeDef   Pin_B5;
    GPIO_InitTypeDef   TX_A9;
    GPIO_InitTypeDef   RX_A10;

    LED_C13.GPIO_Pin   = GPIO_Pin_13;                       // RobotDyn HW has a LED on C13
    LED_C13.GPIO_Mode  = GPIO_Mode_Out_PP;
    LED_C13.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &LED_C13);

    Pin_B5.GPIO_Pin   = GPIO_Pin_5;                         // Scope probe on Pin B5
    Pin_B5.GPIO_Mode  = GPIO_Mode_Out_PP;
    Pin_B5.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &Pin_B5);

    TX_A9.GPIO_Pin   = GPIO_Pin_9;                          // Usart1 TX on Pin A9
    TX_A9.GPIO_Mode  = GPIO_Mode_AF_PP;
    TX_A9.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &TX_A9);

    RX_A10.GPIO_Pin   = GPIO_Pin_10;                        // Usart1 RX on Pin A10
    RX_A10.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    RX_A10.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &RX_A10);
}


static void InitHW_Usart1(void)
{
    USART_InitTypeDef  U1;

    U1.USART_BaudRate            = 9600;
    U1.USART_WordLength          = USART_WordLength_8b;
    U1.USART_StopBits            = USART_StopBits_1;
    U1.USART_Parity              = USART_Parity_No;
    U1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    U1.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &U1);
    USART_Cmd(USART1, ENABLE);
}


//
// The following stubs needed to keep the linker from squawking.
//
void _close(void)
{
}

void _fstat(void)
{
}

void _isatty(void)
{
}

void _lseek(void)
{
}

void _read(void)
{
}











