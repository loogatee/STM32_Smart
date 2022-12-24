
#include <stddef.h>
#include "stm32f10x.h"


static uint32_t    TickerCount;



void Ticker_Init(void)
{
    TickerCount = 0;
}


uint32_t Ticker_Get( void )
{
    return TickerCount;
}

uint32_t Ticker_GetDelta( uint32_t OriginalTime )
{
    uint32_t v = TickerCount;

    if( v >= OriginalTime )
        return( v - OriginalTime );
    else
        return( ~OriginalTime + 1 + v );

}

// See SysTick_Handler(void) in file stm32f1xx_it.c
void Ticker_Interrupt(void)
{
    // GPIOB->ODR ^= GPIO_Pin_5;       // uncomment if using scope probe
    ++TickerCount;
}








