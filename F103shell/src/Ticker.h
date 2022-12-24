
#ifndef TICKER_H_
#define TICKER_H_


void     Ticker_Init(void);
uint32_t Ticker_Get(void);
uint32_t Ticker_GetDelta(uint32_t OriginalTime);
void     Ticker_Interrupt(void);


#endif /* TICKER_H_ */
