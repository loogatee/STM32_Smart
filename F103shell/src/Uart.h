
#ifndef _UART_H_
#define _UART_H_


#define dev_USART1    1
#define dev_USART2    2
#define dev_USART6    6


#define SERIAL_CONSOLE     dev_USART1




#define SERO_TYPE_ONECHAR    0x01
#define SERO_TYPE_STR        0x02
#define SERO_TYPE_32         0x03
#define SERO_TYPE_32N        0x04
#define SERO_TYPE_8          0x05
#define SERO_TYPE_8N         0x06


#define ASCII_BACKSPACE         8
#define ASCII_LINEFEED          10
#define ASCII_CARRIAGERETURN    13
#define ASCII_SPACE             32
#define ASCII_TILDE             126
#define ASCII_DELETE            127





// Uart output
void UartOut_Init(void);
void UartOut_Process(void);

void Uart_PrintCH (char ch);
void Uart_PrintSTR(const char *pstr);
void Uart_Print32 (const char *pstr, uint32_t val);
void Uart_Print32N(const char *pstr, uint32_t val);
void Uart_Print8  (const char *pstr, uint8_t  val);
void Uart_Print8N (const char *pstr, uint8_t  val);
void Uart_Send    (uint32_t otype, char *sptr, uint32_t *completionptr, uint32_t aval);

// Uart input
void UartIn_Init(void);
void UartIn_Process(void);
void UartIn_SignalCmdDone(void);




#endif
