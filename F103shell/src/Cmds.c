#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_rcc.h"
#include "Utils.h"
#include "Uart.h"
#include "Cmds.h"
#include "Ticker.h"

typedef struct
{
    uint8_t  RTC_Hours;
    uint8_t  RTC_Minutes;
    uint8_t  RTC_Seconds;
    uint8_t  RTC_Date;
    uint8_t  RTC_Wday;
    uint8_t  RTC_Month;
    uint16_t RTC_Year;
} RTC_DateTimeTypeDef;


// (UnixTime = 00:00:00 01.01.1970 = JD0 = 2440588)
#define JULIAN_DATE_BASE    2440588




#ifdef DEBUGBUILD
#define VERSION_STR     "STM32_Smart Cortex-M3 TestBed Version D1_"
#else
#define VERSION_STR     "STM32_Smart Cortex-M3 TestBed Version R1_"
#endif
#define VERSION_MAJOR   1
#define VERSION_MINOR   2
#define VERSION_DATE    "12/28/2022 13:44\r\n"


#define  SAFE_MEM_ADDR           &cmds_freebuf[0]


typedef enum
{
    CMDSM_WAITFORLINE = 0,
    CMDSM_MD,
    CMDSM_ST
} Commands_t;


typedef enum
{
    DO_INIT = 0,
    DO_PROCESS,
    DO_PROCESSa,
} SubCommands_t;

// extern void init_gpioI2C( void );


static const char *gWeeks[7] =
{
   (const char *)"Mon  ",     // 0
   (const char *)"Tue  ",     // 1
   (const char *)"Wed  ",
   (const char *)"Thu  ",
   (const char *)"Fri  ",
   (const char *)"Sat  ",     // 5
   (const char *)"Sun  "      // 6
};

static uint32_t    cmds_freebuf[2];
static bool        cmds_input_ready;
static char       *cmds_InpPtr;
static uint32_t    cmds_completion;
static uint32_t    cmds_MDaddr;
static uint32_t    cmds_count1;
static uint32_t    cmds_Wtime;
static uint8_t     cmds_blink1;
static uint8_t     cmds_blink2;
static uint8_t     cmds_Kbl1;
static uint8_t     cmds_Kbl2;
static uint8_t     cmds_TA[8];

static RTC_DateTimeTypeDef  cmds_DTrtc;
static Commands_t           cmds_state_machine;
static SubCommands_t        cmds_substate;

static uint32_t dbgcount1;



static bool cmds_MD ( void );
static bool cmds_R  ( void );
static bool cmds_T  ( void );
static bool cmds_BL ( void );
static bool cmds_CL ( void );
static bool cmds_LB ( void );
static bool cmds_ST ( void );
static bool cmds_RTC( void );


extern   RCC_ClocksTypeDef  rclocks;


void CMDS_Init(void)
{
    cmds_input_ready   = FALSE;
    cmds_state_machine = CMDSM_WAITFORLINE;
    cmds_MDaddr        = (uint32_t)0x20000000;     // start of ram STM32F103xx

    cmds_blink1 = cmds_Kbl1  = '2';
    cmds_blink2 = cmds_Kbl2  = '0';

    *((uint32_t *)SAFE_MEM_ADDR) = 0xBC00BC99;
}



void CMDS_Process(void)
{
    bool  signal_done = TRUE;
    char  *S=cmds_InpPtr;
    
    switch( cmds_state_machine )
    {
    case CMDSM_WAITFORLINE:
        
        if( cmds_input_ready == FALSE ) { return; }
        cmds_input_ready = FALSE;
        cmds_substate    = DO_INIT;
        
        if( S[0] == 'b' && S[1] == 'l')                          signal_done = cmds_BL();
        else if( S[0] == 'c' && S[1] == 'l')                     signal_done = cmds_CL();
        else if( S[0] == 'l' && S[1] == 'b')                     signal_done = cmds_LB();
        else if( S[0] == 'm' && S[1] == 'd')                     signal_done = cmds_MD();
        else if( S[0] == 's' && S[1] == 't')                     signal_done = cmds_ST();
        else if( S[0] == 'r' && S[1] == 't' && S[2] == 'c')      signal_done = cmds_RTC();
        else if( S[0] == 'r' )                                   signal_done = cmds_R();
        else if( S[0] == 't' )                                   signal_done = cmds_T();
        else if( S[0] == 'v' )                                   signal_done = CMDS_DisplayVersion();
        break;
        
    case CMDSM_MD:     signal_done = cmds_MD();    break;
    case CMDSM_ST:     signal_done = cmds_ST();    break;
    }

    if( signal_done == TRUE ) { UartIn_SignalCmdDone(); }
}








bool CMDS_DisplayVersion(void)
{
    Uart_PrintSTR("\r\n");
    Uart_Print8N (VERSION_STR, VERSION_MINOR);
    Uart_PrintSTR(", ");
    Uart_PrintSTR(VERSION_DATE);
    
    return TRUE;
}


void CMDS_SetInputStr(char *StrInp)
{
    cmds_InpPtr      = StrInp;
    cmds_input_ready = TRUE;
}


static bool cmds_BL ( void )
{
    size_t  slen = strlen(cmds_InpPtr);
    char    c2   = cmds_InpPtr[2];
    char    c4   = cmds_InpPtr[4];

    if(slen == 2)
    {
        Uart_PrintSTR( "BL[12] 0|1|2    (0=Off,1=On,2=Blink)");
    }
    else if( slen >= 5 )
    {
        switch(c2)
        {
        case '1': if( c4 == '0' || c4 == '1' || c4 == '2' ) { cmds_blink1 = c4; } break;
        case '2': if( c4 == '0' || c4 == '1' || c4 == '2' ) { cmds_blink2 = c4; } break;
        }
    }
    return TRUE;
}


/**
 * Called from main when the 500ms ticker expires
 *
 */
void CMDS_LedBlinks(void)
{
    if ((cmds_blink1 != cmds_Kbl1) || (cmds_blink1 == '2'))
    {
        cmds_Kbl1 = cmds_blink1;
        switch(cmds_blink1)
        {
          case '0':    GPIOC->ODR |=  GPIO_Pin_13;   break;    // off
          case '1':    GPIOC->ODR &= ~GPIO_Pin_13;   break;    // on
          case '2':    GPIOC->ODR ^=  GPIO_Pin_13;   break;    // blink
        }
    }

    if ((cmds_blink2 != cmds_Kbl2) || (cmds_blink2 == '2'))
    {
        cmds_Kbl2 = cmds_blink2;
        switch(cmds_blink2)
        {
          case '0':    GPIOB->ODR &= ~GPIO_Pin_5;   break;    // off
          case '1':    GPIOB->ODR |=  GPIO_Pin_5;   break;    // on
          case '2':    GPIOB->ODR ^=  GPIO_Pin_5;   break;    // blink
        }
    }
}






static bool cmds_LB ( void )
{
    return TRUE;
}

//
//
static bool cmds_CL ( void )
{
    size_t  slen = strlen(cmds_InpPtr);

    if(slen == 2)
    {
        Uart_PrintSTR( "placeholder");
    }
    return TRUE;
}





static bool cmds_R( void )
{
    uint32_t  tmpw;

    if( cmds_InpPtr[1] == 'm' )
    {
        tmpw = HtoI(&cmds_InpPtr[3]) & 0xFFFFFFFC;                              // bits 0 and 1 forced to 0
        Uart_Print32N( "0x", tmpw );
        Uart_Print32( ": ", (uint32_t)*((uint32_t *)tmpw) );
    }
    else if( cmds_InpPtr[1] == 'p' )
    {
        ;
    }
    else if( cmds_InpPtr[1] == 'c' )
    {
        Uart_PrintSTR("nada 2\n\r");
    }

    
    return TRUE;
}


static bool cmds_T( void )
{
    int  tmpI;
    uint32_t  tmp32;



    if( cmds_InpPtr[1] == '1' )
    {
        tmpI = AtoI("452");
        Uart_Print32( "452: 0x", (uint32_t)tmpI );
        
        tmpI = AtoI("-4392");
        if( tmpI == -4392 )
            Uart_PrintSTR( "-4392 Good\r\n" );
        else
            Uart_PrintSTR( "Conversion did not yield -4392\r\n" );
    }
    else if( cmds_InpPtr[1] == '2' )
    {
        tmp32 = (uint32_t)*((uint32_t *)SAFE_MEM_ADDR);
        Uart_Print32( "SAFE_MEM_ADDR  = ", (uint32_t)SAFE_MEM_ADDR);
        Uart_Print32( "*SAFE_MEM_ADDR = ", tmp32 );
    }
    else if( cmds_InpPtr[1] == '3' )
    {
        Uart_Print32("SYSCLK_Frequency: ", rclocks.SYSCLK_Frequency);     // 72,000,000
        Uart_Print32("HCLK_Frequency:   ", rclocks.HCLK_Frequency);       // 72,000,000
        Uart_Print32("PCLK1_Frequency:  ", rclocks.PCLK1_Frequency);      // 36,000,000
        Uart_Print32("PCLK2_Frequency:  ", rclocks.PCLK2_Frequency);      // 72,000,000
        Uart_Print32("ADCCLK_Frequency: ", rclocks.ADCCLK_Frequency);     // 36,000,000

    }
    else if( cmds_InpPtr[1] == '4' )
    {
        Uart_Print32( "ticker:    ", Ticker_Get() );
        Uart_Print32( "dbgcount1: ", dbgcount1);
    }
    
    return TRUE;
}


static bool cmds_MD(void)
{
    uint32_t  i;
    bool retv = FALSE;

    switch( cmds_substate )
    {
    case DO_INIT:

        if( strlen(cmds_InpPtr) > 2 ) { cmds_MDaddr = HtoI(&cmds_InpPtr[3]) & 0xFFFFFFFC; }
        cmds_count1        = 0;
        cmds_state_machine = CMDSM_MD;
        cmds_substate      = DO_PROCESS;
        cmds_completion    = 1;
        /* no break */

    case DO_PROCESS:

        if( cmds_completion == 1 )
        {
            Uart_Print32N( "0x", cmds_MDaddr );
            Uart_Print8N( ": ", (uint8_t)*((uint8_t *)cmds_MDaddr++) );

            for( i=0; i < 15; i++ )
            {
                Uart_Print8N( " ", (uint8_t)*((uint8_t *)cmds_MDaddr++) );
            }

            Uart_Send( SERO_TYPE_STR, (char *)"\r\n", &cmds_completion, 0 );

            if( ++cmds_count1 == 4 )
            {
                cmds_state_machine = CMDSM_WAITFORLINE;
                ItoH( cmds_MDaddr, &cmds_InpPtr[2] );
                retv = TRUE;
            }
        }
        break;

    case DO_PROCESSa:    // fall thru
    default:             break;
    }

    return retv;
}











// Magic Sauce from:   https://github.com/avislab/STM32F103/blob/master/Example_RTC/main.c
static uint32_t RTC_GetRTC_Counter(RTC_DateTimeTypeDef* RTC_DateTimeStruct)
{
    uint8_t  a,m;
    uint16_t y;
    uint32_t JDN;

    a = (14-RTC_DateTimeStruct->RTC_Month) / 12;
    y = RTC_DateTimeStruct->RTC_Year + 4800 - a;
    m = RTC_DateTimeStruct->RTC_Month + (12*a) - 3;

    JDN  = RTC_DateTimeStruct->RTC_Date;
    JDN += (153*m+2) / 5;
    JDN += 365 * y;
    JDN += y / 4;
    JDN += -y / 100;
    JDN += y / 400;
    JDN  = JDN - 32045;
    JDN  = JDN - JULIAN_DATE_BASE;
    JDN *= 86400;
    JDN += (RTC_DateTimeStruct->RTC_Hours*3600);
    JDN += (RTC_DateTimeStruct->RTC_Minutes*60);
    JDN += (RTC_DateTimeStruct->RTC_Seconds);

    return JDN;
}





// Set Time (Internal RTC of the STM32F103 chip)
//
//  012345678901234567890
//  st 38 12 03 31 01 17
//
static bool cmds_ST(void)
{
    uint8_t  i,k;
    bool     rc=FALSE;

    switch (cmds_substate)
    {
    case DO_INIT:

        if (strlen(cmds_InpPtr) == 2)
        {
            Uart_PrintSTR( "st mins hrs wkday day mon yr      (Mon=00)\r\n" );
            rc = TRUE;
            break;
        }

        for (i=3,k=0; i < 20; i += 3)
        {
            cmds_InpPtr[i+2] = 0;
            cmds_TA[k++]     = (uint8_t)AtoI(&cmds_InpPtr[i]);
        }

        cmds_DTrtc.RTC_Hours   = cmds_TA[1];
        cmds_DTrtc.RTC_Minutes = cmds_TA[0];
        cmds_DTrtc.RTC_Seconds = 0;

        cmds_DTrtc.RTC_Date    = cmds_TA[3];
        cmds_DTrtc.RTC_Month   = cmds_TA[4];
        cmds_DTrtc.RTC_Wday    = cmds_TA[2];
        cmds_DTrtc.RTC_Year    = cmds_TA[5] + 2000;

        PWR_BackupAccessCmd(ENABLE);
        RCC_BackupResetCmd(ENABLE);
        RCC_BackupResetCmd(DISABLE);
        RCC_LSEConfig(RCC_LSE_ON);
  
        cmds_state_machine = CMDSM_ST;
        cmds_substate      = DO_PROCESSa;
        dbgcount1          = 0;
        break;

    case DO_PROCESSa:

        if ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY)
        {
        	dbgcount1++;
        	goto OUT;
        }

        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        RTC_SetPrescaler(0x7FFF);
        RCC_RTCCLKCmd(ENABLE);

        cmds_count1   = RTC_GetRTC_Counter(&cmds_DTrtc);
        cmds_Wtime    = Ticker_Get();
        cmds_substate = DO_PROCESS;
        break;

    case DO_PROCESS:

        if (Ticker_GetDelta(cmds_Wtime) < 200) { goto OUT; }

        RTC_SetCounter(cmds_count1);
        cmds_state_machine = CMDSM_WAITFORLINE;
        rc = TRUE;
        // fall thru
    }

OUT:
    return rc;
}






//
//     Internal RTC of the STM32F103 chip
//
static bool cmds_RTC( void )
{
    uint8_t    Secs,Mins,Hours,Wday,Day,Mon;
    uint16_t   Yr;
    uint32_t   Count,time,t1,a,b,c,d,e,m;
    uint64_t   jd,jdn;

    Count = RTC_GetCounter();

    time = Count;  t1 = (time / 60);   Secs  = time - (t1*60);
    time = t1;     t1 = (time / 60);   Mins  = time - (t1*60);
    time = t1;     t1 = (time / 24);   Hours = time - (t1*24);

    // Magic Sauce from:   https://github.com/avislab/STM32F103/blob/master/Example_RTC/main.c
    // routine:            void RTC_GetDateTime()
    jd   = ((Count+43200)/(86400>>1)) + (2440587<<1) + 1;
    jdn  = jd >> 1;
    Wday = jdn % 7;
    a    = jdn + 32044;
    b    = (4*a+3)/146097;
    c    = a - (146097*b)/4;
    d    = (4*c+3)/1461;
    e    = c - (1461*d)/4;
    m    = (5*e+2)/153;
    Day  = e - (153*m+2)/5 + 1;
    Mon  = m + 3 - 12*(m/10);
    Yr   = 100*b + d - 4800 + (m/10);

    printf( "%s %02d/%02d/%d  %02d:%02d:%02d\n\r", gWeeks[Wday], Mon, Day, Yr, Hours, Mins, Secs);

    return TRUE;
}


//===================================================================================================

void CMDS_GetBlinks(uint8_t *B1, uint8_t *B2)
{
    *B1 = cmds_blink1;
    *B2 = cmds_blink2;
}





































