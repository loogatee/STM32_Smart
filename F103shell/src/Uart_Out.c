
#include <stddef.h>
#include <string.h>
#include "stm32f10x.h"
#include "Uart.h"
#include "Utils.h"






#define SERO_STATE_GETJOB      0
#define SERO_STATE_DOCHARS     1
#define SERO_SQENTRYS          30

#define LEN_PRINTF_BUF         1024


typedef struct
{                                       // Items on the Serial Job Queue
    uint32_t    sr_otype;               // Output Type.  See SERD_OTYPE_XX
    char       *sr_sptr;                // pointer to string to be printed
    uint32_t   *sr_compPtr;             // if != 0, sets to 1 on completion
    uint32_t    sr_dval;                // data value as 8-bit or 32-bit
} SERI_t;




                                                    // OUTPUT related global variables:
static uint32_t    serd_num_Qitems;                 //   Number of Items on the Queue
static uint32_t    serd_inn_Qindex;                 //   Item will be deposited here
static uint32_t    serd_out_Qindex;                 //   Item will be removed from here
static SERI_t     *serd_active_Qitem;               //   pointer to currently active item
static SERI_t      serd_Q_items[SERO_SQENTRYS];     //   Serial Job Data Items
static uint32_t    serd_ostate_machine;             //   holds state of Serial Output machine
static char        serd_databuf[16];                //   data buffer for value conversion
static uint8_t   serd_pfbuf[LEN_PRINTF_BUF];
static uint32_t  serd_pfindex;






void UartOut_Init( void )
{
    serd_num_Qitems     = 0;
    serd_inn_Qindex     = 0;
    serd_out_Qindex     = 0;
    serd_ostate_machine = SERO_STATE_GETJOB;
//  serd_pfindex        = 0;
}


//---------------------------------------------------------------------------------------
//  Queues up a Job on the 'Serial Jobs Output List' (Implemented as a Circular Q)
//  This is the 'Front End' or Producer of Serial Data.   The idea is to stash the
//  parameters quickly in a Queue, then let the 'Back End' (the Consumer) process the
//  data off the Queue, and perform the printing.
//  The data is destined for USART2.
//     **Note the use of void for return. Alternative is return -1 when Q is full.
//       Consider using completionptr to let the Q drain, then keep printing.
//
//
//        otype - The type of output requested.  see SERO_TYPE_ defines
//
//        sptr  - Pointer to string to be printed.
//
//        completionptr - IF 0, then there is no completion signal.   IF
//                        non-zero, this address will be signaled with a 1 
//                        to indicate request has been completed.
//
//        aval - Value that can optionally be printed along with sptr.
//               Value can be 8-bit, or 32-bit.   otype will control this.
//
void Uart_Send( uint32_t otype, char *sptr, uint32_t *completionptr, uint32_t aval )
{
    SERI_t  *lqitem;                                                            // Pointer to Array Element where the data will go

    if( serd_num_Qitems != SERO_SQENTRYS )                                      // Proceed if the Queue is not full
    {
        lqitem = &serd_Q_items[serd_inn_Qindex];                                // item will go in at index serd_inn_qindex
        ++serd_num_Qitems;                                                      // Queue size is increased by 1
        if( ++serd_inn_Qindex == SERO_SQENTRYS ) { serd_inn_Qindex = 0; }       // where the next item will go:  wrap if necessary

        lqitem->sr_dval    = aval;                                              // aval goes into element on the Q
        lqitem->sr_otype   = otype;                                             // otype goes into element on the Q
        lqitem->sr_sptr    = sptr;                                              // sptr goes into element on the Q
        lqitem->sr_compPtr = completionptr;                                     // completionptr goes into element on the Q
        
        if( completionptr ) { *completionptr = 0; }                             // if pointer is valid, store 0, indicating Not Done
    }
                                                                                // Else the Q is full.  Effectively tosses the data
}                                                                               //   note intention not to return -1 if Q is full






void UartOut_Process( void )
{
    char  end_of_string;

    switch( serd_ostate_machine )
    {
    case SERO_STATE_GETJOB:                                                              // Looking for items on the Queue

        if( serd_num_Qitems == 0 ) { return; }                                           // return IMMEDIATELY if Queue is empty

        serd_active_Qitem   = &serd_Q_items[serd_out_Qindex];                            // Item to operate on is at 'serd_out_qindex'
        serd_ostate_machine = SERO_STATE_DOCHARS;                                        // switch state to 'Processing Characters'

        switch( serd_active_Qitem->sr_otype )                                            // sr_otype dictates the action
        {
          case SERO_TYPE_ONECHAR: serd_databuf[0] = (uint8_t)serd_active_Qitem->sr_dval; // Character to print is placed in databuf
                                  serd_databuf[1] = 0;                                   // string terminator
                                  serd_active_Qitem->sr_sptr = serd_databuf;             // sptr point to the data in databuf
                                  break;
                                    
          case SERO_TYPE_32:      ItoH(serd_active_Qitem->sr_dval, serd_databuf);        // 32-bit to Hex Data Conversion.  Place in databuf
                                  serd_databuf[8]  = ASCII_CARRIAGERETURN;               // tack on <CR> at the end
                                  serd_databuf[9]  = ASCII_LINEFEED;                     // tack on <LF> at the end
                                  serd_databuf[10] = 0;                                  // this is the string terminator
                                  break;
                                  
          case SERO_TYPE_32N:     ItoH(serd_active_Qitem->sr_dval, serd_databuf);        // 8-bit to Hex Data Conversion.  Place in databuf
                                  break;
                                    
          case SERO_TYPE_8:       BtoH((uint8_t)serd_active_Qitem->sr_dval, serd_databuf); // 8-bit to Hex Data Conversion.  Place in databuf
                                  serd_databuf[2] = ASCII_CARRIAGERETURN;                  // tack on <CR> at the end
                                  serd_databuf[3] = ASCII_LINEFEED;                        // tack on <LF> at the end
                                  serd_databuf[4] = 0;                                     // this is the string terminator
                                  break;
                                  
          case SERO_TYPE_8N:      BtoH((uint8_t)serd_active_Qitem->sr_dval, serd_databuf); // 8-bit to Hex Data Conversion.  Place in databuf
                                  break;                          
        }
        
        return;                                                                          // could FALL_THRU, but don't be a CPU hog!


    case SERO_STATE_DOCHARS:                                                             // Actively printing out characters


        if( !(USART1->SR & USART_FLAG_TC) ) { return; }
        USART1->DR     = *serd_active_Qitem->sr_sptr++;


        end_of_string  = *serd_active_Qitem->sr_sptr;                               // Examine character just past the one printed

        if( end_of_string == 0 )                                                    // Found the string terminator ?
        {
            if( serd_active_Qitem->sr_otype >= SERO_TYPE_32 )                       // Any of these types: need to print out the val in databuf
            {                                                                       //    [ ordering in SERD_OTYPE is important!! ]
                serd_active_Qitem->sr_otype = SERO_TYPE_STR;                        // change type to STR
                serd_active_Qitem->sr_sptr  = (char *)serd_databuf;                 // data is in serd_databuf
            }
            else                                                                    // ELSE this print job is done
            {
                serd_ostate_machine = SERO_STATE_GETJOB;                            // switch state:  look for another job
                --serd_num_Qitems;                                                  // Can now decrement Queue size by 1
                if( ++serd_out_Qindex == SERO_SQENTRYS ) { serd_out_Qindex = 0; }   // index to next element in the Circular Q.  Wrap if necessary

                if( serd_active_Qitem->sr_compPtr != 0 )                            // Is there a valid Completion Pointer ?
                    *serd_active_Qitem->sr_compPtr = 1;                             //    signal a 1 to that address to indicate completion
            }
        }
    }
}









void Uart_PrintCH( char ch )
{
    Uart_Send( SERO_TYPE_ONECHAR, 0, 0, (uint32_t)ch );
}

void Uart_PrintSTR( const char *pstr )
{
    Uart_Send( SERO_TYPE_STR, (char *)pstr, 0, 0 );
}

void Uart_Print32( const char *pstr, uint32_t val )
{
    Uart_Send( SERO_TYPE_32, (char *)pstr, 0, val );
}

void Uart_Print32N( const char *pstr, uint32_t val )
{
    Uart_Send( SERO_TYPE_32N, (char *)pstr, 0, val );
}

void Uart_Print8( const char *pstr, uint8_t val )
{
    Uart_Send( SERO_TYPE_8, (char *)pstr, 0, (uint32_t)val );
}

void Uart_Print8N( const char *pstr, uint8_t val )
{
    Uart_Send( SERO_TYPE_8N, (char *)pstr, 0, (uint32_t)val );
}


int _write( void *fp, char *buf, uint32_t len )
{
    uint32_t  remaining = (LEN_PRINTF_BUF - serd_pfindex);

    if (remaining < (len+2)) { serd_pfindex=0; }                    // if it won't fit, wrap

    strncpy((void *)&serd_pfbuf[serd_pfindex], buf, len);
    Uart_PrintSTR((const char *)&serd_pfbuf[serd_pfindex]);

    serd_pfindex               += len;
    serd_pfbuf[serd_pfindex++]  = 0;

    if (serd_pfindex >= (LEN_PRINTF_BUF - 6)) { serd_pfindex=0; }   // wrap if its close to the end

    return len;
}










