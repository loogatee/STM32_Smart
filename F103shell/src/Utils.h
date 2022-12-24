#ifndef _UTILS_H_
#define _UTILS_H_

typedef unsigned char   bool;

#define true   1
#define TRUE   1
#define false  0
#define FALSE  0



void     BtoH  (uint8_t  val, char *S);
void     BtoHnz(uint8_t  val, char *S);
void     ItoH  (uint32_t val, char *S);
uint32_t HtoI  (const char *ptr);
int      AtoI  (const char *p);
uint16_t HtoU16(char *pstr);

//void     hammer( u8 nparm );


#endif
