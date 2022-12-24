#ifndef CMDS_H
#define CMDS_H

void CMDS_Init(void);
void CMDS_SetInputStr(char *StrInp);
void CMDS_Process(void);
bool CMDS_DisplayVersion(void);
void CMDS_GetBlinks(uint8_t *B1, uint8_t *B2);

#endif

