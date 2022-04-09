#include "main.h"

#define   HS_Gate_Drive_Control_REG	0xb8

void DRV8305_Init(void);
void Clear_DRV8305_Fault(void);
void Read_DRV8305_REG(void);
void Phase_Align(void);
void Phase1_AB(void);
void Phase2_CB(void);
void Phase3_CA(void);
void Phase4_BA(void);
void Phase5_BC(void);
void Phase6_AC(void);
void Phase_Stop(void);


