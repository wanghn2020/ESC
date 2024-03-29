#include "main.h"

#define   HS_Gate_Drive_Control_REG	0xb8

void DRV8305_Init(void);
void Clear_DRV8305_Fault(void);
void Read_DRV8305_REG(uint8_t Addr);
void Phase_Align(void);
void Phase1_AB(void);
void Phase12_AB_CB(void);
void Phase2_CB(void);
void Phase23_CB_CA(void);
void Phase3_CA(void);
void Phase34_CA_BA(void);
void Phase4_BA(void);
void Phase45_BC_AC(void);
void Phase5_BC(void);
void Phase56_AC_AB(void);
void Phase6_AC(void);
void Phase7_AB(void);
void Phase_Stop(void);

void Delay(__IO uint32_t nCount);
