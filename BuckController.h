

#ifndef _BUCK_CONTROLLER_H_
#define _BUCK_CONTROLLER_H_

typedef struct BUCK_CONTROLLER_X
{
	float IL_in;
	float Uc_in;
	
	float vol_Target;
	float cur_Target;
	
	float PwmRefOut;

	PID_REG BuckUcPid;
	PID_REG BuckILPid;
	
	RMP_CTR BuckRmp;

}BUCK_CONTROLLER, *pBUCK_CONTROLLER;

void initBuckController(pBUCK_CONTROLLER pBuckCon);
void calcBuckController(pBUCK_CONTROLLER pBuckCon);

void initDspInterrupt(void);
void calcDspInterrupt(double *Target, double *UcIn, double *ILIn, double *PwmRef, double *Display);

extern BUCK_CONTROLLER BuckConA;

#endif