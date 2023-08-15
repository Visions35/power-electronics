
#ifndef _CONTROL_FUNCTION_H_
#define _CONTROL_FUNCTION_H_

#define CON_TS 1e-4

typedef struct PID_REG_X
{
	float Kp;
	float Ki;
	float Ts;

	float Ref;
	float Fdb;

	float LimitHigh;
	float LimitLow;

	float Err;

	float PidKpOut;
	float PidKiOut;

	float PidOut;
}PID_REG, *pPID_REG;

void calcPidReg(pPID_REG v);
void restartPidReg(pPID_REG v);

typedef struct RMP_CTR_X
{
	float Err;
	float MinErr;
	float Step;

	float Tartget;
	float outputValue;
}RMP_CTR, *pRMP_CTR;

void calcRmp(pRMP_CTR v);
void restartRmp(pRMP_CTR v);

#endif


