

#include <math.h>
#include "ControlFunction.h"
#include "BuckController.h"

BUCK_CONTROLLER BuckConA;


void initBuckController(pBUCK_CONTROLLER pBuckCon)
{
	pBuckCon->BuckUcPid.Kp = 5;
	pBuckCon->BuckUcPid.Ki = 0.01;
	pBuckCon->BuckUcPid.Ts = CON_TS;

	pBuckCon->BuckUcPid.LimitHigh = 30;
	pBuckCon->BuckUcPid.LimitLow = 0.0;

	pBuckCon->BuckILPid.Kp = 5;
	pBuckCon->BuckILPid.Ki = 0.01;
	pBuckCon->BuckILPid.Ts = CON_TS;

	pBuckCon->BuckILPid.LimitHigh = 1.0;
	pBuckCon->BuckILPid.LimitLow = 0.0;

	pBuckCon->BuckRmp.MinErr = 0.2;
	pBuckCon->BuckRmp.Step = 0.02;

}


void calcBuckController(pBUCK_CONTROLLER pBuckCon)
{
	pBuckCon->BuckRmp.Tartget = pBuckCon->vol_Target;//目标值给谐波补偿
	calcRmp(&pBuckCon->BuckRmp);


//谐波补偿计算输出值给pid参考值，反馈为电压输入
	pBuckCon->BuckUcPid.Ref = pBuckCon->BuckRmp.outputValue;
	pBuckCon->BuckUcPid.Fdb = pBuckCon->Uc_in;
//计算pwm输出值
	calcPidReg(&pBuckCon->BuckUcPid);

	pBuckCon->BuckILPid.Ref = pBuckCon->BuckUcPid.PidOut;
	pBuckCon->BuckILPid.Fdb = pBuckCon->IL_in;
	calcPidReg(&pBuckCon->BuckILPid);

	pBuckCon->PwmRefOut = pBuckCon->BuckILPid.PidOut;

}

void initDspInterrupt(void)
{
	initBuckController(&BuckConA);
}

void calcDspInterrupt(double *Target, double *UcIn, double *ILIn, double *PwmRef, double *Display)
{
	BuckConA.vol_Target = *Target;
	BuckConA.Uc_in = *UcIn;
	BuckConA.IL_in = *ILIn;

	calcBuckController(&BuckConA);

	*PwmRef = BuckConA.PwmRefOut;

	Display[0] = BuckConA.vol_Target;
	Display[1] = BuckConA.BuckUcPid.Ref;
	Display[2] = BuckConA.BuckUcPid.Fdb;
	Display[3] = BuckConA.BuckILPid.Ref;
	Display[4] = BuckConA.BuckILPid.Fdb;
	Display[5] = BuckConA.BuckUcPid.PidOut;
	Display[6] = BuckConA.BuckILPid.PidOut;
}