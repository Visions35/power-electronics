
#include <math.h>
#include "ControlFunction.h"

void calcPidReg(pPID_REG v)
{
	v->Err = v->Ref - v->Fdb;

	v->PidKpOut = v->Kp * v->Err;

	v->PidKiOut += v->Ki * v->Err * v->Ts;

	if (v->PidKiOut > v->LimitHigh)
	{
		v->PidKiOut = v->LimitHigh;
	}
	else if (v->PidKiOut < v->LimitLow)
	{
		v->PidKiOut = v->LimitLow;
	}

	v->PidOut = v->PidKpOut + v->PidKiOut;

	if (v->PidOut > v->LimitHigh)
	{
		v->PidOut = v->LimitHigh;
	}
	else if (v->PidOut < v->LimitLow)
	{
		v->PidOut = v->LimitLow;
	}
}

void restartPidReg(pPID_REG v)
{
	v->PidKiOut = 0;
	v->PidKpOut = 0;
	v->PidOut = 0;
}

void calcRmp(pRMP_CTR v)
{
	v->Err = v->Tartget - v->outputValue;

	if (v->Err > v->MinErr)
	{
		v->outputValue += v->Step;
	}
	else if (v->Err < (-1.0*v->MinErr))
	{
		v->outputValue -= v->Step;
	}
	else
	{
		v->outputValue = v->outputValue;
	}
}

void restartRmp(pRMP_CTR v)
{
	v->outputValue = 0;
}
