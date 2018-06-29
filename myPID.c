#include "myPID.h"

unsigned char IncPID_Init(IncPID_TypeDef *myPID) 
{
	if(myPID->Kp<0 || myPID->Ki<0 || myPID->Kd<0 || myPID->SampleTime<=0 ) return 0;
	if(myPID->UpLimits<=myPID->DownLimits)  return 0;
	myPID->Ki=myPID->Ki * myPID->SampleTime;
	myPID->Kd=myPID->Kd / myPID->SampleTime;
	if(myPID->ControllerDirection<0)
	{
		myPID->Kp=-myPID->Kp;
		myPID->Ki=-myPID->Ki;
		myPID->Kd=-myPID->Kd;
	}
	myPID->LastError=0;
	myPID->PrevError=0;
	myPID->A=myPID->Kp+myPID->Ki+myPID->Kd;
	myPID->B=2*myPID->Kd+myPID->Kp;
	myPID->C=myPID->Kd;
	return 1;
}

unsigned char IncPID_SetPID(IncPID_TypeDef *myPID,double kp,double ki,double kd) 
{
	if(myPID->Kp<0 || myPID->Ki<0 || myPID->Kd<0) return 0;
	myPID->Kp=kp;
	myPID->Ki=ki * myPID->SampleTime;
	myPID->Kd=kd / myPID->SampleTime;
	if(myPID->ControllerDirection<0)
	{
		myPID->Kp=-myPID->Kp;
		myPID->Ki=-myPID->Ki;
		myPID->Kd=-myPID->Kd;
	}
	myPID->A=myPID->Kp+myPID->Ki+myPID->Kd;
	myPID->B=-(2*myPID->Kd+myPID->Kp);
	myPID->C=myPID->Kd;
	return 1;
}

double  IncPID_Compute(IncPID_TypeDef *myPID,double NextPoint)
{
  double Error;
	double output=0;
  Error = myPID->SetPoint - NextPoint;
	if(myPID->ControlAccuracy>0)
	{
		if((Error<=myPID->ControlAccuracy )&& (Error >= -myPID->ControlAccuracy))
		Error=0;	
	}
	output=(myPID->A * Error )+ (myPID->B * myPID->LastError) +(myPID->C * myPID->PrevError);
	if(output> myPID->UpLimits)  output=myPID->UpLimits;
	else if(output< myPID->DownLimits) output=myPID->DownLimits;
	myPID->PrevError=myPID->LastError;                  
  myPID->LastError=Error;
	return output;
}

