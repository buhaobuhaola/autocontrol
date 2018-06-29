#ifndef _MYPID_H
#define _MYPID_H


typedef struct
{
	double  SetPoint;                                 //设定目标 Desired Value
  double  Kp;                               				//比例常数 Proportional Const
  double  Ki;                                 			//积分常数 Integral Const
  double  Kd;                               				//微分常数 Derivative Const
	double	UpLimits;                                	//PID输出上限	
	double	DownLimits;                               //PID输出下限
	double  ControlAccuracy;													//控制精度
	double 	SampleTime;                               //采样时间
	int		ControllerDirection;               					//方向，负数方向，0或正数正向
	
	double	A;																				//Error参数       不设置
	double	B;																				//Error[-1]参数   不设置
	double	C;																				//Error[-2]参数   不设置
  double  LastError;                                //Error[-1]       不设置
  double  PrevError;                                //Error[-2]       不设置
} IncPID_TypeDef;
//初始化PID控制器，初始化前需正确配置mypid参数
extern unsigned char IncPID_Init(IncPID_TypeDef *myPID);
//PID参数设置，主要用于运行过程设置，PID控制初始化过程可不调用
extern unsigned char IncPID_SetPID(IncPID_TypeDef *myPID,double kp,double ki,double kd);
//放置定时器中调用，定时器时间与SampleTime值一致
extern double  IncPID_Compute(IncPID_TypeDef *myPID,double NextPoint);

#endif

