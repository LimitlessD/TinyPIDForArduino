//#include "stdio.h"

/**************************PID******************************/
typedef struct 
{
	float TargetValue;  //目标值(一个应该由传感器测得的值)
	float CurrentValue;  //当前值(由传感器获得的当前值)
    float ErrNow;  //目前偏差(用于计算比例)
	float Last_Err;  //上一次误差(用于计算积分)
    float integral;  //积分值(误差的积分)
	float Kp,Ki,Kd;  //比例常数，积分常数，微分常数
	float OutputValue;  //输出值
} PID_TypeDef;

void PID_Init();//初始化PID各项参数
float PID_operation(float value);//执行PID调试


void PID_Init(PID_TypeDef PID)
{
	PID.TargetValue = 0.0;
	PID.CurrentValue = 0.0;
	PID.ErrNow = 0.0;
	PID.Last_Err = 0.0;
    PID.integral = 0.0;
    PID.Kp = 0.0;  //比例常数
	PID.Ki = 0.0;  //积分常数
	PID.Kd = 0.0;  //微分常数
    //常数的调整非常重要，这里的参数是经过几次试验得出的经验参数
	PID.OutputValue = 0.0;
}


float PID_Calculate(PID_TypeDef PID, float TargetValue)
{
	PID.TargetValue = TargetValue;  //确定目标值
    //PID.CurrentValue = GetCurrentValue(unsigned char SensorId);
	PID.ErrNow = PID.TargetValue - PID.CurrentValue;  //计算偏差量
	PID.integral += PID.ErrNow;  //计算从初始化到现在误差的和****************************************************//
	PID.OutputValue =   PID.Kp*PID.ErrNow + //直接通过误差*某个比例达到线性调参
                        PID.Ki*PID.integral + //通过累计的误差量*某个比例
                        PID.Kd*(PID.ErrNow-PID.Last_Err);  //用误差的变化率达到指数调参
	PID.Last_Err = PID.ErrNow;  //将当前误差值存入上一误差值
    PID.CurrentValue=PID.CurrentValue+PID.OutputValue;//更新当前的控制值
	return PID.OutputValue;  //返回当前值
}