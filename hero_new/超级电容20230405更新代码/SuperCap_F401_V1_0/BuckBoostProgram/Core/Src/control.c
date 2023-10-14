#include "main.h"
#include "control.h"
#include "adc.h"
#include "tim.h"
#include "iwdg.h"


uint16_t ADC_Value_buffer[30];
uint8_t error_flag;
uint8_t State_flag;


float V_C,V_IN,I_IN,PIN;
float V_C_last,V_IN_last,I_IN_last;

PID Buck_boost={KP_pre_P,KI_pre_P,KD_pre_P,0,0,0};
PID P_buf={KP_pre_B,KI_pre_B,KD_pre_B,0,0,0};
float Duty = 0.1f;
float Duty_buck,Duty_boost;
float plimit = 33.0f;//初始上电限制功率33W

uint32_t count0;

void Get_value(void)
{
	uint16_t value_temp[3] = {0,0,0};
	uint8_t i;	
	float V_C_temp,V_IN_temp,I_IN_temp;
	
	HAL_ADC_Stop_DMA(&hadc1);
	for(i=0;i<30;i++)
	{
		if(i%3==0)
			value_temp[0] += ADC_Value_buffer[i];
		else if(i%3==1)
			value_temp[1] += ADC_Value_buffer[i];
		else if(i%3==2)
			value_temp[2] += ADC_Value_buffer[i];	
	}
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Value_buffer,30);
	I_IN_temp = 3.2f/4096 * value_temp[0]/10 *4;
	V_IN_temp = 3.3f/4096 * value_temp[1]/10 *20;
	V_C_temp = 3.3f/4096 * value_temp[2]/10 *20;
	
	I_IN = 0.8f*I_IN_temp + 0.2f*I_IN_last;
	V_IN = 0.8f*V_IN_temp + 0.2f*V_IN_last;
	V_C = 0.8f*V_C_temp + 0.2f*V_C_last;
	
	I_IN_last = I_IN;
	V_IN_last = V_IN;
	V_C_last = V_C;
	
	PIN = V_IN*I_IN;
}

void Start(void)
{
	Duty = V_C_real/V_IN_real*0.8f;

	//限制占最大最小占空比
	if(Duty <= 0.1f)
		Duty = 0.1f;
	else if(Duty >= 1.7f)
		Duty = 1.7f;
	//占空比分配
	if(Duty <0.9f)
	{
		Duty_buck = Duty;
		Duty_boost = 0.9f;
	}
	else
	{
		Duty_buck = 0.9f;
		Duty_boost = 1.8f - Duty;
	}
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000*Duty_buck-1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000*Duty_boost-1);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
}

void Stop(void)
{
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
}

//控制
void PID_cal(void)
{
	
	Buck_boost.error = plimit * Plimit_UVCap_Coff - P_IN_k;
//	Buck_boost.error = plimit - P_IN_k;
	Duty+=Buck_boost.Kp*(Buck_boost.error-Buck_boost.last_error)+Buck_boost.Ki*Buck_boost.error+Buck_boost.Kd*(Buck_boost.error-2*Buck_boost.last_error+Buck_boost.pre_error);
	Buck_boost.last_error=Buck_boost.error;
	Buck_boost.pre_error=Buck_boost.last_error;
	
//	Duty=1.0f;
	
	//限制占最大最小占空比
	if(Duty <= 0.1f)
		Duty = 0.1f;
	else if(Duty >= 1.7f)
		Duty = 1.7f;
	//占空比分配
	if(Duty <0.9f)
	{
		Duty_buck = Duty;
		Duty_boost = 0.9f;
	}
	else
	{
		Duty_buck = 0.9f;
		Duty_boost = 1.8f - Duty;
	}
	//更新占空比
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000*Duty_buck-1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000*Duty_boost-1);
}

//保护
void Protect(void)
{
	//保护逻辑
	//电容组过压保护
	if(V_C_real > 26.9f)
	{
		Stop();
		State_flag = 0x01;//stop energy exchange
		error_flag = 0x01;//报错
	}
	//电容组即将欠压逻辑判定
	else if(V_C_real < 10)
	{
		error_flag = 0x02;//报错
	}		
	//输入过流保护
//	else if(PIN > (plimit + 10))
//	{
//		Stop();
//		error_flag = 0x03;//报错
//	}
	//裁判系统数据丢失
	
	
}

//状态切换
void State_Set(void)
{
	//电容组充满电，停止能量交换
	if((V_C_real > 26.4f)&&(State_flag == 0x00))//电容额定最高电压
//	if(V_C_real > 26.0f)//电容额定最高电压
	{
		State_flag = 0x01;//电容满电状态，停止能量交换
		Stop();	
	}
	
	
	

	//如果功率限制得不及时，这里可以把0改大一些，这样可以对超功率做一点预判
	else if(Buck_boost.error<0 && State_flag == 0x01)//从满电状态切换到能量交换状态
	{
		State_flag = 0x00;
		Start();
	}
	else if(V_C_real < 25.5f && State_flag == 0x01)//超级电容会逐渐漏电，当电压低于25.9V的时候切换到能量交换状态
	{
		State_flag = 0x00;
		Start();
	}	
	
	
}

uint8_t testtt1,testtt2;
uint8_t CAP_Charge = 0;//表征电容是否在充电的变量，若为1表示电容正在充电，为0表示当前电容没在充电
uint8_t CAP_DisCharge = 0;//表征电容是否在放电的变量，若为1表示电容正在向外放电，为0表示当前电容没在向外放电
void I_IN_Correct_calcu(void)//实测发现电流采样值会随电容电压降低导致采样值偏小，这里做一个人为补偿
{
	static float V_Cap_his[101];
	uint8_t i;
	static uint8_t CNTDischarge,CNTcharge;
	
	
	for(i=100;i>0;i--)
	{
		V_Cap_his[i] = V_Cap_his[i-1];//移位
	}
	V_Cap_his[0] = V_C_real;
	
	CNTDischarge = 0;
	CNTcharge = 0;
	
	
	for(i=0;i<81;i=i+2)
	{
		if(V_Cap_his[i] < V_Cap_his[i+20])//当前值小于上一个值，表示电容电压在下降
		{
			CNTDischarge++;
		}

		if(V_Cap_his[i] > V_Cap_his[i+20])//当前值大于上一个值，表示电容电压在上升
		{
			CNTcharge++;
		}		
		
	}
	
	testtt1 = CNTDischarge;
	testtt2 = CNTcharge;
	
	if((CNTcharge>15)&&(V_Cap_his[0] - V_Cap_his[100] -0.01f >0))
	{
		CAP_Charge =1;
	}
	else
	{
		CAP_Charge =0;
	}
	
	
	if((CNTDischarge>20)&&(V_Cap_his[100] - V_Cap_his[0] -0.01f >0))//只有在电容向外放电状态下才会有这种现象
	{
		CAP_DisCharge = 1;
		if(V_C_real>21)//当电容电压大于21V的时候
		{
			I_IN_Correct = 1.0f;
		}
		else if(V_C_real>13)//当电容电压在13V~21V之间的时候
		{
			I_IN_Correct = (21 - V_C_real)*0.016f + 1;
		}
		else if(V_C_real>5)//当电容电压在5V~13V之间的时候
		{
			I_IN_Correct = (13 - V_C_real)*0.02f + 1.128f;
		}
		else//当电容电压小于5V的时候
		{
			I_IN_Correct = 1.32f;
		}		
		
		
	}
	else//电容充电状态下不用做补偿
	{
		CAP_DisCharge = 0;
		I_IN_Correct = 1.0f;
	}
	
}



//根据裁判系统的实测功率对Plimit进行修正，这一部分可以考虑在F103里面实现
void P_Limit_Correction(void)
{


}

float Vap_History[5];//每隔100ms记录,记录5
float Plimit_UVCap_Coff = 0.1f;//电容电压较低的时候的功率限制所乘的系数，
//当电容电压较低(并且正在向电容充电)的时候，降低功率限制值，限制向电容的充电电流，实现充电软启
void P_Limit_Soft_Start(void)
{
	
	
	if(V_C_real<1)//电容严重缺电
	{
		Plimit_UVCap_Coff = 0.1f;
	}
	else//电容未严重缺电
	{
		if((CAP_Charge == 1)&&(State_flag == 0x00))//当前处于能量交换状态，并且电容正在充电
		{
			if(V_C_real<5)
			{
				Plimit_UVCap_Coff = 0.2f + (V_C_real-1)*0.1f;
			}
			else if(V_C_real<8)
			{
				Plimit_UVCap_Coff = 0.7f;
			}
			else
			{
				Plimit_UVCap_Coff = 1.0f;
			}
		}
		else
		{
			if(V_C_real<6)
			{
				Plimit_UVCap_Coff = 0.8f;
			}
			else
			{
				Plimit_UVCap_Coff = 1.0f;
			}
		}
	}
	

}



//TIM2中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t i;
	static uint32_t LED_TEMP;
	
	
	if(htim == &htim2)
	{
		State_Set();//状态切换逻辑
		
		I_IN_Correct_calcu();
		
		Protect();
		
		P_Limit_Soft_Start();

		HAL_IWDG_Refresh(&hiwdg);//每次进中断就喂狗一次
		
		
		
		//板子上的红色欠压指示灯的逻辑
		if(V_C_real<15)
		{
			LED_CAP_UV_ON;
		}
		else if(V_C_real>17)
		{
			LED_CAP_UV_OFF;
		}
		
		
		//板子上的充放电状态指示灯逻辑
		if(State_flag == 0x01)//当前BUCK-BOOST关闭，没有进行能量交换
		{
				LED_CAP_STA_OFF;
		}
		else if(CAP_DisCharge == 1)//当前电容正在向外放电,指示灯闪烁
		{
			if(i % 200 == 0)//200ms执行一次
			{
				LED_CAP_STA_TOGGLE;
			}
		}
		else//当前电容正在充电
		{
			LED_CAP_STA_ON;
		}
		
		
		
		//板子上运行指示灯的逻辑
		LED_TEMP = i/50 %35;//STA灯闪烁
		switch(LED_TEMP)
		{
//			case 0:{LED_RUN_ON;break;}
//			case 4:{LED_RUN_OFF;break;}
//			case 10:{LED_RUN_ON;break;}
//			case 12:{LED_RUN_OFF;break;}
//			case 14:{LED_RUN_ON;break;}
//			case 16:{LED_RUN_OFF;break;}
		}		
		
		if(i % 100 == 0)//100ms执行一次
		{
			
			
		}		
		
			i++;
		
	}
}

////TIM2中断回调
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == &htim2)
//	{
////		count0++;
//		
////		Get_value();
////		PID_cal();
////		State_Set();	
////		Protect();
//		
//		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
//	}
//}











