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
float plimit = 33.0f;//��ʼ�ϵ����ƹ���33W

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

	//����ռ�����Сռ�ձ�
	if(Duty <= 0.1f)
		Duty = 0.1f;
	else if(Duty >= 1.7f)
		Duty = 1.7f;
	//ռ�ձȷ���
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

//����
void PID_cal(void)
{
	
	Buck_boost.error = plimit * Plimit_UVCap_Coff - P_IN_k;
//	Buck_boost.error = plimit - P_IN_k;
	Duty+=Buck_boost.Kp*(Buck_boost.error-Buck_boost.last_error)+Buck_boost.Ki*Buck_boost.error+Buck_boost.Kd*(Buck_boost.error-2*Buck_boost.last_error+Buck_boost.pre_error);
	Buck_boost.last_error=Buck_boost.error;
	Buck_boost.pre_error=Buck_boost.last_error;
	
//	Duty=1.0f;
	
	//����ռ�����Сռ�ձ�
	if(Duty <= 0.1f)
		Duty = 0.1f;
	else if(Duty >= 1.7f)
		Duty = 1.7f;
	//ռ�ձȷ���
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
	//����ռ�ձ�
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000*Duty_buck-1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000*Duty_boost-1);
}

//����
void Protect(void)
{
	//�����߼�
	//�������ѹ����
	if(V_C_real > 26.9f)
	{
		Stop();
		State_flag = 0x01;//stop energy exchange
		error_flag = 0x01;//����
	}
	//�����鼴��Ƿѹ�߼��ж�
	else if(V_C_real < 10)
	{
		error_flag = 0x02;//����
	}		
	//�����������
//	else if(PIN > (plimit + 10))
//	{
//		Stop();
//		error_flag = 0x03;//����
//	}
	//����ϵͳ���ݶ�ʧ
	
	
}

//״̬�л�
void State_Set(void)
{
	//����������磬ֹͣ��������
	if((V_C_real > 26.4f)&&(State_flag == 0x00))//���ݶ��ߵ�ѹ
//	if(V_C_real > 26.0f)//���ݶ��ߵ�ѹ
	{
		State_flag = 0x01;//��������״̬��ֹͣ��������
		Stop();	
	}
	
	
	

	//����������Ƶò���ʱ��������԰�0�Ĵ�һЩ���������ԶԳ�������һ��Ԥ��
	else if(Buck_boost.error<0 && State_flag == 0x01)//������״̬�л�����������״̬
	{
		State_flag = 0x00;
		Start();
	}
	else if(V_C_real < 25.5f && State_flag == 0x01)//�������ݻ���©�磬����ѹ����25.9V��ʱ���л�����������״̬
	{
		State_flag = 0x00;
		Start();
	}	
	
	
}

uint8_t testtt1,testtt2;
uint8_t CAP_Charge = 0;//���������Ƿ��ڳ��ı�������Ϊ1��ʾ�������ڳ�磬Ϊ0��ʾ��ǰ����û�ڳ��
uint8_t CAP_DisCharge = 0;//���������Ƿ��ڷŵ�ı�������Ϊ1��ʾ������������ŵ磬Ϊ0��ʾ��ǰ����û������ŵ�
void I_IN_Correct_calcu(void)//ʵ�ⷢ�ֵ�������ֵ������ݵ�ѹ���͵��²���ֵƫС��������һ����Ϊ����
{
	static float V_Cap_his[101];
	uint8_t i;
	static uint8_t CNTDischarge,CNTcharge;
	
	
	for(i=100;i>0;i--)
	{
		V_Cap_his[i] = V_Cap_his[i-1];//��λ
	}
	V_Cap_his[0] = V_C_real;
	
	CNTDischarge = 0;
	CNTcharge = 0;
	
	
	for(i=0;i<81;i=i+2)
	{
		if(V_Cap_his[i] < V_Cap_his[i+20])//��ǰֵС����һ��ֵ����ʾ���ݵ�ѹ���½�
		{
			CNTDischarge++;
		}

		if(V_Cap_his[i] > V_Cap_his[i+20])//��ǰֵ������һ��ֵ����ʾ���ݵ�ѹ������
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
	
	
	if((CNTDischarge>20)&&(V_Cap_his[100] - V_Cap_his[0] -0.01f >0))//ֻ���ڵ�������ŵ�״̬�²Ż�����������
	{
		CAP_DisCharge = 1;
		if(V_C_real>21)//�����ݵ�ѹ����21V��ʱ��
		{
			I_IN_Correct = 1.0f;
		}
		else if(V_C_real>13)//�����ݵ�ѹ��13V~21V֮���ʱ��
		{
			I_IN_Correct = (21 - V_C_real)*0.016f + 1;
		}
		else if(V_C_real>5)//�����ݵ�ѹ��5V~13V֮���ʱ��
		{
			I_IN_Correct = (13 - V_C_real)*0.02f + 1.128f;
		}
		else//�����ݵ�ѹС��5V��ʱ��
		{
			I_IN_Correct = 1.32f;
		}		
		
		
	}
	else//���ݳ��״̬�²���������
	{
		CAP_DisCharge = 0;
		I_IN_Correct = 1.0f;
	}
	
}



//���ݲ���ϵͳ��ʵ�⹦�ʶ�Plimit������������һ���ֿ��Կ�����F103����ʵ��
void P_Limit_Correction(void)
{


}

float Vap_History[5];//ÿ��100ms��¼,��¼5
float Plimit_UVCap_Coff = 0.1f;//���ݵ�ѹ�ϵ͵�ʱ��Ĺ����������˵�ϵ����
//�����ݵ�ѹ�ϵ�(������������ݳ��)��ʱ�򣬽��͹�������ֵ����������ݵĳ�������ʵ�ֳ������
void P_Limit_Soft_Start(void)
{
	
	
	if(V_C_real<1)//��������ȱ��
	{
		Plimit_UVCap_Coff = 0.1f;
	}
	else//����δ����ȱ��
	{
		if((CAP_Charge == 1)&&(State_flag == 0x00))//��ǰ������������״̬�����ҵ������ڳ��
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



//TIM2�жϻص�
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t i;
	static uint32_t LED_TEMP;
	
	
	if(htim == &htim2)
	{
		State_Set();//״̬�л��߼�
		
		I_IN_Correct_calcu();
		
		Protect();
		
		P_Limit_Soft_Start();

		HAL_IWDG_Refresh(&hiwdg);//ÿ�ν��жϾ�ι��һ��
		
		
		
		//�����ϵĺ�ɫǷѹָʾ�Ƶ��߼�
		if(V_C_real<15)
		{
			LED_CAP_UV_ON;
		}
		else if(V_C_real>17)
		{
			LED_CAP_UV_OFF;
		}
		
		
		//�����ϵĳ�ŵ�״ָ̬ʾ���߼�
		if(State_flag == 0x01)//��ǰBUCK-BOOST�رգ�û�н�����������
		{
				LED_CAP_STA_OFF;
		}
		else if(CAP_DisCharge == 1)//��ǰ������������ŵ�,ָʾ����˸
		{
			if(i % 200 == 0)//200msִ��һ��
			{
				LED_CAP_STA_TOGGLE;
			}
		}
		else//��ǰ�������ڳ��
		{
			LED_CAP_STA_ON;
		}
		
		
		
		//����������ָʾ�Ƶ��߼�
		LED_TEMP = i/50 %35;//STA����˸
		switch(LED_TEMP)
		{
//			case 0:{LED_RUN_ON;break;}
//			case 4:{LED_RUN_OFF;break;}
//			case 10:{LED_RUN_ON;break;}
//			case 12:{LED_RUN_OFF;break;}
//			case 14:{LED_RUN_ON;break;}
//			case 16:{LED_RUN_OFF;break;}
		}		
		
		if(i % 100 == 0)//100msִ��һ��
		{
			
			
		}		
		
			i++;
		
	}
}

////TIM2�жϻص�
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











