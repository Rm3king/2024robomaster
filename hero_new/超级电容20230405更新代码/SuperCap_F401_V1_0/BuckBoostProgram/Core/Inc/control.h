#ifndef __CONTROL_H__
#define __CONTROL_H__


//���ʻ�PID����
#define KP_pre_P 0.001f//0.001f
#define KI_pre_P 0.00001f//0.00001f
#define KD_pre_P 0.000005f
//��������PID����
#define KP_pre_B 0.0001f
#define KI_pre_B 0.0001f
#define KD_pre_B 0


//3��LED��
//LED1��ʾ����״̬�����ս�����˸��ʾ������������
//LED2��ʾ��ǰBUCK-BOOST����״̬�����ʾ�����У�������ʾ��ǰ���ڳ�磬������ʾ���ڷŵ�
//LED3��˸��ʾ�������ݵ�ѹ����12V
//#define LED_RUN_ON					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET)
//#define LED_RUN_OFF					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)
//#define LED_RUN_TOGGLE			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12)

//#define LED_CAP_STA_ON			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)
//#define LED_CAP_STA_OFF			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)
//#define LED_CAP_STA_TOGGLE	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_11)

//#define LED_CAP_UV_ON				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
//#define LED_CAP_UV_OFF			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)
//#define LED_CAP_UV_TOGGLE		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10)

#define LED_RUN_ON					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
#define LED_RUN_OFF					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)
#define LED_RUN_TOGGLE			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10)

#define LED_CAP_STA_ON			;
#define LED_CAP_STA_OFF			;
#define LED_CAP_STA_TOGGLE	;


#define LED_CAP_UV_ON				;
#define LED_CAP_UV_OFF			;
#define LED_CAP_UV_TOGGLE		;


//#define LED_CAP_UV_ON				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
//#define LED_CAP_UV_OFF			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)
//#define LED_CAP_UV_TOGGLE		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10)


//���ؿ��صĿ���
#define Load_Switch_ON			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)
#define Load_Switch_OFF			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)


typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	
	float error;
	float last_error;
	float pre_error;
}PID;



extern float V_C_real,V_IN_real,I_IN_real;
extern float I_IN_Correct;//ʵ�ⷢ�ֵ��ݵ�ѹ��16V���µ�������ֵƫС��12VʱƫСԼ10%��
extern float Plimit_UVCap_Coff;//���ݵ�ѹ�ϵ͵�ʱ��Ĺ����������˵�ϵ����

extern float P_IN_k;


void Get_value(void);
void Start(void);
void Stop(void);
void PID_cal(void);

void State_Set(void);

void Protect(void);






#endif
