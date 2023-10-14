#ifndef __CONTROL_H__
#define __CONTROL_H__


//功率环PID参数
#define KP_pre_P 0.001f//0.001f
#define KI_pre_P 0.00001f//0.00001f
#define KD_pre_P 0.000005f
//缓冲能量PID参数
#define KP_pre_B 0.0001f
#define KI_pre_B 0.0001f
#define KD_pre_B 0


//3个LED灯
//LED1表示运行状态，按照节律闪烁表示程序正常运行
//LED2表示当前BUCK-BOOST运行状态，灭表示不运行，慢闪表示当前正在充电，快闪表示正在放电
//LED3闪烁表示超级电容电压低于12V
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


//负载开关的控制
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
extern float I_IN_Correct;//实测发现电容电压在16V以下电流采样值偏小，12V时偏小约10%。
extern float Plimit_UVCap_Coff;//电容电压较低的时候的功率限制所乘的系数，

extern float P_IN_k;


void Get_value(void);
void Start(void);
void Stop(void);
void PID_cal(void);

void State_Set(void);

void Protect(void);






#endif
