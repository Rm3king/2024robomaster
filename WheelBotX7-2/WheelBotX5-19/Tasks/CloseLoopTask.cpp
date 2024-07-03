#include "CloseLoopTask.h"
#include "MotorTask.h"
#include "ControlTask.h"
#include "IMUTask.h"
#include "Filter.h"
#include "RemoterTask.h"

extern cRemoter *Msg_Remoter;
extern cRobotControl *RobotControl;
extern cINS *INS;


TX_THREAD	CloseLoopThread;
uint8_t 	CloseLoopThreadStack[1024]={0};
float outbut[2];
//float greavity_forward = 71.0f;//����ǰ������,71.0f
volatile float realaccelX;
uint8_t StartFlag;
uint8_t offground_flag = 1;
uint16_t offground_timer = 0;
float leg_vel[2];
float L_leg_dot[2];
float R_leg_dot[2];
float Leg_Ldot[2];
float Leg_Rdot[2];
float L_T[2];
float R_T[2];
float leg_dot;
uint16_t CheckOG_Timer=0;
void CloseLoopThreadFun(ULONG initial_input)
{
	
	ULONG timer;

	cFilterBTW2_40Hz VelocityBTW;
	cFilterBTW2_40Hz DisplacementBTW;
	
	float LastPsaiYaw = 0.0f;
	for(;;)
	{
		timer = tx_time_get();

		float psait = QCS.Yaw(INS->Q)-LastPsaiYaw;
		if(psait>PI){psait-=2*PI;}
		else if(psait<-PI){psait+=2*PI;}
		LastPsaiYaw =QCS.Yaw(INS->Q);
		
		/*Body radian calculate*/
		RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw	+= psait;
		RobotControl->ChasisControl.ObserveVal.ChasisRoll		= QCS.Roll(INS->Q);
		RobotControl->ChasisControl.ObserveVal.ChasisPitch		= QCS.Pitch(INS->Q);
			
//		if(RobotControl->ComDown.RobotDeath==1)
//		{
//			StartFlag = 0;
//			//RobotControl->ChasisControl.SetCFGENStatue(1);
//		}


		
		if(RobotControl->CheckRobotMode()==ROBOTMODE_IDLE)
		{
			StartFlag = 0;
			/*Shutdown!*/
			RobotControl->ChasisControl.MotorUnits->KF9025[0].UpdateTorque(0);
			RobotControl->ChasisControl.MotorUnits->KF9025[1].UpdateTorque(0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[0].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[1].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[2].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[3].MITUpdate(0,0,0,0,0);	
			
			/*Displacement setting to observated value*/
			//��ʼ����̼�
			RobotControl->ChasisControl.MotorUnits->InitOdomentor();
			RobotControl->ChasisControl.VelKF->ResetKF();
			RobotControl->ChasisControl.TargetVal.X[2] = RobotControl->ChasisControl.ObserveVal.X[2];
			RobotControl->ChasisControl.TargetVal.X[3] = 0.0f;
			offground_flag = 1;
			offground_timer = 0;
		}
		else if(RobotControl->CheckRobotMode()==ROBOTMODE_ESCAPE)
		{
			StartFlag = 0;
			/*Open-loop control*/
			RobotControl->ChasisControl.MotorUnits->LEGMotor[0].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[1].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[2].MITUpdate(0,0,0,0,0);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[3].MITUpdate(0,0,0,0,0);

			float WheelIq[2]={0};
			RobotControl->ChasisControl.RefreshChasisHead();
			RobotControl->ChasisControl.GetEsacpeVelocity(WheelIq);
			
			RobotControl->ChasisControl.LoopEscape[0].SetRef(WheelIq[0]);
			RobotControl->ChasisControl.LoopEscape[1].SetRef(WheelIq[1]);
			
			RobotControl->ChasisControl.LoopEscape[0].PID_Cal(RobotControl->ChasisControl.MotorUnits->KF9025[0].GetSpeed());
			RobotControl->ChasisControl.LoopEscape[1].PID_Cal(RobotControl->ChasisControl.MotorUnits->KF9025[1].GetSpeed());
			
			RobotControl->ChasisControl.MotorUnits->KF9025[0].UpdateTorque(RobotControl->ChasisControl.LoopEscape[0].GetOut());
			RobotControl->ChasisControl.MotorUnits->KF9025[1].UpdateTorque(RobotControl->ChasisControl.LoopEscape[1].GetOut());			

			
			/*Displacement setting to observated value*/
			RobotControl->ChasisControl.MotorUnits->InitOdomentor();
			RobotControl->ChasisControl.VelKF->ResetKF();
			RobotControl->ChasisControl.ObserveVal.X[2] = 0.0f;
			RobotControl->ChasisControl.TargetVal.X[2] = 0.0f;
			RobotControl->ChasisControl.TargetVal.X[3] = 0.0f;
			offground_flag = 1;
			offground_timer = 0;
		}
		else
		{
			/*Chasis follow gimbal*/
			float WheelMotor[2]={0};
			
			/*Leg ramp start*/
			static float Lentmp[2]={0};
			
			/*Leg motors*/
			/*FT = [PendulumForce PendulumTorque]   Torque = [Motor3Torque(backmotor)  Motor2Torque(frontmotor)] */
			float FT_L[2]={0};float TorqueL[2]={0};
			float FT_R[2]={0};float TorqueR[2]={0};
						

			if(RobotControl->CheckPartMode(ROBOTPART_ID_BLC)==ROBOTPART_STATUS_START)
			{		
				static float coff[2];static uint16_t addtimes;
				static ULONG TIM;
			  //�ý����쵽LEG_BOTTON ����Ϊ1/250
				if(StartFlag==0)
				{
					StartFlag = 1;addtimes  = 0;
					RobotControl->ChasisControl.LoopLen[0].Reset();RobotControl->ChasisControl.LoopLen[1].Reset();
					
					/*PowerOn:Middle Length*/
					RobotControl->ChasisControl.SetLegLen(LEG_BOTTOM);
					
					Lentmp[0] = RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen();
					Lentmp[1] = RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen();
					
					coff[0]=(RobotControl->ChasisControl.GetLegLen() - Lentmp[0])/50;
					coff[1]=(RobotControl->ChasisControl.GetLegLen() - Lentmp[1])/50;
					
				}
				else if(StartFlag==1)
				{

					Lentmp[0]+=coff[0];
					Lentmp[1]+=coff[1];
					/*������*/
					if(++addtimes==49)
					{
						addtimes=0;
						StartFlag = 2;
						RobotControl->ChasisControl.SetLegLen(LEG_LEN1);
						RobotControl->ChasisControl.SetBalanceFlag(1);
						Lentmp[0] = RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen();
						Lentmp[1] = RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen();
						coff[0]=(RobotControl->ChasisControl.GetLegLen() - Lentmp[0])/100;
						coff[1]=(RobotControl->ChasisControl.GetLegLen() - Lentmp[1])/100;
						RobotControl->ChasisControl.TargetVal.X[2] = RobotControl->ChasisControl.ObserveVal.X[2];
					}

				}
				else if(StartFlag==2)
				{
					Lentmp[0]+=coff[0];
					Lentmp[1]+=coff[1];
					if(++addtimes==99)
					{
						addtimes = 0;
						StartFlag = 0;
						RobotControl->ChasisControl.SetBalanceFlag(2);
						RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_NORMAL);
					}
				}
			}
			else
			{
				/*Length close loop*/
				static uint8_t LastLenFlag = RobotControl->ChasisControl.GetLegLenFlag();
				static float coff[2];static uint16_t addtimes;
				static uint16_t move_time; static uint16_t jump_time;
				
				if(RobotControl->ChasisControl.GetLegLenFlag()!=LastLenFlag)
				{
					StartFlag=1;addtimes=0;
					if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_JUMP )
					{
					if(RobotControl->ChasisControl.LastJumpMode==JUMP_READY){
						move_time=49;
					}
					else if(RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH){
						move_time=29;
					}
					else if(RobotControl->ChasisControl.LastJumpMode==JUMP_SHRINK){
						move_time=29;
					}
					else if(RobotControl->ChasisControl.LastJumpMode==JUMP_STOP){
						move_time=29;
					}
					}
					else {move_time=199;}
					/*PowerOn:Middle Length*/
					RobotControl->ChasisControl.LoopLen[0].Reset();RobotControl->ChasisControl.LoopLen[1].Reset();
					coff[0]=(RobotControl->ChasisControl.GetLegLen()-Lentmp[0])/200;
					coff[1]=(RobotControl->ChasisControl.GetLegLen()-Lentmp[1])/200;
				}
				else if(StartFlag==1)
				{
					Lentmp[0]+=coff[0];
					Lentmp[1]+=coff[1];
					if(++addtimes==move_time)
					{StartFlag=0;}
				}
				else
				{Lentmp[0] = RobotControl->ChasisControl.GetLegLen();Lentmp[1]=Lentmp[0];}			
				LastLenFlag = RobotControl->ChasisControl.GetLegLenFlag();
			}	
			if(RobotControl->ChasisControl.GetCFGENFlag())
			{
				float CFGerror = RobotControl->ChasisControl.GetCFGControlError();
				
				/*Keep follow*/
				RobotControl->ChasisControl.LoopCFG.PID_Cal(CFGerror);
				RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw = RobotControl->ChasisControl.LoopCFG.GetOut();
			}
			else
			{RobotControl->ChasisControl.LoopCFG.Reset();}
			/*Robot roll*/
			RobotControl->ChasisControl.LoopRoll.SetRef(0.0f);
			RobotControl->ChasisControl.LoopRoll.PID_Cal(QCS.Roll(INS->Q));
			
			//�ȳ�//
			//�ȳ�����  �Ƿ���Roll��
//			RobotControl->ChasisControl.LoopLen[0].SetRef(Lentmp[0] + RobotControl->ChasisControl.LoopRoll.GetOut());
//			RobotControl->ChasisControl.LoopLen[1].SetRef(Lentmp[1] - RobotControl->ChasisControl.LoopRoll.GetOut());
			//�������ȳ��� �ص� ��Roll
//			RobotControl->ChasisControl.LoopLen[0].SetRef(Lentmp[0]);
//			RobotControl->ChasisControl.LoopLen[1].SetRef(Lentmp[1]);
			RobotControl->ChasisControl.LoopLen_Spf[0].SetRef(Lentmp[0] + RobotControl->ChasisControl.LoopRoll.GetOut());
			RobotControl->ChasisControl.LoopLen_Spf[1].SetRef(Lentmp[1] - RobotControl->ChasisControl.LoopRoll.GetOut());
//			RobotControl->ChasisControl.LoopLen_Spf[0].SetRef(Lentmp[0]);
//			RobotControl->ChasisControl.LoopLen_Spf[1].SetRef(Lentmp[1]);
			//�ȳ���  ������ʱ�����ȳ������� ����ʱP��DС  ���ʱ  PС D��
//			if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_JUMP)
//			{
//				if(RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH||RobotControl->ChasisControl.LastJumpMode==JUMP_READY||RobotControl->ChasisControl.LastJumpMode==JUMP_SHRINK||RobotControl->ChasisControl.LastJumpMode==JUMP_STOP)
//				{
//				RobotControl->ChasisControl.LoopLen[0].Set_Kp(700);
//				RobotControl->ChasisControl.LoopLen[0].Set_Kd(300);						
//				}
//				else
//				{
//				RobotControl->ChasisControl.LoopLen[0].Set_Kp(200);
//				RobotControl->ChasisControl.LoopLen[0].Set_Kd(700);
//				}
//			}
//			else
//			{
//				RobotControl->ChasisControl.LoopLen[0].Set_Kp(400);
//				RobotControl->ChasisControl.LoopLen[0].Set_Kd(500);
//			}
			RobotControl->ChasisControl.LoopLen[0].PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen());
			RobotControl->ChasisControl.LoopLen[1].PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen());
			//Ϊ״̬�۲���׼������
			volatile float LegRealrad = 0.5f*(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian()+RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian());
			//volatile float LegRealrad = RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian();
			volatile float costheta;
			volatile float sintheta;	
			static float   ThetaLast;
			float Tlqr[2]={0};
			/* �ȳ��ٶȼ��� */
			leg_vel[0] = RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetVelocity();
			leg_vel[1] = RobotControl->ChasisControl.MotorUnits->LEGMotor[1].GetVelocity();
			RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCRevCal_Radian(leg_vel,L_leg_dot);
			RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCVelCal(leg_vel,Leg_Ldot);
			leg_vel[0] = RobotControl->ChasisControl.MotorUnits->LEGMotor[2].GetVelocity();
			leg_vel[1] = RobotControl->ChasisControl.MotorUnits->LEGMotor[3].GetVelocity();
			RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCRevCal_Radian(leg_vel,R_leg_dot);
			RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCVelCal(leg_vel,Leg_Rdot);
			/*Set observation*/
			RobotControl->ChasisControl.ObserveVal.X[0] = LegRealrad + RobotControl->ChasisControl.ObserveVal.ChasisPitch - PI_Half ;
			costheta = arm_cos_f32(RobotControl->ChasisControl.ObserveVal.X[0]);
			sintheta = arm_sin_f32(RobotControl->ChasisControl.ObserveVal.X[0]);
			//RobotControl->ChasisControl.ObserveVal.X[1] = 0.2f*RobotControl->ChasisControl.ObserveVal.X[1] + 400.0f*(RobotControl->ChasisControl.ObserveVal.X[0] - ThetaLast);
			RobotControl->ChasisControl.ObserveVal.X[1] = Leg_Ldot[1]-Leg_Rdot[1]-INS->Gyro[1];
			RobotControl->ChasisControl.ObserveVal.X[4] = -RobotControl->ChasisControl.ObserveVal.ChasisPitch;
/**/		RobotControl->ChasisControl.ObserveVal.X[5] = INS->Gyro[1];/*This is the really pitch. Different control board position should be treated differently*/
			/*Calculate body displacement and body velocity. Calculated from wheel and leg.*/
			/*Velocity Kalman filter calculate*/
			float q_inv[4] = {INS->Q[0],-INS->Q[1],-INS->Q[2],INS->Q[3]};
			float a_body[4] = {0,INS->Accel[0],INS->Accel[1],INS->Accel[2]};
			float a_world[4] = {0};
			float tmp[4] = {0};
			arm_quaternion_product_f32(INS->Q,a_body,tmp,1);
			arm_quaternion_product_f32(tmp,q_inv,a_world,1);
			float body_Accel = sqrtf(a_world[1]*a_world[1]+a_world[2]*a_world[2])*arm_cos_f32(atan2f(a_world[2],a_world[1])-QCS.Yaw(INS->Q));
			volatile float SinPhi = arm_sin_f32(RobotControl->ChasisControl.ObserveVal.ChasisPitch);
			volatile float CosPhi = arm_cos_f32(RobotControl->ChasisControl.ObserveVal.ChasisPitch);
			//�ںϹߵ��ռ���ٶ�  ����
/**/  realaccelX = ((-RobotControl->INS->Accel[0]+RobotControl->INS->Accel[2])+9.8f*(SinPhi-CosPhi))/(SinPhi+CosPhi);
			RobotControl->ChasisControl.accelX=realaccelX;
			
			/*This is good*/   //��̼ƺͼ��ٶ��ں�
			RobotControl->ChasisControl.VelKF->UpdateKalman(RobotControl->ChasisControl.MotorUnits->GetVel(), realaccelX);
			/*This is slow to be stable*/
//			if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_ROLLING)
//			{
//				RobotControl->ChasisControl.VelKF->ResetKF();
//				RobotControl->ChasisControl.ObserveVal.X[2] = 0.0f;
//				RobotControl->ChasisControl.TargetVal.X[2] = 0.0f;
//			}
			/////RobotControl->ChasisControl.VelKF->UpdateKalman(RobotControl->ChasisControl.MotorUnits->GetVel()+sintheta*LegRealLength*RobotControl->ChasisControl.ObserveVal.X[1], realaccelX);
			
			//С����ʱ�����̼�
       
//			if(RobotControl->ChasisControl.GetChasisMode()!=ROBOTPART_CHASIS_ROLLING){
				//�ںϺ�õ��ֵ��λ�ƺ��ٶ�  ��ΪLQR״̬����
//				RobotControl->ChasisControl.obervalx[0]=RobotControl->ChasisControl.VelKF->KF.xhat.pData[0]+RobotControl->ChasisControl.obervalx[1]*0.1+0.5*realaccelX*0.1*0.1;
//				RobotControl->ChasisControl.obervalx[1]=RobotControl->ChasisControl.VelKF->KF.xhat.pData[1]+realaccelX*0.1;
				
				RobotControl->ChasisControl.obervalx[0]=RobotControl->ChasisControl.VelKF->KF.xhat.pData[0];
				RobotControl->ChasisControl.obervalx[1]=RobotControl->ChasisControl.VelKF->KF.xhat.pData[1];
//			}
			RobotControl->ChasisControl.ObserveVal.X[2] = RobotControl->ChasisControl.obervalx[0];
			RobotControl->ChasisControl.ObserveVal.X[3] = RobotControl->ChasisControl.obervalx[1];
			
			/*LQR*/ 	
			/*Set reference*/
//			RobotControl->ChasisControl.TargetVal.X[0] = 0;
			RobotControl->ChasisControl.TargetVal.X[1] = 0;
      //�ٶȽ�Сʱ  ״̬�仯���� ʹ��Ŀ��ֵ����۲�ֵ ��֤������ƽ�� 
			if(fabs(RobotControl->ChasisControl.GetForwardVelocity())<=0.1f)
			{
				RobotControl->ChasisControl.TargetVal.X[2] = DisplacementBTW.BTW2Cal(RobotControl->ChasisControl.TargetVal.X[2] + 0.002f*RobotControl->ChasisControl.GetForwardVelocity());
				RobotControl->ChasisControl.TargetVal.X[3] = VelocityBTW.BTW2Cal(RobotControl->ChasisControl.GetForwardVelocity());
			}
			else
			{
				RobotControl->ChasisControl.TargetVal.X[2] = DisplacementBTW.BTW2Cal(RobotControl->ChasisControl.ObserveVal.X[2] + 0.002f*RobotControl->ChasisControl.GetForwardVelocity());
				RobotControl->ChasisControl.TargetVal.X[3] = VelocityBTW.BTW2Cal(RobotControl->ChasisControl.GetForwardVelocity());
			}
//			RobotControl->ChasisControl.TargetVal.X[2] = DisplacementBTW.BTW2Cal(RobotControl->ChasisControl.ObserveVal.X[2] + 0.002f*RobotControl->ChasisControl.GetForwardVelocity());
//			RobotControl->ChasisControl.TargetVal.X[3] = VelocityBTW.BTW2Cal(RobotControl->ChasisControl.GetForwardVelocity());
			RobotControl->ChasisControl.TargetVal.X[4] = 0;
//			RobotControl->ChasisControl.TargetVal.X[4] = -RobotControl->ChasisControl.TargetVal.ChasisPitch;
			RobotControl->ChasisControl.TargetVal.X[5] = 0;
			
			ThetaLast = RobotControl->ChasisControl.ObserveVal.X[0];
			
			/*Off ground check*/
			{
				/*change*/
				volatile float LegRealLength=0.5f*(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen()+RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen());
				float FT_L_Real[2]={0};float TorqueL_Real[2]={-RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetToqReal(),-RobotControl->ChasisControl.MotorUnits->LEGMotor[1].GetToqReal()};
				float FT_R_Real[2]={0};float TorqueR_Real[2]={RobotControl->ChasisControl.MotorUnits->LEGMotor[2].GetToqReal(),RobotControl->ChasisControl.MotorUnits->LEGMotor[3].GetToqReal()};
				RobotControl->testTorque1[0]=-RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetToqReal();
				RobotControl->testTorque1[1]=-RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetToqReal();
				RobotControl->testTorque2[0]=-RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetToqReal();
				RobotControl->testTorque2[1]=-RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetToqReal();
				static float LastLength = LegRealLength;
				float Lengthdot;
				static float LastLengthdot;
				static float LastThetaDot 	= RobotControl->ChasisControl.ObserveVal.X[1];
				static float LastTheta 		= RobotControl->ChasisControl.ObserveVal.X[0];
				RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCRevCal(FT_L_Real,TorqueL_Real);
				RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCRevCal(FT_R_Real,TorqueR_Real);
				
				float Freal = FT_L_Real[0] + FT_R_Real[0];
				float Treal = FT_L_Real[1] + FT_R_Real[1];
				
				
				Lengthdot = LegRealLength - LastLength;
				
				
				volatile float P = Freal*costheta + Treal*sintheta/LegRealLength;
				volatile float Zw = 
					(INS->Accel[2]-GACCEL)
					-(Lengthdot-LastLengthdot)*costheta
					+2*(LegRealLength-LastLength)*(RobotControl->ChasisControl.ObserveVal.X[0]-LastTheta)*sintheta
					+LegRealLength*(RobotControl->ChasisControl.ObserveVal.X[1]-LastThetaDot)*sintheta
					+LegRealLength*(RobotControl->ChasisControl.ObserveVal.X[1]*RobotControl->ChasisControl.ObserveVal.X[1])*costheta;
				
				LastLength		= LegRealLength;
				LastLengthdot	= Lengthdot;
				LastThetaDot	= RobotControl->ChasisControl.ObserveVal.X[1];
				LastTheta		= RobotControl->ChasisControl.ObserveVal.X[0];
				
				volatile float Fn = P + MWHEEL*GACCEL + MWHEEL*Zw;
				RobotControl->ChasisControl.Set_OG_Fn(Fn);
				RobotControl->testTorqueR[0]=Fn;
				
				
				//�����⵽�ڿ�  ����K21 K22 K25 K26ȫ������   ���ðڸ�������״̬
				if(Fn<RobotControl->ChasisControl.Get_CheckOG_Value())//����25,��60
				{
					
			if(RobotControl->ChasisControl.Get_MoveMode()==Move_Jump&& RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH)
				{		
				RobotControl->ChasisControl.SetOG(2);      					
				}
			else
			{
				RobotControl->ChasisControl.SetOG(1);
			}
				}
				//��������˴�����Ծ�������� ��K21 K22������ȫ������  ��֤������
//				else if(RobotControl->ChasisControl.GetChasisMode()==ROBOTPART_CHASIS_JUMP&& RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH )
//				{
//					RobotControl->ChasisControl.SetOG(1);
//				}
				//��������˴�������״̬   ��LQR������������
				else
				{
					RobotControl->ChasisControl.SetOG(0);
					//RobotControl->ChasisControl.TargetVal.X[0] = 0.0f;
				}
				if(Fn>70.0f){RobotControl->ChasisControl.SetLand(1);}
				else {RobotControl->ChasisControl.SetLand(0);}
			}
			if(offground_flag == 0)
			{
				if(++offground_timer > 1000)
				{
					offground_flag = 1;
				}
			}
			/*Fall out dectection*/
			if(RobotControl->ChasisControl.GetBalanceFlag()==2)
			{
				static uint16_t FallNum;
				if(fabs(RobotControl->ChasisControl.ObserveVal.ChasisPitch)>0.40f)
				{
					if(++FallNum>150)
					{
						RobotControl->SetRobotMode(ROBOTMODE_ESCAPE);
						RobotControl->SetPartMode(ROBOTPART_ID_LEG,ROBOTPART_STATUS_IDLE);
						RobotControl->SetPartMode(ROBOTPART_ID_BLC,ROBOTPART_STATUS_IDLE);
						
						if(RobotControl->CheckPartMode(ROBOTPART_ID_BST)!=ROBOTPART_STATUS_NORMAL)
						{RobotControl->SetPartMode(ROBOTPART_ID_BST,ROBOTPART_STATUS_START);}
						if(RobotControl->CheckPartMode(ROBOTPART_ID_GIM)!=ROBOTPART_STATUS_NORMAL)
						{RobotControl->SetPartMode(ROBOTPART_ID_GIM,ROBOTPART_STATUS_START);}
						FallNum=0;
					}
				}
				else
				{
					FallNum = 0;
				}
			}
			
			/*Chasis Yaw close-loop*/
			//С����
			static float LastYaw;
//			RobotControl->ChasisControl.LoopYaw.SetRef(0.2f*LastYaw+0.8f*RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw);
//			RobotControl->ChasisControl.LoopYaw.PID_Cal(RobotControl->ChasisControl.ObserveVal.ChasisPsaiYaw);
			LastYaw = RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw;
		
				RobotControl->ChasisControl.LoopGyro.SetRef(RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw);
  			RobotControl->ChasisControl.LoopGyro.PID_Cal(INS->Gyro[2]);
//			LastYaw = RobotControl->ChasisControl.TargetVal.ChasisPsaiYaw;
//			RobotControl->ChasisControl.LoopYaw.SetRef(0);
//			RobotControl->ChasisControl.LoopYaw.PID_Cal(INS->Gyro[2]);
//			}
			//������  �ص����� ͬʱ�ص���
			if(RobotControl->ChasisControl.CheckOG())
			{
				//��غ���ٻص� Ŀ��״̬
					RobotControl->ChasisControl.TargetVal.X[2] = RobotControl->ChasisControl.ObserveVal.X[2];				
				if(RobotControl->ChasisControl.Get_MoveMode()==Move_Fly)
				{
				RobotControl->ChasisControl.SetLegLen(LEG_LEN4);		
				RobotControl->ChasisControl.TargetVal.X[0]=-0.300f;
					RobotControl->ChasisControl.Set_GravityForward(5.0f);
					RobotControl->ChasisControl.Set_FlyMid_Flag(1);
				}
				else if(RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH)
				{		
				RobotControl->ChasisControl.TargetVal.X[0]=0.000f;	
        RobotControl->ChasisControl.Set_GravityForward(71.0f);       					
				}
				else
				{		
				RobotControl->ChasisControl.TargetVal.X[0]=0.0f;
        RobotControl->ChasisControl.Set_GravityForward(71.0f);					
				}
			}
			else if(RobotControl->ChasisControl.CheckOG()==2)
			{		
				RobotControl->ChasisControl.TargetVal.X[0]=0.3f;	
        RobotControl->ChasisControl.Set_GravityForward(71.0f);       					
			}
			else
			{	
				//�����ҵ���ּ���yawת�����
				//���� ƽ��ʱ�ص�
				RobotControl->ChasisControl.TargetVal.X[0]=0.0f;
				WheelMotor[0] -= RobotControl->ChasisControl.LoopGyro.GetOut();
				WheelMotor[1] += RobotControl->ChasisControl.LoopGyro.GetOut();
//				RobotControl->ChasisControl.Set_GravityForward(60.0f);
        if(RobotControl->ChasisControl.Get_MoveMode()==Move_Fly)
				{				
					RobotControl->ChasisControl.SetLegLen(LEG_LEN2);
				RobotControl->ChasisControl.Set_GravityForward(60.0f);//greavity_forward = 71;
					if(RobotControl->ChasisControl.Get_FlyMid_Flag()==1)
					{
						RobotControl->ChasisControl.Set_MoveMode(Move_Normal);
						RobotControl->ChasisControl.Set_FlyMid_Flag(0);
						RobotControl->ChasisControl.SetLegLen(LEG_LEN1);
					}
					
				}
//				else if(RobotControl->ChasisControl.LastJumpMode==JUMP_STRETCH)
//				{		
//				RobotControl->ChasisControl.TargetVal.X[0]=0.000f;	
//        RobotControl->ChasisControl.Set_GravityForward(71.0f);       					
//				}
			}
			
			RobotControl->ChasisControl.LQR.RefreshLQRK(0.5f*(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen()+RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen()),RobotControl->ChasisControl.CheckOG());
			RobotControl->ChasisControl.LQR.LQRCal(Tlqr); 
			/*Wheel Banlance*/
			//���ȳ�  �ص�LQR
			WheelMotor[0] += Tlqr[0];
			WheelMotor[1] += Tlqr[0];
			
			RobotControl->testTorque1[0]=Tlqr[0];
			RobotControl->testTorque1[1]=Tlqr[1];
			
			/*Theta different control*/   
			//���滷���������
			RobotControl->ChasisControl.LoopTheta.SetRef(RobotControl->ChasisControl.LoopTheta.MAXTHETA);
			RobotControl->ChasisControl.LoopTheta.PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian() - RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian());
//			/* �ȳ��ٶȼ��� */
//			leg_vel[0] = RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetVelocity();
//			leg_vel[1] = RobotControl->ChasisControl.MotorUnits->LEGMotor[1].GetVelocity();
//			RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCRevCal_Radian(leg_vel,L_leg_dot);
//			RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCVelCal(leg_vel,Leg_Ldot);
//			leg_vel[0] = RobotControl->ChasisControl.MotorUnits->LEGMotor[2].GetVelocity();
//			leg_vel[1] = RobotControl->ChasisControl.MotorUnits->LEGMotor[3].GetVelocity();
//			RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCRevCal_Radian(leg_vel,R_leg_dot);
//			RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCVelCal(leg_vel,Leg_Rdot);
			RobotControl->ChasisControl.LoopLen_Dot[0].SetRef(0);
			RobotControl->ChasisControl.LoopLen_Dot[0].PID_Cal(L_leg_dot[0]);
			RobotControl->ChasisControl.LoopLen_Dot[1].SetRef(0);
			RobotControl->ChasisControl.LoopLen_Dot[1].PID_Cal(R_leg_dot[0]);
			/* �ٶȷ������滷 */
			RobotControl->ChasisControl.LoopTheta_Spf.SetRef(RobotControl->ChasisControl.LoopTheta_Spf.MAXTHETA);
			RobotControl->ChasisControl.LoopTheta_Spf.PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumRadian() - RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumRadian(),-(Leg_Ldot[1]+Leg_Rdot[1]));
			/* �ȳ��ٶȷ���PID���� */
			RobotControl->ChasisControl.LoopLen_Spf[0].PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[0].GetPendulumLen(),-Leg_Ldot[0]);
			RobotControl->ChasisControl.LoopLen_Spf[1].PID_Cal(RobotControl->ChasisControl.MotorUnits->LinkSolver[1].GetPendulumLen(),-Leg_Rdot[0]);
			/*�ȳ��ٶ�*/
			
			/*VMC*/
			/*Force*/
      //����roll��
			RobotControl->ChasisControl.LoopRollOffset.SetRef(0.0f);
			RobotControl->ChasisControl.LoopRollOffset.PID_Cal(RobotControl->ChasisControl.ObserveVal.ChasisRoll);
			//�����ƽ��״̬ ������� ������ǰ����
			if(RobotControl->ChasisControl.GetBalanceFlag() != 2)
			{
			FT_L[0] = -RobotControl->ChasisControl.LoopLen_Spf[0].GetOut();//RobotControl->ChasisControl.Get_GravityForward()
			FT_R[0] = -RobotControl->ChasisControl.LoopLen_Spf[1].GetOut();				
			}
			else if(RobotControl->ChasisControl.CheckOG())
			{	
			FT_L[0] = -(RobotControl->ChasisControl.LoopLen_Spf[0].GetOut()+RobotControl->ChasisControl.Get_GravityForward());
			FT_R[0] = -(RobotControl->ChasisControl.LoopLen_Spf[1].GetOut()+RobotControl->ChasisControl.Get_GravityForward());	
//			FT_L[0] = -(RobotControl->ChasisControl.LoopLen_Spf[0].GetOut());
//			FT_R[0] = -(RobotControl->ChasisControl.LoopLen_Spf[1].GetOut());				
			}
			else
			{
			FT_L[0] = -(RobotControl->ChasisControl.LoopLen_Spf[0].GetOut()+RobotControl->ChasisControl.LoopRollOffset.GetOut()+RobotControl->ChasisControl.Get_GravityForward());
			FT_R[0] = -(RobotControl->ChasisControl.LoopLen_Spf[1].GetOut()-RobotControl->ChasisControl.LoopRollOffset.GetOut()+RobotControl->ChasisControl.Get_GravityForward());
//			FT_L[0] = -(RobotControl->ChasisControl.LoopLen_Spf[0].GetOut());
//			FT_R[0] = -(RobotControl->ChasisControl.LoopLen_Spf[1].GetOut());
			}
			//���滷
			/*Torque*/
//			FT_L[1] =  Tlqr[1] + RobotControl->ChasisControl.LoopTheta.GetOut();
//			FT_R[1] =  Tlqr[1] - RobotControl->ChasisControl.LoopTheta.GetOut();
//			FT_L[1] =  Tlqr[1];
//			FT_R[1] =  Tlqr[1] ;
			//�������滷
//			FT_L[1] = RobotControl->ChasisControl.LoopTheta.GetOut();
//			FT_R[1] = -RobotControl->ChasisControl.LoopTheta.GetOut();
			FT_L[1] =Tlqr[1]+RobotControl->ChasisControl.LoopTheta_Spf.GetOut();
			FT_R[1] =Tlqr[1]-RobotControl->ChasisControl.LoopTheta_Spf.GetOut();				
			/*Torque*/
			//С������Ҫ�ջ� ���ջ�������෴�������ӵ� FT��
			RobotControl->ChasisControl.MotorUnits->LinkSolver[0].VMCCal(FT_L,TorqueL);
			RobotControl->ChasisControl.MotorUnits->LinkSolver[1].VMCCal(FT_R,TorqueR);
			
			/*��ת��Ҫ�ջ�*/
			
			/*Output!*/
			
			TorqueL[0] = TorqueL[0]>T_MAX?T_MAX:TorqueL[0];
			TorqueL[0] = TorqueL[0]<T_MIN?T_MIN:TorqueL[0];
			TorqueL[1] = TorqueL[1]>T_MAX?T_MAX:TorqueL[1];
			TorqueL[1] = TorqueL[1]<T_MIN?T_MIN:TorqueL[1];

			TorqueR[0] = TorqueR[0]>T_MAX?T_MAX:TorqueR[0];
			TorqueR[0] = TorqueR[0]<T_MIN?T_MIN:TorqueR[0];
			TorqueR[1] = TorqueR[1]>T_MAX?T_MAX:TorqueR[1];
			TorqueR[1] = TorqueR[1]<T_MIN?T_MIN:TorqueR[1];
			
			/*Open-loop control*/
			/*LQR Calculate*/
			//��ⲻƽ��
			if(RobotControl->ChasisControl.GetBalanceFlag()!=0){
			   RobotControl->ChasisControl.MotorUnits->KF9025[0].UpdateTorque(WheelMotor[0]);
			   RobotControl->ChasisControl.MotorUnits->KF9025[1].UpdateTorque(-WheelMotor[1]);
			}
			
			RobotControl->testTorqueR[0]=WheelMotor[0];
			RobotControl->testTorqueR[1]=-WheelMotor[1];
			RobotControl->ChasisControl.MotorUnits->LEGMotor[0].MITUpdate(0,0,0,0,TorqueL[0]);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[1].MITUpdate(0,0,0,0,TorqueL[1]);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[2].MITUpdate(0,0,0,0,-TorqueR[0]);
			RobotControl->ChasisControl.MotorUnits->LEGMotor[3].MITUpdate(0,0,0,0,-TorqueR[1]);
			RobotControl->ChasisControl.LastChasisMode=RobotControl->ChasisControl.GetChasisMode();

			R_T[0]=RobotControl->ChasisControl.MotorUnits->LEGMotor[2].GetToqReal();
			R_T[1]=RobotControl->ChasisControl.MotorUnits->LEGMotor[3].GetToqReal();
			L_T[0]=RobotControl->ChasisControl.MotorUnits->LEGMotor[0].GetToqReal();
			L_T[1]=RobotControl->ChasisControl.MotorUnits->LEGMotor[1].GetToqReal();
		
			
			
		}
		tx_thread_sleep_until(&timer,2);
	}
}