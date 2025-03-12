/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

//float LQR_K_R[12]={ -1.5171 ,  -0.1347  , -2.4105,   -0.9858 ,   0.8685  ,  0.0783,
//    1.2392 ,   0.1251 ,   2.9650 ,   1.0868,   10.7689 ,   0.5026};

//Q=diag([20 0.1 80 110 700 1]);
//    R=[90 0;0 4]; 
float LQR_K_R[12]={ 
   -2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

//float LQR_K_R[12]={   -2.1340  , -0.2028 ,  -0.8838 ,  -1.3155 ,   1.2601 ,   0.1103,
//    2.3696  , 0.2669  ,  1.5572 ,   2.2473  , 12.2365   , 0.4599};
//float Poly_Coefficient[12][4]={	{-46.8734500193858,	32.4883608520456,	-17.0885922501780	,-0.536065047533883},
//																{1.48246010807197,	-2.36607779875741,	-1.53521740398882,	-0.0213682479756981},
//																{-44.9737664784523,	25.1746395712299,	-4.80854925156531,	-2.84031633786197},
//																{-4.93762493870657,	3.91974634758477,	-2.79459683826947,	-1.00823815038500},
//																{-194.609164251463,	117.144266080074,	-25.3701698043778	,2.32296290792753},
//																{-10.7217041372824,	6.59258111237609,	-1.47162116809482,	0.166761140976817},
//																{-141.430351511940,	86.5035702356333,	-19.2831262431850,	2.08471526597486},
//																{-12.4480584697546,	7.57666027947902,	-1.66833442186198	,0.196027465825959},
//																{-646.233100548413,	386.625379822743,	-82.6337764346849,	6.98716202318258},
//																{-227.994193295510,	135.591639019836,	-28.7547957179339	,2.43623020558302},
//																{205.536462887607,	-116.019511024166	,22.4916342449141,9.77817098877540},
//																{15.8984823843716	,-9.14713791307948,	1.83367299366907	,0.420742565153764}};
	
float Poly_Coefficient[12][4]={	{-88.3079710751263,	68.9068310796955,	-30.0003802287502,	-0.197774178106864},
																{1.52414598059982	,-1.09343038036609,	-2.82688593867512,	0.0281973842051861},
																{-21.8700750609220	,12.7421672466682,	-2.58779676995074	,-0.750848242540331},
																{-29.3271263750692,	17.6067629457167,	-4.23484645974363	,-1.08976980288501},
																{-147.771748892911,	94.0665615939814,	-22.5139626085997	,2.53224765312440},
																{-6.72857056332562,	4.46216499907277,	-1.14328671767927	,0.176775242328476},
																{-43.1495035855057,	35.1427890165576,	-12.7617044245710	,3.36940801739176},
																{4.14428184617563,	-2.56933858132474,	0.479050092243477	,0.248175261724735},
																{-229.898177881547	,144.949258291255	,-33.9196587052128,	3.44291788865558},
																{-329.509693153293,	207.219295206736,	-48.3799707459102	,4.952560575479143},
																{380.589246401548,	-223.660017597103	,46.1696952431268	,9.82308882692083},
																{26.1010681824798	,-15.7241310513153	,3.39175554658673	,0.278568898146322}};
//三次多项式拟合系数
//float Poly_Coefficient[12][4]={	{-92.0699600773357	,71.4868555011764	,-30.6921933953314,	-0.0958002007084524},
//																{1.60098559368738,	-1.13274122580887	,-2.82116637582756	,0.0295182225494464},
//																{-21.1867196303270,	12.3610554853386,	-2.51566924070518	,-0.755562691415545},
//																{-30.6461230131359,	18.4403464720723,	-4.42893895932222	,-1.07006891622098},
//																{-141.783593718613,	90.5321293186876,	-21.7824629497436	,2.47606043845602},
//																{-6.49940206537698,	4.32462034853376	,-1.11404205284405,	0.174401976130811},
//																{3.09605574012049	,7.37880709057633	,-6.81650374507351	,2.87205502564828},
//																{4.83479253102295	,-3.01643745917309,	0.586384046364414	,0.237251571342193},
//																{-224.130470954818	,141.560444143461	,-33.2264057886601,	3.39152168796696},
//																{-313.237104933888,	197.784837724870	,-46.4812453696717,	4.81419185198688},
//																{360.850043501824,	-212.560933205359	,44.0392588918109 ,	9.96626295435711},
//																{25.0961164366780	,-15.1533421691331,	3.28051567574422	,0.286235660771812}};
vmc_leg_t right;

extern INS_t INS;
extern vmc_leg_t left;																
chassis_t chassis_move;
															
PidTypeDef LegR_Pid;//右腿的腿长pd
PidTypeDef Tp_Pid;//防劈叉补偿pd
PidTypeDef Turn_Pid;//转向pd
PidTypeDef RollR_Pid;
uint32_t CHASSR_TIME=1;				
void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}

  ChassisR_init(&chassis_move,&right,&LegR_Pid);//初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
  Pensation_init(&Tp_Pid,&Turn_Pid);//补偿pid初始化
	roll_pid_init(&RollR_Pid);
	while(1)
	{	
		chassisR_feedback_update(&chassis_move,&right,&INS);//更新数据
		
	  chassisR_control_loop(&chassis_move,&right,&INS,LQR_K_R,&LegR_Pid);//控制计算
   
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T);//右边轮毂电机
			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1,chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//右边轮毂电机	
			osDelay(CHASSR_TIME);
		}
	
	}
}

void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legr)
{
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[0],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[1],8,MIT_MODE);//发送id为8
	
	wheel_motor_init(&chassis->wheel_motor[0],1,MIT_MODE);//发送id为1
	
	VMC_init(vmc);//给杆长赋值
	
	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	  osDelay(1);
	}

	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//右边轮毂电机
	  osDelay(1);
	}
}

void Pensation_init(PidTypeDef *Tp,PidTypeDef *turn)
{//补偿pid初始化：防劈叉补偿、偏航角补偿

	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

}

void roll_pid_init(PidTypeDef *roll_pid)
{
	const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	
	PID_init(roll_pid, PID_POSITION, roll, ROLL_PID_MAX_OUT,ROLL_PID_MAX_IOUT);
}
void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
  vmc->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[1].para.pos;
		
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[0];
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->roll=ins->Roll;
	chassis->theta_err=0.0f-(vmc->theta+left.theta);
	
	if(ins->Pitch<(3.1415926f/6.0f)&&ins->Pitch>(-3.1415926f/6.0f))
	{//根据pitch角度判断倒地自起是否完成
		chassis->recover_flag=0;
	}
}
uint32_t count_roll = 0; 
uint8_t right_flag=0;
extern uint8_t left_flag;
void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,PidTypeDef *leg)
{
	VMC_calc_1_right(vmcr,ins,((float)CHASSR_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是3*0.001秒
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
	}
		
	//chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算
  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点

	chassis->leg_tp=PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算
	
	chassis->wheel_motor[0].wheel_T=(LQR_K[0]*(vmcr->theta-0.0f)
																	+LQR_K[1]*(vmcr->d_theta-0.0f)
																	+LQR_K[2]*(chassis->x_filter-chassis->x_set)
																	+LQR_K[3]*(chassis->v_filter-chassis->v_set)
																	+LQR_K[4]*(chassis->myPithR-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroR-0.0f));
	
	//右边髋关节输出力矩				
	vmcr->Tp=(LQR_K[6]*(vmcr->theta-0.0f)
					+LQR_K[7]*(vmcr->d_theta-0.0f)
					+LQR_K[8]*(chassis->x_filter-chassis->x_set)
					+LQR_K[9]*(chassis->v_filter-chassis->v_set)
					+LQR_K[10]*(chassis->myPithR-0.0f)
					+LQR_K[11]*(chassis->myPithGyroR-0.0f));
				
	chassis->wheel_motor[0].wheel_T=chassis->wheel_motor[0].wheel_T-chassis->turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis->wheel_motor[0].wheel_T,-1.0f,1.0f);	
	
	vmcr->Tp=vmcr->Tp+chassis->leg_tp;//髋关节输出力矩


	chassis->now_roll_set = PID_Calc(&RollR_Pid,chassis->roll,chassis->roll_set);


//	vmcr->F0=13.0f/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
	jump_loop_r(chassis,vmcr,leg);
		
	right_flag=ground_detectionR(vmcr,ins);//右腿离地检测
	 
	 if(chassis->recover_flag==0)		
	 {//倒地自起不需要检测是否离地	 
		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
				chassis->wheel_motor[0].wheel_T=0.0f;
				vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				chassis->turn_set=chassis->total_yaw;
				vmcr->Tp=vmcr->Tp+chassis->leg_tp;		
		}
		else
		{//没有离地
			vmcr->leg_flag=0;//置为0
			
		}
	 }
	 else if(chassis->recover_flag==1)
	 {
		 vmcr->Tp=0.0f;
	 }	 
	 
	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 
	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩

	//额定扭矩
  mySaturate(&vmcr->torque_set[1],-3.0f,3.0f);	
	mySaturate(&vmcr->torque_set[0],-3.0f,3.0f);		
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}


void jump_loop_r(chassis_t *chassis,vmc_leg_t *vmcr,PidTypeDef *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_r == 0)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.07f) ;//前馈+pd
			if(vmcr->L0<0.1f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if(chassis->jump_status_r == 1)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.4f) ;//前馈+pd
			if(vmcr->L0>0.16f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if(chassis->jump_status_r == 2)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcr->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
		}

		if(chassis->jump_status_r == 3&&chassis->jump_status_l == 3)
		{
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
	}

}

