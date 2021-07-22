/*
 * @Descripttion: 裁判系统通信解析
 * @version: 参考深大RM2019代码修改
 * @Author: Xpj
 * @LastEditors: Xpj
 * @Date: 2020-08-10 15:56:10
 */
#include "judge.h"

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "crc.h"
#include "bsp_usart.h"
#include "bsp_judge.h"
#include "bsp_can.h"

uint8_t Judgement_Buf[JUDGEMENT_BUF_LEN];

/*-------------------------2021--------------------------------*/
#if JUDGE_VERSION == JUDGE_21


/***************RM2021*****************************/
/*****************系统数据定义**********************/
ext_game_status_t       			GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_HP_t          		GameRobotHP;				//0x0003
ext_dart_status_t					DartStatus;					//0x0004
ext_ICRA_buff_debuff_zone_status_t  IcraBuffDebuffZoneStatus;	//0x0005
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_referee_warning_t				RefereeWarning;				//0x0104
ext_dart_remaining_time_t			DartRemainingTime;			//0x0105
ext_game_robot_status_t				GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_t						BuffMusk;					//0x0204
aerial_robot_energy_t			AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207
ext_bullet_remaining_t				BulletRemaining;			//0x0208
ext_rfid_status_t					RfridStatus;				//0x0209
ext_dart_client_cmd_t				DartClientCmd;				//0x020A
ext_robot_interactive_t			Robot_Interactive;		//0x0301
ext_robot_command_t					Robot_Command;				//0x0303

xFrameHeader             			FrameHeader;		//发送帧头信息
/****************************************************/

bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID


/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = false;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
bool Power_Heat_Data_Updata=false;
bool Shoot_Update=false;
float Max_Shoot_Speed=0;
float Max_Chassic_Power=0;
#ifdef Test_Mode
	float Shoot_Speed[500]={0};
	float Shoot_Speed_Aver=0;
#endif
uint32_t Shoot_cnt=0;

/**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]); 
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001 //比赛状态数据
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002 //比赛结果数据
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_blood:       //0x0003 //比赛机器人血量
						memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
					break;
					
					case ID_darts_took_off:        //0X0004 //飞镖发射状态
						memcpy(&DartStatus,(ReadFromUsart + DATA),LEN_darts_took_off);
					break;

					case ID_game_icra_buff:			//0x0005 //人工智能挑战赛加成与惩罚区状态
						memcpy(&IcraBuffDebuffZoneStatus,(ReadFromUsart + DATA),LEN_game_icra_buff);
					break;
		
					case ID_event_data:    				//0x0101//场地事件//与空中有关
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
						Event_Decode(EventData.event_type);
					break;
					
					case ID_supply_projectile_action:   //0x0102 //场地补给站动作标识数据//字节偏移量为3的数据（大小一个字节）是补弹数量 50：50 颗子弹； 
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;

					case ID_judge_warning: //0X0104 //裁判系统警告
						memcpy(&RefereeWarning,(ReadFromUsart + DATA),LEN_judge_warning);
					break;
					
					case ID_dart_take_off_count_down: //0X0105//飞镖发射口倒计时
						memcpy(&DartRemainingTime,(ReadFromUsart + DATA),LEN_dart_take_off_count_down);
					break;

					case ID_game_robot_state:      		//0x0201//机器人状态数据
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202 //实时功率热量
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
						Power_Heat_Data_Updata=true;
						if(Max_Chassic_Power<PowerHeatData.chassis_power)
									Max_Chassic_Power=PowerHeatData.chassis_power;
					break;
					
					case ID_game_robot_pos:      		//0x0203 //机器人位置
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204 //机器人增益
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205 //空中机器人能量状态
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206 //伤害状态数据
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//装甲伤害扣血
						{
							Hurt_Data_Update = true;
							if(Gimbal_Position==0 && Aim==false)
							{
								if(RobotHurt.armor_id==0)
									Inspect_Position=5;
								else if(RobotHurt.armor_id==1)
									Inspect_Position=6;
							}
						}
					break;
					
					case ID_shoot_data:      			//0x0207//实时射击数据
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						Shoot_Update=true;
						++Shoot_cnt;
						#ifdef Test_Mode
							Shoot_Speed[Shoot_cnt-1]=ShootData.bullet_speed;
							for(uint32_t i=0; i<Shoot_cnt; ++i)
								Shoot_Speed_Aver+=Shoot_Speed[i];
							Shoot_Speed_Aver/=Shoot_cnt;
							if(Shoot_cnt>=499)
							{
								Shoot_cnt=0;
								memset(Shoot_Speed,0,500);
								Shoot_Speed_Aver=0;
							}
							if(Max_Shoot_Speed<ShootData.bullet_speed)
								Max_Shoot_Speed=ShootData.bullet_speed;
						#endif
					break;

					case ID_bullet_surplus:			//0x0208//弹丸剩余发射数
						memcpy(&BulletRemaining,(ReadFromUsart + DATA),LEN_bullet_surplus);
					break;

					case ID_robot_rfid_state:		//0x0209//机器人RFID状态
						memcpy(&RfridStatus,(ReadFromUsart + DATA),LEN_robot_rfid_state);
					break;

					case ID_darts_user_based_order:		//0x020A//飞镖客户端指令数据
						memcpy(&DartClientCmd,(ReadFromUsart + DATA),LEN_darts_user_based_order);
					break;
					
					case ID_robot_interactive:	//机器人交互
						memcpy(&Robot_Interactive,(ReadFromUsart + DATA),ReadFromUsart[DATA_LENGTH]);
						// Receive_Robot_Interactive();
					break;
					
					case ID_robot_command:	//客户端下发信息
						memcpy(&Robot_Command,(ReadFromUsart + DATA),LEN_robot_command);
						Robot_Command_Receive();
					break;
					
					default:
						break;
				}
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}
	
	return retval_tf;//对数据正误做处理
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//读取当前机器人ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_id1_17mm_cooling_limit;
}



/**
  * @brief  当前等级对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	return GameRobotStat.shooter_id1_17mm_cooling_rate;
}


bool Judge_If_Death(void)
{
	if(GameRobotStat.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


#elif JUDGE_VERSION == JUDGE_20


/***************RM2020*****************************/
/*****************系统数据定义**********************/
ext_game_status_t       			GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_HP_t          		GameRobotHP;				//0x0003
ext_dart_status_t					DartStatus;					//0x0004
ext_ICRA_buff_debuff_zone_status_t  IcraBuffDebuffZoneStatus;	//0x0005
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_referee_warning_t				RefereeWarning;				//0x0104
ext_dart_remaining_time_t			DartRemainingTime;			//0x0105
ext_game_robot_status_t				GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_t						BuffMusk;					//0x0204
ext_aerial_robot_energy_t			AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207
ext_bullet_remaining_t				BulletRemaining;			//0x0208
ext_rfid_status_t					RfridStatus;				//0x0209
ext_dart_client_cmd_t				DartClientCmd;				//0x020A

xFrameHeader             			FrameHeader;		//发送帧头信息
/****************************************************/

bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID


/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
#define BLUE  0
#define RED   1

/**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]); 
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001 //比赛状态数据
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002 //比赛结果数据
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_blood:       //0x0003 //比赛机器人血量
						memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
					break;
					
					case ID_darts_took_off:        //0X0004 //飞镖发射状态
						memcpy(&DartStatus,(ReadFromUsart + DATA),LEN_darts_took_off);
					break;

					case ID_game_icra_buff:			//0x0005 //人工智能挑战赛加成与惩罚区状态
						memcpy(&IcraBuffDebuffZoneStatus,(ReadFromUsart + DATA),LEN_game_icra_buff);
					break;
		
					case ID_event_data:    				//0x0101//场地事件//与空中有关
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102 //场地补给站动作标识数据//字节偏移量为3的数据（大小一个字节）是补弹数量 50：50 颗子弹； 
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;

					case ID_judge_warning: //0X0104 //裁判系统警告
						memcpy(&RefereeWarning,(ReadFromUsart + DATA),LEN_judge_warning);
					break;
					
					case ID_dart_take_off_count_down: //0X0105//飞镖发射口倒计时
						memcpy(&DartRemainingTime,(ReadFromUsart + DATA),LEN_dart_take_off_count_down);
					break;

					case ID_game_robot_state:      		//0x0201//机器人状态数据
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202 //实时功率热量
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
						// Vcan_Send(PowerHeatData.shooter_heat0);
					break;
					
					case ID_game_robot_pos:      		//0x0203 //机器人位置
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204 //机器人增益
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205 //空中机器人能量状态
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206 //伤害状态数据
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
						{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
					break;
					
					case ID_shoot_data:      			//0x0207//实时射击数据
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						//JUDGE_ShootNumCount();//发弹量统
					break;

					case ID_bullet_surplus:			//0x0208//弹丸剩余发射数
						memcpy(&BulletRemaining,(ReadFromUsart + DATA),LEN_bullet_surplus);
					break;

					case ID_robot_rfid_state:		//0x0209//机器人RFID状态
						memcpy(&RfridStatus,(ReadFromUsart + DATA),LEN_robot_rfid_state);
					break;

					case ID_darts_user_based_order:		//0x020A//飞镖客户端指令数据
						memcpy(&DartClientCmd,(ReadFromUsart + DATA),LEN_darts_user_based_order);
					break;
				}
				//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
//				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
//				{
//					//如果一个数据包出现了多帧数据,则再次读取
//					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
//				}
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}
	
	return retval_tf;//对数据正误做处理
}

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//读取当前机器人ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_heat0;
}

/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  实时热量
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_heat0_cooling_limit;
}



/**
  * @brief  当前等级对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级17mm冷却速度
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	return GameRobotStat.shooter_heat0_cooling_rate;
}


bool Judge_If_Death(void)
{
	if(GameRobotStat.remain_HP == 0 && JUDGE_sGetDataState() == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

#endif
