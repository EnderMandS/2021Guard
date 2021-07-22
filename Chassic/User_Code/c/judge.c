/*
 * @Descripttion: ����ϵͳͨ�Ž���
 * @version: �ο����RM2019�����޸�
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
/*****************ϵͳ���ݶ���**********************/
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

xFrameHeader             			FrameHeader;		//����֡ͷ��Ϣ
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID


/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = false;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
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
  * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]); 
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001 //����״̬����
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002 //�����������
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_blood:       //0x0003 //����������Ѫ��
						memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
					break;
					
					case ID_darts_took_off:        //0X0004 //���ڷ���״̬
						memcpy(&DartStatus,(ReadFromUsart + DATA),LEN_darts_took_off);
					break;

					case ID_game_icra_buff:			//0x0005 //�˹�������ս���ӳ���ͷ���״̬
						memcpy(&IcraBuffDebuffZoneStatus,(ReadFromUsart + DATA),LEN_game_icra_buff);
					break;
		
					case ID_event_data:    				//0x0101//�����¼�//������й�
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
						Event_Decode(EventData.event_type);
					break;
					
					case ID_supply_projectile_action:   //0x0102 //���ز���վ������ʶ����//�ֽ�ƫ����Ϊ3�����ݣ���Сһ���ֽڣ��ǲ������� 50��50 ���ӵ��� 
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;

					case ID_judge_warning: //0X0104 //����ϵͳ����
						memcpy(&RefereeWarning,(ReadFromUsart + DATA),LEN_judge_warning);
					break;
					
					case ID_dart_take_off_count_down: //0X0105//���ڷ���ڵ���ʱ
						memcpy(&DartRemainingTime,(ReadFromUsart + DATA),LEN_dart_take_off_count_down);
					break;

					case ID_game_robot_state:      		//0x0201//������״̬����
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202 //ʵʱ��������
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
						Power_Heat_Data_Updata=true;
						if(Max_Chassic_Power<PowerHeatData.chassis_power)
									Max_Chassic_Power=PowerHeatData.chassis_power;
					break;
					
					case ID_game_robot_pos:      		//0x0203 //������λ��
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204 //����������
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205 //���л���������״̬
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206 //�˺�״̬����
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//װ���˺���Ѫ
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
					
					case ID_shoot_data:      			//0x0207//ʵʱ�������
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

					case ID_bullet_surplus:			//0x0208//����ʣ�෢����
						memcpy(&BulletRemaining,(ReadFromUsart + DATA),LEN_bullet_surplus);
					break;

					case ID_robot_rfid_state:		//0x0209//������RFID״̬
						memcpy(&RfridStatus,(ReadFromUsart + DATA),LEN_robot_rfid_state);
					break;

					case ID_darts_user_based_order:		//0x020A//���ڿͻ���ָ������
						memcpy(&DartClientCmd,(ReadFromUsart + DATA),LEN_darts_user_based_order);
					break;
					
					case ID_robot_interactive:	//�����˽���
						memcpy(&Robot_Interactive,(ReadFromUsart + DATA),ReadFromUsart[DATA_LENGTH]);
						// Receive_Robot_Interactive();
					break;
					
					case ID_robot_command:	//�ͻ����·���Ϣ
						memcpy(&Robot_Command,(ReadFromUsart + DATA),LEN_robot_command);
						Robot_Command_Receive();
					break;
					
					default:
						break;
				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}
	
	return retval_tf;//����������������
}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_id1_17mm_cooling_heat;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_id1_17mm_cooling_limit;
}



/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
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
/*****************ϵͳ���ݶ���**********************/
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

xFrameHeader             			FrameHeader;		//����֡ͷ��Ϣ
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������
uint8_t Judge_Self_ID;//��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;//�����߻����˶�Ӧ�Ŀͻ���ID


/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������
#define BLUE  0
#define RED   1

/**
  * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
  */
bool Judge_Read_Data(uint8_t *ReadFromUsart)
{
	bool retval_tf = FALSE;//������ȷ����־,ÿ�ε��ö�ȡ����ϵͳ���ݺ�������Ĭ��Ϊ����
	
	uint16_t judge_length;//ͳ��һ֡���ݳ��� 
	
	int CmdID = 0;//�������������
	
	/***------------------*****/
	//�����ݰ��������κδ���
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//д��֡ͷ����,�����ж��Ƿ�ʼ�洢��������
	memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//ͳ��һ֡���ݳ���,����CR16У��
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]); 
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001 //����״̬����
						memcpy(&GameState, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002 //�����������
						memcpy(&GameResult, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_blood:       //0x0003 //����������Ѫ��
						memcpy(&GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_HP);
					break;
					
					case ID_darts_took_off:        //0X0004 //���ڷ���״̬
						memcpy(&DartStatus,(ReadFromUsart + DATA),LEN_darts_took_off);
					break;

					case ID_game_icra_buff:			//0x0005 //�˹�������ս���ӳ���ͷ���״̬
						memcpy(&IcraBuffDebuffZoneStatus,(ReadFromUsart + DATA),LEN_game_icra_buff);
					break;
		
					case ID_event_data:    				//0x0101//�����¼�//������й�
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102 //���ز���վ������ʶ����//�ֽ�ƫ����Ϊ3�����ݣ���Сһ���ֽڣ��ǲ������� 50��50 ���ӵ��� 
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;

					case ID_judge_warning: //0X0104 //����ϵͳ����
						memcpy(&RefereeWarning,(ReadFromUsart + DATA),LEN_judge_warning);
					break;
					
					case ID_dart_take_off_count_down: //0X0105//���ڷ���ڵ���ʱ
						memcpy(&DartRemainingTime,(ReadFromUsart + DATA),LEN_dart_take_off_count_down);
					break;

					case ID_game_robot_state:      		//0x0201//������״̬����
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202 //ʵʱ��������
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
						// Vcan_Send(PowerHeatData.shooter_heat0);
					break;
					
					case ID_game_robot_pos:      		//0x0203 //������λ��
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204 //����������
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205 //���л���������״̬
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206 //�˺�״̬����
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      			//0x0207//ʵʱ�������
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						//JUDGE_ShootNumCount();//������ͳ
					break;

					case ID_bullet_surplus:			//0x0208//����ʣ�෢����
						memcpy(&BulletRemaining,(ReadFromUsart + DATA),LEN_bullet_surplus);
					break;

					case ID_robot_rfid_state:		//0x0209//������RFID״̬
						memcpy(&RfridStatus,(ReadFromUsart + DATA),LEN_robot_rfid_state);
					break;

					case ID_darts_user_based_order:		//0x020A//���ڿͻ���ָ������
						memcpy(&DartClientCmd,(ReadFromUsart + DATA),LEN_darts_user_based_order);
					break;
				}
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
//				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
//				{
//					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
//					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
//				}
			}
		}
		//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
			Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
		}
	}
	
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//����������
	}
	else		//ֻҪCRC16У�鲻ͨ����ΪFALSE
	{
		Judge_Data_TF = FALSE;//����������
	}
	
	return retval_tf;//����������������
}

/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//��ȡ��ǰ������ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  �����Ƿ����
  * @param  void
  * @retval  TRUE����   FALSE������
  * @attention  �ڲ��ж�ȡ������ʵʱ�ı䷵��ֵ
  */
bool JUDGE_sGetDataState(void)
{
	return Judge_Data_TF;
}

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_fGetChassisPower(void)
{
	return (PowerHeatData.chassis_power);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	GameRobotStat.robot_level;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooter_heat0;
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm
  * @attention  ʵʱ����
  */
float JUDGE_usGetSpeedHeat17(void)
{
	return ShootData.bullet_speed;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	return GameRobotStat.shooter_heat0_cooling_limit;
}



/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ�17mm��ȴ�ٶ�
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
