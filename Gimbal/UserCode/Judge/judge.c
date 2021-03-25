#include "judge.h"

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
#include "crc.h"


/*-------------------------2018--------------------------------*/
#if JUDGE_VERSION == JUDGE_18
/*****************ϵͳ���ݶ���**********************/
extGameRobotState_t       RobotState;		//0x0001
extRobotHurt_t            HurtData;			//0x0002
extShootData_t            ShootData;		//0x0003
extPowerHeatData_t        PowerHeatData;	//0x0004
extRfidDetect_t           RfidDetect;		//0x0005
extGameResult_t			  GameResultData;	//0x0006
extGetBuff_t			  GetBuffData;		//0x0007
extGameRobotPos_t		  GameRobotPosData;	//0x0008


xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
xShowData                 ShowData;
/****************************************************/

bool Judge_Data_TF = FALSE;//���������Ƿ����,������������

/**************����ϵͳ���ݸ���****************/
uint16_t ShootNum;//ͳ�Ʒ�����,0x0003����һ������Ϊ������һ��
bool Hurt_Data_Update = FALSE;//װ�װ��˺������Ƿ����,ÿ��һ���˺���TRUE,Ȼ��������FALSE,������������


//��ǰ�ȼ���Ӧ����������,18���
#define HEAT_LEVEL1 120         //240
#define HEAT_LEVEL2 240         //360
#define HEAT_LEVEL3 480         //480

//��ǰ�ȼ���Ӧ��ǹ����ȴ
#define COLD_LEVEL1 120         //40
#define COLD_LEVEL2 240         //60
#define COLD_LEVEL3 480         //80

portTickType shoot_time;//������ʱ����

portTickType shoot_ping;//����������շ����ӳ�

/*********************************����ϵͳ���ݶ�ȡ**************************************/

/**
  * @brief  ��ȡ��������,loop��ѭ�����ô˺�������ȡ����
  * @param  ��������
  * @retval �Ƿ�������ж�������
  * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д������
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
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//��У�������˵�����ݿ���
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
				switch(CmdID)
				{
					case ID_STATE:         //0x0001
						memcpy(&RobotState, (ReadFromUsart + DATA), LEN_STATE);
					break;
					
					case ID_HURT:          //0x0002
						memcpy(&HurtData, (ReadFromUsart + DATA), LEN_HURT);
						if(HurtData.hurtType == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
						
					break;
					
					case ID_SHOOT:         //0x0003
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_SHOOT);
						JUDGE_ShootNumCount();//������ͳ��,��������˫ǹ��,��׼
					break;
					
					case ID_POWER_HEAT:    //0x0004
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_POWER_HEAT);
					break;
					
					case ID_RFID:          //0x0005
						memcpy(&RfidDetect, (ReadFromUsart + DATA), LEN_RFID);
					break;
					
					case ID_GAME_RESULT:   //0x0006
						memcpy(&GameResultData, (ReadFromUsart + DATA), LEN_GAME_RESULT);
					break;
					
					case ID_BUFF_GET:      //0x0007
						memcpy(&GetBuffData, (ReadFromUsart + DATA), LEN_BUFF_GET);
					break;
					
					case ID_POSITION:      //0x0008
						memcpy(&GameRobotPosData, (ReadFromUsart + DATA), LEN_POSITION);
					break;
				}
				
				//�׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�����ж�һ�����ݰ��Ƿ��ж�֡����
				if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL) == 0xA5)
				{
					//���һ�����ݰ������˶�֡����,���ٴζ�ȡ
					Judge_Read_Data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + FrameHeader.DataLength + LEN_TAIL);
				}
			}
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


/**************************�û��Զ��������ϴ����ͻ���******************************/

/**
  * @brief  �ϴ��Զ�������
  * @param  void
  * @retval void
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void JUDGE_Show_Data(void)
{
	//��ʵ����22�ͺ�,������ռ�Ҳ����ν
	uint8_t Show_Pack[50] = {0};//���ȴ���ͷ5+ָ��2+����13+β2����,�����u8��Ϊ�˺ü����ֽ���
	int i;//ѭ�����ʹ���
	
	//֡ͷЭ��
	FrameHeader.SOF        = JUDGE_FRAME_HEADER;
	FrameHeader.Seq        = 0;		//Ϊʲô��0,�ٷ�����û��˵
	FrameHeader.DataLength = LEN_SHOW;
	
	//д��֡ͷ
	memcpy( Show_Pack, &FrameHeader, LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum( Show_Pack, LEN_HEADER );
	
	//д��������
	ShowData.CmdID = ID_SHOW;
	
	//д������
	ShowData.data1 = Capvoltage_Percent();//��ʾ����ʣ�����      //(float)ShootNum;//��һ����ʾ���Ƿ�����
	ShowData.data2 = 66.66;//Fric_GetHeatInc();//�ڶ�����ʾ��ǰ����(Ŀ������)
	ShowData.data3 = 88.88;
	ShowData.mask  = 0x00;//��ȫ��,��6λ��Ч
	
	if(VISION_IfAutoRed() == TRUE)
	{
		ShowData.mask &= 0x1f;//��6λ��0
	}
	else if(VISION_IfAutoRed() == FALSE)
	{
		ShowData.mask |= 0x20;//��6λ��1
	}
	
	
	memcpy( Show_Pack + LEN_HEADER, &ShowData, (LEN_CMDID + LEN_SHOW) );
	
	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum( Show_Pack, (LEN_HEADER + LEN_CMDID + LEN_SHOW + LEN_TAIL) );
	
	//������õ�����ͨ��������λ���͵�����ϵͳ
	for (i = 0; i < (LEN_HEADER + LEN_CMDID + LEN_SHOW + LEN_TAIL); i++)
	{
		UART5_SendChar( Show_Pack[i] );
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
	return (PowerHeatData.chassisPower);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���60)
  * @attention  
  */
float JUDGE_fGetRemainEnergy(void)
{
	return (PowerHeatData.chassisPowerBuffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	RobotState.robotLevel;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval 17mm
  * @attention  
  */
uint16_t JUDGE_usGetRemoteHeat17(void)
{
	return PowerHeatData.shooterHeat0;
}

/**
  * @brief  ͳ�Ʒ�����
  * @param  void
  * @retval void
  * @attention  
  */
void JUDGE_ShootNumCount(void)
{
	ShootNum++;
	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
uint16_t JUDGE_usGetShootNum(void)
{
	return ShootNum;
}

/**
  * @brief  ����������
  * @param  void
  * @retval void
  * @attention 
  */
void JUDGE_ShootNum_Clear(void)
{
	ShootNum = 0;
}

/**
  * @brief  ��ȡǹ������
  * @param  void
  * @retval ��ǰ�ȼ���������
  * @attention  
  */
uint16_t JUDGE_usGetHeatLimit(void)
{
	if (RobotState.robotLevel == 1)//1��
	{
		return HEAT_LEVEL1;
	}
	else if (RobotState.robotLevel == 2)//2��
	{
		return HEAT_LEVEL2;
	}
	else if (RobotState.robotLevel == 3)//3��
	{
		return HEAT_LEVEL3;
	}
	else//��ֹ����������,ǿ���޵���С
	{
		return HEAT_LEVEL1;
	}
}

/**
  * @brief  ��ǰ�ȼ���Ӧ��ǹ��ÿ����ȴֵ
  * @param  void
  * @retval ��ǰ�ȼ���ȴ�ٶ�
  * @attention  
  */
uint16_t JUDGE_usGetShootCold(void)
{
	if (RobotState.robotLevel == 1)//1��
	{
		return COLD_LEVEL1;
	}
	else if (RobotState.robotLevel == 2)//2��
	{
		return COLD_LEVEL2;
	}
	else if (RobotState.robotLevel == 3)//3��
	{
		return COLD_LEVEL3;
	}
	else//��ֹ����������,ǿ���޵���С
	{
		return COLD_LEVEL1;
	}
}

/****************�����Զ������ж���*******************/
/**
  * @brief  װ�װ��˺������Ƿ����
  * @param  void
  * @retval TRUE�Ѹ���   FALSEû����
  * @attention  
  */
bool JUDGE_IfArmorHurt(void)
{
	static portTickType ulCurrent = 0;
	static uint32_t ulDelay = 0;
	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

	
	ulCurrent = xTaskGetTickCount();

	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
	{
		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
		ulDelay = ulCurrent + TIME_STAMP_200MS;//
		IfHurt = TRUE;
	}
	else if (ulCurrent > ulDelay)//
	{
		IfHurt = FALSE;
	}
	
	return IfHurt;
}



/************************��������Ԥ���ú���****************************/

/**
  * @brief  ����ʣ�๦��
  * @param  void
  * @retval ʣ�๦��W
  * @attention  ����ʼ�ʵʱ����,�������ݱ���
  */
float JUDGE_fGetChassisResiduePower(void)
{
	return (80 - PowerHeatData.chassisPower);
}

/**
  * @brief  ���㹩���������ݵĵ���ֵ
  * @param  void
  * @retval ��������ֵ
  * @attention  �������ݱ���
  */
float JUDGE_fGetSuper_Cap_Ele(void)
{
	return ((80 - PowerHeatData.chassisPower) / PowerHeatData.chassisCurrent);
}


/*-------------------------2019--------------------------------*/
#elif JUDGE_VERSION == JUDGE_19

/*****************ϵͳ���ݶ���**********************/
ext_game_state_t       				GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_survivors_t          GameRobotSurvivors;			//0x0003
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_supply_projectile_booking_t		SupplyProjectileBooking;	//0x0103
ext_game_robot_state_t			  	GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_musk_t						BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207

xFrameHeader              FrameHeader;		//����֡ͷ��Ϣ
ext_SendClientData_t      ShowData;			//�ͻ�����Ϣ
ext_CommunatianData_t     CommuData;		//����ͨ����Ϣ
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
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

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
					
					case ID_game_robot_survivors:       //0x0003 //���������˴������
						memcpy(&GameRobotSurvivors, (ReadFromUsart + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101//�����¼�//������й�
						memcpy(&EventData, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102 //���ز���վ������ʶ����//�ֽ�ƫ����Ϊ3�����ݣ���Сһ���ֽڣ��ǲ������� 50��50 ���ӵ��� 
						memcpy(&SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103//���󲹸�վ�������ݣ��ɲ����ӷ���
						memcpy(&SupplyProjectileBooking, (ReadFromUsart + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201//������״̬����
						memcpy(&GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&BuffMusk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(RobotHurt.hurt_type == 0)//��װ�װ���������˺�
						{	Hurt_Data_Update = TRUE;	}//װ������ÿ����һ�����ж�Ϊ�ܵ�һ���˺�
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
						//JUDGE_ShootNumCount();//������ͳ
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

///**
//  * @brief  �ϴ��Զ�������
//  * @param  void
//  * @retval void
//  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
//  */
//#define send_max_len     200
//unsigned char CliendTxBuffer[send_max_len];
//void JUDGE_Show_Data(void)
//{
//	static u8 datalength,i;
//	uint8_t judge_led = 0xff;//��ʼ��ledΪȫ��
//	static uint8_t auto_led_time = 0;
//	static uint8_t buff_led_time = 0;
//	
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	
//	ShowData.txFrameHeader.SOF = 0xA5;
//	ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);
//	ShowData.txFrameHeader.Seq = 0;
//	memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//д��֡ͷ����
//	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(xFrameHeader));//д��֡ͷCRC8У����
//	
//	ShowData.CmdID = 0x0301;
//	
//	ShowData.dataFrameHeader.data_cmd_id = 0xD180;//�����ͻ��˵�cmd,�ٷ��̶�
//	//ID�Ѿ����Զ���ȡ����
//	ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//�����ߵ�ID
//	ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	
//	/*- �Զ������� -*/
//	ShowData.clientData.data1 = (float)Capvoltage_Percent();//����ʣ�����
//	ShowData.clientData.data2 = (float)Base_Angle_Measure();//����ǶȲ�
//	ShowData.clientData.data3 = GIMBAL_PITCH_Judge_Angle();//��̨̧ͷ�Ƕ�
//	
//	/***************Ť��ָʾ****************/
//	if(Chassis_IfCORGI() == TRUE)//Ťƨ��ģʽ,��һ�ŵ�����
//	{
//		judge_led &= 0xfe;//��1λ��0,���
//	}
//	else
//	{
//		judge_led |= 0x01;//��1λ��1
//	}
//	
//	/****************45��ģʽ****************/
//	if(Chassis_IfPISA() == TRUE)//45��ģʽ���ڶ��ŵ�����
//	{
//		judge_led &= 0xfd;//��2λ��0,���
//	}
//	else
//	{
//		judge_led |= 0x02;//��2λ��1
//	}
//	
//	/***************����ָʾ****************/
//	if(VISION_isColor() == ATTACK_RED)//�����
//	{
//		judge_led &= 0xfb;//��3λ��0,���
//	}
//	else if(VISION_isColor() == ATTACK_BLUE)//������ɫ
//	{
//		judge_led |= 0x04;//��3λ��1
//	}
//	else//������
//	{
//		auto_led_time++;
//		if(auto_led_time > 3)
//		{
//			(judge_led)^=(1<<2);//��3λȡ��,��˸
//			auto_led_time = 0;
//		}
//	}
//	
//	/****************���ָʾ***************/
//	if(VISION_BuffType() == VISION_RBUFF_CLOCKWISE	//˳ʱ��
//			|| VISION_BuffType() == VISION_BBUFF_CLOCKWISE)
//	{
//		judge_led &= 0xf7;//��4λ��0,���
//	}
//	else if(VISION_BuffType() == VISION_RBUFF_ANTI	//��ʱ��
//			|| VISION_BuffType() == VISION_BBUFF_ANTI)
//	{
//		judge_led |= 0x08;//��4λ��1
//	}
//	else//�����
//	{
//		buff_led_time++;
//		if(buff_led_time > 3)
//		{
//			(judge_led)^=(1<<3);//��4λȡ��,��˸
//			buff_led_time = 0;
//		}
//	}
//	/*--------------*/
//	ShowData.clientData.masks = judge_led;//0~5λ0���,1�̵�
//	
//	//���д�����ݶ�
//	memcpy(	
//			CliendTxBuffer + 5, 
//			(uint8_t*)&ShowData.CmdID, 
//			(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
//		  );			
//			
//	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(ShowData));//д�����ݶ�CRC16У����	

//	datalength = sizeof(ShowData); 
//	for(i = 0;i < datalength;i++)
//	{
//		USART_SendData(UART5,(uint16_t)CliendTxBuffer[i]);
//		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
//	}	 
//}

///**
//  * @brief  �������ݸ�����
//  * @param  void
//  * @retval void
//  * @attention  
//  */
//#define Teammate_max_len     200
//unsigned char TeammateTxBuffer[Teammate_max_len];
//bool Send_Color = 0;
//bool First_Time_Send_Commu = FALSE;
//uint16_t send_time = 0;
//void Send_to_Teammate(void)
//{
//	static u8 datalength,i;
//	
//	Send_Color = is_red_or_blue();//�жϷ��͸��ڱ�����ɫ,17�ڱ�(��),7�ڱ�(��)��
//	
//	memset(TeammateTxBuffer,0,200);
//	
//	CommuData.txFrameHeader.SOF = 0xA5;
//	CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
//	CommuData.txFrameHeader.Seq = 0;
//	memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
//	Append_CRC8_Check_Sum(TeammateTxBuffer, sizeof(xFrameHeader));	
//	
//	CommuData.CmdID = 0x0301;
//	
//	   
//	CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//�����ߵ�ID
//	
//	
//	if( Senty_Run() == TRUE)
//	{
//		Senty_Run_Clean_Flag();
//		First_Time_Send_Commu = TRUE;
//	}
//	
//	if( First_Time_Send_Commu == TRUE )
//	{
//		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//��0x0200-0x02ff֮��ѡ��
//		send_time++;
//		if(send_time >= 20)
//		{
//			First_Time_Send_Commu = FALSE;
//		}
//		if(Send_Color == BLUE)//�Լ��������������ڱ�
//		{
//			CommuData.dataFrameHeader.receiver_ID = 17;//������ID
//		}
//		else if(Send_Color == RED)//�Լ��Ǻ죬�������ڱ�
//		{
//			CommuData.dataFrameHeader.receiver_ID = 7;//������ID
//		}
//	}
//	else
//	{
//		CommuData.dataFrameHeader.data_cmd_id = 0x0255;
//		send_time = 0;
//		CommuData.dataFrameHeader.receiver_ID = 88;//������ID��������
//	}
//	
//	CommuData.interactData.data[0] = 0;//���͵����� //��С��Ҫ���������ı�������   
//	
//	memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
//	Append_CRC16_Check_Sum(TeammateTxBuffer,sizeof(CommuData));
//	
//	datalength = sizeof(CommuData); 
//	if( First_Time_Send_Commu == TRUE )
//	{
//		for(i = 0;i < datalength;i++)
//		{
//			USART_SendData(UART5,(uint16_t)TeammateTxBuffer[i]);
//			while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
//		}	 
//	}
//}

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

/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
//void determine_ID(void)
//{
//	Color = is_red_or_blue();
//	if(Color == BLUE)
//	{
//		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//����ͻ���ID
//	}
//	else if(Color == RED)
//	{
//		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
//	}
//}

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

///**
//  * @brief  ͳ�Ʒ�����
//  * @param  void
//  * @retval void
//  * @attention  
//  */
//portTickType shoot_time;//������ʱ����
//portTickType shoot_ping;//����������շ����ӳ�
//float Shoot_Speed_Now = 0;
//float Shoot_Speed_Last = 0;
//void JUDGE_ShootNumCount(void)
//{
//	Shoot_Speed_Now = ShootData.bullet_speed;
//	if(Shoot_Speed_Last != Shoot_Speed_Now)//��Ϊ��float�ͣ�������������ȫ���,�����ٶȲ���ʱ˵��������һ�ŵ�
//	{
//		ShootNum++;
//		Shoot_Speed_Last = Shoot_Speed_Now;
//	}
//	shoot_time = xTaskGetTickCount();//��ȡ���跢��ʱ��ϵͳʱ��
//	shoot_ping = shoot_time - REVOL_uiGetRevolTime();//�����ӳ�
//}

/**
  * @brief  ��ȡ������
  * @param  void
  * @retval ������
  * @attention ��������˫ǹ��
  */
//uint16_t JUDGE_usGetShootNum(void)
//{
//	return ShootNum;
//}

///**
//  * @brief  ����������
//  * @param  void
//  * @retval void
//  * @attention 
//  */
//void JUDGE_ShootNum_Clear(void)
//{
//	ShootNum = 0;
//}

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

///****************�����Զ������ж���*******************/
///**
//  * @brief  װ�װ��˺������Ƿ����
//  * @param  void
//  * @retval TRUE�Ѹ���   FALSEû����
//  * @attention  
//  */
//bool JUDGE_IfArmorHurt(void)
//{
//	static portTickType ulCurrent = 0;
//	static uint32_t ulDelay = 0;
//	static bool IfHurt = FALSE;//Ĭ��װ�װ崦������״̬

//	
//	ulCurrent = xTaskGetTickCount();

//	if (Hurt_Data_Update == TRUE)//װ�װ����ݸ���
//	{
//		Hurt_Data_Update = FALSE;//��֤���жϵ��´�װ�װ��˺�����
//		ulDelay = ulCurrent + TIME_STAMP_200MS;//
//		IfHurt = TRUE;
//	}
//	else if (ulCurrent > ulDelay)//
//	{
//		IfHurt = FALSE;
//	}
//	
//	return IfHurt;
//}

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
