#ifndef _JUDGE_H
#define _JUDGE_H


#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
//����ϵͳ�汾Ԥ����
#define		JUDGE_18		18
#define		JUDGE_19		19
#define		JUDGE_20		20
#define 	JUDGE_21		21
#define		JUDGE_VERSION	JUDGE_21

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1

#define JUDGEMENT_BUF_LEN      200
extern uint8_t Judgement_Buf[JUDGEMENT_BUF_LEN];
/*-------------------------2018--------------------------------*/
#if JUDGE_VERSION == JUDGE_18

//2018����ϵͳ�ٷ��ӿ�Э��


//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16


/* RFID������ */
#define    CARD_ATTACK        ((uint8_t)0x00)
#define    CARD_PROTECT       ((uint8_t)0x01)
#define    CARD_BLOOD_RED     ((uint8_t)0x02)
#define    CARD_BLOOD_BLUE    ((uint8_t)0x03)
#define    CARD_HEAL_RED      ((uint8_t)0x04)
#define    CARD_HEAL_BLUE     ((uint8_t)0x05)
#define    CARD_COLD_RED      ((uint8_t)0x06)
#define    CARD_COLD_BLUE     ((uint8_t)0x07)
#define    CARD_FORT          ((uint8_t)0x08)



typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;


//frame header��ʽ

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)


//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;



/***************������ID********************/

/* 

   ID: 0x0001  Byte:  8    ����������״̬       ����Ƶ�� 10Hz      
   ID: 0x0002  Byte:  1    �˺�����             �ܵ��˺�ʱ����      
   ID: 0x0003  Byte:  6    ʵʱ�������         ���䵯��ʱ����  
   ID: 0x0004  Byte: 20    ʵʱ���ʺ���������   ����Ƶ�� 50Hz   20MS����һ��
   ID: 0x0005  Byte:  2    ʵʱ���ؽ�������     ��⵽RFID��ʱ ����Ƶ�� 10Hz 
   ID: 0X0006  Byte:  1    ����ʤ������         ��������ʱ����һ�� 
   ID: 0X0007  Byte:  2    Buff ��ȡ����        ������غ���һ��
   ID: 0X0008  Byte: 16    ������λ�ó�����Ϣ   ����Ƶ�� 50Hz        
   ID: 0x0100  Byte: 13    �Զ�������           ����Ƶ�� 10Hz
*/


//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_STATE       = 0x0001,
	ID_HURT 	   = 0x0002,
	ID_SHOOT       = 0x0003,
	ID_POWER_HEAT  = 0x0004,
	ID_RFID        = 0x0005,
	ID_GAME_RESULT = 0x0006,
	ID_BUFF_GET    = 0x0007,
	ID_POSITION    = 0x0008,
	ID_SHOW        = 0x0100,

} CmdID;


//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
	LEN_STATE       =  8,	//0x0001
	LEN_HURT        =  1,	//0x0002
	LEN_SHOOT       =  6,	//0x0003
	LEN_POWER_HEAT  = 20,	//0x0004
	LEN_RFID        =  2,	//0x0005
	LEN_GAME_RESULT =  1,	//0x0006
	LEN_BUFF_GET    =  2,	//0x0007
	LEN_POSITION    = 16,	//0x0008
	LEN_SHOW        = 13,	//0x0100
	
} JudgeDataLength;




     



/* ID: 0x0001  Byte:  8 */
typedef __packed struct
{
	uint16_t stageRemainTime;
	uint8_t  gameProcess;
	uint8_t  robotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	
} extGameRobotState_t;


/* ID: 0x0002  Byte:  1 */
typedef __packed struct
{
	uint8_t armorType :4;
	uint8_t hurtType  :4;

} extRobotHurt_t;	


/* ID: 0x0003  Byte:  6 */
typedef __packed struct
{
	uint8_t bulletType;
	uint8_t bulletFreq;
	float   bulletSpeed;
	
} extShootData_t;


/* ID: 0x0004  Byte: 20 */
typedef __packed struct
{
	float chassisVolt;
	float chassisCurrent;
	float chassisPower;//˲ʱ����
	float chassisPowerBuffer;//60������������
	uint16_t shooterHeat0;//17mm
	uint16_t shooterHeat1;//42mm
	
} extPowerHeatData_t;


/* ID: 0x0005  Byte:  2 */
typedef __packed struct
{
	uint8_t cardType;
	uint8_t cardIdx;
	
} extRfidDetect_t;


/* ID: 0X0006  Byte:  1 */
typedef __packed struct
{
	uint8_t winner;
	
} extGameResult_t;


/* ID: 0X0007  Byte:  2 */
typedef __packed struct
{
	uint8_t buffType;
	uint8_t buffAddition;
	
} extGetBuff_t;


/* ID: 0X0008  Byte: 16 */
typedef __packed struct
{
	float  x;
	float  y;
	float  z;
	float  yaw;
	
} extGameRobotPos_t;


/* ID: 0x0100  Byte: 13 */
typedef __packed struct
{
	uint16_t CmdID;//������ 0x0100
	float data1;//������ʾ������ֵ
	float data2;
	float data3;
	uint8_t mask;//��6λ��Ӧ6��ָʾ��
	
} xShowData;


typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;


/**********����ϵͳ���ݶ�ȡ*************/
bool Judge_Read_Data(uint8_t *ReadFormUsart);//��ѭ��4ms����һ��

/****�û��Զ��������ϴ����ͻ���****/
void JUDGE_Show_Data(void);

/*****�������ݸ����жϺ���********/
bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
float JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);

void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);

uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);

/*******�����Զ������ж���******/
bool JUDGE_IfArmorHurt(void);

/*****��������Ԥ���ú���******/
float JUDGE_fGetChassisResiduePower(void);
float JUDGE_fGetSuper_Cap_Ele(void);


/*-------------------------2019--------------------------------*/
#elif JUDGE_VERSION == JUDGE_19

#define    LEN_HEADER    5        //֡ͷ��
#define    LEN_CMDID     2        //�����볤��
#define    LEN_TAIL      2	      //֡βCRC16


//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5�ֽ�֡ͷ,ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************������ID********************/

/* 

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1    �����������         		������������      
	ID: 0x0003  Byte:  2    ���������˴������   		1Hz����  
	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı���� 
	ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ�����      �����ӷ��ͣ�10Hz 
	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz       
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
	
*/





//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_state       			= 0x0001,//����״̬����
	ID_game_result 	   				= 0x0002,//�����������
	ID_game_robot_survivors       	= 0x0003,//���������˴������
	ID_event_data  					= 0x0101,//�����¼����� 
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_supply_projectile_booking 	= 0x0103,//���ز���վԤԼ�ӵ�����
	ID_game_robot_state    			= 0x0201,//������״̬����
	ID_power_heat_data    			= 0x0202,//ʵʱ������������
	ID_game_robot_pos        		= 0x0203,//������λ������
	ID_buff_musk					= 0x0204,//��������������
	ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
	ID_robot_hurt					= 0x0206,//�˺�״̬����
	ID_shoot_data					= 0x0207,//ʵʱ�������

} CmdID;


//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors       		=  2,	//0x0003
	LEN_event_data  					=  4,	//0x0101
	LEN_supply_projectile_action        =  3,	//0x0102
	LEN_supply_projectile_booking		=  2,	//0x0103
	LEN_game_robot_state    			= 15,	//0x0201
	LEN_power_heat_data   				= 14,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  3,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  6,	//0x0207
	
} JudgeDataLength;

/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

/* ID: 0x0001  Byte:  3    ����״̬���� */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    ����������� */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    ���������˴������ */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    �����¼����� */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    ���ز���վ������ʶ���� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ����� */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0201  Byte: 15    ������״̬���� */
typedef __packed struct 
{ 
	uint8_t robot_id;   //������ID��������У�鷢��
	uint8_t robot_level;  //1һ����2������3����
	uint16_t remain_HP;  //������ʣ��Ѫ��
	uint16_t max_HP; //��������Ѫ��
	uint16_t shooter_heat0_cooling_rate;  //������ 17mm �ӵ�������ȴ�ٶ� ��λ /s
	uint16_t shooter_heat0_cooling_limit;   // ������ 17mm �ӵ���������
	uint16_t shooter_heat1_cooling_rate;   //������ 42mm �ӵ�������ȴ�ٶ�
	uint16_t shooter_heat1_cooling_limit;   //������ 42mm �ӵ���������
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; //���ص�Դ������
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 14    ʵʱ������������ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   
	uint16_t chassis_current;    
	float chassis_power;   //˲ʱ���� 
	uint16_t chassis_power_buffer;//60������������
	uint16_t shooter_heat0;//17mm
	uint16_t shooter_heat1;  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    ������λ������ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    �������������� */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    ���л���������״̬���� */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    �˺�״̬���� */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    ʵʱ������� */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 


/* 
	
	�������ݣ�����һ��ͳһ�����ݶ�ͷ�ṹ��
	���������� ID���������Լ������ߵ� ID ���������ݶΣ�
	�����������ݵİ��ܹ������Ϊ 128 ���ֽڣ�
	��ȥ frame_header,cmd_id,frame_tail �Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ�
	�ʶ����͵��������ݶ����Ϊ 113��
	������������ 0x0301 �İ�����Ƶ��Ϊ 10Hz��

	������ ID��
	1��Ӣ��(��)��
	2������(��)��
	3/4/5������(��)��
	6������(��)��
	7���ڱ�(��)��
	11��Ӣ��(��)��
	12������(��)��
	13/14/15������(��)��
	16������(��)��
	17���ڱ�(��)�� 
	�ͻ��� ID�� 
	0x0101 ΪӢ�۲����ֿͻ���( ��) ��
	0x0102 �����̲����ֿͻ��� ((�� )��
	0x0103/0x0104/0x0105�����������ֿͻ���(��)��
	0x0106�����в����ֿͻ���((��)�� 
	0x0111��Ӣ�۲����ֿͻ���(��)��
	0x0112�����̲����ֿͻ���(��)��
	0x0113/0x0114/0x0115�������ֿͻ��˲���(��)��
	0x0116�����в����ֿͻ���(��)�� 
*/
/* �������ݽ�����Ϣ��0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180
	����Ƶ�ʣ����� 10Hz


	1.	�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	�ֽ�ƫ���� 	��С 	˵�� 				��ע 
	0 			2 		���ݵ����� ID 		0xD180 
	2 			2 		���ߵ� ID 			��ҪУ�鷢���߻����˵� ID ��ȷ�� 
	4 			2 		�ͻ��˵� ID 		ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ��� 
	6 			4 		�Զ��帡������ 1 	 
	10 			4 		�Զ��帡������ 2 	 
	14 			4 		�Զ��帡������ 3 	 
	18 			1 		�Զ��� 8 λ���� 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF
	�������� �����˼�ͨ�ţ�0x0301��
	����Ƶ�ʣ����� 10Hz  

	�ֽ�ƫ���� 	��С 	˵�� 			��ע 
	0 			2 		���ݵ����� ID 	0x0200~0x02FF 
										���������� ID ��ѡȡ������ ID �����ɲ������Զ��� 
	
	2 			2 		�����ߵ� ID 	��ҪУ�鷢���ߵ� ID ��ȷ�ԣ� 
	
	4 			2 		�����ߵ� ID 	��ҪУ������ߵ� ID ��ȷ�ԣ�
										���粻�ܷ��͵��жԻ����˵�ID 
	
	6 			n 		���ݶ� 			n ��ҪС�� 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //���ݶ�,n��ҪС��113
} robot_interactive_data_t;

//֡ͷ  ������   ���ݶ�ͷ�ṹ  ���ݶ�   ֡β
//�ϴ��ͻ���
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//֡ͷ
	uint16_t		 						CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	client_custom_data_t  					clientData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_SendClientData_t;


//�����˽�����Ϣ
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//֡ͷ
	uint16_t								CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  	 			interactData;//���ݶ�
	uint16_t		 						FrameTail;//֡β
}ext_CommunatianData_t;

bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);

bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
float JUDGE_usGetSpeedHeat17(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);


#elif JUDGE_VERSION == JUDGE_20

#define	LEN_HEADER		5	//֡ͷ����
#define	LEN_CMDID		2	//������ID����
#define	LEN_TAIL		2	//֡β����

//����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
#define		JUDGE_FRAME_HEADER		(0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,  //֡ͷƫ��Ϊ�㣬��5�ֽ�
	CMD_ID               = 5, //ƫ��Ϊ5����2�ֽ�
	DATA                 = 7, //ƫ��Ϊ7���ֽ���ΪN
	
}JudgeFrameOffset;

//5�ֽ�֡ͷ,���Ե�ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************������ID********************/

/* 

	ID: 0x0001  Byte:  3    ����״̬����       						����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1    �����������         					������������      
	ID: 0x0003  Byte:  32   ����������Ѫ������  		 			1Hz����  
	ID��0x0004	Byte:  3	���ڷ���״̬							���ڷ���ʱ����
	ID��0x0005	Byte:  3	�˹�������ս���ӳ���ͷ���״̬			 1HZ����
	ID: 0x0101  Byte:  4    �����¼�����   							1HZ����
	ID: 0x0102  Byte:  4    ���ز���վ������ʶ����    				�����ı���� 
	ID: 0x0104	Byte:  2	���о�������							���淢������
	ID: 0x0105	Byte:  1	���ڷ���ڵ���ʱ						1HZ����
	ID: 0X0201  Byte: 18    ������״̬����        					10Hz
	ID: 0X0202  Byte: 16    ʵʱ������������   						50Hz       
	ID: 0x0203  Byte: 16    ������λ������           				10Hz
	ID: 0x0204  Byte:  1    ��������������           				1HZ
	ID: 0x0205  Byte:  3    ���л���������״̬����,ֻ�п��л��������ط� 10HZ
	ID: 0x0206  Byte:  1    �˺�״̬����           					�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           					�ӵ��������
	ID: 0X0208	Byte:  2    ����ʣ�෢�����������л����ˣ��ڱ��������Լ� ICRA �����˷��� 1HZ
	ID:	0X0209	Byte:  4	������RFID״̬							1HZ
	ID:	0x020A	Byte:  12	���ڻ����˿ͻ���ָ������				10HZ
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
	
*/
//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_state       			= 0x0001,//����״̬����
	ID_game_result 	   				= 0x0002,//�����������
	ID_game_robot_blood       		= 0x0003,//����������Ѫ������
	ID_darts_took_off				= 0x0004,//���ڷ���״̬
	ID_game_icra_buff				= 0x0005,//�˹�������ս���ӳ���ͷ���״̬
	ID_event_data  					= 0x0101,//�����¼����� 
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_judge_warning				= 0x0104,//���о���
	ID_dart_take_off_count_down		= 0x0105,//���ڷ���ڵ���ʱ
	ID_game_robot_state    			= 0x0201,//������״̬����
	ID_power_heat_data    			= 0x0202,//ʵʱ������������
	ID_game_robot_pos        		= 0x0203,//������λ������
	ID_buff_musk					= 0x0204,//��������������
	ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
	ID_robot_hurt					= 0x0206,//�˺�״̬����
	ID_shoot_data					= 0x0207,//ʵʱ�������
	ID_bullet_surplus				= 0x0208,//����ʣ�෢����
	ID_robot_rfid_state				= 0x0209,//������RFID״̬
	ID_darts_user_based_order		= 0x020A, //���ڿͻ���ָ��

} CmdID;

//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_HP       			=  32,	//0x0003
	LEN_darts_took_off					=  3, 	//0x0004
	LEN_game_icra_buff					=  3,	//0x0005
	LEN_event_data  					=  4,	//0x0101
	LEN_supply_projectile_action        =  3,	//0x0102
	LEN_judge_warning					=  2,	//0x0104
	LEN_dart_take_off_count_down		=  1,	//0x0105
	LEN_game_robot_state    			= 18,	//0x0201
	LEN_power_heat_data   				= 16,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  3,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  6,	//0x0207
	LEN_bullet_surplus					=  2,	//0x0208
	LEN_robot_rfid_state				=  4,   //0x0209
	LEN_darts_user_based_order			=  12,   //0x020A
} JudgeDataLength;


/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

// ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
} ext_game_status_t;

// ����������ݣ�0x0002������Ƶ�ʣ������������ͣ����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;


// ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;

// ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

// �˹�������ս���ӳ���ͷ���״̬��0x0005������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
} ext_ICRA_buff_debuff_zone_status_t;

//�����¼����ݣ�0x0101������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;

// ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

// ����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t shooter_heat0_speed_limit;
 uint8_t shooter_heat1_speed_limit;
 uint8_t max_chassis_power;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

// ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_heat0; 
 uint16_t shooter_heat1; 
 uint16_t mobile_shooter_heat2;
} ext_power_heat_data_t;

// ������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

// ���������棺0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_t;

// ���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint16_t energy_point;
 uint8_t attack_time;
} ext_aerial_robot_energy_t;

// �˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ������
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;


// ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

// �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�1Hz ���ڷ��ͣ����л����ˣ��ڱ��������Լ� ICRA ���������ط��ͣ����ͷ�Χ����һ������
typedef __packed struct
{
 uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;

// ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�

typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_status_t;


// ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/*****************ϵͳ���ݶ���**********************/
extern	ext_game_status_t       			GameState;					//0x0001
extern	ext_game_result_t            		GameResult;					//0x0002
extern	ext_game_robot_HP_t          		GameRobotHP;				//0x0003
extern	ext_dart_status_t					DartStatus;					//0x0004
extern	ext_ICRA_buff_debuff_zone_status_t  IcraBuffDebuffZoneStatus;	//0x0005
extern	ext_event_data_t        			EventData;					//0x0101
extern	ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
extern	ext_referee_warning_t				RefereeWarning;				//0x0104
extern	ext_dart_remaining_time_t			DartRemainingTime;			//0x0105
extern	ext_game_robot_status_t				GameRobotStat;				//0x0201
extern	ext_power_heat_data_t		  		PowerHeatData;				//0x0202
extern	ext_game_robot_pos_t				GameRobotPos;				//0x0203
extern	ext_buff_t						BuffMusk;					//0x0204
extern	ext_aerial_robot_energy_t			AerialRobotEnergy;			//0x0205
extern	ext_robot_hurt_t					RobotHurt;					//0x0206
extern	ext_shoot_data_t					ShootData;					//0x0207
extern	ext_bullet_remaining_t				BulletRemaining;			//0x0208
extern	ext_rfid_status_t					RfridStatus;				//0x0209
extern	ext_dart_client_cmd_t				DartClientCmd;				//0x020A
extern	xFrameHeader             			FrameHeader;		//����֡ͷ��Ϣ

bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);

bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
float JUDGE_usGetSpeedHeat17(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);


#elif JUDGE_VERSION == JUDGE_21

#define	LEN_HEADER		5	//֡ͷ����
#define	LEN_CMDID		2	//������ID����
#define	LEN_TAIL		2	//֡β����

//����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
#define		JUDGE_FRAME_HEADER		(0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,  //֡ͷƫ��Ϊ�㣬��5�ֽ�
	CMD_ID               = 5, //ƫ��Ϊ5����2�ֽ�
	DATA                 = 7, //ƫ��Ϊ7���ֽ���ΪN
	
}JudgeFrameOffset;

//5�ֽ�֡ͷ,���Ե�ƫ��λ��
typedef enum
{
	SOF          = 0,//��ʼλ
	DATA_LENGTH  = 1,//֡�����ݳ���,�����������ȡ���ݳ���
	SEQ          = 3,//�����
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************������ID********************/

/* 

	ID: 0x0001  Byte:  3    ����״̬����       						����Ƶ�� 1Hz      
	ID: 0x0002  Byte:  1    �����������         					������������      
	ID: 0x0003  Byte:  32   ����������Ѫ������  		 			1Hz����  
	ID��0x0004	Byte:  3	���ڷ���״̬							���ڷ���ʱ����
	ID��0x0005	Byte:  3	�˹�������ս���ӳ���ͷ���״̬			 1HZ����
	ID: 0x0101  Byte:  4    �����¼�����   							1HZ����
	ID: 0x0102  Byte:  4    ���ز���վ������ʶ����    				�����ı���� 
	ID: 0x0104	Byte:  2	���о�������							���淢������
	ID: 0x0105	Byte:  1	���ڷ���ڵ���ʱ						1HZ����
	ID: 0X0201  Byte: 18    ������״̬����        					10Hz
	ID: 0X0202  Byte: 16    ʵʱ������������   						50Hz       
	ID: 0x0203  Byte: 16    ������λ������           				10Hz
	ID: 0x0204  Byte:  1    ��������������           				1HZ
	ID: 0x0205  Byte:  3    ���л���������״̬����,ֻ�п��л��������ط� 10HZ
	ID: 0x0206  Byte:  1    �˺�״̬����           					�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           					�ӵ��������
	ID: 0X0208	Byte:  2    ����ʣ�෢�����������л����ˣ��ڱ��������Լ� ICRA �����˷��� 1HZ
	ID:	0X0209	Byte:  4	������RFID״̬							1HZ
	ID:	0x020A	Byte:  12	���ڻ����˿ͻ���ָ������				10HZ
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz
	
*/
//������ID,�����жϽ��յ���ʲô����
typedef enum
{ 
	ID_game_state       			= 0x0001,//����״̬����
	ID_game_result 	   				= 0x0002,//�����������
	ID_game_robot_blood       		= 0x0003,//����������Ѫ������
	ID_darts_took_off				= 0x0004,//���ڷ���״̬
	ID_game_icra_buff				= 0x0005,//�˹�������ս���ӳ���ͷ���״̬
	ID_event_data  					= 0x0101,//�����¼����� 
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_judge_warning				= 0x0104,//���о���
	ID_dart_take_off_count_down		= 0x0105,//���ڷ���ڵ���ʱ
	ID_game_robot_state    			= 0x0201,//������״̬����
	ID_power_heat_data    			= 0x0202,//ʵʱ������������
	ID_game_robot_pos        		= 0x0203,//������λ������
	ID_buff_musk					= 0x0204,//��������������
	ID_aerial_robot_energy			= 0x0205,//���л���������״̬����
	ID_robot_hurt					= 0x0206,//�˺�״̬����
	ID_shoot_data					= 0x0207,//ʵʱ�������
	ID_bullet_surplus				= 0x0208,//����ʣ�෢����
	ID_robot_rfid_state				= 0x0209,//������RFID״̬
	ID_darts_user_based_order		= 0x020A, //���ڿͻ���ָ��

} CmdID;

//���������ݶγ�,���ݹٷ�Э�������峤��
typedef enum
{
	LEN_game_state       				=  11,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_HP       			=  32,	//0x0003
	LEN_darts_took_off					=  3, 	//0x0004
	LEN_game_icra_buff					=  11,	//0x0005
	LEN_event_data  					=  4,	//0x0101
	LEN_supply_projectile_action        =  4,	//0x0102
	LEN_judge_warning					=  2,	//0x0104
	LEN_dart_take_off_count_down		=  1,	//0x0105
	LEN_game_robot_state    			= 27,	//0x0201
	LEN_power_heat_data   				= 16,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  2,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  7,	//0x0207
	LEN_bullet_surplus					=  6,	//0x0208
	LEN_robot_rfid_state				=  4,   //0x0209
	LEN_darts_user_based_order			=  12,   //0x020A
} JudgeDataLength;

/* �Զ���֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

// ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

// ����������ݣ�0x0002������Ƶ�ʣ������������ͣ����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;


// ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz�����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;

// ���ڷ���״̬��0x0004������Ƶ�ʣ����ڷ�����ͣ����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

// �˹�������ս���ӳ���ͷ���״̬��0x0005������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����л����ˡ�
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
	
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
 uint16_t blue1_bullet_left;
 uint16_t blue2_bullet_left;
} ext_ICRA_buff_debuff_zone_status_t;

//�����¼����ݣ�0x0101������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ��0x0102������Ƶ�ʣ������������ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢�����ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;

// ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����������ˡ�
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

// ����������״̬��0x0201������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
typedef __packed struct
{
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_id1_17mm_cooling_rate;
 uint16_t shooter_id1_17mm_cooling_limit;
 uint16_t shooter_id1_17mm_speed_limit;
 uint16_t shooter_id2_17mm_cooling_rate;
 uint16_t shooter_id2_17mm_cooling_limit;
 uint16_t shooter_id2_17mm_speed_limit;

 uint16_t shooter_id1_42mm_cooling_rate;
 uint16_t shooter_id1_42mm_cooling_limit;
 uint16_t shooter_id1_42mm_speed_limit;
 uint16_t chassis_power_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;
}ext_game_robot_status_t;

// ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz�����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_id1_17mm_cooling_heat;
 uint16_t shooter_id2_17mm_cooling_heat;
 uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

// ������λ�ã�0x0203������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

// ���������棺0x0204������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_t;

// ���л���������״̬��0x0205������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t attack_time;
} aerial_robot_energy_t;

// �˺�״̬��0x0206������Ƶ�ʣ��˺��������ͣ����ͷ�Χ����һ������
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;


// ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������ͣ����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

// �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�1Hz ���ڷ��ͣ����л����ˣ��ڱ��������Լ� ICRA ���������ط��ͣ����ͷ�Χ����һ������
typedef __packed struct
{
 uint16_t bullet_remaining_num_17mm;
 uint16_t bullet_remaining_num_42mm;
 uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

// ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ�����ˡ�

typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_status_t;


// ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ�����ˡ�
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/*****************ϵͳ���ݶ���**********************/
extern	ext_game_status_t       			GameState;					//0x0001
extern	ext_game_result_t            		GameResult;					//0x0002
extern	ext_game_robot_HP_t          		GameRobotHP;				//0x0003
extern	ext_dart_status_t					DartStatus;					//0x0004
extern	ext_ICRA_buff_debuff_zone_status_t  IcraBuffDebuffZoneStatus;	//0x0005
extern	ext_event_data_t        			EventData;					//0x0101
extern	ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
extern	ext_referee_warning_t				RefereeWarning;				//0x0104
extern	ext_dart_remaining_time_t			DartRemainingTime;			//0x0105
extern	ext_game_robot_status_t				GameRobotStat;				//0x0201
extern	ext_power_heat_data_t		  		PowerHeatData;				//0x0202
extern	ext_game_robot_pos_t				GameRobotPos;				//0x0203
extern	ext_buff_t						BuffMusk;					//0x0204
extern	aerial_robot_energy_t			AerialRobotEnergy;			//0x0205
extern	ext_robot_hurt_t					RobotHurt;					//0x0206
extern	ext_shoot_data_t					ShootData;					//0x0207
extern	ext_bullet_remaining_t				BulletRemaining;			//0x0208
extern	ext_rfid_status_t					RfridStatus;				//0x0209
extern	ext_dart_client_cmd_t				DartClientCmd;				//0x020A
extern	xFrameHeader             			FrameHeader;		//����֡ͷ��Ϣ

extern bool Hurt_Data_Update;
extern bool Power_Heat_Data_Updata;

bool Judge_Read_Data(uint8_t *ReadFromUsart);
void JUDGE_Show_Data(void);
void Send_to_Teammate(void);
bool is_red_or_blue(void);
void determine_ID(void);

bool JUDGE_sGetDataState(void);
float JUDGE_fGetChassisPower(void);
uint16_t JUDGE_fGetRemainEnergy(void);
uint8_t JUDGE_ucGetRobotLevel(void);
uint16_t JUDGE_usGetRemoteHeat17(void);
float JUDGE_usGetSpeedHeat17(void);
void JUDGE_ShootNumCount(void);
uint16_t JUDGE_usGetShootNum(void);
void JUDGE_ShootNum_Clear(void);
uint16_t JUDGE_usGetHeatLimit(void);
uint16_t JUDGE_usGetShootCold(void);
bool JUDGE_IfArmorHurt(void);
bool Judge_If_Death(void);


#endif //�汾��
/*-------------------------------------------------------------*/

#endif //ͷ�ļ�
