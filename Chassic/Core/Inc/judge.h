#ifndef _JUDGE_H
#define _JUDGE_H


#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
//裁判系统版本预编译
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

//2018裁判系统官方接口协议


//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16


/* RFID卡类型 */
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


//frame header格式

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)


//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;



/***************命令码ID********************/

/* 

   ID: 0x0001  Byte:  8    比赛机器人状态       发送频率 10Hz      
   ID: 0x0002  Byte:  1    伤害数据             受到伤害时发送      
   ID: 0x0003  Byte:  6    实时射击数据         发射弹丸时发送  
   ID: 0x0004  Byte: 20    实时功率和热量数据   发送频率 50Hz   20MS发送一次
   ID: 0x0005  Byte:  2    实时场地交互数据     检测到RFID卡时 发送频率 10Hz 
   ID: 0X0006  Byte:  1    比赛胜负数据         比赛结束时发送一次 
   ID: 0X0007  Byte:  2    Buff 获取数据        激活机关后发送一次
   ID: 0X0008  Byte: 16    机器人位置朝向信息   发送频率 50Hz        
   ID: 0x0100  Byte: 13    自定义数据           限制频率 10Hz
*/


//命令码ID,用来判断接收的是什么数据
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


//命令码数据段长,根据官方协议来定义长度
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
	float chassisPower;//瞬时功率
	float chassisPowerBuffer;//60焦耳缓冲能量
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
	uint16_t CmdID;//命令码 0x0100
	float data1;//浮点显示具体数值
	float data2;
	float data3;
	uint8_t mask;//低6位对应6个指示灯
	
} xShowData;


typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;


/**********裁判系统数据读取*************/
bool Judge_Read_Data(uint8_t *ReadFormUsart);//主循环4ms调用一次

/****用户自定义数据上传到客户端****/
void JUDGE_Show_Data(void);

/*****裁判数据辅助判断函数********/
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

/*******底盘自动闪避判断用******/
bool JUDGE_IfArmorHurt(void);

/*****超级电容预留用函数******/
float JUDGE_fGetChassisResiduePower(void);
float JUDGE_fGetSuper_Cap_Ele(void);


/*-------------------------2019--------------------------------*/
#elif JUDGE_VERSION == JUDGE_19

#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16


//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  2    比赛机器人存活数据   		1Hz发送  
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/





//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			= 0x0001,//比赛状态数据
	ID_game_result 	   				= 0x0002,//比赛结果数据
	ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据
	ID_event_data  					= 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_supply_projectile_booking 	= 0x0103,//场地补给站预约子弹数据
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk					= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					= 0x0206,//伤害状态数据
	ID_shoot_data					= 0x0207,//实时射击数据

} CmdID;


//命令码数据段长,根据官方协议来定义长度
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

/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  2    场地补给站预约子弹数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;   //机器人ID，可用来校验发送
	uint8_t robot_level;  //1一级，2二级，3三级
	uint16_t remain_HP;  //机器人剩余血量
	uint16_t max_HP; //机器人满血量
	uint16_t shooter_heat0_cooling_rate;  //机器人 17mm 子弹热量冷却速度 单位 /s
	uint16_t shooter_heat0_cooling_limit;   // 机器人 17mm 子弹热量上限
	uint16_t shooter_heat1_cooling_rate;   //机器人 42mm 子弹热量冷却速度
	uint16_t shooter_heat1_cooling_limit;   //机器人 42mm 子弹热量上限
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; //主控电源输出情况
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   
	uint16_t chassis_current;    
	float chassis_power;   //瞬时功率 
	uint16_t chassis_power_buffer;//60焦耳缓冲能量
	uint16_t shooter_heat0;//17mm
	uint16_t shooter_heat1;  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 


/* 
	
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/
/* 交互数据接收信息：0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;


//机器人交互信息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
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

#define	LEN_HEADER		5	//帧头长度
#define	LEN_CMDID		2	//命令码ID长度
#define	LEN_TAIL		2	//帧尾长度

//数据帧起始字节，固定值为 0xA5
#define		JUDGE_FRAME_HEADER		(0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,  //帧头偏移为零，共5字节
	CMD_ID               = 5, //偏移为5，共2字节
	DATA                 = 7, //偏移为7，字节数为N
	
}JudgeFrameOffset;

//5字节帧头,各自的偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       						发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         					比赛结束后发送      
	ID: 0x0003  Byte:  32   比赛机器人血量数据  		 			1Hz发送  
	ID：0x0004	Byte:  3	飞镖发射状态							飞镖发射时发送
	ID：0x0005	Byte:  3	人工智能挑战赛加成与惩罚区状态			 1HZ发送
	ID: 0x0101  Byte:  4    场地事件数据   							1HZ发送
	ID: 0x0102  Byte:  4    场地补给站动作标识数据    				动作改变后发送 
	ID: 0x0104	Byte:  2	裁判警告数据							警告发生后发送
	ID: 0x0105	Byte:  1	飞镖发射口倒计时						1HZ发送
	ID: 0X0201  Byte: 18    机器人状态数据        					10Hz
	ID: 0X0202  Byte: 16    实时功率热量数据   						50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           				10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           				1HZ
	ID: 0x0205  Byte:  3    空中机器人能量状态数据,只有空中机器人主控发 10HZ
	ID: 0x0206  Byte:  1    伤害状态数据           					伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           					子弹发射后发送
	ID: 0X0208	Byte:  2    弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送 1HZ
	ID:	0X0209	Byte:  4	机器人RFID状态							1HZ
	ID:	0x020A	Byte:  12	飞镖机器人客户端指令数据				10HZ
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/
//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			= 0x0001,//比赛状态数据
	ID_game_result 	   				= 0x0002,//比赛结果数据
	ID_game_robot_blood       		= 0x0003,//比赛机器人血量数据
	ID_darts_took_off				= 0x0004,//飞镖发射状态
	ID_game_icra_buff				= 0x0005,//人工智能挑战赛加成与惩罚区状态
	ID_event_data  					= 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_judge_warning				= 0x0104,//裁判警告
	ID_dart_take_off_count_down		= 0x0105,//飞镖发射口倒计时
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk					= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					= 0x0206,//伤害状态数据
	ID_shoot_data					= 0x0207,//实时射击数据
	ID_bullet_surplus				= 0x0208,//弹丸剩余发射数
	ID_robot_rfid_state				= 0x0209,//机器人RFID状态
	ID_darts_user_based_order		= 0x020A, //飞镖客户端指令

} CmdID;

//命令码数据段长,根据官方协议来定义长度
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


/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

// 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
} ext_game_status_t;

// 比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;


// 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。
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

// 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

// 人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人。
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

//场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

//补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;

// 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

// 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人
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

// 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。
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

// 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

// 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_t;

// 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
 uint16_t energy_point;
 uint8_t attack_time;
} ext_aerial_robot_energy_t;

// 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;


// 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

// 子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人
typedef __packed struct
{
 uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;

// 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。

typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_status_t;


// 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
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

/*****************系统数据定义**********************/
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
extern	xFrameHeader             			FrameHeader;		//发送帧头信息

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

#define	LEN_HEADER		5	//帧头长度
#define	LEN_CMDID		2	//命令码ID长度
#define	LEN_TAIL		2	//帧尾长度

//数据帧起始字节，固定值为 0xA5
#define		JUDGE_FRAME_HEADER		(0xA5)

typedef enum 
{
	FRAME_HEADER         = 0,  //帧头偏移为零，共5字节
	CMD_ID               = 5, //偏移为5，共2字节
	DATA                 = 7, //偏移为7，字节数为N
	
}JudgeFrameOffset;

//5字节帧头,各自的偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       						发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         					比赛结束后发送      
	ID: 0x0003  Byte:  32   比赛机器人血量数据  		 			1Hz发送  
	ID：0x0004	Byte:  3	飞镖发射状态							飞镖发射时发送
	ID：0x0005	Byte:  3	人工智能挑战赛加成与惩罚区状态			 1HZ发送
	ID: 0x0101  Byte:  4    场地事件数据   							1HZ发送
	ID: 0x0102  Byte:  4    场地补给站动作标识数据    				动作改变后发送 
	ID: 0x0104	Byte:  2	裁判警告数据							警告发生后发送
	ID: 0x0105	Byte:  1	飞镖发射口倒计时						1HZ发送
	ID: 0X0201  Byte: 18    机器人状态数据        					10Hz
	ID: 0X0202  Byte: 16    实时功率热量数据   						50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           				10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           				1HZ
	ID: 0x0205  Byte:  3    空中机器人能量状态数据,只有空中机器人主控发 10HZ
	ID: 0x0206  Byte:  1    伤害状态数据           					伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           					子弹发射后发送
	ID: 0X0208	Byte:  2    弹丸剩余发射数，仅空中机器人，哨兵机器人以及 ICRA 机器人发送 1HZ
	ID:	0X0209	Byte:  4	机器人RFID状态							1HZ
	ID:	0x020A	Byte:  12	飞镖机器人客户端指令数据				10HZ
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/
//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			= 0x0001,//比赛状态数据
	ID_game_result 	   				= 0x0002,//比赛结果数据
	ID_game_robot_blood       		= 0x0003,//比赛机器人血量数据
	ID_darts_took_off				= 0x0004,//飞镖发射状态
	ID_game_icra_buff				= 0x0005,//人工智能挑战赛加成与惩罚区状态
	ID_event_data  					= 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_judge_warning				= 0x0104,//裁判警告
	ID_dart_take_off_count_down		= 0x0105,//飞镖发射口倒计时
	ID_game_robot_state    			= 0x0201,//机器人状态数据
	ID_power_heat_data    			= 0x0202,//实时功率热量数据
	ID_game_robot_pos        		= 0x0203,//机器人位置数据
	ID_buff_musk					= 0x0204,//机器人增益数据
	ID_aerial_robot_energy			= 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					= 0x0206,//伤害状态数据
	ID_shoot_data					= 0x0207,//实时射击数据
	ID_bullet_surplus				= 0x0208,//弹丸剩余发射数
	ID_robot_rfid_state				= 0x0209,//机器人RFID状态
	ID_darts_user_based_order		= 0x020A, //飞镖客户端指令

} CmdID;

//命令码数据段长,根据官方协议来定义长度
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

/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} xFrameHeader;

// 比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

// 比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。
typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;


// 机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。
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

// 飞镖发射状态：0x0004。发送频率：飞镖发射后发送，发送范围：所有机器人。
typedef __packed struct
{
 uint8_t dart_belong; 
 uint16_t stage_remaining_time; 
} ext_dart_status_t;

// 人工智能挑战赛加成与惩罚区状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人。
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

//场地事件数据：0x0101。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

//补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

// 裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
typedef __packed struct
{
 uint8_t level;
 uint8_t foul_robot_id; 
} ext_referee_warning_t;

// 飞镖发射口倒计时：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

// 比赛机器人状态：0x0201。发送频率：10Hz，发送范围：单一机器人
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

// 实时功率热量数据：0x0202。发送频率：50Hz，发送范围：单一机器人。
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

// 机器人位置：0x0203。发送频率：10Hz，发送范围：单一机器人
typedef __packed struct
{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;

// 机器人增益：0x0204。发送频率：1Hz 周期发送，发送范围：单一机器人。
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_t;

// 空中机器人能量状态：0x0205。发送频率：10Hz，发送范围：单一机器人。
typedef __packed struct
{
 uint8_t attack_time;
} aerial_robot_energy_t;

// 伤害状态：0x0206。发送频率：伤害发生后发送，发送范围：单一机器人
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;


// 实时射击信息：0x0207。发送频率：射击后发送，发送范围：单一机器人。
typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;

// 子弹剩余发射数：0x0208。发送频率：1Hz 周期发送，空中机器人，哨兵机器人以及 ICRA 机器人主控发送，发送范围：单一机器人
typedef __packed struct
{
 uint16_t bullet_remaining_num_17mm;
 uint16_t bullet_remaining_num_42mm;
 uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

// 机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人。

typedef __packed struct
{
 uint32_t rfid_status;
} ext_rfid_status_t;


// 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人。
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

/*****************系统数据定义**********************/
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
extern	xFrameHeader             			FrameHeader;		//发送帧头信息

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


#endif //版本号
/*-------------------------------------------------------------*/

#endif //头文件
