#include "main.h"
#include <string.h>
#include "Serial.h"
#include "OLED.h"
#include "CAN_middle.h"
#include "CAN_MSG.h"
#include "LED.h"
#include "SSPC.h"
#include "Pro.h"

extern IWDG_HandleTypeDef hiwdg;
KEY sky_gnd_key;
/*ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½*/
extern UART_HandleTypeDef huart1;
extern uint8_t USART1_RxFrame[32];
extern uint8_t USART1_RxFinish;

/*SSPCÄ£ï¿½ï¿½*/
extern FC_SendData Read_data; 
extern uint8_t CAN_SendBuff[8];
extern uint8_t CAN_RxFinish;
static uint8_t SSPC_Open_Key;	// ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½Ö¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø¸ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½
static uint8_t SSPC_Close_Key;	// ï¿½ï¿½ï¿½ï¿½Ä£Ê½ï¿½ï¿½Ö¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ø¸ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½
static uint8_t Vbus28_Full_Flag;
static uint32_t tick_V28check;
static uint32_t Unlock_ENS_tick;
static uint8_t  SSPC_ACK_flag;
uint8_t SSPC_Data[8];

/*ï¿½Ë¿Ú³ï¿½Ê¼×´Ì¬*/
CAN_CHN_Status can = {.chn1 = 0,.chn2 = 1,.chn3 = 1,.chn4 = 0,.chn5 = 0,.chn6 = 0,.chn7 = 1,.chn8 = 1};
OpCmd_t last_time[8];                //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Î¼ï¿½Â¼
uint8_t lastOp_valid_flag[8];        //ï¿½ï¿½ï¿½ï¿½È·ï¿½ï¿½ï¿½ï¿½Ò»ï¿½Î²ï¿½ï¿½ï¿½
uint16_t SSPC_Lock_flag;             //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö¾Î»

/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½*/
volatile uint32_t ENG_StartTick; // ï¿½ï¿½ï¿½Õµï¿½0x80ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½FC_IO_CMDï¿½ï¿ªÊ¼ï¿½ï¿½Ê±
volatile uint32_t ENG_StopTick;
uint8_t ENG_Start_Lock;			//ï¿½ï¿½ï¿½Õµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1,ï¿½ï¿½ï¿½ï¿½ï¿½Ø¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì£ï¿½0ï¿½ï¿½ï¿½ï¿½ï¿½Ô½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì£ï¿½ï¿½ï¿½ï¿½Ô´ï¿½Í¨ï¿½ï¿½4
uint8_t ENG_Stop_Lock;
uint8_t Eng_Num_Flag;  	// 1ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1 0ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½2

#define MAX_FRAME_PER_CALL  20   // Ã¿´Î×î¶à´¦Àí20Ö¡£¬¿É¸ù¾ÝÊµ¼ÊÇé¿öµ÷Õû
volatile uint8_t sspc_ack_received = 0;

CAN_Frame_t can_queue[CAN_QUEUE_SIZE];
volatile uint8_t queue_head = 0;
volatile uint8_t queue_tail = 0;
volatile uint8_t queue_count = 0;

uint8_t lock_channel = 0;   // ±£´æËø¶¨Ê±µÄÍ¨µÀºÅ
static uint8_t P1_UNLOCK_V = 0;
static uint8_t P2_UNLOCK_V = 0;


/**
*@brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
*@retval
*/
void Process(void)
{
	  uint32_t now = HAL_GetTick();
	  ENG_START_6S(now);
		if(sky_gnd_key.last_stable == GPIO_PIN_SET)/*statu:SKY*/
		{
			SSPC_Close_Key = 0;
			if(SSPC_Open_Key == 0){
				 SSPC_Open_Key = 1;
				 SSPC_Init(sky_gnd_key.last_stable);
				 LED_ON();
			}
		}
		else                            /*statu:GROUND*/
		{
			SSPC_Open_Key = 0;
			LED_TURN(now,Pro_LED);
			if(SSPC_Close_Key == 0)
			{
				SSPC_Close_Key = 1;
				Vbus28_Full_Flag = 0;
				SSPC_Init(sky_gnd_key.last_stable);
			}
		}
/*ï¿½ï¿½ï¿½Õ·É¿ï¿½Ö¸ï¿½ï¿½*/
		if(USART1_RxFinish == 1)
			{
				USART1_RxFinish = 0;
				Fly_Control();
			}
		  
/*ï¿½ï¿½ï¿½ï¿½SSPCï¿½ï¿½ï¿½ï¿½*/
		/*if(CAN_RxFinish == 1)
		{
	        memcpy(SSPC_Data,CAN_SendBuff,8);
			SSPC_Cmd(now,SSPC_Data);
			CAN_RxFinish = 0;
//			OLED_ShowCANWord();
		if(SSPC_Lock_flag)
		{
			SSPC_CHN_Unlock(now,SSPC_Data);
		}
        }*/
	   Process_CAN_Queue(now);
}

/**
*@brief  ï¿½É¿Ø¿ï¿½ï¿½ï¿½ï¿½ï¿½Ï¢ï¿½ï¿½ï¿½ï¿½
*@retval  ï¿½ï¿½ï¿½Ô·ï¿½ï¿½Í±ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½ï¿½ë£º0xEB 0x92 0xFF 0x00 0x00 0x7C
*/
void Fly_Control(void)
{
	uint8_t IO_Cmd1 = 0;
	uint8_t IO_Cmd2 = 0;
	if(USART1_RxFrame[2] == 0xFF){SSPC_Set();SSPC_Open_Key = 0;}
	if(USART1_RxFrame[3] == 0xFF && USART1_RxFrame[4] ==0x00)
	{
		Eng_Num_Flag = 1;
		IO_Cmd1 = USART1_RxFrame[2];
		FC_IO_CMD(IO_Cmd1,1);
	}
	if(USART1_RxFrame[3] == 0x00 && USART1_RxFrame[4] ==0xFF)
	{
		Eng_Num_Flag = 0;
		IO_Cmd2 = USART1_RxFrame[2];
		FC_IO_CMD(IO_Cmd2,0);
	}
		
}

/**
*@brief  ï¿½É¿ï¿½Í¨ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
*@param  cmd:Ö¸ï¿½ï¿½ï¿½ï¿½
*@param  eng:ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
*@retval
*/
void FC_IO_CMD(uint8_t cmd,uint8_t eng)
{
  switch(eng)
	{
		case 1:       /*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½1*/
		    if(cmd&0x80)
			{
				if(ENG_Start_Lock != 1)
				{
					ENG_Start_Lock = 1;				//ï¿½ï¿½ï¿½ï¿½ï¿½Ø¸ï¿½ï¿½ï¿½Í¨ï¿½ï¿½4
				    ENG_StartTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_4,0);
				    LogChannelOp(CHN_4,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(cmd&0x40)
			{
				if(ENG_Stop_Lock != 1)
				{
					ENG_Stop_Lock = 1;
				    ENG_StopTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0);
				    LogChannelOp(CHN_2,SSPC_FUNC_CHN_CLOSE,0);
				}

			}
			if(cmd&0x08)
			{
                if(can.chn5 == 0)
                {
					can.chn5 = 1;
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0);
					LogChannelOp(CHN_6,SSPC_FUNC_CHN_OPEN,0);
					LogChannelOp(CHN_5,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(!(cmd&0x08)&&can.chn5 == 1)
			{
				can.chn5 = 0;
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0);
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
				LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
			}
		break;

		case 0:       /*·¢¶¯»ú2*/
			 if(cmd&0x80)
			{
				if(ENG_Start_Lock != 1)
				{
					ENG_Start_Lock = 1;
				    ENG_StartTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_4,0);
				    LogChannelOp(CHN_4,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(cmd&0x40)
			{
				if(ENG_Stop_Lock != 1)
				{
					ENG_Stop_Lock = 1;
				    ENG_StopTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0);
				    LogChannelOp(CHN_2,SSPC_FUNC_CHN_CLOSE,0);
				}

			}
			if(cmd&0x08)
			{
                if(can.chn5 == 0)
                {
					can.chn5 = 1;
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0);
					LogChannelOp(CHN_6,SSPC_FUNC_CHN_OPEN,0);
					LogChannelOp(CHN_5,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(!(cmd&0x08)&&can.chn5 == 1)
			{
				can.chn5 = 0;
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0);
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
				LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
			}
		break;		
		default:
			switch(cmd)
			{
				case 0x04: can.chn7^=1;                                                  /*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?*/
                   if(can.chn7 == 1)
                   {
						SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0);
						LogChannelOp(CHN_7,SSPC_FUNC_CHN_OPEN,0);
					}
				    else 
					{
						SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_7,0);
						LogChannelOp(CHN_7,SSPC_FUNC_CHN_CLOSE,0);
					}
					break;
//		  case 0x02:                                                        break;/*Ó¦ï¿½ï¿½ï¿½ï¿½Ø³ï¿½ï¿½*/
//	  	  case 0x01:                                                        break;/*ï¿½ï¿½ï¿½ï¿½ï¿½ØºÉ¿ï¿½ï¿½ï¿½*/
			}break;
	}
}

/**
 * @brief ï¿½Õµï¿½SSPCÖ¸ï¿½ï¿½ï¿½ï¿½ï¿?
 * @retval 
 */
void SSPC_Cmd(uint32_t now,uint8_t *Data)
{
    switch(Data[2])
	{
		case SSPC_STAT_LOCK_ERR       : SSPC_Lock_flag ++;lock_channel = Data[3];break;
		case SSPC_STAT_REPORT_VIN_TEMP :
		case SSPC_STAT_REPORT_VOUT_I  : SSPC_CHN_Read(now,&Read_data,Data);break;
		case SSPC_STAT_CMD_ACK        :	if(SSPC_Lock_flag!=0&&Data[6] == 0x4F &&Data[7] == 0x4B)
		                                 {
											SSPC_ACK_flag = 1;
										 };break;
	    default: break;
	}

}



/**
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½6s
 * @retval 
 */
void ENG_START_6S(uint32_t now)
{
  static uint8_t ENG_Start_Finsh_Lock = 1;		// ï¿½ï¿½Ê¼ï¿½ï¿½Ò»ï¿½ï¿½
  static uint8_t ENG_Stop_Finsh_Lock = 1;		// ï¿½ï¿½Ê¼ï¿½ï¿½Ò»ï¿½ï¿½
  if(ENG_Start_Lock && (now-ENG_StartTick <= ENG_START_DELAY))  // ï¿½ï¿½ï¿½ï¿½6ï¿½ï¿½
  {
    ENG_Start_Lock = 1;					//ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?6ï¿½ë£¬	ENG_Start_Lock ï¿½ï¿½ï¿½ï¿½Îª1		
	ENG_Start_Finsh_Lock = 0;			//ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?6ï¿½ë£¬ENG_Start_Finsh_Lockï¿½ï¿½0
  }
	else 	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î´ï¿½ò¿?»ï¿½ï¿½ß³ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Í»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
  {
	if(ENG_Start_Finsh_Lock == 0)		//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½6ï¿½ï¿½
	{
		ENG_Start_Finsh_Lock = 1;		// ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ò´ï¿½ï¿½ï¿½6ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¸ï¿½ENG_Start_Finsh_Lockï¿½ï¿½1
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_4,0);	// ï¿½ï¿½ï¿½ï¿½ï¿½Ï£ï¿½ï¿½Ø±ï¿½Í¨ï¿½ï¿½4
		LogChannelOp(CHN_4,SSPC_FUNC_CHN_CLOSE,0);
	}
	ENG_Start_Lock = 0;					//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  }

  if(ENG_Stop_Lock && (now-ENG_StopTick <= ENG_STOP_DELAY))
  {
    ENG_Stop_Lock = 1;
	ENG_Stop_Finsh_Lock = 0;
  }
	else 
  {
	if(ENG_Stop_Finsh_Lock == 0)
	{
		ENG_Stop_Finsh_Lock = 1;
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);
		LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
	}
	ENG_Stop_Lock = 0;
  }
}

/**
 * @brief  ï¿½È´ï¿½SSPCï¿½ï¿½ï¿½ï¿½ï¿½Ø¸ï¿½
 * @retval 
 */
void Waiting_SSPC(void)
{
    uint32_t tick = HAL_GetTick();
    sspc_ack_received = 0;   // Çå¿Õ±êÖ¾£¬¿ªÊ¼µÈ´ý

    while (1)   // Ò»Ö±µÈµ½ ACK ÎªÖ¹£¨°´ÄãµÄÒªÇó£¬²»Éè³¬Ê±£©
    {
        uint32_t now = HAL_GetTick();
        LED_TURN(now, Wait_SSPC_LED);

        // Ã¿ 2 Ãë·¢Ò»´Î¹Ø±ÕËùÓÐÍ¨µÀµÄÃüÁî£¨Ô­ÓÐÂß¼­£©
        if (now - tick >= SSPC_START_WAIT) {
            tick = now;   
            SSPC_SendCmd(SSPC_ID, SSPC_FUNC_CHN_CLOSE, SSPC_CHN_ALL, 0);
        }

        HAL_IWDG_Refresh(&hiwdg);

        if (sspc_ack_received) {
            break;   // ÊÕµ½ ACK£¬ÍË³öµÈ´ý
        }
    }
}
/**
 * @brief  OLEDï¿½ï¿½Ê¾SSPCï¿½ï¿½ï¿½ï¿½
 * @retval 
 */
void OLED_ShowCANWord(void)
{
	OLED_ShowString(1,1,"DevID:",OLED_8X16);
	OLED_ShowHexNum(64,1,CAN_SendBuff[0],2,OLED_8X16);
	OLED_ShowHexNum(80,1,CAN_SendBuff[1],2,OLED_8X16);
	OLED_ShowString(1,16,"FUN  :",OLED_8X16);
	OLED_ShowHexNum(64,16,CAN_SendBuff[2],2,OLED_8X16);
	OLED_ShowString(1,32,"CHN  :",OLED_8X16);
	OLED_ShowHexNum(64,32,CAN_SendBuff[3],2,OLED_8X16);
	OLED_ShowString(1,48,"STATUS:",OLED_8X16);
	OLED_ShowHexNum(64,48,CAN_SendBuff[4],2,OLED_8X16);
	OLED_ShowHexNum(80,48,CAN_SendBuff[5],2,OLED_8X16);
	OLED_ShowHexNum(96,48,CAN_SendBuff[6],2,OLED_8X16);
	OLED_ShowHexNum(112,48,CAN_SendBuff[7],2,OLED_8X16);
	OLED_Update();
}

/**
* @brief SSPCï¿½ï¿½ï¿½ï¿½
* @param 
*/
void SSPC_Set(void)
{
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_ALL,0);      //ï¿½Ø±Õ¹ï¿½ï¿½ï¿½Í¨ï¿½ï¿½
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_REPORT_CYC,0,0x1E8480);     //ï¿½Ï±ï¿½ï¿½ï¿½ï¿½ï¿½50ms
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_UVP,SSPC_CHN_ALL,0x2710);   //Ç·Ñ¹10V
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_OVP,SSPC_CHN_ALL,0x7530);   //ï¿½ï¿½Ñ¹50V
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_5_8,0xC350);  //Í¨ï¿½ï¿½5-8ï¿½î¶¨ï¿½ï¿½ï¿½ï¿½50A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_1,0x7530);    //Í¨ï¿½ï¿½1  ï¿½î¶¨ï¿½ï¿½ï¿½ï¿½30A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_2,0x2710);    //Í¨ï¿½ï¿½234ï¿½î¶¨ï¿½ï¿½ï¿½ï¿½10A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_3,0x2710);
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_4,0x2710);
	//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_SAVE_FLASH,0,0);              //ï¿½ï¿½ï¿½ï¿½
	
}

/** 
 * @brief  CANÖ¡Èë¶Ó
 * @param  data: Ö¡Êý¾Ý
 * @param  id: Ö¡ID
 * @retval  
 * */
void CAN_Enqueue(uint8_t *data, uint32_t id)
{
// Èç¹û¶ÓÁÐÒÑÂú£¬¸²¸Ç×î¾ÉÖ¡£¨±£Áô×îÐÂÊý¾Ý£©
    if (queue_count >= CAN_QUEUE_SIZE) {
        // ¶ÁÖ¸ÕëÇ°½ø£¬¶ªÆú×î¾ÉÖ¡
        queue_head = (queue_head + 1) % CAN_QUEUE_SIZE;
        queue_count--;
    }
    // ¿½±´Êý¾Ýµ½¶ÓÁÐÎ²²¿
    memcpy(can_queue[queue_tail].data, data, 8);
    can_queue[queue_tail].id = id;
    // ¸üÐÂ¶ÓÎ²Ë÷Òý
    queue_tail = (queue_tail + 1) % CAN_QUEUE_SIZE;
    queue_count++;
	
}

/**
  * @brief ´Ó¶ÓÁÐÖÐÈ¡³öËùÓÐ´ý´¦ÀíÖ¡²¢ÒÀ´ÎÖ´ÐÐ
  * @param now µ±Ç°ÏµÍ³Ê±¼ä£¨HAL_GetTick()£©
  */
void Process_CAN_Queue(uint32_t now)
{
    // Ñ­»·´¦Àí£¬Ö±µ½¶ÓÁÐÎª¿Õ
    while (queue_count > 0) {
        uint8_t temp_data[8];
       // uint32_t temp_id;

        // ÁÙ½çÇø±£»¤£¨·ÀÖ¹ÖÐ¶Ï¸ÉÈÅ£©
        __disable_irq();
        if (queue_count == 0) {
            __enable_irq();
            break;
        }
        // È¡³ö¶ÓÊ×Ö¡
        memcpy(temp_data, can_queue[queue_head].data, 8);
       // temp_id = can_queue[queue_head].id;
        // ÒÆ¶¯¶ÓÊ×Ö¸Õë
        queue_head = (queue_head + 1) % CAN_QUEUE_SIZE;
        queue_count--;
        __enable_irq();

        // ------ ´¦Àí¸ÃÖ¡£¨Ô­ `if(CAN_RxFinish == 1)` ÖÐµÄÂß¼­£© ------
        SSPC_Cmd(now, temp_data);   // ½âÎöÃüÁî²¢Ö´ÐÐ
        if (SSPC_Lock_flag) {
            SSPC_CHN_Unlock(now, temp_data);
        }
    }
}

/**
* @brief SSPCï¿½ï¿½Ê¼ï¿½ï¿½
* @param flag:ï¿½Ø¿Õ¿ï¿½ï¿½Ø±ï¿½Ö¾Î»
* @param 
*/
void SSPC_Init(uint8_t flag)
{
	if(flag == GPIO_PIN_SET)
	{
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_1,0);HAL_Delay(SSPC_SendDelay);
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);HAL_Delay(SSPC_SendDelay);   /*ï¿½ï¿½Ï¨ï¿½ð¿ª¹ï¿½*/
	//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0);HAL_Delay(SSPC_SendDelay);   /*ï¿½ï¿½ECU*/
	 // SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0);HAL_Delay(SSPC_SendDelay);   /*ï¿½ï¿½28V*/
     // SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0); HAL_Delay(SSPC_SendDelay);  /*ï¿½ò¿ª¶ï¿½ï¿?*/
	 
	  LogChannelOp(CHN_1,SSPC_FUNC_CHN_CLOSE,0); 
	  LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
	//LogChannelOp(CHN_3,SSPC_FUNC_CHN_OPEN,0);
	 // LogChannelOp(CHN_8,SSPC_FUNC_CHN_OPEN,0);
	 // LogChannelOp(CHN_7,SSPC_FUNC_CHN_OPEN,0);
  }
	
	else if(flag == GPIO_PIN_RESET)
	{
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5_8,0);HAL_Delay(SSPC_SendDelay);
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);HAL_Delay(SSPC_SendDelay);   /*ï¿½ï¿½ï¿½Øºï¿½*/
	  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);HAL_Delay(SSPC_SendDelay);   /*ï¿½ï¿½Ï¨ï¿½ï¿½*/
	  //SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0);HAL_Delay(SSPC_SendDelay);   /*ï¿½ï¿½ECU*/
	  LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0);
	  LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
	 // LogChannelOp(CHN_3,SSPC_FUNC_CHN_OPEN,0);
	  LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
	  LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
	  LogChannelOp(CHN_7,SSPC_FUNC_CHN_CLOSE,0);
	  LogChannelOp(CHN_8,SSPC_FUNC_CHN_CLOSE,0);
  }
}

/**
 * @brief  SSPCï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
 * @retval 
 */
void SSPC_CHN_Unlock(uint32_t now,uint8_t *data)
{
    static uint16_t time = 0; 
	static uint16_t count = 0;
	if(count != SSPC_Lock_flag)	//	ï¿½ï¿½Ä¿ï¿½ï¿½ï¿½ï¿½0ï¿½Ä»ï¿½
	{
       count = SSPC_Lock_flag;
	   time  = 1;
	   SSPC_ACK_flag = 0;
	}
	
	if(time == 1)
	{
	    if(lock_channel <= SSPC_CHN_4 && P1_UNLOCK_V)
	    {
		    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_UNLOCK,SSPC_CHN_1_4,0);/*ï¿½ï¿½ï¿½Í?ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½*/
	    }
	    else if (lock_channel>= SSPC_CHN_5 && lock_channel <= SSPC_CHN_8 && P2_UNLOCK_V)
	    {
		    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_UNLOCK,SSPC_CHN_5_8,0);
	    }
		Unlock_ENS_tick = HAL_GetTick();
		time = 2;
	}
	if(time == 2)
	{
		if(HAL_GetTick()-Unlock_ENS_tick >SSPC_UNLOCK_ENS)
		{
			time = 3;
		}
	}
	if(time == 3)
	{
		
	    if(SSPC_ACK_flag ==1)
		{
			SSPC_ACK_flag = 0;
			SSPC_Recover();
			SSPC_Lock_flag = 0;
		    time = 0;
		}

		
	}
}

/**
 * @brief  SSPCï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½
 * @retval 
 */
void 
SSPC_CHN_Read(uint32_t now,FC_SendData* readdata,uint8_t data[8])
{
	switch(data[3])
	{
		case SSPC_CHN_1:   break;
		case SSPC_CHN_5:  if(data[2] == SSPC_STAT_REPORT_VOUT_I){readdata->Ichn5H = data[6];readdata->Ichn5L = data[7];}  break;
		case SSPC_CHN_6:  if(data[2] == SSPC_STAT_REPORT_VOUT_I){readdata->Ichn6H = data[6];readdata->Ichn6L = data[7];}  break;
	//case SSPC_CHN_7:   readdata->Ichn7H = data[6];readdata->Ichn7L = data[7];  break;
		case SSPC_CHN_8:   readdata->Ichn8H = data[6];readdata->Ichn8L = data[7]; break;
		case SSPC_CHN_5_8: readdata->Vchn8H = data[4];readdata->Vchn8L = data[5];
							//Vcheck_28Vbus(now,data[4],data[5]);
							if (Byte2_TO_U16(data[4],data[5]) > 0x2710)
							{P2_UNLOCK_V = 1;}
							else{P2_UNLOCK_V = 0;}
							break;
		case SSPC_CHN_1_4 : readdata->Vbus12H = data[4];readdata->Vbus12L = data[5];
							if(Byte2_TO_U16(data[4],data[5]) > 0x2710){P1_UNLOCK_V = 1;}
							else{P1_UNLOCK_V = 0;}
							break;
		default :break;
	}
}

/**
 * @brief  ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¼
 * @retval 
 */
void Log_UnlockOp(uint16_t id)
{
	if(id == SSPC_CHN_1_4 )
	{
		LogChannelOp(CHN_1,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_2,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_3,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_4,SSPC_FUNC_UNLOCK,0);
	}
	else
 {
	 LogChannelOp(CHN_5,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_6,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_7,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_8,SSPC_FUNC_UNLOCK,0);
 }
}

/**
 * @brief  Í¨ï¿½ï¿½ï¿½ï¿½ï¿½Ü»Ö¸ï¿½ï¿½ï¿½ï¿½ï¿½
 * @retval 
 */
void SSPC_Recover(void)
{
	GetAllChnLastOp(last_time,lastOp_valid_flag);
	for(uint8_t ch=0;ch<8;ch++)
	{
		if(lastOp_valid_flag[ch] == 1 )
		{
			switch(last_time[ch].op_code)
			{
				case SSPC_FUNC_CHN_OPEN :
					switch(ch){
						case 0:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0); break;
						case 1:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0); break;
						case 2:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0); break;
						case 3: break;
						case 4:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);  break;
						case 5:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0); break;
						case 6:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0); break;
						case 7:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0); break;
						default :break;
					}   break;
				case SSPC_FUNC_CHN_CLOSE: 
					 switch(ch){
						case 0:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_1,0); break;
						case 1:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0); break;
						case 2:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_3,0); break;
						case 3:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_4,0); break;
						case 4:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0); break;
						case 5:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0); break;
						case 6:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_7,0); break;
						case 7:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_8,0); break;
						default :break;
					}   break;
				default :break;
			}
		}
		HAL_Delay(SSPC_SendDelay);
	}
}

/**
 * @brief  28Vï¿½ï¿½Ñ¹ï¿½ï¿½ï¿?
 * @param  now:ï¿½ï¿½Ç°Ê±ï¿½ï¿½
 * @param  VH:ï¿½ï¿½Ñ¹ï¿½ï¿½Î»
 * @param  VL:ï¿½ï¿½Ñ¹ï¿½ï¿½Î»
 * @retval 
 */
void Vcheck_28Vbus(uint32_t now,uint8_t VH,uint8_t VL)
{
	static uint8_t counter = 0;
  	if(Byte2_TO_U16(VH,VL) > 0x6B6C&& SKY_GND_FLAG == GPIO_PIN_SET&&counter == 0&& Vbus28_Full_Flag == 0)
	{
  	 SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);
     LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0); 
	 }
	if(now - tick_V28check > Wait_SSPC_28V )
	{
		tick_V28check = now;
		if(Byte2_TO_U16(VH,VL) > 0x6B6C && SKY_GND_FLAG == GPIO_PIN_SET && Vbus28_Full_Flag == 0)
	   {
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			 counter++; 
			 if(counter == 3)
		  	{
				counter = 0;
				Vbus28_Full_Flag = 1;
				//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);
				//LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0);
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_3,0);
				LogChannelOp(CHN_3,SSPC_FUNC_CHN_CLOSE,0);
				return;				
	    	}
		}
		 else{ 
			counter = 0;
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			return;
		}
	}
	else {return;}
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4)
  {
    GPIO_PinState now_io = SKY_GND_FLAG;
    if(now_io == sky_gnd_key.curr_read)
    {
        sky_gnd_key.filter_cnt++;
        if(sky_gnd_key.filter_cnt >= 20)
        {
            sky_gnd_key.last_stable = now_io;
            sky_gnd_key.filter_cnt = 20;
        }
    }
    else
    {
        sky_gnd_key.curr_read = now_io;
        sky_gnd_key.filter_cnt = 0;
    }
  }
}

