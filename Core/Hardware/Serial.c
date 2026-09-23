#include "main.h"
#include <string.h>
#include "Serial.h"
#include "Serial_MSG.h"
#include "OLED.h"
#include "LED.h"
/*USART1
PA9： TX
PA10：RX*/

/*USART3
PB10： TX
PB11： RX*/


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern FC_SendData Read_data;   
extern FC_SendData Zero_data;

EP_DataTypeDef ep_data;
EP_DataTypeDef bat_12v;             // 12V 电池解包数据（RS485）
EP_DataTypeDef bat_28v;             // 28V 电池解包数据（RS485）

static  uint8_t MCU_PowerOn_Flag = 0;
static  uint8_t Packet_Ready_flag;
uint8_t USART1_TxBusy = 0;    //发送忙标志位，1:发送中，0:空闲
uint8_t USART3_TxBusy = 0;    //发送忙标志位，1:发送中，0:空闲

uint8_t USART1_RXBuffer[1];          // 串口1单次中断接收缓冲区
uint8_t USART1_TXBuffer[32];         // 串口1发送缓冲区
uint8_t USART1_RxFrame[32];          // 串口1接收帧缓存
uint8_t USART1_RxIdx    = 0;         // 串口1接收索引
volatile uint8_t USART1_RxFinish = 0;// 串口1一帧接收完成标志
uint32_t U1_last_tick = 0;           // 串口1计时标志


uint8_t USART3_RXBuffer[1];          // 串口3单次中断接收缓冲区
uint8_t USART3_TXBuffer[32];         // 串口3发送缓冲区
uint8_t USART3_RxTmp[EP_Rsp_Len];    // 串口3临时组帧缓存（校验通过后才提交）
uint8_t USART3_RxBuf[2][EP_Rsp_Len]; // 串口3应答槽 [0]=12V [1]=28V
uint8_t USART3_RxIdx    = 0;         // 串口3接收索引
volatile uint8_t USART3_RxReady = 0; // 串口3应答就绪位图 bit0=12V bit1=28V
static uint8_t  U3_State = 0;        // 串口3轮询状态 0=IDLE 1=SLOT_12V 2=SLOT_28V
static uint32_t U3_slot_tick = 0;    // 串口3当前时隙起始tick
//static uint8_t  U3_Data_Type;        // 串口3发送数据类型


/**
 * @brief  中断方式发送串口数据包
 * @param  huart: 串口句柄指针
 * @param  pData: 数据包首地址
 * @param  len:   数据包有效字节长度
 * @retval 1发送繁忙失败，0发送启动成功
 */
uint8_t UART_SendData(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
	  if(huart->Instance == USART1)
	  {
	  	 if(USART1_TxBusy) return 1;
       USART1_TxBusy = 1;
	  }
	  else if(huart->Instance == USART3)
	  {
	  	 if(USART3_TxBusy) return 1;
       USART3_TxBusy = 1;
	  }
    HAL_UART_Transmit_IT(huart, pData, len);
		return 0 ;
}

/**
* @brief 双字拼接16位无符号整数
* @retval
*/
uint16_t Byte2_TO_U16(uint8_t high,uint8_t low)
{
	return ((uint16_t)high << 8) | low;
}

/**
* @brief 协议包配置以及发送(飞控)
* @retval
*/
void FC_Packet_Make(FC_SendData *data)
{
    USART1_TXBuffer[0] = FC_Packet_Head1;
    USART1_TXBuffer[1] = FC_Packet_Head2;

    USART1_TXBuffer[2] = data->Vbus12H;
    USART1_TXBuffer[3] = data->Vbus12L;
                         
    USART1_TXBuffer[4] = data->Vchn8H;  
    USART1_TXBuffer[5] = data->Vchn8L; 
                         
    USART1_TXBuffer[6] = data->Ichn8H; 
    USART1_TXBuffer[7] = data->Ichn8L; 
                          
    USART1_TXBuffer[8] = data->Ichn7H; 
    USART1_TXBuffer[9] = data->Ichn7L; 
                          
    USART1_TXBuffer[10] = data->Ichn6H; 
    USART1_TXBuffer[11] = data->Ichn6L; 
	                        
    USART1_TXBuffer[12] = data->Ichn5H; 
    USART1_TXBuffer[13] = data->Ichn5L; 
                          
    USART1_TXBuffer[14] = data->Ep_collect_V_H; 
    USART1_TXBuffer[15] = data->Ep_collect_V_L; 
		
    USART1_TXBuffer[16] = data->Ep_Current_H; 
    USART1_TXBuffer[17] = data->Ep_Current_L; 
    USART1_TXBuffer[18] = Calc_CheckSum(USART1_TXBuffer, 18);
//  	OLED_ShowHexNum(100,1,USART1_TXBuffer[18],2,OLED_8X16);
//  	OLED_Update();
    UART_SendData(&huart1, USART1_TXBuffer, 19);
}



/**
* @brief 数据拼接及搬运（应急电源）
* @param *buf:读取应急电源接收数据包
* @param *p_accVolt: 输出累计总压原始值
* @param *p_collectVolt:输出采集总压原始值
* @param *p_rawCurr:输出电流原始值
* @param *p_Soc：输出SOC原始值
* @retval
*/
void EP_DataCombine(uint8_t *buf,uint16_t*p_accVolt,uint16_t*p_collectVolt,
	                  int16_t*p_rawCurr,uint16_t*p_Soc)
{
	*p_accVolt     = Byte2_TO_U16(buf[4],buf[5]);
	*p_collectVolt = Byte2_TO_U16(buf[6],buf[7]);
	*p_rawCurr     = Byte2_TO_U16(buf[8],buf[9]);
	*p_Soc         = Byte2_TO_U16(buf[10],buf[11]);
}

/**
* @brief 数据换算及搬运（应急电源）
* @param *buf            :读取应急电源接收数据包
* @param *raw_accVolt    : 输出累计总压原始值
* @param *raw_collectVolt:输出采集总压原始值
* @param *raw_Curr       :输出电流原始值
* @param *raw_Soc        :输出SOC原始值
* @param *out_accVolt    :输出累计电压实际值
* @param *out_collectVolt:输出采集电压实际值
* @param *out_curr       :输出电流实际值
* @param *out_soc        :输出电量实际值
* @retval
*/

void EP_0x90_Convert(uint8_t *buf,uint16_t*raw_accVolt,
	                   uint16_t*raw_collectVolt,int16_t*raw_Curr,uint16_t*raw_soc,
                     float *out_accVolt,float *out_collectVolt,
										 int16_t *out_curr,float *out_soc)
{
	EP_DataCombine(buf,raw_accVolt,raw_collectVolt,raw_Curr,raw_soc);
  
	*out_accVolt     = (float)*raw_accVolt*0.1f;
  *out_collectVolt = (float)*raw_collectVolt*0.1f;
	*out_curr        = (float)*raw_Curr -30000;
	*out_soc         = (float)*raw_soc*0.1f;
}
/**
* @brief 串口三应急电源状态读取
* @param type: 0:电压状态 1：电流状态
* @param data: 传入数组
* @retval type0：0x80:电压过压 0x88：电压欠压
* @retval type1: 0x00:电流正常 0x80：电流过流
*/
uint8_t EP_Status_Read(uint8_t type,uint8_t *data)
{
	if(type == 0)
	{
		if(data[4]& MASK_OP){return 0x80;}
		else if(data[4]& MASK_UP){return 0x88;}
	}
	if(type == 1)
	{
		if(data[6]& MASK_ALL_OVERCUR){return 0x80;}
	}
	return 0;
}

/**
* @brief 12V 电池应答解析（应答帧 13 字节）
* @param buf: 12V 应答帧，buf[4..5]采集总压 [6..7]累计总压 [8..9]电流 [10..11]SOC
*/
static void EP_Parse_12V(const uint8_t *buf)
{
    bat_12v.volt_collect = Byte2_TO_U16(buf[4], buf[5]);   //采集总压
    bat_12v.volt_acc     = Byte2_TO_U16(buf[6], buf[7]);   //累计总压
    bat_12v.raw_current  = (int16_t)Byte2_TO_U16(buf[8], buf[9]);
    bat_12v.soc_raw      = Byte2_TO_U16(buf[10], buf[11]);

    Read_data.Vchn8H = buf[4];   Read_data.Vchn8L = buf[5];   //12V 采集总压
    Read_data.Ichn8H = buf[8];   Read_data.Ichn8L = buf[9];   //12V 电流

    Packet_Ready_flag = 1;      //方案a：任一电池应答成功即置位
}

/**
* @brief 28V 电池应答解析（映射与改造前一致，飞控侧零改动）
* @param buf: 28V 应答帧，buf[4..5]采集总压 [6..7]累计总压 [8..9]电流 [10..11]SOC
*/
static void EP_Parse_28V(const uint8_t *buf)
{
    bat_28v.volt_collect = Byte2_TO_U16(buf[4], buf[5]);   //采集总压
    bat_28v.volt_acc     = Byte2_TO_U16(buf[6], buf[7]);   //累计总压
    bat_28v.raw_current  = (int16_t)Byte2_TO_U16(buf[8], buf[9]);
    bat_28v.soc_raw      = Byte2_TO_U16(buf[10], buf[11]);

    Read_data.Ep_collect_V_H = buf[4];  Read_data.Ep_collect_V_L = buf[5];
    Read_data.Ep_Current_H   = buf[8];  Read_data.Ep_Current_L   = buf[9];

    Packet_Ready_flag = 1;
}

/**
* @brief 累加和校验
* @retval
*/
uint8_t Calc_CheckSum(uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return (uint8_t)(sum % 256);
}


/**
* @brief 协议包配置以及发送(应急电源)
* @param cmd：输入相应的指令
* @retval 1:发送忙 0：发送成功
*/
uint8_t EP_Packet_Make(uint8_t dev_addr, uint8_t cmd)
{
	if(USART3_TxBusy)return 1;
	USART3_TXBuffer[0] = EP_Packet_Head1;
    USART3_TXBuffer[1] = dev_addr;
	
	USART3_TXBuffer[2] = cmd;
	USART3_TXBuffer[3] = 0x08;
	USART3_TXBuffer[4] = 0x00;
	USART3_TXBuffer[5] = 0x00;
	USART3_TXBuffer[6] = 0x00;
    USART3_TXBuffer[7] = 0x00;
    USART3_TXBuffer[8] = 0x00;
    USART3_TXBuffer[9] = 0x00;
    USART3_TXBuffer[10] = 0x00;
    USART3_TXBuffer[11] = 0x00;
	
	USART3_TXBuffer[12] = Calc_CheckSum(USART3_TXBuffer, 12);
	UART_SendData(&huart3,USART3_TXBuffer,EP_Packet_Len);
  return 0;
}


/**
 * @brief 串口发送忙超时等待
 * @param huart 串口句柄
 * @retval 0等待成功空闲 1超时失败
 */
uint8_t UART_TimeOut(UART_HandleTypeDef *huart)
{
    uint32_t tick = 0;
    if(huart->Instance == USART1)
    {
        while(USART1_TxBusy && tick++ < 10000);
    }
    else if(huart->Instance == USART3)
    {
        while(USART3_TxBusy && tick++ < 10000);
    }

    // 超时仍处于忙碌状态返回失败
    if((huart->Instance == USART1 && USART1_TxBusy) ||
       (huart->Instance == USART3 && USART3_TxBusy))
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 串口1轮询访问函数
 * @retval 
 */
void USART1_LoopCall(void)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t tick_diff = tick_now - U1_last_tick;
    // 阶段1：上电3秒内，只发送零帧，周期发送
    if(MCU_PowerOn_Flag == 0)
    {
        if(tick_diff > FC_Zero_Time)
        {
            MCU_PowerOn_Flag = 1;    // 3秒到，切换正常采集模式
            U1_last_tick = tick_now; // 重置计时，开始正常数据周期
        }
        else
        {
            if(tick_diff >= FC_Send_Cycle)
            {
                FC_Packet_Make(&Zero_data); 
            }
        }
    }
    // 阶段2：3秒过后，发送正常采集数据
    else
    {
        
        if(tick_diff >= FC_Send_Cycle && Packet_Ready_flag == 1)
        {
            Packet_Ready_flag = 0;
            U1_last_tick = tick_now;
            FC_Packet_Make(&Read_data); 
        }
    }
}

/**
 * @brief 串口3轮询访问函数
 * @retval 
 */
void USART3_LoopCall(void)
{
    uint32_t tick_now = HAL_GetTick();

    // 1. 先消费已就绪的应答（与状态机解耦，迟到的应答也能落对槽）
    if(USART3_RxReady & 0x01)
    {
        USART3_RxReady &= ~0x01;
        EP_Parse_12V(USART3_RxBuf[0]);
    }
    if(USART3_RxReady & 0x02)
    {
        USART3_RxReady &= ~0x02;
        EP_Parse_28V(USART3_RxBuf[1]);
    }

    // 2. 三态轮询：单台时隙 EP_Slot_MS(40ms)，一轮 80ms，与飞控帧 80ms 对齐
    switch(U3_State)
    {
        case 0:     // IDLE：立刻发 12V 查询
            if(EP_Packet_Make(EP_12V_REQ_ADDR, EP_CMD_0x90_ASK) == 0)
            {
                U3_slot_tick = tick_now;
                U3_State = 1;
            }
            break;

        case 1:     // SLOT_12V：满 40ms 发 28V 查询
            if(tick_now - U3_slot_tick >= EP_Slot_MS)
            {
                if(EP_Packet_Make(EP_28V_REQ_ADDR, EP_CMD_0x90_ASK) == 0)
                {
                    U3_slot_tick = tick_now;
                    U3_State = 2;
                }
            }
            break;

        case 2:     // SLOT_28V：满 40ms 回 IDLE，一轮共 80ms
            if(tick_now - U3_slot_tick >= EP_Slot_MS)
            {
                U3_State = 0;
            }
            break;

        default:
            U3_State = 0;
            break;
    }
}


/**
 * @brief 串口接收回调函数
 * @retval 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
        // 存入单字
		USART1_RxFrame[USART1_RxIdx++] = USART1_RXBuffer[0];
        // 帧头0xEB 0x92，定长6字节
        if(USART1_RxIdx == 1 && USART1_RxFrame[0] != FC_Packet_Head1)
        {
            USART1_RxIdx = 0;
        }
        else if(USART1_RxIdx >= 2)
        {   
            if(USART1_RxFrame[0]== FC_Packet_Head1 && USART1_RxFrame[1]== FC_Address)
            {
                if(USART1_RxIdx >= FC_Rsp_Len)
                {
                    uint16_t sum = 0;
                    for(uint8_t i = 0; i < FC_Rsp_Len-1; i++)
                    {
                        sum += USART1_RxFrame[i];
                    }
                    uint8_t check = sum % 256;
										/*OLED_ShowHexNum(100,1,check,2,OLED_8X16);
										OLED_Update();*/
                    if(check == USART1_RxFrame[5])
                    {
                        USART1_RxFinish = 1;
                    }
                    USART1_RxIdx = 0;
                }
            }
            else
            {
                USART1_RxIdx = 0;
            }
        }
		HAL_UART_Receive_IT(&huart1, USART1_RXBuffer, 1);
	}
 
	else if(huart->Instance == USART3)
	{
		USART3_RxTmp[USART3_RxIdx++] = USART3_RXBuffer[0];
    // 帧头 0xA5，地址0x01(12V)/0x02(28V)，帧长13字节
        if(USART3_RxIdx == 1 && USART3_RxTmp[0] != EP_Packet_Head1)
        {
            USART3_RxIdx = 0;
        }
        else if(USART3_RxIdx >=2)
        {
            if(USART3_RxTmp[0]==EP_Packet_Head1 && (USART3_RxTmp[1]==EP_12V_Address || USART3_RxTmp[1]==EP_28V_Address))
            {
                if(USART3_RxIdx >= EP_Rsp_Len)
                {
                    uint16_t sum = 0;
                    for(uint8_t i = 0; i < EP_Rsp_Len-1; i++)
                    {
                        sum += USART3_RxTmp[i];
                    }
                    uint8_t check = sum % 256;
    //						OLED_ShowHexNum(100,1,check,2,OLED_8X16);
    //						OLED_Update();
                    if(check == USART3_RxTmp[12])
                    {
                        // 校验通过，按应答地址投递到对应槽
                        if(USART3_RxTmp[1] == EP_12V_Address)
                        {
                            memcpy(USART3_RxBuf[0], USART3_RxTmp, EP_Rsp_Len);
                            USART3_RxReady |= 0x01;
                        }
                        else
                        {
                            memcpy(USART3_RxBuf[1], USART3_RxTmp, EP_Rsp_Len);
                            USART3_RxReady |= 0x02;
                        }
                    }
                    USART3_RxIdx = 0;
                }
            }
            else
            {
                USART3_RxIdx = 0;
            }
        }
        HAL_UART_Receive_IT(&huart3, USART3_RXBuffer, 1);
    }
}

/**
 * @brief 串口发送回调函数
 * @retval 
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        USART1_TxBusy = 0;
    }
    else if(huart->Instance == USART3)
    {
        USART3_TxBusy = 0;
    }
}


/**
 * @brief 串口错误回调（ORE/FE/NE/PE）：清标志并重挂接收，防止永久失聪
 * @retval
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);     //读SR+DR，清PE/FE/NE/ORE
        USART1_RxIdx = 0;
        HAL_UART_Receive_IT(&huart1, USART1_RXBuffer, 1);
    }
    else if(huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_PEFLAG(huart);     //读SR+DR，清PE/FE/NE/ORE
        USART3_RxIdx = 0;
        HAL_UART_Receive_IT(&huart3, USART3_RXBuffer, 1);
    }
}
