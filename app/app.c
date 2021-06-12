#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "common.h"
#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "lorawan_node_driver.h"
#include "hdc1000.h"
#include "sensors_test.h"
#include "opt3001.h"
#include "mpl3115.h"
#include "ST7789v.h"
#include "XPT2046.h"

#define MAX_DATA_LEN 80

extern DEVICE_MODE_T device_mode;
extern DEVICE_MODE_T *Device_Mode_str;
down_list_t *pphead = NULL;
char devEUI[]="AT+DEVEUI=009569000000F627,D391010220102816,1";
char appEUI[]="AT+APPEUI=88650AA038E73FFB";
char appKEY[]="AT+APPKEY=7054061A8CA122107F6F8FF3FCE2DDC2,0";
char appclassc[]="AT+CLASS=2\r\nAT+CONFIRM=1\r\nAT+SAVE";
char timesync[]="AT+TIMESYNC";

//0:未收到; 1:仅收到帧头; 2:收到合法控制指令; 3:收到帧头帧尾但是长度不对; 
//4:收到的不是控制指令;
uint8_t cmd_check = 0;  

//0:不自动上报; 1:立即上报一次; 2:每分钟上报数据;
uint8_t Data_Up = 0;

uint8_t send_buf[13];

uint8_t time[30];
uint8_t local_time[6];
char slocal_time[8];
uint32_t gettimetick;
uint32_t tickcount;

float get_temp;   //温度
float get_lux;    //光照
float get_humi;   //湿度
float get_Press;  //气压

uint32_t data_tickcount = 0;
uint8_t data_i = 0;
uint16_t data_lux[MAX_DATA_LEN] = {0};

void SetLocalTime(uint8_t time[30]);
void UpData(void);


//-----------------Users application--------------------------
int LoRaWAN_Func_Process(void)
{
    static DEVICE_MODE_T dev_stat = NO_MODE;

    uint16_t temper = 0;
	  int i = 0;

    switch((uint8_t)device_mode)
    {
    /* 指令模式 */
    case CMD_CONFIG_MODE:
    {
        /* 如果不是command Configuration function, 则进入if语句,只执行一次 */
        if(dev_stat != CMD_CONFIG_MODE)
        {
            dev_stat = CMD_CONFIG_MODE;
            debug_printf("\r\n[Command Mode]\r\n");

            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
        }
        /* 等待usart2产生中断 */
        if(UART_TO_PC_RECEIVE_FLAG)
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
					  if(strstr((char *)UART_TO_PC_RECEIVE_BUFFER,"AT+TIMESYNC") != NULL)
						{
							i = 1;
							gettimetick = HAL_GetTick();
						}
            lpusart1_send_data(UART_TO_PC_RECEIVE_BUFFER,UART_TO_PC_RECEIVE_LENGTH);
//						HAL_Delay(1000);
        }
        /* 等待lpuart1产生中断 */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            if(strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO:") != NULL)
						{
//							memset(time,0,30);
							memcpy(time,strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO: ") + 11,17);
							debug_printf("GetRightTime:\r\n");
//							debug_printf("%s\r\n",time);
							gettimetick = 0;
							i = 0;
							tickcount = HAL_GetTick();
							SetLocalTime(time);
						}
//            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
				if(i > 0 && i < 3 && HAL_GetTick() > gettimetick+ 5000)
				{
//					lpusart1_send_data(timesync,sizeof(timesync));
					nodeCmdConfig(timesync);
					gettimetick = HAL_GetTick();
					i++;
				}
    }
    break;

    /* 透传模式 */
    case DATA_TRANSPORT_MODE:
    {
        /* 如果不是data transport function,则进入if语句,只执行一次 */
        if(dev_stat != DATA_TRANSPORT_MODE)
        {
            dev_stat = DATA_TRANSPORT_MODE;
            debug_printf("\r\n[Transperant Mode]\r\n");

            /* 模块入网判断 */
            if(nodeJoinNet(JOIN_TIME_120_SEC) == false)
            {
                return 1;
            }

            temper = HDC1000_Read_Temper()/1000;

            nodeDataCommunicate((uint8_t*)&temper,sizeof(temper),&pphead);
        }

        /* 等待usart2产生中断 */
        if(UART_TO_PC_RECEIVE_FLAG && GET_BUSY_LEVEL)  //Ensure BUSY is high before sending data
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            nodeDataCommunicate((uint8_t*)UART_TO_PC_RECEIVE_BUFFER, UART_TO_PC_RECEIVE_LENGTH, &pphead);
        }

        /* 如果模块正忙, 则发送数据无效，并给出警告信息 */
        else if(UART_TO_PC_RECEIVE_FLAG && (GET_BUSY_LEVEL == 0))
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            debug_printf("--> Warning: Don't send data now! Module is busy!\r\n");
        }

        /* 等待lpuart1产生中断 */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /*工程模式*/
    case PRO_TRAINING_MODE:
    {

        /* 如果不是Class C云平台数据采集模式, 则进入if语句,只执行一次 */
        if(dev_stat != PRO_TRAINING_MODE)
        {
            dev_stat = PRO_TRAINING_MODE;
            debug_printf("\r\n[Project Mode]\r\n");
//					  Node_Hard_Reset();

//            lpusart1_send_data(devEUI,sizeof(devEUI));
//            lpusart1_send_data(appEUI,sizeof(appEUI));
//            lpusart1_send_data(appKEY,sizeof(appKEY));
//			
//            lpusart1_send_data(appclassc,sizeof(appclassc));
					  nodeCmdConfig(devEUI);
					  nodeCmdConfig(appEUI);
					  nodeCmdConfig(appKEY);
					  nodeCmdConfig(appclassc);
					  
					  LCD_Clear(WHITE);
					  LCD_ShowString(30,120,"Connecting To Server...",BLUE);
					  
            if(nodeJoinNet(JOIN_TIME_120_SEC) == false)
						{
							LCD_Clear(WHITE);
							LCD_ShowString(30,120,"Join FAILED!",BLUE);
              return 1;
						}

            //get time 
					  nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);        
//            lpusart1_send_data(timesync,sizeof(timesync));
						nodeCmdConfig(timesync);
						LCD_Clear(WHITE);
					  LCD_ShowString(30,120,"TIME SYNC...",BLUE);
						i = 1;
						HAL_Delay(10);
            while(true)   //开机后同步时间，如果没有获取到就重新同步
            {
							
							  HAL_Delay(5000);
                if(UART_TO_LRM_RECEIVE_FLAG)
                {
                    UART_TO_LRM_RECEIVE_FLAG = 0;
                    if(strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO: ") != NULL)
                    {
											  i = 0;
											  memset(time,0,30);
											  memcpy(time,strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO:") + 11,17);
                        debug_printf("GetRightTime\r\n");
											  SetLocalTime(time);
											  tickcount = HAL_GetTick();
											  data_tickcount = HAL_GetTick();
                        break;
                    }
                }
								
								if(i <= 3)
								{
									i++;
//								  lpusart1_send_data(timesync,sizeof(timesync));
									nodeCmdConfig(timesync);
								}
								else
								{
									debug_printf("Time Sync Error!\r\n");
									i = 0;
									return 1;
								}
            }
						
						/*这里LCD显示设备EUI,第一次时间，屏幕尺寸240*320*/
						LCD_Clear(WHITE);
						sprintf(slocal_time,"%02d:%02d:%02d",local_time[3],local_time[4],local_time[5]);
						LCD_ShowString(8,8,"DevEUI:009569000000F627",BLUE);
						LCD_ShowString(8,24,"First Sync Time:",BLUE);
						LCD_ShowString(8,40,slocal_time,BLUE);
						LCD_ShowString(8,56,"Data Demodulation",BLUE);
						LCD_ShowString(30,72,"Report Mode:",BLUE);
						LCD_ShowString(30,104,"Upload Data:",BLUE);
        }
				

				if(HAL_GetTick() >= data_tickcount + 500)
				{
					/*获取传感器温度、亮度、湿度、气压*/
					get_temp = HDC1000_Read_Temper() / 1000.0;
					get_lux = OPT3001_Get_Lux();
					get_humi = HDC1000_Read_Humidi() / 1000.0;
					get_Press = MPL3115_ReadPressure() / 100000.0;
					
					data_tickcount = HAL_GetTick();
					data_lux[data_i] = 320 - get_lux / 50;
					data_lux[data_i] = (data_lux[data_i] >= 200) ? data_lux[data_i] : 200;
					data_lux[data_i] = (data_lux[data_i] <= 320) ? data_lux[data_i] : 320;
//					debug_printf("%d",get_lux);
					if(data_i < MAX_DATA_LEN - 1)
						data_i++;
					else
					{
						memcpy(data_lux,data_lux + 1,2 * (MAX_DATA_LEN - 1));
						data_i = MAX_DATA_LEN - 1;
					}
					
					LCD_Fill(0,200,240,320,WHITE);
//					LCD_DrawLine(0,250,3,320,RED);
//					LCD_DrawLine(3,320,6,180,RED);
//					LCD_DrawLine(6,180,9,320,RED);
					for(int j = 1;j < data_i;j++)
					{
						LCD_DrawLine(3*(j - 1),(uint16_t)data_lux[j - 1],3*j,(uint16_t)data_lux[j],RED);
					}
				}
				
				if(UART_TO_LRM_RECEIVE_FLAG)
				{
					UART_TO_LRM_RECEIVE_FLAG = 0;
					cmd_check = 4;
					
					if(UART_TO_LRM_RECEIVE_BUFFER[0] == 0xBB)
					{
						cmd_check = 1;
						
						if(UART_TO_LRM_RECEIVE_BUFFER[3] == 0x0F)
						{
							cmd_check = 2;
							if(UART_TO_LRM_RECEIVE_LENGTH != 4)
								cmd_check = 3;
						}
						switch(cmd_check)
						{
							case 1:
								debug_printf("No end.\r\n");
								break;
							case 2:
								if(UART_TO_LRM_RECEIVE_BUFFER[1] == 0x00)
								{
									Data_Up = 1;
									LCD_ShowString(30,88,"Once",BLUE);
								}
								if(UART_TO_LRM_RECEIVE_BUFFER[1] == 0x01)
								{
									Data_Up = 2;
									LCD_ShowString(30,88,"Every Minute",BLUE);
								}
								break;
							case 3:
								debug_printf("Length Err\r\n");
								break;
							case 4:
								debug_printf("Not cmd.\r\n");
								break;
							default:
								debug_printf("程序好像出问题了\r\n");
								break;
						}
						
						if(cmd_check == 2 && Data_Up == 1)
						{
              debug_printf("Update Once\r\n");
							UpData();
							cmd_check = 0;
							Data_Up = 0;
							debug_printf("Update Complete\r\n");
						}
					}
				}
				
				if(HAL_GetTick() >= tickcount + 1000)
				{
					tickcount = HAL_GetTick();
					local_time[5]++;
					if(local_time[5] >= 60)  //每分钟同步一次时间
					{
						debug_printf("Auto Time SYNC\r\n");
//						lpusart1_send_data(timesync,sizeof(timesync));
						nodeCmdConfig(timesync);
						i = 1;
            while(true)
            {
							HAL_Delay(500);
							if(UART_TO_LRM_RECEIVE_FLAG)
							{
									UART_TO_LRM_RECEIVE_FLAG = 0;
									if(strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO: ") != NULL)
									{
										  i = 0;
											memset(time,0,30);
											memcpy(time,strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO:") + 11,17);
											debug_printf("GetRightTime\r\n");
											SetLocalTime(time);
											tickcount = HAL_GetTick();
											break;
									}
							}
							if(i >= 10)
							{
								i = 0;
//								lpusart1_send_data(timesync,sizeof(timesync));
								nodeCmdConfig(timesync);
								debug_printf("Time Sync Again!\r\n");
							}
							else
							{
								i++;
							}
            }
						if(Data_Up == 2) //整分钟上传
						{
							debug_printf("Minutes Update.\r\n");
							UpData();
						}
					}
				}
    }
    break;

    default:
			LCD_ShowString(30,120,"Press KEY2 TO",BLUE);
		  LCD_ShowString(30,150,"ENTER PROJECT MODE",BLUE);
      return 0;
    }
		return 0;
}


/**
 * @brief   开发板版本信息和按键使用说明信息打印
 * @details 上电所有灯会短暂亮100ms
 * @param   无
 * @return  无
 */
void LoRaWAN_Borad_Info_Print(void)
{
    debug_printf("\r\n\r\n");
    PRINT_CODE_VERSION_INFO("%s",CODE_VERSION);
    debug_printf("\r\n");
    debug_printf("--> Press Key1 to: \r\n");
    debug_printf("-->  - Enter command Mode\r\n");
    debug_printf("-->  - Enter Transparent Mode\r\n");
    debug_printf("--> Press Key2 to: \r\n");
    debug_printf("-->  - Enter Project Trainning Mode\r\n");
    LEDALL_ON;
    HAL_Delay(100);
    LEDALL_OFF;
}

void SetLocalTime(uint8_t time[30])
{
	local_time[0] = atoi((char *)time);
	local_time[1] = atoi((char *)time + 3);
	local_time[2] = atoi((char *)time + 6);
	local_time[3] = atoi((char *)time + 9);
	local_time[4] = atoi((char *)time + 12);
	local_time[5] = atoi((char *)time + 15);
	debug_printf("LocalTime:");
	debug_printf("20%02d.%02d.%02d  ",local_time[0],local_time[1],local_time[2]);
//	for(int j = 0;j < 2;j++)
//	{
//		debug_printf("%02d.",local_time[j]);
//	}
//	debug_printf("%02d  ",local_time[2]);
	for(int j = 3;j < 5;j++)
	{
		debug_printf("%02d:",local_time[j]);
	}
	debug_printf("%02d\r\n",local_time[5]);
}

void UpData()
{
	memset(send_buf,0xFF,13);
	
	send_buf[0] = 0xAA;
	send_buf[12] = 0x0F;
	send_buf[1] = local_time[3] / 10 * 16 + local_time[3] % 10;
	send_buf[2] = local_time[4] / 10 * 16 + local_time[4] % 10;
	send_buf[3] = local_time[5] / 10 * 16 + local_time[5] % 10;
	
	if(UART_TO_LRM_RECEIVE_BUFFER[2] & 0x01)
	{
		LCD_ShowString(30,120,"LUX:Y",BLUE);
		send_buf[4] = get_lux / 255;
		send_buf[5] = (uint8_t)get_lux - send_buf[4] * 255;
	}
	else
		LCD_ShowString(30,120,"LUX:N",BLUE);
	if(UART_TO_LRM_RECEIVE_BUFFER[2] & 0x02)
	{
		LCD_ShowString(30,136,"PRESSURE:Y",BLUE);
		send_buf[6] = get_Press / 255;
		send_buf[7] = (uint8_t)get_Press - send_buf[6] * 255;
	}
	else
		LCD_ShowString(30,136,"PRESSURE:N",BLUE);
	if(UART_TO_LRM_RECEIVE_BUFFER[2] & 0x04)
	{
		LCD_ShowString(30,152,"TEMPERATURE:Y",BLUE);
		send_buf[8] = get_temp / 255;
		send_buf[9] = (uint8_t)get_temp - send_buf[8] * 255;
	}
	else
		LCD_ShowString(30,152,"TEMPERATURE:N",BLUE);
	if(UART_TO_LRM_RECEIVE_BUFFER[2] & 0x08)
	{
		LCD_ShowString(30,168,"HUMIDITY:Y",BLUE);
		send_buf[10] = get_humi / 255;
		send_buf[11] = (uint8_t)get_humi - send_buf[10] * 255;
	}
	else
		LCD_ShowString(30,168,"HUMIDITY:N",BLUE);
	nodeDataCommunicate((uint8_t *)send_buf,13,&pphead);
	debug_printf("Sending...\r\n");

}
