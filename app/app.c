#include <string.h>
#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "lorawan_node_driver.h"
#include "hdc1000.h"
#include "sensors_test.h"


extern DEVICE_MODE_T device_mode;
extern DEVICE_MODE_T *Device_Mode_str;
down_list_t *pphead = NULL;
uint8_t devEUI[]="AT+DEVEUI=009569000000F627,D391010220102816,1\r\n";
uint8_t appEUI[]="AT+APPEUI=88650AA038E73FFB\r\n";
uint8_t appKEY[]="AT+APPKEY=7054061A8CA122107F6F8FF3FCE2DDC2,0\r\n";
uint8_t appclassc[]="AT+CLASS=2\r\nAT+CONFIRM=1\r\nAT+SAVE\r\n";
uint8_t timesync[]="AT+TIMESYNC\r\n";

uint8_t time[30];
uint8_t local_time[6];
uint32_t gettimetick;
uint32_t tickcount;

void SetLocalTime(uint8_t time[30]);

//-----------------Users application--------------------------
void LoRaWAN_Func_Process(void)
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
				if(i > 0 && i < 3 && HAL_GetTick() > gettimetick+ 3000)
				{
					lpusart1_send_data(timesync,sizeof(timesync));
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
                return;
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
					  Node_Hard_Reset();

            lpusart1_send_data(devEUI,sizeof(devEUI));
            lpusart1_send_data(appEUI,sizeof(appEUI));
            lpusart1_send_data(appKEY,sizeof(appKEY));
			
            lpusart1_send_data(appclassc,sizeof(appclassc));
            if(nodeJoinNet(JOIN_TIME_120_SEC) == false)
            {
                return;
            }

            //get time 
            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
            lpusart1_send_data(timesync,sizeof(timesync));
						i = 1;
						HAL_Delay(10);
            while(true)   //开机后同步时间，如果没有获取到就重新同步
            {
							
							  HAL_Delay(300);
                if(UART_TO_LRM_RECEIVE_FLAG)
                {
                    UART_TO_LRM_RECEIVE_FLAG = 0;
                    if(strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO: ") != NULL)
                    {
											  memset(time,0,30);
											  memcpy(time,strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO:") + 11,17);
                        debug_printf("GetRightTime\r\n");
											  SetLocalTime(time);
											  tickcount = HAL_GetTick();
                        break;
                    }
                }
								i++;
								if(i > 3)
								{
									lpusart1_send_data(timesync,sizeof(timesync));
									i = 0;
								}
            }
						
        }
        else
        {
           
        }
				if(HAL_GetTick() >= tickcount + 1000)
				{
					tickcount = HAL_GetTick();
					local_time[5]++;
					if(local_time[5] >= 60)  //每分钟同步一次时间
					{
						lpusart1_send_data(timesync,sizeof(timesync));
						i = 1;
						HAL_Delay(10);
            while(true)
            {
							HAL_Delay(300);
							if(UART_TO_LRM_RECEIVE_FLAG)
							{
									UART_TO_LRM_RECEIVE_FLAG = 0;
									if(strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO: ") != NULL)
									{
											memset(time,0,30);
											memcpy(time,strstr((char *)UART_TO_LRM_RECEIVE_BUFFER,"+RTC INFO:") + 11,17);
											debug_printf("GetRightTime\r\n");
											SetLocalTime(time);
											tickcount = HAL_GetTick();
											break;
									}
							}
							i++;
							if(i > 3)
							{
								lpusart1_send_data(timesync,sizeof(timesync));
								i = 0;
							}
            }
					}
				}
    }
    break;

    default:
        break;
    }
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
	for(int j = 0;j < 2;j++)
	{
		debug_printf("%02d.",local_time[j]);
	}
	debug_printf("%02d  ",local_time[2]);
	for(int j = 3;j < 5;j++)
	{
		debug_printf("%02d:",local_time[j]);
	}
	debug_printf("%02d\r\n",local_time[5]);
}
