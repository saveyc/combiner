																			   /**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f107.h"
#include "fun.h"
#include "logic_ctrl.h"
#include "can_bus.h"
#include <string.h>

/* Exported function prototypes ----------------------------------------------*/
void Time_Update(void);
void Delay(uint32_t nCount);


#define INVALUE    0
#define VALUE      1

/* ETHERNET errors */
#define  ETH_ERROR              ((uint32_t)0)
#define  ETH_SUCCESS            ((uint32_t)1)
#define  DP83848_PHY_ADDRESS       0x01

#define	UDP_RECV_BUFF_SIZE		300
#define UDP_SEND_BUFF_SIZE              300
#define	CLIENT_DIS_CONNECT		0
#define	CLIENT_CONNECT_OK		1
#define	CLIENT_CONNECT_RECV		2
#define	CLIENT_RE_CONNECT		3

#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
#define DEST_IP_ADDR2   2
#define DEST_IP_ADDR3   100
#define DEST_PORT       3000

#define LOCAL_UDP_PORT  DEST_PORT
/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   mac_addr[0]
#define MAC_ADDR1   mac_addr[1]
#define MAC_ADDR2   mac_addr[2]
#define MAC_ADDR3   mac_addr[3]
#define MAC_ADDR4   mac_addr[4]
#define MAC_ADDR5   mac_addr[5]
/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   2
#define IP_ADDR3   130
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0
/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   2
#define GW_ADDR3   1

#define  IN1_0_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8))&0x1)
#define  IN1_1_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10))&0x1)
#define  IN1_2_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12))&0x1)
#define  IN1_3_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13))&0x1)
#define  IN1_4_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14))&0x1)
#define  IN1_5_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15))&0x1)
#define  IN1_6_STATE            ((~GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1))&0x1)

#define  IN2_0_STATE            ((~GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))&0x1)
#define  IN2_1_STATE            ((~GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8))&0x1)
#define  IN2_2_STATE            ((~GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7))&0x1)
#define  IN2_3_STATE            ((~GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6))&0x1)
#define  IN2_4_STATE            ((~GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_15))&0x1)
#define  IN2_5_STATE            ((~GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_14))&0x1)
#define  IN2_6_STATE            ((~GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13))&0x1)

#define  IN3_0_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9))&0x1)
#define  IN3_1_STATE            ((~GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11))&0x1)

#define  OUT_1_0_(BitVal)       GPIO_WriteBit(GPIOB,GPIO_Pin_9,BitVal)
#define  OUT_1_1_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_0,BitVal)
#define  OUT_1_2_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_1,BitVal)
#define  OUT_1_3_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_2,BitVal)
#define  OUT_1_4_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_3,BitVal)
#define  OUT_1_5_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_4,BitVal)
#define  OUT_1_6_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_5,BitVal)
#define  OUT_1_7_(BitVal)       GPIO_WriteBit(GPIOE,GPIO_Pin_6,BitVal)

#define  DIP1_STATE             GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)
#define  DIP2_STATE             GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)
#define  DIP3_STATE             GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)
#define  DIP4_STATE             GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)
#define  DIP5_STATE             GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)
#define  DIP6_STATE             GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)
#define  DIP7_STATE             GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)
#define  DIP8_STATE             GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3)

#define	 LED_STATE		GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_4)
#define	 LED_ON			GPIO_ResetBits(GPIOD,GPIO_Pin_4)
#define	 LED_OFF		GPIO_SetBits(GPIOD,GPIO_Pin_4)

#define	 UART1_RX_485	        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
#define	 UART1_TX_485	        GPIO_SetBits(GPIOA, GPIO_Pin_11);

#define  SEND_READY             0
#define	 RECV_DATA		1
#define	 SEND_DATA		2
#define  SENDING_DATA           3
#define	 RECV_DATA_END	        4

#define  UserAppEnFlagAddress      0x800F800

#define UART1_BUFF_SIZE  50
extern u8  uart1_send_buff[UART1_BUFF_SIZE];
extern u8  uart1_recv_buff[UART1_BUFF_SIZE];
extern u16 uart1_send_count;
extern u16 uart1_recv_count;
extern u8  uart1_commu_state;
extern u16 uart1_tmr;

extern u16 modulestate_readcnt; 
extern u16 modulestate_index;

void uart_recv_timeout(void);
void uart1_send(void);

void Modbus_RTU_Write_Single_Reg_Cmd(u16 reg_addr,u16 data);
void Modbus_RTU_Write_Multi_Reg_Cmd(u16 station_no,u16 reg_addr,u16 reg_num,u16* data);
void Modbus_RTU_Read_Reg_Cmd(u16 station_no,u16 reg_addr,u16 reg_num);
void Modbus_RTU_Comm_Process(void);
void DEBUG_process(u8 *p_data, u16 len);

extern u8  heart_dely;
extern u8  local_station;
extern u8  isHost;

extern u16 upload_600ms;
extern u16 upload_600ms_3;
extern u16 upload_600ms_2;
extern u16 upload_600ms_1;

extern u8  mac_addr[6];

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

