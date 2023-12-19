#ifndef _FUN_H
#define _FUN_H

/********命令定义**********/
#define		MSG_NULL_TYPE			                     0x0000
/**********心跳命令***********/
#define         HEART_MSG_TYPE                           0x1150
/**********主动发送命令*******/
#define		SEND_MSG_MODULE_STATUS_TYPE                  0x1601
#define		SEND_MSG_ALARM_TYPE                          0x1602
/**********接收命令**********/
#define         RECV_MSG_READ_PARA_CMD_TYPE              0x1603
#define         RECV_MSG_WRITE_PARA_CMD_TYPE             0x1604
#define         RECV_MSG_START_CMD_TYPE                  0x1610
#define         RECV_MSG_RESET_CMD_TYPE                  0x1611
#define         RECV_MSG_FUNC_SELECT_CMD_TYPE            0x1612
// 接收急停指令
#define         RECV_MSG_FUNC_EMERGENCY_STOP_TYPE        0x1613 
// boot指令
#define         RECV_MSG_BOOT_CMD_TYPE                   0x1F01
/**********回复接收命令******/
#define		REPLY_RECV_MSG_READ_PARA_CMD_TYPE            0x9603
#define         REPLY_RECV_MSG_WRITE_PARA_CMD_TYPE       0x9604
#define         REPLY_RECV_MSG_START_CMD_TYPE            0x9610
//反馈接收到了急停指令
#define         REPLY_RECV_MSG_FUNC_EMERGENCY_STOP_TYPE  0x9613

#define         HEART_DELAY    5

#pragma pack (1) 
typedef struct {
	u8  MSG_TAG[2];
        u32 MSG_ID;
        u16 MSG_LENGTH;
	u8  MSG_CRC;
	u16 MSG_TYPE;
}MSG_HEAD_DATA;
#pragma pack () 

typedef struct {
        u16 *queue; /* 指向存储队列的数组空间 */
        u16 front, rear, len; /* 队首指针（下标），队尾指针（下标），队列长度变量 */
        u16 maxSize; /* queue数组长度 */
}MSG_SEND_QUEUE;

void InitSendMsgQueue(void);
void AddSendMsgToQueue(u16 msg);
u16 GetSendMsgFromQueue(void);
void recv_message_from_server(u8 *point,u16 *len);
void send_message_to_server(void);

#endif
