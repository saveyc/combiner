#ifndef __CAN_BUS_H
#define __CAN_BUS_H
//#pragma pack (1)

#define  EMERGENCY_STOP          0xEE

enum
{
    CAN_FUNC_ID_PARA_DATA = 0x1,  //用户参数命令(读写)
    CAN_FUNC_ID_START_CMD = 0x2,  //启动停止命令(启动命令包含高低速)
    CAN_FUNC_ID_MODULE_STATUS = 0x3,  //模块实时状态信息(周期指令)
    CAN_FUNC_ID_RESET_CMD = 0x4,  //复位命令
    CAN_FUNC_ID_FUNC_SELECT_CMD = 0x5,  //堵包使能命令
    CAN_FUNC_ID_READ_MODULE_STATUS = 0x6, //读取从模块状态信息
    CAN_FUNC_ID_EMERGENCY_STOP_STATUS = 0x7, //急停信号
    CAN_FUNC_ID_UPSTREAM_STOP_CMD = 0x8,     //上游停止信号
    CAN_FUNC_ID_BOARDCAST_CMD = 0x9,         //广播命令集
    CAN_FUNC_ID_BOOT_MODE     = 0xF           // boot 模式
};

// 一帧can可以发送接收完毕的命令
enum
{
    CAN_FUNC_EMERGENCY_RESET_CMD = 0x1,      //急停复位信号                  
};

enum
{
    CAN_SEG_POLO_NONE   = 0,
    CAN_SEG_POLO_FIRST  = 1,
    CAN_SEG_POLO_MIDDLE = 2,
    CAN_SEG_POLO_FINAL  = 3
};
/*
|    28~22    |    21~20      |    19~12     |     11~8     |    7~0      |    ExtID
|  dst_id(7)  |  seg_polo(2)  |  seg_num(8)  |  func_id(4)  |  src_id(8)  |
*/
typedef struct
{
    u8 seg_polo;
    u8 seg_num;
    u8 func_id;
    u8 src_id;
    u8 dst_id;
} sCanFrameExtID;

typedef struct
{
    sCanFrameExtID extId;
    u8 data_len;
    u8 data[8];
} sCanFrameExt;

typedef struct
{
    u8  SegPolo;
    u8  SegNum;
    u16 SegBytes;
}sCanCtrl;


typedef struct {
    sCanFrameExt *queue; /* 指向存储队列的数组空间 */
    u16 front, rear, len; /* 队首指针（下标），队尾指针（下标），队列长度变量 */
    u16 maxSize; /* queue数组长度 */
} sCAN_SEND_QUEUE;

//#pragma pack ()

#define CAN_PACK_DATA_LEN     8
#define CAN_TX_BUFF_SIZE      300
#define CAN_RX_BUFF_SIZE      300



void InitCanSendQueue(void);
void can_bus_frame_receive(CanRxMsg rxMsg);
void can_bus_send_msg(u8* pbuf, u16 send_tot_len, u8 func_id, u8 src_id);
void can_bus_send_read_user_paras(u8 station_no);
void can_bus_reply_read_user_paras(USER_PARAS_T* user_para);
void can_bus_send_write_user_paras(USER_PARAS_T* user_para);
void can_bus_reply_write_user_paras(USER_PARAS_T* user_para);
void can_bus_send_start_cmd(u8 cmd);
void can_bus_send_reset_cmd(void);
void can_bus_send_func_select_cmd(u8* pbuf);
void can_bus_send_module_status(void);
void can_send_frame_process();
void can_bus_send_func_emergency_cmd(void);
void can_bus_send_func_upStreamStop_cmd(void);

void vcanbus_oenframe_send(u8* pbuf, u8 type, u16 len);

#endif