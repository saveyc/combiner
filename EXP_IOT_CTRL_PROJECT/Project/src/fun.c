#include "main.h"
#include "udpclient.h"

#define msgSendQueueSize  100
u16 msgQueueBuff[msgSendQueueSize];
MSG_SEND_QUEUE msgSendQueue;

void initQueue(MSG_SEND_QUEUE *q, u16 ms)
{
    q->maxSize = ms;
    q->queue = msgQueueBuff;
    q->front = q->rear = 0;
}
void enQueue(MSG_SEND_QUEUE *q, u16 x)
{
    //队列满
    if((q->rear + 1) % q->maxSize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxSize; // 求出队尾的下一个位置
    q->queue[q->rear] = x; // 把x的值赋给新的队尾
}
u16 outQueue(MSG_SEND_QUEUE *q)
{
    //队列空
    if(q->front == q->rear)
    {
        return 0;
    }
    q->front = (q->front + 1) % q->maxSize; // 使队首指针指向下一个位置
    return q->queue[q->front]; // 返回队首元素
}
void InitSendMsgQueue(void)
{
    initQueue(&msgSendQueue, msgSendQueueSize);
}
void AddSendMsgToQueue(u16 msg)
{
    enQueue(&msgSendQueue, msg);
}
u16 GetSendMsgFromQueue(void)
{
    return (outQueue(&msgSendQueue));
}
u8 recv_msg_check(u8 *point, u16 len)
{
    u16 i = 0;
    u8  sum = 0;

    if((point[0] != 0xAA) || (point[1] != 0xAA))
        return 0;
    if((point[6] | point[7] << 8) != len)
        return 0;
    sum = point[9];
    for(i = 1 ; i < len - 9 ; i++)
    {
        sum ^= point[9 + i];
    }
    if(sum != point[8])
        return 0;

    return 1;
}

void recv_msg_process(u8 *point)
{
    MSG_HEAD_DATA *head = (MSG_HEAD_DATA *)point;
    u8 buf[10] = { 0 };
    
    if( head->MSG_TYPE == RECV_MSG_BOOT_CMD_TYPE )
    {
        //BKP_WriteBackupRegister(BKP_DR8, 0x55);
        //NVIC_SystemReset();
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
        if(FLASH_ErasePage(UserAppEnFlagAddress) == FLASH_COMPLETE)
        {
            FLASH_ProgramHalfWord(UserAppEnFlagAddress,0x0000);
        }
        FLASH_Lock();
        NVIC_SystemReset();
    }
    switch(head->MSG_TYPE)
    {
    case RECV_MSG_READ_PARA_CMD_TYPE:
        if(point[11] == local_station)
        {
            user_paras_slave = user_paras_local;
            AddSendMsgToQueue(REPLY_RECV_MSG_READ_PARA_CMD_TYPE);
        }
        else
        {
            can_bus_send_read_user_paras(point[11]);
        }
        break;
    case RECV_MSG_WRITE_PARA_CMD_TYPE:
        user_paras_temp = *(USER_PARAS_T *)(point+sizeof(MSG_HEAD_DATA));
        if(user_paras_temp.Station_No == local_station)
        {
            write_user_paras((u16*)(point+11+6));
            read_user_paras();
            user_paras_slave = user_paras_local;
            AddSendMsgToQueue(REPLY_RECV_MSG_WRITE_PARA_CMD_TYPE);
        }
        else
        {
            can_bus_send_write_user_paras(&user_paras_temp);
        }
        break;
    case RECV_MSG_START_CMD_TYPE:             //启停信息
        Reset_Ctrl_Handle();                  //有故障先复位故障
        reset_start_time_cnt = 0;
        Speed_Ctrl_Process(point[11]);
        vcanbus_oenframe_send(&(point[11]), CAN_FUNC_ID_START_CMD,1);
//        can_bus_send_start_cmd(point[11]);
        break;
    case RECV_MSG_RESET_CMD_TYPE:            //复位信息
        Reset_Ctrl_Handle();
        reset_start_time_cnt = 500;
        //can_bus_send_reset_cmd();
        vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_RESET_CMD, 0);
        g_emergency_stop = 0;                        //清除急停状态
        break;
    case RECV_MSG_FUNC_SELECT_CMD_TYPE:        //功能选择
        g_block_disable_flag = point[11];
        //can_bus_send_func_select_cmd(point+11);
        vcanbus_oenframe_send(&(point[11]), CAN_FUNC_ID_FUNC_SELECT_CMD, 1);
        break;
    case RECV_MSG_FUNC_EMERGENCY_STOP_TYPE:   //急停指令
        Speed_Ctrl_Process(EMERGENCY_STOP);
        //can_bus_send_func_emergency_cmd();
        vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_EMERGENCY_STOP_STATUS, 0);
        g_emergency_stop = 1;                        //清除急停状态
//        AddSendMsgToQueue(REPLY_RECV_MSG_FUNC_EMERGENCY_STOP_TYPE);
        break;
    default:
        break;
    }
}
void recv_message_from_server(u8 *point, u16 *len)
{
    if(recv_msg_check(point, *len) == 0)
    {
        *len = 0;
        return;
    }
    recv_msg_process(point);
    *len = 0;
}
void send_heart_msg(u8 *buf, u16 *len, u16 type)
{
    u8 sum = (type & 0xFF) ^ (type >> 8);

    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x0B;
    buf[7] = 0x00;
    buf[8] = sum;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;

    *len = 11;
}
void send_reply_read_para_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;

    sendlen = 11 + 12 + sizeof(BELT_PARAS_T)*user_paras_slave.Belt_Number;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    for(i = 0; i < sendlen - 11 ; i++)
    {
        buf[11 + i] = *((u8*)(&user_paras_slave)+i);
    }
    sum = buf[9];
    for(i = 1 ; i < sendlen - 9 ; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;

    *len = sendlen;
}
void send_reply_write_para_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;

    sendlen = 11 + 12 + sizeof(BELT_PARAS_T)*user_paras_slave.Belt_Number;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    for(i = 0; i < sendlen - 11 ; i++)
    {
        buf[11 + i] = *((u8*)(&user_paras_slave)+i);
    }
    sum = buf[9];
    for(i = 1 ; i < sendlen - 9 ; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;

    *len = sendlen;
}
void send_msg_module_status_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;
    MODULE_STATUS_T* module_status_t;
    
    module_status_t = GetModuleStatusDataFromQueue();
    
    if(module_status_t == NULL)
    {
        *len = 0;
        return;
    }
    
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = 1;//反馈信息的模块数
    buf[12] = (*module_status_t).station_no;//模块站号
    buf[13] = (*module_status_t).belt_number;//皮带数量
    memcpy(buf+14,(u8*)((*module_status_t).inverter_status),sizeof(INVERTER_STATUS_T)*(*module_status_t).belt_number);
    sendlen = 14 + sizeof(INVERTER_STATUS_T)*(*module_status_t).belt_number;
    sum = buf[9];
    for(i = 1 ; i < sendlen - 9 ; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;

    *len = sendlen;
}
void send_message_to_server(void)
{
    u16 msg_type;

    if(tcp_client_list[0].tcp_send_en == 0)//tcp发送成功或没在发送
    {
        msg_type = GetSendMsgFromQueue();
    }
    else//tcp正在发送
    {
        return;
    }

    switch(msg_type)
    {
    case SEND_MSG_MODULE_STATUS_TYPE:
        send_msg_module_status_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        if(tcp_client_list[0].tcp_send_len != 0)
        {
            tcp_client_list[0].tcp_send_en = 1;
        }
        break;
    case REPLY_RECV_MSG_READ_PARA_CMD_TYPE:
        send_reply_read_para_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case REPLY_RECV_MSG_WRITE_PARA_CMD_TYPE:
        send_reply_write_para_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    default:
        if(tcp_client_list[0].tcp_client_statue == CLIENT_CONNECT_OK)
        {
//            if( heart_dely == 0 )
//            {
//                send_heart_msg(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), HEART_MSG_TYPE);
//                tcp_client_list[0].tcp_send_en = 1;
//                heart_dely = HEART_DELAY;
//            }
        }
        break;
    }
}
