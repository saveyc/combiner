#include "main.h"

USER_PARAS_T  user_paras_local;
USER_PARAS_T  user_paras_slave;
USER_PARAS_T  user_paras_temp;

sButton_Info  bPhoto1Info;
sButton_Info  bPhoto2Info;
sButton_Info  bPhoto3Info;
sButton_Info  bPhoto4Info;
sButton_Info  bPhoto5Info;
sButton_Info  bResetInfo;
sButton_Info  bStartInfo;
sButton_Info  bStopInfo;
sButton_Info  bStreamInfo;
sButton_Info  bEmergencyInfo;

//add 23/12/12
//急停复位信号
sButton_Info  bEmergencyResetInfo;
//上游屯包光电
sButton_Info  bUpStockInfo;
//下游屯包光电
sButton_Info  bDownStockInfo;
//合流机控制信号
sButton_Info  bCombinerInInfo;
//屯包插包切换信号
sButton_Info  bSwitchInInfo;

INVERTER_STATUS_T  inverter_status_buffer[10];//本机状态

#define moduleStatusQueueSize  20
MODULE_STATUS_T  moduleStatusBuff[moduleStatusQueueSize];
MODULE_STATUS_QUEUE  moduleStatusQueue;

#define uartSendQueueSize  50
COMM_NODE_T uartSendQueueBuff[uartSendQueueSize];
COMM_SEND_QUEUE uartSendQueue;
COMM_NODE_T comm_node;

// 
#define uarttmpQueueSize  60
COMM_NODE_T uarttmpQueueBuff[uarttmpQueueSize];

u8  comm_busy_flag;
u8  polling_num;//轮询站号(从1开始)

u8  g_remote_start_flag;   //远程起停状态改变标记(0:无变化 1:停止变启动 2:启动变停止)
u8  g_remote_start_status; //远程起停状态(0:停止状态 1:运行状态)
u8  g_remote_speed_status; //远程高低速状态(0:低速 1:高速)
u8  g_link_up_stream_status; //联动上游信号
u8  g_link_down_stream_status; //联动下游信号
u8  g_set_start_status; //设置电机起停状态(0:停止 1:运行)
u8  g_set_speed_status; //设置电机高低速状态(0:低速 1:高速)
u8  g_read_start_status; //当前的电机起停状态(0:停止 1:运行)
u8  g_alarm_type; //故障类别(bit0:电机故障,bit1:堵包故障,bit2:485通讯故障)
u8  g_speed_gear_status; //当前的速度档位(0:停止 1~5:五档速度)
u8  g_block_disable_flag = 0; //堵包检测功能禁止标记
// add 2023/11/02
u8  g_link_down_phototrig_status = 0;      //下游的光电是否触发过

//u16 start_delay_time_cnt;
//u16 stop_delay_time_cnt;
u32 block_check_time_cnt[10];
u16 reset_start_time_cnt = 0;
u8  reset_start_flag = 0;
// can 接收到的急停信号状态 add 2023/11/02
u8  g_emergency_stop = 0;

// 每一段皮带的停止状态保持倒计时 add 2023/11/02
u16 logic_upload_stopStatus[BELT_NUMMAX] = { 0 };
// 每一段皮带上次的速度记录 add 2023/11/02
u16 logic_upload_lastRunStatus[BELT_NUMMAX] = { 0 };

// 屯包判断
u16 stockFullFlag = 0;

//add 240131  直接联动的情况下  启动必须要判断联动信号   0无直接联动  1 有直接联动
u16 extralAllow = 1;

//add 240220  插包模式 或者是屯包模式  1 插包模式 0 屯包模式
u16 stockInsertMod = 0;

//add 240226  启动时有一段时间保持低速
u16 startKeepLow = 0;

//add 240226  设置的目标速度
u16 setSpeedGear = 0;

//240304
sMODULE_ERR_T   moduleErrt;

void LogicModuleErrInit(void)
{   
    u16 i = 0;
    u16 j = 0;
    moduleErrt.start_cnt = 0;
    moduleErrt.stop_cnt = 0;

    for (i = 0; i < MODULE_NUMSTATE; i++) {
        for (j = 0; j < 10; j++) {
            moduleErrt.moduleErr[i].moduleerr[j] = 0;
            moduleErrt.moduleErr[i].inputState[j] = 0xFFFF;
        }
    }
}

void LogicModuleErrOutput(void)
{
    u16 i = 0;
    u8 blockflag = 0;
    u8 localflag = 0;
    u8 errflag = 0;
    u16 j = 0;

    if (isHost == 0) {
        return;
    }
    if (moduleErrt.start_cnt > 0) {
        moduleErrt.start_cnt--;
    }
    if (moduleErrt.stop_cnt > 0) {
        moduleErrt.stop_cnt--;
    }

    if ((g_speed_gear_status == 0) || (g_speed_gear_status == EMERGENCY_STOP)) {
        B_RUN_1_OUT_(Bit_SET);
        B_RUN_2_OUT_(Bit_RESET);
    }
    else {
        B_RUN_1_OUT_(Bit_RESET);
        B_RUN_2_OUT_(Bit_SET);
    }


    for (i = 0; i < MODULE_NUMSTATE; i++) {
        for (j = 0; j < 10; j++) {
            if (((moduleErrt.moduleErr[i].moduleerr[j] >> 1) & 0x1) == 1) {
                blockflag = 1;
            }
            if ((moduleErrt.moduleErr[i].moduleerr[j] & 0x1) == 1) {
                errflag = 1;
            }

            if (((moduleErrt.moduleErr[i].moduleerr[j] >> 8) & 0xFF) == 1) {
                errflag = 1;
            }
            if (((moduleErrt.moduleErr[i].inputState[j] >> 1) & 0x1) == 0) {
                localflag = 1;
            }
        }
    }
    if (errflag == 1) {
        B_RUN_3_OUT_(Bit_SET);
    }
    else {
        B_RUN_3_OUT_(Bit_RESET);
    }

    if (blockflag == 1) {
        B_RUN_4_OUT_(Bit_SET);
    }
    else {
        B_RUN_4_OUT_(Bit_RESET);
    }

    if (localflag == 1) {
        B_RUN_5_OUT_(Bit_SET);
    }
    else {
        B_RUN_5_OUT_(Bit_RESET);
    }
}

void InitUartSendQueue(void)
{
    COMM_SEND_QUEUE *q;
    
    q = &uartSendQueue;
    q->maxSize = uartSendQueueSize;
    q->queue = uartSendQueueBuff;
    q->front = q->rear = 0;
}

void AddUarttmpData2Queue(COMM_NODE_T x)
{
    COMM_SEND_QUEUE *q = &uartSendQueue;
    //队列满
    if((q->rear + 1) % q->maxSize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxSize; // 求出队尾的下一个位置
    q->queue[q->rear] = x; // 把x的值赋给新的队尾
}

COMM_NODE_T* GetUartSendDataFromQueue(void)
{
    COMM_SEND_QUEUE *q = &uartSendQueue;
    //队列空
    if(q->front == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % (q->maxSize); // 使队首指针指向下一个位置
    return (COMM_NODE_T*)(&(q->queue[q->front])); // 返回队首元素
}
u8 IsUartSendQueueFree(void)
{
    COMM_SEND_QUEUE *q = &uartSendQueue;
    //队列空
    if(q->front == q->rear)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


#if 1 

void logic_uarttmp_init(void)
{
    u16 i = 0;
    for (i = 0; i < uarttmpQueueSize; i++) {
        uarttmpQueueBuff[i].value = INVALUE;
    }
}


void AddUartSendData2Queue(COMM_NODE_T x) 
{
    u16 i = 0;

    if ((logic_upload_stopStatus[x.inverter_no - 1] != 0) && (x.rw_flag == 1)) {
        return;
    }

    // 屯包只取决于最后的判断
    if (((user_paras_local.belt_para[x.inverter_no - 1].Func_Select_Switch >> 6) == 1) ||
        ((user_paras_local.belt_para[x.inverter_no - 1].Func_Select_Switch >> 7) == 1) ||
        ((user_paras_local.belt_para[x.inverter_no - 1].Func_Select_Switch >> 8) == 1)) {
        for (i = 0; i < uarttmpQueueSize; i++) {
            if (uarttmpQueueBuff[i].value == VALUE) {
                if ((x.rw_flag == 1) && (uarttmpQueueBuff[i].inverter_no == x.inverter_no)) {
                    uarttmpQueueBuff[i].value = INVALUE;
                }
            }
        }
    }
    
    for (i = 0; i < uarttmpQueueSize;i++) {
        if (uarttmpQueueBuff[i].value == INVALUE) {
            uarttmpQueueBuff[i] = x;
            uarttmpQueueBuff[i].value = VALUE;
            return;
        }
    }
}


void logic_cycle_decrease(void)
{
    u16 i = 0;
    for (i = 0; i < uarttmpQueueSize; i++) {
        if (uarttmpQueueBuff[i].value == VALUE) {
            if (uarttmpQueueBuff[i].comm_interval > 0) {
                uarttmpQueueBuff[i].comm_interval--;
            }
            if (uarttmpQueueBuff[i].comm_interval == 0) {
                AddUarttmpData2Queue(uarttmpQueueBuff[i]);
                uarttmpQueueBuff[i].value = INVALUE;
            }
        }
    }
}


#endif



//
void InitModuleStatusQueue(void)
{
    MODULE_STATUS_QUEUE *q;
    
    q = &moduleStatusQueue;
    q->maxSize = moduleStatusQueueSize;
    q->queue = moduleStatusBuff;
    q->front = q->rear = 0;
}
void AddModuleStatusData2Queue(MODULE_STATUS_T x)
{
    MODULE_STATUS_QUEUE *q = &moduleStatusQueue;
    //队列满
    if((q->rear + 1) % q->maxSize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxSize; // 求出队尾的下一个位置
    q->queue[q->rear] = x; // 把x的值赋给新的队尾
}
MODULE_STATUS_T* GetModuleStatusDataFromQueue(void)
{
    MODULE_STATUS_QUEUE *q = &moduleStatusQueue;
    //队列空
    if(q->front == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % q->maxSize; // 使队首指针指向下一个位置
    return (MODULE_STATUS_T*)(&(q->queue[q->front])); // 返回队首元素
}
u8 IsModuleStatusQueueFree(void)
{
    MODULE_STATUS_QUEUE *q = &moduleStatusQueue;
    //队列空
    if(q->front == q->rear)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* scan every 1ms */
void InputScanProc()
{
    u8 buf[10] = { 0 };

    u8 l_bit_reset_in;
    u8 l_bit_photo_1_in;
    u8 l_bit_photo_2_in;
    u8 l_bit_photo_3_in;
    u8 l_bit_photo_4_in;
    u8 l_bit_photo_5_in;
    u8 l_bit_start_in;
    u8 l_bit_stop_in;
    u8 l_bit_stream_in;
    u8 l_bit_emergency_in;

    u8 l_bit_emergencyreset_in;
    u8 l_bit_upstock_in;
    u8 l_bit_downstock_in;
    u8 l_bit_combiner_in;
    u8 l_bit_switch_in; 
    
    l_bit_reset_in = B_RESET_IN_STATE;
    l_bit_photo_1_in = B_PHOTO_1_IN_STATE;
    l_bit_photo_2_in = B_PHOTO_2_IN_STATE;
    l_bit_photo_3_in = B_PHOTO_3_IN_STATE;
    l_bit_photo_4_in = B_PHOTO_4_IN_STATE;
    l_bit_photo_5_in = B_PHOTO_5_IN_STATE;
    l_bit_start_in = B_START_IN_STATE;
    l_bit_stop_in = B_STOP_IN_STATE;
    l_bit_stream_in = B_STREAM_IN_STATE;
    l_bit_emergency_in = B_EMERGENCY_IN_STATE;

    l_bit_emergencyreset_in = B_EMERGENCYRESET_IN_STATE;
    l_bit_upstock_in = B_UP_STOCK_IN_STATE;
    l_bit_downstock_in = B_DOWN_STOCK_IN_STATE;
    l_bit_combiner_in = B_COMBINER_IN_STATE;
    l_bit_switch_in = B_SWITCH_IN_STATE;




    
    //处理复位输入信号
    if(  bResetInfo.input_info.input_state != l_bit_reset_in
      && bResetInfo.input_info.input_confirm_times == 0)
    {
        bResetInfo.input_info.input_middle_state = l_bit_reset_in;
    }
    if(  bResetInfo.input_info.input_middle_state == l_bit_reset_in
      && bResetInfo.input_info.input_middle_state != bResetInfo.input_info.input_state)
    {
        bResetInfo.input_info.input_confirm_times++;
        if(bResetInfo.input_info.input_confirm_times > 50)//按钮消抖时间50ms
        {
            bResetInfo.input_info.input_state = bResetInfo.input_info.input_middle_state;
            bResetInfo.input_info.input_confirm_times = 0;
            //取上升沿
            if (bResetInfo.input_info.input_state == 1) {
                Reset_Ctrl_Handle();
                reset_start_time_cnt = 500;
            }
        }
    }
    else
    {
        bResetInfo.input_info.input_middle_state = bResetInfo.input_info.input_state;
        bResetInfo.input_info.input_confirm_times = 0;
    }

    //处理光电1输入信号
    bPhoto1Info.trig_cnt++;
    if(  bPhoto1Info.input_info.input_state != l_bit_photo_1_in
      && bPhoto1Info.input_info.input_confirm_times == 0)
    {
        bPhoto1Info.input_info.input_middle_state = l_bit_photo_1_in;
    }
    if(  bPhoto1Info.input_info.input_middle_state == l_bit_photo_1_in
      && bPhoto1Info.input_info.input_middle_state != bPhoto1Info.input_info.input_state)
    {
        bPhoto1Info.input_info.input_confirm_times++;
        if(bPhoto1Info.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bPhoto1Info.input_info.input_state = bPhoto1Info.input_info.input_middle_state;
            bPhoto1Info.input_info.input_confirm_times = 0;
            //记录光电1 有变化
            inverter_status_buffer[0].input_status |= (0x1 << 7);
            bPhoto1Info.blocktrig_flag = 1;
            // 上升沿时 判断处理积放
            if (bPhoto1Info.input_info.input_state == 1) {
                Linkage_Stop_Photo_Ctrl_Handle(0);
            }
        }
    }
    else
    {
        bPhoto1Info.input_info.input_middle_state = bPhoto1Info.input_info.input_state;
        bPhoto1Info.input_info.input_confirm_times = 0;
    }
    if (bPhoto1Info.trig_cnt > 500) {
        bPhoto1Info.trig_cnt = 0;
        if (bPhoto1Info.input_info.input_state == 1) {
            Linkage_Stop_Photo_Ctrl_Handle(0);
        }
    }
    //处理光电2输入信号
    bPhoto2Info.trig_cnt++;
    if(  bPhoto2Info.input_info.input_state != l_bit_photo_2_in
      && bPhoto2Info.input_info.input_confirm_times == 0)
    {
        bPhoto2Info.input_info.input_middle_state = l_bit_photo_2_in;
    }
    if(  bPhoto2Info.input_info.input_middle_state == l_bit_photo_2_in
      && bPhoto2Info.input_info.input_middle_state != bPhoto2Info.input_info.input_state)
    {
        bPhoto2Info.input_info.input_confirm_times++;
        if(bPhoto2Info.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bPhoto2Info.input_info.input_state = bPhoto2Info.input_info.input_middle_state;
            bPhoto2Info.input_info.input_confirm_times = 0;
            bPhoto2Info.blocktrig_flag = 1;
            if (bPhoto2Info.input_info.input_state == 1) {
                Linkage_Stop_Photo_Ctrl_Handle(1);
            }
        }
    }
    else
    {
        bPhoto2Info.input_info.input_middle_state = bPhoto2Info.input_info.input_state;
        bPhoto2Info.input_info.input_confirm_times = 0;
    }
    if (bPhoto2Info.trig_cnt > 500) {
        bPhoto2Info.trig_cnt = 0;
        if (bPhoto2Info.input_info.input_state == 1) {
            Linkage_Stop_Photo_Ctrl_Handle(1);
        }
    }

    //处理光电3输入信号
    bPhoto3Info.trig_cnt++;
    if(  bPhoto3Info.input_info.input_state != l_bit_photo_3_in
      && bPhoto3Info.input_info.input_confirm_times == 0)
    {
        bPhoto3Info.input_info.input_middle_state = l_bit_photo_3_in;
    }
    if(  bPhoto3Info.input_info.input_middle_state == l_bit_photo_3_in
      && bPhoto3Info.input_info.input_middle_state != bPhoto3Info.input_info.input_state)
    {
        bPhoto3Info.input_info.input_confirm_times++;
        if(bPhoto3Info.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bPhoto3Info.input_info.input_state = bPhoto3Info.input_info.input_middle_state;
            bPhoto3Info.input_info.input_confirm_times = 0;
            bPhoto3Info.blocktrig_flag = 1;
            if (bPhoto3Info.input_info.input_state == 1) {
                Linkage_Stop_Photo_Ctrl_Handle(2);
            }
        }
    }
    else
    {
        bPhoto3Info.input_info.input_middle_state = bPhoto3Info.input_info.input_state;
        bPhoto3Info.input_info.input_confirm_times = 0;
    }
    if (bPhoto3Info.trig_cnt > 500) {
        bPhoto3Info.trig_cnt = 0;
        if (bPhoto3Info.input_info.input_state == 1) {
            Linkage_Stop_Photo_Ctrl_Handle(2);
        }
    }
    //处理光电4输入信号
    bPhoto4Info.trig_cnt++;
    if(  bPhoto4Info.input_info.input_state != l_bit_photo_4_in
      && bPhoto4Info.input_info.input_confirm_times == 0)
    {
        bPhoto4Info.input_info.input_middle_state = l_bit_photo_4_in;
    }
    if(  bPhoto4Info.input_info.input_middle_state == l_bit_photo_4_in
      && bPhoto4Info.input_info.input_middle_state != bPhoto4Info.input_info.input_state)
    {
        bPhoto4Info.input_info.input_confirm_times++;
        if(bPhoto4Info.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bPhoto4Info.input_info.input_state = bPhoto4Info.input_info.input_middle_state;
            bPhoto4Info.input_info.input_confirm_times = 0;
            bPhoto4Info.blocktrig_flag = 1;
            if (bPhoto4Info.input_info.input_state == 1) {
                Linkage_Stop_Photo_Ctrl_Handle(3);
            }
        }
    }
    else
    {
        bPhoto4Info.input_info.input_middle_state = bPhoto4Info.input_info.input_state;
        bPhoto4Info.input_info.input_confirm_times = 0;
    }

    if (bPhoto4Info.trig_cnt > 500) {
        bPhoto4Info.trig_cnt = 0;
        if (bPhoto4Info.input_info.input_state == 1) {
            Linkage_Stop_Photo_Ctrl_Handle(3);
        }
    }
    //处理光电5输入信号
    bPhoto5Info.trig_cnt++;
    if(  bPhoto5Info.input_info.input_state != l_bit_photo_5_in
      && bPhoto5Info.input_info.input_confirm_times == 0)
    {
        bPhoto5Info.input_info.input_middle_state = l_bit_photo_5_in;
    }
    if(  bPhoto5Info.input_info.input_middle_state == l_bit_photo_5_in
      && bPhoto5Info.input_info.input_middle_state != bPhoto5Info.input_info.input_state)
    {
        bPhoto5Info.input_info.input_confirm_times++;
        if(bPhoto5Info.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bPhoto5Info.input_info.input_state = bPhoto5Info.input_info.input_middle_state;
            bPhoto5Info.input_info.input_confirm_times = 0;
            bPhoto5Info.blocktrig_flag = 1;
            if (bPhoto5Info.input_info.input_state == 1) {
                Linkage_Stop_Photo_Ctrl_Handle(4);
            }
        }
    }
    else
    {
        bPhoto5Info.input_info.input_middle_state = bPhoto5Info.input_info.input_state;
        bPhoto5Info.input_info.input_confirm_times = 0;
    }
    if (bPhoto5Info.trig_cnt > 500) {
        bPhoto5Info.trig_cnt = 0;
        if (bPhoto5Info.input_info.input_state == 1) {
            Linkage_Stop_Photo_Ctrl_Handle(4);
        }
    }

    //处理启动输入信号
    if(  bStartInfo.input_info.input_state != l_bit_start_in
      && bStartInfo.input_info.input_confirm_times == 0)
    {
        bStartInfo.input_info.input_middle_state = l_bit_start_in;
    }
    if(  bStartInfo.input_info.input_middle_state == l_bit_start_in
      && bStartInfo.input_info.input_middle_state != bStartInfo.input_info.input_state)
    {
        bStartInfo.input_info.input_confirm_times++;
        if(bStartInfo.input_info.input_confirm_times > 100)//消抖时间100ms
        {
            bStartInfo.input_info.input_state = bStartInfo.input_info.input_middle_state;
            bStartInfo.input_info.input_confirm_times = 0;
            if(bStartInfo.input_info.input_state == 1)
            {
                if(isHost)//主机
                {
                    Reset_Ctrl_Handle();                  //有故障先复位故障
                    reset_start_time_cnt = 0;
                    Speed_Ctrl_Process(2);
                    //can_bus_send_start_cmd(2);
                    buf[0] = 2;
                    vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_START_CMD, 1);
                }
                else {  //非主机 只启动自己
                    Reset_Ctrl_Handle();                  //有故障先复位故障
                    reset_start_time_cnt = 0;
                    Speed_Ctrl_Process(2);
                }
            }
            //如果是一根线 控制启停
            if (bStartInfo.input_info.input_state == 0)
            {
//                if (isHost)//主机
//                {
//                    Reset_Ctrl_Handle();                  //有故障先复位故障
//                    reset_start_time_cnt = 0;
//                    Speed_Ctrl_Process(0);
//                    //can_bus_send_start_cmd(2);
//                    buf[0] = 0;
//                    vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_START_CMD, 1);
//                }
//                else {  //非主机 只启动自己
//                    Reset_Ctrl_Handle();                  //有故障先复位故障
//                    reset_start_time_cnt = 0;
//                    Speed_Ctrl_Process(0);
//                }
            }

        }
    }
    else
    {
        bStartInfo.input_info.input_middle_state = bStartInfo.input_info.input_state;
        bStartInfo.input_info.input_confirm_times = 0;
    }

    //处理停止输入信号
    if(  bStopInfo.input_info.input_state != l_bit_stop_in
      && bStopInfo.input_info.input_confirm_times == 0)
    {
        bStopInfo.input_info.input_middle_state = l_bit_stop_in;
    }
    if(  bStopInfo.input_info.input_middle_state == l_bit_stop_in
      && bStopInfo.input_info.input_middle_state != bStopInfo.input_info.input_state)
    {
        bStopInfo.input_info.input_confirm_times++;
        if(bStopInfo.input_info.input_confirm_times > 100)//消抖时间100ms
        {
            bStopInfo.input_info.input_state = bStopInfo.input_info.input_middle_state;
            bStopInfo.input_info.input_confirm_times = 0;
            if(bStopInfo.input_info.input_state == 1)
            {
                if(isHost)//主机
                {
                    Speed_Ctrl_Process(0);
                    //can_bus_send_start_cmd(0);
                    buf[0] = 0;
                    vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_START_CMD, 1);
                }
                else {  //非主机 只停自己
                    Speed_Ctrl_Process(0);
                }
            }
        }
    }
    else
    {
        bStopInfo.input_info.input_middle_state = bStopInfo.input_info.input_state;
        bStopInfo.input_info.input_confirm_times = 0;
    }

    //处理下游准入信号
    bStreamInfo.trig_cnt++;
    if (bStreamInfo.input_info.input_state != l_bit_stream_in
        && bStreamInfo.input_info.input_confirm_times == 0)
    {
        bStreamInfo.input_info.input_middle_state = l_bit_stream_in;
    }
    if (bStreamInfo.input_info.input_middle_state == l_bit_stream_in
        && bStreamInfo.input_info.input_middle_state != bStreamInfo.input_info.input_state)
    {
        bStreamInfo.input_info.input_confirm_times++;
        if ((bStreamInfo.input_info.input_middle_state == 1) && (bStreamInfo.input_info.input_confirm_times > 20))//消抖时间20ms
        {
            bStreamInfo.input_info.input_state = bStreamInfo.input_info.input_middle_state;
            bStreamInfo.input_info.input_confirm_times = 0;
            bStreamInfo.blocktrig_flag = 1;

            if (bStreamInfo.input_info.input_state == 1) {
                if (user_paras_local.Belt_Number >= 1) {
                    Linkage_stream_extra_signal(user_paras_local.Belt_Number - 1, 1);
                }
            }
            Linkage_stream_extrasignal_process();
        }

        if ((bStreamInfo.input_info.input_middle_state == 0) && (bStreamInfo.input_info.input_confirm_times > 200))//消抖时间200ms
        {
            bStreamInfo.input_info.input_state = bStreamInfo.input_info.input_middle_state;
            bStreamInfo.input_info.input_confirm_times = 0;
        }

        if (bStreamInfo.input_info.input_state == 0) {
//            Linkage_stream_extra_signal(user_paras_local.Belt_Number - 1, 0);
        }

    }
    else
    {
        bStreamInfo.input_info.input_middle_state = bStreamInfo.input_info.input_state;
        bStreamInfo.input_info.input_confirm_times = 0;
    }

    if (bStreamInfo.trig_cnt > 2000) {
        bStreamInfo.trig_cnt = 0;
        Linkage_stream_extrasignal_process();
        if (bStreamInfo.input_info.input_state == 1) {
            if (user_paras_local.Belt_Number >= 1) {
                Linkage_stream_extra_signal(user_paras_local.Belt_Number - 1, 1);
            }
        }
    }

    //处理急停输入信号
    bEmergencyInfo.trig_cnt++;
    if (bEmergencyInfo.input_info.input_state != l_bit_emergency_in
        && bEmergencyInfo.input_info.input_confirm_times == 0)
    {
        bEmergencyInfo.input_info.input_middle_state = l_bit_emergency_in;
    }
    if (bEmergencyInfo.input_info.input_middle_state == l_bit_emergency_in
        && bEmergencyInfo.input_info.input_middle_state != bEmergencyInfo.input_info.input_state)
    {
        bEmergencyInfo.input_info.input_confirm_times++;
        if (bEmergencyInfo.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bEmergencyInfo.input_info.input_state = bEmergencyInfo.input_info.input_middle_state;
            bEmergencyInfo.input_info.input_confirm_times = 0;
            if (bEmergencyInfo.input_info.input_state == 1) {
                Speed_Ctrl_Process(EMERGENCY_STOP);
                can_bus_send_func_emergency_cmd();
                g_emergency_stop = 1;
                buf[0] = 0;
                vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_EMERGENCY_STOP_STATUS, 0);

            }
//            Speed_Ctrl_Process(EMERGENCY_STOP);
        }
    }
    else
    {
        bEmergencyInfo.input_info.input_middle_state = bEmergencyInfo.input_info.input_state;
        bEmergencyInfo.input_info.input_confirm_times = 0;
    }
    if (bEmergencyInfo.trig_cnt > 2000) {
        bEmergencyInfo.trig_cnt = 0;
        if (bEmergencyInfo.input_info.input_state == 1) {
            Speed_Ctrl_Process(EMERGENCY_STOP);
            //can_bus_send_func_emergency_cmd();
            buf[0] = 0;
            vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_EMERGENCY_STOP_STATUS, 0);
        }
    }



    //处理急停复位信号
    bEmergencyResetInfo.trig_cnt++;
    if (bEmergencyResetInfo.input_info.input_state != l_bit_emergencyreset_in
        && bEmergencyResetInfo.input_info.input_confirm_times == 0)
    {
        bEmergencyResetInfo.input_info.input_middle_state = l_bit_emergencyreset_in;
    }
    if (bEmergencyResetInfo.input_info.input_middle_state == l_bit_emergencyreset_in
        && bEmergencyResetInfo.input_info.input_middle_state != bEmergencyResetInfo.input_info.input_state)
    {
        bEmergencyResetInfo.input_info.input_confirm_times++;
        if (bEmergencyResetInfo.input_info.input_confirm_times > 50)//消抖时间50ms
        {
            bEmergencyResetInfo.input_info.input_state = bEmergencyResetInfo.input_info.input_middle_state;
            bEmergencyResetInfo.input_info.input_confirm_times = 0;
            if (bEmergencyResetInfo.input_info.input_state == 1) {
                g_emergency_stop = 0;
                buf[0] = 0;
                vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_RESET_CMD, 0);

            }
        }
    }
    else
    {
        bEmergencyResetInfo.input_info.input_middle_state = bEmergencyResetInfo.input_info.input_state;
        bEmergencyResetInfo.input_info.input_confirm_times = 0;
    }
    if (bEmergencyResetInfo.trig_cnt > 2000) {
        bEmergencyResetInfo.trig_cnt = 0;
        if (bEmergencyResetInfo.input_info.input_state == 1) {
            buf[0] = 0;
            vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_RESET_CMD, 0);
        }
    }



    //处理上游屯包光电信号
    bUpStockInfo.trig_cnt++;
    if (bUpStockInfo.input_info.input_state != l_bit_upstock_in
        && bUpStockInfo.input_info.input_confirm_times == 0)
    {
        bUpStockInfo.input_info.input_middle_state = l_bit_upstock_in;
    }
    if (bUpStockInfo.input_info.input_middle_state == l_bit_upstock_in
        && bUpStockInfo.input_info.input_middle_state != bUpStockInfo.input_info.input_state)
    {
        bUpStockInfo.input_info.input_confirm_times++;
        if (bUpStockInfo.input_info.input_confirm_times > 20)//消抖时间50ms
        {
            bUpStockInfo.input_info.input_state = bUpStockInfo.input_info.input_middle_state;
            bUpStockInfo.input_info.input_confirm_times = 0;
            if (bUpStockInfo.input_info.input_state == 1) {
                bUpStockInfo.button_hold_time = 0;
            }
        }
    }
    else
    {
        bUpStockInfo.input_info.input_middle_state = bUpStockInfo.input_info.input_state;
        bUpStockInfo.input_info.input_confirm_times = 0;
    }
    if (bUpStockInfo.trig_cnt > 2000) {
        bUpStockInfo.trig_cnt = 0;
        if (bUpStockInfo.input_info.input_state == 1) {
        }
    }

    //高电平保持时间
    if (bUpStockInfo.input_info.input_state == 1) {
        bUpStockInfo.button_hold_time++;
        if (bUpStockInfo.button_hold_time >= 3000) {
            bUpStockInfo.button_hold_time = 3000;
        }
    }


    //处理下游屯包光电信号
    bDownStockInfo.trig_cnt++;
    if (bDownStockInfo.input_info.input_state != l_bit_downstock_in
        && bDownStockInfo.input_info.input_confirm_times == 0)
    {
        bDownStockInfo.input_info.input_middle_state = l_bit_downstock_in;
    }
    if (bDownStockInfo.input_info.input_middle_state == l_bit_downstock_in
        && bDownStockInfo.input_info.input_middle_state != bDownStockInfo.input_info.input_state)
    {
        bDownStockInfo.input_info.input_confirm_times++;
        if (bDownStockInfo.input_info.input_confirm_times > 20)//消抖时间50ms
        {
            bDownStockInfo.input_info.input_state = bDownStockInfo.input_info.input_middle_state;
            bDownStockInfo.input_info.input_confirm_times = 0;
            if (bDownStockInfo.input_info.input_state == 1) {

            }
        }
    }
    else
    {
        bDownStockInfo.input_info.input_middle_state = bDownStockInfo.input_info.input_state;
        bDownStockInfo.input_info.input_confirm_times = 0;
    }
    if (bDownStockInfo.trig_cnt > 2000) {
        bDownStockInfo.trig_cnt = 0;
        if (bDownStockInfo.input_info.input_state == 1) {
        }
    }


    //处理合流机控制信号(下游)
    bCombinerInInfo.trig_cnt++;
    bCombinerInInfo.combine_cnt++;
    if (bCombinerInInfo.input_info.input_state != l_bit_combiner_in
        && bCombinerInInfo.input_info.input_confirm_times == 0)
    {
        bCombinerInInfo.input_info.input_middle_state = l_bit_combiner_in;
    }
    if (bCombinerInInfo.input_info.input_middle_state == l_bit_combiner_in
        && bCombinerInInfo.input_info.input_middle_state != bCombinerInInfo.input_info.input_state)
    {
        bCombinerInInfo.input_info.input_confirm_times++;
        if (bCombinerInInfo.input_info.input_confirm_times > 20)//消抖时间50ms
        {
            bCombinerInInfo.input_info.input_state = bCombinerInInfo.input_info.input_middle_state;
            bCombinerInInfo.input_info.input_confirm_times = 0;
            bCombinerInInfo.combine_cnt = 0;
            if (bCombinerInInfo.input_info.input_state == 0) {
                bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_DOWN;
                bCombinerInInfo.button_hold_time = 0;
            }
            if (bCombinerInInfo.input_info.input_state == 1) {
                bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_UP;
            }
        }
    }
    else
    {
        bCombinerInInfo.input_info.input_middle_state = bCombinerInInfo.input_info.input_state;
        bCombinerInInfo.input_info.input_confirm_times = 0;
    }
    if (bCombinerInInfo.trig_cnt > 2000) {
        bCombinerInInfo.trig_cnt = 0;
        if (bCombinerInInfo.input_info.input_state == 1) {
        }
    }
    //低电平保持时间
    if (bCombinerInInfo.input_info.input_state == 0) {
        bCombinerInInfo.button_hold_time++;
        if (bCombinerInInfo.button_hold_time >= 80000) {
            bCombinerInInfo.button_hold_time = 80000;
        }
    }
    

    //处理合流机屯包插包切换信号
    bSwitchInInfo.trig_cnt++;
    bSwitchInInfo.combine_cnt++;
    if ((bSwitchInInfo.input_info.input_state != l_bit_switch_in)
        && (bSwitchInInfo.input_info.input_confirm_times == 0))
    {
        bSwitchInInfo.input_info.input_middle_state = l_bit_switch_in;
    }
    if ((bSwitchInInfo.input_info.input_middle_state == l_bit_switch_in)
        && (bSwitchInInfo.input_info.input_middle_state != bSwitchInInfo.input_info.input_state))
    {
        bSwitchInInfo.input_info.input_confirm_times++;
        if (bSwitchInInfo.input_info.input_confirm_times > 20)//消抖时间50ms
        {
            bSwitchInInfo.input_info.input_state = bSwitchInInfo.input_info.input_middle_state;
            bSwitchInInfo.input_info.input_confirm_times = 0;
            bSwitchInInfo.combine_cnt = 0;
            if (bSwitchInInfo.input_info.input_state == 0) {
                bSwitchInInfo.input_info.input_trig_mode = INPUT_TRIG_DOWN;
            }
            if (bSwitchInInfo.input_info.input_state == 1) {
                bSwitchInInfo.input_info.input_trig_mode = INPUT_TRIG_UP;
            }
        }
    }
    else
    {
        bSwitchInInfo.input_info.input_middle_state = bSwitchInInfo.input_info.input_state;
        bSwitchInInfo.input_info.input_confirm_times = 0;
    }
    if (bSwitchInInfo.trig_cnt > 2000) {
        bSwitchInInfo.trig_cnt = 0;
        if (bSwitchInInfo.input_info.input_state == 1) {
        }
    }


}

//启动,停止, 急停变速时改变所有皮带的速度  
void Speed_Ctrl_Process(u8 speed_gear)
{
    COMM_NODE_T  comm_node_new;
    u8 i;
    u8 stockFlag = 0;
    
    //开始屯包时
    for (i = 0; i < user_paras_local.Belt_Number; i++)
    {
        if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) == 1) ||
            ((user_paras_local.belt_para[i].Func_Select_Switch >> 7) == 1) ||
            ((user_paras_local.belt_para[i].Func_Select_Switch >> 8) == 1)) {
            bCombinerInInfo.input_info.input_state = 0xFF;
            stockFlag = 1;
        }
    }
    
//  if(speed_gear > 6) return;

    if (speed_gear == EMERGENCY_STOP) {
        InitUartSendQueue();
        logic_uarttmp_init();
        for (i = 0; i < user_paras_local.Belt_Number; i++)
        {
            comm_node_new.rw_flag = 1;
            comm_node_new.inverter_no = i + 1;
            comm_node_new.speed_gear = 0;
            comm_node_new.comm_interval = 1;
            comm_node_new.comm_retry = 3;
            AddUartSendData2Queue(comm_node_new);
        }
        g_speed_gear_status = 0;
        if (stockFlag == 0) {
            B_RUN_STA_OUT_(Bit_RESET);
        }
        return;
    }
    
    if(g_speed_gear_status == 0 && speed_gear > 0)//启动
    {
        InitUartSendQueue();
        logic_uarttmp_init();
        for(i=0; i<user_paras_local.Belt_Number; i++)
        {
            if (((user_paras_local.belt_para[user_paras_local.Belt_Number - i - 1].Func_Select_Switch >> 6) == 1) ||
                ((user_paras_local.belt_para[user_paras_local.Belt_Number - i - 1].Func_Select_Switch >> 7) == 1) ||
                ((user_paras_local.belt_para[user_paras_local.Belt_Number - i - 1].Func_Select_Switch >> 8) == 1)) {
                continue;
            }
            
            if (extralAllow == 0) {
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = user_paras_local.Belt_Number - i;
                comm_node_new.speed_gear = speed_gear;
                comm_node_new.comm_interval = user_paras_local.belt_para[user_paras_local.Belt_Number - i - 1].Start_Delay_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }

        }
        g_speed_gear_status = speed_gear;
        if (stockFlag == 0) {
            B_RUN_STA_OUT_(Bit_SET);
        }
    }
    else if( speed_gear == 0)//停止
    {
        InitUartSendQueue();
        logic_uarttmp_init();
        for(i=0; i<user_paras_local.Belt_Number; i++)
        {
            //独立的停止时间
            comm_node_new.rw_flag = 1;
            comm_node_new.inverter_no = i+1;
            comm_node_new.speed_gear = speed_gear;
            comm_node_new.comm_interval = user_paras_local.belt_para[i].Stop_Delay_Time + user_paras_local.belt_para[i].Stop_Clear_Time * 1000;
            comm_node_new.comm_retry = 3;
            AddUartSendData2Queue(comm_node_new);
        }
        g_speed_gear_status = speed_gear;
        if (stockFlag == 0) {
            B_RUN_STA_OUT_(Bit_RESET);
        }
    }
    else if((g_speed_gear_status != 0) && (speed_gear != 0))//变速
    {

        for(i=0; i<user_paras_local.Belt_Number; i++)
        {
            if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) == 1) ||
                ((user_paras_local.belt_para[i].Func_Select_Switch >> 7) == 1) ||
                ((user_paras_local.belt_para[i].Func_Select_Switch >> 8) == 1)) {
                continue;
            }
            if (extralAllow == 0) {
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = i + 1;
                comm_node_new.speed_gear = speed_gear;
                comm_node_new.comm_interval = 10;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
            if ((extralAllow == 1) && (((inverter_status_buffer[i].fault_code >> 4) &0x1) == 1)){
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = i + 1;
                comm_node_new.speed_gear = speed_gear;
                comm_node_new.comm_interval = 10;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }

        }
        g_speed_gear_status = speed_gear;
        if (stockFlag == 0) {
            B_RUN_STA_OUT_(Bit_SET);
        }
    }
}
// 获取光电状态
u8 get_photo_input_status(u8 belt_index)
{
    if(belt_index == 0)
    {
        if (bPhoto1Info.blocktrig_flag == 1) {
            bPhoto1Info.blocktrig_flag = 0;
            return 0;
        }
        return B_PHOTO_1_IN_STATE;
    }
    else if(belt_index == 1)
    {
        if (bPhoto2Info.blocktrig_flag == 1) {
            bPhoto2Info.blocktrig_flag = 0;
            return 0;
        }
        return B_PHOTO_2_IN_STATE;
    }
    else if(belt_index == 2)
    {
        if (bPhoto3Info.blocktrig_flag == 1) {
            bPhoto3Info.blocktrig_flag = 0;
            return 0;
        }
        return B_PHOTO_3_IN_STATE;
    }
    else if(belt_index == 3)
    {
        if (bPhoto4Info.blocktrig_flag == 1) {
            bPhoto4Info.blocktrig_flag = 0;
            return 0;
        }
        return B_PHOTO_4_IN_STATE;
    }
    else if(belt_index == 4)
    {
        if (bPhoto5Info.blocktrig_flag == 1) {
            bPhoto5Info.blocktrig_flag = 0;
            return 0;
        }
        return B_PHOTO_5_IN_STATE;
    }
    else
    {
        return 0;
    }
}
// 获取下游光电触发状态
u8 logic_getDownStreamPhotoTrigStatus(u8 i) 
{
    u8 flag = 0;
    u16 j = 0;

    //如果是最后一段皮带
    if (i == (user_paras_local.Belt_Number - 1)) {
        //配置了有外部输入信号的情况
        for (j = 0; j < user_paras_local.Belt_Number; j++) {
            if ((user_paras_local.belt_para[j].Func_Select_Switch >> 4) & 0x1)
            {
                if ((bStreamInfo.input_info.input_state == 1) && (bStreamInfo.blocktrig_flag == 0)) {
                    flag = 0;
                }
                if (bStreamInfo.input_info.input_state == 0) {
                    flag = 1;
                }
                if (bStreamInfo.blocktrig_flag == 1) {
                    bStreamInfo.blocktrig_flag = 0;
                    flag = 1;
                }
                bStreamInfo.blocktrig_flag = 0;
                return flag;
            }
        }
        // 没有外部输入信号的情况  有下游 则取下游信号 没有下游  则只判断本皮带对应的光电
        flag = g_link_down_phototrig_status ;
        g_link_down_phototrig_status = 0;
    }
    else {     // 中间段皮带
        switch (i)
        {
        case 0:
            flag = bPhoto2Info.blocktrig_flag;
            bPhoto2Info.blocktrig_flag = 0;
            break;
        case 1:
            flag = bPhoto3Info.blocktrig_flag;
            bPhoto3Info.blocktrig_flag = 0;
            break;
        case 2:
            flag = bPhoto4Info.blocktrig_flag;
            bPhoto4Info.blocktrig_flag = 0;
            break;
        case 3:
            flag = bPhoto5Info.blocktrig_flag;
            bPhoto5Info.blocktrig_flag = 0;
            break;
        default:
            flag = 1;
            break;
        }
    }

    return flag;
}


u8 get_inverter_fault_status(INVERTER_STATUS_T inverter_status)
{
    //if(inverter_status.fault_code & 0x1)//485通讯故障
    //    return 1;
    if((inverter_status.fault_code>>1) & 0x1)//堵包故障
        return 1;
    if(((inverter_status.fault_code>>8)&0xFF) != 0)//变频器故障码
        return 1;
    return 0;
}

//联动停止_光电触发控制
void Linkage_Stop_Photo_Ctrl_Handle(u8 photo_index)
{
    COMM_NODE_T  comm_node_new;
    u8  link_down_status = 1;
    u16 i = 0;
    u16 j = 0;
    
    if(user_paras_local.Belt_Number == 0) return;

    //for (i = 0; i < user_paras_local.Belt_Number; i++) {
    //    if ((user_paras_local.belt_para[i].Func_Select_Switch >> 4) && (bStreamInfo.input_info.input_state == 1))
    //    {
    //        return;
    //    }
    //}
    
    if(((user_paras_local.belt_para[photo_index].Func_Select_Switch>>3)&0x1) == 0)//未启用积放功能
    {
        return;
    }
    //确认下游的状态
    //若该段皮带为最后一段皮带
    if(photo_index+1 == user_paras_local.Belt_Number)
    {
        if (user_paras_local.Down_Stream_No != 0)//配置了下游站号
        {
            //            link_down_status = 1;
            link_down_status = g_link_down_stream_status;
        }

        //如果加了外部准入信号
        for (i = 0; i < user_paras_local.Belt_Number; i++) {
            if ((user_paras_local.belt_para[i].Func_Select_Switch >> 4) & 0x1)
            {
                if (bStreamInfo.input_info.input_state == 0)
                {
                    link_down_status = 0;
                }
                else if (bStreamInfo.input_info.input_state == 1) {
                    link_down_status = 1;
                }
                break;
            }
        }
    }
    else
    {
        link_down_status = (inverter_status_buffer[photo_index+1].fault_code>>4)&0x1;
    }


    if((user_paras_local.belt_para[photo_index].Func_Select_Switch>>1)&0x1)//联动功能开启
    {
        if((inverter_status_buffer[photo_index].fault_code>>4)&0x1)//本地皮带运行状态
        {
            if(link_down_status == 0)//下游不允许进入
            {

                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = photo_index + 1;
                comm_node_new.speed_gear = 0;
                comm_node_new.comm_interval = user_paras_local.belt_para[photo_index].Link_Stop_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);

                if (photo_index == 0) {
                    if ((user_paras_local.belt_para[photo_index].Func_Select_Switch >> 5) & 0x1) { //上游同步停止
                        can_bus_send_func_upStreamStop_cmd();
                        can_bus_send_func_upStreamStop_cmd();
                        return;
                    }
                }

                //上游同步停止
                for (j = photo_index; j >= 1; j--) {
                    if ((user_paras_local.belt_para[j].Func_Select_Switch >> 5) & 0x1) //上游同步停止
                    {
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = j;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = user_paras_local.belt_para[j - 1].Link_Stop_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);

                        if (j == 1) {
                            if ((user_paras_local.belt_para[j-1].Func_Select_Switch >> 5) & 0x1) { //上游同步停止
                                can_bus_send_func_upStreamStop_cmd();
                                can_bus_send_func_upStreamStop_cmd();
                                return;
                            }
                        }
                    }
                    else {
                        return;;
                    }
                }
            }
        }
    }
}

// 一般只用到了允许启动
// 外部允许供包信号
void Linkage_stream_extra_signal(u8 inverter_index, u8 start_flag)
{
    COMM_NODE_T  comm_node_new;
    u16 i = 0;

    for (i = 0; i < user_paras_local.Belt_Number; i++) {
        if ((user_paras_local.belt_para[i].Func_Select_Switch >> 4) & 0x1)
        {
            if ((user_paras_local.belt_para[inverter_index].Func_Select_Switch >> 1) & 0x1)//联动功能开启
            {
                if (start_flag == 1)//联动启动
                {
                    if ((((inverter_status_buffer[inverter_index].fault_code >> 4) & 0x1) == 0)
                        && (get_inverter_fault_status(inverter_status_buffer[inverter_index]) != 1))//本地皮带停止状态且没有报警
                    {
                        //启动
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = inverter_index + 1;
                        comm_node_new.speed_gear = g_speed_gear_status;
                        comm_node_new.comm_interval = user_paras_local.belt_para[inverter_index].Link_Start_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);
                    }
                }
                else//联动停止
                {

                    if ((inverter_status_buffer[inverter_index].fault_code >> 4) & 0x1)//本地皮带运行状态
                    {
                        //停止
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = inverter_index + 1;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = user_paras_local.belt_para[inverter_index].Link_Stop_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);
                    }
                }
            }
            return;
        }
    }
}

//联动起停_上下游触发控制
void Linkage_Stream_Ctrl_Handle(u8 inverter_index,u8 start_flag)
{
    COMM_NODE_T  comm_node_new;
    
    if((user_paras_local.belt_para[inverter_index].Func_Select_Switch>>1)&0x1)//联动功能开启
    {
        if(start_flag == 1)//联动启动
        {
            if((((inverter_status_buffer[inverter_index].fault_code>>4)&0x1) == 0)
              && (get_inverter_fault_status(inverter_status_buffer[inverter_index]) != 1))//本地皮带停止状态且没有报警
            {
                //启动
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = inverter_index+1;
                comm_node_new.speed_gear = g_speed_gear_status;
                comm_node_new.comm_interval = user_paras_local.belt_para[inverter_index].Link_Start_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
        else//联动停止
        {
            // 如果设置了光电联动启停 则优先用光电联动启停
            if(((user_paras_local.belt_para[inverter_index].Func_Select_Switch>>3)&0x1) == 1)//启用积放功能就由光电触发停止
                return;
            if((inverter_status_buffer[inverter_index].fault_code>>4)&0x1)//本地皮带运行状态
            {
                //停止
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = inverter_index+1;
                comm_node_new.speed_gear = 0;
                comm_node_new.comm_interval = user_paras_local.belt_para[inverter_index].Link_Stop_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
    }
}
//堵包检测   依据该段皮带的下游皮带的光电信号综合判断 该段皮带的堵包状态  
void Block_Check_Ctrl_Handle(void)
{
    COMM_NODE_T  comm_node_new;
    u8  i;
    
    if(g_block_disable_flag == 1) return;
    
    for(i=0; i<user_paras_local.Belt_Number; i++)
    {
        if((user_paras_local.belt_para[i].Func_Select_Switch>>2)&0x1)//堵包检测开启
        {
            if((inverter_status_buffer[i].fault_code>>4)&0x1)//本地皮带运行状态
            {
                if((get_photo_input_status(i) == 1) && (logic_getDownStreamPhotoTrigStatus(i) == 0))//光电被遮挡
                {
                    block_check_time_cnt[i]++;
                    if(block_check_time_cnt[i] >= user_paras_local.belt_para[i].Block_Check_Time)
                    {
                        block_check_time_cnt[i] = 0;
                        //停止
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = i+1;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = 1;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);
                        L_ALARM_OUT_(Bit_SET);//报警指示灯输出
                        inverter_status_buffer[i].fault_code |= (0x1<<1);//堵包故障
                    }
                }
                else
                {
                    block_check_time_cnt[i] = 0;
                }
            }
            else
            {
                block_check_time_cnt[i] = 0;
            }
        }
    }
}

//add 240131 外部允许信号直接联动
void Linkage_stream_extrasignal_process(void)
{
    COMM_NODE_T  comm_node_new;
    u16 i = 0;
    u16 j = 0;
    u16 flag = 0;

    extralAllow = 0;

    for (i = 0; i < user_paras_local.Belt_Number; i++) {
        if ((user_paras_local.belt_para[i].Func_Select_Switch >> 4) & 0x1)
        {
            flag = 1;
        }
    }
    if (flag == 0) {
        return;
    }

    for (j = 0; j < user_paras_local.Belt_Number; j++) {
        if (((user_paras_local.belt_para[j].Func_Select_Switch >> 1) & 0x1 == 1) ||
            ((user_paras_local.belt_para[j].Func_Select_Switch >> 3) & 0x1 == 1) ||
            ((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1 == 1) ||
            ((user_paras_local.belt_para[j].Func_Select_Switch >> 7) & 0x1 == 1) ||
            ((user_paras_local.belt_para[j].Func_Select_Switch >> 8) & 0x1 == 1)) {
            return;
        }
    }

    extralAllow = 1;

    for (j = 0; j < user_paras_local.Belt_Number; j++) {
        if (bStreamInfo.input_info.input_state == 1) {
            if ((((inverter_status_buffer[j].fault_code >> 4) & 0x1) == 0)
                && (get_inverter_fault_status(inverter_status_buffer[j]) != 1))//本地皮带停止状态且没有报警
            {
                //启动
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = j + 1;
                comm_node_new.speed_gear = g_speed_gear_status;
                comm_node_new.comm_interval = user_paras_local.belt_para[j].Link_Start_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
        else {
            if ((inverter_status_buffer[j].fault_code >> 4) & 0x1)//本地皮带运行状态
            {
                //停止
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = j + 1;
                comm_node_new.speed_gear = 0;
                comm_node_new.comm_interval = user_paras_local.belt_para[j].Link_Stop_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
    }
}

//堵包复位,变频器故障复位
void Reset_Ctrl_Handle(void)
{
    COMM_NODE_T  comm_node_new;
    u8  i;
    
    for(i=0; i<user_paras_local.Belt_Number; i++)
    {
        if((inverter_status_buffer[i].fault_code>>1)&0x1)//堵包复位
        {
            inverter_status_buffer[i].fault_code &= ~(0x1<<1);
        }
        if(((inverter_status_buffer[i].fault_code>>8)&0xFF) != 0)//变频器复位
        {
            comm_node_new.rw_flag = 2;
            comm_node_new.inverter_no = i+1;
            comm_node_new.speed_gear = 0;
            comm_node_new.comm_interval = 1;
            comm_node_new.comm_retry = 3;
            AddUartSendData2Queue(comm_node_new);
        }
    }
    L_ALARM_OUT_(Bit_RESET);//报警指示灯熄灭
//    reset_start_time_cnt = 500;
//    g_emergency_stop = 0;
}
//复位后启动变频器
void Reset_Start_Inverter_Handle(void)
{
    COMM_NODE_T  comm_node_new;
    u16 i;
    u8 flag = 0;
    
    if(user_paras_local.Belt_Number == 0) return;
    if(g_speed_gear_status == 0) return;
    if(reset_start_flag != 1) return;
    
    reset_start_flag = 0;

    //开始屯包时
    for (i = 0; i < user_paras_local.Belt_Number; i++)
    {
        if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) == 1) ||
            ((user_paras_local.belt_para[i].Func_Select_Switch >> 7) == 1) ||
            ((user_paras_local.belt_para[i].Func_Select_Switch >> 8) == 1)) {
            bCombinerInInfo.input_info.input_state = 0xFF;
            flag = 1;
        }
    }

    if (flag == 1) {
        return;
    }
    else {
        //if (bStreamInfo.input_info.input_state == 0) {
            for (i = user_paras_local.Belt_Number; i > 0; i--)
            {
                if ((((inverter_status_buffer[i - 1].fault_code >> 4) & 0x1) == 0)
                     && (get_inverter_fault_status(inverter_status_buffer[i-1]) != 1) )//本地皮带停止状态且没有报警
                {
                    comm_node_new.rw_flag = 1;
                    comm_node_new.inverter_no = i;
                    comm_node_new.speed_gear = g_speed_gear_status;
                    comm_node_new.comm_interval = user_paras_local.belt_para[i - 1].Start_Delay_Time;
                    comm_node_new.comm_retry = 3;
                    AddUartSendData2Queue(comm_node_new);
                }
            }
        //}  
    }  
}

void read_user_paras(void)
{
    u16 i;
    u16 data;
    
    for(i=0; i<USER_PARA_DATA_LEN; i++)
    {
        data = *((u16*)(UserParaStartAddress+2*i));
        *((u16*)(&user_paras_local)+3+i) = data;
    }
    user_paras_local.Station_No = local_station;
    user_paras_local.Version_No_L = 0x0009;
    user_paras_local.Version_No_H = 0x0100;
    //参数有效性校验
    if(user_paras_local.Up_Stream_No > 255)
    {
        user_paras_local.Up_Stream_No = 0;
    }
    if(user_paras_local.Down_Stream_No > 255)
    {
        user_paras_local.Down_Stream_No = 0;
    }
    if(user_paras_local.Belt_Number > 10)
    {
        user_paras_local.Belt_Number = 0;
    }

    for (i = 0; i < user_paras_local.Belt_Number; i++) {
        stopspeed_default[i] = user_paras_local.belt_para[i].Gear_2_Speed_Freq;
    }
}
void write_user_paras(u16* para)
{
    u8 i;
    
    FLASH_Unlock();
    FLASH_ErasePage(UserParaStartAddress);
    
    for(i=0; i<USER_PARA_DATA_LEN; i++)
    {
        FLASH_ProgramHalfWord(UserParaStartAddress+2*i,para[i]);
    }
    
    FLASH_Lock();
}


void logic_upstream_io_allow_output(void)
{
    u16 i = 0;
    u16 flag = 0;
    u16 j = 0;

    // 23/12/19 存在屯包的情况
    if (user_paras_local.Belt_Number == 0) {
        return;
    }
    if(g_speed_gear_status == 0){
      B_STREAM_OUT_(Bit_RESET);
      return;
    }

    //屯包模式
    if (stockInsertMod == 0) {
        for (i = 0; i < user_paras_local.Belt_Number; i++)
        {
            if ((((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) ||
                (((user_paras_local.belt_para[i].Func_Select_Switch >> 7) & 0x1) == 1) ||
                (((user_paras_local.belt_para[i].Func_Select_Switch >> 8) & 0x1) == 1)) {
                flag = 1;
                break;
            }
        }

        if (flag == 1) {
            if (bCombinerInInfo.input_info.input_state == 1) {
                B_STREAM_OUT_(Bit_RESET);
            }
            else {
                if (stockFullFlag == 0) {
                    B_STREAM_OUT_(Bit_SET);
                }
                else {
                    B_STREAM_OUT_(Bit_RESET);
                }
            }

            if (bCombinerInInfo.input_info.input_state == 1) {
                if ((bCombinerInInfo.button_hold_time < 20000)  && (bCombinerInInfo.combine_cnt >= 1)) {
                        bCombinerInInfo.combine_cnt = 1;
                        B_STREAM_OUT_(Bit_SET);
                }
                else if((bCombinerInInfo.button_hold_time >= 20000) && (bCombinerInInfo.combine_cnt >= 4000)) {
                    bCombinerInInfo.combine_cnt = 4000;
                    B_STREAM_OUT_(Bit_SET);
                }
            }

     

        }
        else {
            if (((inverter_status_buffer[0].fault_code >> 4) & 0x1) == 1) {
                B_STREAM_OUT_(Bit_SET);
            }
            else {
                B_STREAM_OUT_(Bit_RESET);
            }
        }
    }
    else if (stockInsertMod == 1) {
        for (i = 0; i < user_paras_local.Belt_Number; i++)
        {
            if ((((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) ||
                (((user_paras_local.belt_para[i].Func_Select_Switch >> 7) & 0x1) == 1) ||
                (((user_paras_local.belt_para[i].Func_Select_Switch >> 8) & 0x1) == 1)) {
                flag = 1;
                break;
            }
        }
        if (flag == 0) {
            if (((inverter_status_buffer[0].fault_code >> 4) & 0x1) == 1) {
                B_STREAM_OUT_(Bit_SET);
            }
            else {
                B_STREAM_OUT_(Bit_RESET);
            }
        }
    }



    for (j = 0; j < user_paras_local.Belt_Number; j++) {
        if ((((inverter_status_buffer[j].input_status >> 1) & 0x1) == 0) ||
            (get_inverter_fault_status(inverter_status_buffer[j]) == 1)) {
            B_STREAM_OUT_(Bit_RESET);
        }
    }
}

// add 23/12/12 屯包逻辑

// 屯包时获取光电状态

u8 logicStockgetCurPhotoState(u8 index, u8 type)
{
    u8 j = 0;

    // 当前段 对应的光电
    if (type == 0) {
        if (index == user_paras_local.Belt_Number) {
            j = bDownStockInfo.input_info.input_state;
            return j;
        }

        switch (index) {
        case 0:
            j = bPhoto1Info.input_info.input_state;
            return j;
            break;
        case 1:
            j = bPhoto2Info.input_info.input_state;
            return j;
            break;
        case 2:
            j = bPhoto3Info.input_info.input_state;
            return j;
            break;
        case 3:
            j = bPhoto4Info.input_info.input_state;
            return j;
            break;
        case 4:
            j = bPhoto5Info.input_info.input_state;
            return j;
            break;
        default:
            break;
        }
    }

    //屯包起始段对应的光电
    if (type == 1) {
        switch (index) {
        case 0:
            j = bUpStockInfo.input_info.input_state;
            return j;
            break;
        case 1:
            j = bPhoto1Info.input_info.input_state;
            return j;
            break;
        case 2:
            j = bPhoto2Info.input_info.input_state;
            return j;
            break;
        case 3:
            j = bPhoto3Info.input_info.input_state;
            return j;
            break;
        case 4:
            j = bPhoto4Info.input_info.input_state;
            return j;
            break;
        default:
            break;
        }
    }
    return j;
}

// 屯包过程   其中的一种方式 一段一段屯包
void logicStockProcess(void)
{

    u16 i = 0;
    u16 j = 0;
    u16 k = 0;
    
    
    COMM_NODE_T  comm_node_new;

    //最上游屯包起始光电状态
    u16  photoUpStat = 0;
    //最上游屯包段皮带索引
    static u16  ModUpIndex = 0;

    //反控合流机停止倒计时
    static u16 combSwitchDown = 0;
    //反控合流机启动倒计时
    static u16 combSwitchUp = 0;

    //临时的光电状态
    u16 photoTmp = 0;
    // 当前屯包段光电状态
    u16 photoCurStat = 0;
    // 当前屯包段索引
    static u16 photoCurIndex = 0;
   // 上一次判断屯包段索引
    static u16 photolastIndex = 0xFFFF;
   // 上一次的屯包状态  0 停止屯包  1 屯过包
    static u16 stockRunStat = INVALUE;


    // 屯包段最多5段
    if (user_paras_local.Belt_Number == 0) {
        return;
    }

    if (user_paras_local.Belt_Number > 5) {
        return;
    }

    if (g_speed_gear_status == 0) {

        if (combSwitchDown == 0) {
            combSwitchDown = 1;
        }
        combSwitchUp = 0;
        stockRunStat = INVALUE;
        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;

        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        return;
    }

    //add 240221 
    for (j = 0; j < user_paras_local.Belt_Number; j++) {
        if ((((inverter_status_buffer[j].input_status >> 1) & 0x1) == 0) ||
            (get_inverter_fault_status(inverter_status_buffer[j]) == 1)) {
            return;
        }
    }

    //判断最上游屯包段 找到屯包起始光电
    for (i = 0; i < user_paras_local.Belt_Number; i++)
    {
        if ((((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) ||
                (((user_paras_local.belt_para[i].Func_Select_Switch >> 7) & 0x1) == 1))
        {
            switch (i) {
            case 0:
                ModUpIndex = 0;
                photoUpStat = bUpStockInfo.input_info.input_state;
                break;
            case 1:
                ModUpIndex = 1;
                photoUpStat = bPhoto1Info.input_info.input_state;
                break;
            case 2:
                ModUpIndex = 2;
                photoUpStat = bPhoto2Info.input_info.input_state;
                break;
            case 3:
                ModUpIndex = 3;
                photoUpStat = bPhoto3Info.input_info.input_state;
                break;
            case 4:
                ModUpIndex = 4;
                photoUpStat = bPhoto4Info.input_info.input_state;
                break;
            default:
                return;
                break;
            }
            break;
        }
    }

    // 没配置屯包功能
    if (user_paras_local.Belt_Number == i) {
        return;
    }


    // 合流机反控启动
    if (bCombinerInInfo.input_info.input_state == 1) {
        if (bCombinerInInfo.input_info.input_trig_mode == INPUT_TRIG_UP) {
            bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_NULL;
            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
            {
                if (((user_paras_local.belt_para[k-1].Func_Select_Switch >> 6) == 1) ||
                    ((user_paras_local.belt_para[k-1].Func_Select_Switch >> 7) == 1) ||
                    ((user_paras_local.belt_para[k-1].Func_Select_Switch >> 8) == 1)) {
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = k;
                        comm_node_new.speed_gear = g_speed_gear_status;
                        comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);

                }
            }
            if (combSwitchDown == 0) {
                combSwitchDown = 1;
            }
            combSwitchUp = 0;
            stockRunStat = INVALUE;
            photoCurIndex = ModUpIndex;
            photolastIndex = 0xFFFF;
            stockFullFlag = 0;
        }
        return;
    }

    // 合流机反控结束
    if (bCombinerInInfo.input_info.input_state == 0) {
        if (bCombinerInInfo.input_info.input_trig_mode == INPUT_TRIG_DOWN) {
            bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_NULL;

            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
            {
                if (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) == 1) ||
                    ((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) == 1) ||
                    ((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 8) == 1)) {
                    comm_node_new.rw_flag = 1;
                    comm_node_new.inverter_no = k;
                    comm_node_new.speed_gear = 0;
                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                    comm_node_new.comm_retry = 3;
                    AddUartSendData2Queue(comm_node_new);
                }
            }
            if (combSwitchDown == 0) {
                combSwitchDown = 1;
            }
            combSwitchUp = 0;
            stockRunStat = INVALUE;
            photoCurIndex = ModUpIndex;
            photolastIndex = 0xFFFF;
            stockFullFlag = 0;
            return;
        }
    }

    // 屯包起始光电无信号
    if (photoUpStat == 0) {

        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;


        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }

        //合流机不反控  
        if (bCombinerInInfo.input_info.input_state == 0) {
            //停止屯包段皮带
            if (stockRunStat == VALUE) {
                stockRunStat = INVALUE;

                for (i = user_paras_local.Belt_Number; i > ModUpIndex; i--)
                {
                    if ((((user_paras_local.belt_para[i - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                        (((user_paras_local.belt_para[i - 1].Func_Select_Switch >> 7) & 0x1) == 1))
                    {
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = i;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = user_paras_local.belt_para[i - 1].Link_Stop_Time;
                        if (bUpStockInfo.button_hold_time < SHORT_TRIG_TIMEONE) {
                            comm_node_new.comm_interval = user_paras_local.belt_para[i - 1].Link_Stop_Time + SHORT_KEEP_TIMEONE;
                        }
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);

                        //if (combSwitchDown == 0) {
                        //    combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                        //}
                        if (bUpStockInfo.button_hold_time < SHORT_TRIG_TIMEONE) {
                            if (combSwitchDown == 0) {
                                combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + SHORT_KEEP_TIMEONE;
                            }
                        }
                        else {
                            if (combSwitchDown == 0) {
                                combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                            }
                        }


                        combSwitchUp = 0;
                    }
                }
            }         
        }
    }  
    // 屯包起始光电有信号
    // 根据光电决定启停
    //一旦最后一节屯包段满了  停止掉前面的屯包皮带  
    if (photoUpStat == 1) {


        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //合流机不反控的情况下  
        if (bCombinerInInfo.input_info.input_state == 0) {

            for (i = photoCurIndex; i < user_paras_local.Belt_Number; i++)
            {
                if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) {
                    photoCurStat = logicStockgetCurPhotoState(i, 0);

                    //该段未屯满
                    if (photoCurStat == 0) {
                        if (photolastIndex != photoCurIndex) {
                            photolastIndex = photoCurIndex;
                            for (j = i + 1; j > photoCurIndex; j--)
                            {
                                if ((((user_paras_local.belt_para[j - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                    (((user_paras_local.belt_para[j - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                    comm_node_new.rw_flag = 1;
                                    comm_node_new.inverter_no = j;
                                    comm_node_new.speed_gear = g_speed_gear_status;
                                    comm_node_new.comm_interval = user_paras_local.belt_para[j - 1].Link_Start_Time;
                                    comm_node_new.comm_retry = 3;
                                    AddUartSendData2Queue(comm_node_new);

                                    stockRunStat = VALUE;
                                }
                            }
                        }
                        stockFullFlag = 0;
                        break;
                    }

                    //该屯包段已经屯满的情况
                    if (photoCurStat == 1) {
                        //查找之后的模块是否还屯包( 并且是否屯满包裹 如果不屯包 则应该停止之前的屯包模块
                        for (j = i + 1; j < user_paras_local.Belt_Number; j++)
                        {

                            if (((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1) == 1) {
                                // 找到没有屯满包裹的模块
                                photoTmp = logicStockgetCurPhotoState(j, 0);

                                if (photoTmp == 0) {
                                    break;
                                }
                            }
                        }

                        if (j != user_paras_local.Belt_Number) {
                            if (photolastIndex != j) {
                                for (k = j + 1; k > photoCurIndex; k--)
                                {
                                    if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                        (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                        comm_node_new.rw_flag = 1;
                                        comm_node_new.inverter_no = k;
                                        comm_node_new.speed_gear = g_speed_gear_status;
                                        comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                        comm_node_new.comm_retry = 3;
                                        AddUartSendData2Queue(comm_node_new);

                                        stockRunStat = VALUE;
                                    }
                                }
                            }
                            photoCurIndex = j;
                            photolastIndex = photoCurIndex;
                            stockFullFlag = 0;
                            break;
                        }
                        if (j == user_paras_local.Belt_Number) {
                            if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                                photoTmp = logicStockgetCurPhotoState(j, 0);
                                if (photoTmp == 0) {
                                    if (photolastIndex != j) {
                                        for (k = j; k > ModUpIndex; k--)
                                        {
                                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                                comm_node_new.rw_flag = 1;
                                                comm_node_new.inverter_no = k;
                                                comm_node_new.speed_gear = g_speed_gear_status;
                                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                                comm_node_new.comm_retry = 3;
                                                AddUartSendData2Queue(comm_node_new);
                                                stockRunStat = VALUE;
                                            }
                                        }
                                    }                                   
                                    photoCurIndex = j;
                                    photolastIndex = photoCurIndex;
                                    if (combSwitchUp == 0) {
                                        combSwitchUp = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Start_Time + 1;
                                    }  
                                    combSwitchDown = 0;
                                    stockFullFlag = 0;
                                    break;
                                }
                                if (photoTmp == 1) {
                                    if (stockRunStat == VALUE) {
                                        for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                                        {
                                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                                comm_node_new.rw_flag = 1;
                                                comm_node_new.inverter_no = k;
                                                comm_node_new.speed_gear = 0;
                                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                                comm_node_new.comm_retry = 3;
                                                AddUartSendData2Queue(comm_node_new);
                                            }
                                        }
                                        if (combSwitchDown == 0) {
                                            combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                                        }                                        
                                        combSwitchUp = 0;
                                        stockRunStat = INVALUE;
                                    }
                                    photoCurIndex = j;
                                    photolastIndex = photoCurIndex;
                                    stockFullFlag = 1;

                                    if (stockRunStat == INVALUE) {
                                        photoCurIndex = ModUpIndex;
                                        photolastIndex = 0xFFFF;
                                    }
                                    break;
                                }
                            }
                            if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 0) {
                                if (stockRunStat == VALUE) {
                                    for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                                    {
                                        if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                            (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                            comm_node_new.rw_flag = 1;
                                            comm_node_new.inverter_no = k;
                                            comm_node_new.speed_gear = 0;
                                            comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                            comm_node_new.comm_retry = 3;
                                            AddUartSendData2Queue(comm_node_new);
                                        }
                                    }
                                    if (combSwitchDown == 0) {
                                        combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                                    }
                                    combSwitchUp = 0;
                                    stockRunStat = INVALUE;
                                }
                                stockFullFlag = 1;

                                if (stockRunStat == INVALUE) {
                                    photoCurIndex = ModUpIndex;
                                    photolastIndex = 0xFFFF;
                                }
                                break;
                               
                            }
                        }
                    }
                    break;
                }
            }

            //无屯包段 但是有跟随的屯包段
            if (i == user_paras_local.Belt_Number) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    photoTmp = logicStockgetCurPhotoState(i, 0);
                    if (photoTmp == 0) {
                        if (photolastIndex != photoCurIndex) {
                            photolastIndex = photoCurIndex;

                            for (k = i; k > ModUpIndex; k--)
                            {
                                if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                    (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                    comm_node_new.rw_flag = 1;
                                    comm_node_new.inverter_no = k;
                                    comm_node_new.speed_gear = g_speed_gear_status;
                                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                    comm_node_new.comm_retry = 3;
                                    AddUartSendData2Queue(comm_node_new);
                                    stockRunStat = VALUE;
                                }
                            }
                            photoCurIndex = i;
                            photolastIndex = photoCurIndex;

                        }
                        if (combSwitchUp == 0) {
                            combSwitchUp = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Start_Time + 1;
                        }
                        combSwitchDown = 0;
                        stockFullFlag = 0;
                    }

                    if (photoTmp == 1) {
                        if (stockRunStat == VALUE) {
                            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                            {
                                if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                    (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                    comm_node_new.rw_flag = 1;
                                    comm_node_new.inverter_no = k;
                                    comm_node_new.speed_gear = 0;
                                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                    comm_node_new.comm_retry = 3;
                                    AddUartSendData2Queue(comm_node_new);
                                }
                            }

                        }
                        if (combSwitchDown == 0) {
                            combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                        }
                        stockRunStat = INVALUE;
                        combSwitchUp = 0;
                        stockFullFlag = 1;

                        if (stockRunStat == INVALUE) {
                            photoCurIndex = ModUpIndex;
                            photolastIndex = 0xFFFF;
                        }

                    }
                }
            }            
        }

        
    }
}


void logicStockProcessTwo(void)
{
    u16 i = 0;
    u16 j = 0;
    u16 k = 0;


    COMM_NODE_T  comm_node_new;

    //最上游屯包起始光电状态
    u16  photoUpStat = 0;
    //最上游屯包段皮带索引
    u16  ModUpIndex = 0;

    //反控合流机停止倒计时
    static u16 combSwitchDown = 0;
    //反控合流机启动倒计时
    static u16 combSwitchUp = 0;

    //临时的光电状态
    u16 photoTmp = 0;
    // 当前屯包段光电状态
    u16 photoCurStat = 0;
    // 当前屯包段索引
    static u16 photoCurIndex = 0;
    // 上一次判断屯包段索引
    static u16 photolastIndex = 0xFFFF;
    // 上一次的屯包状态  0 停止屯包  1 屯过包
    static u16 stockRunStat = INVALUE;




    // 屯包段最多5段
    if (user_paras_local.Belt_Number == 0) {
        return;
    }

    if (user_paras_local.Belt_Number > 5) {
        return;
    }

    if (g_speed_gear_status == 0) {
        combSwitchUp = 0;
        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        return;
    }

    //判断最上游屯包段 找到屯包起始光电
    for (i = 0; i < user_paras_local.Belt_Number; i++)
    {
        if ((((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) ||
            (((user_paras_local.belt_para[i].Func_Select_Switch >> 7) & 0x1) == 1))
        {
            switch (i) {
            case 0:
                ModUpIndex = 0;
                photoUpStat = bUpStockInfo.input_info.input_state;
                break;
            case 1:
                ModUpIndex = 1;
                photoUpStat = bPhoto1Info.input_info.input_state;
                break;
            case 2:
                ModUpIndex = 2;
                photoUpStat = bPhoto2Info.input_info.input_state;
                break;
            case 3:
                ModUpIndex = 3;
                photoUpStat = bPhoto3Info.input_info.input_state;
                break;
            case 4:
                ModUpIndex = 4;
                photoUpStat = bPhoto4Info.input_info.input_state;
                break;
            default:
                return;
                break;
            }
            break;
        }
    }

    // 没配置屯包功能
    if (user_paras_local.Belt_Number == i) {
        return;
    }

    // 合流机反控启动
    if (bCombinerInInfo.input_info.input_state == 1) {
        if (bCombinerInInfo.input_info.input_trig_mode == INPUT_TRIG_UP) {
            bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_NULL;
            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
            {
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = k;
                comm_node_new.speed_gear = g_speed_gear_status;
                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
//            combSwitchDown = 1;
//            combSwitchUp = 0;
            stockRunStat = INVALUE;
            stockFullFlag = 0;
        }
    }

    // 合流机反控停止
    if (bCombinerInInfo.input_info.input_state == 0) {
        if (bCombinerInInfo.input_info.input_trig_mode == INPUT_TRIG_DOWN) {
            bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_NULL;

            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
            {
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = k;
                comm_node_new.speed_gear = 0;
                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
//            combSwitchDown = 1;
//            combSwitchUp = 0;
            stockRunStat = INVALUE;
            stockFullFlag = 0;
        }
    }

    // 屯包起始光电无信号
    if (photoUpStat == 0) {

        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;


        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }

        //合流机不反控  
        if (bCombinerInInfo.input_info.input_state == 0) {
            //停止屯包段皮带
            if (stockRunStat == VALUE) {
                stockRunStat = INVALUE;

                for (i = user_paras_local.Belt_Number; i > ModUpIndex; i--)
                {
                    if ((((user_paras_local.belt_para[i - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                        (((user_paras_local.belt_para[i - 1].Func_Select_Switch >> 7) & 0x1) == 1))
                    {
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = i;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = user_paras_local.belt_para[i - 1].Link_Stop_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);
                    }
                }
                combSwitchUp = 0;
                if (combSwitchDown == 0) {
                    combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                }

            }
        }
    }
    // 屯包起始光电有信号    
    // 根据光电决定启停
    //一旦最后一节屯包段满了  停止掉前面的屯包皮带  
    if (photoUpStat == 1) {


        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //合流机不反控的情况下  
        if (bCombinerInInfo.input_info.input_state == 0) {

            //最后一段反控合流机屯包
//            if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
//                photoTmp = logicStockgetCurPhotoState(user_paras_local.Belt_Number, 0);
//                if (photoTmp == 0) {
//                    if (stockRunStat == INVALUE) {
//                        for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
//                        {
//                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
//                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
//                                comm_node_new.rw_flag = 1;
//                                comm_node_new.inverter_no = k;
//                                comm_node_new.speed_gear = g_speed_gear_status;
//                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
//                                comm_node_new.comm_retry = 3;
//                                AddUartSendData2Queue(comm_node_new);
//                            }
//                        }
//                        stockRunStat = VALUE;
//                    }
////                    combSwitchUp = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Start_Time + 1;
//                    stockFullFlag = 0;
//                }
//
//                if (photoTmp == 1) {
//                    if (stockRunStat == VALUE) {
//                        for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
//                        {
//                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
//                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
//                                comm_node_new.rw_flag = 1;
//                                comm_node_new.inverter_no = k;
//                                comm_node_new.speed_gear = 0;
//                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
//                                comm_node_new.comm_retry = 3;
//                                AddUartSendData2Queue(comm_node_new);
//                            }
//                        }
////                        combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
//                        stockRunStat = INVALUE;
//                    }
//                    stockFullFlag = 1;
//                }
//            }

            //最后一段反控合流机屯包
            if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                photoTmp = logicStockgetCurPhotoState(user_paras_local.Belt_Number, 0);
                if (photoTmp == 0) {
                    if (stockRunStat == INVALUE) {
                        for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                        {
                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                comm_node_new.rw_flag = 1;
                                comm_node_new.inverter_no = k;
                                comm_node_new.speed_gear = g_speed_gear_status;
                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                comm_node_new.comm_retry = 3;
                                AddUartSendData2Queue(comm_node_new);
                            }
                        }
                        stockRunStat = VALUE;
                    }
                    //                    combSwitchUp = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Start_Time + 1;
                    stockFullFlag = 0;
                    //for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                    //{
                    //    if (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) {
                    //        photoTmp = logicStockgetCurPhotoState(k-1, 0);
                    //        if (photoTmp == 1) {
                    //            combSwitchUp = user_paras_local.belt_para[k-1].Link_Start_Time + 1;
                    //            combSwitchDown = 0;
                    //        }
                    //        if (photoTmp == 0) {
                    //            combSwitchUp = 0;
                    //            combSwitchDown = user_paras_local.belt_para[k - 1].Link_Stop_Time + 1;
                    //        }
                    //    }
                    //}
                    
                }

                if (photoTmp == 1) {
                    if (stockRunStat == VALUE) {
                        for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                        {
                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                comm_node_new.rw_flag = 1;
                                comm_node_new.inverter_no = k;
                                comm_node_new.speed_gear = 0;
                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                comm_node_new.comm_retry = 3;
                                AddUartSendData2Queue(comm_node_new);
                            }
                        }
                        //                        combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                        stockRunStat = INVALUE;
                    }
                    stockFullFlag = 1;
                }

                for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                {
                    if (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) {
                        photoTmp = logicStockgetCurPhotoState(k - 1, 0);
                        if (photoTmp == 1) {
                            if (combSwitchUp == 0) {
                                combSwitchUp = user_paras_local.belt_para[k - 1].Link_Start_Time + 1;
                            }
                            combSwitchDown = 0;
                        }
                        if (photoTmp == 0) {
                            combSwitchUp = 0;
                            if (combSwitchDown == 0) {
                                combSwitchDown = user_paras_local.belt_para[k - 1].Link_Stop_Time + 1;
                            }
                        }
                        break;
                    }
                }
            }

            //最后一段不反控合流机屯包
            if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 0) {
                for (j = user_paras_local.Belt_Number; j > ModUpIndex; j--) {
                    if (((user_paras_local.belt_para[j - 1].Func_Select_Switch >> 6) & 0x1) == 1) {
                        photoTmp = logicStockgetCurPhotoState(j - 1, 0);

                        if (photoTmp == 0) {
                            if (stockRunStat == INVALUE) {
                                for (k = j; k > ModUpIndex; k--)
                                {
                                    if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                        (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                        comm_node_new.rw_flag = 1;
                                        comm_node_new.inverter_no = k;
                                        comm_node_new.speed_gear = g_speed_gear_status;
                                        comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                        comm_node_new.comm_retry = 3;
                                        AddUartSendData2Queue(comm_node_new);
                                    }
                                }
                                stockRunStat = VALUE;
                            } 
                            if (combSwitchDown == 0) {
                                combSwitchDown = 2;
                            }
                            combSwitchUp = 0;
                            stockFullFlag = 0;
                        }

                        if (photoTmp == 1) {
                            if (stockRunStat == VALUE) {
                                for (k = j; k > ModUpIndex; k--)
                                {
                                    if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                        (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                        comm_node_new.rw_flag = 1;
                                        comm_node_new.inverter_no = k;
                                        comm_node_new.speed_gear = 0;
                                        comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                        comm_node_new.comm_retry = 3;
                                        AddUartSendData2Queue(comm_node_new);
                                    }
                                }
                                stockRunStat = INVALUE;
                            }
                            if (combSwitchDown == 0) {
                                combSwitchDown = 2;
                            }
                            stockFullFlag = 1;
                        }
                        break;
                    }
                }      
            }
        }
    }

    
}


//插包模式1
void logicInsertProcessOne(void)
{

    u16 i = 0;
    u16 j = 0;
    u16 k = 0;


    COMM_NODE_T  comm_node_new;

    //最上游屯包起始光电状态
    u16  photoUpStat = 0;
    //最上游屯包段皮带索引
    static u16  ModUpIndex = 0;

    //反控合流机停止倒计时
    static u16 combSwitchDown = 0;
    //反控合流机启动倒计时
    static u16 combSwitchUp = 0;

    //临时的光电状态
    u16 photoTmp = 0;
    // 当前屯包段光电状态
    u16 photoCurStat = 0;
    // 当前屯包段索引
    static u16 photoCurIndex = 0;
    // 上一次判断屯包段索引
    static u16 photolastIndex = 0xFFFF;
    // 上一次的屯包状态  0 停止屯包  1 屯过包
    static u16 stockRunStat = INVALUE;


    for (j = 0; j < user_paras_local.Belt_Number; j++)
    {
        if ((((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1) == 1) ||
            (((user_paras_local.belt_para[j].Func_Select_Switch >> 7) & 0x1) == 1)) {
            break;
        }
    }

    // 没配置屯包功能
    if (user_paras_local.Belt_Number == j) {
        return;
    }


    // 屯包段最多5段
    if (user_paras_local.Belt_Number == 0) {
        return;
    }

    if (user_paras_local.Belt_Number > 5) {
        return;
    }

    if (bCombinerInInfo.input_info.input_state == 1) {
        B_STREAM_OUT_(Bit_SET);
    }
    else {
        if (stockFullFlag == 0) {
            B_STREAM_OUT_(Bit_SET);
        }
        else {
            B_STREAM_OUT_(Bit_RESET);
        }
    }

    //add 240221 
    for (j = 0; j < user_paras_local.Belt_Number; j++) {
        if ((((inverter_status_buffer[j].input_status >> 1) & 0x1) == 0) ||
            (get_inverter_fault_status(inverter_status_buffer[j]) == 1)) {
            B_STREAM_OUT_(Bit_RESET);


            if (combSwitchDown == 0) {
                combSwitchDown = 1;
            }
            combSwitchUp = 0;
            stockRunStat = INVALUE;
            photoCurIndex = ModUpIndex;
            photolastIndex = 0xFFFF;

            return;
        }
    }


    if (g_speed_gear_status == 0) {

        if (combSwitchDown == 0) {
            combSwitchDown = 1;
        }
        combSwitchUp = 0;
        stockRunStat = INVALUE;
        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;

        // 插包时反控合流机伺服段
#if INSERT_CONT
        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
#endif
       
        B_STREAM_OUT_(Bit_RESET);
        return;
    }

    //判断最上游屯包段 找到屯包起始光电
    for (i = 0; i < user_paras_local.Belt_Number; i++)
    {
        if ((((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) ||
            (((user_paras_local.belt_para[i].Func_Select_Switch >> 7) & 0x1) == 1))
        {
            switch (i) {
            case 0:
                ModUpIndex = 0;
                photoUpStat = bUpStockInfo.input_info.input_state;
                break;
            case 1:
                ModUpIndex = 1;
                photoUpStat = bPhoto1Info.input_info.input_state;
                break;
            case 2:
                ModUpIndex = 2;
                photoUpStat = bPhoto2Info.input_info.input_state;
                break;
            case 3:
                ModUpIndex = 3;
                photoUpStat = bPhoto3Info.input_info.input_state;
                break;
            case 4:
                ModUpIndex = 4;
                photoUpStat = bPhoto4Info.input_info.input_state;
                break;
            default:
                return;
                break;
            }
            break;
        }
    }

    // 没配置屯包功能
    if (user_paras_local.Belt_Number == i) {
        return;
    }


    // 合流机反控启动
    if (bCombinerInInfo.input_info.input_state == 1) {
        if (bCombinerInInfo.input_info.input_trig_mode == INPUT_TRIG_UP) {
            bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_NULL;
            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
            {
                if (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) == 1) ||
                    ((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) == 1) ||
                    ((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 8) == 1)) {
                    comm_node_new.rw_flag = 1;
                    comm_node_new.inverter_no = k;
                    comm_node_new.speed_gear = g_speed_gear_status;
                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                    comm_node_new.comm_retry = 3;
                    AddUartSendData2Queue(comm_node_new);

                }
            }
            if (combSwitchDown == 0) {
                combSwitchDown = 1;
            }
            combSwitchUp = 0;
            stockRunStat = INVALUE;
            photoCurIndex = ModUpIndex;
            photolastIndex = 0xFFFF;
            stockFullFlag = 0;
        }
        return;
    }

    // 合流机反控结束
    if (bCombinerInInfo.input_info.input_state == 0) {
        if (bCombinerInInfo.input_info.input_trig_mode == INPUT_TRIG_DOWN) {
            bCombinerInInfo.input_info.input_trig_mode = INPUT_TRIG_NULL;

            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
            {
                if (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) == 1) ||
                    ((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) == 1) ||
                    ((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 8) == 1)) {
                    comm_node_new.rw_flag = 1;
                    comm_node_new.inverter_no = k;
                    comm_node_new.speed_gear = 0;
                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                    comm_node_new.comm_retry = 3;
                    AddUartSendData2Queue(comm_node_new);
                }
            }
            if (combSwitchDown == 0) {
                combSwitchDown = 1;
            }
            combSwitchUp = 0;
            stockRunStat = INVALUE;
            photoCurIndex = ModUpIndex;
            photolastIndex = 0xFFFF;
            stockFullFlag = 0;
            return;
        }
    }

    // 屯包起始光电无信号
    if (photoUpStat == 0) {

        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;

        // 插包时反控合流机伺服段
#if INSERT_CONT
        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
#endif
        

        //合流机不反控  
        if (bCombinerInInfo.input_info.input_state == 0) {
            //停止屯包段皮带
            if (stockRunStat == VALUE) {
                stockRunStat = INVALUE;

                for (i = user_paras_local.Belt_Number; i > ModUpIndex; i--)
                {
                    if ((((user_paras_local.belt_para[i - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                        (((user_paras_local.belt_para[i - 1].Func_Select_Switch >> 7) & 0x1) == 1))
                    {
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = i;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = user_paras_local.belt_para[i - 1].Link_Stop_Time;
                        // add 240220
                        if (bUpStockInfo.button_hold_time < SHORT_TRIG_TIMEONE) {
                            comm_node_new.comm_interval = user_paras_local.belt_para[i - 1].Link_Stop_Time + SHORT_KEEP_TIMEONE;
                        }
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);

                        if (bUpStockInfo.button_hold_time < SHORT_TRIG_TIMEONE) {
                            if (combSwitchDown == 0) {
                                combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + SHORT_KEEP_TIMEONE;
                            }
                        }
                        else {
                            if (combSwitchDown == 0) {
                                combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                            }
                        }

                        combSwitchUp = 0;
                    }
                }
            }
        }
    }
    // 屯包起始光电有信号
    // 根据光电决定启停
    //一旦最后一节屯包段满了  停止掉前面的屯包皮带  
    if (photoUpStat == 1) {


        // 插包时反控合流机伺服段
#if INSERT_CONT
        //启动合流机 输出高电平
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        //停止控制合流机 输出低电平
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }
#endif
        

        //合流机不反控的情况下  
        if (bCombinerInInfo.input_info.input_state == 0) {

            for (i = photoCurIndex; i < user_paras_local.Belt_Number; i++)
            {
                if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) {
                    photoCurStat = logicStockgetCurPhotoState(i, 0);

                    //该段未屯满
                    if (photoCurStat == 0) {
                        if (photolastIndex != photoCurIndex) {
                            photolastIndex = photoCurIndex;
                            for (j = i + 1; j > photoCurIndex; j--)
                            {
                                if ((((user_paras_local.belt_para[j - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                    (((user_paras_local.belt_para[j - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                    comm_node_new.rw_flag = 1;
                                    comm_node_new.inverter_no = j;
                                    comm_node_new.speed_gear = g_speed_gear_status;
                                    comm_node_new.comm_interval = user_paras_local.belt_para[j - 1].Link_Start_Time;
                                    comm_node_new.comm_retry = 3;
                                    AddUartSendData2Queue(comm_node_new);

                                    stockRunStat = VALUE;
                                }
                            }
                        }
                        stockFullFlag = 0;
                        break;
                    }

                    //该屯包段已经屯满的情况
                    if (photoCurStat == 1) {
                        //查找之后的模块是否还屯包( 并且是否屯满包裹 如果不屯包 则应该停止之前的屯包模块
                        for (j = i + 1; j < user_paras_local.Belt_Number; j++)
                        {

                            if (((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1) == 1) {
                                // 找到没有屯满包裹的模块
                                photoTmp = logicStockgetCurPhotoState(j, 0);

                                if (photoTmp == 0) {
                                    break;
                                }
                            }
                        }

                        if (j != user_paras_local.Belt_Number) {
                            if (photolastIndex != j) {
                                for (k = j + 1; k > photoCurIndex; k--)
                                {
                                    if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                        (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                        comm_node_new.rw_flag = 1;
                                        comm_node_new.inverter_no = k;
                                        comm_node_new.speed_gear = g_speed_gear_status;
                                        comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                        comm_node_new.comm_retry = 3;
                                        AddUartSendData2Queue(comm_node_new);

                                        stockRunStat = VALUE;
                                    }
                                }
                            }
                            photoCurIndex = j;
                            photolastIndex = photoCurIndex;
                            stockFullFlag = 0;
                            break;
                        }
                        if (j == user_paras_local.Belt_Number) {
                            /*if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                                photoTmp = logicStockgetCurPhotoState(j, 0);
                                if (photoTmp == 0) {
                                    if (photolastIndex != j) {
                                        for (k = j; k > ModUpIndex; k--)
                                        {
                                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                                comm_node_new.rw_flag = 1;
                                                comm_node_new.inverter_no = k;
                                                comm_node_new.speed_gear = g_speed_gear_status;
                                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                                comm_node_new.comm_retry = 3;
                                                AddUartSendData2Queue(comm_node_new);
                                                stockRunStat = VALUE;
                                            }
                                        }
                                    }
                                    photoCurIndex = j;
                                    photolastIndex = photoCurIndex;
                                    if (combSwitchUp == 0) {
                                        combSwitchUp = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Start_Time + 1;
                                    }
                                    combSwitchDown = 0;
                                    stockFullFlag = 0;
                                    break;
                                }
                                if (photoTmp == 1) {
                                    if (stockRunStat == VALUE) {
                                        for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                                        {
                                            if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                                (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                                comm_node_new.rw_flag = 1;
                                                comm_node_new.inverter_no = k;
                                                comm_node_new.speed_gear = 0;
                                                comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                                comm_node_new.comm_retry = 3;
                                                AddUartSendData2Queue(comm_node_new);
                                            }
                                        }
                                        if (combSwitchDown == 0) {
                                            combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                                        }
                                        combSwitchUp = 0;
                                        stockRunStat = INVALUE;
                                    }
                                    photoCurIndex = j;
                                    photolastIndex = photoCurIndex;
                                    stockFullFlag = 1;

                                    if (stockRunStat == INVALUE) {
                                        photoCurIndex = ModUpIndex;
                                        photolastIndex = 0xFFFF;
                                    }
                                    break;
                                }
                            }*/
                            /*if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 0) {*/
                                if (stockRunStat == VALUE) {
                                    for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                                    {
                                        if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                            (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                            comm_node_new.rw_flag = 1;
                                            comm_node_new.inverter_no = k;
                                            comm_node_new.speed_gear = 0;
                                            comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                            comm_node_new.comm_retry = 3;
                                            AddUartSendData2Queue(comm_node_new);
                                        }
                                    }
                                    if (combSwitchDown == 0) {
                                        combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                                    }
                                    combSwitchUp = 0;
                                    stockRunStat = INVALUE;
                                }
                                stockFullFlag = 1;

                                if (stockRunStat == INVALUE) {
                                    photoCurIndex = ModUpIndex;
                                    photolastIndex = 0xFFFF;
                                }
                                break;

                            //}
                        }
                    }
                    break;
                }
            }

            //无屯包段 但是有跟随的屯包段
            if (i == user_paras_local.Belt_Number) {
                /*if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    photoTmp = logicStockgetCurPhotoState(i, 0);
                    if (photoTmp == 0) {
                        if (photolastIndex != photoCurIndex) {
                            photolastIndex = photoCurIndex;

                            for (k = i; k > ModUpIndex; k--)
                            {
                                if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                    (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                    comm_node_new.rw_flag = 1;
                                    comm_node_new.inverter_no = k;
                                    comm_node_new.speed_gear = g_speed_gear_status;
                                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Start_Time;
                                    comm_node_new.comm_retry = 3;
                                    AddUartSendData2Queue(comm_node_new);
                                    stockRunStat = VALUE;
                                }
                            }
                            photoCurIndex = i;
                            photolastIndex = photoCurIndex;

                        }
                        if (combSwitchUp == 0) {
                            combSwitchUp = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Start_Time + 1;
                        }
                        combSwitchDown = 0;
                        stockFullFlag = 0;
                    }

                    if (photoTmp == 1) {
                        if (stockRunStat == VALUE) {
                            for (k = user_paras_local.Belt_Number; k > ModUpIndex; k--)
                            {
                                if ((((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 6) & 0x1) == 1) ||
                                    (((user_paras_local.belt_para[k - 1].Func_Select_Switch >> 7) & 0x1) == 1)) {
                                    comm_node_new.rw_flag = 1;
                                    comm_node_new.inverter_no = k;
                                    comm_node_new.speed_gear = 0;
                                    comm_node_new.comm_interval = user_paras_local.belt_para[k - 1].Link_Stop_Time;
                                    comm_node_new.comm_retry = 3;
                                    AddUartSendData2Queue(comm_node_new);
                                }
                            }

                        }
                        if (combSwitchDown == 0) {
                            combSwitchDown = user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Link_Stop_Time + 1;
                        }
                        stockRunStat = INVALUE;
                        combSwitchUp = 0;
                        stockFullFlag = 1;

                        if (stockRunStat == INVALUE) {
                            photoCurIndex = ModUpIndex;
                            photolastIndex = 0xFFFF;
                        }

                    }
                }*/
            }
        }
    }
}


void logicStockInsertProcess(void)
{
    //if (bSwitchInInfo.input_info.input_state == 1) {
    //    logicInsertProcessOne();
    //    stockInsertMod = 1;
    //}
    //else if (bSwitchInInfo.input_info.input_state == 0) {
    //    logicStockProcess();
    //    stockInsertMod = 0;
    //}

    logicInsertProcessOne();
    stockInsertMod = 1;
}