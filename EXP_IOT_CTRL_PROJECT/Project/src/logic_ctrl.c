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
//��ͣ��λ�ź�
sButton_Info  bEmergencyResetInfo;
//�����Ͱ����
sButton_Info  bUpStockInfo;
//�����Ͱ����
sButton_Info  bDownStockInfo;
//�����������ź�
sButton_Info  bCombinerInInfo;
//�Ͱ�����л��ź�
sButton_Info  bSwitchInInfo;

INVERTER_STATUS_T  inverter_status_buffer[10];//����״̬

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
u8  polling_num;//��ѯվ��(��1��ʼ)

u8  g_remote_start_flag;   //Զ����ͣ״̬�ı���(0:�ޱ仯 1:ֹͣ������ 2:������ֹͣ)
u8  g_remote_start_status; //Զ����ͣ״̬(0:ֹͣ״̬ 1:����״̬)
u8  g_remote_speed_status; //Զ�̸ߵ���״̬(0:���� 1:����)
u8  g_link_up_stream_status; //���������ź�
u8  g_link_down_stream_status; //���������ź�
u8  g_set_start_status; //���õ����ͣ״̬(0:ֹͣ 1:����)
u8  g_set_speed_status; //���õ���ߵ���״̬(0:���� 1:����)
u8  g_read_start_status; //��ǰ�ĵ����ͣ״̬(0:ֹͣ 1:����)
u8  g_alarm_type; //�������(bit0:�������,bit1:�°�����,bit2:485ͨѶ����)
u8  g_speed_gear_status; //��ǰ���ٶȵ�λ(0:ֹͣ 1~5:�嵵�ٶ�)
u8  g_block_disable_flag = 0; //�°���⹦�ܽ�ֹ���
// add 2023/11/02
u8  g_link_down_phototrig_status = 0;      //���εĹ���Ƿ񴥷���

//u16 start_delay_time_cnt;
//u16 stop_delay_time_cnt;
u32 block_check_time_cnt[10];
u16 reset_start_time_cnt = 0;
u8  reset_start_flag = 0;
// can ���յ��ļ�ͣ�ź�״̬ add 2023/11/02
u8  g_emergency_stop = 0;

// ÿһ��Ƥ����ֹͣ״̬���ֵ���ʱ add 2023/11/02
u16 logic_upload_stopStatus[BELT_NUMMAX] = { 0 };
// ÿһ��Ƥ���ϴε��ٶȼ�¼ add 2023/11/02
u16 logic_upload_lastRunStatus[BELT_NUMMAX] = { 0 };

// �Ͱ��ж�
u16 stockFullFlag = 0;

//add 240131  ֱ�������������  ��������Ҫ�ж������ź�   0��ֱ������  1 ��ֱ������
u16 extralAllow = 1;

//add 240220  ���ģʽ �������Ͱ�ģʽ  1 ���ģʽ 0 �Ͱ�ģʽ
u16 stockInsertMod = 0;

//add 240226  ����ʱ��һ��ʱ�䱣�ֵ���
u16 startKeepLow = 0;

//add 240226  ���õ�Ŀ���ٶ�
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
    //������
    if((q->rear + 1) % q->maxSize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxSize; // �����β����һ��λ��
    q->queue[q->rear] = x; // ��x��ֵ�����µĶ�β
}

COMM_NODE_T* GetUartSendDataFromQueue(void)
{
    COMM_SEND_QUEUE *q = &uartSendQueue;
    //���п�
    if(q->front == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % (q->maxSize); // ʹ����ָ��ָ����һ��λ��
    return (COMM_NODE_T*)(&(q->queue[q->front])); // ���ض���Ԫ��
}
u8 IsUartSendQueueFree(void)
{
    COMM_SEND_QUEUE *q = &uartSendQueue;
    //���п�
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

    // �Ͱ�ֻȡ���������ж�
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
    //������
    if((q->rear + 1) % q->maxSize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxSize; // �����β����һ��λ��
    q->queue[q->rear] = x; // ��x��ֵ�����µĶ�β
}
MODULE_STATUS_T* GetModuleStatusDataFromQueue(void)
{
    MODULE_STATUS_QUEUE *q = &moduleStatusQueue;
    //���п�
    if(q->front == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % q->maxSize; // ʹ����ָ��ָ����һ��λ��
    return (MODULE_STATUS_T*)(&(q->queue[q->front])); // ���ض���Ԫ��
}
u8 IsModuleStatusQueueFree(void)
{
    MODULE_STATUS_QUEUE *q = &moduleStatusQueue;
    //���п�
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




    
    //����λ�����ź�
    if(  bResetInfo.input_info.input_state != l_bit_reset_in
      && bResetInfo.input_info.input_confirm_times == 0)
    {
        bResetInfo.input_info.input_middle_state = l_bit_reset_in;
    }
    if(  bResetInfo.input_info.input_middle_state == l_bit_reset_in
      && bResetInfo.input_info.input_middle_state != bResetInfo.input_info.input_state)
    {
        bResetInfo.input_info.input_confirm_times++;
        if(bResetInfo.input_info.input_confirm_times > 50)//��ť����ʱ��50ms
        {
            bResetInfo.input_info.input_state = bResetInfo.input_info.input_middle_state;
            bResetInfo.input_info.input_confirm_times = 0;
            //ȡ������
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

    //������1�����ź�
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
        if(bPhoto1Info.input_info.input_confirm_times > 50)//����ʱ��50ms
        {
            bPhoto1Info.input_info.input_state = bPhoto1Info.input_info.input_middle_state;
            bPhoto1Info.input_info.input_confirm_times = 0;
            //��¼���1 �б仯
            inverter_status_buffer[0].input_status |= (0x1 << 7);
            bPhoto1Info.blocktrig_flag = 1;
            // ������ʱ �жϴ������
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
    //������2�����ź�
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
        if(bPhoto2Info.input_info.input_confirm_times > 50)//����ʱ��50ms
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

    //������3�����ź�
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
        if(bPhoto3Info.input_info.input_confirm_times > 50)//����ʱ��50ms
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
    //������4�����ź�
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
        if(bPhoto4Info.input_info.input_confirm_times > 50)//����ʱ��50ms
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
    //������5�����ź�
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
        if(bPhoto5Info.input_info.input_confirm_times > 50)//����ʱ��50ms
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

    //�������������ź�
    if(  bStartInfo.input_info.input_state != l_bit_start_in
      && bStartInfo.input_info.input_confirm_times == 0)
    {
        bStartInfo.input_info.input_middle_state = l_bit_start_in;
    }
    if(  bStartInfo.input_info.input_middle_state == l_bit_start_in
      && bStartInfo.input_info.input_middle_state != bStartInfo.input_info.input_state)
    {
        bStartInfo.input_info.input_confirm_times++;
        if(bStartInfo.input_info.input_confirm_times > 100)//����ʱ��100ms
        {
            bStartInfo.input_info.input_state = bStartInfo.input_info.input_middle_state;
            bStartInfo.input_info.input_confirm_times = 0;
            if(bStartInfo.input_info.input_state == 1)
            {
                if(isHost)//����
                {
                    Reset_Ctrl_Handle();                  //�й����ȸ�λ����
                    reset_start_time_cnt = 0;
                    Speed_Ctrl_Process(2);
                    //can_bus_send_start_cmd(2);
                    buf[0] = 2;
                    vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_START_CMD, 1);
                }
                else {  //������ ֻ�����Լ�
                    Reset_Ctrl_Handle();                  //�й����ȸ�λ����
                    reset_start_time_cnt = 0;
                    Speed_Ctrl_Process(2);
                }
            }
            //�����һ���� ������ͣ
            if (bStartInfo.input_info.input_state == 0)
            {
//                if (isHost)//����
//                {
//                    Reset_Ctrl_Handle();                  //�й����ȸ�λ����
//                    reset_start_time_cnt = 0;
//                    Speed_Ctrl_Process(0);
//                    //can_bus_send_start_cmd(2);
//                    buf[0] = 0;
//                    vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_START_CMD, 1);
//                }
//                else {  //������ ֻ�����Լ�
//                    Reset_Ctrl_Handle();                  //�й����ȸ�λ����
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

    //����ֹͣ�����ź�
    if(  bStopInfo.input_info.input_state != l_bit_stop_in
      && bStopInfo.input_info.input_confirm_times == 0)
    {
        bStopInfo.input_info.input_middle_state = l_bit_stop_in;
    }
    if(  bStopInfo.input_info.input_middle_state == l_bit_stop_in
      && bStopInfo.input_info.input_middle_state != bStopInfo.input_info.input_state)
    {
        bStopInfo.input_info.input_confirm_times++;
        if(bStopInfo.input_info.input_confirm_times > 100)//����ʱ��100ms
        {
            bStopInfo.input_info.input_state = bStopInfo.input_info.input_middle_state;
            bStopInfo.input_info.input_confirm_times = 0;
            if(bStopInfo.input_info.input_state == 1)
            {
                if(isHost)//����
                {
                    Speed_Ctrl_Process(0);
                    //can_bus_send_start_cmd(0);
                    buf[0] = 0;
                    vcanbus_oenframe_send(&(buf[0]), CAN_FUNC_ID_START_CMD, 1);
                }
                else {  //������ ֻͣ�Լ�
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

    //��������׼���ź�
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
        if ((bStreamInfo.input_info.input_middle_state == 1) && (bStreamInfo.input_info.input_confirm_times > 20))//����ʱ��20ms
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

        if ((bStreamInfo.input_info.input_middle_state == 0) && (bStreamInfo.input_info.input_confirm_times > 200))//����ʱ��200ms
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

    //����ͣ�����ź�
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
        if (bEmergencyInfo.input_info.input_confirm_times > 50)//����ʱ��50ms
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



    //����ͣ��λ�ź�
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
        if (bEmergencyResetInfo.input_info.input_confirm_times > 50)//����ʱ��50ms
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



    //���������Ͱ�����ź�
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
        if (bUpStockInfo.input_info.input_confirm_times > 20)//����ʱ��50ms
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

    //�ߵ�ƽ����ʱ��
    if (bUpStockInfo.input_info.input_state == 1) {
        bUpStockInfo.button_hold_time++;
        if (bUpStockInfo.button_hold_time >= 3000) {
            bUpStockInfo.button_hold_time = 3000;
        }
    }


    //���������Ͱ�����ź�
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
        if (bDownStockInfo.input_info.input_confirm_times > 20)//����ʱ��50ms
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


    //��������������ź�(����)
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
        if (bCombinerInInfo.input_info.input_confirm_times > 20)//����ʱ��50ms
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
    //�͵�ƽ����ʱ��
    if (bCombinerInInfo.input_info.input_state == 0) {
        bCombinerInInfo.button_hold_time++;
        if (bCombinerInInfo.button_hold_time >= 80000) {
            bCombinerInInfo.button_hold_time = 80000;
        }
    }
    

    //����������Ͱ�����л��ź�
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
        if (bSwitchInInfo.input_info.input_confirm_times > 20)//����ʱ��50ms
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

//����,ֹͣ, ��ͣ����ʱ�ı�����Ƥ�����ٶ�  
void Speed_Ctrl_Process(u8 speed_gear)
{
    COMM_NODE_T  comm_node_new;
    u8 i;
    u8 stockFlag = 0;
    
    //��ʼ�Ͱ�ʱ
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
    
    if(g_speed_gear_status == 0 && speed_gear > 0)//����
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
    else if( speed_gear == 0)//ֹͣ
    {
        InitUartSendQueue();
        logic_uarttmp_init();
        for(i=0; i<user_paras_local.Belt_Number; i++)
        {
            //������ֹͣʱ��
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
    else if((g_speed_gear_status != 0) && (speed_gear != 0))//����
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
// ��ȡ���״̬
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
// ��ȡ���ι�紥��״̬
u8 logic_getDownStreamPhotoTrigStatus(u8 i) 
{
    u8 flag = 0;
    u16 j = 0;

    //��������һ��Ƥ��
    if (i == (user_paras_local.Belt_Number - 1)) {
        //���������ⲿ�����źŵ����
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
        // û���ⲿ�����źŵ����  ������ ��ȡ�����ź� û������  ��ֻ�жϱ�Ƥ����Ӧ�Ĺ��
        flag = g_link_down_phototrig_status ;
        g_link_down_phototrig_status = 0;
    }
    else {     // �м��Ƥ��
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
    //if(inverter_status.fault_code & 0x1)//485ͨѶ����
    //    return 1;
    if((inverter_status.fault_code>>1) & 0x1)//�°�����
        return 1;
    if(((inverter_status.fault_code>>8)&0xFF) != 0)//��Ƶ��������
        return 1;
    return 0;
}

//����ֹͣ_��紥������
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
    
    if(((user_paras_local.belt_para[photo_index].Func_Select_Switch>>3)&0x1) == 0)//δ���û��Ź���
    {
        return;
    }
    //ȷ�����ε�״̬
    //���ö�Ƥ��Ϊ���һ��Ƥ��
    if(photo_index+1 == user_paras_local.Belt_Number)
    {
        if (user_paras_local.Down_Stream_No != 0)//����������վ��
        {
            //            link_down_status = 1;
            link_down_status = g_link_down_stream_status;
        }

        //��������ⲿ׼���ź�
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


    if((user_paras_local.belt_para[photo_index].Func_Select_Switch>>1)&0x1)//�������ܿ���
    {
        if((inverter_status_buffer[photo_index].fault_code>>4)&0x1)//����Ƥ������״̬
        {
            if(link_down_status == 0)//���β��������
            {

                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = photo_index + 1;
                comm_node_new.speed_gear = 0;
                comm_node_new.comm_interval = user_paras_local.belt_para[photo_index].Link_Stop_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);

                if (photo_index == 0) {
                    if ((user_paras_local.belt_para[photo_index].Func_Select_Switch >> 5) & 0x1) { //����ͬ��ֹͣ
                        can_bus_send_func_upStreamStop_cmd();
                        can_bus_send_func_upStreamStop_cmd();
                        return;
                    }
                }

                //����ͬ��ֹͣ
                for (j = photo_index; j >= 1; j--) {
                    if ((user_paras_local.belt_para[j].Func_Select_Switch >> 5) & 0x1) //����ͬ��ֹͣ
                    {
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = j;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = user_paras_local.belt_para[j - 1].Link_Stop_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);

                        if (j == 1) {
                            if ((user_paras_local.belt_para[j-1].Func_Select_Switch >> 5) & 0x1) { //����ͬ��ֹͣ
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

// һ��ֻ�õ�����������
// �ⲿ�������ź�
void Linkage_stream_extra_signal(u8 inverter_index, u8 start_flag)
{
    COMM_NODE_T  comm_node_new;
    u16 i = 0;

    for (i = 0; i < user_paras_local.Belt_Number; i++) {
        if ((user_paras_local.belt_para[i].Func_Select_Switch >> 4) & 0x1)
        {
            if ((user_paras_local.belt_para[inverter_index].Func_Select_Switch >> 1) & 0x1)//�������ܿ���
            {
                if (start_flag == 1)//��������
                {
                    if ((((inverter_status_buffer[inverter_index].fault_code >> 4) & 0x1) == 0)
                        && (get_inverter_fault_status(inverter_status_buffer[inverter_index]) != 1))//����Ƥ��ֹͣ״̬��û�б���
                    {
                        //����
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = inverter_index + 1;
                        comm_node_new.speed_gear = g_speed_gear_status;
                        comm_node_new.comm_interval = user_paras_local.belt_para[inverter_index].Link_Start_Time;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);
                    }
                }
                else//����ֹͣ
                {

                    if ((inverter_status_buffer[inverter_index].fault_code >> 4) & 0x1)//����Ƥ������״̬
                    {
                        //ֹͣ
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

//������ͣ_�����δ�������
void Linkage_Stream_Ctrl_Handle(u8 inverter_index,u8 start_flag)
{
    COMM_NODE_T  comm_node_new;
    
    if((user_paras_local.belt_para[inverter_index].Func_Select_Switch>>1)&0x1)//�������ܿ���
    {
        if(start_flag == 1)//��������
        {
            if((((inverter_status_buffer[inverter_index].fault_code>>4)&0x1) == 0)
              && (get_inverter_fault_status(inverter_status_buffer[inverter_index]) != 1))//����Ƥ��ֹͣ״̬��û�б���
            {
                //����
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = inverter_index+1;
                comm_node_new.speed_gear = g_speed_gear_status;
                comm_node_new.comm_interval = user_paras_local.belt_para[inverter_index].Link_Start_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
        else//����ֹͣ
        {
            // ��������˹��������ͣ �������ù��������ͣ
            if(((user_paras_local.belt_para[inverter_index].Func_Select_Switch>>3)&0x1) == 1)//���û��Ź��ܾ��ɹ�紥��ֹͣ
                return;
            if((inverter_status_buffer[inverter_index].fault_code>>4)&0x1)//����Ƥ������״̬
            {
                //ֹͣ
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
//�°����   ���ݸö�Ƥ��������Ƥ���Ĺ���ź��ۺ��ж� �ö�Ƥ���Ķ°�״̬  
void Block_Check_Ctrl_Handle(void)
{
    COMM_NODE_T  comm_node_new;
    u8  i;
    
    if(g_block_disable_flag == 1) return;
    
    for(i=0; i<user_paras_local.Belt_Number; i++)
    {
        if((user_paras_local.belt_para[i].Func_Select_Switch>>2)&0x1)//�°���⿪��
        {
            if((inverter_status_buffer[i].fault_code>>4)&0x1)//����Ƥ������״̬
            {
                if((get_photo_input_status(i) == 1) && (logic_getDownStreamPhotoTrigStatus(i) == 0))//��类�ڵ�
                {
                    block_check_time_cnt[i]++;
                    if(block_check_time_cnt[i] >= user_paras_local.belt_para[i].Block_Check_Time)
                    {
                        block_check_time_cnt[i] = 0;
                        //ֹͣ
                        comm_node_new.rw_flag = 1;
                        comm_node_new.inverter_no = i+1;
                        comm_node_new.speed_gear = 0;
                        comm_node_new.comm_interval = 1;
                        comm_node_new.comm_retry = 3;
                        AddUartSendData2Queue(comm_node_new);
                        L_ALARM_OUT_(Bit_SET);//����ָʾ�����
                        inverter_status_buffer[i].fault_code |= (0x1<<1);//�°�����
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

//add 240131 �ⲿ�����ź�ֱ������
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
                && (get_inverter_fault_status(inverter_status_buffer[j]) != 1))//����Ƥ��ֹͣ״̬��û�б���
            {
                //����
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = j + 1;
                comm_node_new.speed_gear = g_speed_gear_status;
                comm_node_new.comm_interval = user_paras_local.belt_para[j].Link_Start_Time;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
        else {
            if ((inverter_status_buffer[j].fault_code >> 4) & 0x1)//����Ƥ������״̬
            {
                //ֹͣ
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

//�°���λ,��Ƶ�����ϸ�λ
void Reset_Ctrl_Handle(void)
{
    COMM_NODE_T  comm_node_new;
    u8  i;
    
    for(i=0; i<user_paras_local.Belt_Number; i++)
    {
        if((inverter_status_buffer[i].fault_code>>1)&0x1)//�°���λ
        {
            inverter_status_buffer[i].fault_code &= ~(0x1<<1);
        }
        if(((inverter_status_buffer[i].fault_code>>8)&0xFF) != 0)//��Ƶ����λ
        {
            comm_node_new.rw_flag = 2;
            comm_node_new.inverter_no = i+1;
            comm_node_new.speed_gear = 0;
            comm_node_new.comm_interval = 1;
            comm_node_new.comm_retry = 3;
            AddUartSendData2Queue(comm_node_new);
        }
    }
    L_ALARM_OUT_(Bit_RESET);//����ָʾ��Ϩ��
//    reset_start_time_cnt = 500;
//    g_emergency_stop = 0;
}
//��λ��������Ƶ��
void Reset_Start_Inverter_Handle(void)
{
    COMM_NODE_T  comm_node_new;
    u16 i;
    u8 flag = 0;
    
    if(user_paras_local.Belt_Number == 0) return;
    if(g_speed_gear_status == 0) return;
    if(reset_start_flag != 1) return;
    
    reset_start_flag = 0;

    //��ʼ�Ͱ�ʱ
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
                     && (get_inverter_fault_status(inverter_status_buffer[i-1]) != 1) )//����Ƥ��ֹͣ״̬��û�б���
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
    //������Ч��У��
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

    // 23/12/19 �����Ͱ������
    if (user_paras_local.Belt_Number == 0) {
        return;
    }
    if(g_speed_gear_status == 0){
      B_STREAM_OUT_(Bit_RESET);
      return;
    }

    //�Ͱ�ģʽ
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

// add 23/12/12 �Ͱ��߼�

// �Ͱ�ʱ��ȡ���״̬

u8 logicStockgetCurPhotoState(u8 index, u8 type)
{
    u8 j = 0;

    // ��ǰ�� ��Ӧ�Ĺ��
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

    //�Ͱ���ʼ�ζ�Ӧ�Ĺ��
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

// �Ͱ�����   ���е�һ�ַ�ʽ һ��һ���Ͱ�
void logicStockProcess(void)
{

    u16 i = 0;
    u16 j = 0;
    u16 k = 0;
    
    
    COMM_NODE_T  comm_node_new;

    //�������Ͱ���ʼ���״̬
    u16  photoUpStat = 0;
    //�������Ͱ���Ƥ������
    static u16  ModUpIndex = 0;

    //���غ�����ֹͣ����ʱ
    static u16 combSwitchDown = 0;
    //���غ�������������ʱ
    static u16 combSwitchUp = 0;

    //��ʱ�Ĺ��״̬
    u16 photoTmp = 0;
    // ��ǰ�Ͱ��ι��״̬
    u16 photoCurStat = 0;
    // ��ǰ�Ͱ�������
    static u16 photoCurIndex = 0;
   // ��һ���ж��Ͱ�������
    static u16 photolastIndex = 0xFFFF;
   // ��һ�ε��Ͱ�״̬  0 ֹͣ�Ͱ�  1 �͹���
    static u16 stockRunStat = INVALUE;


    // �Ͱ������5��
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

        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //���������� ����ߵ�ƽ
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

    //�ж��������Ͱ��� �ҵ��Ͱ���ʼ���
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

    // û�����Ͱ�����
    if (user_paras_local.Belt_Number == i) {
        return;
    }


    // ��������������
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

    // ���������ؽ���
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

    // �Ͱ���ʼ������ź�
    if (photoUpStat == 0) {

        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;


        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //���������� ����ߵ�ƽ
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }

        //������������  
        if (bCombinerInInfo.input_info.input_state == 0) {
            //ֹͣ�Ͱ���Ƥ��
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
    // �Ͱ���ʼ������ź�
    // ���ݹ�������ͣ
    //һ�����һ���Ͱ�������  ֹͣ��ǰ����Ͱ�Ƥ��  
    if (photoUpStat == 1) {


        //���������� ����ߵ�ƽ
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //�����������ص������  
        if (bCombinerInInfo.input_info.input_state == 0) {

            for (i = photoCurIndex; i < user_paras_local.Belt_Number; i++)
            {
                if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) {
                    photoCurStat = logicStockgetCurPhotoState(i, 0);

                    //�ö�δ����
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

                    //���Ͱ����Ѿ����������
                    if (photoCurStat == 1) {
                        //����֮���ģ���Ƿ��Ͱ�( �����Ƿ��������� ������Ͱ� ��Ӧ��ֹ֮ͣǰ���Ͱ�ģ��
                        for (j = i + 1; j < user_paras_local.Belt_Number; j++)
                        {

                            if (((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1) == 1) {
                                // �ҵ�û������������ģ��
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

            //���Ͱ��� �����и�����Ͱ���
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

    //�������Ͱ���ʼ���״̬
    u16  photoUpStat = 0;
    //�������Ͱ���Ƥ������
    u16  ModUpIndex = 0;

    //���غ�����ֹͣ����ʱ
    static u16 combSwitchDown = 0;
    //���غ�������������ʱ
    static u16 combSwitchUp = 0;

    //��ʱ�Ĺ��״̬
    u16 photoTmp = 0;
    // ��ǰ�Ͱ��ι��״̬
    u16 photoCurStat = 0;
    // ��ǰ�Ͱ�������
    static u16 photoCurIndex = 0;
    // ��һ���ж��Ͱ�������
    static u16 photolastIndex = 0xFFFF;
    // ��һ�ε��Ͱ�״̬  0 ֹͣ�Ͱ�  1 �͹���
    static u16 stockRunStat = INVALUE;




    // �Ͱ������5��
    if (user_paras_local.Belt_Number == 0) {
        return;
    }

    if (user_paras_local.Belt_Number > 5) {
        return;
    }

    if (g_speed_gear_status == 0) {
        combSwitchUp = 0;
        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //���������� ����ߵ�ƽ
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

    //�ж��������Ͱ��� �ҵ��Ͱ���ʼ���
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

    // û�����Ͱ�����
    if (user_paras_local.Belt_Number == i) {
        return;
    }

    // ��������������
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

    // ����������ֹͣ
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

    // �Ͱ���ʼ������ź�
    if (photoUpStat == 0) {

        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;


        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //���������� ����ߵ�ƽ
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }

        //������������  
        if (bCombinerInInfo.input_info.input_state == 0) {
            //ֹͣ�Ͱ���Ƥ��
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
    // �Ͱ���ʼ������ź�    
    // ���ݹ�������ͣ
    //һ�����һ���Ͱ�������  ֹͣ��ǰ����Ͱ�Ƥ��  
    if (photoUpStat == 1) {


        //���������� ����ߵ�ƽ
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //�����������ص������  
        if (bCombinerInInfo.input_info.input_state == 0) {

            //���һ�η��غ������Ͱ�
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

            //���һ�η��غ������Ͱ�
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

            //���һ�β����غ������Ͱ�
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


//���ģʽ1
void logicInsertProcessOne(void)
{

    u16 i = 0;
    u16 j = 0;
    u16 k = 0;


    COMM_NODE_T  comm_node_new;

    //�������Ͱ���ʼ���״̬
    u16  photoUpStat = 0;
    //�������Ͱ���Ƥ������
    static u16  ModUpIndex = 0;

    //���غ�����ֹͣ����ʱ
    static u16 combSwitchDown = 0;
    //���غ�������������ʱ
    static u16 combSwitchUp = 0;

    //��ʱ�Ĺ��״̬
    u16 photoTmp = 0;
    // ��ǰ�Ͱ��ι��״̬
    u16 photoCurStat = 0;
    // ��ǰ�Ͱ�������
    static u16 photoCurIndex = 0;
    // ��һ���ж��Ͱ�������
    static u16 photolastIndex = 0xFFFF;
    // ��һ�ε��Ͱ�״̬  0 ֹͣ�Ͱ�  1 �͹���
    static u16 stockRunStat = INVALUE;


    for (j = 0; j < user_paras_local.Belt_Number; j++)
    {
        if ((((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1) == 1) ||
            (((user_paras_local.belt_para[j].Func_Select_Switch >> 7) & 0x1) == 1)) {
            break;
        }
    }

    // û�����Ͱ�����
    if (user_paras_local.Belt_Number == j) {
        return;
    }


    // �Ͱ������5��
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

        // ���ʱ���غ������ŷ���
#if INSERT_CONT
        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //���������� ����ߵ�ƽ
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

    //�ж��������Ͱ��� �ҵ��Ͱ���ʼ���
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

    // û�����Ͱ�����
    if (user_paras_local.Belt_Number == i) {
        return;
    }


    // ��������������
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

    // ���������ؽ���
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

    // �Ͱ���ʼ������ź�
    if (photoUpStat == 0) {

        photoCurIndex = ModUpIndex;
        photolastIndex = 0xFFFF;

        // ���ʱ���غ������ŷ���
#if INSERT_CONT
        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }

        //���������� ����ߵ�ƽ
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
#endif
        

        //������������  
        if (bCombinerInInfo.input_info.input_state == 0) {
            //ֹͣ�Ͱ���Ƥ��
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
    // �Ͱ���ʼ������ź�
    // ���ݹ�������ͣ
    //һ�����һ���Ͱ�������  ֹͣ��ǰ����Ͱ�Ƥ��  
    if (photoUpStat == 1) {


        // ���ʱ���غ������ŷ���
#if INSERT_CONT
        //���������� ����ߵ�ƽ
        if (combSwitchUp > 0) {
            combSwitchUp--;
            if (combSwitchUp == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_SET);
                }
            }
        }
        //ֹͣ���ƺ����� ����͵�ƽ
        if (combSwitchDown > 0) {
            combSwitchDown--;
            if (combSwitchDown == 0) {
                if (((user_paras_local.belt_para[user_paras_local.Belt_Number - 1].Func_Select_Switch >> 8) & 0x1) == 1) {
                    B_COMBINER_OUT(Bit_RESET);
                }
            }
        }
#endif
        

        //�����������ص������  
        if (bCombinerInInfo.input_info.input_state == 0) {

            for (i = photoCurIndex; i < user_paras_local.Belt_Number; i++)
            {
                if (((user_paras_local.belt_para[i].Func_Select_Switch >> 6) & 0x1) == 1) {
                    photoCurStat = logicStockgetCurPhotoState(i, 0);

                    //�ö�δ����
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

                    //���Ͱ����Ѿ����������
                    if (photoCurStat == 1) {
                        //����֮���ģ���Ƿ��Ͱ�( �����Ƿ��������� ������Ͱ� ��Ӧ��ֹ֮ͣǰ���Ͱ�ģ��
                        for (j = i + 1; j < user_paras_local.Belt_Number; j++)
                        {

                            if (((user_paras_local.belt_para[j].Func_Select_Switch >> 6) & 0x1) == 1) {
                                // �ҵ�û������������ģ��
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

            //���Ͱ��� �����и�����Ͱ���
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