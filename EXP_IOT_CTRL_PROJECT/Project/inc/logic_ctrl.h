#ifndef _LOGIC_H
#define _LOGIC_H

#define  B_START_IN_STATE              IN1_0_STATE   //启动信号
#define  B_STOP_IN_STATE               IN1_1_STATE   //停止信号
#define  B_EMERGENCY_IN_STATE          IN1_2_STATE   //急停信号
#define  B_EMERGENCYRESET_IN_STATE     IN1_3_STATE   //急停复位信号
#define  B_DOWN_STOCK_IN_STATE         IN1_4_STATE   //下游屯包光电（主要对接合流机屯包段）
#define  B_UP_STOCK_IN_STATE           IN1_5_STATE   //上游屯包光电（主要对接上游非IOT控制的屯包段）

#define  B_COMBINER_IN_STATE           IN2_6_STATE   //合流机允许输入信号


#define  B_PHOTO_1_IN_STATE    IN2_0_STATE   //光电1输入
#define  B_PHOTO_2_IN_STATE    IN2_1_STATE   //光电2输入
#define  B_PHOTO_3_IN_STATE    IN2_2_STATE   //光电3输入
#define  B_PHOTO_4_IN_STATE    IN2_3_STATE   //光电4输入
#define  B_PHOTO_5_IN_STATE    IN2_4_STATE   //光电5输入
#define  B_RESET_IN_STATE      IN2_5_STATE   //复位按钮输入
#define  B_STREAM_IN_STATE     IN2_6_STATE   //下游准入信号输入

#define  B_RUN_1_OUT_(BitVal)   OUT_1_0_(BitVal)   //启动输出1
#define  B_RUN_2_OUT_(BitVal)   OUT_1_1_(BitVal)   //启动输出2
#define  B_RUN_3_OUT_(BitVal)   OUT_1_2_(BitVal)   //启动输出3
#define  B_RUN_4_OUT_(BitVal)   OUT_1_3_(BitVal)   //启动输出4
#define  B_RUN_5_OUT_(BitVal)   OUT_1_4_(BitVal)   //启动输出5
#define  B_STREAM_OUT_(BitVal)  OUT_1_5_(BitVal)   //联动输出  (如果屯包 低电平不允许屯包 高电平允许屯包)
#define  L_ALARM_OUT_(BitVal)   OUT_1_6_(BitVal)   //报警指示灯输出
#define  B_RUN_STA_OUT_(BitVal) OUT_1_7_(BitVal)   //报警指示灯输出
#define  B_COMBINER_OUT(BitVal) OUT_1_7_(BitVal)   //运行状态  (如果屯包 高电平控制下游运行 低电平不控制）

// 屯包时改变此点位的定义
#if 0
#define  B_RUN_STA_OUT_(BitVal) OUT_1_7_(BitVal)   //整线启动状态输出  
#else
#endif

#define  INPUT_TRIG_NULL    0
#define  INPUT_TRIG_UP      1
#define  INPUT_TRIG_DOWN    2

#define  BELT_NUMMAX        10
#define  STOPSTATUS_MAX     2305

#define  ONLY_IO            0
#define  STACK_TYPE         1

typedef struct {
    u8  input_state;
    u8  input_middle_state;
    u8  input_confirm_times;
    u8  input_trig_mode; //null,up,down
}sInput_Info;

typedef struct {
    sInput_Info  input_info;
    u16          button_state; //按钮有没有被触发
    u16          button_hold_time; //按钮按住的时间计时(ms)
    u16          trig_cnt;
    u16          blocktrig_flag;       //触发过 用在堵包判断中
}sButton_Info;

#define USER_PARA_DATA_LEN    143

typedef struct {
    u16 Func_Select_Switch;    //功能选择开关(bit0:是否启用自动调速;bit1:前后联动功能;bit2:堵包检测功能;bit3:积放功能(使用光电) bit4(有外部准入信号)) bit5 上游同步停止
                               //bit6:屯包  bit7:与下游同步屯包  bit8:控制合流机屯包 bit9 暂停 bit10 共用设备
    u16 Gear_1_Speed_Freq;     //第一档速度/频率(速度整型表示浮点数,如:12表示1.2m/s)(频率值整型表示浮点数,如:1234表示12.34hz)
    u16 Gear_2_Speed_Freq;     //第二档速度/频率
    u16 Gear_3_Speed_Freq;     //第三档速度/频率
    u16 Gear_4_Speed_Freq;     //第四档速度/频率
    u16 Gear_5_Speed_Freq;     //第五档速度/频率
    u16 Sleep_Mode_Speed_Freq; //休眠模式速度/频率
    u16 Start_Delay_Time;      //启动延时时间(ms)
    u16 Stop_Delay_Time;       //停止延时时间(ms)
    u16 Block_Check_Time;      //堵包检测时间(s)
    u16 Sleep_Check_Time;      //休眠检测时间(s)
    // add 24/01/04
    u16 Stop_Clear_Time;       // 停机清包时间
    u16 Link_Start_Time;       // 联动启动时间
    u16 Link_Stop_Time;        // 联动停止时间
}BELT_PARAS_T;

typedef struct {
    u16 Version_No_L;       //模块版本号(低字)
    u16 Version_No_H;       //模块版本号(高字)
    u16 Station_No;         //模块站号
    u16 Up_Stream_No;       //上游站号
    u16 Down_Stream_No;     //下游站号
    u16 Belt_Number;        //皮带机数量(装车线最大5,卸车线最大10)
    BELT_PARAS_T belt_para[10];
}USER_PARAS_T;

typedef struct {
    u16 fault_code;       //bit15~8(变频器故障码),bit5(急停状态),bit4:(运行状态),bit3(编码器状态),bit2(调速完成状态),bit1(堵包故障),bit0(485通讯故障)
    u16 input_status;     //bit7(光电一是否触发过) bit6(光电二),bit5(光电一),bit4~0(变频器DI输入状态(bit0:启动信号bit1:远程信号bit4:编码器输入))
    u16 line_speed;       //线速度实时反馈
    u16 electric_current; //变频器实时电流
    u16 inverter_freq;    //变频器输出频率
}INVERTER_STATUS_T;

typedef struct {
    u16 station_no;
    u16 belt_number;
    INVERTER_STATUS_T inverter_status[10];
}MODULE_STATUS_T;

typedef struct {
    MODULE_STATUS_T *queue; /* 指向存储队列的数组空间 */
    u16 front, rear, len; /* 队首指针（下标），队尾指针（下标），队列长度变量 */
    u16 maxSize; /* queue数组长度 */
}MODULE_STATUS_QUEUE;

typedef struct {
    u16 rw_flag;         //读写标记
    u16 inverter_no;     //变频器站号
    u16 speed_gear;      //写入速度的档位
    u16 comm_interval;   //通讯间隔时间
    u16 comm_retry;      //通讯重试次数
    u16 value;           //是否有效
}COMM_NODE_T;

typedef struct {
    COMM_NODE_T *queue; /* 指向存储队列的数组空间 */
    u16 front, rear, len; /* 队首指针（下标），队尾指针（下标），队列长度变量 */
    u16 maxSize; /* queue数组长度 */
}COMM_SEND_QUEUE;

extern sButton_Info  bModeInfo;
extern sButton_Info  bStartInfo;
extern sButton_Info  bHSpeedInfo;
extern sButton_Info  bLSpeedInfo;
extern sButton_Info  bPauseInfo;
extern sButton_Info  bStreamInfo;
extern sButton_Info  bEmergencyInfo;

//add 23/12/12
//急停复位信号
extern sButton_Info  bEmergencyResetInfo;
//上游屯包光电
extern sButton_Info  bUpStockInfo;
//下游屯包光电
extern sButton_Info  bDownStockInfo;
//合流机控制信号
extern sButton_Info  bCombinerInInfo;

extern u8  g_remote_start_flag;
extern u8  g_remote_start_status;
extern u8  g_remote_speed_status;
extern u8  g_link_up_stream_status;
extern u8  g_link_down_stream_status;
extern u8  g_set_start_status;
extern u8  g_set_speed_status;
extern u8  g_read_start_status;
extern u8  g_block_disable_flag;
extern u8  g_speed_gear_status; //当前的速度档位(0:停止 1~5:五档速度)

extern u8  g_link_down_phototrig_status;      //下游的光电是否触发过

extern u16 start_delay_time_cnt;
extern u16 stop_delay_time_cnt;

extern u16 reset_start_time_cnt;
extern u8  reset_start_flag;

// can 接收到的急停信号
extern u8  g_emergency_stop;

extern USER_PARAS_T  user_paras_local;
extern USER_PARAS_T  user_paras_slave;
extern USER_PARAS_T  user_paras_temp;

extern MODULE_STATUS_T  module_status_buffer[];
extern INVERTER_STATUS_T  inverter_status_buffer[];

extern u16 logic_upload_stopStatus[];
extern u16 logic_upload_lastRunStatus[];

extern COMM_NODE_T comm_node;
extern u8  comm_busy_flag;
extern u8  polling_num;

extern u16 stopspeed_default[];
extern u16 freq_check_cnt;

void read_user_paras(void);
void write_user_paras(u16* para);
void InputScanProc();
void Speed_Ctrl_Process(u8 speed_gear);
void Linkage_Stop_Photo_Ctrl_Handle(u8 photo_index);
void Linkage_Stream_Ctrl_Handle(u8 inverter_index,u8 start_flag);
void Block_Check_Ctrl_Handle(void);
void Reset_Ctrl_Handle(void);
void Reset_Start_Inverter_Handle(void);

void InitUartSendQueue(void);
void AddUartSendData2Queue(COMM_NODE_T x);
COMM_NODE_T* GetUartSendDataFromQueue(void);
u8 IsUartSendQueueFree(void);
void InitModuleStatusQueue(void);
void AddModuleStatusData2Queue(MODULE_STATUS_T x);
MODULE_STATUS_T* GetModuleStatusDataFromQueue(void);
u8 IsModuleStatusQueueFree(void);
void Linkage_stream_extra_signal(u8 inverter_index, u8 start_flag);
void logic_upstream_io_allow_output(void);

u8 get_inverter_fault_status(INVERTER_STATUS_T inverter_status);

void logic_uarttmp_init(void);
void logic_cycle_decrease(void);


// 屯包过程
void logicStockProcess(void);
void logicStockProcessTwo(void);

#endif