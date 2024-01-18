//#include "stm32f10x.h"
//#include "stm32f107.h"
#include "main.h"

u8  uart1_send_buff[UART1_BUFF_SIZE];
u8  uart1_recv_buff[UART1_BUFF_SIZE];
u16 uart1_send_count;
u16 uart1_recv_count;
u8  uart1_commu_state = SEND_READY;
u16 uart1_tmr = 0;
u8  uart1_recv_retry;
u8  g_speed_set_sta;

u16 stopspeed_default[10] = {0};
u16 freq_check_cnt = 0;

/////////////////////////  CRC   /////////////////////////
const u8 CRCHi[ ] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
};
const u8 CRCLo[ ] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
};
u16 CRC_sub( u8 *head, u8 *tail, u16 InitVal, u8 exchange )
{
    u8	CRC_Hi;
    u8	CRC_Lo;
    u8	i;
    u16 result;
    CRC_Hi = InitVal >> 8;
    CRC_Lo = InitVal;

    while( head < tail )
    {
        i = CRC_Hi ^ *head++;
        CRC_Hi = CRC_Lo ^ CRCHi[ i ];
        CRC_Lo = CRCLo[ i ];
    }

    if( ! exchange )
    {
        result = 0;
        result = CRC_Hi;
        result = result << 8;
        result |= CRC_Lo;
    }
    else
    {
        result = 0;
        result = CRC_Lo;
        result = result << 8;
        result |= CRC_Hi;
    }
    return(result);
}
u16 get_CRC( u8 *head, u16 len )
{
    return( CRC_sub( head, head + len, 0xFFFF, 0 ) );
}
////////////////////////////////////////////////////////////

//1ms定时
void uart_recv_timeout(void)
{
    if(uart1_tmr > 0)
    {
        uart1_tmr--;
        if(uart1_tmr == 0)
        {
            uart1_commu_state = RECV_DATA_END;
        }
    }

    if(comm_node.comm_interval > 0)
    {
        comm_node.comm_interval--;
        if(comm_node.comm_interval == 0)
        {
            uart1_commu_state = SEND_DATA;
        }
    }
}

void uart1_send(void)
{
    u16 i;

    UART1_TX_485;

    USART_GetFlagStatus(USART1, USART_FLAG_TC);
    for(i = 0 ; i < uart1_send_count ; i++)
    {
        USART_SendData(USART1, uart1_send_buff[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
    UART1_RX_485;
    uart1_commu_state = RECV_DATA;
    uart1_recv_count = 0;
#if ONLY_IO
    uart1_tmr = 2;
#else
    uart1_tmr = 35;
#endif
}

void Modbus_RTU_Write_Single_Reg_Cmd(u16 reg_addr,u16 data)
{
    u16 crc_Cal;
    
    uart1_send_buff[0] = 0x01;
    uart1_send_buff[1] = 0x06;
    uart1_send_buff[2] = (reg_addr>>8)&0xFF;
    uart1_send_buff[3] = reg_addr&0xFF;
    uart1_send_buff[4] = (data>>8)&0xFF;
    uart1_send_buff[5] = data&0xFF;
    crc_Cal = get_CRC(uart1_send_buff, 6);
    uart1_send_buff[6] = (crc_Cal>>8)&0XFF;
    uart1_send_buff[7] = crc_Cal&0xFF;
    
    uart1_send_count = 8;
    uart1_send();
}

void Modbus_RTU_Write_Multi_Reg_Cmd(u16 station_no,u16 reg_addr,u16 reg_num,u16* data)
{
    u16 crc_Cal;
    u8 i;
    
    uart1_send_buff[0] = station_no;
    uart1_send_buff[1] = 0x10;
    uart1_send_buff[2] = (reg_addr>>8)&0xFF;
    uart1_send_buff[3] = reg_addr&0xFF;
    uart1_send_buff[4] = (reg_num>>8)&0xFF;
    uart1_send_buff[5] = reg_num&0xFF;
    uart1_send_buff[6] = reg_num*2;
    for(i=0; i<reg_num; i++)
    {
        uart1_send_buff[7+i*2] = (data[i]>>8)&0xFF;
        uart1_send_buff[8+i*2] = data[i]&0xFF;
    }
    crc_Cal = get_CRC(uart1_send_buff, 7+i*2);
    uart1_send_buff[7+i*2] = (crc_Cal>>8)&0XFF;
    uart1_send_buff[8+i*2] = crc_Cal&0xFF;
    
    uart1_send_count = 9+i*2;
    uart1_send();
}
void Modbus_RTU_Read_Reg_Cmd(u16 station_no,u16 reg_addr,u16 reg_num)
{
    u16 crc_Cal;
    
    uart1_send_buff[0] = station_no;
    uart1_send_buff[1] = 0x03;
    uart1_send_buff[2] = (reg_addr>>8)&0xFF;
    uart1_send_buff[3] = reg_addr&0xFF;
    uart1_send_buff[4] = (reg_num>>8)&0xFF;
    uart1_send_buff[5] = reg_num&0xFF;
    crc_Cal = get_CRC(uart1_send_buff, 6);
    uart1_send_buff[6] = (crc_Cal>>8)&0XFF;
    uart1_send_buff[7] = crc_Cal&0xFF;
    
    uart1_send_count = 8;
    uart1_send();
}
void comm_send_rw_comand()
{
    u16 write_data[4];
    BELT_PARAS_T belt_para_t;
    
    if(comm_node.rw_flag == 0)//读
    {
        Modbus_RTU_Read_Reg_Cmd(comm_node.inverter_no,0x7200,7);
    }
    else if(comm_node.rw_flag == 1)//写速度
    {
        belt_para_t = user_paras_local.belt_para[comm_node.inverter_no-1];

        //处理急停信号
        if ((bEmergencyInfo.input_info.input_state == 1) || (g_emergency_stop == 1)) {
            if (belt_para_t.Func_Select_Switch & 0x1)
            {
                write_data[0] = 5;//停止
                write_data[1] = stopspeed_default[comm_node.inverter_no - 1];
                write_data[2] = 0;

            }
            else
            {
                write_data[0] = 5;//停止
                write_data[1] = 0;
                write_data[2] = stopspeed_default[comm_node.inverter_no - 1];
            }
        }
        else {
            if (comm_node.speed_gear == 0)
            {
                //write_data[0] = 5;//停止
                //write_data[1] = 0;
                //write_data[2] = 0;

                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 5;//停止
                    write_data[1] = stopspeed_default[comm_node.inverter_no - 1];
                    write_data[2] = 0;

                }
                else
                {
                    write_data[0] = 5;//停止
                    write_data[1] = 0;
                    write_data[2] = stopspeed_default[comm_node.inverter_no - 1];
                }
            }
            else if (comm_node.speed_gear == 1)
            {
                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = belt_para_t.Gear_1_Speed_Freq;
                    write_data[2] = 0;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_1_Speed_Freq;
                }
                else
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = 0;
                    write_data[2] = belt_para_t.Gear_1_Speed_Freq;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_1_Speed_Freq;
                }
            }
            else if (comm_node.speed_gear == 2)
            {
                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = belt_para_t.Gear_2_Speed_Freq;
                    write_data[2] = 0;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_2_Speed_Freq;
                }
                else
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = 0;
                    write_data[2] = belt_para_t.Gear_2_Speed_Freq;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_2_Speed_Freq;
                }
            }
            else if (comm_node.speed_gear == 3)
            {
                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = belt_para_t.Gear_3_Speed_Freq;
                    write_data[2] = 0;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_3_Speed_Freq;
                }
                else
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = 0;
                    write_data[2] = belt_para_t.Gear_3_Speed_Freq;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_3_Speed_Freq;
                }
            }
            else if (comm_node.speed_gear == 4)
            {
                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = belt_para_t.Gear_4_Speed_Freq;
                    write_data[2] = 0;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_4_Speed_Freq;
                }
                else
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = 0;
                    write_data[2] = belt_para_t.Gear_4_Speed_Freq;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_4_Speed_Freq;
                }
            }
            else if (comm_node.speed_gear == 5)
            {
                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = belt_para_t.Gear_5_Speed_Freq;
                    write_data[2] = 0;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_5_Speed_Freq;
                }
                else
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = 0;
                    write_data[2] = belt_para_t.Gear_5_Speed_Freq;
                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Gear_5_Speed_Freq;
                }
            }
            else if (comm_node.speed_gear == 6)
            {
                if (belt_para_t.Func_Select_Switch & 0x1)
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = belt_para_t.Sleep_Mode_Speed_Freq;
                    write_data[2] = 0;
//                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Sleep_Mode_Speed_Freq;
                }
                else
                {
                    write_data[0] = 1;//正转运行
                    write_data[1] = 0;
                    write_data[2] = belt_para_t.Sleep_Mode_Speed_Freq;
//                    stopspeed_default[comm_node.inverter_no - 1] = belt_para_t.Sleep_Mode_Speed_Freq;
                }
            }
        }
        
        write_data[3] = belt_para_t.Func_Select_Switch&0x1;
        Modbus_RTU_Write_Multi_Reg_Cmd(comm_node.inverter_no,0x7000,4,(u16*)&write_data);

        if ((bEmergencyInfo.input_info.input_state == 1) || (comm_node.speed_gear == 0) || (g_emergency_stop == 1)) {
            g_speed_set_sta = 0;
        }
        else {
            if (belt_para_t.Func_Select_Switch & 0x1)
            {
                if (write_data[1] == 0)
                {
                    g_speed_set_sta = 0;
                }
                else
                {
                    g_speed_set_sta = 1;
                }
            }
            else
            {
                if (write_data[2] == 0)
                {
                    g_speed_set_sta = 0;
                }
                else
                {
                    g_speed_set_sta = 1;
                }
            }
        }

        if (comm_node.inverter_no == 1)
        {
            if (g_speed_set_sta == 1)
                B_RUN_1_OUT_(Bit_SET);
            else
                B_RUN_1_OUT_(Bit_RESET);
        }
        else if (comm_node.inverter_no == 2)
        {
            if (g_speed_set_sta == 1)
                B_RUN_2_OUT_(Bit_SET);
            else
                B_RUN_2_OUT_(Bit_RESET);
        }
        else if (comm_node.inverter_no == 3)
        {
            if (g_speed_set_sta == 1)
                B_RUN_3_OUT_(Bit_SET);
            else
                B_RUN_3_OUT_(Bit_RESET);
        }
        else if (comm_node.inverter_no == 4)
        {
            if (g_speed_set_sta == 1)
                B_RUN_4_OUT_(Bit_SET);
            else
                B_RUN_4_OUT_(Bit_RESET);
        }
        else if (comm_node.inverter_no == 5)
        {
            if (g_speed_set_sta == 1)
                B_RUN_5_OUT_(Bit_SET);
            else
                B_RUN_5_OUT_(Bit_RESET);
        }
        
    }
    else if(comm_node.rw_flag == 2)//写复位
    {
        write_data[0] = 0x8;//保护复位
        Modbus_RTU_Write_Multi_Reg_Cmd(comm_node.inverter_no,0x7000,1,(u16*)&write_data);
    }
}
u8 comm_recv_rw_comand(u8* buf,u16 len)
{
    u16 crc_Rev,crc_Cal;
    u16 data[7];
    u8  i;
    u8  run_status;
    u8  run_status_new;
    COMM_NODE_T  comm_node_new;
    
    if(len < 4)
        return 0;
    crc_Cal = get_CRC(buf, len-2);
    crc_Rev = (buf[len-2]<<8)|buf[len-1];
    if(crc_Cal != crc_Rev)//CRC交验错误
        return 0;
    if(buf[0] != comm_node.inverter_no)//站号不同
        return 0;
    if(comm_node.rw_flag == 0)//读
    {
        if(buf[1] != 0x03)//功能码错误
            return 0;
        for(i=0; i<7; i++)
        {
            data[i] = buf[3+i*2]<<8 | buf[4+i*2];
        }
        
        run_status = (inverter_status_buffer[comm_node.inverter_no-1].fault_code>>4)&0x1;//当前运行状态
        run_status_new = (((data[0]&0xFF)==0x1)?1:0);
        if(run_status != run_status_new)//运行状态改变
        {
            if(comm_node.inverter_no > 1)//控制上游皮带起停
            {
                if(run_status == 1)
                {
                    Linkage_Stream_Ctrl_Handle(comm_node.inverter_no-2,0);
                }
                else
                {
                    Linkage_Stream_Ctrl_Handle(comm_node.inverter_no-2,1);
                }
            }
        }
        
        inverter_status_buffer[comm_node.inverter_no-1].fault_code &= (0x22);//保留堵包故障位  清掉其他的报警 急停状态不应该清掉
        inverter_status_buffer[comm_node.inverter_no-1].fault_code |= (data[0]&0xFF00);//变频器故障码
        inverter_status_buffer[comm_node.inverter_no-1].fault_code |= (run_status_new<<4);//运行状态
        inverter_status_buffer[comm_node.inverter_no-1].fault_code |= ((data[1]&0x1)<<3);//编码器状态
        inverter_status_buffer[comm_node.inverter_no-1].fault_code |= ((data[5]&0x1)<<2);//调速完成状态
        //本地切换成远程 恢复原来的状态
        if ((inverter_status_buffer[comm_node.inverter_no - 1].input_status & 0x02) != (data[6] & 0x02)) {
            if ((data[6] & 0x02) == 1) {
                comm_node_new.rw_flag = 1;
                comm_node_new.inverter_no = comm_node.inverter_no;
                comm_node_new.speed_gear = logic_upload_lastRunStatus[comm_node.inverter_no - 1];
                comm_node_new.comm_interval = 1;
                comm_node_new.comm_retry = 3;
                AddUartSendData2Queue(comm_node_new);
            }
        }
        inverter_status_buffer[comm_node.inverter_no-1].input_status |= (data[6]&0x1F);
        inverter_status_buffer[comm_node.inverter_no-1].line_speed = data[2];
        inverter_status_buffer[comm_node.inverter_no-1].electric_current = data[3];
        inverter_status_buffer[comm_node.inverter_no-1].inverter_freq = data[4];
        inverter_status_buffer[comm_node.inverter_no - 1].fault_code |= (g_emergency_stop << 5);//急停状态
    }
    else//写
    {
        if(buf[1] != 0x10)//功能码错误
        {
            return 0;
        }
        else//速度写入成功
        {
            if(buf[0] == 1)
            {
                if(g_speed_set_sta == 1)
                    B_RUN_1_OUT_(Bit_SET);
                else
                    B_RUN_1_OUT_(Bit_RESET);
            }
            else if(buf[0] == 2)
            {
                if(g_speed_set_sta == 1)
                    B_RUN_2_OUT_(Bit_SET);
                else
                    B_RUN_2_OUT_(Bit_RESET);
            }
            else if(buf[0] == 3)
            {
                if(g_speed_set_sta == 1)
                    B_RUN_3_OUT_(Bit_SET);
                else
                    B_RUN_3_OUT_(Bit_RESET);
            }
            else if(buf[0] == 4)
            {
                if(g_speed_set_sta == 1)
                    B_RUN_4_OUT_(Bit_SET);
                else
                    B_RUN_4_OUT_(Bit_RESET);
            }
            else if(buf[0] == 5)
            {
                if(g_speed_set_sta == 1)
                    B_RUN_5_OUT_(Bit_SET);
                else
                    B_RUN_5_OUT_(Bit_RESET);
            }
        }
    }
    
    return 1;
}
void Modbus_RTU_Comm_Process(void)
{
    COMM_NODE_T* comm_node_tmp;
    u16 i = 0;
    u16 flag = 0;
    
    if(comm_busy_flag == 0)
    {
        comm_node_tmp = GetUartSendDataFromQueue();
        if(comm_node_tmp == NULL)
        {
            return;
        }
        else
        {
            comm_node = *comm_node_tmp;

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
                if ((logic_upload_stopStatus[comm_node.inverter_no - 1] != 0) && (comm_node.speed_gear != 0) && (comm_node.rw_flag == 1)) {
                    return;
                }
                if ((logic_upload_stopStatus[comm_node.inverter_no - 1] == 0) && (comm_node.speed_gear == 0) && (comm_node.rw_flag == 1)) {
                    logic_upload_stopStatus[comm_node.inverter_no - 1] = STOPSTATUS_MAX;
                }
            }
            
            comm_node.comm_interval = 2;

            logic_upload_lastRunStatus[comm_node.inverter_no - 1] = comm_node.speed_gear;

            comm_busy_flag = 1;
            uart1_commu_state = SEND_READY;
#if ONLY_IO
            comm_node.comm_retry = 0;
#else
            comm_node.comm_retry = 2;
#endif

        }
    }
    else
    {
        if(uart1_commu_state == SEND_DATA)
        {

            comm_send_rw_comand();
        }
        else if(uart1_commu_state == RECV_DATA_END)
        {
            if(comm_recv_rw_comand(uart1_recv_buff,uart1_recv_count))
            {
                //清除485故障
                inverter_status_buffer[comm_node.inverter_no-1].fault_code &= ~0x1;
                comm_busy_flag = 0;
            }
            else
            {
                if(comm_node.comm_retry > 0)
                {
                    comm_node.comm_retry--;
                    comm_node.comm_interval = 2;
                    uart1_commu_state = SEND_READY;
                }
                else
                {
                    inverter_status_buffer[comm_node.inverter_no-1].fault_code |= 0x1;//485通讯故障
                    comm_busy_flag = 0;
                }
            }
            uart1_recv_count = 0;
        }
    }
}
