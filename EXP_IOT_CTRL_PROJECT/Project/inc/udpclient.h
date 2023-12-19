#ifndef __TCPCLIENT_H__
#define __TCPCLIENT_H__

#define	TCP_CLIENT_MAX_NUM	1

struct tcp_client_table
{
    u8  tcp_recv_buf[UDP_RECV_BUFF_SIZE];
    u8  tcp_send_buf[UDP_SEND_BUFF_SIZE];
    u8  tcp_send_en;
    u16 tcp_recv_len;
    u16 tcp_send_len;
    u8  tcp_client_statue;
    u16 tcp_client_Delay_reg;
    struct udp_pcb *echoclient_pcb;
    u8  dest_ip[4];
    u16 dest_port;
    u8  connect_is_ok;
    u8  connect_dely;
};
extern struct tcp_client_table	tcp_client_list[];
extern void udp_send_process(u8 udp_offset);
extern void udp_echoclient_connect(u8 udp_offset);
extern void udp_client_process(void);
//extern void Udp_Recv_BroadCast_Init(void);

#endif 