#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_
//#include "base_struct.h"
#define SPI_BUF_SIZE 256 
#define END_POS float

typedef struct
{
float att[3];
float att_rate[3];
float acc_b[3];
float q[4][3];
float dq[4][3];
float tau[4][3];
float bat_v[4];
char connect[4];
char connect_motor[4][2];
char ready[4][2];
}_SPI_RX;

extern _SPI_RX spi_rx;


typedef struct
{
float q_set[4][3];
float q_reset[4][3];
float tau_ff[4][3];
float t_to_i;
float max_i;
float kp,ki,kd;
char en_motor,reser_q,reset_err;
}_SPI_TX;

extern _SPI_TX spi_tx;

//----------------------------------
typedef struct
{
char en_record;
char is_touch[4],is_ground[4];
char force_en_flag[4];
char leg_state[4];
END_POS epos_n_tar[4];	
END_POS epos_n_now[4];
END_POS depos_n_tar[4];	
END_POS depos_n_now[4];
float sita_tar[4][3];
float sita_now[4][3];	
float tau_tar[4][3];
float tau_now[4][3];	
END_POS GRF_n_tar[4];	
END_POS GRF_n_now[4];	
float com_n_tar[3];	
float com_n_now[3];	
float dcom_n_tar[3];	
float dcom_n_now[3];
float ground_att_now[3];	
float att_now[3];
float datt_now[3];
float att_tar[3];
float datt_tar[3];
float temp_record[10];
}_NAV_RX;

extern _NAV_RX nav_rx;


typedef struct
{
	char connect;
	char ocu_mode;
	float rc_spd_b[3],rc_rate_b[3];
	int key_ud,key_lr,key_x,key_y,key_a,key_b,key_ll,key_rr,key_st,key_back;   
//SDK
	char request_gait;
	float exp_spd_o[3];
	float exp_att_o[3];
	float exp_pos_o[3];
	float exp_q_o[4][3];
	END_POS exp_GRF_o[4];

}_NAV_TX;

extern _NAV_TX nav_tx;

#endif