#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_
#define END_POS float

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