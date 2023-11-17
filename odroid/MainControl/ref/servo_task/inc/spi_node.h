#ifndef _SPI_NODE_H_
#define _SPI_NODE_H_

#define SPI_BUF_SIZE 256 

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

#endif