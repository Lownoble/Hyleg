#define PI M_PI
#define INIT_ANGLE_L1 45.0/180*PI
#define INIT_ANGLE_L2 45.0/180*PI
#define MAX_ANGLE_L1 1.08
#define MIN_ANGLE_L1 -0.56
#define MAX_ANGLE_L2 1.61
#define MIN_ANGLE_L2 0.83

#define INIT_ANGLE_R1 45.0/180*PI
#define INIT_ANGLE_R2 45.0/180*PI
#define MAX_ANGLE_R1 2.12
#define MIN_ANGLE_R1 0.49
#define MAX_ANGLE_R2 0.74
#define MIN_ANGLE_R2 -0.10

#define V_LIMIT 0.06
#define BOTH_RATIO 1.2

#define THETA1_1 0.7227
#define THETA1_2 0.8481
#define THETA2_1 1.0472
#define THETA2_2 0.5236
#define THETA3_1 0.0
#define THETA3_2 0.0
#define THETA4_1 0.0
#define THETA4_2 0.7854

extern int tra_cnt;
extern int stand_flag;
extern int swing_flag;
extern float stand_trajectory[100][2];
extern float swing_trajectory[100][2];

extern int test_cnt;
extern float test_trajectory[10];


float* FK(float theta1,float theta2);
float* IK(float x, float y);
void trajectory(float H, float FH, float LF, float LB);
void trajectory_left(float H, float FH, float LF, float LB);
void trajectory_right(float H, float FH, float LF, float LB);
void trajectory_circle(float H, float R);
void trajectory_square(float H, float square_length, float square_width);
void trajectory_horizontal(float H, float length);
void trajectory_vertiacal(float H, float width);
void trajectory_offset();
void trajectory_walk();
void change_H();
