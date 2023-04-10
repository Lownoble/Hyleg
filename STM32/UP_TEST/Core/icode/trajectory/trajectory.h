#define PI M_PI
#define INIT_ANGLE1 45.0/180*PI
#define INIT_ANGLE2 45.0/180*PI


extern int tra_cnt;
extern int stand_flag;
extern int swing_flag;
extern float stand_trajectory[100][2];
extern float swing_trajectory[100][2];


float* FK(float theta1,float theta2);
float* IK(float x, float y);
void trajectory(float H, float FH, float LF, float LB);
