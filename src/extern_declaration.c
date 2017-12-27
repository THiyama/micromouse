#include "wall_control_define.h"

/*	CMT用の外部変数	*/
volatile int cmt_count;
volatile int trapezoidal_count;
volatile int trapezoidal_count_1;
volatile int trapezoidal_count_2;
volatile int trapezoidal_count_3;
volatile int klothoid_count;
volatile int caluculate_count;

volatile int caluculate_mode;

/*	台形直進加速用の外部変数定義	*/
volatile double cmt_interrupt_trapezoidal_velocity;
volatile double cmt_interrupt_trapezoidal_velocity_right;
volatile double cmt_interrupt_trapezoidal_velocity_left;
volatile double cmt_interrupt_trapezoidal_velocity_rotate;
volatile int mtu0_tgrb_interrupt_trapezoidal_tgra;
volatile int mtu2_tgrb_interrupt_trapezoidal_tgra;

/*	現在のモーター速度	*/
double now_left_velocity;
double now_right_velocity;
double extern_max_velocity=0;
double extern_max_velocity_rotate=0;

/*	mtu割り込み検知用フラグ	*/
char mtu0_tgrb_flag;
char mtu2_tgrb_flag;

/*	AD変換値の格納	*/
volatile int ad_data[7];
volatile int ad_data_before[7];
volatile int ad_data_after[7];

/*	直進、超新地検知用フラグ	*/
char straight_rotate_flag=0;
char acceleration_flag=0;

/*		*/
int goal_flag=0;


double cmt_angular_acceleration;
double cmt_angular_velocity;
double cmt_machine_angular;

/*		*/
signed char now_x_coordinate = 0;
signed char now_y_coordinate = 0;


signed char x_direction = 0;
signed char y_direction = 1;
signed char buff_x_direction = 0;
signed char buff_y_direction = 0;

/*		*/
char sensor_led_flg = 0;
char sensor_log_flg = 0;
/*		*/
char debug[10] ;

/*		*/
signed char mode_count=0;
/*		*/
unsigned char shortest_path[1000];
unsigned char renew_shortest_path[1000];
unsigned char renew_oblique_shortest_path[1000];


char map_break_flag;

int sensor_log[1001];

//制御定数
double wall_control_constant=WALL_CONTROL;
double left_forward_oblique_wall_control_constant=LEFT_FORWARD_OBLIQUE_WALL_CONTROL;
double right_forward_oblique_wall_control_constant=RIGHT_FORWARD_OBLIQUE_WALL_CONTROL;
double left_oblique_wall_control_constant=LEFT_OBLIQUE_WALL_CONTROL;
double right_oblique_wall_control_constant=RIGHT_OBLIQUE_WALL_CONTROL;

//大回り速度
double big_slalom_center_velocity = 0.75;

int slalom_mode=0;

int aaaaaa_count=0;
//
unsigned char step_map_queue[16][16];

unsigned short straight_priority_step_map_queue[16][16];

int circuit_flg=0;
/*		*/
/*		*/
/*		*/

