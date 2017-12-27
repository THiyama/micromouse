/*
 * rx631_init.h
 *
 *  Created on: 2016/07/10
 *      Author: PCーUSER
 */

#ifndef RX631_INIT_H_
#define RX631_INIT_H_

#define ADD_COUNT_VALUE 10

void Usage_init();
void XTAL_init(void);
void IO_init(void);
void CMT_0_init(void);
void CMT_1_init(void);
void __delay_ms(int time_ms);
void __delay_us(int time_us);
void AD_init();
void MTU0_init();
void MTU4_init();
void MTU2_init();
int AD_con(int ad_ch);
void led(int pattern);
void choose_mode();


void start_switch();
void start_led();
void determine_led();

double Lipo_ad_data(int ad_data_lipo);


 /*	CMT用の外部変数	*/
extern int cmt_count;
extern int trapezoidal_count;
extern int trapezoidal_count_1;
extern int trapezoidal_count_2;
extern int trapezoidal_count_3;
extern int klothoid_count;
extern int caluculate_count;

extern int circuit_flg ;
extern char acceleration_flag;


extern int caluculate_mode;

/*	台形直進加速用の外部変数定義	*/
extern double cmt_interrupt_trapezoidal_velocity;
extern double cmt_interrupt_trapezoidal_velocity_right;
extern double cmt_interrupt_trapezoidal_velocity_left;
extern double cmt_interrupt_trapezoidal_velocity_rotate;
extern int mtu0_tgrb_interrupt_trapezoidal_tgra;
extern int mtu2_tgrb_interrupt_trapezoidal_tgra;

/*	mtu割り込み検知用フラグ	*/
extern int mtu0_tgrb_flag;
extern int mtu2_tgrb_flag;

/*	AD変換値の格納	*/
volatile extern int ad_data_before[7];
volatile extern int ad_data_after[7];
volatile extern int ad_data[7];

/*	直進、超新地検知用フラグ	*/
extern char straight_rotate_flag;

/*	現在のモーター速度	*/
extern double now_left_velocity;
extern double now_right_velocity;
extern double extern_max_velocity;

/*		*/
extern signed char now_x_coordinate;
extern signed char now_y_coordinate;


extern signed char x_direction;
extern signed char y_direction;
extern signed char buff_x_direction;
extern signed char buff_y_direction;

/*		*/
extern char sensor_led_flg;
extern char sensor_log_flg;

extern signed char mode_count;

extern unsigned char shortest_path[255];


extern int sensor_log[1001];

extern double cmt_angular_acceleration;
extern double cmt_angular_velocity;
extern double cmt_machine_angular;
#endif /* RX631_INIT_H_ */
