/*
 * motor.h
 *
 *  Created on: 2016/07/19
 *      Author: PCーUSER
 */

#ifndef MOTOR_H_
#define MOTOR_H_

void MD_init();
void MOTOR_reset();

double trapezoidal_acceleration_straight_distance(double distance, double max_velocity,
		double initial_velocity, double finish_velocity, double initial_acceleration ,double finish_acceleration);
double oblique_trapezoidal_acceleration_straight_distance(double distance, double max_velocity,
		double initial_velocity, double finish_velocity, double initial_acceleration ,double finish_acceleration);
double counter_trapezoidal_acceleration_straight_distance(double distance, double max_velocity,
		double initial_velocity, double finish_velocity, double initial_acceleration ,double finish_acceleration);

void determine_tgra(double want_velocity, double want_time);

void trapezoidal_straight(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceration);

void trapezoidal_acceleration_straight(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceration);

void trapezoidal_acceleration_right_rotate(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceration);
void trapezoidal_acceleration_left_rotate(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceration);

void trapezoidal_acceleration_l(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceration);
void trapezoidal_acceleration_r(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceration);


void small_slalom(double offset_before_distance,double offset_after_distance,double machine_velocity, double angular_acceleration ,double arc_start_angular ,double arc_end_angular,double klothoid_end_angular,signed char direction, signed char wall_out_flg,int wall_out_value);

void hoge();
void piyo();

int motor_r(double velocity);	//右モータを指定した速度、指定した距離走らせる関数
int motor_l(double velocity);	//右モータを指定した速度、指定した距離走らせる関数

void motor_act_r(double distance, double velocity);	//右モータを指定した速度、指定した距離走らせる関数
void motor_act_l(double distance, double velocity); //左モータを指定した速度、指定した距離走らせる関数

void rotate_machine(int degree, int velocity); //指定した角度を指定した速度[m/s]で旋回させる関数

void pwm0(int period, int duty);
void pwm2(int period, int duty);
void pwm4(int period, int duty);

void higashinihon_circuit(double velocity,double acceleration);

extern int cmt_count;
extern int trapezoidal_count;
extern int klothoid_count;
extern int calucurate_count;
extern int calucurate_mode;

extern double cmt_interrupt_trapezoidal_velocity;
extern double cmt_interrupt_trapezoidal_velocity_right;
extern double cmt_interrupt_trapezoidal_velocity_left;
extern double cmt_interrupt_trapezoidal_velocity_rotate;
extern int mtu0_tgrb_interrupt_trapezoidal_tgra;
extern int mtu2_tgrb_interrupt_trapezoidal_tgra;

/*	mtu割り込み検知用フラグ	*/
extern int mtu0_tgrb_flag;
extern int mtu2_tgrb_flag;

/*	直進、超新地検知用フラグ	*/
extern char straight_rotate_flag;

extern char acceleration_flag;


/*	現在のモーター速度	*/
extern double now_left_velocity;
extern double now_right_velocity;
extern double extern_max_velocity;
extern double extern_max_velocity_rotate;


extern double cmt_angular_acceleration;
extern double cmt_angular_velocity;
extern double cmt_machine_angular;

extern double slalom_mode ;

//大回り速度
extern double big_slalom_center_velocity;
#endif /* MOTOR_H_ */
