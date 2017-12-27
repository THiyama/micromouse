/*
 * wall_control.h
 *
 *  Created on: 2016/09/06
 *      Author: PCーUSER
 */

#ifndef WALL_CONTROL_H_
#define WALL_CONTROL_H_

//壁制御計算
float wall_control();
float oblique_wall_control(int dir);
char hihumint_dead_wall();

void wall_out(double, int, signed char);

void wall_log();

volatile extern int ad_data[7];
volatile extern char sensor_log_flg;
//大回り速度
extern double big_slalom_center_velocity;

extern char map_break_flag;

//制御定数
extern double wall_control_constant;
extern double left_forward_oblique_wall_control_constant;
extern double right_forward_oblique_wall_control_constant;
extern double left_oblique_wall_control_constant;
extern double right_oblique_wall_control_constant;

#endif /* WALL_CONTROL_H_ */
