/*
 * motor_define.h
 *
 *  Created on: 2016/07/22
 *      Author: PCーUSER
 */

#ifndef MOTOR_DEFINE_H_
#define MOTOR_DEFINE_H_

//物理パラメータ
#define ANGLE_ROTATION 1.8				//PKE243Aのステップ角
#define PATTERN_EXCITATION 2.0			//2相励磁を使用

//距離を小さくするには、タイヤ径を大きく
//距離を大きくするには、タイヤ径を小さく
//#define LEFT_DIAMETER_TIRE 0.04769				//タイヤの直径
//#define RIGHT_DIAMETER_TIRE 0.04769				//タイヤの直径
#define LEFT_DIAMETER_TIRE 0.05025				//タイヤの直径
#define RIGHT_DIAMETER_TIRE 0.05025				//タイヤの直径
//左に曲がる時は

#define MY_PI 3.14159265				//円周率
#define ONE_REVOLUTION 360.0			//1回転

#define MTU_COUNT_FREQUENCY 50000000.0/4.0	//TCNT1カウントに要する周波数

#define TREAD_WIDTH 83.8
#define TREAD_WIDTH_LEFT	82.04					//トレッド幅（暫定）
//#define TREAD_WIDTH_RIGHT	83.5000					//トレッド幅（暫定）
#define TREAD_WIDTH_RIGHT	82.04					//トレッド幅（暫定）

#define STEP_LENGTH	(TREAD_WIDTH * MY_PI * (ANGLE_ROTATION/PATTERN_EXCITATION) / ONE_REVOLUTION)
//↑　1ステップで進む距離　↑

//迷路情報
#define BLOCK_LENGTH 0.180	//1区画180mm

//PWMの停止
#define MTU_STOP MTU.TSTR.BIT.CST0 = 0;MTU.TSTR.BIT.CST2 = 0;straight_rotate_flag = 0

//小回り左回り
#define small_left_parameter_1 0.00291,0.0355096,0.45,70.000000,20.000000,70.000000,100.20,3,0,400

//小回り右回り
#define small_right_parameter_1 0.00560005,0.0388,0.45,70.000000,20.000000,70.000000,99.480100000000,1,0,400

//大回り90°左回り
//重心速度0.75パラメータ
#define big_left_parameter_1 0.0267,0.103601,0.75,55.000000,31.000000,57.000000,87.7000000,3,1,83
#define big_left_parameter_1_cut_after 0.0267,0.0,0.75,55.000000,31.000000,57.000000,87.70000000,3,1,83
//重心速度0.80パラメータ
#define big_left_parameter_2 0.0167,0.103601,0.8,55.000000,31.000000,57.000000,87.7000000,3,1,83
#define big_left_parameter_2_cut_after 0.0167,0.0,0.8,55.000000,31.000000,57.000000,87.7000000,3,1,83
//重心速度0.85パラメータ
#define big_left_parameter_3 0.0167,0.103601,0.85,55.000000,31.000000,57.000000,87.7000000,3,1,83
#define big_left_parameter_3_cut_after 0.0167,0.0,0.85,55.000000,31.000000,57.000000,87.7000000,3,1,83


//大回り90°右回り
//重心速度0.75パラメータ
#define big_right_parameter_1 0.02035,0.10871,0.75,55.000000,33.000000,57.000000,88.000000,1,1,117
#define big_right_parameter_1_cut_after 0.02035,0.0,0.75,55.000000,33.000000,57.000000,88.000000,1,1,117
//重心速度0.80パラメータ
#define big_right_parameter_2 0.01035,0.10871,0.80,55.000000,33.000000,57.000000,88.0000,1,1,117
#define big_right_parameter_2_cut_after 0.01035,0.0,0.80,55.000000,33.000000,57.000000,88.00000,1,1,117
//重心速度0.90パラメータ
#define big_right_parameter_3 0.02035,0.10871,0.85,55.000000,33.000000,57.000000,88.00000,1,1,117
#define big_right_parameter_3_cut_after 0.02035,0.0,0.85,55.000000,33.000000,57.000000,88.00000,1,1,117

//大回り180°左回り
//重心速度0.75パラメータ
#define big_180_left_parameter_1 0.0040000,0.110000,0.75,47.820000,45.000000,136.000000,191.2000000,3,1,83
#define big_180_left_parameter_1_cut_after 0.0040000,0.0,0.75,47.820000,45.000000,136.000000,191.2000000,3,1,83
//重心速度0.80パラメータs
#define big_180_left_parameter_2 0.0040000,0.11500000,0.80,50.820000,45.000000,136.000000,190.2000000,3,1,83
#define big_180_left_parameter_2_cut_after 0.0040000,0.0,0.80,50.820000,45.000000,136.000000,190.2000000,3,1,83
//重心速度0.90パラメータ
#define big_180_left_parameter_3 0.0040000,0.1250000,0.85,68.0000,45.000000,136.000000,182.2000000,3,1,83
#define big_180_left_parameter_3_cut_after 0.0040000,0.0,0.85,68.0000,45.000000,136.000000,182.2000000,3,1,83

//大回り180°右回り
//重心速度0.75パラメータ
#define big_180_right_parameter_1 0.0040000,0.120000,0.75,43.0000,45.000000,136.000000,193.5000000,1,1,117
#define big_180_right_parameter_1_cut_after 0.0040000,0.0,0.75,43.0000,45.000000,136.000000,193.5000000,1,1,117
//重心速度0.80パラメータ
#define big_180_right_parameter_2 0.0040000,0.1150000,0.80,47.0000,45.000000,136.000000,192.5000000,1,1,117
#define big_180_right_parameter_2_cut_after 0.0040000,0.0,0.80,47.0000,45.000000,136.000000,192.5000000,1,1,117
//重心速度0.90パラメータ
#define big_180_right_parameter_3 0.0040000,0.145000,0.85,60.0000,45.000000,136.000000,182.000000,1,1,117
#define big_180_right_parameter_3_cut_after 0.0040000,0.0,0.85,60.0000,45.000000,136.000000,182.00000,1,1,117

//左斜め45°入り口
#define left_45_in_oblique_parameter_1 0.0030000,0.0770000,0.75,74.000000,10.000000,35.000000,49.4000000,3,1,83
//右斜め45°入り口
#define right_45_in_oblique_parameter_1 0.002000,0.08000,0.75,74.000000,10.000000,35.000000,51.3000000,1,1,127

//左斜め45°出口
#define left_45_out_oblique_parameter_1 0.01030000,0.03500,0.75,34.000000,63.000000-45,75.000000-45,90.000000-45,3,0,400
#define left_45_out_oblique_cut_after_parameter_1 0.01030000,0.000,0.75,34.000000,63.000000-45,75.000000-45,90.000000-45,3,0,400
//右斜め45°出口
#define right_45_out_oblique_parameter_1 0.003000,0.0300000,0.75,34.000000,63.000000-45,75.000000-45,90.2000000-45,1,0,400
#define right_45_out_oblique_cut_after_parameter_1 0.003000,0.000000,0.75,34.000000,63.000000-45,75.000000-45,90.2000000-45,1,0,400

//左斜め135°入り口
#define left_135_in_oblique_parameter_1 0.0180500000,0.0860000,0.75,65.000000,50.000000,115.000000,138.73000000,3,1,75
//右斜め135°入り口
#define right_135_in_oblique_parameter_1 0.01010000,0.0880000,0.75,65.000000,50.000000,120.000000,138.730000,1,1,117

//左斜め135°出口
#define left_135_out_oblique_parameter_1 0.0150000,0.1280000,0.75,72.000000,95.000000-45,175.000000-45,186.000000-45,3,0,400
#define left_135_out_oblique_cut_after_parameter_1 0.0150000,0.00000,0.75,72.000000,95.000000-45,175.000000-45,186.000000-45,3,0,400
//右斜め135°出口
#define right_135_out_oblique_parameter_1 0.013000,0.128000000,0.75,72.000000,95.000000-45,175.000000-45,186.5000000-45,1,0,400
#define right_135_out_oblique_cut_after_parameter_1 0.00300,0.0000000,0.75,72.000000,95.000000-45,175.000000-45,186.5000000-45,1,0,400


//斜め内90°左
#define left_oblique_90_parameter_1 0.015005,0.07401,0.55,50.000000,40.000000,70.000000,90.0000000,3,0,400

//斜め内90°右
#define right_oblique_90_parameter_1 0.05503005,0.0701,0.55,50.000000,40.000000,70.000000,90.0000000,1,0,400


//大回り速度
extern double big_slalom_center_velocity;


#define MOTOR_EXITATION PORTA.PODR.BIT.B1		//励磁をkiru

#endif /* MOTOR_DEFINE_H_ */
