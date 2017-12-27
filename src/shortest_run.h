/*
 * shortest_run.h
 *
 *  Created on: 2016/10/13
 *      Author: PCーUSER
 */

#ifndef SHORTEST_RUN_H_
#define SHORTEST_RUN_H_


#include "wall_control.h"
#include "wall_control_define.h"
#include "motor.h"
#include "motor_define.h"
#include "serial.h"
#include "iodefine.h"
#include "rx631_init.h"
#include "search.h"

#define STRAIGHT 0
#define RIGHT_TURN 1
#define TURN 2
#define LEFT_TURN 3
#define GOAL 4
#define LEFT_SMALL_SLALOM 5
#define RIGHT_SMALL_SLALOM 6
#define LEFT_BIG_SLALOM 7
#define RIGHT_BIG_SLALOM 8
#define LEFT_180_BIG_SLALOM 9
#define RIGHT_180_BIG_SLALOM 10
#define LEFT_45_IN_OBLIQUE_SLALOM 11
#define RIGHT_45_IN_OBLIQUE_SLALOM 12
#define LEFT_45_OUT_OBLIQUE_SLALOM 13
#define RIGHT_45_OUT_OBLIQUE_SLALOM 14
#define LEFT_135_IN_OBLIQUE_SLALOM 15
#define RIGHT_135_IN_OBLIQUE_SLALOM 16
#define LEFT_135_OUT_OBLIQUE_SLALOM 17
#define RIGHT_135_OUT_OBLIQUE_SLALOM 18
#define LEFT_90_OBLIQUE_SLALOM 19
#define RIGHT_90_OBLIQUE_SLALOM 20
#define OBLIQUE_STRAIGHT 21

#define D_STRAIGHT_WEIGHT 5

#define BREAK_VALUE 30

void make_shortest_path_by_umuo_method(char ,char);
void make_straight_priority_step_map(char ,char);

char return_should_direction() ;

void adachi_shortest_path(char x_goal,char y_goal);
void slalom_adachi_shortest_path(char x_goal,char y_goal);


//最短経路走行　圧縮なし
void rotate_shortest_run(double velocity,double acceleration);
void slalom_shortest_run(double velocity,double acceleration);



//最短経路PATHの直線区間圧縮(rotate_straight_shortest_runの中で使う関数)
int compress_straight_shortest_path(int );
int renew_compress_straight_shortest_path(int );
int renew_compress_oblique_straight_shortest_path(int );

void initialize_shortest_path();

//最短経路走行　圧縮あり
void rotate_straight_shortest_run(double velocity,double acceleration);
void slalom_straight_shortest_run(double velocity,double initial_acceleration,double finish_acceleration);

//新しいパスに変換する関数
void renew_slalom_shortest_path();
void renew_big_slalom_shortest_path();
void renew_oblique_big_slalom_shortest_path();

//新しいパスを走る関数
void renew_slalom_shortest_run(double velocity,double initial_acceleration,double finish_acceleration);
void renew_big_slalom_shortest_run(double velocity,double initial_acceleration,double finish_acceleration,int parameter);
void renew_oblique_big_slalom_shortest_run(double velocity,double initial_acceleration,double finish_acceleration,int parameter);

//最短導出のときに使う超信地ターンと直進
void shortest_left_turn();
void shortest_right_turn();
void shortest_straight();
void shortest_turn();


void shortest_straight_slalom();
void shortest_left_slalom();
void shortest_right_slalom();


void contest_shortest_big();
void contest_shortest_oblique();

//大回り速度
extern double big_slalom_center_velocity;

extern double left_forward_oblique_wall_control_constant;
extern double right_forward_oblique_wall_control_constant;
extern double left_oblique_wall_control_constant;
extern double right_oblique_wall_control_constant;

extern int calucurate_count;
extern int calucurate_mode;


#endif /* SHORTEST_RUN_H_ */
