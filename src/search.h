/*
 * search.h
 *
 *  Created on: 2016/09/11
 *      Author: PCーUSER
 */

#ifndef SEARCH_H_
#define SEARCH_H_

#include "wall_control.h"
#include "wall_control_define.h"
#include "motor.h"
#include "motor_define.h"
#include "serial.h"
#include "iodefine.h"
#include "rx631_init.h"
#include "shortest_run.h"
#include "math.h"

#define B_NORTH_WALL 0
#define B_EAST_WALL 1
#define B_NORTH_FLAG 2
#define B_EAST_FLAG 3
#define B_NORTH_BUFF_WALL 4
#define B_EAST_BUFF_WALL 5
#define B_NORTH_BUFF_FLAG 6
#define B_EAST_BUFF_FLAG 7

//Xに壁情報を、Yに欲しい壁の情報を引数に、その値を返す
#define READ_WALL_DATA(X,Y) ((X>>Y)&(0x01))

#define TARGET_X_COORDINATE	1
#define TARGET_Y_COORDINATE 0
#define BREAK_TIME 180

//
void left_hand(unsigned char x_goal, unsigned char y_goal);

//
void adachi_method(unsigned char x_goal, unsigned char y_goal);
void slalom_adachi_method(unsigned char x_goal, unsigned char y_goal);
void slalom_furukawa_adachi_method(unsigned char x_goal, unsigned char y_goal);

//未知区間が現れるまでパス生成をし、パスを走り、移動先での方向座標を更新する関数をまとめる関数
void known_interval();

//未知区間があるまでの区間を計算する関数
int make_known_path(int ,int ,int ,unsigned char ,unsigned char );
//その座標の周囲4箇所が未探索か判別する関数(北、東、南、西の順で優先)
char distinct_unknown_area(int ,int ,int );
//その座標が既知区間加速が出来るかどうかを判定する関数
char can_do_known_interval_acceleration(int ,int ,int);
//その座標が未知区間かどうかを判定する関数
char get_unknown_interval(int ,int );

//既知区間パスを走る関数
void run_known_interval();

//移動先での方向座標を更新する関数
void update_after_known_interval();
//既知区間パスを走った後に方向を更新する関数
void update_after_run_direction();
//既知区間パスを走った後に座標を更新する関数
void update_after_run_coordinate();

//任意の座標が未探索なら１を返す関数
int is_unsearched_area(int x_coordinate,int y_coordinate,int now_direction);

int should_move_on_direction();
int return_step_min(int forward_step, int right_step, int left_step,
		int backward_step);


//壁が3枚とも入っている区画に壁を入れる関数
void add_three_wall_coordinate(int ,int );

//今居る座標を中心とした場合の4つの柱の周りの壁を入れる関数
void add_all_unknown_wall();

//柱の3辺に壁が入っていない場合、残りの1辺に壁を入れる関数
void add_unknown_wall_by_pillar(int ,int ,int );

//
void wall_information_display();

//
void update_direction(char direction);
void should_move_on_update_direction(char direction);
//
void update_coordinate();

//
void update_wall_information(char, char);

//queue関係
void enqueue_x(int x);
void enqueue_y(int x);
int dequeue_x(void);
int dequeue_y(void);

void initialize_x();
void initialize_y();
char queue_full_x();
char queue_full_y();
char queue_empty_x();
char queue_empty_y();

//
void update_step_map();
void update_step_map_queue(char, char);

void wall_init();
void search_init(char, char);

//探索のときに使う超信地ターンと直進
void search_left_turn();
void search_right_turn();
void search_straight();
void search_turn();
void search_turn_blind_alley();

void search_left_slalom();
void search_right_slalom();

void a_wall_add(char, char, char);
void a_wall_add_kai(char, char, char);
void a_wall_remove(char, char, char);
char a_wall_read(char, char, char);

void a_wall_flag_add(char, char, char);
void a_wall_flag_add_kai(char, char, char);
void a_wall_flag_remove(char, char, char);
char a_wall_flag_read(char, char, char);

void all_wall_flag_add();

//前の壁情報保存
void save_now_wall_information();
void read_buff_wall_information(char, char);
void buff_wall_init();

void all_wall_flag_remove();

char compass_convert(char);
char abs_compass_convert(char);

//その座標がマップの範囲内か調べる関数
//返り値は、マップ内なら1、マップ外なら0を返す
char check_map_out(int ,int );

char min_three_value(int ,int ,int );

/*		*/
extern char debug[10];

extern int caluculate_mode;
#endif /* SEARCH_H_ */
