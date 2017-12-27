/*
 * shortest_run.c
 *
 *  Created on: 2016/10/13
 *      Author: PCーUSER
 */

#include "shortest_run.h"

//extern unsigned char shortest_path[1000];
extern unsigned char renew_shortest_path[1000];
int shortest_path_count;
extern char map_break_flag;

//16bit型の16行16列配列
extern unsigned short straight_priority_step_map_queue[16][16];

//直進優先歩数マップを作成する関数
void make_straight_priority_step_map(char x_goal, char y_goal) {

	int x = 0, y = 0;
	int x_coordinate = 0, y_coordinate = 0;
	int straight_weight = 0;
	int straight_count = 0;
	int step = 0;
	int direction;

	int buff_x_coordinate = 0;
	int buff_y_coordinate = 0;

	int i, j;

	all_wall_flag_add();

	//直進優先用歩数マップの初期化
	for (i = 0; i < 16; i++) {
		for (j = 0; j < 16; j++) {
			straight_priority_step_map_queue[i][j] = 65535;
		}
	}
	straight_weight = D_STRAIGHT_WEIGHT;

	straight_priority_step_map_queue[x_goal][y_goal] = 0;

	enqueue_x(x_goal);
	enqueue_y(y_goal);

	while ((queue_empty_x() == 0) && (queue_empty_y() == 0)) {
		x_coordinate = dequeue_x();
		y_coordinate = dequeue_y();

		buff_x_coordinate = x_coordinate;
		buff_y_coordinate = y_coordinate;

		for (direction = 0; direction < 4; direction++) {

			x_coordinate = buff_x_coordinate;
			y_coordinate = buff_y_coordinate;

			if (direction == 0) {
				x = 0;
				y = 1;
			} else if (direction == 1) {
				x = 1;
				y = 0;
			} else if (direction == 2) {
				x = 0;
				y = -1;
			} else if (direction == 3) {
				x = -1;
				y = 0;
			}

			while (1) {
				//迷路の範囲内かつ、前壁がなくかつ、次のマスの歩数が高いなら
				if (check_map_out(x_coordinate + x, y_coordinate + y)
						== 1&& a_wall_read(x_coordinate, y_coordinate, direction)
						== 0
						&& straight_priority_step_map_queue[x_coordinate + x][y_coordinate
						+ y]
						> straight_priority_step_map_queue[x_coordinate][y_coordinate]+D_STRAIGHT_WEIGHT) {

					//直進の数が増えると、増加量減少
					step = straight_weight - straight_count;

					straight_priority_step_map_queue[x_coordinate + x][y_coordinate
							+ y] =
							straight_priority_step_map_queue[x_coordinate][y_coordinate]
									+ step;

					//座標を前に進める
					x_coordinate += x;
					y_coordinate += y;

					//座標を進めた先が、分岐出来る場所なら
					//みんとから見て右方向が分岐できるなら、オア、みんとから見て左方向が分岐できるなら
					if ((check_map_out(x_coordinate + y,
							y_coordinate + x * (-1)) == 1
							&& a_wall_read(x_coordinate, y_coordinate,
									(direction + 1) % 4) == 0
							&& straight_priority_step_map_queue[x_coordinate
									+ y * 1][y_coordinate + x * (-1)]
									> straight_priority_step_map_queue[x_coordinate][y_coordinate]
											+ D_STRAIGHT_WEIGHT)
							|| (check_map_out(x_coordinate + y * (-1),
									y_coordinate + x) == 1
									&& a_wall_read(x_coordinate, y_coordinate,
											(direction + 3) % 4) == 0
									&& straight_priority_step_map_queue[x_coordinate
											+ y * (-1)][y_coordinate + x * 1]
											> straight_priority_step_map_queue[x_coordinate][y_coordinate])
									+ D_STRAIGHT_WEIGHT) {
						enqueue_x(x_coordinate);
						enqueue_y(y_coordinate);
					}

					//直進数カウント
					straight_count++;
					if (straight_count == straight_weight) {
						straight_count = straight_weight - 1;
					}

				} else {
					straight_count = 0;
					straight_weight = D_STRAIGHT_WEIGHT;
					break;
				}
			}
		}
	}
}
//make_straight_priority_step_mapで作った歩数マップを元に最短パスを作成する関数
void make_shortest_path_by_umuo_method(char x_goal, char y_goal) {

	int direction; //今向いてる方向
	int should_direction; //進むべき方向
	int now_step = 0;

	int x, y;

	MTU_STOP
	;

	shortest_path_count = 0;

	initialize_shortest_path();

	//半区間進む
	update_coordinate();
	//半区画分の直進パス
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

	make_straight_priority_step_map(x_goal, y_goal);

	while (1) {
		//現在の歩数をstepに代入
		now_step =
				straight_priority_step_map_queue[now_x_coordinate][now_y_coordinate];
		direction = abs_compass_convert(0);

		direction %= 4;

		if (direction == 0) {
			x = 0;
			y = 1;
		} else if (direction == 1) {
			x = 1;
			y = 0;
		} else if (direction == 2) {
			x = 0;
			y = -1;
		} else if (direction == 3) {
			x = -1;
			y = 0;
		}

		should_direction = return_should_direction();

		if ((now_x_coordinate == x_goal && now_y_coordinate == y_goal)
				|| PORTC.PIDR.BIT.B6 == 0) {

			shortest_path[shortest_path_count] = STRAIGHT;
			shortest_path_count++;
			shortest_path[shortest_path_count] = 4;
			shortest_path_count++;
			MTU_STOP
			;
			return;
		} else if (should_direction == 0) {
			shortest_straight();
		} else if (should_direction == 1) {
			shortest_right_slalom();
		} else if (should_direction == 3) {
			shortest_left_slalom();
		} else {
			shortest_turn();
		}
	}
	all_wall_flag_remove();

}
char return_should_direction() {
	volatile int forward_step = 65535;
	volatile int right_step = 65535;
	volatile int left_step = 65535;
	volatile int now_step;
	volatile int wall_flg = 0;

	int min;

	volatile signed char should_move_on_x_direction;
	volatile signed char should_move_on_y_direction;

	if (0
			== a_wall_read(now_x_coordinate, now_y_coordinate,
					abs_compass_convert(3))) { // 左壁がないなら
		should_move_on_x_direction = -1 * y_direction;
		should_move_on_y_direction = 1 * x_direction;
		left_step = straight_priority_step_map_queue[now_x_coordinate
				+ should_move_on_x_direction][now_y_coordinate
				+ should_move_on_y_direction];
		wall_flg++;
	}
	if (0
			== a_wall_read(now_x_coordinate, now_y_coordinate,
					abs_compass_convert(0))) { // 前壁がないなら
		should_move_on_x_direction = 1 * x_direction;
		should_move_on_y_direction = 1 * y_direction;
		forward_step = straight_priority_step_map_queue[now_x_coordinate
				+ should_move_on_x_direction][now_y_coordinate
				+ should_move_on_y_direction];
		wall_flg++;
	}
	if (0
			== a_wall_read(now_x_coordinate, now_y_coordinate,
					abs_compass_convert(1))) { //右壁がないなら
		should_move_on_x_direction = 1 * y_direction;
		should_move_on_y_direction = -1 * x_direction;
		right_step = straight_priority_step_map_queue[now_x_coordinate
				+ should_move_on_x_direction][now_y_coordinate
				+ should_move_on_y_direction];
		wall_flg++;
	}
	now_step =
			straight_priority_step_map_queue[now_x_coordinate][now_y_coordinate];
	if (wall_flg == 0) {
		led(0);
		return 4;
	}

	min = min_three_value(forward_step, right_step, left_step);

	return min;
}

//最短経路走行ちょうしんち　圧縮なし
void rotate_shortest_run(double velocity, double acceleration) {
	int i;

	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, velocity, 0.15, 0.45,
			acceleration);
	for (i = 0; i <= 255; i++) {
		if (shortest_path[i] == 4) { //ゴール到達
			MTU_STOP
			;
			break;
		} else if (shortest_path[i + 1] != 0) { //減速
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, velocity,
					velocity, 0.15, acceleration);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if (shortest_path[i - 1] != 0) { //加速
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, velocity,
					0.15, velocity, acceleration);
		} else if (shortest_path[i] == 0) { //直進
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, velocity,
					velocity, velocity, acceleration);
		} else if (shortest_path[i] == 1) { //右旋回
			trapezoidal_acceleration_right_rotate(90, 0.3, 0.15, 0.15, 2.00);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if (shortest_path[i] == 3) { //左旋回
			trapezoidal_acceleration_left_rotate(90, 0.3, 0.15, 0.15, 2.00);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if (shortest_path[i] == 2) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
	}
}

//最短経路走行スラローム　圧縮なし
void slalom_shortest_run(double velocity, double acceleration) {
	int i;

	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, velocity, 0.15,
			velocity, acceleration);
	for (i = 1; i <= 255; i++) {
		if (shortest_path[i] == 4) { //ゴール到達
			MTU_STOP
			;
			break;
		} else if (shortest_path[i] == 1) { //右旋回
			small_slalom(small_right_parameter_1);
		} else if (shortest_path[i] == 3) { //左旋回
			small_slalom(small_left_parameter_1);
		} else if (shortest_path[i] == 2) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			__delay_ms(BREAK_TIME);
		} else if (shortest_path[i + 1] != 0 && shortest_path[i - 1] != 0) {
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, velocity, 0.45,
					0.45, acceleration);
		} else if (shortest_path[i + 1] != 0) {
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, velocity,
					velocity, 0.45, acceleration);
		} else if (shortest_path[i - 1] != 0) {
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, velocity, 0.45,
					velocity, acceleration);
		} else if (shortest_path[i] == 0) { //直進
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, velocity,
					velocity, velocity, acceleration);
		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
	}
}

//最短経路PATHの直線区間圧縮(rotate_straight_shortest_runの中で使う関数)
int compress_straight_shortest_path(int i) {
	int straight_count = 0;
	for (i; i <= 255; i++) {
		if (shortest_path[i] == 0) {
			straight_count++;
		} else
			break;

	}
	return straight_count;
}
int renew_compress_straight_shortest_path(int i) {
	int straight_count = 0;
	for (i; i <= 255; i++) {
		if (renew_shortest_path[i] == 0) {
			straight_count++;
		} else
			break;
	}
	return straight_count;
}
int renew_compress_oblique_straight_shortest_path(int i) {
	int oblique_straight_count = 0;
	for (i; i <= 255; i++) {
		if (renew_shortest_path[i] == OBLIQUE_STRAIGHT) {
			oblique_straight_count++;
		} else
			break;

	}
	return oblique_straight_count;
}

//新しいパスに作り変える関数
void renew_slalom_shortest_path() {
	volatile int i = 0;
	volatile int j = 0;
	volatile int straight_compress_value = 0;

	while (1) {
		map_break_flag = 0;
		if (shortest_path[i] == GOAL) { //ゴール到達
			renew_shortest_path[j] = GOAL;
			break;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == STRAIGHT) {
			renew_shortest_path[j] = LEFT_BIG_SLALOM;
			i += 2;
		} else if (i != 0&& shortest_path[i] == STRAIGHT
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == STRAIGHT) {
			renew_shortest_path[j] = RIGHT_BIG_SLALOM;
			i += 2;
		} else if (shortest_path[i] == RIGHT_TURN) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if (shortest_path[i] == LEFT_TURN) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else if (shortest_path[i] == TURN) { //180度旋回
			renew_shortest_path[j] = TURN;
		} else if (shortest_path[i] == STRAIGHT) {

		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		i++;
		j++;

		map_break_flag = 0;
	}
	i = 0;
	j = 0;
	while (1) {
		map_break_flag = 0;
		if (renew_shortest_path[i] == GOAL) { //ゴール到達
			renew_shortest_path[j] = GOAL;
			break;
		} else if (renew_shortest_path[i] == LEFT_BIG_SLALOM) {
			renew_shortest_path[j] = LEFT_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_BIG_SLALOM) {
			renew_shortest_path[j] = RIGHT_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_SMALL_SLALOM) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_SMALL_SLALOM) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else if (renew_shortest_path[i] == TURN) { //180度旋回
			renew_shortest_path[j] = TURN;
		} else if (renew_shortest_path[i] == STRAIGHT) {

			straight_compress_value = renew_compress_straight_shortest_path(i);
			renew_shortest_path[j] = straight_compress_value + 100;
			i += (straight_compress_value - 1);
		}
		i++;
		j++;
	}
	for (j; j < 1000; j++) {
		renew_shortest_path[i] = BREAK_VALUE;
	}
}

//新しいパスを走る関数
void renew_slalom_shortest_run(double velocity, double initial_acceleration,
		double finish_acceleration) {
	int i = 0;
	double init = 0.0;
	double init_2 = 0.0;

	while (1) {
		map_break_flag = 0;
		if (renew_shortest_path[i] == GOAL) { //ゴール到達
			MTU_STOP
			;
			__delay_ms(500);
			MOTOR_EXITATION = 0;
			return;
		} else if (renew_shortest_path[i] == RIGHT_BIG_SLALOM) {
			small_slalom(big_right_parameter_1);
		} else if (renew_shortest_path[i] == LEFT_BIG_SLALOM) {
			small_slalom(big_left_parameter_1);
		} else if (renew_shortest_path[i] == RIGHT_SMALL_SLALOM) { //右旋回
			small_slalom(small_right_parameter_1);
		} else if (renew_shortest_path[i] == LEFT_SMALL_SLALOM) { //左旋回
			small_slalom(small_left_parameter_1);
		} else if (renew_shortest_path[i] == TURN) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if (renew_shortest_path[i] > 100) { //直進
			if (i == 0) {
				init = big_slalom_center_velocity - 0.15;
			}
			if (renew_shortest_path[i + 1] != GOAL
					&& (renew_shortest_path[i + 1] == RIGHT_SMALL_SLALOM
							|| renew_shortest_path[i + 1] == LEFT_SMALL_SLALOM)) {
				init_2 = big_slalom_center_velocity - 0.45;
			}
			if (i != 0
					&& (renew_shortest_path[i - 1] == RIGHT_SMALL_SLALOM
							|| renew_shortest_path[i - 1] == LEFT_SMALL_SLALOM)) {
				init = big_slalom_center_velocity - 0.45;
			}
			if (renew_shortest_path[i + 1] == GOAL) {
				init_2 = big_slalom_center_velocity - 0.15;
			}

			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5 * (renew_shortest_path[i] - 100), velocity,
					big_slalom_center_velocity - init,
					big_slalom_center_velocity - init_2, initial_acceleration,
					finish_acceleration);
			init = 0.0;
			init_2 = 0.0;
		} else if (renew_shortest_path[i] == BREAK_VALUE) {
			return;
		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		i++;
	}
}

//新しいパスに作り変える関数
void renew_big_slalom_shortest_path() {
	volatile int i = 0;
	volatile int j = 0;
	volatile int straight_compress_value = 0;

	while (1) {
		map_break_flag = 0;
		if (shortest_path[i] == GOAL) { //ゴール到達
			renew_shortest_path[j] = GOAL;
			break;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == STRAIGHT) {
			renew_shortest_path[j] = LEFT_BIG_SLALOM;
			i += 2;
		} else if (i != 0&& shortest_path[i] == STRAIGHT
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == STRAIGHT) {
			renew_shortest_path[j] = RIGHT_BIG_SLALOM;
			i += 2;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == RIGHT_TURN
				&& shortest_path[i + 2] == RIGHT_TURN
				&& shortest_path[i + 3] == STRAIGHT) {
			renew_shortest_path[j] = RIGHT_180_BIG_SLALOM;
			i += 3;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == LEFT_TURN
				&& shortest_path[i + 3] == STRAIGHT) {
			renew_shortest_path[j] = LEFT_180_BIG_SLALOM;
			i += 3;
		} else if (shortest_path[i] == RIGHT_TURN) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if (shortest_path[i] == LEFT_TURN) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else if (shortest_path[i] == TURN) { //180度旋回
			renew_shortest_path[j] = TURN;
		} else if (shortest_path[i] == STRAIGHT) {
			renew_shortest_path[j] = STRAIGHT;
		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		i++;
		j++;

		map_break_flag = 0;
	}
	i = 0;
	j = 0;
	while (1) {
		map_break_flag = 0;
		if (renew_shortest_path[i] == GOAL) { //ゴール到達
			renew_shortest_path[j] = GOAL;
			break;
		} else if (renew_shortest_path[i] == LEFT_BIG_SLALOM) {
			renew_shortest_path[j] = LEFT_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_BIG_SLALOM) {
			renew_shortest_path[j] = RIGHT_BIG_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_180_BIG_SLALOM) {
			renew_shortest_path[j] = LEFT_180_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_180_BIG_SLALOM) {
			renew_shortest_path[j] = RIGHT_180_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_SMALL_SLALOM) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_SMALL_SLALOM) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else if (renew_shortest_path[i] == TURN) { //180度旋回
			renew_shortest_path[j] = TURN;
		} else if (i
				== 0&& renew_shortest_path[0] == STRAIGHT && renew_shortest_path[1] != STRAIGHT) {
			renew_shortest_path[0] = STRAIGHT;
		} else if (renew_shortest_path[i] == STRAIGHT) {

			straight_compress_value = renew_compress_straight_shortest_path(i);
			renew_shortest_path[j] = straight_compress_value + 100;
			i += (straight_compress_value - 1);
		}
		i++;
		j++;
	}
	for (j; j < 1000; j++) {
		renew_shortest_path[j] = BREAK_VALUE;
	}
}

//新しいパスを走る関数
void renew_big_slalom_shortest_run(double velocity, double initial_acceleration,
		double finish_acceleration, int parameter) {
	int i = 0;
	double init = 0.0;
	double init_2 = 0.0;

	double before_distance = 0.0;
	double start_distance = 0.0;
	double start_acceleration = 0.0;

	while (1) {
		map_break_flag = 0;
		if (renew_shortest_path[i] == GOAL
				|| renew_shortest_path[i] == BREAK_VALUE) { //ゴール到達

			MTU_STOP
			;
			__delay_ms(500);
			MOTOR_EXITATION = 0;
			return;
		} else if (PORTC.PIDR.BIT.B6 == 0 || hihumint_dead_wall() == 1) {
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			sensor_led_flg = 0;

			led(2);
			while (PORTC.PIDR.BIT.B6 == 1)
				;

			read_buff_wall_information(TARGET_X_COORDINATE,
			TARGET_Y_COORDINATE);

			return;
		} else if (renew_shortest_path[i] == RIGHT_180_BIG_SLALOM) {
			if (renew_shortest_path[i + 1] == RIGHT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_BIG_SLALOM) {
				if (parameter == 1) {
					small_slalom(big_180_right_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_180_right_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_180_right_parameter_3_cut_after);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_180_right_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_180_right_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_180_right_parameter_3);
				}
			}
		} else if (renew_shortest_path[i] == LEFT_180_BIG_SLALOM) {
			if (renew_shortest_path[i + 1] == RIGHT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_BIG_SLALOM) {
				if (parameter == 1) {
					small_slalom(big_180_left_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_180_left_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_180_left_parameter_3_cut_after);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_180_left_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_180_left_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_180_left_parameter_3);
				}
			}
		} else if (renew_shortest_path[i] == RIGHT_BIG_SLALOM) {
			if (renew_shortest_path[i + 1] == RIGHT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_BIG_SLALOM) {
				if (parameter == 1) {
					small_slalom(big_right_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_right_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_right_parameter_3_cut_after);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_right_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_right_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_right_parameter_3);
				}
			}
		} else if (renew_shortest_path[i] == LEFT_BIG_SLALOM) {
			if (renew_shortest_path[i + 1] == RIGHT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_BIG_SLALOM) {
				if (parameter == 1) {
					small_slalom(big_left_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_left_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_left_parameter_3_cut_after);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_left_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_left_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_left_parameter_3);
				}
			}
		} else if (renew_shortest_path[i] == RIGHT_SMALL_SLALOM) { //右旋回
			small_slalom(small_right_parameter_1);
		} else if (renew_shortest_path[i] == LEFT_SMALL_SLALOM) { //左旋回
			small_slalom(small_left_parameter_1);
		} else if (renew_shortest_path[i] == TURN) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if (renew_shortest_path[i] > 100) { //直進
			before_distance = 0.0;

			if (parameter == 1) {
				big_slalom_center_velocity = 0.75;
			} else if (parameter == 2) {
				big_slalom_center_velocity = 0.8;
			} else if (parameter == 3) {
				big_slalom_center_velocity = 0.85;
			}

			//斜めがないので重心速度は一定でおｋ

			if (i == 0) {
				init = big_slalom_center_velocity - 0.15;
				if (renew_shortest_path[i + 1] != LEFT_SMALL_SLALOM
						|| renew_shortest_path[i + 1] != RIGHT_SMALL_SLALOM) {
					start_distance = 0.032;
				} else {
					start_distance = 0.0;
				}
				start_acceleration = initial_acceleration - 2.0;
			}
			if (renew_shortest_path[i + 1] == RIGHT_SMALL_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_SMALL_SLALOM) {
				init_2 = big_slalom_center_velocity - 0.45;
			}
			if (renew_shortest_path[i + 1] == RIGHT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_180_BIG_SLALOM) {
				before_distance = WALL_OUT_BEFORE_DISTANCE;
			}
			if (i != 0
					&& (renew_shortest_path[i - 1] == RIGHT_SMALL_SLALOM
							|| renew_shortest_path[i - 1] == LEFT_SMALL_SLALOM)) {
				init = big_slalom_center_velocity - 0.45;
			}
			if (renew_shortest_path[i + 1] == GOAL
					|| renew_shortest_path[i + 1] == BREAK_VALUE) {
				init_2 = big_slalom_center_velocity - 0.15;
			}

			trapezoidal_acceleration_straight_distance(
					BLOCK_LENGTH * 0.5 * (renew_shortest_path[i] - 100)
							- before_distance + start_distance, velocity,
					big_slalom_center_velocity - init,
					big_slalom_center_velocity - init_2,
					initial_acceleration - start_acceleration,
					finish_acceleration);
			init = 0.0;
			init_2 = 0.0;
			before_distance = 0.0;
			start_distance = 0.0;
			start_acceleration = 0.0;
			big_slalom_center_velocity = 0.75;
		} else if (renew_shortest_path[i] == STRAIGHT) {
			//開幕小回り
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.15, 0.45, initial_acceleration,
					finish_acceleration);
		} else if (renew_shortest_path[i] == BREAK_VALUE) {
			return;
		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		i++;
	}
}

//新しいパスに作り変える関数(斜め)
void renew_oblique_big_slalom_shortest_path() {
	volatile int i = 0;
	volatile int j = 0;
	volatile int straight_compress_value = 0;
	volatile int oblique_straight_compress_value = 0;
	int oblique_flag = 0;

	while (1) {
		map_break_flag = 0;
		if (shortest_path[i] == GOAL) { //ゴール到達
			oblique_flag = 0;
			renew_shortest_path[j] = GOAL;
			break;
		} else if (i == 1 && shortest_path[i] == RIGHT_TURN) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if ((i == 3 || i == 2) && shortest_path[i] == LEFT_TURN) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else if (shortest_path[i - 1] == RIGHT_TURN
				&& shortest_path[i] == LEFT_TURN
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == RIGHT_TURN) {
			//左斜め内90°
			renew_shortest_path[j] = LEFT_90_OBLIQUE_SLALOM;
			i += 1;
		} else if (shortest_path[i - 1] == LEFT_TURN
				&& shortest_path[i] == RIGHT_TURN
				&& shortest_path[i + 1] == RIGHT_TURN
				&& shortest_path[i + 2] == LEFT_TURN) {
			//右斜め内90°
			renew_shortest_path[j] = RIGHT_90_OBLIQUE_SLALOM;
			i += 1;
		} else if (i != 0&& shortest_path[i] == STRAIGHT
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == LEFT_TURN) {
			//45右斜め入り口
			oblique_flag = 1;
			renew_shortest_path[j] = RIGHT_45_IN_OBLIQUE_SLALOM;
			i += 1;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == RIGHT_TURN) {
			//45左斜め入り口
			oblique_flag = 1;
			renew_shortest_path[j] = LEFT_45_IN_OBLIQUE_SLALOM;
			i += 1;
		} else if (oblique_flag
				== 1&& i != 0&& shortest_path[i - 1] == LEFT_TURN
				&& shortest_path[i] == RIGHT_TURN
				&& shortest_path[i + 1] == STRAIGHT) {
			//45右斜め出口
			oblique_flag = 0;
			renew_shortest_path[j] = RIGHT_45_OUT_OBLIQUE_SLALOM;
			i += 1;
		} else if (oblique_flag
				== 1&& i != 0&& shortest_path[i - 1] == RIGHT_TURN
				&& shortest_path[i] == LEFT_TURN
				&& shortest_path[i + 1] == STRAIGHT) {
			//45左斜め出口
			oblique_flag = 0;
			renew_shortest_path[j] = LEFT_45_OUT_OBLIQUE_SLALOM;
			i += 1;
		} else if (i != 0&& shortest_path[i] == STRAIGHT
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == RIGHT_TURN
		&& shortest_path[i + 3] == LEFT_TURN) {
			//135右斜め入り口
			oblique_flag = 1;
			renew_shortest_path[j] = RIGHT_135_IN_OBLIQUE_SLALOM;
			i += 2;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == LEFT_TURN
				&& shortest_path[i + 3] == RIGHT_TURN) {
			//135左斜め入り口
			oblique_flag = 1;
			renew_shortest_path[j] = LEFT_135_IN_OBLIQUE_SLALOM;
			i += 2;
		} else if (oblique_flag
				== 1&&i != 0&& shortest_path[i - 1] == RIGHT_TURN
				&& shortest_path[i] == LEFT_TURN
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == STRAIGHT) {
			//135左斜め出口
			oblique_flag = 0;
			renew_shortest_path[j] = LEFT_135_OUT_OBLIQUE_SLALOM;
			i += 2;
		} else if (oblique_flag == 1&&i != 0&& shortest_path[i - 1] == LEFT_TURN
		&& shortest_path[i] == RIGHT_TURN
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == STRAIGHT) {
			//135右斜め出口
			oblique_flag = 0;
			renew_shortest_path[j] = RIGHT_135_OUT_OBLIQUE_SLALOM;
			i += 2;
		} else if (oblique_flag == 1
				&& ((shortest_path[i] == LEFT_TURN
						&& shortest_path[i + 1] == RIGHT_TURN)
						|| (i != 2 && shortest_path[i] == RIGHT_TURN
								&& shortest_path[i + 1] == LEFT_TURN))) {
			//斜め直進
			renew_shortest_path[j] = OBLIQUE_STRAIGHT;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == STRAIGHT) {
			renew_shortest_path[j] = LEFT_BIG_SLALOM;
			i += 2;
		} else if (i != 0&& shortest_path[i] == STRAIGHT
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == STRAIGHT) {
			renew_shortest_path[j] = RIGHT_BIG_SLALOM;
			i += 2;
		} else if (i != 0&& shortest_path[i] == STRAIGHT
		&& shortest_path[i + 1] == RIGHT_TURN
		&& shortest_path[i + 2] == RIGHT_TURN
		&& shortest_path[i + 3] == STRAIGHT) {
			renew_shortest_path[j] = RIGHT_180_BIG_SLALOM;
			i += 3;
		} else if (shortest_path[i] == STRAIGHT
				&& shortest_path[i + 1] == LEFT_TURN
				&& shortest_path[i + 2] == LEFT_TURN
				&& shortest_path[i + 3] == STRAIGHT) {
			renew_shortest_path[j] = LEFT_180_BIG_SLALOM;
			i += 3;
		} else if (shortest_path[i] == TURN) { //180度旋回
			renew_shortest_path[j] = TURN;
		} else if (shortest_path[i] == STRAIGHT) {
			renew_shortest_path[j] = STRAIGHT;
		} else if (shortest_path[i] == RIGHT_TURN) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if (shortest_path[i] == LEFT_TURN) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else {
			//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		i++;
		j++;

		map_break_flag = 0;
	}
	i = 0;
	j = 0;
	while (1) {
		map_break_flag = 0;
		if (renew_shortest_path[i] == GOAL) { //ゴール到達
			renew_shortest_path[j] = GOAL;
			i++;
			j++;
			break;
		} else if (renew_shortest_path[i] == LEFT_BIG_SLALOM) {
			renew_shortest_path[j] = LEFT_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_BIG_SLALOM) {
			renew_shortest_path[j] = RIGHT_BIG_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_180_BIG_SLALOM) {
			renew_shortest_path[j] = LEFT_180_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_180_BIG_SLALOM) {
			renew_shortest_path[j] = RIGHT_180_BIG_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_SMALL_SLALOM) { //右旋回
			renew_shortest_path[j] = RIGHT_SMALL_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_SMALL_SLALOM) { //左旋回
			renew_shortest_path[j] = LEFT_SMALL_SLALOM;
		} else if (renew_shortest_path[i] == TURN) { //180度旋回
			renew_shortest_path[j] = TURN;
		} else if (renew_shortest_path[i] == LEFT_90_OBLIQUE_SLALOM) {
			//斜め内左90
			renew_shortest_path[j] = LEFT_90_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_90_OBLIQUE_SLALOM) {
			//斜め内右90
			renew_shortest_path[j] = RIGHT_90_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_45_IN_OBLIQUE_SLALOM) {
			//45左斜め入り口
			renew_shortest_path[j] = LEFT_45_IN_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_45_IN_OBLIQUE_SLALOM) {
			//45右斜め入り口
			renew_shortest_path[j] = RIGHT_45_IN_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_45_OUT_OBLIQUE_SLALOM) {
			//45左斜め出口
			renew_shortest_path[j] = LEFT_45_OUT_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_45_OUT_OBLIQUE_SLALOM) {
			//45右斜め出口
			renew_shortest_path[j] = RIGHT_45_OUT_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_135_IN_OBLIQUE_SLALOM) {
			//135左斜め入り口
			renew_shortest_path[j] = LEFT_135_IN_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_135_IN_OBLIQUE_SLALOM) {
			//135右斜め入り口
			renew_shortest_path[j] = RIGHT_135_IN_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == LEFT_135_OUT_OBLIQUE_SLALOM) {
			//135左斜め出口
			renew_shortest_path[j] = LEFT_135_OUT_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == RIGHT_135_OUT_OBLIQUE_SLALOM) {
			//135右斜め出口
			renew_shortest_path[j] = RIGHT_135_OUT_OBLIQUE_SLALOM;
		} else if (renew_shortest_path[i] == OBLIQUE_STRAIGHT) {

			oblique_straight_compress_value =
					renew_compress_oblique_straight_shortest_path(i);
			renew_shortest_path[j] = oblique_straight_compress_value + 200;
			i += (oblique_straight_compress_value - 1);
		} else if (i
				== 0&& renew_shortest_path[0] == STRAIGHT && renew_shortest_path[1] != STRAIGHT) {
			renew_shortest_path[0] = STRAIGHT;
		} else if (renew_shortest_path[i] == STRAIGHT) {

			straight_compress_value = renew_compress_straight_shortest_path(i);
			renew_shortest_path[j] = straight_compress_value + 100;
			i += (straight_compress_value - 1);
		}
		i++;
		j++;
	}
	for (j; j < 1000; j++) {
		renew_shortest_path[j] = BREAK_VALUE;
	}
}

//新しいパスを走る関数(斜め)
void renew_oblique_big_slalom_shortest_run(double velocity,
		double initial_acceleration, double finish_acceleration, int parameter) {
	int i = 0;
	double init = 0.0;
	double init_2 = 0.0;
	double buff_wall_control_constant;
	double before_distance = 0.0;
	double start_distance = 0.0;
	double start_acceleration = 0.0;
	double oblique_init_1 = 0, oblique_init_2 = 0;

	while (1) {
		map_break_flag = 0;
		if (renew_shortest_path[i] == GOAL
				|| renew_shortest_path[i] == BREAK_VALUE) { //ゴール到達
			MTU_STOP
			;
			__delay_ms(500);
			MOTOR_EXITATION = 0;
			return;

		} else if (hihumint_dead_wall() == 1 || PORTC.PIDR.BIT.B6 == 0) {
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			sensor_led_flg = 0;

			led(2);
			while (PORTC.PIDR.BIT.B6 == 1)
				;

			read_buff_wall_information(TARGET_X_COORDINATE,
			TARGET_Y_COORDINATE);

			return;
		}

		//45°入り口出口
		else if (renew_shortest_path[i] == LEFT_45_IN_OBLIQUE_SLALOM) {
			small_slalom(left_45_in_oblique_parameter_1);
		} else if (renew_shortest_path[i] == RIGHT_45_IN_OBLIQUE_SLALOM) {
			small_slalom(right_45_in_oblique_parameter_1);
		} else if (renew_shortest_path[i] == LEFT_45_OUT_OBLIQUE_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				small_slalom(left_45_out_oblique_parameter_1);
			} else {
				small_slalom(left_45_out_oblique_cut_after_parameter_1);
			}
		} else if (renew_shortest_path[i] == RIGHT_45_OUT_OBLIQUE_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				small_slalom(right_45_out_oblique_parameter_1);
			} else {
				small_slalom(right_45_out_oblique_cut_after_parameter_1);
			}
		}
		//135°入り口出口
		else if (renew_shortest_path[i] == LEFT_135_IN_OBLIQUE_SLALOM) {
			small_slalom(left_135_in_oblique_parameter_1);
		} else if (renew_shortest_path[i] == RIGHT_135_IN_OBLIQUE_SLALOM) {
			small_slalom(right_135_in_oblique_parameter_1);
		} else if (renew_shortest_path[i] == LEFT_135_OUT_OBLIQUE_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				small_slalom(left_135_out_oblique_parameter_1);
			} else {
				small_slalom(left_135_out_oblique_cut_after_parameter_1);
			}
		} else if (renew_shortest_path[i] == RIGHT_135_OUT_OBLIQUE_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				small_slalom(right_135_out_oblique_parameter_1);
			} else {
				small_slalom(right_135_out_oblique_cut_after_parameter_1);
			}
		} else if (renew_shortest_path[i] == LEFT_90_OBLIQUE_SLALOM) {
			small_slalom(left_oblique_90_parameter_1);
		} else if (renew_shortest_path[i] == RIGHT_90_OBLIQUE_SLALOM) {
			small_slalom(right_oblique_90_parameter_1);
		} else if (renew_shortest_path[i] == RIGHT_180_BIG_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				if (parameter == 1) {
					small_slalom(big_180_right_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_180_right_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_180_right_parameter_3);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_180_right_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_180_right_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_180_right_parameter_3_cut_after);
				}
			}

		} else if (renew_shortest_path[i] == LEFT_180_BIG_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				if (parameter == 1) {
					small_slalom(big_180_left_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_180_left_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_180_left_parameter_3);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_180_left_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_180_left_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_180_left_parameter_3_cut_after);
				}
			}
		} else if (renew_shortest_path[i] == RIGHT_BIG_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				if (parameter == 1) {
					small_slalom(big_right_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_right_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_right_parameter_3);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_right_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_right_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_right_parameter_3_cut_after);
				}
			}
		} else if (renew_shortest_path[i] == LEFT_BIG_SLALOM) {
			if ((renew_shortest_path[i + 1] == GOAL)
					|| (renew_shortest_path[i + 1] > 100
							&& renew_shortest_path[i + 2] == GOAL)) {
				if (parameter == 1) {
					small_slalom(big_left_parameter_1);
				} else if (parameter == 2) {
					small_slalom(big_left_parameter_2);
				} else if (parameter == 3) {
					small_slalom(big_left_parameter_3);
				}
			} else {
				if (parameter == 1) {
					small_slalom(big_left_parameter_1_cut_after);
				} else if (parameter == 2) {
					small_slalom(big_left_parameter_2_cut_after);
				} else if (parameter == 3) {
					small_slalom(big_left_parameter_3_cut_after);
				}
			}
		} else if (renew_shortest_path[i] == RIGHT_SMALL_SLALOM) { //右旋回
			small_slalom(small_right_parameter_1);
		} else if (renew_shortest_path[i] == LEFT_SMALL_SLALOM) { //左旋回
			small_slalom(small_left_parameter_1);
		} else if (renew_shortest_path[i] == TURN) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if (renew_shortest_path[i] > 200) {
			buff_wall_control_constant = wall_control_constant;
			left_forward_oblique_wall_control_constant =
			LEFT_FORWARD_OBLIQUE_WALL_CONTROL;
			right_forward_oblique_wall_control_constant =
			RIGHT_FORWARD_OBLIQUE_WALL_CONTROL;
			/*
			 oblique_trapezoidal_acceleration_straight_distance(
			 BLOCK_LENGTH * 0.5 * 1.41421356 * (renew_shortest_path[i] - 200),
			 velocity, big_slalom_center_velocity,
			 big_slalom_center_velocity, initial_acceleration,
			 finish_acceleration);
			 */

			if (renew_shortest_path[i + 1] == LEFT_90_OBLIQUE_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_90_OBLIQUE_SLALOM) {
				big_slalom_center_velocity = 0.55;
				oblique_init_1 = 0.2;
			}
			if (renew_shortest_path[i - 1] == LEFT_90_OBLIQUE_SLALOM
					|| renew_shortest_path[i - 1] == RIGHT_90_OBLIQUE_SLALOM) {
				big_slalom_center_velocity = 0.55;
				oblique_init_2 = 0.2;
			}

			oblique_trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5 * 1.41421356 * (renew_shortest_path[i] - 200),
					1.2, big_slalom_center_velocity + oblique_init_1,
					big_slalom_center_velocity + oblique_init_2, 1.2, 1.2);

			big_slalom_center_velocity = 0.75;
			oblique_init_1 = 0;
			oblique_init_2 = 0;
			wall_control_constant = buff_wall_control_constant;
		} else if (renew_shortest_path[i] > 100) { //直進

			if (parameter == 1) {
				big_slalom_center_velocity = 0.75;
			} else if (parameter == 2) {
				big_slalom_center_velocity = 0.8;
			} else if (parameter == 3) {
				big_slalom_center_velocity = 0.85;
			}

			//直線の次が斜め入り口なら重心速度を0.75に戻す→なんなら0.7でもよいのでは？再現性ないものは下げてもよし。
			if (renew_shortest_path[i + 1] == LEFT_45_IN_OBLIQUE_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_45_IN_OBLIQUE_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_135_IN_OBLIQUE_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_135_IN_OBLIQUE_SLALOM) {
				oblique_init_2 = big_slalom_center_velocity - 0.75;
				big_slalom_center_velocity = 0.75;
			}
			if (renew_shortest_path[i - 1] == LEFT_45_IN_OBLIQUE_SLALOM
					|| renew_shortest_path[i - 1] == RIGHT_45_IN_OBLIQUE_SLALOM
					|| renew_shortest_path[i - 1] == LEFT_135_IN_OBLIQUE_SLALOM
					|| renew_shortest_path[i - 1] == RIGHT_135_IN_OBLIQUE_SLALOM) {
				oblique_init_1 = big_slalom_center_velocity - 0.75;
				big_slalom_center_velocity = 0.75;
			}

			if (i == 0) { //一番最初の直線
				init = big_slalom_center_velocity - 0.15;
				if (renew_shortest_path[i + 1] != LEFT_SMALL_SLALOM
						|| renew_shortest_path[i + 1] != RIGHT_SMALL_SLALOM) {
					start_distance = 0.032;
				} else {
					start_distance = 0.0;
				}
				start_acceleration = initial_acceleration - 2.0;
			}

			if (renew_shortest_path[i + 1] == RIGHT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_BIG_SLALOM
					|| renew_shortest_path[i + 1] == RIGHT_180_BIG_SLALOM
					|| renew_shortest_path[i + 1] == LEFT_180_BIG_SLALOM) {
				before_distance = WALL_OUT_BEFORE_DISTANCE;
			}
			if (i != 0
					&& (renew_shortest_path[i - 1] == RIGHT_SMALL_SLALOM
							|| renew_shortest_path[i - 1] == LEFT_SMALL_SLALOM)) {
				init = big_slalom_center_velocity - 0.45;
			}
			if (renew_shortest_path[i + 1] == GOAL) {
				init_2 = big_slalom_center_velocity - 0.15;
			}

			trapezoidal_acceleration_straight_distance(
					BLOCK_LENGTH * 0.5 * (renew_shortest_path[i] - 100)
							- before_distance + start_distance, velocity,
					big_slalom_center_velocity - init + oblique_init_1,
					big_slalom_center_velocity - init_2 + oblique_init_2,
					initial_acceleration - start_acceleration,
					finish_acceleration);
			init = 0.0;
			init_2 = 0.0;
			oblique_init_1 = 0;
			oblique_init_2 = 0;
			before_distance = 0.0;
			start_distance = 0.0;
			start_acceleration = 0.0;
		} else if (renew_shortest_path[i] == STRAIGHT) {
//開幕小回り
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.15, 0.45, initial_acceleration,
					finish_acceleration);
		} else if (renew_shortest_path[i] == BREAK_VALUE) {
			return;
		} else {
//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		i++;
	}
}

//最短経路走行　圧縮あり
void rotate_straight_shortest_run(double velocity, double acceleration) {
	int i;
	int compress_value;

	for (i = 0; i <= 255; i++) {
		if (shortest_path[i] == 4) { //ゴール到達
			trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5,
					velocity, velocity, 0.15, acceleration, acceleration);
			MTU_STOP
			;
			break;
		} else if (shortest_path[i] == 0) { //直進
			compress_value = compress_straight_shortest_path(i);
			trapezoidal_acceleration_straight(
			BLOCK_LENGTH * 0.5 * compress_value, velocity, 0.15, 0.15,
					acceleration);
			MTU_STOP
			;
			i += (compress_value - 1);
		} else if (shortest_path[i] == 1) { //右旋回
			trapezoidal_acceleration_right_rotate(90, 0.3, 0.15, 0.15, 2.00);
		} else if (shortest_path[i] == 3) { //左旋回
			trapezoidal_acceleration_left_rotate(90, 0.3, 0.15, 0.15, 2.00);
		} else if (shortest_path[i] == 2) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else {
//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
	}
}

//最短経路走行スラローム　圧縮あり
void slalom_straight_shortest_run(double velocity, double initial_acceleration,
		double finish_acceleration) {
	int i;
	int comp = 0;
	int compress_value;
	double init = 0.0;
	char start_flg = 0;
	char debug_count = 0;

	char bug_count = 0;

	while (1) {
		map_break_flag = 0;
		if (shortest_path[i] == 4) { //ゴール到達
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15, 2.0, 4.0);

			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			break;
		} else if (shortest_path[i] == 1) { //右旋回
			small_slalom(small_right_parameter_1);

		} else if (shortest_path[i] == 3) { //左旋回
			small_slalom(small_left_parameter_1);
		} else if (shortest_path[i] == 2) { //180度旋回
			trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
			MTU_STOP
			;
			__delay_ms(BREAK_TIME);
		} else if ((shortest_path[1] == 1 || shortest_path[1] == 3)
				&& start_flg == 0) {
			bug_count++;
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45, initial_acceleration,
					finish_acceleration);

		} else if (shortest_path[i] == 0) { //直進
			debug_count++;
			if (start_flg == 0) {
				init = 0.3;
			}
			compress_value = compress_straight_shortest_path(i);

			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5 * compress_value, velocity, 0.45 - init, 0.45,
					initial_acceleration, finish_acceleration);

			i += (compress_value - 1);

			init = 0.0;

		} else {
//ここに入るのはバグ
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			led(3);
			__delay_ms(100);
			led(0);
			__delay_ms(100);
			break;
		}
		start_flg = 1;
		i++;

		map_break_flag = 0;
	}
	/*
	 MTU_STOP
	 ;
	 MOTOR_EXITATION = 1;
	 start_switch();
	 myprintf("debug = %d\n bug = %d\n compress_value = %d\n", debug_count,
	 bug_count, compress_value);
	 for (i = 0; i < 30; i++) {
	 myprintf("path = %d\n", shortest_path[i]);
	 }

	 MOTOR_EXITATION = 1;
	 trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5 * compress_value, 0.45,
	 0.15, 0.45, acceleration);

	 MTU_STOP
	 ;
	 MOTOR_EXITATION = 1;
	 */
}

void initialize_shortest_path() {
	int i = 0;
	for (i = 0; i < 1000; i++) {
		shortest_path[i] = 0;
		renew_shortest_path[i] = 0;
	}
}

//ちょうしんちによる足立法ベース最短パス
void adachi_shortest_path(char x_goal, char y_goal) {

	int direction;
//	update_wall_information(now_x_coordinate, now_y_coordinate);

	MTU_STOP
	;

	initialize_shortest_path();
	all_wall_flag_add();
//半区間進む
//trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45,2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

//座標を(0,1)に更新
	update_coordinate();

	while (1) {
		update_step_map_queue(x_goal, y_goal);	//壁情報より歩数マップを更新する

		direction = should_move_on_direction();

		if ((now_x_coordinate == x_goal && now_y_coordinate == y_goal)
				|| PORTC.PIDR.BIT.B6 == 0) {

//update_wall_information(now_x_coordinate, now_y_coordinate);
			update_step_map_queue(x_goal, y_goal);
//trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45,0.2, 1.0);
//			shortest_path[shortest_path_count] = STRAIGHT;
//			shortest_path_count++;

			shortest_path[shortest_path_count] = 4;
			shortest_path_count++;
			MTU_STOP
			;
			return;
		} else if (0 == direction) {
			shortest_straight();
		} else if (1 == direction) {
			shortest_right_turn();
		} else if (3 == direction) {
			shortest_left_turn();
		} else {
			shortest_turn();
		}
	}

	all_wall_flag_remove();
}

//スラロームでの足立法によるパス生成
void slalom_adachi_shortest_path(char x_goal, char y_goal) {

	int direction;
//	update_wall_information(now_x_coordinate, now_y_coordinate);

	MTU_STOP
	;

	all_wall_flag_add();
	shortest_path_count = 0;

	initialize_shortest_path();
//半区間進む
//trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45,2.0);
//半区画分の直進パス
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

//座標を(0,1)に更新
	update_coordinate();

	while (1) {
		update_step_map_queue(x_goal, y_goal);	//壁情報より歩数マップを更新する

		direction = should_move_on_direction();
		if (hihumint_dead_wall() == 1) {
			read_buff_wall_information(x_goal, y_goal);
			MTU_STOP
			;
			MOTOR_EXITATION = 0;
			sensor_led_flg = 0;
			now_x_coordinate = 0;
			now_y_coordinate = 0;
			x_direction = 0;
			y_direction = 1;
			break;
		} else if ((now_x_coordinate == x_goal && now_y_coordinate == y_goal)
				|| PORTC.PIDR.BIT.B6 == 0) {

//update_wall_information(now_x_coordinate, now_y_coordinate);
			update_step_map_queue(x_goal, y_goal);
//trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45,0.2, 1.0);

			shortest_path[shortest_path_count] = STRAIGHT;
			shortest_path_count++;
			shortest_path[shortest_path_count] = 4;
			shortest_path_count++;
			MTU_STOP
			;
			return;
		} else if (0 == direction) {
			shortest_straight();
		} else if (1 == direction) {
			shortest_right_slalom();
		} else if (3 == direction) {
			shortest_left_slalom();
		} else {
			shortest_turn();
		}
	}
	all_wall_flag_remove();
}

void contest_shortest_big() {
	map_break_flag = 0;
	mode_count = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
//最短スラローム最低パラメータ
		led(1);
		__delay_ms(1000);
//最短経路導出//
		slalom_adachi_shortest_path(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_slalom_shortest_path();
//最短経路導出終了//

//					PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//					start_switch();
//					for (i = 0; i <= 255; i++)
//						myprintf("saitan =%d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//					start_switch();
//					__delay_ms(500);

//最短走行//

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_slalom_shortest_run(1.6, 2.4, 2.8);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 1) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//
//		slalom_adachi_shortest_path(TARGET_X_COORDINATE,
//		TARGET_Y_COORDINATE);

		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();
//最短経路導出終了//

//							PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//							start_switch();
//							for (i = 0; i <= 255; i++)
//								myprintf("saitan = %d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//							start_switch();
//							__delay_ms(500);
//最短走行//

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_big_slalom_shortest_run(1.6, 2.24, 2.4, 1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 2) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//
		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();
//最短経路導出終了//

//							PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//							start_switch();
//							for (i = 0; i <= 255; i++)
//								myprintf("saitan = %d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//							start_switch();
//							__delay_ms(500);
//最短走行//

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_big_slalom_shortest_run(1.8, 2.4, 2.4, 1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 3) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//
		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();
//最短経路導出終了//

//							PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//							start_switch();
//							for (i = 0; i <= 255; i++)
//								myprintf("saitan = %d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//							start_switch();
//							__delay_ms(500);
//最短走行//

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_big_slalom_shortest_run(1.7, 2.4, 2.4, 1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 4) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//
		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_big_slalom_shortest_run(1.8, 2.6, 2.8, 1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 5) {
		//最短スラローム
		led(1);
		__delay_ms(1000);
		//最短経路導出//
		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;

		renew_big_slalom_shortest_run(2.0, 2.4, 3.2, 1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
		//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

		//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 6) {
		//最短スラローム
		led(1);
		__delay_ms(1000);
		//最短経路導出//
		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;

		renew_big_slalom_shortest_run(2.0, 2.4, 3.0, 2);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
		//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

		//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 7) {
		//最短スラローム
		led(1);
		__delay_ms(1000);
		//最短経路導出//
		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_big_slalom_shortest_path();

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;

		renew_big_slalom_shortest_run(2.0, 2.6, 3.0, 2);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
		//最短走行終了//
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

		//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else {

	}
}

void contest_shortest_oblique() {
	int i;
	map_break_flag = 0;
	mode_count = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//

		make_shortest_path_by_umuo_method(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);
		renew_oblique_big_slalom_shortest_path();
//最短経路導出終了//

//					PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//					start_switch();
//					for (i = 0; i <= 255; i++)
//						myprintf("saitan =%d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//					start_switch();
//					__delay_ms(500);
//最短走行//

		PORTA.PODR.BIT.B1 = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_oblique_big_slalom_shortest_run(1.2, 2.0, 2.0, 2);
		MTU_STOP
		;
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1) {
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			return;
		}
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;
		__delay_ms(100);
		MOTOR_EXITATION = 0;
	} else if (mode_count == 1) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//
		slalom_adachi_shortest_path(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_oblique_big_slalom_shortest_path();
//最短経路導出終了//

//					PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//					start_switch();
//					for (i = 0; i <= 255; i++)
//						myprintf("saitan =%d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//					start_switch();
//					__delay_ms(500);
//最短走行//

		PORTA.PODR.BIT.B1 = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_oblique_big_slalom_shortest_run(1.4, 2.0, 2.0, 2);
		MTU_STOP
		;
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1) {
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			return;
		}
//最短走行終了//
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;
	} else if (mode_count == 2) {
//最短スラローム
		led(1);
		__delay_ms(1000);
//最短経路導出//
		slalom_adachi_shortest_path(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_oblique_big_slalom_shortest_path();
//最短経路導出終了//

//					PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
//					start_switch();
//					for (i = 0; i <= 255; i++)
//						myprintf("saitan =%d\n", renew_shortest_path[i]);
//↑パス表示デバッグ
//					start_switch();
//					__delay_ms(500);
//最短走行//

		PORTA.PODR.BIT.B1 = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_oblique_big_slalom_shortest_run(1.4, 2.2, 2.2, 2);
		MTU_STOP
		;
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1) {
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			return;
		}
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	} else if (mode_count == 3) {
//最短スラローム
		led(1);

		__delay_ms(1000);
//最短経路導出//
		slalom_adachi_shortest_path(TARGET_X_COORDINATE,
		TARGET_Y_COORDINATE);

		renew_oblique_big_slalom_shortest_path();
//最短経路導出終了//

		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
//↓パス表示デバッグ
		/*
		 start_switch();
		 for (i = 0; i <= 255; i++)
		 myprintf("saitan =%d\n", renew_shortest_path[i]);
		 //↑パス表示デバッグ
		 start_switch();
		 __delay_ms(500);
		 */
//最短走行//
		PORTA.PODR.BIT.B1 = 1;		//励磁をkiru
		sensor_led_flg = 1;
//		slalom_shortest_run(0.6, 2.5);
		renew_oblique_big_slalom_shortest_run(1.6, 2.4, 2.4, 2);
		MTU_STOP
		;
		PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return;
//最短走行終了//
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		MOTOR_EXITATION = 1;
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

//重ね探索

		__delay_ms(500);

		sensor_led_flg = 1;

		MOTOR_EXITATION = 1;
		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1) {
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			return;
		}
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		update_direction(1);

		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
		MTU_STOP
		;

	}
}

//最短経路導出
void shortest_left_turn() {
	led(4);

	update_direction(0);
	update_coordinate();

//	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15,	2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

//	trapezoidal_acceleration_left_rotate(90, 0.3, 0.15, 0.15, 2.00);
	shortest_path[shortest_path_count] = LEFT_TURN;
	shortest_path_count++;

//	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45, 2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

}
void shortest_right_turn() {
	led(1);

	update_direction(2);
	update_coordinate();

//	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15,	2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

//	trapezoidal_acceleration_right_rotate(90, 0.3, 0.15, 0.15, 2.00);
	shortest_path[shortest_path_count] = RIGHT_TURN;
	shortest_path_count++;

//	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45,	2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

}
void shortest_straight() {
	led(2);

	update_direction(3);
	update_coordinate();

//trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, 0.45, 0.45, 0.45, 2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

}
void shortest_turn() {
	led(7);

	update_direction(1);
	update_coordinate();

//	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15,	2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

	shortest_path[shortest_path_count] = TURN;
	shortest_path_count++;

	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;

}
void shortest_straight_slalom() {
	led(2);

	update_direction(3);
	update_coordinate();

//trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, 0.45, 0.45, 0.45, 2.0);
	shortest_path[shortest_path_count] = STRAIGHT;
	shortest_path_count++;
}
void shortest_left_slalom() {
	led(4);

	update_direction(0);
	update_coordinate();

//	small_slalom(left_parameter_1);
	shortest_path[shortest_path_count] = LEFT_TURN;
	shortest_path_count++;

}
void shortest_right_slalom() {
	led(1);

	update_direction(2);
	update_coordinate();

//	small_slalom(left_parameter_3);
	shortest_path[shortest_path_count] = RIGHT_TURN;
	shortest_path_count++;

}
