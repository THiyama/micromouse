/*
 * search.c
 *
 *  Created on: 2016/09/11
 *      Author: PCーUSER
 */
#include "search.h"

#define QUEUE_MAX 200
/*
 signed char north[16][16];
 signed char east[16][16];
 signed char buff_north[16][16];
 signed char buff_east[16][16];
 signed char north_flag[16][16];
 signed char east_flag[16][16];
 signed char buff_north_flag[16][16];
 signed char buff_east_flag[16][16];*/

unsigned char wall_state[16][16];

unsigned char step_map[16][16];
extern unsigned char step_map_queue[16][16];

//extern unsigned char shortest_path[1000];
extern unsigned char renew_shortest_path[1000];
extern char map_break_flag;

char known_interval_goal_flag = 0;

//queue関連
int tail_x, tail_y;
int head_x, head_y;
signed char Q_x[QUEUE_MAX], Q_y[QUEUE_MAX];

/*
 void left_hand_method(int left_wall_distance, int right_wall_distance,
 int left_forward_wall_distance, int right_forward_wall_distance) {

 if (left_wall_distance <= LEFT_WALL_THRESHOLD) {
 trapezoidal_acceleration_straight(BLOCK_LENGTH*0.5, 0.3, 0.3, 0.2, 1.0);
 __delay_ms(100);
 trapezoidal_acceleration_left_rotate(90, 0.3, 0.2, 0.2, 1.00);
 __delay_ms(100);
 trapezoidal_acceleration_straight(BLOCK_LENGTH*0.5, 0.3, 0.2, 0.3, 1.0);
 }
 else if (left_forward_wall_distance <= LEFT_FORWARDWALL_THRESHOLD
 && right_forward_wall_distancetance <= RIGHT_FORWARDWALL_THRESHOLD) {
 trapezoidal_acceleration_straight(BLOCK_LENGTH*90, 0.3, 0.3, 0.3, 3.00);
 }
 else if (right_wall_distance <= RIGHT_WALL_THRESHOLD) {
 trapezoidal_acceleration_straight(BLOCK_LENGTH*0.5, 0.3, 0.3, 0.2, 1.0);
 __delay_ms(100);
 trapezoidal_acceleration_right_rotate(90, 0.5, 0.2, 0.2, 1.00);
 __delay_ms(100);
 trapezoidal_acceleration_straight(BLOCK_LENGTH*0.5, 0.3, 0.2, 0.3, 1.0);
 }
 else {
 trapezoidal_acceleration_right_rotate(180, 0.3, 0.2, 0.2, 2.0);
 __delay_ms(100);
 }
 }
 */
void left_hand(unsigned char x_goal, unsigned char y_goal) {

	x_direction = 0;
	y_direction = 1;

	//探索用変数群の初期化
	search_init(x_goal, y_goal);

	//半区間進む
	trapezoidal_acceleration_straight_distance(
	BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45, 2.0, 4.0);

	//座標を(0,1)に更新
	update_coordinate();

	while (1) {
		if (now_x_coordinate == x_goal && now_y_coordinate == y_goal) {

			update_wall_information(now_x_coordinate, now_y_coordinate);
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15, 2.0, 4.0);

			trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45,
					0.2, 1.0);
			MTU_STOP
			;
			return;
		}
		if (ad_data[1] <= LEFT_WALL_THRESHOLD) {
			search_left_turn();
		} else if (ad_data[0] <= LEFT_FORWARDWALL_THRESHOLD
				&& ad_data[4] <= RIGHT_FORWARDWALL_THRESHOLD) {
			search_straight();
		} else if (ad_data[3] <= RIGHT_WALL_THRESHOLD) {
			search_right_turn();
		} else {
			search_turn();
		}
		/*
		 myprintf("%d,%d\n", x_direction, y_direction);
		 myprintf("%d,", now_x_coordinate);
		 myprintf("%d\n", now_y_coordinate);
		 */
	}
}

void adachi_method(unsigned char x_goal, unsigned char y_goal) {

	int direction;

//	update_wall_information(now_x_coordinate, now_y_coordinate);

//半区間進む
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45,
			2.0);
	//座標を(0,1)に更新
	update_coordinate();

	while (1) {
		update_wall_information(now_x_coordinate, now_y_coordinate);	//壁生やす
		update_step_map_queue(x_goal, y_goal);	//壁情報より歩数マップを更新する

		direction = should_move_on_direction();

		if ((now_x_coordinate == x_goal && now_y_coordinate == y_goal)
				|| PORTC.PIDR.BIT.B6 == 0) {

			update_wall_information(now_x_coordinate, now_y_coordinate);
			update_step_map_queue(x_goal, y_goal);
			trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45,
					0.2, 1.0);
			MTU_STOP
			;
			return;
		} else if (0 == direction) {
			search_straight();
		} else if (1 == direction) {
			search_right_turn();
		} else if (3 == direction) {
			search_left_turn();
		} else {
			search_turn();
		}
	}
}
void slalom_adachi_method(unsigned char x_goal, unsigned char y_goal) {

	int direction;
	double init = 0.0;
	sensor_led_flg = 1;
//	update_wall_information(now_x_coordinate, now_y_coordinate);

	MOTOR_EXITATION = 1;
	caluculate_count = 0;
//半区間進む
	if (x_goal == TARGET_X_COORDINATE && y_goal == TARGET_Y_COORDINATE) {
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.15, 0.45, 2.0, 4.0);

	} else {
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45, 2.0, 4.0);

	}

	//座標を(0,1)に更新
	update_coordinate();

	while (1) {
		map_break_flag = 0;
		update_wall_information(now_x_coordinate, now_y_coordinate);	//壁生やす
		update_step_map_queue(x_goal, y_goal);	//壁情報より歩数マップを更新する

		direction = should_move_on_direction();

		if (step_map_queue[now_x_coordinate][now_y_coordinate] == 255) {
			trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5, 0.45,
					0.45, 0.15, 2.0, 2.0);

			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			sensor_led_flg = 0;
			led(1);
			while (PORTC.PIDR.BIT.B6 == 1)
				;

			read_buff_wall_information(x_goal, y_goal);

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

			read_buff_wall_information(x_goal, y_goal);

			return;
		} else if ((now_x_coordinate == x_goal && now_y_coordinate == y_goal)) {

			update_wall_information(now_x_coordinate, now_y_coordinate);
			update_step_map_queue(x_goal, y_goal);
			save_now_wall_information();
			caluculate_mode = 0;
			caluculate_count = 0;

			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15, 2.0, 4.0);

			MTU_STOP
			;
			__delay_ms(100);
			return;
		} else if (0 == direction) {
			search_straight();
		} else if (1 == direction) {
			search_right_slalom();
		} else if (3 == direction) {
			search_left_slalom();
		} else if (4 == direction) {
			search_turn_blind_alley();
		} else {
			search_turn();
		}
	}
}

void slalom_furukawa_adachi_method(unsigned char x_goal, unsigned char y_goal) {

	int direction;
	int f_direction;
	double init = 0.0;
	int i;
	int known_path = 0;

	sensor_led_flg = 1;
//	update_wall_information(now_x_coordinate, now_y_coordinate);

	caluculate_count = 0;
	MOTOR_EXITATION = 1;
//半区間進む
	if (x_goal == TARGET_X_COORDINATE && y_goal == TARGET_Y_COORDINATE) {
		caluculate_count = -50;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.10, 0.45, 1.5, 4.0);

	} else {
		if (TARGET_X_COORDINATE == 7 && TARGET_Y_COORDINATE == 7
				&& a_wall_read(7, 7, 3) == 0 && a_wall_read(7, 7, 2) == 0) {
			caluculate_count = -50;
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5, 0.45, 0.10, 0.45, 1.5, 4.0);

		} else {
			counter_trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.3, 0.15, 0.10, 0.1, 1.5, 4.0);
			MTU_STOP
			;
			__delay_ms(200);
			caluculate_count = -50;
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.10, 0.45, 1.5, 4.0);
		}
	}

	//座標を(0,1)に更新
	update_coordinate();

	while (1) {
		map_break_flag = 0;
		update_wall_information(now_x_coordinate, now_y_coordinate);	//壁生やす
		add_all_unknown_wall(); // 見えない壁がみんとには見えるらしい
		//add_three_wall_coordinate(now_x_coordinate, now_y_coordinate); //壁が3枚とも入っている区画に壁を入れる
		update_step_map_queue(x_goal, y_goal);	//壁情報より歩数マップを更新する

		direction = should_move_on_direction();

		known_path = make_known_path(now_x_coordinate, now_y_coordinate,
				abs_compass_convert(0), x_goal, y_goal);

		if (step_map_queue[now_x_coordinate][now_y_coordinate] == 255) {
			trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5, 0.45,
					0.45, 0.15, 2.0, 2.0);
			MTU_STOP
			;
			__delay_ms(100);
			MOTOR_EXITATION = 0;
			sensor_led_flg = 0;
			led(1);
			while (PORTC.PIDR.BIT.B6 == 1)
				;

			read_buff_wall_information(x_goal, y_goal);

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

			read_buff_wall_information(x_goal, y_goal);

			return;
		} else if (known_path != 0) {
			//既知区間加速
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * known_path, 0.75, 0.45, 0.45, 3.0, 3.0);
			/*if(known_interval_goal_flag == 1){
			 update_coordinate();
			 }*/
			known_interval_goal_flag = 0;
		} else if (now_x_coordinate == x_goal && now_y_coordinate == y_goal) {

			update_wall_information(now_x_coordinate, now_y_coordinate);
			update_step_map_queue(x_goal, y_goal);
			save_now_wall_information();
			trapezoidal_acceleration_straight_distance(
			BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15, 2.0, 4.0);

			MTU_STOP
			;
			__delay_ms(100);
			return;
		} else if (0
				!= is_unsearched_area(now_x_coordinate, now_y_coordinate,
						abs_compass_convert(0))
				&& a_wall_read(now_x_coordinate, now_y_coordinate,
						abs_compass_convert(0)) == 0) {
			search_straight();
		} else if (0
				!= is_unsearched_area(now_x_coordinate, now_y_coordinate,
						abs_compass_convert(1))
				&& a_wall_read(now_x_coordinate, now_y_coordinate,
						abs_compass_convert(1)) == 0) {
			search_right_slalom();
		} else if (0
				!= is_unsearched_area(now_x_coordinate, now_y_coordinate,
						abs_compass_convert(3))
				&& a_wall_read(now_x_coordinate, now_y_coordinate,
						abs_compass_convert(3)) == 0) {
			search_left_slalom();
		} else if (0 == direction) {
			search_straight();
		} else if (1 == direction) {
			search_right_slalom();
		} else if (3 == direction) {
			search_left_slalom();
		} else if (4 == direction) {
			search_turn_blind_alley();
		} else {
			search_turn();
		}
	}

}

//壁が3枚とも入っている区画に壁を入れる関数
//バグあり
void add_three_wall_coordinate(int x_coordinate, int y_coordinate) {
	int i, j;
	int buff = 0;
	int x = 0, y = 0;
	char wall_count = 0;

	for (i = 0; i < 8; i++) {
		if (i == 0) {
			x = 0;
			y = 1;
		} else if (i == 1) {
			x = 1;
			y = 1;
		} else if (i == 2) {
			x = 1;
			y = 0;
		} else if (i == 3) {
			x = 1;
			y = -1;
		} else if (i == 4) {
			x = 0;
			y = -1;
		} else if (i == 5) {
			x = -1;
			y = -1;
		} else if (i == 6) {
			x = -1;
			y = 0;
		} else if (i == 7) {
			x = -1;
			y = 1;
		}

		if (x_coordinate + x == 0 && y_coordinate + y == 0) {
			return;
		}
		if (x_coordinate + x == TARGET_X_COORDINATE
				&& y_coordinate + y == TARGET_Y_COORDINATE) {
			return;
		}

		for (j = 0; j < 4; j++) {
			if (a_wall_read(x_coordinate + x, y_coordinate + y, j)) {
				wall_count++;
			} else {
				buff = j;
			}
		}
		if (wall_count == 3) {
			a_wall_add_kai(x_coordinate + x, y_coordinate + y, buff);
		}
	}
}

//今居る座標を中心とした場合の4つの柱の周りの壁を入れる関数
/*
 * 引数はなし、今いる座標を勝手に取得。
 * 基本、壁情報を取得した直後、歩数マップを展開する前に呼ぶ
 * 絶対座標を基準に考える
 */
void add_all_unknown_wall() {
	int i = 0;

	for (i = 0; i < 4; i++) {
		add_unknown_wall_by_pillar(now_x_coordinate, now_y_coordinate, i);
	}
}

//柱の3辺に壁が入っていない場合、残りの1辺に壁を入れる関数
/*
 * 座標の引数は中心座標
 */
void add_unknown_wall_by_pillar(int x_coordinate, int y_coordinate,
		int direction) {

	int x = 0, y = 0;
	int wall_coordinate_x = 0, wall_coordinate_y;
	int wall_1 = 0, wall_2 = 0;

	direction %= 4;

	if (direction == 0) {
		x = 1;
		y = 1;
		wall_1 = 3;
		wall_2 = 2;
	} else if (direction == 1) {
		x = 1;
		y = -1;
		wall_1 = 3;
		wall_2 = 0;
	} else if (direction == 2) {
		x = -1;
		y = -1;
		wall_1 = 1;
		wall_2 = 0;
	} else if (direction == 3) {
		x = -1;
		y = 1;
		wall_1 = 1;
		wall_2 = 2;
	}

	wall_coordinate_x = x_coordinate + x;
	wall_coordinate_y = y_coordinate + y;

	if ((wall_coordinate_x < 0) || (wall_coordinate_x > 15)) {
		//範囲外アクセス防止
		return;
	}
	if ((wall_coordinate_y < 0) || (wall_coordinate_y > 15)) {
		//範囲外アクセス防止
		return;
	}

	if (a_wall_read(x_coordinate, y_coordinate, direction) == 0
			&& a_wall_read(x_coordinate, y_coordinate, (direction + 1) % 4)
					== 0) {
		//ある壁と、その時計回りの壁がないなら
		if (a_wall_flag_read(wall_coordinate_x, wall_coordinate_y, wall_1) == 1
				&& a_wall_read(wall_coordinate_x, wall_coordinate_y, wall_1)
						== 0) {
			//斜め先の区間、時計回りの壁が探索済みで、かつ、壁がないなら
			//斜め先の区間、反時計周りの壁を探索済みにし、かつ壁を生やす
			a_wall_add_kai(wall_coordinate_x, wall_coordinate_y, wall_2);
			a_wall_flag_add_kai(wall_coordinate_x, wall_coordinate_y, wall_2);
		}

		else if (a_wall_flag_read(wall_coordinate_x, wall_coordinate_y, wall_2)
				== 1
				&& a_wall_read(wall_coordinate_x, wall_coordinate_y, wall_2)
						== 0) {
			//斜め先の区間、反対時計回りの壁が探索済みで、かつ、壁がないなら
			//斜め先の区間、時計周りの壁を探索済みにし、かつ壁を生やす
			a_wall_add_kai(wall_coordinate_x, wall_coordinate_y, wall_1);
			a_wall_flag_add_kai(wall_coordinate_x, wall_coordinate_y, wall_1);
		}

		else {
			//全てに当てはまらないなら
			return;
		}

	} else {
		return;
	}
}
//任意の座標の周囲が未探索なら１を返す関数
int is_unsearched_area(int x_coordinate, int y_coordinate, int now_direction) {

	int i, j;
	now_direction %= 4;
	if (now_direction == 0) {
		for (j = 0; j < 4; j++) {
			if (a_wall_flag_read(x_coordinate, y_coordinate + 1, j) == 0) {
				return 1;
			}
		}
	} else if (now_direction == 1) {
		for (j = 0; j < 4; j++) {
			if (a_wall_flag_read(x_coordinate + 1, y_coordinate, j) == 0) {
				return 1;
			}
		}
	} else if (now_direction == 2) {
		for (j = 0; j < 4; j++) {
			if (a_wall_flag_read(x_coordinate, y_coordinate - 1, j) == 0) {
				return 1;
			}
		}
	} else if (now_direction == 3) {
		for (j = 0; j < 4; j++) {
			if (a_wall_flag_read(x_coordinate - 1, y_coordinate, j) == 0) {
				return 1;
			}
		}
	} else {
		return 0;
	}

	return 0;

}

int should_move_on_direction() {
	volatile int forward_step = 255;
	volatile int right_step = 255;
	volatile int left_step = 255;
	volatile int now_step;
	volatile int wall_flg = 0;

	volatile signed char should_move_on_x_direction;
	volatile signed char should_move_on_y_direction;

	if (0
			== a_wall_read(now_x_coordinate, now_y_coordinate,
					abs_compass_convert(3))) { // 左壁がないなら
		should_move_on_x_direction = -1 * y_direction;
		should_move_on_y_direction = 1 * x_direction;
		left_step =
				step_map_queue[now_x_coordinate + should_move_on_x_direction][now_y_coordinate
						+ should_move_on_y_direction];
		wall_flg++;
	}
	if (0
			== a_wall_read(now_x_coordinate, now_y_coordinate,
					abs_compass_convert(0))) { // 前壁がないなら
		should_move_on_x_direction = 1 * x_direction;
		should_move_on_y_direction = 1 * y_direction;
		forward_step = step_map_queue[now_x_coordinate
				+ should_move_on_x_direction][now_y_coordinate
				+ should_move_on_y_direction];
		wall_flg++;
	}
	if (0
			== a_wall_read(now_x_coordinate, now_y_coordinate,
					abs_compass_convert(1))) { //右壁がないなら
		should_move_on_x_direction = 1 * y_direction;
		should_move_on_y_direction = -1 * x_direction;
		right_step = step_map_queue[now_x_coordinate
				+ should_move_on_x_direction][now_y_coordinate
				+ should_move_on_y_direction];
		wall_flg++;
	}
	now_step = step_map_queue[now_x_coordinate][now_y_coordinate];
	if (wall_flg == 0) {
		led(0);
		return 4;
	}
	if (forward_step == now_step - 1) {
		//現在の歩数より前方の歩数が1低いなら→0を返す。(前方に進む)
		led(2);
		return 0;
	} else if (right_step == now_step - 1) {
		led(1);
		return 1;
	} else if (left_step == now_step - 1) {
		led(4);
		return 3;
	} else {
		led(0);
		return 2;
	}
}

//既知区間加速

//未知区間が現れるまでパス生成をし、パスを走り、移動先での方向座標を更新する関数をまとめる関数
void known_interval_acceleration() {

}

//未知区間があるまでの区間を計算する関数
/*
 * 返り値は既知区間出来る数
 * 出来ないなら0が返ってくる
 */
int make_known_path(int x_coordinate, int y_coordinate, int now_direction,
		unsigned char x_goal, unsigned char y_goal) {
	int x = 0, y = 0;
	now_direction = now_direction % 4;

	while (1) {
		if (a_wall_read(x_coordinate + x, y_coordinate + y, now_direction)
				== 1) {
			break; //北壁があるならループを抜ける
		}
		if (can_do_known_interval_acceleration(x_coordinate + x,
				y_coordinate + y, now_direction) == 0) {
			//既知区間加速が出来ないなら
			break;
		}

		if (now_direction == 0) {
			y += 1;
		} else if (now_direction == 1) {
			x += 1;
		} else if (now_direction == 2) {
			y -= 1;
		} else if (now_direction == 3) {
			x -= 1;
		}

		update_coordinate();/*
		 if (now_x_coordinate == x_goal && now_y_coordinate == y_goal) {
		 now_x_coordinate -= x;
		 now_y_coordinate -= y;
		 known_interval_goal_flag = 1;
		 }*/
	}
	return (abs(x) + abs(y));
}

//その座標が既知区間加速が出来るかどうかを判定する関数
/*
 * 座標と方向を与えて、北の座標の周囲に未知がないなら1を返す
 * 北の座標の北の座標が未知区間で、北の座標の左右が既知区間、あるいは壁があるなら、2を返す
 * その座標の北壁(ひふみんとから見て)がないなら1を返す
 */
char can_do_known_interval_acceleration(int x_coordinate, int y_coordinate,
		int now_direction) {

	int adachi_direction = 0;
	adachi_direction = should_move_on_direction();

	now_direction = now_direction % 4;

	if (1
			== is_unsearched_area(x_coordinate, y_coordinate,
					(now_direction + 1) % 4)
			&& a_wall_read(x_coordinate, y_coordinate, (now_direction + 1) % 4)
					== 0) {
		//右が未知区間かつ右壁がないなら
		return 0;
	} else if (1
			== is_unsearched_area(x_coordinate, y_coordinate,
					(now_direction + 3) % 4)
			&& a_wall_read(x_coordinate, y_coordinate, (now_direction + 3) % 4)
					== 0) {
		//左が未知区間かつ左壁がないなら
		return 0;
	} else if (1
			== is_unsearched_area(x_coordinate, y_coordinate, now_direction)) {
		//前が未知区間なら
		return 0;
	} else if (adachi_direction != 0) {
		return 0;
	}
	return 1;
}
//その座標の周囲4箇所が未探索か判別する関数(北、東、南、西の順で優先,ひふみんとから見た方向を返す)
/*
 * 方向の引数はひふみんとから見た北壁の方向
 */
char distinct_unknown_area(int x_coordinate, int y_coordinate,
		int now_direction) {

	if (0
			!= is_unsearched_area(x_coordinate, y_coordinate,
					abs_compass_convert(now_direction))) {
		return now_direction; //ひふみんとから見た北を返す
	} else if (0
			!= is_unsearched_area(x_coordinate, y_coordinate,
					abs_compass_convert((now_direction + 1) % 4))) {
		return ((now_direction + 1) % 4); //ひふみんとから見た東を返す
	} else if (0
			!= is_unsearched_area(x_coordinate, y_coordinate,
					abs_compass_convert((now_direction + 2) % 4))) {
		return ((now_direction + 2) % 4); //ひふみんとから見た南を返す
	} else if (0
			!= is_unsearched_area(x_coordinate, y_coordinate,
					abs_compass_convert((now_direction + 3) % 4))) {
		return ((now_direction + 3) % 4); //ひふみんとから見た西を返す
	} else {
		return 4;
	}
}

//その座標が未知区間かどうかを判定する関数
/*未知区間でないなら0を返す。未知区間なら1を返す。*/
char get_unknown_interval(int x_coordinate, int y_coordinate) {
	int i;
	for (i = 0; i < 4; i++) {
		if (a_wall_flag_read(x_coordinate, y_coordinate, i) == 0) {
			return 1;
		}
	}
	return 0;
}

//既知区間パスを走る関数
void run_known_interval() {

}

//移動先での方向座標を更新する関数
void update_after_known_interval() {

}
//既知区間パスを走った後に方向を更新する関数
void update_after_run_direction() {

}
//既知区間パスを走った後に座標を更新する関数
void update_after_run_coordinate() {

}

void update_wall_information(char x_coordinate, char y_coordinate) {

//方向の翻訳はa_wall_addで行っている

	if (ad_data[1] >= LEFT_WALL_THRESHOLD) { // 左壁
		a_wall_add(x_coordinate, y_coordinate, 3);
	}
	if (ad_data[2] >= FORWARDWALL_THRESHOLD) { // 前壁
		a_wall_add(x_coordinate, y_coordinate, 0);
	}
	if (ad_data[3] >= RIGHT_WALL_THRESHOLD) { //右壁
		a_wall_add(x_coordinate, y_coordinate, 1);
	}

	a_wall_flag_add(x_coordinate, y_coordinate, 0);
	a_wall_flag_add(x_coordinate, y_coordinate, 1);
	a_wall_flag_add(x_coordinate, y_coordinate, 3);
//	a_wall_add();
//	update_step_map_queue();
}

void update_step_map() {

	int i;
	signed char x, y;

	for (i = 0; i <= 255; i++) {
		for (y = 15; y >= 0; y--) {
			for (x = 0; x <= 15; x++) {

				if (step_map[x][y] == i) {
					if (step_map[x][y - 1] > i && y > 0
							&& a_wall_read(x, y, 2) == 0) {
						step_map[x][y - 1] = i + 1;
					}
					if (step_map[x][y + 1] > i && y < 15
							&& a_wall_read(x, y, 0) == 0) {
						step_map[x][y + 1] = i + 1;
					}
					if (step_map[x - 1][y] > i && x > 0
							&& a_wall_read(x, y, 3) == 0) {
						step_map[x - 1][y] = i + 1;
					}
					if (step_map[x + 1][y] > i && x < 15
							&& a_wall_read(x, y, 1) == 0) {
						step_map[x + 1][y] = i + 1;
					}
				}
			}
		}
	}
}
void search_left_turn() {
	led(4);

	update_direction(0);
	update_coordinate();

	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15,
			2.0);
	MTU_STOP
	;
	__delay_ms(BREAK_TIME);
	trapezoidal_acceleration_left_rotate(90, 0.3, 0.15, 0.15, 2.00);
	MTU_STOP
	;
	__delay_ms(BREAK_TIME);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45,
			2.0);

}

void search_right_turn() {
	led(1);

	update_direction(2);
	update_coordinate();

	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15,
			2.0);
	MTU_STOP
	;
	__delay_ms(BREAK_TIME);
	trapezoidal_acceleration_right_rotate(90, 0.3, 0.15, 0.15, 2.00);
	MTU_STOP
	;
	__delay_ms(BREAK_TIME);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 0.5, 0.45, 0.15, 0.45,
			2.0);

}

void search_straight() {
	led(2);

	update_direction(3);
	update_coordinate();

	trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1, 0.45, 0.45,
			0.45, 2.0, 4.0);

}

void search_turn() {
	led(7);

	trapezoidal_acceleration_straight_distance(
	BLOCK_LENGTH * 0.5, 0.45, 0.45, 0.15, 2.0, 8.0);

	MTU_STOP
	;
	__delay_ms(BREAK_TIME);

	if (ad_data[1] > LEFT_WALL_REFERENCE && ad_data[3] < RIGHT_WALL_REFERENCE) {
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
	} else if (ad_data[3] > RIGHT_WALL_REFERENCE
			&& ad_data[1] < LEFT_WALL_REFERENCE) {
		trapezoidal_acceleration_left_rotate(180, 0.3, 0.15, 0.15, 2.0);
	} else {
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
	}
	MTU_STOP
	;
	__delay_ms(BREAK_TIME);

	if(a_wall_read(now_x_coordinate,now_y_coordinate,abs_compass_convert(0)) == 1){
		counter_trapezoidal_acceleration_straight_distance(0.1, 0.15, 0.15, 0.15,
				3.0, 3.0);
		MTU_STOP
		;
		__delay_ms(BREAK_TIME);
		caluculate_count = 0;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.10, 0.45, 1.3, 4.0);

	} else {
		caluculate_count = 0;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5, 0.45, 0.10, 0.45, 1.3, 4.0);
	}


	update_direction(1);
	update_coordinate();


}

void search_turn_blind_alley() {
	double buff_wall_control_constant;
	led(7);

	update_direction(1);
	update_coordinate();

	buff_wall_control_constant = wall_control_constant;
	wall_control_constant = 0.0;
	trapezoidal_acceleration_straight_distance(
	BLOCK_LENGTH * 0.2, 0.45, 0.45, 0.15, 2.0, 8.0);

	while (1) {
		trapezoidal_acceleration_straight_distance(0.001, 0.15, 0.15, 0.15, 2.0,
				8.0);
		if (ad_data[2] > 290)
			break;
	}
	trapezoidal_acceleration_straight_distance(0.05, 0.15, 0.15, 0.15, 2.0,
			8.0);

	MTU_STOP
	;
	__delay_ms(BREAK_TIME);

	if (ad_data[1] > LEFT_WALL_REFERENCE && ad_data[3] < RIGHT_WALL_REFERENCE) {
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
	} else if (ad_data[3] > RIGHT_WALL_REFERENCE
			&& ad_data[1] < LEFT_WALL_REFERENCE) {
		trapezoidal_acceleration_left_rotate(180, 0.3, 0.15, 0.15, 2.0);
	} else {
		trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15, 2.0);
	}
	MTU_STOP
	;
	__delay_ms(BREAK_TIME);
	counter_trapezoidal_acceleration_straight_distance(0.1, 0.15, 0.15, 0.15,
			3.0, 3.0);
	MTU_STOP
	;
	__delay_ms(BREAK_TIME + 200);

	caluculate_count = 0;
	trapezoidal_acceleration_straight_distance(
	BLOCK_LENGTH * 0.5 + 0.032, 0.45, 0.10, 0.45, 1.3, 4.0);

	wall_control_constant = buff_wall_control_constant;

}

void search_left_slalom() {
	led(4);

	update_direction(0);
	update_coordinate();

	small_slalom(small_left_parameter_1);

}

void search_right_slalom() {
	led(1);

	update_direction(2);
	update_coordinate();

	small_slalom(small_right_parameter_1);

}

void a_wall_add(char wall_x_coordinate, char wall_y_coordinate, char compass) {

//	compass = compass_convert(compass);

	compass = abs_compass_convert(compass);

	if (compass == 0) {  //kita
		if (wall_y_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_NORTH_WALL;
	} else if (compass == 1) { //higasi
		if (wall_x_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_EAST_WALL;
	} else if (compass == 2) { //minami
		if (wall_y_coordinate == 0)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate - 1] |= 1
				<< B_NORTH_WALL;
	} else if (compass == 3) { //nisi
		if (wall_x_coordinate == 0)
			return;
		wall_state[wall_x_coordinate - 1][wall_y_coordinate] |=
				1 << B_EAST_WALL;
	}
}
void a_wall_add_kai(char wall_x_coordinate, char wall_y_coordinate,
		char compass) {

//	compass = compass_convert(compass);

	if (compass == 0) {  //kita
		if (wall_y_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_NORTH_WALL;
	} else if (compass == 1) { //higasi
		if (wall_x_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_EAST_WALL;
	} else if (compass == 2) { //minami
		if (wall_y_coordinate == 0)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate - 1] |= 1
				<< B_NORTH_WALL;
	} else if (compass == 3) { //nisi
		if (wall_x_coordinate == 0)
			return;
		wall_state[wall_x_coordinate - 1][wall_y_coordinate] |=
				1 << B_EAST_WALL;
	}
}

void a_wall_remove(char wall_x_coordinate, char wall_y_coordinate, char compass) {

//	compass = compass_convert(compass);

	compass = abs_compass_convert(compass);

	if (compass == 0) {  //kita
		if (wall_y_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] &= 0xff
				& (0 << B_NORTH_WALL);
	} else if (compass == 1) { //higasi
		if (wall_x_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] &= 0xff
				& (0 << B_EAST_WALL);
	} else if (compass == 2) { //minami
		if (wall_y_coordinate == 0)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate - 1] &= 0xff
				& (0 << B_NORTH_WALL);
	} else if (compass == 3) { //nisi
		if (wall_x_coordinate == 0)
			return;
		wall_state[wall_x_coordinate - 1][wall_y_coordinate] &= 0xff
				& (0 << B_EAST_WALL);
	}
}

char a_wall_read(char wall_x_coordinate, char wall_y_coordinate, char compass) {

	if (compass == 0) {
		if (wall_y_coordinate == 15) {
			return 1;
		}
		return READ_WALL_DATA(wall_state[wall_x_coordinate][wall_y_coordinate],
				B_NORTH_WALL);
	} else if (compass == 1) {
		if (wall_x_coordinate == 15) {
			return 1;
		}
		return READ_WALL_DATA(wall_state[wall_x_coordinate][wall_y_coordinate],
				B_EAST_WALL);
	} else if (compass == 2) {
		if (wall_y_coordinate == 0) {
			return 1;
		}
		return READ_WALL_DATA(
				wall_state[wall_x_coordinate][wall_y_coordinate - 1],
				B_NORTH_WALL);
	} else if (compass == 3) {
		if (wall_x_coordinate == 0) {
			return 1;
		}
		return READ_WALL_DATA(
				wall_state[wall_x_coordinate - 1][wall_y_coordinate],
				B_EAST_WALL);
	} else
		return 0;
}

void a_wall_flag_add(char wall_x_coordinate, char wall_y_coordinate,
		char compass) {

//	compass = compass_convert(compass);

	compass = abs_compass_convert(compass);

	if (compass == 0) {  //kita
		if (wall_y_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_NORTH_FLAG;

	} else if (compass == 1) { //higasi
		if (wall_x_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_EAST_FLAG;
	} else if (compass == 2) { //minami
		if (wall_y_coordinate == 0)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate - 1] |= 1
				<< B_NORTH_FLAG;
	} else if (compass == 3) { //nisi
		if (wall_x_coordinate == 0)
			return;
		wall_state[wall_x_coordinate - 1][wall_y_coordinate] |=
				1 << B_EAST_FLAG;
	}
}
void a_wall_flag_add_kai(char wall_x_coordinate, char wall_y_coordinate,
		char compass) {

//	compass = compass_convert(compass);
	if (compass == 0) {  //kita
		if (wall_y_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_NORTH_FLAG;

	} else if (compass == 1) { //higasi
		if (wall_x_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] |= 1 << B_EAST_FLAG;
	} else if (compass == 2) { //minami
		if (wall_y_coordinate == 0)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate - 1] |= 1
				<< B_NORTH_FLAG;
	} else if (compass == 3) { //nisi
		if (wall_x_coordinate == 0)
			return;
		wall_state[wall_x_coordinate - 1][wall_y_coordinate] |=
				1 << B_EAST_FLAG;
	}
}

void a_wall_flag_remove(char wall_x_coordinate, char wall_y_coordinate,
		char compass) {

//	compass = compass_convert(compass);

	compass = abs_compass_convert(compass);

	if (compass == 0) {  //kita
		if (wall_y_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] &= 0xff
				& (0 << B_NORTH_FLAG);
	} else if (compass == 1) { //higasi
		if (wall_x_coordinate == 15)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate] &= 0xff
				& (0 << B_EAST_FLAG);
	} else if (compass == 2) { //minami
		if (wall_y_coordinate == 0)
			return;
		wall_state[wall_x_coordinate][wall_y_coordinate - 1] &= 0xff
				& (0 << B_NORTH_FLAG);
	} else if (compass == 3) { //nisi
		if (wall_x_coordinate == 0)
			return;
		wall_state[wall_x_coordinate - 1][wall_y_coordinate] &= 0xff
				& (0 << B_EAST_FLAG);
	}
}

char min_three_value(int forward_step, int right_step, int left_step) {

	int min;

	min = forward_step;

	if (right_step < min) {
		min = right_step;
	}
	if (left_step < min) {
		min = left_step;
	}

	if (min == forward_step) {
		return 0;
	} else if (min == right_step) {
		return 1;
	} else if (min == left_step) {
		return 3;
	}
	return 4;
}

char a_wall_flag_read(char wall_x_coordinate, char wall_y_coordinate,
		char compass) {

	if (compass == 0) {
		if (wall_y_coordinate == 15) {
			return 1;
		}
		return READ_WALL_DATA(wall_state[wall_x_coordinate][wall_y_coordinate],
				B_NORTH_FLAG);
	} else if (compass == 1) {
		if (wall_x_coordinate == 15) {
			return 1;
		}
		return READ_WALL_DATA(wall_state[wall_x_coordinate][wall_y_coordinate],
				B_EAST_FLAG);
	} else if (compass == 2) {
		if (wall_y_coordinate == 0) {
			return 1;
		}
		return READ_WALL_DATA(
				wall_state[wall_x_coordinate][wall_y_coordinate - 1],
				B_NORTH_FLAG);
	} else if (compass == 3) {
		if (wall_x_coordinate == 0) {
			return 1;
		}
		return READ_WALL_DATA(
				wall_state[wall_x_coordinate - 1][wall_y_coordinate],
				B_EAST_FLAG);
	} else {
		return 0;
	}
}

void all_wall_flag_add() {
	int x, y;
	for (x = 0; x < 16; x++) {
		for (y = 0; y < 16; y++) {
			if (a_wall_flag_read(x, y, 0) == 0) {
				a_wall_add(x, y, 0);
			}
			if (a_wall_flag_read(x, y, 1) == 0) {
				a_wall_add(x, y, 1);
			}
		}
	}
}

void all_wall_flag_remove() {
	int x, y;
	for (x = 0; x < 16; x++) {
		for (y = 0; y < 16; y++) {
			if (a_wall_flag_read(x, y, 0) == 0) {
				a_wall_remove(x, y, 0);
			}
			if (a_wall_flag_read(x, y, 1) == 0) {
				a_wall_remove(x, y, 1);
			}
		}
	}
}

void wall_information_display() {
	signed char x, y;

	extern unsigned short straight_priority_step_map_queue[16][16];
//	char step_count = 0;
//	search_init();
//update_step_map();

	for (y = 15; y >= -1; y--) {
		for (x = 0; x < 17; x++) {
			myprintf("+");
			if (x == 16)
				break;
			if (READ_WALL_DATA(wall_state[x][y],B_NORTH_WALL) == 1 || y == -1)
				myprintf("-----");
			else
				myprintf("     ");
		}
		if (y == -1)
			break;
		myprintf("\n|");
		for (x = 0; x < 16; x++) {
			/*if (y == 16){
			 break;
			 }*/
			myprintf("%5d", straight_priority_step_map_queue[x][y]);
			if (READ_WALL_DATA(wall_state[x][y],B_EAST_WALL) == 1)
				myprintf("|");
			else
				myprintf(" ");
		}
		myprintf("\n");
	}
	myprintf("\n%d,%d", now_x_coordinate, now_y_coordinate);
//	myprintf("%d",north[15][15]);

	myprintf("\n\n\n");

}

void update_direction(char direction) {
	buff_x_direction = x_direction;
	buff_y_direction = y_direction;
	if (direction == 0) { //左回転なら
		x_direction = -1 * buff_y_direction;
		y_direction = 1 * buff_x_direction;
	} else if (direction == 1) { //180度回転なら
		x_direction = -1 * buff_x_direction;
		y_direction = -1 * buff_y_direction;
	} else if (direction == 2) { //右回転なら
		x_direction = 1 * buff_y_direction;
		y_direction = -1 * buff_x_direction;
	} else { //直進なら
		x_direction = 1 * buff_x_direction;
		y_direction = 1 * buff_y_direction;
	}
}
//
void update_coordinate() {

	now_x_coordinate += x_direction;
	now_y_coordinate += y_direction;

}

void wall_init() {
	int x, y;
	for (x = 0; x < 16; x++) {
		for (y = 0; y < 16; y++) {
			//
			if (y == 15) {
				wall_state[x][y] |= 1 << B_NORTH_WALL;
				wall_state[x][y] |= 1 << B_NORTH_FLAG;
			}
			//
			if (x == 15) {
				wall_state[x][y] |= 1 << B_EAST_WALL;
				wall_state[x][y] |= 1 << B_EAST_FLAG;
			}
			//スタート座標右の壁
			if (x == 0 && y == 0) {
				wall_state[x][y] |= 1 << B_EAST_WALL;
				wall_state[x][y] |= 1 << B_EAST_FLAG;
				wall_state[x][y] |= 1 << B_NORTH_FLAG;
			}
		}
	}
}

void search_init(char x_goal, char y_goal) {

	int x, y;

	initialize_x();
	initialize_y();

	enqueue_x(x_goal);
	enqueue_y(y_goal);

	for (x = 0; x < 16; x++) {
		for (y = 0; y < 16; y++) {
			if (x == x_goal && y == y_goal) {
				step_map[x][y] = 0;
				step_map_queue[x][y] = 0;
			} else {
				step_map[x][y] = 255;
				step_map_queue[x][y] = 255;
			}
		}
	}
}

char compass_convert(char compass) {

	char a;

	if (y_direction == -1)
		a = 2;
	else
		a = x_direction;
	compass += a;
	return compass %= 4;
}

char abs_compass_convert(char compass) {

	if (x_direction == 0 && y_direction == 1) {
		compass += 0;
	} else if (x_direction == 1 && y_direction == 0) {
		compass += 1;
	} else if (x_direction == 0 && y_direction == -1) {
		compass += 2;
	} else if (x_direction == -1 && y_direction == 0) {
		compass += 3;
	}
	return compass %= 4;
}

//その座標がマップの範囲内か調べる関数
//返り値は、マップ内なら1、マップ外なら0を返す
char check_map_out(int x_coordinate, int y_coordinate) {

	if (x_coordinate < 0 || x_coordinate > 15 || y_coordinate < 0
			|| y_coordinate > 15) {
		return 0;
	}
	return 1;
}

//前の壁情報保存
void save_now_wall_information() {
	int x, y;

	for (x = 0; x < 16; x++) {
		for (y = 0; y < 16; y++) {
			wall_state[x][y] &= 0x0f;
			wall_state[x][y] |= wall_state[x][y] << 4;
		}
	}

}
void read_buff_wall_information(char x_goal, char y_goal) {
	int x, y;

	map_break_flag = 1;
	now_x_coordinate = 0;
	now_y_coordinate = 0;
	x_direction = 0;
	y_direction = 1;
	for (x = 0; x < 16; x++) {
		for (y = 0; y < 16; y++) {
			wall_state[x][y] &= 0xf0;
			wall_state[x][y] |= wall_state[x][y] >> 4;
			/*
			 north[x][y] = buff_north[x][y];
			 north_flag[x][y] = buff_north_flag[x][y];
			 east[x][y] = buff_east[x][y];
			 east_flag[x][y] = buff_east_flag[x][y];
			 */
		}
	}
	update_step_map_queue(x_goal, y_goal);
}

//queue関係
void update_step_map_queue(char x_goal, char y_goal) {
	int x, y;

	wall_init();
	search_init(x_goal, y_goal);

	while ((queue_empty_x() == 0) && (queue_empty_y() == 0)) {
//	if(step == 2) myprintf("\nx=%d,y=%d\n",x,y);
		x = dequeue_x();
		y = dequeue_y();

		if ((y > 0) && (step_map_queue[x][y - 1] == 255)
				&& (a_wall_read(x, y - 1, 0) == 0)) {
			step_map_queue[x][y - 1] = step_map_queue[x][y] + 1;
			enqueue_x(x);
			enqueue_y(y - 1);
		}
		if ((y < 15) && (step_map_queue[x][y + 1] == 255)
				&& (a_wall_read(x, y + 1, 2) == 0)) {
			step_map_queue[x][y + 1] = step_map_queue[x][y] + 1;
			enqueue_x(x);
			enqueue_y(y + 1);
		}
		if ((x > 0) && (step_map_queue[x - 1][y] == 255)
				&& (a_wall_read(x - 1, y, 1) == 0)) {
			step_map_queue[x - 1][y] = step_map_queue[x][y] + 1;
			enqueue_x(x - 1);
			enqueue_y(y);
		}
		if ((x < 15) && (step_map_queue[x + 1][y] == 255)
				&& (a_wall_read(x + 1, y, 3) == 0)) {
			step_map_queue[x + 1][y] = step_map_queue[x][y] + 1;
			enqueue_x(x + 1);
			enqueue_y(y);
		}
//		wall_information_display();
	}

	/*
	 while(1){
	 if (step_map_queue[dequeue_x()][dequeue_y()] == i) {
	 if (step_map_queue[dequeue_x()][dequeue_y() - 1] > i
	 && dequeue_y() > 0
	 && a_wall_read(dequeue_x(), dequeue_y() - 1, 2) == 0) {
	 enqueue_x(dequeue_x());
	 enqueue_y(dequeue_y() - 1);
	 step_map_queue[dequeue_x()][dequeue_y() - 1] = i + 1;
	 }
	 if (step_map_queue[dequeue_x()][dequeue_y() + 1] > i
	 && dequeue_y() < 15 && a_wall_read(x, y, 0) == 0) {
	 enqueue_x(dequeue_x());
	 enqueue_y(dequeue_y() + 1);
	 step_map_queue[dequeue_x()][dequeue_y() + 1] = i + 1;
	 }
	 if (step_map_queue[dequeue_x() - 1][dequeue_y()] > i
	 && dequeue_x() > 0
	 && a_wall_read(dequeue_x(), dequeue_y(), 3) == 0) {
	 enqueue_x(dequeue_x() - 1);
	 enqueue_y(dequeue_y());
	 step_map_queue[dequeue_x() - 1][dequeue_y()] = i + 1;
	 }
	 if (step_map_queue[dequeue_x() + 1][dequeue_y()] > i
	 && dequeue_x() < 15
	 && a_wall_read(dequeue_x(), dequeue_y(), 1) == 0) {
	 enqueue_x(dequeue_x() + 1);
	 enqueue_y(dequeue_y());
	 step_map_queue[dequeue_x() + 1][dequeue_y()] = i + 1;
	 }
	 }
	 }
	 */
}

void enqueue_x(int x) {

	if (queue_full_x())
		return;	//キュー配列が満タンなら何もせず終了

	Q_x[tail_x] = x;	//キュー配列の最後尾に引数を代入

	if (tail_x + 1 == QUEUE_MAX)	//キュー配列の最後尾に1足した数が配列数なら最後尾を0に（リング状バッファ処理）
		tail_x = 0;
	else
//上記以外なら最後尾を右に1ずらす
		tail_x++;

}
void enqueue_y(int x) {

	if (queue_full_y())
		return;

	Q_y[tail_y] = x;

	if (tail_y + 1 == QUEUE_MAX)
		tail_y = 0;
	else
		tail_y++;

}
int dequeue_x(void) {
	int x;	//デキューする変数を格納する一時変数

	if (queue_empty_x())	//デキューするものが無かったら0を返す
		return 0;

	x = Q_x[head_x];	//先頭の数字をxに一時格納
	if (head_x + 1 == QUEUE_MAX)	//キュー配列の先頭に1足した数字が配列数なら先頭を0に（リング状バッファ処理）
		head_x = 0;
	else
//先頭を1右にずらす
		head_x++;

	return x;	//x（先頭の数字）を返す
}

int dequeue_y(void) {
	int y;

	if (queue_empty_y())
		return 0;

	y = Q_y[head_y];
	if (head_y + 1 == QUEUE_MAX)
		head_y = 0;
	else
		head_y++;

	return y;
}

void initialize_x() {
	head_x = tail_x = 0;	//headとtailを0に初期化
}

void initialize_y() {
	head_y = tail_y = 0;
}
char queue_full_x() {
	return (head_x == (tail_x + 1) % QUEUE_MAX) ? 1 : 0;
}
char queue_full_y() {
	return (head_y == (tail_y + 1) % QUEUE_MAX) ? 1 : 0;
}
char queue_empty_x() {
	return (head_x == tail_x) ? 1 : 0;
}
char queue_empty_y() {
	return (head_y == tail_y) ? 1 : 0;
}

