/*
 * contest.c
 *
 *  Created on: 2016/10/15
 *      Author: PCーUSER
 */
#include "contest.h"

extern char map_break_flag;

extern unsigned char renew_shortest_path[1000];

void contest_search_shortest() {
	int i, j;
	while (1) {
		map_break_flag = 0;
		mode_count = 0;
		__delay_ms(250);
		MOTOR_reset();
		__delay_ms(250);
		MOTOR_EXITATION = 1;
		choose_mode();
		if (mode_count != 0 && mode_count != 1 && mode_count != 2) {
			start_led();
		} else {
			determine_led();
		}
		if (mode_count == 0) {
			//探索
			contest_search();
		} else if (mode_count == 1) {
			//大回りと小回り
			caluculate_mode = 0;
			caluculate_count =0;
			contest_shortest_big();
		} else if (mode_count == 2) {
			//斜め
			caluculate_mode = 0;
			caluculate_count =0;
			contest_shortest_oblique();
		} else if (mode_count == 3) {
		/*	MOTOR_EXITATION = 0;
			make_straight_priority_step_map(TARGET_X_COORDINATE,TARGET_Y_COORDINATE);
			while(PORTC.PIDR.BIT.B6 == 1);
			wall_information_display();
		*/} else if (mode_count == 4) {
		} else if (mode_count == 5) {
		} else if (mode_count == 6) {
		} else {
		}
	}
}

void circuit(double velocity, double initial_acceleration,
		double finish_acceleration) {
	int i = 0;
	extern int circuit_flg;
	circuit_flg = 1;

	trapezoidal_acceleration_straight_distance(0.18 * 14, velocity, 0.15, big_slalom_center_velocity,
			initial_acceleration, finish_acceleration);
	small_slalom(big_right_parameter_1);
	for (i = 0; i < 7; i++) {
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 13, velocity, big_slalom_center_velocity, big_slalom_center_velocity, initial_acceleration,
				finish_acceleration);
		small_slalom(big_right_parameter_1);
	}

	trapezoidal_acceleration_straight_distance(
	BLOCK_LENGTH * 3, velocity, big_slalom_center_velocity, 0.15, initial_acceleration,
			finish_acceleration);
	MTU_STOP
	;
	__delay_ms(1000);
	MOTOR_EXITATION = 0;
	circuit_flg =0 ;
}

void contest_search() {
	int i;
	mode_count = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
		//片道探索
		sensor_led_flg = 1;
		PORTA.PODR.BIT.B1 = 1;		//励磁をkiru

		slalom_adachi_method(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);
		if (map_break_flag == 1)
			return ;
		sensor_led_flg = 0;
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);
		led(7);
		__delay_ms(100);
		led(0);
		__delay_ms(100);

		//片道探索のあと、方角を北に変更
		x_direction = 0;
		y_direction = 1;

		//片道探索のあと、座標を0,0に。
		now_x_coordinate = 0;
		now_y_coordinate = 0;
	} else if (mode_count == 1) {
		//往復探索
		//重ね探索
		//2周目重ね探索
		x_direction = 0;
		y_direction = 1;

		//探索用変数群の初期化
		wall_init();
		search_init(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);

		sensor_led_flg = 1;
		MOTOR_EXITATION = 1;		//励磁をkiru

		slalom_adachi_method(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);
		if (map_break_flag == 1)
			return ;
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

		__delay_ms(500);

		sensor_led_flg = 1;

		slalom_adachi_method(0, 0);
		if (map_break_flag == 1)
			return ;
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

		//2往復終了

	} else if (mode_count == 2) {
		//重ね探索
		//2周目重ね探索
		x_direction = 0;
		y_direction = 1;

		//探索用変数群の初期化
		wall_init();
		search_init(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);

		sensor_led_flg = 1;
		MOTOR_EXITATION = 1;		//励磁をkiru

		slalom_adachi_method(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);
		if (map_break_flag == 1)
			return ;
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

		__delay_ms(500);
		MOTOR_EXITATION = 0;

		__delay_ms(5000);

		sensor_led_flg = 1;

		//最短経路導出//
		slalom_adachi_shortest_path(0, 0);
		renew_big_slalom_shortest_path();
		//最短経路導出終了//
/*
							PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		//↓パス表示デバッグ
							start_switch();
							for (i = 0; i <= 255; i++)
								myprintf("saitan =%d\n", renew_shortest_path[i]);
		//↑パス表示デバッグ

							start_switch();
							__delay_ms(500);
							*/
		//最短走行//

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
		//		slalom_shortest_run(0.6, 2.5);
		renew_big_slalom_shortest_run(1.2, 2.0, 2.0,1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return ;
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
	}
	else if(mode_count == 3){

		//往復探索
		//重ね探索
		//2周目重ね探索
		x_direction = 0;
		y_direction = 1;

		//探索用変数群の初期化
		wall_init();
		search_init(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);

		sensor_led_flg = 1;
		MOTOR_EXITATION = 1;		//励磁をkiru

		slalom_furukawa_adachi_method(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);
		if (map_break_flag == 1)
			return ;
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

		__delay_ms(500);

		sensor_led_flg = 1;

		slalom_furukawa_adachi_method(0, 0);
		if (map_break_flag == 1)
			return ;
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
		//2往復終了
	}else if (mode_count == 4) {
		//重ね探索
		//2周目重ね探索
		x_direction = 0;
		y_direction = 1;

		//探索用変数群の初期化
		wall_init();
		search_init(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);

		sensor_led_flg = 1;
		MOTOR_EXITATION = 1;		//励磁をkiru

		slalom_furukawa_adachi_method(TARGET_X_COORDINATE, TARGET_Y_COORDINATE);
		if (map_break_flag == 1)
			return ;
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

		__delay_ms(500);
		MOTOR_EXITATION = 0;

		__delay_ms(5000);

		sensor_led_flg = 1;

		//最短経路導出//
		slalom_adachi_shortest_path(0, 0);
		renew_big_slalom_shortest_path();
		//最短経路導出終了//
/*
							PORTA.PODR.BIT.B1 = 0;		//励磁をkiru
		//↓パス表示デバッグ
							start_switch();
							for (i = 0; i <= 255; i++)
								myprintf("saitan =%d\n", renew_shortest_path[i]);
		//↑パス表示デバッグ

							start_switch();
							__delay_ms(500);
							*/
		//最短走行//

		MOTOR_EXITATION = 1;		//励磁をkiru
		sensor_led_flg = 1;
		//		slalom_shortest_run(0.6, 2.5);
		renew_big_slalom_shortest_run(1.2, 2.0, 2.0,1);
		MTU_STOP
		;
		__delay_ms(300);
		MOTOR_EXITATION = 0;		//励磁をkiru
		sensor_led_flg = 0;
		if (map_break_flag == 1)
			return ;
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
	}
}
