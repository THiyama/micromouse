/*
 * wall_control.c
 *
 *  Created on: 2016/09/06
 *      Author: PCーUSER
 */

#include "wall_control.h"
#include "wall_control_define.h"
#include "iodefine.h"
#include "search.h"
#include "motor.h"
#include "motor_define.h"
#include "math.h"

extern int sensor_log[1001];

//壁制御を行う関数
//左のセンサ値、右のセンサ値を引数とする
float wall_control() {

	//現在位置の偏差
	int wall_control_deviation = 0.0;

	//制御量
	float wall_control_quantity = 0.0;

	//壁との偏差計算
	int right_threshold, left_threshold;
	static int before_left_wall_distance;
	static int before_right_wall_distance;

	static int right_monotonic_decrease_count = 0;
	static int left_monotonic_decrease_count = 0;
	static int right_monotonic_increase_count = 0;
	static int left_monotonic_increase_count = 0;

	static int counter_left = 0;
	static int counter_right = 0;

	int left_diff, right_diff;

	left_diff = ad_data[1] - before_left_wall_distance;
	right_diff = ad_data[3] - before_right_wall_distance;

	if (left_diff <= 0) {
		left_monotonic_decrease_count++;
		left_monotonic_increase_count = 0;
	} else if (left_diff > 0 && ad_data[1] > LEFT_WALL_THRESHOLD) {
		left_monotonic_increase_count++;
		left_monotonic_decrease_count = 0;
		counter_left = 0;
	}
	if (right_diff <= 0) {
		right_monotonic_decrease_count++;
		right_monotonic_increase_count = 0;
	} else if (right_diff > 0 && ad_data[3] > RIGHT_WALL_THRESHOLD) {
		right_monotonic_increase_count++;
		right_monotonic_decrease_count = 0;
		counter_right = 0;
	}

	if (left_monotonic_decrease_count > LEFT_DECREASE_COUNT) {
		counter_left = 1;
	}
	if (right_monotonic_decrease_count > RIGHT_DECREASE_COUNT) {
		counter_right = 1;
	}
	before_left_wall_distance = ad_data[1];
	before_right_wall_distance = ad_data[3];

	//吸い込まれ対策
	//単調減少が3回続くと吸い込まれ対策に入る
	if (counter_right == 1) {
		led(3);
		right_threshold = RIGHT_WALL_REFERENCE + 1000; //変化量が3回連続単調減少していたら、閾値を引き上げる
	} else {
		right_threshold = RIGHT_WALL_THRESHOLD; //変化量が3回連続単調増加していてかつ、ad_dataが250以上なら、設定通りの閾値
	}

	if (counter_left == 1) {
		led(6);
		left_threshold = LEFT_WALL_REFERENCE + 1000;  //変化量が一定以上なら、閾値を引き上げる
	} else {
		left_threshold = LEFT_WALL_THRESHOLD; //変化量が一定以下なら、設定通りの閾値
	}

	if ((ad_data[3] > right_threshold) && (ad_data[1] > left_threshold)) {
		//両壁がある時
		led(2);
		wall_control_deviation = (ad_data[1] - LEFT_WALL_REFERENCE)
				- (ad_data[3] - RIGHT_WALL_REFERENCE);
	} else if ((ad_data[3] <= right_threshold)
			&& (ad_data[1] <= left_threshold)) {
		led(5);
		//両方の壁がない時
		wall_control_deviation = 0;
	} else if (ad_data[3] > right_threshold) {
		//右センサだけ使える時
		led(6);
		wall_control_deviation = -2 * (ad_data[3] - RIGHT_WALL_REFERENCE);
	} else {
		//左センサだけ使える時
		led(3);
		wall_control_deviation = 2 * (ad_data[1] - LEFT_WALL_REFERENCE);
	}

//	wall_control_deviation=(left_wall_distance-LEFT_WALL_REFERENCE)-(right_wall_distance-RIGHT_WALL_REFERENCE);
//	wall_control_deviation=0.0;
	//制御量の計算
	wall_control_quantity = wall_control_constant
			* (float) wall_control_deviation;

	return wall_control_quantity;	//制御量を返す
//	return 2.0;
}
float oblique_wall_control(int dir) {

	//制御量
	float forward_wall_control_quantity = 0.0;
	float side_wall_control_quantity = 0.0;

	if (dir == 0) {
		if (ad_data[0] > LEFT_FORWARD_OBLIQUE_WALL_THRESHOLD) {
			forward_wall_control_quantity = ad_data[0]
					- LEFT_FORWARD_OBLIQUE_WALL_THRESHOLD;
			led(6);
			return (left_forward_oblique_wall_control_constant
					* forward_wall_control_quantity);
		} else if (ad_data[1] > LEFT_OBLIQUE_WALL_THRESHOLD) {
			side_wall_control_quantity = ad_data[1]
					- LEFT_OBLIQUE_WALL_THRESHOLD;
			led(5);
			return (left_oblique_wall_control_constant
					* side_wall_control_quantity);
		}
		led(2);
		return 0.0;
	} else if (dir == 1) {
		if (ad_data[4] > RIGHT_FORWARD_OBLIQUE_WALL_THRESHOLD) {
			forward_wall_control_quantity = ad_data[4]
					- RIGHT_FORWARD_OBLIQUE_WALL_THRESHOLD;
			led(3);
			return (right_forward_oblique_wall_control_constant
					* forward_wall_control_quantity);
		} else if (ad_data[3] > RIGHT_OBLIQUE_WALL_THRESHOLD) {
			side_wall_control_quantity = ad_data[3]
					- RIGHT_OBLIQUE_WALL_THRESHOLD;
			led(1);
			return (right_oblique_wall_control_constant
					* side_wall_control_quantity);

		}
		led(2);
		return 0.0;
	}
	return 0;
}

void wall_out(double before_distance, int wall_out_value, signed char direction) {

	extern signed char now_x_coordinate;
	extern signed char now_y_coordinate;

	char wall_out_flg = 0;

	signed int deviation = 0;
	signed int left_wall_difference = 0, right_wall_difference = 0;
	signed int left_pillar_difference = 0, right_pillar_difference = 0;
	int before_ad_data_1 = 0;
	int before_ad_data_3 = 0;

	char pillar_out_flg = 0;

	before_ad_data_1 = ad_data[1];
	before_ad_data_3 = ad_data[3];

	if (direction == -1) {

		right_wall_difference = ad_data[3] - RIGHT_WALL_REFERENCE;
		right_pillar_difference = ad_data[3] - RIGHT_WALL_THRESHOLD;
		while (1) {
			if (ad_data[3] < RIGHT_WALL_THRESHOLD - right_pillar_difference) {
				pillar_out_flg++;
				wall_out_flg = 0;
			} else if (ad_data[3]
					> RIGHT_WALL_THRESHOLD + right_wall_difference) {
				wall_out_flg++;
				pillar_out_flg = 0;
			} else {
				pillar_out_flg = 0;
				wall_out_flg = 0;
			}
			if (pillar_out_flg > 10) {
				//壁がないときの反応→柱切れ
				while (1) {
					if (ad_data[3]
							> RIGHT_PILLAR_OUT_THRESHOLD
									+ right_pillar_difference) {
						led(3);
						wall_out_flg = 1;
					}
					if (wall_out_flg == 1) {
						while (1) {
							trapezoidal_acceleration_straight_distance(0.001,
									big_slalom_center_velocity,
									big_slalom_center_velocity,
									big_slalom_center_velocity, 2.0, 2.0);
							if (ad_data[3]
									< RIGHT_PILLAR_OUT_THRESHOLD
											+ right_pillar_difference) {
								trapezoidal_acceleration_straight_distance(
										before_distance,
										big_slalom_center_velocity,
										big_slalom_center_velocity,
										big_slalom_center_velocity, 2.0, 2.0);
								return;
							}

							if (PORTC.PIDR.BIT.B6 == 0) {
								return;
							}
						}
					}
					if (PORTC.PIDR.BIT.B6 == 0) {
						return;
					}
					trapezoidal_acceleration_straight_distance(0.0001,
							big_slalom_center_velocity,
							big_slalom_center_velocity,
							big_slalom_center_velocity, 2.0, 2.0);
				}
			} else if (wall_out_flg > 10) {
				//壁があるとき
				led(1);

//				if (abs(deviation) < 40) { //僅差がわずかなら通常壁切れ
				while (1) {
					deviation = RIGHT_WALL_REFERENCE - ad_data[3];
					//deviationが正なら左にずれている。負なら右にずれている。

					if (ad_data[3] < (wall_out_value + right_wall_difference)) {
						trapezoidal_acceleration_straight_distance(
								before_distance, big_slalom_center_velocity,
								big_slalom_center_velocity,
								big_slalom_center_velocity, 2.0, 2.0);
						return;
					}

					if (PORTC.PIDR.BIT.B6 == 0) {
						return;
					}
					trapezoidal_acceleration_straight_distance(0.0001,
							big_slalom_center_velocity,
							big_slalom_center_velocity,
							big_slalom_center_velocity, 2.0, 2.0);
				}
//				}
				/*else { //僅差が大きいなら

				 diffrence = ad_data[3] - before_ad_data_3;
				 if (diffrence > 20) {
				 trapezoidal_acceleration_straight_distance(before_distance,
				 big_slalom_center_velocity,
				 big_slalom_center_velocity,
				 big_slalom_center_velocity, 2.0, 2.0);
				 return;
				 } else {
				 trapezoidal_acceleration_straight_distance(0.0001,
				 big_slalom_center_velocity, big_slalom_center_velocity,
				 big_slalom_center_velocity, 2.0, 2.0);
				 }

				 return;
				 if (PORTC.PIDR.BIT.B6 == 0) {
				 return;
				 }
				 before_ad_data_3 = ad_data[3];
				 }
				 */
			}
		}

	} else {

		left_wall_difference = ad_data[1] - LEFT_WALL_REFERENCE;
		left_pillar_difference = ad_data[1] - LEFT_WALL_THRESHOLD;
		while (1) {
			if (ad_data[1] < LEFT_WALL_THRESHOLD - left_pillar_difference) {
				pillar_out_flg++;
				wall_out_flg = 0;
			} else if (ad_data[1]
					> LEFT_WALL_THRESHOLD + left_wall_difference) {
				wall_out_flg++;
				pillar_out_flg = 0;
			} else {
				pillar_out_flg = 0;
				wall_out_flg = 0;
			}
			if (pillar_out_flg > 10) {
				//壁がないときの反応→柱切れ
				while (1) {
					if (ad_data[1]
							> LEFT_PILLAR_OUT_THRESHOLD
									+ left_pillar_difference) {
						led(3);
						wall_out_flg = 1;
					}
					if (wall_out_flg == 1) {
						while (1) {
							trapezoidal_acceleration_straight_distance(0.001,
									big_slalom_center_velocity,
									big_slalom_center_velocity,
									big_slalom_center_velocity, 2.0, 2.0);
							if (ad_data[1]
									< LEFT_PILLAR_OUT_THRESHOLD
											+ left_pillar_difference) {
								trapezoidal_acceleration_straight_distance(
										before_distance,
										big_slalom_center_velocity,
										big_slalom_center_velocity,
										big_slalom_center_velocity, 2.0, 2.0);
								return;
							}

							if (PORTC.PIDR.BIT.B6 == 0) {
								return;
							}
						}
					}

					if (PORTC.PIDR.BIT.B6 == 0) {
						return;
					}
					trapezoidal_acceleration_straight_distance(0.0001,
							big_slalom_center_velocity,
							big_slalom_center_velocity,
							big_slalom_center_velocity, 2.0, 2.0);
				}
			} else if (wall_out_flg > 10) {
				//壁があるとき
				led(1);

//				if (abs(deviation) < 40) { //僅差がわずかなら通常壁切れ
				while (1) {
					deviation = LEFT_WALL_REFERENCE - ad_data[1];
					//deviationが正なら右にずれている。負なら左にずれている。

					if (ad_data[1] < wall_out_value + left_wall_difference) {
						trapezoidal_acceleration_straight_distance(
								before_distance, big_slalom_center_velocity,
								big_slalom_center_velocity,
								big_slalom_center_velocity, 2.0, 2.0);
						return;
					}

					if (PORTC.PIDR.BIT.B6 == 0) {
						return;
					}
					trapezoidal_acceleration_straight_distance(0.0001,
							big_slalom_center_velocity,
							big_slalom_center_velocity,
							big_slalom_center_velocity, 2.0, 2.0);
				}
//				} else { //僅差が大きいなら
				/*					while (1) {
				 diffrence = ad_data[1] - before_ad_data_1;
				 if (diffrence > 30) {
				 trapezoidal_acceleration_straight_distance(
				 before_distance, big_slalom_center_velocity,
				 big_slalom_center_velocity,
				 big_slalom_center_velocity, 2.0, 2.0);
				 return;
				 } else {
				 trapezoidal_acceleration_straight_distance(0.0001,
				 big_slalom_center_velocity,
				 big_slalom_center_velocity,
				 big_slalom_center_velocity, 2.0, 2.0);
				 }

				 if (PORTC.PIDR.BIT.B6 == 0) {
				 return;
				 }
				 before_ad_data_1 = ad_data[1];
				 }
				 }
				 */
			}
		}
	}
}

char hihumint_dead_wall() {
	if ((ad_data[0] > (LEFT_FORWARDWALL_FAIL - WALL_FAIL_SAFE_DIFF))
			&& (ad_data[4] > (RIGHT_FORWARDWALL_FAIL - WALL_FAIL_SAFE_DIFF))) {
		led(2);
		map_break_flag = 1;
		return 1;
	} else if ((ad_data[0] > LEFT_FORWARDWALL_FAIL)
			|| (ad_data[4] > RIGHT_FORWARDWALL_FAIL)) {
		led(3);
		map_break_flag = 1;
		return 1;
	} else {
		map_break_flag = 0;
		return 0;
	}
}

void wall_log() {
	trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5,
			big_slalom_center_velocity, 0.15, big_slalom_center_velocity, 2.0,
			2.0);

	sensor_log_flg = 1;
	trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 2.5,
			big_slalom_center_velocity, big_slalom_center_velocity, 0.15, 2.0,
			2.0);

	MTU_STOP
	;
	__delay_ms(100);
	MOTOR_EXITATION = 0;

	sensor_log_flg = 0;
}
