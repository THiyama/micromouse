/*
 * adjust.c
 *
 *  Created on: 2016/10/21
 *      Author: PCーUSER
 */

#include "adjust.h"

void adjust_slalom(int number) {
	int i;

	sensor_led_flg = 1;
	wall_control_constant = 0.00013;
	caluculate_count = 0;
	switch (number) {
	case LEFT_SMALL_SLALOM:
		trapezoidal_acceleration_straight_distance(0.032 + BLOCK_LENGTH * 1.5,
				0.45, 0.10, 0.45, 1.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(small_left_parameter_1);
//			trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1, 0.45,
//					0.45, 0.45, 2.0, 2.0);
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.0, 0.45,
				0.45, 0.15, 2.0, 3.0);
		MTU_STOP
		;

		break;
	case RIGHT_SMALL_SLALOM:
		trapezoidal_acceleration_straight_distance(0.032 + BLOCK_LENGTH * 1.5,
				0.45, 0.15, 0.45, 1.0, 3.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(small_right_parameter_1);
			//		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1, 0.45,
			//				0.45, 0.45, 2.0, 2.0);
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.0, 0.45,
				0.45, 0.15, 2.0, 3.0);
		MTU_STOP
		;

		break;

	case LEFT_BIG_SLALOM:
		big_slalom_center_velocity = 0.8;
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			//	wall_control_constant = 0.0;
			small_slalom(big_left_parameter_2);
			wall_control_constant = 0.0;
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);
		MTU_STOP
		;

		break;

	case RIGHT_BIG_SLALOM:

		big_slalom_center_velocity = 0.75;
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			//	wall_control_constant = 0.0;
			small_slalom(big_right_parameter_1);
			wall_control_constant = 0.0;
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.0,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);
		MTU_STOP
		;

		break;

	case LEFT_180_BIG_SLALOM:

		big_slalom_center_velocity = 0.85;
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			//	wall_control_constant = 0.0;
			small_slalom(big_180_left_parameter_3);
			wall_control_constant = 0.0;
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);

		MTU_STOP
		;
		break;

	case RIGHT_180_BIG_SLALOM:

		big_slalom_center_velocity = 0.85;
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			//	wall_control_constant = 0.0;
			small_slalom(big_180_right_parameter_3);
			wall_control_constant = 0.0;
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);

		MTU_STOP
		;
		break;
	case LEFT_45_IN_OBLIQUE_SLALOM:
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(left_45_in_oblique_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 1.5 * 1.41421356, big_slalom_center_velocity,
				big_slalom_center_velocity, 0.20, 2.0, 4.0);

		MTU_STOP
		;
		break;

	case RIGHT_45_IN_OBLIQUE_SLALOM:
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 0.5 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(right_45_in_oblique_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 1.5 * 1.41421356, big_slalom_center_velocity,
				big_slalom_center_velocity, 0.20, 2.0, 4.0);

		MTU_STOP
		;
		break;

	case LEFT_135_IN_OBLIQUE_SLALOM:
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.5 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);
		wall_control_constant = 0.0;

		for (i = 0; i < 1; i++) {
			small_slalom(left_135_in_oblique_parameter_1);
		}

		oblique_trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 2.5 * 1.41421356, big_slalom_center_velocity,
				big_slalom_center_velocity, 0.20, 2.0, 2.0);

		MTU_STOP
		;
		break;

	case RIGHT_135_IN_OBLIQUE_SLALOM:
		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 3 + 0.032,
				big_slalom_center_velocity, 0.15, big_slalom_center_velocity,
				2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(right_135_in_oblique_parameter_1);
		}

		oblique_trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 2 * 1.41421356, big_slalom_center_velocity,
				big_slalom_center_velocity, big_slalom_center_velocity, 2.0,
				2.0);
		/*		small_slalom(left_135_out_oblique_parameter_1);

		 wall_control_constant = 0.00013;
		 small_slalom(left_135_in_oblique_parameter_1);
		 oblique_trapezoidal_acceleration_straight_distance(
		 BLOCK_LENGTH * 2* 1.41421356, big_slalom_center_velocity,
		 big_slalom_center_velocity, 0.75, 2.0, 2.0);
		 small_slalom(right_135_out_oblique_parameter_1);
		 trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1 + 0.032,
		 big_slalom_center_velocity, 0.75, 0.15,
		 2.0, 2.0);*/

		MTU_STOP
		;
		break;

	case LEFT_45_OUT_OBLIQUE_SLALOM:
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity, 0.15,
				big_slalom_center_velocity, 2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(left_45_out_oblique_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);

		MTU_STOP
		;
		break;

	case RIGHT_45_OUT_OBLIQUE_SLALOM:
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity, 0.15,
				big_slalom_center_velocity, 2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(right_45_out_oblique_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);

		MTU_STOP
		;
		break;

	case LEFT_135_OUT_OBLIQUE_SLALOM:

		wall_control_constant = 0.0;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity, 0.15,
				big_slalom_center_velocity, 2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(left_135_out_oblique_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);

		MTU_STOP
		;
		break;

	case RIGHT_135_OUT_OBLIQUE_SLALOM:

		wall_control_constant = 0.0;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity, 0.15,
				big_slalom_center_velocity, 2.0, 2.0);

		for (i = 0; i < 1; i++) {
			wall_control_constant = 0.0;
			small_slalom(right_135_out_oblique_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(BLOCK_LENGTH * 1.5,
				big_slalom_center_velocity, big_slalom_center_velocity, 0.20,
				2.0, 4.0);

		MTU_STOP
		;
		break;

	case LEFT_90_OBLIQUE_SLALOM:
		wall_control_constant = 0.0;
		big_slalom_center_velocity = 0.55;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity, 0.2,
				big_slalom_center_velocity, 2.0, 2.0);

//		small_slalom(right_135_in_oblique_parameter_1);

		for (i = 0; i < 1; i++) {
			small_slalom(left_oblique_90_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity,
				big_slalom_center_velocity, 0.20, 2.0, 4.0);

		MTU_STOP
		;
		break;

	case RIGHT_90_OBLIQUE_SLALOM:
		big_slalom_center_velocity = 0.55;
		wall_control_constant = 0.0;
		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5, big_slalom_center_velocity, 0.2,
				big_slalom_center_velocity, 2.8, 2.0);

//		small_slalom(left_135_in_oblique_parameter_1);

		for (i = 0; i < 1; i++) {
			small_slalom(right_oblique_90_parameter_1);
		}

		trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 1.41421356, big_slalom_center_velocity,
				big_slalom_center_velocity, 0.20, 2.0, 4.0);

		MTU_STOP
		;
		break;

	case OBLIQUE_STRAIGHT:

		oblique_trapezoidal_acceleration_straight_distance(
		BLOCK_LENGTH * 0.5 * 8 * 1.41421356, 1.2, 0.15, 0.15, 1, 1.0);

		MTU_STOP
		;
		break;
	}

	__delay_ms(1000);
	MOTOR_EXITATION = 0;
	sensor_led_flg = 0;

}
void adjust_all_slalom() {
	while (1) {
		mode_count = 0;
		sensor_led_flg = 0;
		MOTOR_EXITATION = 0;
		choose_mode();
		determine_led();

		if (mode_count == 0) {
			adjust_small_slalom();
		} else if (mode_count == 1) {
			adjust_big_slalom();
		} else if (mode_count == 2) {
			adjust_big_180_slalom();
		} else if (mode_count == 3) {
			adjust_oblique_slalom();
		} else if (mode_count == 4) {
			adjust_90_oblique_slalom();
		} else if (mode_count == 5) {
		} else if (mode_count == 6) {

		} else if (mode_count == 7) {

		} else {

		}
	}
}

void adjust_small_slalom() {
	mode_count = 0;
	sensor_led_flg = 0;
	MOTOR_EXITATION = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
		adjust_slalom(LEFT_SMALL_SLALOM);
	} else if (mode_count == 1) {
		adjust_slalom(RIGHT_SMALL_SLALOM);
	} else if (mode_count == 2) {
	} else if (mode_count == 3) {
	} else if (mode_count == 4) {
	} else if (mode_count == 5) {
	} else if (mode_count == 6) {
	} else if (mode_count == 7) {
	} else {
	}
	return;
}
void adjust_big_slalom() {
	mode_count = 0;
	sensor_led_flg = 0;
	MOTOR_EXITATION = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
		adjust_slalom(LEFT_BIG_SLALOM);
	} else if (mode_count == 1) {
		adjust_slalom(RIGHT_BIG_SLALOM);
	} else if (mode_count == 2) {
	} else if (mode_count == 3) {
	} else if (mode_count == 4) {
	} else if (mode_count == 5) {
	} else if (mode_count == 6) {
	} else if (mode_count == 7) {
	} else {
	}
	return;
}

void adjust_big_180_slalom() {
	mode_count = 0;
	sensor_led_flg = 0;
	MOTOR_EXITATION = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
		adjust_slalom(LEFT_180_BIG_SLALOM);
	} else if (mode_count == 1) {
		adjust_slalom(RIGHT_180_BIG_SLALOM);
	} else if (mode_count == 2) {
	} else if (mode_count == 3) {
	} else if (mode_count == 4) {
	} else if (mode_count == 5) {
	} else if (mode_count == 6) {
	} else if (mode_count == 7) {
	} else {
	}
	return;
}

void adjust_oblique_slalom() {
	mode_count = 0;
	sensor_led_flg = 0;
	MOTOR_EXITATION = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
		adjust_slalom(LEFT_45_IN_OBLIQUE_SLALOM);
	} else if (mode_count == 1) {
		adjust_slalom(RIGHT_45_IN_OBLIQUE_SLALOM);
	} else if (mode_count == 2) {
		adjust_slalom(LEFT_135_IN_OBLIQUE_SLALOM);
	} else if (mode_count == 3) {
		adjust_slalom(RIGHT_135_IN_OBLIQUE_SLALOM);
	} else if (mode_count == 4) {
		adjust_slalom(LEFT_45_OUT_OBLIQUE_SLALOM);
	} else if (mode_count == 5) {
		adjust_slalom(RIGHT_45_OUT_OBLIQUE_SLALOM);
	} else if (mode_count == 6) {
		adjust_slalom(LEFT_135_OUT_OBLIQUE_SLALOM);
	} else if (mode_count == 7) {
		adjust_slalom(RIGHT_135_OUT_OBLIQUE_SLALOM);
	} else {
	}
	return;
}

void adjust_90_oblique_slalom() {
	mode_count = 0;
	sensor_led_flg = 0;
	MOTOR_EXITATION = 0;
	choose_mode();
	start_led();
	if (mode_count == 0) {
		adjust_slalom(LEFT_90_OBLIQUE_SLALOM);
	} else if (mode_count == 1) {
		adjust_slalom(RIGHT_90_OBLIQUE_SLALOM);
	} else if (mode_count == 2) {
		adjust_slalom(OBLIQUE_STRAIGHT);
	} else {
	}
	return;
}
