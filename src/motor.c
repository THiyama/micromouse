/*
 * motor.c
 *
 *  Created on: 2016/07/19
 *      Author: PCーUSER
 */

#include "motor.h"
#include "math.h"
#include "serial.h"
#include "iodefine.h"
#include "motor_define.h"
#include "rx631_init.h"
#include "wall_control.h"
#include "wall_control_define.h"

unsigned short imai_hairetsu[10000];

//モーター初期設定用関数
//引数等なし、コメントに書いてある通りの動作
void MD_init() {
	PORTB.PODR.BIT.B7 = 1;		//SET_Reset1_OUT=>RESET
	PORTB.PODR.BIT.B6 = 1;		//SET_Reset2_OUT=>RESET
	__delay_ms(10);
	PORTB.PODR.BIT.B7 = 0;		//SET_Reset1_OUT=>RESET
	PORTB.PODR.BIT.B6 = 0;		//SET_Reset2_OUT=>RESET
	PORTA.PODR.BIT.B1 = 0;		//SLEEP1_OFF
	PORTA.PODR.BIT.B3 = 0;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 1;		//SET_CW/CCW2_OUT

	PORTA.PODR.BIT.B1 = 1;		//励磁をかける

}
void MOTOR_reset(){
	PORTB.PODR.BIT.B7 = 1;		//SET_Reset1_OUT=>RESET
	PORTB.PODR.BIT.B6 = 1;		//SET_Reset2_OUT=>RESET
	__delay_ms(50);
	PORTB.PODR.BIT.B7 = 0;		//SET_Reset1_OUT=>RESET
	PORTB.PODR.BIT.B6 = 0;		//SET_Reset2_OUT=>RESET
}
double counter_trapezoidal_acceleration_straight_distance(double distance,
		double max_velocity, double initial_velocity, double finish_velocity,
		double initial_acceleration, double finish_acceleration) {

	double distance_1 = 0.0;
	double distance_2 = 0.0;
	double distance_3 = 0.0;

	double velocity = 0.0;
	double caluculate_distance = 0.0;

	char distance_flag = 0;

	PORTA.PODR.BIT.B3 = 1;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 0;		//SET_CW/CCW2_OUT

	/*	直進、超新地検知用フラグを直進に（straight_rotate_flag=1;）	*/
	straight_rotate_flag = 3;
	distance_1 = (max_velocity * max_velocity
			- initial_velocity * initial_velocity) / (2 * initial_acceleration);
	distance_3 = (max_velocity * max_velocity
			- finish_velocity * finish_velocity) / (2 * finish_acceleration);
	distance_2 = distance - (distance_1 + distance_3);

	if (distance_1 < 0.0) {
		distance_2 -= distance_1;
		distance_1 = 0;
	}
	if (distance_3 < 0.0) {
		distance_2 -= distance_3;
		distance_3 = 0;
	}

	if (distance_2 < 0.0) {
		distance_1 = (2 * finish_acceleration * distance
				+ finish_velocity * finish_velocity
				- initial_velocity * initial_velocity)
				/ (2 * (initial_acceleration + finish_acceleration));
		distance_2 = 0.0;
		distance_3 = (2 * initial_acceleration * distance
				+ initial_velocity * initial_velocity
				- finish_velocity * finish_velocity)
				/ (2 * (initial_acceleration + finish_acceleration));

		if (distance_1 < 0.0) {
			distance_3 -= distance_1;
			distance_1 = 0.0;
		}
		if (distance_3 < 0.0) {
			distance_1 -= distance_3;
			distance_3 = 0.0;
		}

		max_velocity = sqrt(
				(2 * initial_acceleration * finish_acceleration * distance
						+ finish_acceleration * initial_velocity
								* initial_velocity
						+ finish_acceleration * finish_velocity
								* finish_velocity)
						/ (initial_acceleration + finish_acceleration));
	}
	distance_2 += distance_1;
	distance_3 += distance_2;

	trapezoidal_count_1 = 0;
	trapezoidal_count_2 = 0;
	trapezoidal_count_3 = 0;
	while (1) {
		if (caluculate_distance < distance_1) {

			caluculate_distance = initial_velocity * trapezoidal_count_1 * 0.001
					+ 0.5 * initial_acceleration * (trapezoidal_count_1 * 0.001)
							* (trapezoidal_count_1 * 0.001);
			velocity = initial_velocity
					+ initial_acceleration * trapezoidal_count_1 * 0.001;

			trapezoidal_count_2 = 0;
			trapezoidal_count_3 = 0;

		} else if (caluculate_distance < distance_2) {

			caluculate_distance = max_velocity * trapezoidal_count_2 * 0.001
					+ distance_1;
			velocity = max_velocity;

			trapezoidal_count_3 = 0;

		} else if (caluculate_distance < distance_3) {

			caluculate_distance = max_velocity * trapezoidal_count_3 * 0.001
					- 0.5 * finish_acceleration * (trapezoidal_count_3 * 0.001)
							* (trapezoidal_count_3 * 0.001) + distance_2;
			velocity = max_velocity
					- finish_acceleration * trapezoidal_count_3 * 0.001;

		} else {
			break;
		}
		cmt_interrupt_trapezoidal_velocity = velocity;
	}

	straight_rotate_flag = 0;
	cmt_interrupt_trapezoidal_velocity = velocity;
	return velocity;
}
double oblique_trapezoidal_acceleration_straight_distance(double distance,
		double max_velocity, double initial_velocity, double finish_velocity,
		double initial_acceleration, double finish_acceleration) {

	double distance_1 = 0.0;
	double distance_2 = 0.0;
	double distance_3 = 0.0;

	double velocity = 0.0;
	double caluculate_distance = 0.0;

	char distance_flag = 0;
	caluculate_mode = 0;
	distance = distance - 0.45 * caluculate_count * 0.001;
	caluculate_count = 0;
	if (distance <= 0) {
		return 0;
	}
	PORTA.PODR.BIT.B3 = 0;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 1;		//SET_CW/CCW2_OUT

	/*	直進、超新地検知用フラグを直進に（straight_rotate_flag=1;）	*/
	straight_rotate_flag = 5;
	distance_1 = (max_velocity * max_velocity
			- initial_velocity * initial_velocity) / (2 * initial_acceleration);
	distance_3 = (max_velocity * max_velocity
			- finish_velocity * finish_velocity) / (2 * finish_acceleration);
	distance_2 = distance - (distance_1 + distance_3);

	if (distance_1 < 0.0) {
		distance_2 -= distance_1;
		distance_1 = 0;
	}
	if (distance_3 < 0.0) {
		distance_2 -= distance_3;
		distance_3 = 0;
	}

	if (distance_2 < 0.0) {
		distance_1 = (2 * finish_acceleration * distance
				+ finish_velocity * finish_velocity
				- initial_velocity * initial_velocity)
				/ (2 * (initial_acceleration + finish_acceleration));
		distance_2 = 0.0;
		distance_3 = (2 * initial_acceleration * distance
				+ initial_velocity * initial_velocity
				- finish_velocity * finish_velocity)
				/ (2 * (initial_acceleration + finish_acceleration));

		if (distance_1 < 0.0) {
			distance_3 -= distance_1;
			distance_1 = 0.0;
		}
		if (distance_3 < 0.0) {
			distance_1 -= distance_3;
			distance_3 = 0.0;
		}

		max_velocity = sqrt(
				(2 * initial_acceleration * finish_acceleration * distance
						+ finish_acceleration * initial_velocity
								* initial_velocity
						+ finish_acceleration * finish_velocity
								* finish_velocity)
						/ (initial_acceleration + finish_acceleration));
	}
	distance_2 += distance_1;
	distance_3 += distance_2;

	trapezoidal_count_1 = 0;
	trapezoidal_count_2 = 0;
	trapezoidal_count_3 = 0;
	while (1) {
		if (caluculate_distance < distance_1) {
			acceleration_flag = 1;

			caluculate_distance = initial_velocity * trapezoidal_count_1 * 0.001
					+ 0.5 * initial_acceleration * (trapezoidal_count_1 * 0.001)
							* (trapezoidal_count_1 * 0.001);
			velocity = initial_velocity
					+ initial_acceleration * trapezoidal_count_1 * 0.001;

			trapezoidal_count_2 = 0;
			trapezoidal_count_3 = 0;

		} else if (caluculate_distance < distance_2) {

			acceleration_flag = 0;
			caluculate_distance = max_velocity * trapezoidal_count_2 * 0.001
					+ distance_1;
			velocity = max_velocity;

			trapezoidal_count_3 = 0;

		} else if (caluculate_distance < distance_3) {

			acceleration_flag = 1;
			caluculate_distance = max_velocity * trapezoidal_count_3 * 0.001
					- 0.5 * finish_acceleration * (trapezoidal_count_3 * 0.001)
							* (trapezoidal_count_3 * 0.001) + distance_2;
			velocity = max_velocity
					- finish_acceleration * trapezoidal_count_3 * 0.001;

		} else {
			break;
		}
		cmt_interrupt_trapezoidal_velocity = velocity;
	}

//	straight_rotate_flag = 0;
	cmt_interrupt_trapezoidal_velocity = velocity;
	return velocity;
}

double trapezoidal_acceleration_straight_distance(double distance,
		double max_velocity, double initial_velocity, double finish_velocity,
		double initial_acceleration, double finish_acceleration) {

	double distance_1 = 0.0;
	double distance_2 = 0.0;
	double distance_3 = 0.0;

	double velocity = 0.0;
	double caluculate_distance = 0.0;

	char distance_flag = 0;

	if (max_velocity == 0.45 && slalom_mode == 0 && caluculate_count != -50) {
		distance = distance - 0.45 * caluculate_count * 0.001;
	}
	caluculate_count = 0;
	caluculate_mode = 1;
	if (distance <= 0) {
		return 0;
	}
	PORTA.PODR.BIT.B3 = 0;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 1;		//SET_CW/CCW2_OUT

	/*	直進、超新地検知用フラグを直進に（straight_rotate_flag=1;）	*/
	straight_rotate_flag = 1;
	distance_1 = (max_velocity * max_velocity
			- initial_velocity * initial_velocity) / (2 * initial_acceleration);
	distance_3 = (max_velocity * max_velocity
			- finish_velocity * finish_velocity) / (2 * finish_acceleration);
	distance_2 = distance - (distance_1 + distance_3);

	if (distance_1 < 0.0) {
		distance_2 -= distance_1;
		distance_1 = 0;
	}
	if (distance_3 < 0.0) {
		distance_2 -= distance_3;
		distance_3 = 0;
	}

	if (distance_2 < 0.0) {
		distance_1 = (2 * finish_acceleration * distance
				+ finish_velocity * finish_velocity
				- initial_velocity * initial_velocity)
				/ (2 * (initial_acceleration + finish_acceleration));
		distance_2 = 0.0;
		distance_3 = (2 * initial_acceleration * distance
				+ initial_velocity * initial_velocity
				- finish_velocity * finish_velocity)
				/ (2 * (initial_acceleration + finish_acceleration));

		if (distance_1 < 0.0) {
			distance_3 -= distance_1;
			distance_1 = 0.0;
		}
		if (distance_3 < 0.0) {
			distance_1 -= distance_3;
			distance_3 = 0.0;
		}

		max_velocity = sqrt(
				(2 * initial_acceleration * finish_acceleration * distance
						+ finish_acceleration * initial_velocity
								* initial_velocity
						+ finish_acceleration * finish_velocity
								* finish_velocity)
						/ (initial_acceleration + finish_acceleration));
	}
	distance_2 += distance_1;
	distance_3 += distance_2;

	trapezoidal_count_1 = 0;
	trapezoidal_count_2 = 0;
	trapezoidal_count_3 = 0;
	caluculate_mode = 0;
	while (1) {
		if (caluculate_distance < distance_1) {

			caluculate_distance = initial_velocity * trapezoidal_count_1 * 0.001
					+ 0.5 * initial_acceleration * (trapezoidal_count_1 * 0.001)
							* (trapezoidal_count_1 * 0.001);
			velocity = initial_velocity
					+ initial_acceleration * trapezoidal_count_1 * 0.001;

			trapezoidal_count_2 = 0;
			trapezoidal_count_3 = 0;

		} else if (caluculate_distance < distance_2) {

			caluculate_distance = max_velocity * trapezoidal_count_2 * 0.001
					+ distance_1;
			velocity = max_velocity;

			trapezoidal_count_3 = 0;

		} else if (caluculate_distance < distance_3) {

			caluculate_distance = max_velocity * trapezoidal_count_3 * 0.001
					- 0.5 * finish_acceleration * (trapezoidal_count_3 * 0.001)
							* (trapezoidal_count_3 * 0.001) + distance_2;
			velocity = max_velocity
					- finish_acceleration * trapezoidal_count_3 * 0.001;

		} else {
			break;
		}
		cmt_interrupt_trapezoidal_velocity = velocity;
		//myprintf("velocity\t%f\n",velocity);
	}

//	straight_rotate_flag = 0;
	cmt_interrupt_trapezoidal_velocity = velocity;
	caluculate_mode = 1;
	return velocity;
}

//台形加速処理
//引数は、進みたい距離、最高速、初速、終速、加速度。返し値はなし。
//動作はコメントアウトの通り
void trapezoidal_acceleration_straight(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceleration) {
	int flag = 3;
	double trapezoidal_distance[4];

	int time_1, time_2, time_3;

	/*


	 int trapezoidal_distance[0] = (1 / (2 * acceration)) * (max_velocity - initial_velocity)
	 * (max_velocity + initial_velocity);
	 int trapezoidal_distance[1] = distance
	 - (1 / (2 * acceration))
	 * (max_velocity
	 ^ 2 - 4 * initial_velocity * end_velocity
	 + 3 * end_velocity ^ 2);
	 int trapezoidal_distance[2] = -(1 / (2 * acceration))
	 * (initial_velocity - end_velocity)(
	 initial_velocity - 3 * end_velocity);
	 */

	char i;

	//その時に必要な速度を格納する変数
	double velocity = 0.0;

	//time_2がマイナスの値であったときに用いる根号値。解の公式により求まる。
	double square_1 = sqrt(
			initial_velocity * initial_velocity + acceleration * distance);

	double velocity_culc[5];

	double square_2, square_3;

	double up_limit_velocity;	//こいつは、指定した距離を加速し続けた場合の最高速
//	double start_velocity;	//こいつは、指定した距離を減速し続けた場合の最低速

	PORTA.PODR.BIT.B3 = 0;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 1;		//SET_CW/CCW2_OUT

	/*	直進、超新地検知用フラグを直進に（straight_rotate_flag=1;）	*/
	straight_rotate_flag = 1;

	velocity_culc[0] = end_velocity * end_velocity;
	velocity_culc[1] = velocity_culc[0] * velocity_culc[0];

	velocity_culc[2] = initial_velocity * initial_velocity;
	velocity_culc[3] = velocity_culc[2] * velocity_culc[2];

	square_2 = sqrt(
			4 * velocity_culc[2] * velocity_culc[1]
					+ 2 * acceleration * velocity_culc[0] * distance
							* (velocity_culc[2] + velocity_culc[0]));
	square_3 = sqrt(
			4 * velocity_culc[3] * velocity_culc[0]
					+ 2 * acceleration * velocity_culc[2] * distance
							* (velocity_culc[2] + velocity_culc[0]));

//	int imai_count;

	/*
	 if (time_2 <= 0) { 	//time_2の値がマイナスであるとき、三角加速を行うようにtime_1とtime_3を変更する。
	 //time_1の値を三角加速用のものに切り替える。この式は、物理計算により求まった
	 time_1 = (int) (((-1 * initial_velocity + square_1) / (acceration))
	 * 1000);
	 //等速過程は三角加速ではないため、time_2を0にする。
	 time_2 = 0;
	 //とりあえず、time_3の値とtime_1の値を同じにする。
	 time_3 = (int) (time_1);
	 }
	 */
	/*

	 if (initial_velocity >= max_velocity) {
	 if (extern_max_velocity != 0)
	 initial_velocity = extern_max_velocity;
	 else
	 initial_velocity = max_velocity;
	 }

	 if (end_velocity >= max_velocity) {
	 end_velocity = max_velocity;
	 }
	 */

//	myprintf("%d,%d,%d\n",time_1,time_2,time_3);
//	myprintf("%f,%f\n",max_velocity,distance);
	//加速時間、等速時間、減速時間の計算
	//加速時間の計算(=time_1)
	time_1 = (((max_velocity - initial_velocity) / acceleration) * 1000) + 0.5;
	//等速時間の計算(=time_2)
	time_2 = (((1 / max_velocity)
			* (distance
					- (1 / (2 * acceleration)) * (max_velocity - end_velocity)
							* (max_velocity + end_velocity)
					- (1 / (2 * acceleration))
							* (max_velocity - initial_velocity)
							* (max_velocity + initial_velocity))) * 1000) + 0.5;

	//減速時間の計算(=time_3)
	time_3 = (((max_velocity - end_velocity) / acceleration) * 1000) + 0.5;

	//以下動作確認未実施
//	myprintf("%d %d %d\n",time_1,time_2,time_3);
	if (time_2 <= 0) { 	//time_2の値がマイナスであるとき、三角加速を行うようにtime_1とtime_3を変更する。
//time_1の値を三角加速用のものに切り替える。この式は、物理計算により求まった
		time_2 = 0;

		/*
		 up_limit_velocity = sqrt(
		 initial_velocity * initial_velocity
		 + 2 * acceleration * distance);
		 if (end_velocity >= up_limit_velocity)
		 end_velocity = up_limit_velocity;
		 */

		time_1 =
				(((-1 * 2 * initial_velocity * velocity_culc[0] + square_2)
						/ (acceleration * (velocity_culc[2] + velocity_culc[0])))
						* 1000) + 0.5;
//等速過程は三角加速ではないため、time_2を0にする。

		time_3 =
				(((-1 * 2 * end_velocity * velocity_culc[2] + square_3)
						/ (acceleration * (velocity_culc[2] + velocity_culc[0])))
						* 1000) + 0.5;

		max_velocity = initial_velocity + acceleration * time_1 * 0.001;
	}
	myprintf("time_1 = %d,time_2 = %d,time_3 = %d\n", time_1, time_2, time_3);
	trapezoidal_distance[0] = initial_velocity * time_1
			+ 0.5 * acceleration * pow(time_1 * pow(10, -3), 2);
	;
	trapezoidal_distance[1] = max_velocity * time_2;
	trapezoidal_distance[2] = end_velocity * time_3
			+ 0.5 * acceleration * pow(time_3 * pow(10, -3), 2);

	for (i = 0; i < 3; i++) {
		myprintf("distance_%d = %f\n", i, trapezoidal_distance[i]);
	}
//	extern_max_velocity = max_velocity;
//	myprintf("%d %d %d\n", time_1, time_2, time_3);
//以上動作確認未実施

//extern global変数であるcmt_countの値をリセットする。cmt割り込みが入るたびに加算されている。
	cmt_count = 0;

	while (1) {

		if (cmt_count <= time_1)
			flag = 0;
		else if (time_1 < cmt_count && cmt_count < time_1 + time_2)
			flag = 1;
		else if (time_1 + time_2 <= cmt_count
				&& cmt_count <= time_1 + time_2 + time_3)
			flag = 2;
		else {
			flag = 3;
		}

		if (velocity < max_velocity && flag == 0) {
			velocity = initial_velocity + (acceleration * cmt_count * 0.001);
//			myprintf("kasoku\n");
		} else if (velocity <= max_velocity && flag == 2) {
			velocity =
					max_velocity
							- (1 * acceleration * (cmt_count - time_1 - time_2)
									* 0.001);
//			myprintf("gensoku\n");
		} else if (velocity >= max_velocity || flag == 1) {
			velocity = max_velocity;
			if (flag == 3) {
				led(6);
				cmt_interrupt_trapezoidal_velocity = end_velocity;
				straight_rotate_flag = 0;
				break;
			}
		} else {
			cmt_interrupt_trapezoidal_velocity = end_velocity;
			straight_rotate_flag = 0;
			break;
		}
//	myprintf("%f\n",velocity);
		cmt_interrupt_trapezoidal_velocity = velocity;
	}
}
//台形加速処理
//引数は、進みたい距離、最高速、初速、終速、加速度。返し値はなし。
//動作はコメントアウトの通り
void trapezoidal_acceleration_left_rotate(double degree, double max_velocity,
		double initial_velocity, double end_velocity, double acceleration) {
	int flag = 3;

	double distance = MY_PI * TREAD_WIDTH_LEFT * 0.001 * degree / 360;

	int time_1, time_2, time_3;

//その時に必要な速度を格納する変数
	double velocity = 0.0;

//time_2がマイナスの値であったときに用いる根号値。解の公式により求まる。
	double square_1 = sqrt(
			initial_velocity * initial_velocity + acceleration * distance);

	double velocity_culc[5];

	double square_2, square_3;

	PORTA.PODR.BIT.B3 = 1;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 1;		//SET_CW/CCW2_OUT

	/*	直進、超新地検知用フラグを直進に（straight_rotate_flag=1;）	*/
	straight_rotate_flag = 2;

	velocity_culc[0] = end_velocity * end_velocity;
	velocity_culc[1] = velocity_culc[0] * velocity_culc[0];

	velocity_culc[2] = initial_velocity * initial_velocity;
	velocity_culc[3] = velocity_culc[2] * velocity_culc[2];

	square_2 = sqrt(
			4 * velocity_culc[2] * velocity_culc[1]
					+ 2 * acceleration * velocity_culc[0] * distance
							* (velocity_culc[2] + velocity_culc[0]));
	square_3 = sqrt(
			4 * velocity_culc[3] * velocity_culc[0]
					+ 2 * acceleration * velocity_culc[2] * distance
							* (velocity_culc[2] + velocity_culc[0]));

//加速時間、等速時間、減速時間の計算
//加速時間の計算(=time_1)
	time_1 = (((max_velocity - initial_velocity) / acceleration) * 1000) + 0.5;
//等速時間の計算(=time_2)
	time_2 = (((1 / max_velocity)
			* (distance
					- (1 / (2 * acceleration)) * (max_velocity - end_velocity)
							* (max_velocity + end_velocity)
					- (1 / (2 * acceleration))
							* (max_velocity - initial_velocity)
							* (max_velocity + initial_velocity))) * 1000) + 0.5;

//減速時間の計算(=time_3)
	time_3 = (((max_velocity - end_velocity) / acceleration) * 1000) + 0.5;

//以下動作確認未実施
//	myprintf("%d %d %d\n",time_1,time_2,time_3);
	if (time_2 <= 0) { 	//time_2の値がマイナスであるとき、三角加速を行うようにtime_1とtime_3を変更する。
//time_1の値を三角加速用のものに切り替える。この式は、物理計算により求まった
		time_2 = 0;

		time_1 =
				(((-1 * 2 * initial_velocity * velocity_culc[0] + square_2)
						/ (acceleration * (velocity_culc[2] + velocity_culc[0])))
						* 1000) + 0.5;
//等速過程は三角加速ではないため、time_2を0にする。

		time_3 =
				(((-1 * 2 * end_velocity * velocity_culc[2] + square_3)
						/ (acceleration * (velocity_culc[2] + velocity_culc[0])))
						* 1000) + 0.5;

		max_velocity = initial_velocity + acceleration * time_1 * 0.001;
	}
//以上動作確認未実施

//extern global変数であるcmt_countの値をリセットする。cmt割り込みが入るたびに加算されている。
	cmt_count = 0;
	while (1) {

		if (cmt_count <= time_1)
			flag = 0;
		else if (time_1 < cmt_count && cmt_count < time_1 + time_2)
			flag = 1;
		else if (time_1 + time_2 <= cmt_count
				&& cmt_count <= time_1 + time_2 + time_3)
			flag = 2;
		else {
			flag = 3;
		}

		if (velocity < max_velocity && flag == 0) {
			velocity = initial_velocity + (acceleration * cmt_count * 0.001);
		} else if (velocity <= max_velocity && flag == 2) {
			velocity =
					max_velocity
							- (1 * acceleration * (cmt_count - time_1 - time_2)
									* 0.001);
		} else if (velocity >= max_velocity || flag == 1) {
			velocity = max_velocity;
		} else {
			cmt_interrupt_trapezoidal_velocity_rotate = end_velocity;
			straight_rotate_flag = 0;
			break;
		}
		cmt_interrupt_trapezoidal_velocity_rotate = velocity;
	}
}

//台形加速処理
//引数は、進みたい距離、最高速、初速、終速、加速度。返し値はなし。
//動作はコメントアウトの通り
void trapezoidal_acceleration_right_rotate(double degree, double max_velocity,
		double initial_velocity, double end_velocity, double acceleration) {
	int flag = 3;

	double distance = MY_PI * TREAD_WIDTH_RIGHT * 0.001 * degree / 360;

	int time_1, time_2, time_3;

//その時に必要な速度を格納する変数
	double velocity = 0.0;

//time_2がマイナスの値であったときに用いる根号値。解の公式により求まる。
	double square_1 = sqrt(
			initial_velocity * initial_velocity + acceleration * distance);

	double velocity_culc[5];

	double square_2, square_3;

	PORTA.PODR.BIT.B3 = 0;		//SET_CW/CCW1_OUT
	PORTA.PODR.BIT.B4 = 0;		//SET_CW/CCW2_OUT

	/*	直進、超新地検知用フラグを直進に（straight_rotate_flag=1;）	*/
	straight_rotate_flag = 2;

	velocity_culc[0] = end_velocity * end_velocity;
	velocity_culc[1] = velocity_culc[0] * velocity_culc[0];

	velocity_culc[2] = initial_velocity * initial_velocity;
	velocity_culc[3] = velocity_culc[2] * velocity_culc[2];

	square_2 = sqrt(
			4 * velocity_culc[2] * velocity_culc[1]
					+ 2 * acceleration * velocity_culc[0] * distance
							* (velocity_culc[2] + velocity_culc[0]));
	square_3 = sqrt(
			4 * velocity_culc[3] * velocity_culc[0]
					+ 2 * acceleration * velocity_culc[2] * distance
							* (velocity_culc[2] + velocity_culc[0]));

//加速時間、等速時間、減速時間の計算
//加速時間の計算(=time_1)
	time_1 = (((max_velocity - initial_velocity) / acceleration) * 1000) + 0.5;
//等速時間の計算(=time_2)
	time_2 = (((1 / max_velocity)
			* (distance
					- (1 / (2 * acceleration)) * (max_velocity - end_velocity)
							* (max_velocity + end_velocity)
					- (1 / (2 * acceleration))
							* (max_velocity - initial_velocity)
							* (max_velocity + initial_velocity))) * 1000) + 0.5;

//減速時間の計算(=time_3)
	time_3 = (((max_velocity - end_velocity) / acceleration) * 1000) + 0.5;

//以下動作確認未実施
//	myprintf("%d %d %d\n",time_1,time_2,time_3);
	if (time_2 <= 0) { 	//time_2の値がマイナスであるとき、三角加速を行うようにtime_1とtime_3を変更する。
//time_1の値を三角加速用のものに切り替える。この式は、物理計算により求まった
		time_2 = 0;

		time_1 =
				(((-1 * 2 * initial_velocity * velocity_culc[0] + square_2)
						/ (acceleration * (velocity_culc[2] + velocity_culc[0])))
						* 1000) + 0.5;
//等速過程は三角加速ではないため、time_2を0にする。

		time_3 =
				(((-1 * 2 * end_velocity * velocity_culc[2] + square_3)
						/ (acceleration * (velocity_culc[2] + velocity_culc[0])))
						* 1000) + 0.5;

		max_velocity = initial_velocity + acceleration * time_1 * 0.001;
	}
//以上動作確認未実施

//extern global変数であるcmt_countの値をリセットする。cmt割り込みが入るたびに加算されている。
	cmt_count = 0;
	while (1) {

		if (cmt_count <= time_1)
			flag = 0;
		else if (time_1 < cmt_count && cmt_count < time_1 + time_2)
			flag = 1;
		else if (time_1 + time_2 <= cmt_count
				&& cmt_count <= time_1 + time_2 + time_3)
			flag = 2;
		else {
			flag = 3;
		}

		if (velocity < max_velocity && flag == 0) {
			velocity = initial_velocity + (acceleration * cmt_count * 0.001);
		} else if (velocity <= max_velocity && flag == 2) {
			velocity =
					max_velocity
							- (1 * acceleration * (cmt_count - time_1 - time_2)
									* 0.001);
		} else if (velocity >= max_velocity || flag == 1) {
			velocity = max_velocity;
		} else {
			cmt_interrupt_trapezoidal_velocity_rotate = end_velocity;
			straight_rotate_flag = 0;
			break;
		}
		cmt_interrupt_trapezoidal_velocity_rotate = velocity;
	}
}

void trapezoidal_acceleration_l(double distance, double max_velocity,
		double initial_velocity, double end_velocity, double acceleration) {

	int flag = 3;

	int time_1 = ((max_velocity - initial_velocity) / acceleration) * 1000;
	int time_2 = ((1 / max_velocity)
			* (distance
					- (1 / acceleration) * (max_velocity - end_velocity)
							* (max_velocity + end_velocity))) * 1000;
	int time_3 = ((max_velocity - end_velocity) / acceleration) * 1000;
	double velocity = 0.0;

	double square = sqrt(
			initial_velocity * initial_velocity + acceleration * distance);

	cmt_count = 0;

	MTU.TSTR.BIT.CST2 = 1; /*	MTU2の開始	*/

	if (time_2 < 0) {
		time_1 = (int) (((-1 * initial_velocity + square) / (acceleration))
				* 1000);
		time_2 = 0;
		time_3 = (int) (time_1);
	}
	while (1) {

		if (cmt_count <= time_1)
			flag = 0;
		else if (time_1 < cmt_count && cmt_count < time_1 + time_2)
			flag = 1;
		else if (time_1 + time_2 <= cmt_count
				&& cmt_count <= time_1 + time_2 + time_3)
			flag = 2;
		else {
			flag = 3;
		}

		cmt_interrupt_trapezoidal_velocity = velocity;

		if (velocity < max_velocity && flag == 0) {
			velocity = initial_velocity + (acceleration * cmt_count * 0.001);
		} else if (velocity <= max_velocity && flag == 2) {
			velocity = initial_velocity
					- (-1 * acceleration
							* (time_1 + time_2 + time_3 - cmt_count) * 0.001);
		} else if (velocity == max_velocity || flag == 1) {
			velocity = max_velocity;
		} else {
			cmt_interrupt_trapezoidal_velocity = velocity;
			MTU.TSTR.BIT.CST2 = 0; /*	MTU2の停止	*/
			break;
		}
	}
}

void small_slalom(double offset_before_distance, double offset_after_distance,
		double machine_velocity, double angular_acceleration,
		double arc_start_angular, double arc_end_angular,
		double klothoid_end_angular, signed char direction,
		signed char wall_out_flg, int wall_out_value) {

	extern double wall_control_constant;

	double machine_angular = 0.0;
	double angular_velocity = 0.0;

	double buff_wall_control_constant = 0;

	slalom_mode = 1;

	klothoid_end_angular = klothoid_end_angular * MY_PI / 180.0;
	arc_start_angular = arc_start_angular * MY_PI / 180.0;
	arc_end_angular = arc_end_angular * MY_PI / 180.0;
	//始めのオフセット区間

	direction -= 2;
	if (wall_out_flg == 1) {
		wall_out(offset_before_distance, wall_out_value, direction);
	} else {
		trapezoidal_acceleration_straight_distance(offset_before_distance,
				machine_velocity, machine_velocity, machine_velocity, 2.0, 2.0);
	}
	cmt_angular_velocity = 0.0;
	cmt_machine_angular = 0.0;
	straight_rotate_flag = 4;

	cmt_interrupt_trapezoidal_velocity_right = machine_velocity;
	cmt_interrupt_trapezoidal_velocity_left = machine_velocity;

	cmt_angular_acceleration = angular_acceleration;

	if (direction == 1)
		led(5);
	else
		led(1);
	//始めのクロソイド区間
	while (cmt_machine_angular <= arc_start_angular) {

		cmt_interrupt_trapezoidal_velocity_right = machine_velocity
				- direction * TREAD_WIDTH / 2 * 0.001 * cmt_angular_velocity;
		cmt_interrupt_trapezoidal_velocity_left = machine_velocity
				+ direction * TREAD_WIDTH / 2 * 0.001 * cmt_angular_velocity;

//		myprintf("%f %f %f\n", cmt_interrupt_trapezoidal_velocity_left,
//				cmt_interrupt_trapezoidal_velocity_right,
//				(cmt_interrupt_trapezoidal_velocity_left
//						+ cmt_interrupt_trapezoidal_velocity_right) / 2.0);
	}
//	myprintf("%f\n", cmt_machine_angular);

	if (direction == 1)
		led(2);
	else
		led(2);

	angular_velocity = cmt_angular_velocity;
	//円弧部分
	while (cmt_machine_angular <= arc_end_angular) {

//		myprintf("%f %f\n", cmt_interrupt_trapezoidal_velocity_left,
//				cmt_interrupt_trapezoidal_velocity_right);
	}

	cmt_angular_velocity = angular_velocity;

	if (direction == 1)
		led(1);
	else
		led(5);	//クロソイド曲線終わりのやつ
	while (cmt_machine_angular <= klothoid_end_angular) {

		cmt_interrupt_trapezoidal_velocity_right = machine_velocity
				- direction * TREAD_WIDTH / 2 * 0.001 * cmt_angular_velocity;
		cmt_interrupt_trapezoidal_velocity_left = machine_velocity
				+ direction * TREAD_WIDTH / 2 * 0.001 * cmt_angular_velocity;

//		myprintf("%f %f\n", cmt_interrupt_trapezoidal_velocity_left,
//				cmt_interrupt_trapezoidal_velocity_right);
	}

//	myprintf("%f\n", cmt_machine_angular);

	//終わりのオフセット区間

	straight_rotate_flag = 4;
	trapezoidal_acceleration_straight_distance(offset_after_distance,
			machine_velocity, machine_velocity, machine_velocity, 2.0, 2.0);

	slalom_mode = 0;
}

void rotate_machine(int degree, int velocity) {
	int distance = (int) (MY_PI * TREAD_WIDTH_RIGHT * degree / 360);
	motor_act_r(distance, velocity);
	motor_act_l(distance, velocity);
}

//velocity[m/s]
int motor_l(double velocity) {
	double mtu0_tgra_value = 0;

	if (velocity == 0) {
//		led(7);
		return 0;	 // velocity=0のとき、プログラムに入らない
	}
//	PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_OFF

	mtu0_tgra_value = MY_PI * LEFT_DIAMETER_TIRE * ANGLE_ROTATION
			* MTU_COUNT_FREQUENCY
			/ (ONE_REVOLUTION * PATTERN_EXCITATION * velocity);
	mtu0_tgra_value = (int) (mtu0_tgra_value + 0.5);

	return (int) mtu0_tgra_value;

}
int motor_r(double velocity) {
	double mtu2_tgra_value = 0;

	if (velocity == 0)
//		led(7);
		return 0;	 // velocity=0のとき、プログラムに入らない

	mtu2_tgra_value = MY_PI * RIGHT_DIAMETER_TIRE * ANGLE_ROTATION
			* MTU_COUNT_FREQUENCY
			/ (ONE_REVOLUTION * PATTERN_EXCITATION * velocity);
	mtu2_tgra_value = (int) (mtu2_tgra_value + 0.5);

	return (int) mtu2_tgra_value;

}

//distance[m],velocity[m/s]
void motor_act_r(double distance, double velocity) {
	double mtu0_tgra_value = 0;

	volatile double time_count = (distance / velocity) * 1000;
	volatile int int_count = 0;

	if (velocity == 0)
		return;	// velocity=0のとき、プログラムに入らない

	mtu0_tgra_value = MY_PI * RIGHT_DIAMETER_TIRE * ANGLE_ROTATION
			* MTU_COUNT_FREQUENCY
			/ (ONE_REVOLUTION * PATTERN_EXCITATION * velocity);
	mtu0_tgra_value = (int) (mtu0_tgra_value + 0.5);

	int_count = (int) (time_count);

	PORTA.PODR.BIT.B1 = 1;		//励磁をかける
	pwm0(mtu0_tgra_value, 20);

	cmt_count = 0;

	while (int_count > cmt_count)
		;
	PORTA.PODR.BIT.B1 = 0;		//励磁をかけない
}

void motor_act_l(double distance, double velocity) {
	double mtu2_tgra_value = 0;

	volatile double time_count = (distance / velocity) * 1000;
	volatile int int_count = 0;

	mtu2_tgra_value = MY_PI * LEFT_DIAMETER_TIRE * ANGLE_ROTATION
			* MTU_COUNT_FREQUENCY
			/ (ONE_REVOLUTION * PATTERN_EXCITATION * velocity);
	mtu2_tgra_value = (int) (mtu2_tgra_value + 0.5);

	int_count = (int) (time_count);

	PORTA.PODR.BIT.B1 = 1;		//励磁をかける
	pwm2(mtu2_tgra_value, 20);

	cmt_count = 0;

	while (int_count > cmt_count)
		;
	PORTA.PODR.BIT.B1 = 0;		//励磁をかけない
}

void determine_tgra(double want_velocity, double want_time) {
	double mtu0_tgra_value = 0;

	mtu0_tgra_value = MY_PI * RIGHT_DIAMETER_TIRE * want_time * ANGLE_ROTATION
			* MTU_COUNT_FREQUENCY
			/ (ONE_REVOLUTION * PATTERN_EXCITATION * want_velocity);
	mtu0_tgra_value = (int) (mtu0_tgra_value + 0.5);

	PORTA.PODR.BIT.B1 = 1;		//SLEEP1_OFF
	pwm0(mtu0_tgra_value, 20);
}

void pwm0(int period, int duty) {
	if (period >= 65535 || period <= duty)
		return;
	MTU0.TGRA = period;
	MTU0.TGRB = duty;

	MTU.TSTR.BIT.CST0 = 1; /*	MTU0の開始	*/
}
void pwm2(int period, int duty) {
	if (period >= 65535 || period <= duty)
		return;
	MTU2.TGRA = period;
	MTU2.TGRB = duty;

	MTU.TSTR.BIT.CST2 = 1; /*	MTU2の開始	*/
}
void pwm4(int period, int duty) {
	MTU4.TGRC = period;
	MTU4.TGRD = duty;

	MTU.TSTR.BIT.CST4 = 1; /*	MTU4の開始	*/
}

void higashinihon_circuit(double velocity, double acceleration) {
	led(7);
	__delay_ms(1000);
	led(3);
	__delay_ms(1000);
	led(1);
	__delay_ms(1000);
	led(0);

	//1回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);
	//2回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);

	//3回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);

	//4回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);

	//5回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);

	//6回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);

	//7回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);

	//8回
	led(2);
	trapezoidal_acceleration_straight(BLOCK_LENGTH * 15, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;
	__delay_ms(200);
	led(1);
	trapezoidal_acceleration_right_rotate(90, 1.0, 0.15, 0.15, 3.0);
	MTU_STOP
	;
	__delay_ms(200);
	led(2);

	trapezoidal_acceleration_straight(BLOCK_LENGTH * 1, velocity, 0.15, 0.15,
			acceleration);
	MTU_STOP
	;

}
