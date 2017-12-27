#include "iodefine.h"
#include "serial.h"
#include "rx631_init.h"
#include "motor.h"
#include "motor_define.h"
#include "wall_control.h"
#include "wall_control_define.h"
#include "search.h"

int main(void) {
	int i;
	for (i = 0; i < 6; i++)
		ad_data[i] = 0;

	Usage_init();
	led(4);
//	__delay_ms(2000);
	PORTA.PODR.BIT.B1 = 0;		//励磁をかけnai
	/*

	 MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
	 MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始
	 */

//	 trapezoidal_acceleration_straight(BLOCK_LENGTH * 5, 0.5, 0.2, 0.2, 1.0);
	/*

	 MTU.TSTR.BIT.CST0 = 0; 	//MTU0の停止
	 MTU.TSTR.BIT.CST2 = 0; 	//MTU2の停止

	 */

//	PORTA.PODR.BIT.B1 = 0;		//励磁をかける
	/*
	 MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
	 MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始
	 */
//	trapezoidal_acceleration_left_rotate(180, 0.5, 0.2, 0.2, 1.0);
//	trapezoidal_acceleration_right_rotate(90, 0.5, 0.2, 0.5, 1.0);
//	myprintf("%d\n\n",a_wall_read(14,0,1));
	while (1) {

		led(7);

		choose_mode();
		if (mode_count != 2 && mode_count != 1) {
			start_led();
		} else {
			determine_led();
		}
		PORTA.PODR.BIT.B1 = 1;
		wall_control_constant = WALL_CONTROL;
		while (1) {
			if (mode_count == 0) {
				//センサ値見る用
				led(0);
				sensor_led_flg = 1;
				MOTOR_EXITATION = 0;
				while (1) {
					myprintf("%4d,%4d,%4d,%4d,%4d, %5f\n", ad_data[0],
							ad_data[1], ad_data[2], ad_data[3], ad_data[4],
							Lipo_ad_data(ad_data[5]));
				}
				//センサ値見る用
			} else if (mode_count == 1) {
				//調整用
				adjust_all_slalom();
				sensor_led_flg = 0;
				//	trapezoidal_acceleration_left_rotate(360*10, 0.3, 0.15, 0.15,
				//2.0);
				MTU_STOP
				;

				__delay_ms(300);
				MOTOR_EXITATION = 0;
				//調整用
			} else if (mode_count == 2) {

				wall_control_constant = WALL_CONTROL;
				contest_search_shortest();

			} else if (mode_count == 3) {

				//			contest_auto_start();

				wall_control_constant = 0;
				trapezoidal_acceleration_straight_distance(
						BLOCK_LENGTH * 9, 2, 0.15, 0.15, 2.3, 3.5);

				MTU_STOP
				;

				__delay_ms(250);
				MOTOR_reset();
				__delay_ms(500);

	//			for(i=0;i<10;i++){
				trapezoidal_acceleration_right_rotate(180, 0.3, 0.15, 0.15,
						2.0);
				MTU_STOP;
				__delay_ms(200);
		//		}
				MTU_STOP
				;

				__delay_ms(250);
				MOTOR_reset();
				__delay_ms(500);
				trapezoidal_acceleration_straight_distance(
						BLOCK_LENGTH * 9, 2., 0.15, 0.15, 2.3, 3.5);
				MTU_STOP
				;

				__delay_ms(250);
				MOTOR_reset();
				__delay_ms(500);

				MOTOR_EXITATION = 1;
			} else if (mode_count == 4) {
				//壁切れとか用
				wall_control_constant = 0;
				wall_log();
				start_switch();
				for (i = 0; i <= 1000; i++) {
					myprintf("%f	%d\n", 0.8 * i * 0.001, sensor_log[i]);
				}
				//壁切れとか用
			} else if (mode_count == 5) {
				//サーキット用

				//1回目
				PORTE.PODR.BIT.B0 = 1;		//LEFT-SENSOR_LED_OFF
				PORTE.PODR.BIT.B1 = 1;		//ForwardLEFT-SENSOR_LED_OFF
				PORTE.PODR.BIT.B2 = 1;		//Forward-SENSOR_LED_OFF
				PORTE.PODR.BIT.B3 = 1;		//RIGHT-SENSOR_LED_OFF
				PORTE.PODR.BIT.B4 = 1;			//ForwardRIGHT-SENSOR_LED_OFF

				__delay_ms(1);

				S12AD.ADCSR.BIT.ADST = 0;

				S12AD.ADANS0.WORD = 0x5F;

				S12AD.ADCSR.BIT.ADST = 1;
				while (S12AD.ADCSR.BIT.ADST == 1)
					;
				S12AD.ADCSR.BIT.ADST = 0;
				ad_data[0] = S12AD.ADDR0;
				ad_data[1] = S12AD.ADDR1;
				ad_data[2] = S12AD.ADDR2;
				ad_data[3] = S12AD.ADDR3;
				ad_data[4] = S12AD.ADDR4;
				ad_data[5] = S12AD.ADDR6;

				myprintf("%d\n",ad_data[0]);
				/*
				for (i = 0; i <= 1000; i++) {
					myprintf("%f	%d\n", 0.8 * i * 0.001, ad_data[1]);
				}
				*/
				//サーキット用1

			} else if (mode_count == 6) {
				//スラローム試す用
				//			sensor_led_flg =1;
				circuit(1.8, 2.5, 2.8);
			} else if (mode_count == 7) {
				circuit(2.0, 2.5, 2.8);
			}
		}
	}
}

//		__delay_ms(10);
//		myprintf("%4d,%4d,%4d,%4d\n",ad_data[0],ad_data[1],ad_data[3],ad_data[4]);

/*for(i=60000;i>100;i=i-100){
 pwm4(i, i/2);
 myprintf("%d\n",i);
 }*/
//		led(0);
/*
 trapezoidal_acceleration_straight(BLOCK_LENGTH * 3, 0.45, 0.15, 0.15, 1.0);
 MTU_STOP
 ;
 __delay_ms(220);
 trapezoidal_acceleration_left_rotate(180, 0.5, 0.15, 0.15, 1.0);
 MTU_STOP
 ;
 __delay_ms(220);
 trapezoidal_acceleration_straight(BLOCK_LENGTH * 3, 0.45, 0.15, 0.15, 1.0);
 MTU_STOP
 ;
 __delay_ms(220);*/

//		left_hand(TARGET_X_COORDINATE,TARGET_Y_COORDINATE);
/*

 a_wall_add(6,6,1);
 a_wall_add(6,5,1);
 a_wall_add(6,4,1);
 a_wall_add(6,3,1);
 a_wall_add(6,2,1);
 a_wall_add(6,1,1);
 a_wall_add(6,0,1);
 a_wall_add(0,6,0);
 a_wall_add(1,6,0);
 a_wall_add(2,6,0);
 a_wall_add(3,6,0);
 a_wall_add(4,6,0);
 a_wall_add(5,6,0);
 a_wall_add(6,6,0);
 a_wall_add(1,0,0);
 a_wall_add(2,0,0);
 a_wall_add(3,0,0);
 a_wall_add(4,0,0);
 a_wall_add(5,0,0);
 a_wall_add(5,1,1);
 */

/*
 a_wall_add(1,0,1);
 a_wall_add(2,0,0);
 a_wall_add(2,1,0);
 a_wall_add(3,1,0);
 a_wall_add(0,2,1);
 a_wall_add(1,2,0);
 a_wall_add(2,2,1);

 a_wall_add(3,0,1);
 a_wall_add(3,1,1);
 a_wall_add(3,2,1);
 a_wall_add(3,3,1);

 a_wall_add(0,3,0);
 a_wall_add(1,3,0);
 a_wall_add(2,3,0);
 a_wall_add(3,3,0);
 a_wall_add(13,5,1);


 a_wall_add(0,1,1);
 */
