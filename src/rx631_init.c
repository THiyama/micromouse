/*
 * rx631_init.c
 *
 *  Created on: 2016/07/10
 *      Author: hihumi
 */

#include "rx631_init.h"
#include "iodefine.h"
#include "serial.h"
#include "wall_control.h"
#include "wall_control_define.h"
#include "motor_define.h"

int cmt_1_interrupt_count = 0;

/*	init関数の呼び出し		*/
void Usage_init(void) {
	XTAL_init();	// クロック初期設定の呼び出し
	IO_init();		// ポート入出力設定の呼び出し
	CMT_0_init();		// CMT初期設定の呼び出し
	CMT_1_init();		// CMT初期設定の呼び出し
	MTU0_init();		// MTU初期設定の呼び出し
	MTU4_init();		// MTU初期設定の呼び出し
	MTU2_init();		// MTU初期設定の呼び出し
	MD_init();
	init_sci1();
	AD_init();		// ADC初期設定の呼び出し

	__delay_ms(1);
}

/*	IOピン設定	*/
void IO_init(void) {
	PORTA.PDR.BIT.B1 = 1;		//SET_Sleep1_OUT
	PORTA.PDR.BIT.B3 = 1;		//SET_CW/CCW1_OUT
	PORTA.PDR.BIT.B4 = 1;		//SET_CW/CCW2_OUT
	PORTB.PDR.BIT.B5 = 1;		//SET_Clock2_OUT
	PORTB.PDR.BIT.B3 = 1;		//SET_Clock1_OUT
	PORTB.PDR.BIT.B5 = 1;		//SET_Reset2_OUT
	PORTB.PDR.BIT.B6 = 1;		//SET_Reset1_OUT
	PORTB.PDR.BIT.B7 = 1;		//SET_CW/CCW2_OUT
	PORTC.PDR.BIT.B2 = 1;		//SET_interface_LED_OUT
	PORTC.PDR.BIT.B3 = 1;		//SET_interface_LED_OUT
	PORTC.PDR.BIT.B4 = 1;		//SET_interface_LED_OUT
	PORTC.PDR.BIT.B5 = 0;		//SET_SWITCH_IN
	PORTC.PDR.BIT.B6 = 0;		//SET_SWITCH_IN
	PORTE.PDR.BIT.B0 = 1;		//SET_SENSOR_LED_OUT
	PORTE.PDR.BIT.B1 = 1;		//SET_SENSOR_LED_OUT
	PORTE.PDR.BIT.B2 = 1;		//SET_SENSOR_LED_OUT
	PORTE.PDR.BIT.B3 = 1;		//SET_SENSOR_LED_OUT
	PORTE.PDR.BIT.B4 = 1;		//SET_SENSOR_LED_OUT
	PORTC.PODR.BIT.B2 = 1;		//interface_BLUELED_OFF
	PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_OFF
	PORTC.PODR.BIT.B4 = 1;		//interface_REDLED_OFF
	PORTE.PODR.BIT.B0 = 0;		//LEFT-SENSOR_LED_OFF
	PORTE.PODR.BIT.B1 = 0;		//ForwardLEFT-SENSOR_LED_OFF
	PORTE.PODR.BIT.B2 = 0;		//Forward-SENSOR_LED_OFF
	PORTE.PODR.BIT.B3 = 0;		//ForwardRIGHT-SENSOR_LED_OFF
	PORTE.PODR.BIT.B4 = 0;		//RIGHT-SENSOR_LED_OFF

	PORT0.PDR.BYTE = 0x00;

	PORTA.PODR.BIT.B3 = 1;		//SET_CW/CCW1_HI
	PORTA.PODR.BIT.B4 = 1;		//SET_CW/CCW2_HI

	sensor_led_flg = 1;
}

void XTAL_init(void) {
	int xtal_wait_time;
	SYSTEM.PRCR.WORD = 0xA503;
	PORT3.PDR.BIT.B6 = 0;
	PORT3.PDR.BIT.B7 = 0;
	SYSTEM.SOSCCR.BIT.SOSTP = 1;
	SYSTEM.LOCOCR.BIT.LCSTP = 1;
	SYSTEM.ILOCOCR.BIT.ILCSTP = 1;
	SYSTEM.HOCOCR.BIT.HCSTP = 1;
	SYSTEM.HOCOPCR.BIT.HOCOPCNT = 1;
	SYSTEM.MOSCWTCR.BIT.MSTS = 0x0f;
	SYSTEM.PLLWTCR.BIT.PSTS = 0x0f;
	SYSTEM.PLLCR2.BIT.PLLEN = 1;
	SYSTEM.PLLCR.BIT.PLIDIV = 0x01;		//分周器（プリスケーラ）	：2分周      (=8MHz)
	SYSTEM.PLLCR.BIT.STC = 0x18;		//PLL回路（逓倍）		：25倍　　　(=200MHz)
	SYSTEM.PLLCR2.BIT.PLLEN = 0;
	SYSTEM.MOSCCR.BYTE = 0x00;			//
	for (xtal_wait_time = 0; xtal_wait_time <= 0x168; xtal_wait_time++)
		;
	SYSTEM.SCKCR.LONG = 0x21C31211;
	SYSTEM.SCKCR.BIT.ICK = 0x01;	//システムクロックへの分周器（プリスケーラ）	：2分周      (100MHz)
	SYSTEM.SCKCR.BIT.PCKB = 0x02;//周辺モジュールクロックへの分周器（プリスケーラ）	：4分周      (50MHz)
	SYSTEM.SCKCR.BIT.PCKA = 0x02;//周辺モジュールクロックへの分周器（プリスケーラ）	：4分周      (50MHz)
	SYSTEM.SCKCR.BIT.BCK = 0x02;//外部バスクロック選択						：4分周      (50MHz)
	SYSTEM.SCKCR.BIT.FCK = 0x02;//FlashIFクロック選択						：4分周      (50MHz)
	SYSTEM.SCKCR3.BIT.CKSEL = 0x04;		//メインクロック入力（200MHz）
	SYSTEM.PRCR.WORD = 0xA500;
}

void CMT_0_init() {
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;	// CMTモジュールのプロテクト解除
	SYSTEM.PRCR.WORD = 0xA500;
	CMT.CMSTR0.BIT.STR0 = 0;			// CMTの停止
	CMT0.CMCR.BIT.CMIE = 0;			// 割り込み禁止
	CMT0.CMCR.BIT.CKS = 0;				// クロックセレクタ：PCLK/8
	CMT0.CMCR.BIT.CMIE = 1;			// 割り込み許可
	CMT0.CMCOR = 6250 - 1;					// CMCNTのコンペアマッチ値の設定
	IR(CMT0,CMI0)=0x00;
	IPR(CMT0,CMI0)=0x03;
	IEN(CMT0,CMI0)=0x01;
	CMT.CMSTR0.BIT.STR0 = 1;			// CMTの開始
}
void CMT_1_init() {
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;	// CMTモジュールのプロテクト解除
	SYSTEM.PRCR.WORD = 0xA500;
	CMT.CMSTR0.BIT.STR1 = 0;			// CMTの停止
	CMT1.CMCR.BIT.CMIE = 0;			// 割り込み禁止
	CMT1.CMCR.BIT.CKS = 0;				// クロックセレクタ：PCLK/8
	CMT1.CMCR.BIT.CMIE = 1;			// 割り込み許可
	CMT1.CMCOR = 6250 - 1;					// CMCNTのコンペアマッチ値の設定
	IR(CMT1,CMI1)=0x00;
	IPR(CMT1,CMI1)=0x04;
//割り込み要因の優先順位（数字高い方が優先度高い）
	IEN(CMT1,CMI1)=0x01;
	CMT.CMSTR0.BIT.STR1 = 1;			// CMTの開始
}

void AD_init() {
	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA17 = 0;
	SYSTEM.PRCR.WORD = 0xA500;
	__delay_ms(10);
	S12AD.ADCSR.BIT.ADST = 0;
	/*		↓ASEL変更↓		*/
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P40PFS.BIT.ASEL = 1;
	MPC.P41PFS.BIT.ASEL = 1;
	MPC.P42PFS.BIT.ASEL = 1;
	MPC.P43PFS.BIT.ASEL = 1;
	MPC.P44PFS.BIT.ASEL = 1;
	MPC.P46PFS.BIT.ASEL = 1;
//	MPC.PE3PFS.BIT.ASEL = 1;
	MPC.PWPR.BIT.B0WI = 1;
	MPC.PWPR.BIT.PFSWE = 0;
	/*		↑ASEL変更↑		*/
	PORT4.PDR.BYTE = 0x00;
	PORT4.PDR.BIT.B6 = 0;
	PORT4.PMR.BYTE = 0x00;
	PORT4.PMR.BIT.B6 = 0;
//	PORTE.PDR.BYTE |= 0x10;
//	PORTE.PMR.BYTE |= 0x10;
	S12AD.ADCSR.BYTE = 0x0c;
	/*
	 S12AD.ADCSR.BIT.EXTRG=0;
	 S12AD.ADCSR.BIT.TRGE=0;
	 S12AD.ADCSR.BIT.CKS=3;
	 S12AD.ADCSR.BIT.ADCS=0;
	 */
	S12AD.ADSSTR01.BIT.SST1 = 0x1E;
	S12AD.ADEXICR.WORD = 0x00;
	S12AD.ADANS0.WORD = 0x5F;
	S12AD.ADADS0.BIT.ADS0 = 0;
	S12AD.ADADC.BIT.ADC = 0x00;
//	S12AD.ADSSTR01.BIT.SST1 = 0x14;
	S12AD.ADCER.BIT.ADRFMT = 0;
	__delay_ms(15);
}

void MTU0_init() {
	/*		*/
	PORTB.PMR.BIT.B3 = 0;

	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PB3PFS.BIT.PSEL = 0x01;
	MPC.PWPR.BIT.B0WI = 1;
	MPC.PWPR.BIT.PFSWE = 0;

	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	SYSTEM.PRCR.WORD = 0xA500;

	PORTB.PDR.BIT.B3 = 1; /* PB3:出力 */
	PORTB.PMR.BIT.B3 = 1; /* PB3:周辺機能 */

	MTU.TSTR.BIT.CST0 = 0; /*	MTU0の停止	*/

	MTU0.TCR.BIT.TPSC = 0x01; /*	タイマプリスケーラ選択：PCKB/4（50MHz）	*/
	MTU0.TCR.BIT.CKEG = 0x00; /*	立ち上がりエッジでカウント	*/
	MTU0.TCR.BIT.CCLR = 0x01; /*	カウンタクリアビット：TGRAのコンペアマッチ/インプットキャプチャでTCNTクリア	*/

	ICU.SEL.BIT.CN0 = 0;

	MTU0.TIER.BIT.TGIEB = 0x01; /*	TGRBのコンペマッチ割り込み許可	*/

//	IR(MTU0,TGIB0)=0x00;
	IPR(MTU0,TGIB0)=0x01;
	IEN(MTU0,TGIB0)=0x01;

	MTU0.TMDR.BIT.BFA = 0;
	MTU0.TMDR.BIT.BFB = 0;

	MTU0.TMDR.BIT.MD = 0x02; /*	動作選択：PWM1モード	*/

	MTU0.TIORH.BIT.IOA = 0x02;
	MTU0.TIORH.BIT.IOB = 0x01;

	MTU0.TGRA = 8000;
	MTU0.TGRB = 20;

//	MTU.TSTR.BIT.CST0 = 1; /*	MTU0の開始	*/

}

void MTU2_init() {
	/*		*/
	PORTB.PMR.BIT.B5 = 0;

	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PB5PFS.BIT.PSEL = 0x01;
	MPC.PWPR.BIT.B0WI = 1;
	MPC.PWPR.BIT.PFSWE = 0;

	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	SYSTEM.PRCR.WORD = 0xA500;

	PORTB.PDR.BIT.B5 = 1; /* PB5:出力 */
	PORTB.PMR.BIT.B5 = 1; /* PB5:周辺機能 */

	MTU.TSTR.BIT.CST2 = 0; /*	MTU2の停止	*/

	MTU2.TCR.BIT.TPSC = 0x01; /*	タイマプリスケーラ選択：PCKB/4（50MHz）	*/
	MTU2.TCR.BIT.CKEG = 0x00; /*	立ち上がりエッジでカウント	*/
	MTU2.TCR.BIT.CCLR = 0x01; /*	カウンタクリアビット：TGRAのコンペアマッチ/インプットキャプチャでTCNTクリア	*/

	MTU2.TMDR.BIT.MD = 0x02; /*	動作選択：PWM1モード	*/

	ICU.SEL.BIT.CN2 = 0;

	MTU2.TIER.BIT.TGIEB = 0x01; /*	TGRBのコンペマッチ割り込み許可	*/

	IPR(MTU2,TGIB2)=0x02;
	IEN(MTU2,TGIB2)=0x01;

	MTU2.TIOR.BIT.IOA = 0x02;
	MTU2.TIOR.BIT.IOB = 0x01;

	MTU2.TGRA = 8000;
	MTU2.TGRB = 20;

//	MTU.TSTR.BIT.CST2 = 1; /*	MTU2の開始	*/
}

void MTU4_init() {
	/*		*/
	MTU.TRWER.BIT.RWE = 0x01;
	MTU.TOER.BIT.OE4C = 0x01;

	PORTE.PMR.BIT.B5 = 0;

	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PE5PFS.BIT.PSEL = 0x01;
	MPC.PWPR.BIT.B0WI = 1;
	MPC.PWPR.BIT.PFSWE = 0;

	SYSTEM.PRCR.WORD = 0xA502;
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	SYSTEM.PRCR.WORD = 0xA500;

	PORTE.PDR.BIT.B5 = 1; /* PE5:出力 */
	PORTE.PMR.BIT.B5 = 1; /* PE5:周辺機能 */

	MTU.TSTR.BIT.CST4 = 0; /*	MTU4の停止	*/

	MTU4.TCR.BIT.TPSC = 0x01; /*	タイマプリスケーラ選択：PCKB/4（50MHz）	*/
	MTU4.TCR.BIT.CKEG = 0x01; /*	立ち下がりエッジでカウント	*/
	MTU4.TCR.BIT.CCLR = 0x01; /*	カウンタクリアビット：TGRAのコンペアマッチ/インプットキャプチャでTCNTクリア	*/

	MTU4.TMDR.BIT.MD = 0x02; /*	動作選択：PWM1モード	*/

	MTU4.TIORL.BIT.IOC = 0x02;
	MTU4.TIORL.BIT.IOD = 0x01;

	MTU4.TGRC = 40000;
	MTU4.TGRD = 20000;
}

void cmt_0_interrupt() {


	cmt_count++;
	trapezoidal_count++;
	klothoid_count++;
	trapezoidal_count_1++;
	trapezoidal_count_2++;
	trapezoidal_count_3++;

	cmt_angular_velocity += cmt_angular_acceleration * 0.001;
	cmt_machine_angular += cmt_angular_velocity * 0.001;

	if (caluculate_mode == 1) {
		caluculate_count++;
	}

	if (wall_control_constant == 0.0) {
		wall_control_constant = 0.0;
	} else if (acceleration_flag == 1) {
		wall_control_constant = 0.00008;
	} else if (cmt_interrupt_trapezoidal_velocity > 1.0) {
		wall_control_constant = 0.0001;
	} else {
		wall_control_constant = WALL_CONTROL;
	}

	if (straight_rotate_flag == 1) {

		MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
		MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始

		mtu0_tgrb_interrupt_trapezoidal_tgra = motor_r(
				cmt_interrupt_trapezoidal_velocity + wall_control()); //制御量を減らす
		mtu2_tgrb_interrupt_trapezoidal_tgra = motor_l(
				cmt_interrupt_trapezoidal_velocity - wall_control()); //制御量を増やす
	} else if (straight_rotate_flag == 2) {

		MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
		MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始

		mtu0_tgrb_interrupt_trapezoidal_tgra = motor_r(
				cmt_interrupt_trapezoidal_velocity_rotate);
		mtu2_tgrb_interrupt_trapezoidal_tgra = motor_l(
				cmt_interrupt_trapezoidal_velocity_rotate);
	} else if (straight_rotate_flag == 3) {

		MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
		MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始

		mtu0_tgrb_interrupt_trapezoidal_tgra = motor_r(
				cmt_interrupt_trapezoidal_velocity);
		mtu2_tgrb_interrupt_trapezoidal_tgra = motor_l(
				cmt_interrupt_trapezoidal_velocity);
	} else if (straight_rotate_flag == 4) {

		MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
		MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始

		mtu0_tgrb_interrupt_trapezoidal_tgra = motor_l(
				cmt_interrupt_trapezoidal_velocity_right);
		mtu2_tgrb_interrupt_trapezoidal_tgra = motor_r(
				cmt_interrupt_trapezoidal_velocity_left);
	} else if (straight_rotate_flag == 5) {

		MTU.TSTR.BIT.CST0 = 1; 	//MTU0の開始
		MTU.TSTR.BIT.CST2 = 1; 	//MTU2の開始

		mtu0_tgrb_interrupt_trapezoidal_tgra = motor_r(
				cmt_interrupt_trapezoidal_velocity - oblique_wall_control(1)); //制御量を減らす
		mtu2_tgrb_interrupt_trapezoidal_tgra = motor_l(
				cmt_interrupt_trapezoidal_velocity - oblique_wall_control(0)); //制御量を増やす
	} else {

//		 MTU.TSTR.BIT.CST0 = 0; 	//MTU0の開始
//		 MTU.TSTR.BIT.CST2 = 0; 	//MTU2の開始

	}

	if (circuit_flg == 1) {
		if (hihumint_dead_wall() == 1) {
			MTU_STOP
			;
			MOTOR_EXITATION = 0;
			sensor_led_flg = 0;
			circuit_flg = 0;
		}
	}
}

void cmt_1_interrupt() {
	int i;
	int time_1 = 1400;
	int time_2 = 800;


	static int sensor_i;

	if (sensor_led_flg) {

		//1回目
		PORTE.PODR.BIT.B0 = 1;		//LEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B1 = 0;		//ForwardLEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B2 = 0;		//Forward-SENSOR_LED_OFF
		PORTE.PODR.BIT.B3 = 1;		//RIGHT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B4 = 0;			//ForwardRIGHT-SENSOR_LED_OFF

		for (i = 0; i < time_1; i++)
			;

		S12AD.ADCSR.BIT.ADST = 0;

		S12AD.ADANS0.WORD = 0x49;

		S12AD.ADCSR.BIT.ADST = 1;
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADCSR.BIT.ADST = 0;
		ad_data_after[0] = S12AD.ADDR0;
		ad_data_after[3] = S12AD.ADDR3;
		ad_data[5] = S12AD.ADDR6;

		//2回目
		PORTE.PODR.BIT.B0 = 0;		//LEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B1 = 1;		//ForwardLEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B2 = 0;		//Forward-SENSOR_LED_OFF
		PORTE.PODR.BIT.B3 = 0;		//RIGHT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B4 = 1;		//ForwardRIGHT-SENSOR_LED_OFF

		for (i = 0; i < time_1; i++)
			;


		S12AD.ADCSR.BIT.ADST = 0;

		S12AD.ADANS0.WORD = 0x52;

		S12AD.ADCSR.BIT.ADST = 1;
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADCSR.BIT.ADST = 0;

		ad_data_after[1] = S12AD.ADDR1;
		ad_data_after[4] = S12AD.ADDR4;
		ad_data[5] = S12AD.ADDR6;

		//3回目
		PORTE.PODR.BIT.B0 = 0;		//LEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B1 = 0;		//ForwardLEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B2 = 1;		//Forward-SENSOR_LED_ON
		PORTE.PODR.BIT.B3 = 0;		//RIGHT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B4 = 0;			//ForwardRIGHT-SENSOR_LED_OFF

		for (i = 0; i < time_1; i++)
			;

		S12AD.ADCSR.BIT.ADST = 0;

		S12AD.ADANS0.WORD = 0x44;

		S12AD.ADCSR.BIT.ADST = 1;
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADCSR.BIT.ADST = 0;
		ad_data_after[2] = S12AD.ADDR2;
		ad_data[5] = S12AD.ADDR6;

		PORTE.PODR.BIT.B0 = 0;		//LEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B1 = 0;		//ForwardLEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B2 = 0;		//Forward-SENSOR_LED_OFF
		PORTE.PODR.BIT.B3 = 0;		//RIGHT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B4 = 0;		//ForwardRIGHT-SENSOR_LED_OFF

		for (i = 0; i < time_2; i++)
			;
		S12AD.ADCSR.BIT.ADST = 0;

		S12AD.ADANS0.WORD = 0x5F;

		S12AD.ADCSR.BIT.ADST = 1;
		while (S12AD.ADCSR.BIT.ADST == 1)
			;
		S12AD.ADCSR.BIT.ADST = 0;

		ad_data_before[0] = S12AD.ADDR0;
		ad_data_before[3] = S12AD.ADDR3;
		ad_data_before[1] = S12AD.ADDR1;
		ad_data_before[2] = S12AD.ADDR2;
		ad_data_before[4] = S12AD.ADDR4;
		ad_data[5] = S12AD.ADDR6;

		ad_data[0] = ad_data_after[0] - ad_data_before[0];
		ad_data[1] = ad_data_after[1] - ad_data_before[1];
		ad_data[2] = ad_data_after[2] - ad_data_before[2];
		ad_data[3] = ad_data_after[3] - ad_data_before[3];
		ad_data[4] = ad_data_after[4] - ad_data_before[4];


	} else {

		PORTE.PODR.BIT.B0 = 0;		//LEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B1 = 0;		//ForwardLEFT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B2 = 0;		//Forward-SENSOR_LED_OFF
		PORTE.PODR.BIT.B3 = 0;		//RIGHT-SENSOR_LED_OFF
		PORTE.PODR.BIT.B4 = 0;		//ForwardRIGHT-SENSOR_LED_OFF

	}
	if (sensor_log_flg == 1) {
		sensor_log[sensor_i] = ad_data[3];
		sensor_i++;
	} else {
		sensor_i = 0;
	}
}

void __delay_ms(int time_ms) {
	cmt_count = 0;
	while (cmt_count != time_ms)
		;
}

void mtu0_tgrb_interrupt() {

//	while(1 == IR(MTU0,TGIB0));

//	TPU0.TSR.BIT.TGFB=0;

	pwm0(mtu0_tgrb_interrupt_trapezoidal_tgra, 20);
//	motor_l(mtu0_tgrb_interrupt_trapezoidal_velocity);

}

void mtu2_tgrb_interrupt() {
	pwm2(mtu2_tgrb_interrupt_trapezoidal_tgra, 20);
}

void led(int pattern) {
	switch (pattern) {
	case 0:	//状態0
		PORTC.PODR.BIT.B2 = 1;		//interface_BLUELED_OFF
		PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_OFF
		PORTC.PODR.BIT.B4 = 1;		//interface_REDLED_OFF
		break;
	case 1:	//状態1
		PORTC.PODR.BIT.B2 = 0;		//interface_BLUELED_ON
		PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_OFF
		PORTC.PODR.BIT.B4 = 1;		//interface_REDLED_OFF
		break;
	case 2:	//状態2
		PORTC.PODR.BIT.B2 = 1;		//interface_BLUELED_OFF
		PORTC.PODR.BIT.B3 = 0;		//interface_YELLOWLED_ON
		PORTC.PODR.BIT.B4 = 1;		//interface_REDLED_OFF
		break;
	case 3:	//状態3
		PORTC.PODR.BIT.B2 = 0;		//interface_BLUELED_ON
		PORTC.PODR.BIT.B3 = 0;		//interface_YELLOWLED_ON
		PORTC.PODR.BIT.B4 = 1;		//interface_REDLED_OFF
		break;
	case 4:	//状態4
		PORTC.PODR.BIT.B2 = 1;		//interface_BLUELED_ON
		PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_ON
		PORTC.PODR.BIT.B4 = 0;		//interface_REDLED_ON
		break;
	case 5:	//状態5
		PORTC.PODR.BIT.B2 = 0;		//interface_BLUELED_ON
		PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_OFF
		PORTC.PODR.BIT.B4 = 0;		//interface_REDLED_ON
		break;
	case 6:	//状態6
		PORTC.PODR.BIT.B2 = 1;		//interface_BLUELED_OFF
		PORTC.PODR.BIT.B3 = 0;		//interface_YELLOWLED_ON
		PORTC.PODR.BIT.B4 = 0;		//interface_REDLED_ON
		break;
	case 7:	//状態7
		PORTC.PODR.BIT.B2 = 0;		//interface_BLUELED_ON
		PORTC.PODR.BIT.B3 = 0;		//interface_YELLOWLED_ON
		PORTC.PODR.BIT.B4 = 0;		//interface_REDLED_ON
		break;
	default:
		PORTC.PODR.BIT.B2 = 1;		//interface_BLUELED_OFF
		PORTC.PODR.BIT.B3 = 1;		//interface_YELLOWLED_OFF
		PORTC.PODR.BIT.B4 = 1;		//interface_REDLED_OFF
		break;
	}
}

double Lipo_ad_data(int ad_data_lipo) {
	double lipo_voltage;

	int R1 = 240;
	int R2 = 39;

	float ref_voltage = 3.3;
	int ad_data_accuracy = 4096;

	lipo_voltage = (double) (ad_data_lipo * ((R1 + R2) / R2)
			* (ref_voltage / ad_data_accuracy));

	return lipo_voltage;
}

void start_led() {
	straight_rotate_flag = 0;
	MTU_STOP
	;
	MOTOR_EXITATION = 1;
	__delay_ms(500);
	led(7);
	__delay_ms(500);
	led(3);
	__delay_ms(500);
	led(1);
	__delay_ms(500);
	led(0);
}

void determine_led() {
	MTU_STOP
	;
	MOTOR_EXITATION = 0;
	led(7);
	__delay_ms(200);
	led(0);
	__delay_ms(200);
	led(7);
	__delay_ms(200);
	led(0);

}

void start_switch() {
	while (1) {
		if (PORTC.PIDR.BIT.B5 == 0) {
			__delay_ms(20);
			if (PORTC.PIDR.BIT.B5 == 0) {
				break;
			}
		} else
			led(7);
	}
}

/*	mode切り替え	*/
void choose_mode() {

	ad_data[0] = 0;
	ad_data[4] = 0;
	sensor_led_flg = 0;

	MOTOR_EXITATION = 0;
	__delay_ms(100);

	while (1) {
		if (PORTC.PIDR.BIT.B5 == 0) {
			__delay_ms(20);
			if (PORTC.PIDR.BIT.B5 == 0) {
				while (PORTC.PIDR.BIT.B5 == 0)
					;
				mode_count++;
			}
		} else if (mode_count != 0 && PORTC.PIDR.BIT.B6 == 0) {
			__delay_ms(20);
			if (PORTC.PIDR.BIT.B6 == 0) {
				while (PORTC.PIDR.BIT.B6 == 0)
					;
				mode_count--;
			}
		}
		if (mode_count == 0) {
			led(0);
			if (PORTC.PIDR.BIT.B6 == 0) {
				__delay_ms(20);
				if (PORTC.PIDR.BIT.B6 == 0) {
					sensor_led_flg = 1;
				}
			}
		} else if (mode_count == 1) {
			led(1);
		} else if (mode_count == 2) {
			led(2);
		} else if (mode_count == 3) {
			led(3);
		} else if (mode_count == 4) {
			led(4);
		} else if (mode_count == 5) {
			led(5);
		} else if (mode_count == 6) {
			led(6);
		} else if (mode_count == 7) {
			led(7);
		} else {
			mode_count = 0;
		}
		if (sensor_led_flg == 1 && ad_data[4] >= 160 && ad_data[0] >= 160)
			break;

	}
//	start_led();
	MTU_STOP
	;
	MOTOR_EXITATION = 1;
	return;
}
