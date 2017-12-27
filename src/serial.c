/*
 * serial.c
 *
 *  Created on: 2016/05/04
 *      Author: �x
 */

#include "iodefine.h"
//���ӁFiodefine.h�̑��d�C���N���[�h���N���肦�܂�
//�C���N���[�h�K�[�h������悤�ɂ��Ă�������
#include <stdio.h>
#include <stdarg.h>

void init_sci1(void) {	//38400 bps

	//�����d�̓��[�h���W�X�^�̃v���e�N�g����
	SYSTEM.PRCR.WORD = 0xA502;

	//SIC1�̒����d�̓��[�h����
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;

	//�����d�̓��[�h���W�X�^�̃v���e�N�g
	SYSTEM.PRCR.WORD = 0xA500;

	SCI1.SCR.BIT.TIE = 0;	//TXI���荞�݋֎~
	SCI1.SCR.BIT.RIE = 0;	//RXI���荞�݋֎~

	SCI1.SCR.BIT.TE = 0;	//���M����֎~
	SCI1.SCR.BIT.RE = 0;	//��M����֎~

	SCI1.SCR.BIT.TEIE = 0;	//�ʐM�I�����荞�݋֎~

	SCI1.SCR.BYTE = 0x00;

	PORT2.PMR.BIT.B6 = 0; //���Ӌ@�\�Ƃ��Ďg��
	PORT3.PMR.BIT.B0 = 0; //���Ӌ@�\�Ƃ��Ďg��

	MPC.PWPR.BIT.B0WI = 0; //PFSWE�v���e�N�g����
	MPC.PWPR.BIT.PFSWE = 1; //PFS�̃v���e�N�g����

	//�[�q�@�\���W�X�^
	MPC.P26PFS.BIT.PSEL = 10; //P26��TXD�Ɍ���
	MPC.P30PFS.BIT.PSEL = 10; //P30��RXD�Ɍ���

	MPC.PWPR.BIT.PFSWE = 0; //PFS�̃v���e�N�g
	MPC.PWPR.BIT.B0WI = 1; //PFSWE�v���e�N�g

	PORT2.PMR.BIT.B6 = 1; //���Ӌ@�\�Ƃ��Ďg��
	PORT3.PMR.BIT.B0 = 1; //���Ӌ@�\�Ƃ��Ďg��

	SCI1.SIMR1.BIT.IICM = 0; //�V���A���C���^�[�t�F�[�X���[�h

	//���M�t�H�[�}�b�g
	SCI1.SMR.BIT.CM = 0; //����������
	SCI1.SMR.BIT.CHR = 0; //�f�[�^��8�r�b�g
	SCI1.SMR.BIT.CKS = 0; //PCLK/1
	SCI1.SMR.BIT.PE = 0; //�p���e�B����
	SCI1.SMR.BIT.STOP = 0; //STOP�@�\

	SCI1.SCMR.BIT.SMIF = 0;

	SCI1.SEMR.BIT.ABCS = 0; //16�T�C�N���œ]�����[�g
	//SCI1.SEMR.BIT.NFEN = 0; //�m�C�Y�����@�\

	SCI1.BRR = 40; //80

	SCI1.SCR.BIT.TE = 1;	//���M���싖��
	SCI1.SCR.BIT.RE = 1;	//��M���싖��

}



//���M����
void put1byte(char c) {

//	while ( SCI1.SCSSR.BIT.TDRE == 0)
//		;
//	SCI1.SCSSR.BIT.TDRE = 0;
//	SCI1.SCTDR = c;

	while ( SCI1.SSR.BIT.TEND == 0)
		;
	SCI1.SSR.BIT.TEND = 0;
	SCI1.TDR = c;

}

void putnbyte(char *buf, int len) {
	int c;
	for (c = 0; c < len; c++) {
		put1byte(buf[c]);
	}
}

int myprintf(const char *fmt, ...) {
	static char buffer[300];
	volatile long len;

	va_list ap;
	va_start(ap, fmt);

	len = vsprintf(buffer, fmt, ap);

	putnbyte(buffer, len);
	va_end(ap);
	return len;
}

/*
 How to use.
 //	init_sci1();
 //	myprintf("%d\n\r", value);
 */
