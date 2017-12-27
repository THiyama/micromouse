/********************************************************************************/
/*                                                                              */
/* Device     : RX/RX600/RX63T                                                  */
/* File Name  : iodefine.h                                                      */
/* Abstract   : Definition of I/O Register.                                     */
/* History    : V1.0F  (2012-06-18)  [Hardware Manual Revision : 1.00]          */
/* Note       : This is a typical example.                                      */
/*                                                                              */
/*  Copyright(c) 2012 Renesas Electronics Corp.                                 */
/*                  And Renesas Solutions Corp. ,All Rights Reserved.           */
/*                                                                              */
/********************************************************************************/
/*                                                                              */
/*  DESCRIPTION : Definition of ICU Register                                    */
/*  CPU TYPE    : RX63T                                                         */
/*                                                                              */
/*  Usage : IR,DTCER,IER,IPR of ICU Register                                    */
/*     The following IR, DTCE, IEN, IPR macro functions simplify usage.         */
/*     The bit access operation is "Bit_Name(interrupt source,name)".           */
/*     A part of the name can be omitted.                                       */
/*     for example :                                                            */
/*       IR(MTU0,TGIA0) = 0;     expands to :                                   */
/*         ICU.IR[126].BIT.IR = 0;                                              */
/*                                                                              */
/*       DTCE(ICU,IRQ0) = 1;     expands to :                                   */
/*         ICU.DTCER[64].BIT.DTCE = 1;                                          */
/*                                                                              */
/*       IEN(CMT0,CMI0) = 1;     expands to :                                   */
/*         ICU.IER[0x03].BIT.IEN4 = 1;                                          */
/*                                                                              */
/*       IPR(SCI12,SCIX0) = 2;   expands to :                                   */
/*       IPR(SCI12,SCI  ) = 2;    // SCIX0,SCIX1,SCIX2,SCIX3 share IPR level.    */
/*         ICU.IPR[122].BIT.IPR = 2;                                            */
/*                                                                              */
/*       IPR(SCI0,RXI0) = 3;     expands to :                                   */
/*       IPR(SCI0,    ) = 3;     // SCI0 uses single IPR for all sources.       */
/*         ICU.IPR[214].BIT.IPR = 3;                                            */
/*                                                                              */
/*  Usage : #pragma interrupt Function_Identifier(vect=**)                      */
/*     The number of vector is "(interrupt source, name)".                      */
/*     for example :                                                            */
/*       #pragma interrupt INT_IRQ0(vect=VECT(ICU,IRQ0))          expands to :  */
/*         #pragma interrupt INT_IRQ0(vect=64)                                  */
/*       #pragma interrupt INT_CMT0_CMI0(vect=VECT(CMT0,CMI0))    expands to :  */
/*         #pragma interrupt INT_CMT0_CMI0(vect=28)                             */
/*       #pragma interrupt INT_MTU0_TGIA0(vect=VECT(MTU0,TGIA0))  expands to :  */
/*         #pragma interrupt INT_MTU0_TGIA0(vect=126)                           */
/*                                                                              */
/*  Usage : MSTPCRA,MSTPCRB,MSTPCRC of SYSTEM Register                          */
/*     The bit access operation is "MSTP(name)".                                */
/*     The name that can be used is a macro name defined with "iodefine.h".     */
/*     for example :                                                            */
/*       MSTP(SCI0) = 0;    // SCI0,SMCI0                         expands to :  */
/*         SYSTEM.MSTPCRB.BIT.MSTPB31 = 0;                                      */
/*       MSTP(MTU4) = 0;    // MTU7,MTU6,MTU5,MTU4,MTU3,MTU2,...  expands to :  */
/*         SYSTEM.MSTPCRA.BIT.MSTPA9  = 0;                                      */
/*       MSTP(CMT3) = 0;    // CMT2,CMT3                          expands to :  */
/*         SYSTEM.MSTPCRA.BIT.MSTPA14 = 0;                                      */
/*                                                                              */
/*                                                                              */
/********************************************************************************/
#ifndef __RX63TIODEFINE_HEADER__
#define __RX63TIODEFINE_HEADER__
#pragma bit_order left
#pragma unpack
struct st_bsc {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char STSCLR:1;
		} BIT;
	} BERCLR;
	char           wk0[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TOEN:1;
			unsigned char IGAEN:1;
		} BIT;
	} BEREN;
	char           wk1[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char MST:3;
			unsigned char :2;
			unsigned char TO:1;
			unsigned char IA:1;
		} BIT;
	} BERSR1;
	char           wk2[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short ADDR:13;
		} BIT;
	} BERSR2;
	char           wk3[4];
	union {
		unsigned short WORD;
		struct {
			unsigned short :4;
			unsigned short BPFB:2;
			unsigned short BPHB:2;
			unsigned short BPGB:2;
			unsigned short BPIB:2;
			unsigned short BPRO:2;
			unsigned short BPRA:2;
		} BIT;
	} BUSPRI;
};

struct st_cac {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char CFME:1;
		} BIT;
	} CACR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char EDGES:2;
			unsigned char TCSS:2;
			unsigned char FMCS:3;
			unsigned char CACREFE:1;
		} BIT;
	} CACR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char DFS:2;
			unsigned char RCDS:2;
			unsigned char RSCS:3;
			unsigned char RPS:1;
		} BIT;
	} CACR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char OVFFCL:1;
			unsigned char MENDFCL:1;
			unsigned char FERRFCL:1;
			unsigned char :1;
			unsigned char OVFIE:1;
			unsigned char MENDIE:1;
			unsigned char FERRIE:1;
		} BIT;
	} CAICR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char OVFF:1;
			unsigned char MENDF:1;
			unsigned char FERRF:1;
		} BIT;
	} CASTR;
	char           wk0[1];
	unsigned short CAULVR;
	unsigned short CALLVR;
	unsigned short CACNTBR;
};

struct st_cmt {
	union {
		unsigned short WORD;
		struct {
			unsigned short :14;
			unsigned short STR1:1;
			unsigned short STR0:1;
		} BIT;
	} CMSTR0;
	char           wk0[14];
	union {
		unsigned short WORD;
		struct {
			unsigned short :14;
			unsigned short STR3:1;
			unsigned short STR2:1;
		} BIT;
	} CMSTR1;
};

struct st_cmt0 {
	union {
		unsigned short WORD;
		struct {
			unsigned short :9;
			unsigned short CMIE:1;
			unsigned short :4;
			unsigned short CKS:2;
		} BIT;
	} CMCR;
	unsigned short CMCNT;
	unsigned short CMCOR;
};

struct st_crc {
	union {
		unsigned char BYTE;
		struct {
			unsigned char DORCLR:1;
			unsigned char :4;
			unsigned char LMS:1;
			unsigned char GPS:2;
		} BIT;
	} CRCCR;
	unsigned char  CRCDIR;
	unsigned short CRCDOR;
};

struct st_dmac {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DMST:1;
		} BIT;
	} DMAST;
};

struct st_dmac0 {
	void          *DMSAR;
	void          *DMDAR;
	unsigned long  DMCRA;
	unsigned short DMCRB;
	char           wk0[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short MD:2;
			unsigned short DTS:2;
			unsigned short :2;
			unsigned short SZ:2;
			unsigned short :6;
			unsigned short DCTG:2;
		} BIT;
	} DMTMD;
	char           wk1[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char DTIE:1;
			unsigned char ESIE:1;
			unsigned char RPTIE:1;
			unsigned char SARIE:1;
			unsigned char DARIE:1;
		} BIT;
	} DMINT;
	union {
		unsigned short WORD;
		struct {
			unsigned short SM:2;
			unsigned short :1;
			unsigned short SARA:5;
			unsigned short DM:2;
			unsigned short :1;
			unsigned short DARA:5;
		} BIT;
	} DMAMD;
	char           wk2[2];
	unsigned long  DMOFR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DTE:1;
		} BIT;
	} DMCNT;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char CLRS:1;
			unsigned char :3;
			unsigned char SWREQ:1;
		} BIT;
	} DMREQ;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ACT:1;
			unsigned char :2;
			unsigned char DTIF:1;
			unsigned char :3;
			unsigned char ESIF:1;
		} BIT;
	} DMSTS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DISEL:1;
		} BIT;
	} DMCSL;
};

struct st_dmac1 {
	void          *DMSAR;
	void          *DMDAR;
	unsigned long  DMCRA;
	unsigned short DMCRB;
	char           wk0[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short MD:2;
			unsigned short DTS:2;
			unsigned short :2;
			unsigned short SZ:2;
			unsigned short :6;
			unsigned short DCTG:2;
		} BIT;
	} DMTMD;
	char           wk1[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char DTIE:1;
			unsigned char ESIE:1;
			unsigned char RPTIE:1;
			unsigned char SARIE:1;
			unsigned char DARIE:1;
		} BIT;
	} DMINT;
	union {
		unsigned short WORD;
		struct {
			unsigned short SM:2;
			unsigned short :1;
			unsigned short SARA:5;
			unsigned short DM:2;
			unsigned short :1;
			unsigned short DARA:5;
		} BIT;
	} DMAMD;
	char           wk2[6];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DTE:1;
		} BIT;
	} DMCNT;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char CLRS:1;
			unsigned char :3;
			unsigned char SWREQ:1;
		} BIT;
	} DMREQ;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ACT:1;
			unsigned char :2;
			unsigned char DTIF:1;
			unsigned char :3;
			unsigned char ESIF:1;
		} BIT;
	} DMSTS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DISEL:1;
		} BIT;
	} DMCSL;
};

struct st_doc {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char DOPCFCL:1;
			unsigned char DOPCF:1;
			unsigned char DOPCIE:1;
			unsigned char :1;
			unsigned char DCSEL:1;
			unsigned char OMS:2;
		} BIT;
	} DOCR;
	char           wk0[1];
	unsigned short DODIR;
	unsigned short DODSR;
};

struct st_dtc {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char RRS:1;
		} BIT;
	} DTCCR;
	char           wk0[3];
	void          *DTCVBR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char SHORT:1;
		} BIT;
	} DTCADMOD;
	char           wk1[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DTCST:1;
		} BIT;
	} DTCST;
	char           wk2[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short ACT:1;
			unsigned short :7;
			unsigned short VECN:8;
		} BIT;
	} DTCSTS;
};

struct st_flash {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char FLWE:2;
		} BIT;
	} FWEPROR;
	char           wk0[7799147];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char FRDMD:1;
		} BIT;
	} FMODR;
	char           wk1[13];
	union {
		unsigned char BYTE;
		struct {
			unsigned char ROMAE:1;
			unsigned char :2;
			unsigned char CMDLK:1;
			unsigned char DFLAE:1;
			unsigned char :1;
			unsigned char DFLRPE:1;
			unsigned char DFLWPE:1;
		} BIT;
	} FASTAT;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ROMAEIE:1;
			unsigned char :2;
			unsigned char CMDLKIE:1;
			unsigned char DFLAEIE:1;
			unsigned char :1;
			unsigned char DFLRPEIE:1;
			unsigned char DFLWPEIE:1;
		} BIT;
	} FAEINT;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char FRDYIE:1;
		} BIT;
	} FRDYIE;
	char           wk2[45];
	union {
		unsigned short WORD;
		struct {
			unsigned short KEY:8;
			unsigned short :4;
			unsigned short DBRE03:1;
			unsigned short DBRE02:1;
			unsigned short DBRE01:1;
			unsigned short DBRE00:1;
		} BIT;
	} DFLRE0;
	char           wk3[14];
	union {
		unsigned short WORD;
		struct {
			unsigned short KEY:8;
			unsigned short DBWE07:1;
			unsigned short DBW006:1;
			unsigned short DBWE05:1;
			unsigned short DBWE04:1;
			unsigned short DBWE03:1;
			unsigned short DBWE02:1;
			unsigned short DBWE01:1;
			unsigned short DBWE00:1;
		} BIT;
	} DFLWE0;
	char           wk4[15198];
	union {
		unsigned char BYTE;
		struct {
			unsigned char FRDY:1;
			unsigned char ILGLERR:1;
			unsigned char ERSERR:1;
			unsigned char PRGERR:1;
			unsigned char SUSRDY:1;
			unsigned char :1;
			unsigned char ERSSPD:1;
			unsigned char PRGSPD:1;
		} BIT;
	} FSTATR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char FCUERR:1;
			unsigned char :2;
			unsigned char FLOCKST:1;
		} BIT;
	} FSTATR1;
	union {
		unsigned short WORD;
		struct {
			unsigned short FEKEY:8;
			unsigned short FENTRYD:1;
			unsigned short :6;
			unsigned short FENTRY0:1;
		} BIT;
	} FENTRYR;
	union {
		unsigned short WORD;
		struct {
			unsigned short FPKEY:8;
			unsigned short :7;
			unsigned short FPROTCN:1;
		} BIT;
	} FPROTR;
	union {
		unsigned short WORD;
		struct {
			unsigned short FRKEY:8;
			unsigned short :7;
			unsigned short FRESET:1;
		} BIT;
	} FRESETR;
	char           wk5[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short CMDR:8;
			unsigned short PCMDR:8;
		} BIT;
	} FCMDR;
	char           wk6[12];
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short ESUSPMD:1;
		} BIT;
	} FCPSR;
	union {
		unsigned short WORD;
		struct {
			unsigned short BCSIZE:1;
			unsigned short :4;
			unsigned short BCADR:11;
		} BIT;
	} DFLBCCNT;
	union {
		unsigned short WORD;
		struct {
			unsigned short :8;
			unsigned short PEERRST:8;
		} BIT;
	} FPESTAT;
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short BCST:1;
		} BIT;
	} DFLBCSTAT;
	char           wk7[24];
	union {
		unsigned short WORD;
		struct {
			unsigned short :8;
			unsigned short PCKA:8;
		} BIT;
	} PCKAR;
};

struct st_gpt {
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :8;
			unsigned char :4;
			unsigned char CST3:1;
			unsigned char CST2:1;
			unsigned char CST1:1;
			unsigned char CST0:1;
		} BIT;
	} GTSTR;
	char           wk0[2];
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char CPHW3:2;
			unsigned char CPHW2:2;
			unsigned char CPHW1:2;
			unsigned char CPHW0:2;
			unsigned char CSHW3:2;
			unsigned char CSHW2:2;
			unsigned char CSHW1:2;
			unsigned char CSHW0:2;
		} BIT;
	} GTHSCR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :4;
			unsigned char CCSW3:1;
			unsigned char CCSW2:1;
			unsigned char CCSW1:1;
			unsigned char CCSW0:1;
			unsigned char CCHW3:2;
			unsigned char CCHW2:2;
			unsigned char CCHW1:2;
			unsigned char CCHW0:2;
		} BIT;
	} GTHCCR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char CSHSL3:4;
			unsigned char CSHSL2:4;
			unsigned char CSHSL1:4;
			unsigned char CSHSL0:4;
		} BIT;
	} GTHSSR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char CSHPL3:4;
			unsigned char CSHPL2:4;
			unsigned char CSHPL1:4;
			unsigned char CSHPL0:4;
		} BIT;
	} GTHPSR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :8;
			unsigned char :4;
			unsigned char WP3:1;
			unsigned char WP2:1;
			unsigned char WP1:1;
			unsigned char WP0:1;
		} BIT;
	} GTWP;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :2;
			unsigned char SYNC3:2;
			unsigned char :2;
			unsigned char SYNC2:2;
			unsigned char :2;
			unsigned char SYNC1:2;
			unsigned char :2;
			unsigned char SYNC0:2;
		} BIT;
	} GTSYNC;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :6;
			unsigned char ETINF:1;
			unsigned char ETIPF:1;
			unsigned char :6;
			unsigned char ETINEN:1;
			unsigned char ETIPEN:1;
		} BIT;
	} GTETINT;
	char           wk1[2];
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char BD33:1;
			unsigned char BD32:1;
			unsigned char BD31:1;
			unsigned char BD30:1;
			unsigned char BD23:1;
			unsigned char BD22:1;
			unsigned char BD21:1;
			unsigned char BD20:1;
			unsigned char BD13:1;
			unsigned char BD12:1;
			unsigned char BD11:1;
			unsigned char BD10:1;
			unsigned char BD03:1;
			unsigned char BD02:1;
			unsigned char BD01:1;
			unsigned char BD00:1;
		} BIT;
	} GTBDR;
	char           wk2[2];
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned short :12;
			unsigned short SWP3:1;
			unsigned short SWP2:1;
			unsigned short SWP1:1;
			unsigned short SWP0:1;
		} BIT;
	} GTSWP;
	char           wk3[102];
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char LPSC:2;
			unsigned char TPSC:2;
			unsigned char LCNTAT:1;
			unsigned char LCTO:3;
			unsigned char :1;
			unsigned char LCINTO:1;
			unsigned char LCINTD:1;
			unsigned char LCINTC:1;
			unsigned char :1;
			unsigned char LCNTS:1;
			unsigned char LCNTCR:1;
			unsigned char LCNTE:1;
		} BIT;
	} LCCR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :8;
			unsigned char :5;
			unsigned char LISO:1;
			unsigned char LISD:1;
			unsigned char LISC:1;
		} BIT;
	} LCST;
	unsigned short LCNT;
	unsigned short LCNTA;
	unsigned short LCNT00;
	unsigned short LCNT01;
	unsigned short LCNT02;
	unsigned short LCNT03;
	unsigned short LCNT04;
	unsigned short LCNT05;
	unsigned short LCNT06;
	unsigned short LCNT07;
	unsigned short LCNT08;
	unsigned short LCNT09;
	unsigned short LCNT10;
	unsigned short LCNT11;
	unsigned short LCNT12;
	unsigned short LCNT13;
	unsigned short LCNT14;
	unsigned short LCNT15;
	unsigned short LCNTDU;
	unsigned short LCNTDL;
};

struct st_gpt0 {
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char OBHLD:1;
			unsigned char OBDFLT:1;
			unsigned char GTIOB:6;
			unsigned char OAHLD:1;
			unsigned char OADFLT:1;
			unsigned char GTIOA:6;
		} BIT;
	} GTIOR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char ADTRBDEN:1;
			unsigned char ADTRBUEN:1;
			unsigned char ADTRADEN:1;
			unsigned char ADTRAUEN:1;
			unsigned char EINT:1;
			unsigned char :3;
			unsigned char GTINTPR:2;
			unsigned char GTINTF:1;
			unsigned char GTINTE:1;
			unsigned char GTINTD:1;
			unsigned char GTINTC:1;
			unsigned char GTINTB:1;
			unsigned char GTINTA:1;
		} BIT;
	} GTINTAD;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :2;
			unsigned char CCLR:2;
			unsigned char :2;
			unsigned char TPCS:2;
			unsigned char :5;
			unsigned char MD:3;
		} BIT;
	} GTCR;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :1;
			unsigned char ADTDB:1;
			unsigned char ADTTB:2;
			unsigned char :1;
			unsigned char ADTDA:1;
			unsigned char ADTTA:2;
			unsigned char :1;
			unsigned char CCRSWT:1;
			unsigned char PR:2;
			unsigned char CCRB:2;
			unsigned char CCRA:2;
		} BIT;
	} GTBER;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :8;
			unsigned char :6;
			unsigned char UDF:1;
			unsigned char UD:1;
		} BIT;
	} GTUDC;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :1;
			unsigned char ADTBL:1;
			unsigned char :1;
			unsigned char ADTAL:1;
			unsigned char :1;
			unsigned char IVTT:3;
			unsigned char IVTC:2;
			unsigned char ITLF:1;
			unsigned char ITLE:1;
			unsigned char ITLD:1;
			unsigned char ITLC:1;
			unsigned char ITLB:1;
			unsigned char ITLA:1;
		} BIT;
	} GTITC;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char TUCF:1;
			unsigned char :3;
			unsigned char DTEF:1;
			unsigned char ITCNT:3;
			unsigned char TCFPU:1;
			unsigned char TCFPO:1;
			unsigned char TCFF:1;
			unsigned char TCFE:1;
			unsigned char TCFD:1;
			unsigned char TCFC:1;
			unsigned char TCFB:1;
			unsigned char TCFA:1;
		} BIT;
	} GTST;
	unsigned short GTCNT;
	unsigned short GTCCRA;
	unsigned short GTCCRB;
	unsigned short GTCCRC;
	unsigned short GTCCRD;
	unsigned short GTCCRE;
	unsigned short GTCCRF;
	unsigned short GTPR;
	unsigned short GTPBR;
	unsigned short GTPDBR;
	char           wk0[2];
	unsigned short GTADTRA;
	unsigned short GTADTBRA;
	unsigned short GTADTDBRA;
	char           wk1[2];
	unsigned short GTADTRB;
	unsigned short GTADTBRB;
	unsigned short GTADTDBRB;
	char           wk2[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short OBE:1;
			unsigned short OAE:1;
			unsigned short :1;
			unsigned short SWN:1;
			unsigned short :3;
			unsigned short NFV:1;
			unsigned short NFS:4;
			unsigned short NVB:1;
			unsigned short NVA:1;
			unsigned short NEB:1;
			unsigned short NEA:1;
		} BIT;
	} GTONCR;
	union {
		unsigned short WORD;
		struct {
			unsigned short :7;
			unsigned short TDFER:1;
			unsigned short :2;
			unsigned short TDBDE:1;
			unsigned short TDBUE:1;
			unsigned short :3;
			unsigned short TDE:1;
		} BIT;
	} GTDTCR;
	unsigned short GTDVU;
	unsigned short GTDVD;
	unsigned short GTDBU;
	unsigned short GTDBD;
	union {
		unsigned short WORD;
		struct {
			unsigned short :14;
			unsigned short SOS:2;
		} BIT;
	} GTSOS;
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short SOTR:1;
		} BIT;
	} GTSOTR;
};

struct st_icu {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char IR:1;
		} BIT;
	} IR[253];
	char           wk0[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DTCE:1;
		} BIT;
	} DTCER[252];
	char           wk1[4];
	union {
		unsigned char BYTE;
		struct {
			unsigned char IEN7:1;
			unsigned char IEN6:1;
			unsigned char IEN5:1;
			unsigned char IEN4:1;
			unsigned char IEN3:1;
			unsigned char IEN2:1;
			unsigned char IEN1:1;
			unsigned char IEN0:1;
		} BIT;
	} IER[32];
	char           wk2[192];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char SWINT:1;
		} BIT;
	} SWINTR;
	char           wk3[15];
	union {
		unsigned short WORD;
		struct {
			unsigned short FIEN:1;
			unsigned short :7;
			unsigned short FVCT:8;
		} BIT;
	} FIR;
	char           wk4[14];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char IPR:4;
		} BIT;
	} IPR[251];
	char           wk5[5];
	unsigned char  DMRSR0;
	char           wk6[3];
	unsigned char  DMRSR1;
	char           wk7[3];
	unsigned char  DMRSR2;
	char           wk8[3];
	unsigned char  DMRSR3;
	char           wk9[243];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char IRQMD:2;
		} BIT;
	} IRQCR[6];
	char           wk10[10];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char FLTEN5:1;
			unsigned char FLTEN4:1;
			unsigned char FLTEN3:1;
			unsigned char FLTEN2:1;
			unsigned char FLTEN1:1;
			unsigned char FLTEN0:1;
		} BIT;
	} IRQFLTE0;
	char           wk11[3];
	union {
		unsigned short WORD;
		struct {
			unsigned short :4;
			unsigned short FCLKSEL5:2;
			unsigned short FCLKSEL4:2;
			unsigned short FCLKSEL3:2;
			unsigned short FCLKSEL2:2;
			unsigned short FCLKSEL1:2;
			unsigned short FCLKSEL0:2;
		} BIT;
	} IRQFLTC0;
	char           wk12[106];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char LVD2ST:1;
			unsigned char LVD1ST:1;
			unsigned char IWDTST:1;
			unsigned char WDTST:1;
			unsigned char OSTST:1;
			unsigned char NMIST:1;
		} BIT;
	} NMISR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char LVD2EN:1;
			unsigned char LVD1EN:1;
			unsigned char IWDTEN:1;
			unsigned char WDTEN:1;
			unsigned char OSTEN:1;
			unsigned char NMIEN:1;
		} BIT;
	} NMIER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char LVD2CLR:1;
			unsigned char LVD1CLR:1;
			unsigned char IWDTCLR:1;
			unsigned char WDTCLR:1;
			unsigned char OSTCLR:1;
			unsigned char NMICLR:1;
		} BIT;
	} NMICLR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char NMIMD:1;
		} BIT;
	} NMICR;
	char           wk13[12];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char NFLTEN:1;
		} BIT;
	} NMIFLTE;
	char           wk14[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char NFCLKSEL:2;
		} BIT;
	} NMIFLTC;
	char           wk15[19819];
	union {
		unsigned long LONG;
		struct {
			unsigned long :26;
			unsigned long IS5:1;
			unsigned long IS4:1;
			unsigned long :2;
			unsigned long IS1:1;
			unsigned long IS0:1;
		} BIT;
	} GRP[13];
	char           wk16[12];
	union {
		unsigned long LONG;
		struct {
			unsigned long :26;
			unsigned long EN5:1;
			unsigned long EN4:1;
			unsigned long :2;
			unsigned long EN1:1;
			unsigned long EN0:1;
		} BIT;
	} GEN[13];
};

struct st_iwdt {
	unsigned char  IWDTRR;
	char           wk0[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short :2;
			unsigned short RPSS:2;
			unsigned short :2;
			unsigned short RPES:2;
			unsigned short CKS:4;
			unsigned short :2;
			unsigned short TOPS:2;
		} BIT;
	} IWDTCR;
	union {
		unsigned short WORD;
		struct {
			unsigned short REFEF:1;
			unsigned short UNDFF:1;
			unsigned short CNTVAL:14;
		} BIT;
	} IWDTSR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char RSTIRQS:1;
		} BIT;
	} IWDTRCR;
	char           wk1[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char SLCSTP:1;
		} BIT;
	} IWDTCSTPR;
};

struct st_mpc {
	union {
		unsigned char BYTE;
		struct {
			unsigned char B0WI:1;
			unsigned char PFSWE:1;
		} BIT;
	} PWPR;
	char           wk0[32];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} P00PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} P01PFS;
	char           wk1[6];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} P10PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} P11PFS;
	char           wk2[8];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P22PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P23PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P24PFS;
	char           wk3[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P30PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P31PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P32PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P33PFS;
	char           wk4[4];
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P40PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P41PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P42PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P43PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P44PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P45PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P46PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char ASEL:1;
		} BIT;
	} P47PFS;
	char           wk5[16];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} P70PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P71PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P72PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P73PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P74PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P75PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P76PFS;
	char           wk6[10];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P91PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P92PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} P93PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} P94PFS;
	char           wk7[5];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PA2PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PA3PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PA4PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PA5PFS;
	char           wk8[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PB0PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PB1PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PB2PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PB3PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} PB4PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char ISEL:1;
			unsigned char :2;
			unsigned char PSEL:4;
		} BIT;
	} PB5PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PB6PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PB7PFS;
	char           wk9[11];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PD3PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PD4PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PD5PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PD6PFS;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PD7PFS;
	char           wk10[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PSEL:4;
		} BIT;
	} PE2PFS;
};

struct st_mpu {
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE0;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE0;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE1;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE1;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE2;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE2;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE3;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE3;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE4;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE4;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE5;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE5;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE6;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE6;
	union {
		unsigned long LONG;
		struct {
			unsigned long RSPN:28;
		} BIT;
	} RSPAGE7;
	union {
		unsigned long LONG;
		struct {
			unsigned long REPN:28;
			unsigned long UAC:3;
			unsigned long V:1;
		} BIT;
	} REPAGE7;
	char           wk0[192];
	union {
		unsigned long LONG;
		struct {
			unsigned long :31;
			unsigned long MPEN:1;
		} BIT;
	} MPEN;
	union {
		unsigned long LONG;
		struct {
			unsigned long :28;
			unsigned long UBAC:3;
		} BIT;
	} MPBAC;
	union {
		unsigned long LONG;
		struct {
			unsigned long :31;
			unsigned long CLR:1;
		} BIT;
	} MPECLR;
	union {
		unsigned long LONG;
		struct {
			unsigned long :29;
			unsigned long DRW:1;
			unsigned long DA:1;
			unsigned long IA:1;
		} BIT;
	} MPESTS;
	char           wk1[4];
	union {
		unsigned long LONG;
		struct {
			unsigned long DEA:32;
		} BIT;
	} MPDEA;
	char           wk2[8];
	union {
		unsigned long LONG;
		struct {
			unsigned long SA:32;
		} BIT;
	} MPSA;
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short S:1;
		} BIT;
	} MPOPS;
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short INV:1;
		} BIT;
	} MPOPI;
	union {
		unsigned long LONG;
		struct {
			unsigned long :8;
			unsigned long HITI:8;
			unsigned long :12;
			unsigned long UHACI:3;
		} BIT;
	} MHITI;
	union {
		unsigned long LONG;
		struct {
			unsigned long :8;
			unsigned long HITD:8;
			unsigned long :12;
			unsigned long UHACD:3;
		} BIT;
	} MHITD;
};

struct st_mtu {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char OE4D:1;
			unsigned char OE4C:1;
			unsigned char OE3D:1;
			unsigned char OE4B:1;
			unsigned char OE4A:1;
			unsigned char OE3B:1;
		} BIT;
	} TOERA;
	char           wk0[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char BDC:1;
			unsigned char N:1;
			unsigned char P:1;
			unsigned char FB:1;
			unsigned char WF:1;
			unsigned char VF:1;
			unsigned char UF:1;
		} BIT;
	} TGCRA;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char PSYE:1;
			unsigned char :2;
			unsigned char TOCL:1;
			unsigned char TOCS:1;
			unsigned char OLSN:1;
			unsigned char OLSP:1;
		} BIT;
	} TOCR1A;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BF:2;
			unsigned char OLS3N:1;
			unsigned char OLS3P:1;
			unsigned char OLS2N:1;
			unsigned char OLS2P:1;
			unsigned char OLS1N:1;
			unsigned char OLS1P:1;
		} BIT;
	} TOCR2A;
	char           wk1[4];
	unsigned short TCDRA;
	unsigned short TDDRA;
	char           wk2[8];
	unsigned short TCNTSA;
	unsigned short TCBRA;
	char           wk3[12];
	union {
		unsigned char BYTE;
		struct {
			unsigned char T3AEN:1;
			unsigned char T3ACOR:3;
			unsigned char T4VEN:1;
			unsigned char T4VCOR:3;
		} BIT;
	} TITCR1A;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char T3ACNT:3;
			unsigned char :1;
			unsigned char T4VCNT:3;
		} BIT;
	} TITCNT1A;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char BTE:2;
		} BIT;
	} TBTERA;
	char           wk4[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char TDER:1;
		} BIT;
	} TDERA;
	char           wk5[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char OLS3N:1;
			unsigned char OLS3P:1;
			unsigned char OLS2N:1;
			unsigned char OLS2P:1;
			unsigned char OLS1N:1;
			unsigned char OLS1P:1;
		} BIT;
	} TOLBRA;
	char           wk6[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char TITM:1;
		} BIT;
	} TITMRA;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char TRG4COR:3;
		} BIT;
	} TITCR2A;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char TRG4CNT:3;
		} BIT;
	} TITCNT2A;
	char           wk7[35];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CCE:1;
			unsigned char :6;
			unsigned char WRE:1;
		} BIT;
	} TWCRA;
	char           wk8[15];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DRS:1;
		} BIT;
	} TMDR2A;
	char           wk9[15];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CST4:1;
			unsigned char CST3:1;
			unsigned char :3;
			unsigned char CST2:1;
			unsigned char CST1:1;
			unsigned char CST0:1;
		} BIT;
	} TSTRA;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SYNC4:1;
			unsigned char SYNC3:1;
			unsigned char :3;
			unsigned char SYNC2:1;
			unsigned char SYNC1:1;
			unsigned char SYNC0:1;
		} BIT;
	} TSYRA;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SCH0:1;
			unsigned char SCH1:1;
			unsigned char SCH2:1;
			unsigned char SCH3:1;
			unsigned char SCH4:1;
			unsigned char :1;
			unsigned char SCH6:1;
			unsigned char SCH7:1;
		} BIT;
	} TCSYSTR;
	char           wk10[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char RWE:1;
		} BIT;
	} TRWERA;
	char           wk11[1925];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char OE7D:1;
			unsigned char OE7C:1;
			unsigned char OE6D:1;
			unsigned char OE7B:1;
			unsigned char OE7A:1;
			unsigned char OE6B:1;
		} BIT;
	} TOERB;
	char           wk12[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char PSYE:1;
			unsigned char :2;
			unsigned char TOCL:1;
			unsigned char TOCS:1;
			unsigned char OLSN:1;
			unsigned char OLSP:1;
		} BIT;
	} TOCR1B;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BF:2;
			unsigned char OLS3N:1;
			unsigned char OLS3P:1;
			unsigned char OLS2N:1;
			unsigned char OLS2P:1;
			unsigned char OLS1N:1;
			unsigned char OLS1P:1;
		} BIT;
	} TOCR2B;
	char           wk13[4];
	unsigned short TCDRB;
	unsigned short TDDRB;
	char           wk14[8];
	unsigned short TCNTSB;
	unsigned short TCBRB;
	char           wk15[12];
	union {
		unsigned char BYTE;
		struct {
			unsigned char T6AEN:1;
			unsigned char T6ACOR:3;
			unsigned char T7VEN:1;
			unsigned char T7VCOR:3;
		} BIT;
	} TITCR1B;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char T6ACNT:3;
			unsigned char :1;
			unsigned char T7VCNT:3;
		} BIT;
	} TITCNT1B;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char BTE:2;
		} BIT;
	} TBTERB;
	char           wk16[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char TDER:1;
		} BIT;
	} TDERB;
	char           wk17[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char OLS3N:1;
			unsigned char OLS3P:1;
			unsigned char OLS2N:1;
			unsigned char OLS2P:1;
			unsigned char OLS1N:1;
			unsigned char OLS1P:1;
		} BIT;
	} TOLBRB;
	char           wk18[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char TITM:1;
		} BIT;
	} TITMRB;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char TRG7COR:3;
		} BIT;
	} TITCR2B;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char TRG7CNT:3;
		} BIT;
	} TITCNT2B;
	char           wk19[19];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CE0A:1;
			unsigned char CE0B:1;
			unsigned char CE0C:1;
			unsigned char CE0D:1;
			unsigned char CE1A:1;
			unsigned char CE1B:1;
			unsigned char CE2A:1;
			unsigned char CE2B:1;
		} BIT;
	} TSYCR;
	char           wk20[15];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CCE:1;
			unsigned char :5;
			unsigned char SCC:1;
			unsigned char WRE:1;
		} BIT;
	} TWCRB;
	char           wk21[15];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char DRS:1;
		} BIT;
	} TMDR2B;
	char           wk22[15];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CST7:1;
			unsigned char CST6:1;
		} BIT;
	} TSTRB;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SYNC7:1;
			unsigned char SYNC6:1;
		} BIT;
	} TSYRB;
	char           wk23[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char RWE:1;
		} BIT;
	} TRWERB;
};

struct st_mtu0 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char CCLR:3;
			unsigned char CKEG:2;
			unsigned char TPSC:3;
		} BIT;
	} TCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char BFE:1;
			unsigned char BFB:1;
			unsigned char BFA:1;
			unsigned char MD:4;
		} BIT;
	} TMDR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOB:4;
			unsigned char IOA:4;
		} BIT;
	} TIORH;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOD:4;
			unsigned char IOC:4;
		} BIT;
	} TIORL;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE:1;
			unsigned char :2;
			unsigned char TCIEV:1;
			unsigned char TGIED:1;
			unsigned char TGIEC:1;
			unsigned char TGIEB:1;
			unsigned char TGIEA:1;
		} BIT;
	} TIER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char TCFV:1;
			unsigned char TGFD:1;
			unsigned char TGFC:1;
			unsigned char TGFB:1;
			unsigned char TGFA:1;
		} BIT;
	} TSR;
	unsigned short TCNT;
	unsigned short TGRA;
	unsigned short TGRB;
	unsigned short TGRC;
	unsigned short TGRD;
	char           wk0[16];
	unsigned short TGRE;
	unsigned short TGRF;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE2:1;
			unsigned char :5;
			unsigned char TGIEF:1;
			unsigned char TGIEE:1;
		} BIT;
	} TIER2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TGFF:1;
			unsigned char TGFE:1;
		} BIT;
	} TSR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char TTSE:1;
			unsigned char TTSB:1;
			unsigned char TTSA:1;
		} BIT;
	} TBTM;
};

struct st_mtu1 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char CCLR:2;
			unsigned char CKEG:2;
			unsigned char TPSC:3;
		} BIT;
	} TCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char MD:4;
		} BIT;
	} TMDR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOB:4;
			unsigned char IOA:4;
		} BIT;
	} TIOR;
	char           wk0[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE:1;
			unsigned char :1;
			unsigned char TCIEU:1;
			unsigned char TCIEV:1;
			unsigned char :2;
			unsigned char TGIEB:1;
			unsigned char TGIEA:1;
		} BIT;
	} TIER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TCFD:1;
			unsigned char :1;
			unsigned char TCFU:1;
			unsigned char TCFV:1;
			unsigned char :2;
			unsigned char TGFB:1;
			unsigned char TGFA:1;
		} BIT;
	} TSR;
	unsigned short TCNT;
	unsigned short TGRA;
	unsigned short TGRB;
	char           wk1[4];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char I2BE:1;
			unsigned char I2AE:1;
			unsigned char I1BE:1;
			unsigned char I1AE:1;
		} BIT;
	} TICCR;
};

struct st_mtu2 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char CCLR:2;
			unsigned char CKEG:2;
			unsigned char TPSC:3;
		} BIT;
	} TCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char MD:4;
		} BIT;
	} TMDR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOB:4;
			unsigned char IOA:4;
		} BIT;
	} TIOR;
	char           wk0[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE:1;
			unsigned char :1;
			unsigned char TCIEU:1;
			unsigned char TCIEV:1;
			unsigned char :2;
			unsigned char TGIEB:1;
			unsigned char TGIEA:1;
		} BIT;
	} TIER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TCFD:1;
			unsigned char :1;
			unsigned char TCFU:1;
			unsigned char TCFV:1;
			unsigned char :2;
			unsigned char TGFB:1;
			unsigned char TGFA:1;
		} BIT;
	} TSR;
	unsigned short TCNT;
	unsigned short TGRA;
	unsigned short TGRB;
};

struct st_mtu3 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char CCLR:3;
			unsigned char CKEG:2;
			unsigned char TPSC:3;
		} BIT;
	} TCR;
	char           wk0[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char BFB:1;
			unsigned char BFA:1;
			unsigned char MD:4;
		} BIT;
	} TMDR1;
	char           wk1[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOB:4;
			unsigned char IOA:4;
		} BIT;
	} TIORH;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOD:4;
			unsigned char IOC:4;
		} BIT;
	} TIORL;
	char           wk2[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE:1;
			unsigned char :2;
			unsigned char TCIEV:1;
			unsigned char TGIED:1;
			unsigned char TGIEC:1;
			unsigned char TGIEB:1;
			unsigned char TGIEA:1;
		} BIT;
	} TIER;
	char           wk3[7];
	unsigned short TCNT;
	char           wk4[6];
	unsigned short TGRA;
	unsigned short TGRB;
	char           wk5[8];
	unsigned short TGRC;
	unsigned short TGRD;
	char           wk6[4];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TCFD:1;
			unsigned char :2;
			unsigned char TCFV:1;
			unsigned char TGFD:1;
			unsigned char TGFC:1;
			unsigned char TGFB:1;
			unsigned char TGFA:1;
		} BIT;
	} TSR;
	char           wk7[11];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TTSB:1;
			unsigned char TTSA:1;
		} BIT;
	} TBTM;
	char           wk8[57];
	unsigned short TGRE;
};

struct st_mtu4 {
	char           wk0[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CCLR:3;
			unsigned char CKEG:2;
			unsigned char TPSC:3;
		} BIT;
	} TCR;
	char           wk1[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char BFB:1;
			unsigned char BFA:1;
			unsigned char MD:4;
		} BIT;
	} TMDR1;
	char           wk2[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOB:4;
			unsigned char IOA:4;
		} BIT;
	} TIORH;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOD:4;
			unsigned char IOC:4;
		} BIT;
	} TIORL;
	char           wk3[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE:1;
			unsigned char TTGE2:1;
			unsigned char :1;
			unsigned char TCIEV:1;
			unsigned char TGIED:1;
			unsigned char TGIEC:1;
			unsigned char TGIEB:1;
			unsigned char TGIEA:1;
		} BIT;
	} TIER;
	char           wk4[8];
	unsigned short TCNT;
	char           wk5[8];
	unsigned short TGRA;
	unsigned short TGRB;
	char           wk6[8];
	unsigned short TGRC;
	unsigned short TGRD;
	char           wk7[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TCFD:1;
			unsigned char :2;
			unsigned char TCFV:1;
			unsigned char TGFD:1;
			unsigned char TGFC:1;
			unsigned char TGFB:1;
			unsigned char TGFA:1;
		} BIT;
	} TSR;
	char           wk8[11];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TTSB:1;
			unsigned char TTSA:1;
		} BIT;
	} TBTM;
	char           wk9[6];
	union {
		unsigned short WORD;
		struct {
			unsigned short BF:2;
			unsigned short :6;
			unsigned short UT4AE:1;
			unsigned short DT4AE:1;
			unsigned short UT4BE:1;
			unsigned short DT4BE:1;
			unsigned short ITA3AE:1;
			unsigned short ITA4VE:1;
			unsigned short ITB3AE:1;
			unsigned short ITB4VE:1;
		} BIT;
	} TADCR;
	char           wk10[2];
	unsigned short TADCORA;
	unsigned short TADCORB;
	unsigned short TADCOBRA;
	unsigned short TADCOBRB;
	char           wk11[40];
	unsigned short TGRE;
	unsigned short TGRF;
};

struct st_mtu5 {
	unsigned short TCNTU;
	unsigned short TGRU;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TPSC:2;
		} BIT;
	} TCRU;
	char           wk0[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char IOC:5;
		} BIT;
	} TIORU;
	char           wk1[9];
	unsigned short TCNTV;
	unsigned short TGRV;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TPSC:2;
		} BIT;
	} TCRV;
	char           wk2[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char IOC:5;
		} BIT;
	} TIORV;
	char           wk3[9];
	unsigned short TCNTW;
	unsigned short TGRW;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TPSC:2;
		} BIT;
	} TCRW;
	char           wk4[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char IOC:5;
		} BIT;
	} TIORW;
	char           wk5[9];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char CMFU5:1;
			unsigned char CMFV5:1;
			unsigned char CMFW5:1;
		} BIT;
	} TSR;
	char           wk6[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char TGIE5U:1;
			unsigned char TGIE5V:1;
			unsigned char TGIE5W:1;
		} BIT;
	} TIER;
	char           wk7[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char CSTU5:1;
			unsigned char CSTV5:1;
			unsigned char CSTW5:1;
		} BIT;
	} TSTR;
	char           wk8[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char CMPCLR5U:1;
			unsigned char CMPCLR5V:1;
			unsigned char CMPCLR5W:1;
		} BIT;
	} TCNTCMPCLR;
};

struct st_mtu7 {
	char           wk0[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char CCLR:3;
			unsigned char CKEG:2;
			unsigned char TPSC:3;
		} BIT;
	} TCR;
	char           wk1[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char BFB:1;
			unsigned char BFA:1;
			unsigned char MD:4;
		} BIT;
	} TMDR1;
	char           wk2[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOB:4;
			unsigned char IOA:4;
		} BIT;
	} TIORH;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IOD:4;
			unsigned char IOC:4;
		} BIT;
	} TIORL;
	char           wk3[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TTGE:1;
			unsigned char TTGE2:1;
			unsigned char :1;
			unsigned char TCIEV:1;
			unsigned char TGIED:1;
			unsigned char TGIEC:1;
			unsigned char TGIEB:1;
			unsigned char TGIEA:1;
		} BIT;
	} TIER;
	char           wk4[8];
	unsigned short TCNT;
	char           wk5[8];
	unsigned short TGRA;
	unsigned short TGRB;
	char           wk6[8];
	unsigned short TGRC;
	unsigned short TGRD;
	char           wk7[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char TCFD:1;
			unsigned char :2;
			unsigned char TCFV:1;
			unsigned char TGFD:1;
			unsigned char TGFC:1;
			unsigned char TGFB:1;
			unsigned char TGFA:1;
		} BIT;
	} TSR;
	char           wk8[11];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char TTSB:1;
			unsigned char TTSA:1;
		} BIT;
	} TBTM;
	char           wk9[6];
	union {
		unsigned short WORD;
		struct {
			unsigned short BF:2;
			unsigned short :6;
			unsigned short UT7AE:1;
			unsigned short DT7AE:1;
			unsigned short UT7BE:1;
			unsigned short DT7BE:1;
			unsigned short ITA6AE:1;
			unsigned short ITA7VE:1;
			unsigned short ITB6AE:1;
			unsigned short ITB7VE:1;
		} BIT;
	} TADCR;
	char           wk10[2];
	unsigned short TADCORA;
	unsigned short TADCORB;
	unsigned short TADCOBRA;
	unsigned short TADCOBRB;
	char           wk11[40];
	unsigned short TGRE;
	unsigned short TGRF;
};

struct st_poe {
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :3;
			unsigned char POE0F:1;
			unsigned char :3;
			unsigned char PIE1:1;
			unsigned char :6;
			unsigned char POE0M:2;
		} BIT;
	} ICSR1;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char OSF1:1;
			unsigned char :5;
			unsigned char OCE1:1;
			unsigned char OIE1:1;
		} BIT;
	} OCSR1;
	char           wk0[4];
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :3;
			unsigned char POE8F:1;
			unsigned char :2;
			unsigned char POE8E:1;
			unsigned char PIE3:1;
			unsigned char :6;
			unsigned char POE8M:2;
		} BIT;
	} ICSR3;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char GPT23HIZ:1;
			unsigned char GPT01HIZ:1;
			unsigned char MTUCH0HIZ:1;
			unsigned char :1;
			unsigned char MTUCH34HIZ:1;
		} BIT;
	} SPOER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char MTU0DZE:1;
			unsigned char MTU0CZE:1;
			unsigned char MTU0BZE:1;
			unsigned char MTU0AZE:1;
		} BIT;
	} POECR1;
	union {
		unsigned short WORD;
		struct {
			unsigned short :5;
			unsigned short MTU3BDZE:1;
			unsigned short MTU4ACZE:1;
			unsigned short MTU4BDZE:1;
		} BIT;
	} POECR2;
	union {
		unsigned short WORD;
		struct {
			unsigned short :6;
			unsigned short GPT3ABZE:1;
			unsigned short GPT2ABZE:1;
			unsigned short :6;
			unsigned short GPT1ABZE:1;
			unsigned short GPT0ABZE:1;
		} BIT;
	} POECR3;
	union {
		unsigned short WORD;
		struct {
			unsigned short :10;
			unsigned short IC5ADDMT34ZE:1;
			unsigned short IC4ADDMT34ZE:1;
			unsigned short IC3ADDMT34ZE:1;
			unsigned short :2;
			unsigned short CMADDMT34ZE:1;
		} BIT;
	} POECR4;
	union {
		unsigned short WORD;
		struct {
			unsigned short :10;
			unsigned short IC5ADDMT0ZE:1;
			unsigned short IC4ADDMT0ZE:1;
			unsigned short :2;
			unsigned short IC1ADDMT0ZE:1;
			unsigned short CMADDMT0ZE:1;
		} BIT;
	} POECR5;
	union {
		unsigned short WORD;
		struct {
			unsigned short :3;
			unsigned short IC4ADDGPT23ZE:1;
			unsigned short IC3ADDGPT23ZE:1;
			unsigned short :1;
			unsigned short IC1ADDGPT23ZE:1;
			unsigned short CMADDGPT23ZE:1;
			unsigned short :2;
			unsigned short IC5ADDGPT01ZE:1;
			unsigned short :1;
			unsigned short IC3ADDGPT01ZE:1;
			unsigned short :1;
			unsigned short IC1ADDGPT01ZE:1;
			unsigned short CMADDGPT01ZE:1;
		} BIT;
	} POECR6;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :3;
			unsigned char POE10F:1;
			unsigned char :2;
			unsigned char POE10E:1;
			unsigned char PIE4:1;
			unsigned char :6;
			unsigned char POE10M:2;
		} BIT;
	} ICSR4;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :3;
			unsigned char POE11F:1;
			unsigned char :2;
			unsigned char POE11E:1;
			unsigned char PIE5:1;
			unsigned char :6;
			unsigned char POE11M:2;
		} BIT;
	} ICSR5;
	union {
		unsigned short WORD;
		struct {
			unsigned char H;
			unsigned char L;
		} BYTE;
		struct {
			unsigned char :7;
			unsigned char MTUCHSEL:1;
			unsigned char OLSEN:1;
			unsigned char :1;
			unsigned char OLSG2B:1;
			unsigned char OLSG2A:1;
			unsigned char OLSG1B:1;
			unsigned char OLSG1A:1;
			unsigned char OLSG0B:1;
			unsigned char OLSG0A:1;
		} BIT;
	} ALR1;
	union {
		unsigned short WORD;
		struct {
			unsigned short :3;
			unsigned short OSTSTF:1;
			unsigned short :2;
			unsigned short OSTSTE:1;
		} BIT;
	} ICSR6;
};

struct st_port0 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PMR;
};

struct st_port1 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PMR;
};

struct st_port2 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PMR;
	char           wk3[34];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char B0:1;
		} BIT;
	} ODR1;
};

struct st_port3 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PMR;
	char           wk3[34];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char B0:1;
		} BIT;
	} ODR0;
};

struct st_port4 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PIDR;
};

struct st_port7 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PMR;
};

struct st_port9 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
		} BIT;
	} PMR;
	char           wk3[40];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char B6:1;
		} BIT;
	} ODR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char B0:1;
		} BIT;
	} ODR1;
};

struct st_porta {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
		} BIT;
	} PMR;
};

struct st_portb {
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
			unsigned char B2:1;
			unsigned char B1:1;
			unsigned char B0:1;
		} BIT;
	} PMR;
	char           wk3[42];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char :1;
			unsigned char B2:1;
		} BIT;
	} ODR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char B4:1;
			unsigned char :1;
			unsigned char B2:1;
		} BIT;
	} ODR1;
};

struct st_portd {
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
		} BIT;
	} PDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
		} BIT;
	} PODR;
	char           wk1[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
		} BIT;
	} PIDR;
	char           wk2[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char B7:1;
			unsigned char B6:1;
			unsigned char B5:1;
			unsigned char B4:1;
			unsigned char B3:1;
		} BIT;
	} PMR;
	char           wk3[44];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char B6:1;
		} BIT;
	} ODR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char B2:1;
		} BIT;
	} ODR1;
};

struct st_porte {
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char B2:1;
		} BIT;
	} PIDR;
	char           wk0[31];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char B2:1;
		} BIT;
	} PMR;
};

struct st_riic {
	union {
		unsigned char BYTE;
		struct {
			unsigned char ICE:1;
			unsigned char IICRST:1;
			unsigned char CLO:1;
			unsigned char SOWP:1;
			unsigned char SCLO:1;
			unsigned char SDAO:1;
			unsigned char SCLI:1;
			unsigned char SDAI:1;
		} BIT;
	} ICCR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BBSY:1;
			unsigned char MST:1;
			unsigned char TRS:1;
			unsigned char :1;
			unsigned char SP:1;
			unsigned char RS:1;
			unsigned char ST:1;
		} BIT;
	} ICCR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char MTWP:1;
			unsigned char CKS:3;
			unsigned char BCWP:1;
			unsigned char BC:3;
		} BIT;
	} ICMR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char DLCS:1;
			unsigned char SDDL:3;
			unsigned char TMWE:1;
			unsigned char TMOH:1;
			unsigned char TMOL:1;
			unsigned char TMOS:1;
		} BIT;
	} ICMR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SMBS:1;
			unsigned char WAIT:1;
			unsigned char RDRFS:1;
			unsigned char ACKWP:1;
			unsigned char ACKBT:1;
			unsigned char ACKBR:1;
			unsigned char NF:2;
		} BIT;
	} ICMR3;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char SCLE:1;
			unsigned char NFE:1;
			unsigned char NACKE:1;
			unsigned char SALE:1;
			unsigned char NALE:1;
			unsigned char MALE:1;
			unsigned char TMOE:1;
		} BIT;
	} ICFER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char HOAE:1;
			unsigned char :1;
			unsigned char DIDE:1;
			unsigned char :1;
			unsigned char GCAE:1;
			unsigned char SAR2E:1;
			unsigned char SAR1E:1;
			unsigned char SAR0E:1;
		} BIT;
	} ICSER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TIE:1;
			unsigned char TEIE:1;
			unsigned char RIE:1;
			unsigned char NAKIE:1;
			unsigned char SPIE:1;
			unsigned char STIE:1;
			unsigned char ALIE:1;
			unsigned char TMOIE:1;
		} BIT;
	} ICIER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char HOA:1;
			unsigned char :1;
			unsigned char DID:1;
			unsigned char :1;
			unsigned char GCA:1;
			unsigned char AAS2:1;
			unsigned char AAS1:1;
			unsigned char AAS0:1;
		} BIT;
	} ICSR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TDRE:1;
			unsigned char TEND:1;
			unsigned char RDRF:1;
			unsigned char NACKF:1;
			unsigned char STOP:1;
			unsigned char START:1;
			unsigned char AL:1;
			unsigned char TMOF:1;
		} BIT;
	} ICSR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SVA:7;
			unsigned char SVA0:1;
		} BIT;
	} SARL0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SVA:2;
			unsigned char FS:1;
		} BIT;
	} SARU0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SVA:7;
			unsigned char SVA0:1;
		} BIT;
	} SARL1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SVA:2;
			unsigned char FS:1;
		} BIT;
	} SARU1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SVA:7;
			unsigned char SVA0:1;
		} BIT;
	} SARL2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SVA:2;
			unsigned char FS:1;
		} BIT;
	} SARU2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char BRL:5;
		} BIT;
	} ICBRL;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char BRH:5;
		} BIT;
	} ICBRH;
	unsigned char  ICDRT;
	unsigned char  ICDRR;
};

struct st_rspi {
	union {
		unsigned char BYTE;
		struct {
			unsigned char SPRIE:1;
			unsigned char SPE:1;
			unsigned char SPTIE:1;
			unsigned char SPEIE:1;
			unsigned char MSTR:1;
			unsigned char MODFEN:1;
			unsigned char TXMD:1;
			unsigned char SPMS:1;
		} BIT;
	} SPCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char SSL3P:1;
			unsigned char SSL2P:1;
			unsigned char SSL1P:1;
			unsigned char SSL0P:1;
		} BIT;
	} SSLP;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char MOIFE:1;
			unsigned char MOIFV:1;
			unsigned char :2;
			unsigned char SPLP2:1;
			unsigned char SPLP:1;
		} BIT;
	} SPPCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PERF:1;
			unsigned char MODF:1;
			unsigned char IDLNF:1;
			unsigned char OVRF:1;
		} BIT;
	} SPSR;
	union {
		unsigned long LONG;
		struct {
			unsigned short H;
		} WORD;
	} SPDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SPSLN:3;
		} BIT;
	} SPSCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char SPECM:3;
			unsigned char :1;
			unsigned char SPCP:3;
		} BIT;
	} SPSSR;
	unsigned char SPBR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char SPLW:1;
			unsigned char SPRDTD:1;
			unsigned char :2;
			unsigned char SPFC:2;
		} BIT;
	} SPDCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SCKDL:3;
		} BIT;
	} SPCKD;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SLNDL:3;
		} BIT;
	} SSLND;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SPNDL:3;
		} BIT;
	} SPND;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char PTE:1;
			unsigned char SPIIE:1;
			unsigned char SPOE:1;
			unsigned char SPPE:1;
		} BIT;
	} SPCR2;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD0;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD1;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD2;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD3;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD4;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD5;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD6;
	union {
		unsigned short WORD;
		struct {
			unsigned short SCKDEN:1;
			unsigned short SLNDEN:1;
			unsigned short SPNDEN:1;
			unsigned short LSBF:1;
			unsigned short SPB:4;
			unsigned short SSLKP:1;
			unsigned short SSLA:3;
			unsigned short BRDV:2;
			unsigned short CPOL:1;
			unsigned short CPHA:1;
		} BIT;
	} SPCMD7;
};

struct st_s12ad {
	union {
		unsigned short WORD;
		struct {
			unsigned short ADST:1;
			unsigned short ADCS:2;
			unsigned short ADIE:1;
			unsigned short :2;
			unsigned short TRGE:1;
			unsigned short EXTRG:1;
			unsigned short DBLE:1;
			unsigned short GBADIE:1;
			unsigned short :1;
			unsigned short DBLANS:5;
		} BIT;
	} ADCSR;
	char           wk0[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short :8;
			unsigned short ANSA:8;
		} BIT;
	} ADANSA;
	char           wk1[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short :8;
			unsigned short ADS:8;
		} BIT;
	} ADADS;
	char           wk2[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char ADC:2;
		} BIT;
	} ADADC;
	char           wk3[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short ADRFMT:1;
			unsigned short :3;
			unsigned short DIAGM:1;
			unsigned short DIAGLD:1;
			unsigned short DIAGVAL:2;
			unsigned short :2;
			unsigned short ACE:1;
			unsigned short DCE:1;
			unsigned short :1;
			unsigned short ADPRC:2;
		} BIT;
	} ADCER;
	union {
		unsigned short WORD;
		struct {
			unsigned short :3;
			unsigned short TRSA:5;
			unsigned short :3;
			unsigned short TRSB:5;
		} BIT;
	} ADSTRGR;
	char           wk4[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short :8;
			unsigned short ANSB:8;
		} BIT;
	} ADANSB;
	char           wk5[2];
	unsigned short ADDBLDR;
	char           wk6[4];
	unsigned short ADRD;
	unsigned short ADDR0;
	unsigned short ADDR1;
	unsigned short ADDR2;
	unsigned short ADDR3;
	unsigned short ADDR4;
	unsigned short ADDR5;
	unsigned short ADDR6;
	unsigned short ADDR7;
	char           wk7[48];
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR0;
	char           wk8[5];
	union {
		unsigned short WORD;
		struct {
			unsigned short :5;
			unsigned short SHANS:3;
			unsigned short SSTSH:8;
		} BIT;
	} ADSHCR;
	char           wk9[11];
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR3;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR4;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR5;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR6;
	union {
		unsigned char BYTE;
		struct {
			unsigned char SST:8;
		} BIT;
	} ADSSTR7;
	char           wk10[6];
	union {
		unsigned short WORD;
		struct {
			unsigned short GBRP:1;
			unsigned short :13;
			unsigned short GBRSCN:1;
			unsigned short PGS:1;
		} BIT;
	} ADGSPCR;
	char           wk11[2];
	unsigned short ADDBLDRA;
	unsigned short ADDBLDRB;
	char           wk12[88];
	union {
		unsigned short WORD;
		struct {
			unsigned short :10;
			unsigned short CEN002:2;
			unsigned short CEN001:2;
			unsigned short CEN000:2;
		} BIT;
	} ADCMPMD0;
	union {
		unsigned short WORD;
		struct {
			unsigned short :5;
			unsigned short VSELL0:1;
			unsigned short VSELH0:1;
			unsigned short :2;
			unsigned short REFH:3;
			unsigned short :1;
			unsigned short REFL:3;
		} BIT;
	} ADCMPMD1;
	union {
		unsigned short WORD;
		struct {
			unsigned short :4;
			unsigned short C002NR:4;
			unsigned short C001NR:4;
			unsigned short C000NR:4;
		} BIT;
	} ADCMPNR0;
	char           wk13[2];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char C002FLAG:1;
			unsigned char C001FLAG:1;
			unsigned char C000FLAG:1;
		} BIT;
	} ADCMPFR;
	char           wk14[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short :5;
			unsigned short POERQ002:1;
			unsigned short POERQ001:1;
			unsigned short POERQ000:1;
			unsigned short :5;
			unsigned short IE002:1;
			unsigned short IE001:1;
			unsigned short IE000:1;
		} BIT;
	} ADCMPSEL;
};

struct st_sci0 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char CM:1;
			unsigned char CHR:1;
			unsigned char PE:1;
			unsigned char PM:1;
			unsigned char STOP:1;
			unsigned char MP:1;
			unsigned char CKS:2;
		} BIT;
	} SMR;
	unsigned char  BRR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TIE:1;
			unsigned char RIE:1;
			unsigned char TE:1;
			unsigned char RE:1;
			unsigned char MPIE:1;
			unsigned char TEIE:1;
			unsigned char CKE:2;
		} BIT;
	} SCR;
	unsigned char  TDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char ORER:1;
			unsigned char FER:1;
			unsigned char PER:1;
			unsigned char TEND:1;
			unsigned char MPB:1;
			unsigned char MPBT:1;
		} BIT;
	} SSR;
	unsigned char  RDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BCP2:1;
			unsigned char :3;
			unsigned char SDIR:1;
			unsigned char SINV:1;
			unsigned char :1;
			unsigned char SMIF:1;
		} BIT;
	} SCMR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char NFEN:1;
			unsigned char ABCS:1;
		} BIT;
	} SEMR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char NFCS:3;
		} BIT;
	} SNFR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IICDL:5;
			unsigned char :2;
			unsigned char IICM:1;
		} BIT;
	} SIMR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char IICACKT:1;
			unsigned char :3;
			unsigned char IICCSC:1;
			unsigned char IICINTM:1;
		} BIT;
	} SIMR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IICSCLS:2;
			unsigned char IICSDAS:2;
			unsigned char IICSTIF:1;
			unsigned char IICSTPREQ:1;
			unsigned char IICRSTAREQ:1;
			unsigned char IICSTAREQ:1;
		} BIT;
	} SIMR3;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char IICACKR:1;
		} BIT;
	} SISR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char CKPH:1;
			unsigned char CKPOL:1;
			unsigned char :1;
			unsigned char MFF:1;
			unsigned char :1;
			unsigned char MSS:1;
			unsigned char CTSE:1;
			unsigned char SSE:1;
		} BIT;
	} SPMR;
};

struct st_sci12 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char CM:1;
			unsigned char CHR:1;
			unsigned char PE:1;
			unsigned char PM:1;
			unsigned char STOP:1;
			unsigned char MP:1;
			unsigned char CKS:2;
		} BIT;
	} SMR;
	unsigned char  BRR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TIE:1;
			unsigned char RIE:1;
			unsigned char TE:1;
			unsigned char RE:1;
			unsigned char MPIE:1;
			unsigned char TEIE:1;
			unsigned char CKE:2;
		} BIT;
	} SCR;
	unsigned char  TDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char ORER:1;
			unsigned char FER:1;
			unsigned char PER:1;
			unsigned char TEND:1;
			unsigned char MPB:1;
			unsigned char MPBT:1;
		} BIT;
	} SSR;
	unsigned char  RDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BCP2:1;
			unsigned char :3;
			unsigned char SDIR:1;
			unsigned char SINV:1;
			unsigned char :1;
			unsigned char SMIF:1;
		} BIT;
	} SCMR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char NFEN:1;
			unsigned char ABCS:1;
			unsigned char :3;
			unsigned char ACS0:1;
		} BIT;
	} SEMR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char NFCS:3;
		} BIT;
	} SNFR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IICDL:5;
			unsigned char :2;
			unsigned char IICM:1;
		} BIT;
	} SIMR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char IICACKT:1;
			unsigned char :3;
			unsigned char IICCSC:1;
			unsigned char IICINTM:1;
		} BIT;
	} SIMR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char IICSCLS:2;
			unsigned char IICSDAS:2;
			unsigned char IICSTIF:1;
			unsigned char IICSTPREQ:1;
			unsigned char IICRSTAREQ:1;
			unsigned char IICSTAREQ:1;
		} BIT;
	} SIMR3;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char IICACKR:1;
		} BIT;
	} SISR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char CKPH:1;
			unsigned char CKPOL:1;
			unsigned char :1;
			unsigned char MFF:1;
			unsigned char :1;
			unsigned char MSS:1;
			unsigned char CTSE:1;
			unsigned char SSE:1;
		} BIT;
	} SPMR;
	char           wk0[18];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char ESME:1;
		} BIT;
	} ESMER;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :4;
			unsigned char BRME:1;
			unsigned char RXDSF:1;
			unsigned char SFSF:1;
		} BIT;
	} CR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char PIBS:3;
			unsigned char PIBE:1;
			unsigned char CF1DS:2;
			unsigned char CF0RE:1;
			unsigned char BFE:1;
		} BIT;
	} CR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char RTS:2;
			unsigned char BCCS:2;
			unsigned char :1;
			unsigned char DFCS:3;
		} BIT;
	} CR2;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char SDST:1;
		} BIT;
	} CR3;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char SHARPS:1;
			unsigned char :2;
			unsigned char RXDXPS:1;
			unsigned char TXDXPS:1;
		} BIT;
	} PCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char AEDIE:1;
			unsigned char BCDIE:1;
			unsigned char PIBDIE:1;
			unsigned char CF1MIE:1;
			unsigned char CF0MIE:1;
			unsigned char BFDIE:1;
		} BIT;
	} ICR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char AEDF:1;
			unsigned char BCDF:1;
			unsigned char PIBDF:1;
			unsigned char CF1MF:1;
			unsigned char CF0MF:1;
			unsigned char BFDF:1;
		} BIT;
	} STR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char AEDCL:1;
			unsigned char BCDCL:1;
			unsigned char PIBDCL:1;
			unsigned char CF1MCL:1;
			unsigned char CF0MCL:1;
			unsigned char BFDCL:1;
		} BIT;
	} STCR;
	unsigned char  CF0DR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char CF0CE7:1;
			unsigned char CF0CE6:1;
			unsigned char CF0CE5:1;
			unsigned char CF0CE4:1;
			unsigned char CF0CE3:1;
			unsigned char CF0CE2:1;
			unsigned char CF0CE1:1;
			unsigned char CF0CE0:1;
		} BIT;
	} CF0CR;
	unsigned char  CF0RR;
	unsigned char  PCF1DR;
	unsigned char  SCF1DR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char CF1CE7:1;
			unsigned char CF1CE6:1;
			unsigned char CF1CE5:1;
			unsigned char CF1CE4:1;
			unsigned char CF1CE3:1;
			unsigned char CF1CE2:1;
			unsigned char CF1CE1:1;
			unsigned char CF1CE0:1;
		} BIT;
	} CF1CR;
	unsigned char  CF1RR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char TCST:1;
		} BIT;
	} TCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char TCSS:3;
			unsigned char TWRC:1;
			unsigned char :1;
			unsigned char TOMS:2;
		} BIT;
	} TMR;
	unsigned char  TPRE;
	unsigned char  TCNT;
};

struct st_smci0 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char GM:1;
			unsigned char BLK:1;
			unsigned char PE:1;
			unsigned char PM:1;
			unsigned char BCP:2;
			unsigned char CKS:2;
		} BIT;
	} SMR;
	unsigned char  BRR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TIE:1;
			unsigned char RIE:1;
			unsigned char TE:1;
			unsigned char RE:1;
			unsigned char MPIE:1;
			unsigned char TEIE:1;
			unsigned char CKE:2;
		} BIT;
	} SCR;
	unsigned char  TDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char ORER:1;
			unsigned char ERS:1;
			unsigned char PER:1;
			unsigned char TEND:1;
			unsigned char MPB:1;
			unsigned char MPBT:1;
		} BIT;
	} SSR;
	unsigned char  RDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BCP2:1;
			unsigned char :3;
			unsigned char SDIR:1;
			unsigned char SINV:1;
			unsigned char :1;
			unsigned char SMIF:1;
		} BIT;
	} SCMR;
};

struct st_smci12 {
	union {
		unsigned char BYTE;
		struct {
			unsigned char GM:1;
			unsigned char BCLK:1;
			unsigned char PE:1;
			unsigned char PM:1;
			unsigned char BCP:2;
			unsigned char CKS:2;
		} BIT;
	} SMR;
	unsigned char  BRR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char TIE:1;
			unsigned char RIE:1;
			unsigned char TE:1;
			unsigned char RE:1;
			unsigned char MPIE:1;
			unsigned char TEIE:1;
			unsigned char CKE:2;
		} BIT;
	} SCR;
	unsigned char  TDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char ORER:1;
			unsigned char ERS:1;
			unsigned char PER:1;
			unsigned char TEND:1;
			unsigned char MPB:1;
			unsigned char MPBT:1;
		} BIT;
	} SSR;
	unsigned char  RDR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char BCP2:1;
			unsigned char :3;
			unsigned char SDIR:1;
			unsigned char SINV:1;
			unsigned char :1;
			unsigned char SMIF:1;
		} BIT;
	} SCMR;
};

struct st_system {
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short MD:1;
		} BIT;
	} MDMONR;
	char           wk0[4];
	union {
		unsigned short WORD;
		struct {
			unsigned short KEY:8;
			unsigned short :7;
			unsigned short ROME:1;
		} BIT;
	} SYSCR0;
	union {
		unsigned short WORD;
		struct {
			unsigned short :15;
			unsigned short RAME:1;
		} BIT;
	} SYSCR1;
	char           wk1[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short SSBY:1;
		} BIT;
	} SBYCR;
	char           wk2[2];
	union {
		unsigned long LONG;
		struct {
			unsigned long ACSE:1;
			unsigned long :1;
			unsigned long MSTPA29:1;
			unsigned long MSTPA28:1;
			unsigned long MSTPA27:1;
			unsigned long :2;
			unsigned long MSTPA24:1;
			unsigned long :6;
			unsigned long MSTPA17:1;
			unsigned long :1;
			unsigned long MSTPA15:1;
			unsigned long MSTPA14:1;
			unsigned long :4;
			unsigned long MSTPA9:1;
			unsigned long :1;
			unsigned long MSTPA7:1;
		} BIT;
	} MSTPCRA;
	union {
		unsigned long LONG;
		struct {
			unsigned long MSTPB31:1;
			unsigned long MSTPB30:1;
			unsigned long :6;
			unsigned long MSTPB23:1;
			unsigned long :1;
			unsigned long MSTPB21:1;
			unsigned long :3;
			unsigned long MSTPB17:1;
			unsigned long :10;
			unsigned long MSTPB6:1;
			unsigned long :1;
			unsigned long MSTPB4:1;
		} BIT;
	} MSTPCRB;
	union {
		unsigned long LONG;
		struct {
			unsigned long :12;
			unsigned long MSTPC19:1;
			unsigned long :18;
			unsigned long MSTPC0:1;
		} BIT;
	} MSTPCRC;
	char           wk3[4];
	union {
		unsigned long LONG;
		struct {
			unsigned long FCK:4;
			unsigned long ICK:4;
			unsigned long :8;
			unsigned long PCKA:4;
			unsigned long PCKB:4;
			unsigned long :4;
			unsigned long PCKD:4;
		} BIT;
	} SCKCR;
	char           wk4[2];
	union {
		unsigned short WORD;
		struct {
			unsigned short :5;
			unsigned short CKSEL:3;
		} BIT;
	} SCKCR3;
	union {
		unsigned short WORD;
		struct {
			unsigned short :2;
			unsigned short STC:6;
			unsigned short :6;
			unsigned short PLIDIV:2;
		} BIT;
	} PLLCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char PLLEN:1;
		} BIT;
	} PLLCR2;
	char           wk5[7];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char MOSTP:1;
		} BIT;
	} MOSCCR;
	char           wk6[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char LCSTP:1;
		} BIT;
	} LOCOCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char ILCSTP:1;
		} BIT;
	} ILOCOCR;
	char           wk7[10];
	union {
		unsigned char BYTE;
		struct {
			unsigned char OSTDE:1;
			unsigned char :6;
			unsigned char OSTDIE:1;
		} BIT;
	} OSTDCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char OSTDF:1;
		} BIT;
	} OSTDSR;
	char           wk8[96];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char MSTS:5;
		} BIT;
	} MOSCWTCR;
	char           wk9[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char PSTS:5;
		} BIT;
	} PLLWTCR;
	char           wk10[25];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :5;
			unsigned char SWRF:1;
			unsigned char WDTRF:1;
			unsigned char IWDTRF:1;
		} BIT;
	} RSTSR2;
	char           wk11[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short SWRR:16;
		} BIT;
	} SWRR;
	char           wk12[28];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char LVD1IDTSEL:2;
		} BIT;
	} LVD1CR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char LVD1MON:1;
			unsigned char LVD1DET:1;
		} BIT;
	} LVD1SR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char LVD2IDTSEL:2;
		} BIT;
	} LVD2CR1;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :6;
			unsigned char LVD2MON:1;
			unsigned char LVD2DET:1;
		} BIT;
	} LVD2SR;
	char           wk13[794];
	union {
		unsigned short WORD;
		struct {
			unsigned short PRKEY:8;
			unsigned short :4;
			unsigned short PRC3:1;
			unsigned short :1;
			unsigned short PRC1:1;
			unsigned short PRC0:1;
		} BIT;
	} PRCR;
	char           wk14[48768];
	union {
		unsigned char BYTE;
		struct {
			unsigned char DPSBY:1;
			unsigned char IOKEEP:1;
		} BIT;
	} DPSBYCR;
	char           wk15[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char DIRQ5E:1;
			unsigned char DIRQ4E:1;
			unsigned char DIRQ3E:1;
			unsigned char DIRQ2E:1;
			unsigned char DIRQ1E:1;
			unsigned char DIRQ0E:1;
		} BIT;
	} DPSIER0;
	char           wk16[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char DNMIE:1;
			unsigned char :2;
			unsigned char DLVD2IE:1;
			unsigned char DLVD1IE:1;
		} BIT;
	} DPSIER2;
	char           wk17[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char DIRQ5F:1;
			unsigned char DIRQ4F:1;
			unsigned char DIRQ3F:1;
			unsigned char DIRQ2F:1;
			unsigned char DIRQ1F:1;
			unsigned char DIRQ0F:1;
		} BIT;
	} DPSIFR0;
	char           wk18[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char DNMIF:1;
			unsigned char :2;
			unsigned char DLVD2IF:1;
			unsigned char DLVD1IF:1;
		} BIT;
	} DPSIFR2;
	char           wk19[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :2;
			unsigned char DIRQ5EG:1;
			unsigned char DIRQ4EG:1;
			unsigned char DIRQ3EG:1;
			unsigned char DIRQ2EG:1;
			unsigned char DIRQ1EG:1;
			unsigned char DIRQ0EG:1;
		} BIT;
	} DPSIEGR0;
	char           wk20[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :3;
			unsigned char DNMIEG:1;
			unsigned char :2;
			unsigned char DLVD2EG:1;
			unsigned char DLVD1EG:1;
		} BIT;
	} DPSIEGR2;
	char           wk21[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char DPSRSTF:1;
			unsigned char :3;
			unsigned char LVD2RF:1;
			unsigned char LVD1RF:1;
			unsigned char LVD0RF:1;
			unsigned char PORF:1;
		} BIT;
	} RSTSR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char CWSF:1;
		} BIT;
	} RSTSR1;
	char           wk22[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :7;
			unsigned char MOFXIN:1;
		} BIT;
	} MOFCR;
	char           wk23[3];
	union {
		unsigned char BYTE;
		struct {
			unsigned char :1;
			unsigned char LVD2E:1;
			unsigned char LVD1E:1;
		} BIT;
	} LVCMPCR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char LVD2LVL:4;
			unsigned char LVD1LVL:4;
		} BIT;
	} LVDLVLR;
	char           wk24[1];
	union {
		unsigned char BYTE;
		struct {
			unsigned char LVD1RN:1;
			unsigned char LVD1RI:1;
			unsigned char LVD1FSAMP:2;
			unsigned char :1;
			unsigned char LVD1CMPE:1;
			unsigned char LVD1DFDIS:1;
			unsigned char LVD1RIE:1;
		} BIT;
	} LVD1CR0;
	union {
		unsigned char BYTE;
		struct {
			unsigned char LVD2RN:1;
			unsigned char LVD2RI:1;
			unsigned char LVD2FSAMP:2;
			unsigned char :1;
			unsigned char LVD2CMPE:1;
			unsigned char LVD2DFDIS:1;
			unsigned char LVD2RIE:1;
		} BIT;
	} LVD2CR0;
	char           wk25[4];
	unsigned char  DPSBKR[32];
};

struct st_wdt {
	unsigned char  WDTRR;
	char           wk0[1];
	union {
		unsigned short WORD;
		struct {
			unsigned short :2;
			unsigned short RPSS:2;
			unsigned short :2;
			unsigned short RPES:2;
			unsigned short CKS:4;
			unsigned short :2;
			unsigned short TOPS:2;
		} BIT;
	} WDTCR;
	union {
		unsigned short WORD;
		struct {
			unsigned short REFEF:1;
			unsigned short UNDFF:1;
			unsigned short CNTVAL:14;
		} BIT;
	} WDTSR;
	union {
		unsigned char BYTE;
		struct {
			unsigned char RSTIRQS:1;
		} BIT;
	} WDTRCR;
};

enum enum_ir {
IR_BSC_BUSERR=16,IR_FCU_FIFERR=21,IR_FCU_FRDYI=23,
IR_ICU_SWINT=27,
IR_CMT0_CMI0,
IR_CMT1_CMI1,
IR_CMT2_CMI2,
IR_CMT3_CMI3,
IR_CAC_FERRF=36,IR_CAC_MENDF,IR_CAC_OVFF,
IR_RSPI0_SPRI0,IR_RSPI0_SPTI0,IR_RSPI0_SPII0,
IR_DOC_DOPCF=57,
IR_ICU_IRQ0=64,IR_ICU_IRQ1,IR_ICU_IRQ2,IR_ICU_IRQ3,IR_ICU_IRQ4,IR_ICU_IRQ5,
IR_S12AD_S12ADI=102,IR_S12AD_S12GBADI,
IR_ICU_GROUP12=114,
IR_SCI12_SCIX0=122,IR_SCI12_SCIX1,IR_SCI12_SCIX2,IR_SCI12_SCIX3,
IR_MTU0_TGIA0,IR_MTU0_TGIB0,IR_MTU0_TGIC0,IR_MTU0_TGID0,IR_MTU0_TCIV0,IR_MTU0_TGIE0,IR_MTU0_TGIF0,
IR_MTU1_TGIA1,IR_MTU1_TGIB1,IR_MTU1_TCIV1,IR_MTU1_TCIU1,
IR_MTU2_TGIA2,IR_MTU2_TGIB2,IR_MTU2_TCIV2,IR_MTU2_TCIU2,
IR_MTU3_TGIA3,IR_MTU3_TGIB3,IR_MTU3_TGIC3,IR_MTU3_TGID3,IR_MTU3_TCIV3,
IR_MTU4_TGIA4,IR_MTU4_TGIB4,IR_MTU4_TGIC4,IR_MTU4_TGID4,IR_MTU4_TCIV4,
IR_MTU5_TGIU5,IR_MTU5_TGIV5,IR_MTU5_TGIW5,
IR_MTU6_TGIA6,IR_MTU6_TGIB6,IR_MTU6_TGIC6,IR_MTU6_TGID6,IR_MTU6_TCIV6,
IR_MTU7_TGIA7=161,IR_MTU7_TGIB7,IR_MTU7_TGIC7,IR_MTU7_TGID7,IR_MTU7_TCIV7,
IR_POE_OEI1,IR_POE_OEI3=168,IR_POE_OEI4,
IR_CMP_CMP0=171,IR_CMP_CMP1,IR_CMP_CMP2,
IR_RIIC0_EEI0=194,IR_RIIC0_RXI0,IR_RIIC0_TXI0,IR_RIIC0_TEI0,
IR_DMAC_DMAC0I,IR_DMAC_DMAC1I,IR_DMAC_DMAC2I,IR_DMAC_DMAC3I,
IR_SCI0_RXI0=214,IR_SCI0_TXI0,IR_SCI0_TEI0,
IR_SCI1_RXI1,IR_SCI1_TXI1,IR_SCI1_TEI1,
IR_GPT0_GTCIA0=226,IR_GPT0_GTCIB0,IR_GPT0_GTCIC0,IR_GPT0_GTCIE0,IR_GPT0_GTCIV0,IR_GPT0_LOCOI,
IR_GPT1_GTCIA1,IR_GPT1_GTCIB1,IR_GPT1_GTCIC1,IR_GPT1_GTCIE1,IR_GPT1_GTCIV1,
IR_GPT2_GTCIA2=238,IR_GPT2_GTCIB2,IR_GPT2_GTCIC2,IR_GPT2_GTCIE2,IR_GPT2_GTCIV2,
IR_GPT3_GTCIA3=244,IR_GPT3_GTCIB3,IR_GPT3_GTCIC3,IR_GPT3_GTCIE3,IR_GPT3_GTCIV3,
IR_SCI12_RXI12=250,IR_SCI12_TXI12,IR_SCI12_TEI12
};

enum enum_dtce {
DTCE_ICU_SWINT=27,
DTCE_CMT0_CMI0,
DTCE_CMT1_CMI1,
DTCE_CMT2_CMI2,
DTCE_CMT3_CMI3,
DTCE_RSPI0_SPRI0=39,DTCE_RSPI0_SPTI0,
DTCE_ICU_IRQ0=64,DTCE_ICU_IRQ1,DTCE_ICU_IRQ2,DTCE_ICU_IRQ3,DTCE_ICU_IRQ4,DTCE_ICU_IRQ5,
DTCE_S12AD_S12ADI=102,DTCE_S12AD_S12GBADI,
DTCE_MTU0_TGIA0=126,DTCE_MTU0_TGIB0,DTCE_MTU0_TGIC0,DTCE_MTU0_TGID0,
DTCE_MTU1_TGIA1=133,DTCE_MTU1_TGIB1,
DTCE_MTU2_TGIA2=137,DTCE_MTU2_TGIB2,
DTCE_MTU3_TGIA3=141,DTCE_MTU3_TGIB3,DTCE_MTU3_TGIC3,DTCE_MTU3_TGID3,
DTCE_MTU4_TGIA4=146,DTCE_MTU4_TGIB4,DTCE_MTU4_TGIC4,DTCE_MTU4_TGID4,DTCE_MTU4_TCIV4,
DTCE_MTU5_TGIU5,DTCE_MTU5_TGIV5,DTCE_MTU5_TGIW5,
DTCE_MTU6_TGIA6,DTCE_MTU6_TGIB6,DTCE_MTU6_TGIC6,DTCE_MTU6_TGID6,
DTCE_MTU7_TGIA7=161,DTCE_MTU7_TGIB7,DTCE_MTU7_TGIC7,DTCE_MTU7_TGID7,DTCE_MTU7_TCIV7,
DTCE_CMP_CMP0=171,DTCE_CMP_CMP1,DTCE_CMP_CMP2,
DTCE_RIIC0_RXI0=195,DTCE_RIIC0_TXI0,
DTCE_DMAC_DMAC0I=198,DTCE_DMAC_DMAC1I,DTCE_DMAC_DMAC2I,DTCE_DMAC_DMAC3I,
DTCE_SCI0_RXI0=214,DTCE_SCI0_TXI0,
DTCE_SCI1_RXI1=217,DTCE_SCI1_TXI1,
DTCE_GPT0_GTCIA0=226,DTCE_GPT0_GTCIB0,DTCE_GPT0_GTCIC0,DTCE_GPT0_GTCIE0,DTCE_GPT0_GTCIV0,DTCE_GPT0_LOCOI,
DTCE_GPT1_GTCIA1,DTCE_GPT1_GTCIB1,DTCE_GPT1_GTCIC1,DTCE_GPT1_GTCIE1,DTCE_GPT1_GTCIV1,
DTCE_GPT2_GTCIA2=238,DTCE_GPT2_GTCIB2,DTCE_GPT2_GTCIC2,DTCE_GPT2_GTCIE2,DTCE_GPT2_GTCIV2,
DTCE_GPT3_GTCIA3=244,DTCE_GPT3_GTCIB3,DTCE_GPT3_GTCIC3,DTCE_GPT3_GTCIE3,DTCE_GPT3_GTCIV3,
DTCE_SCI12_RXI12=250,DTCE_SCI12_TXI12
};

enum enum_ier {
IER_BSC_BUSERR=0x02,
IER_FCU_FIFERR=0x02,IER_FCU_FRDYI=0x02,
IER_ICU_SWINT=0x03,
IER_CMT0_CMI0=0x03,
IER_CMT1_CMI1=0x03,
IER_CMT2_CMI2=0x03,
IER_CMT3_CMI3=0x03,
IER_CAC_FERRF=0x04,IER_CAC_MENDF=0x04,IER_CAC_OVFF=0x04,
IER_RSPI0_SPRI0=0x04,IER_RSPI0_SPTI0=0x05,IER_RSPI0_SPII0=0x05,
IER_DOC_DOPCF=0x07,
IER_ICU_IRQ0=0x08,IER_ICU_IRQ1=0x08,IER_ICU_IRQ2=0x08,IER_ICU_IRQ3=0x08,IER_ICU_IRQ4=0x08,IER_ICU_IRQ5=0x08,
IER_S12AD_S12ADI=0x0C,IER_S12AD_S12GBADI=0x0C,
IER_ICU_GROUP12=0x0E,
IER_SCI12_SCIX0=0x0F,IER_SCI12_SCIX1=0x0F,IER_SCI12_SCIX2=0x0F,IER_SCI12_SCIX3=0x0F,
IER_MTU0_TGIA0=0x0F,IER_MTU0_TGIB0=0x0F,IER_MTU0_TGIC0=0x10,IER_MTU0_TGID0=0x10,IER_MTU0_TCIV0=0x10,IER_MTU0_TGIE0=0x10,IER_MTU0_TGIF0=0x10,
IER_MTU1_TGIA1=0x10,IER_MTU1_TGIB1=0x10,IER_MTU1_TCIV1=0x10,IER_MTU1_TCIU1=0x11,
IER_MTU2_TGIA2=0x11,IER_MTU2_TGIB2=0x11,IER_MTU2_TCIV2=0x11,IER_MTU2_TCIU2=0x11,
IER_MTU3_TGIA3=0x11,IER_MTU3_TGIB3=0x11,IER_MTU3_TGIC3=0x11,IER_MTU3_TGID3=0x12,IER_MTU3_TCIV3=0x12,
IER_MTU4_TGIA4=0x12,IER_MTU4_TGIB4=0x12,IER_MTU4_TGIC4=0x12,IER_MTU4_TGID4=0x12,IER_MTU4_TCIV4=0x12,
IER_MTU5_TGIU5=0x12,IER_MTU5_TGIV5=0x13,IER_MTU5_TGIW5=0x13,
IER_MTU6_TGIA6=0x13,IER_MTU6_TGIB6=0x13,IER_MTU6_TGIC6=0x13,IER_MTU6_TGID6=0x13,IER_MTU6_TCIV6=0x13,
IER_MTU7_TGIA7=0x14,IER_MTU7_TGIB7=0x14,IER_MTU7_TGIC7=0x14,IER_MTU7_TGID7=0x14,IER_MTU7_TCIV7=0x14,
IER_POE_OEI1=0x14,IER_POE_OEI3=0x15,IER_POE_OEI4=0x15,
IER_CMP_CMP0=0x15,IER_CMP_CMP1=0x15,IER_CMP_CMP2=0x15,
IER_RIIC0_EEI0=0x18,IER_RIIC0_RXI0=0x18,IER_RIIC0_TXI0=0x18,IER_RIIC0_TEI0=0x18,
IER_DMAC_DMAC0I=0x18,IER_DMAC_DMAC1I=0x18,IER_DMAC_DMAC2I=0x19,IER_DMAC_DMAC3I=0x19,
IER_SCI0_RXI0=0x1A,IER_SCI0_TXI0=0x1A,IER_SCI0_TEI0=0x1B,
IER_SCI1_RXI1=0x1B,IER_SCI1_TXI1=0x1B,IER_SCI1_TEI1=0x1B,
IER_GPT0_GTCIA0=0x1C,IER_GPT0_GTCIB0=0x1C,IER_GPT0_GTCIC0=0x1C,IER_GPT0_GTCIE0=0x1C,IER_GPT0_GTCIV0=0x1C,IER_GPT0_LOCOI=0x1C,
IER_GPT1_GTCIA1=0x1D,IER_GPT1_GTCIB1=0x1D,IER_GPT1_GTCIC1=0x1D,IER_GPT1_GTCIE1=0x1D,IER_GPT1_GTCIV1=0x1D,
IER_GPT2_GTCIA2=0x1D,IER_GPT2_GTCIB2=0x1D,IER_GPT2_GTCIC2=0x1E,IER_GPT2_GTCIE2=0x1E,IER_GPT2_GTCIV2=0x1E,
IER_GPT3_GTCIA3=0x1E,IER_GPT3_GTCIB3=0x1E,IER_GPT3_GTCIC3=0x1E,IER_GPT3_GTCIE3=0x1E,IER_GPT3_GTCIV3=0x1F,
IER_SCI12_RXI12=0x1F,IER_SCI12_TXI12=0x1F,IER_SCI12_TEI12=0x1F
};

enum enum_ipr {
IPR_BSC_BUSERR=0,
IPR_FCU_FIFERR=1,IPR_FCU_FRDYI=2,
IPR_ICU_SWINT=3,
IPR_CMT0_CMI0=4,
IPR_CMT1_CMI1=5,
IPR_CMT2_CMI2=6,
IPR_CMT3_CMI3=7,
IPR_CAC_FERRF=36,IPR_CAC_MENDF=36,IPR_CAC_OVFF=36,
IPR_RSPI0_SPRI0=39,IPR_RSPI0_SPTI0=40,IPR_RSPI0_SPII0=41,
IPR_DOC_DOPCF=57,
IPR_ICU_IRQ0=64,IPR_ICU_IRQ1=65,IPR_ICU_IRQ2=66,IPR_ICU_IRQ3=67,IPR_ICU_IRQ4=68,IPR_ICU_IRQ5=69,
IPR_S12AD_S12ADI=102,IPR_S12AD_S12GBADI=103,
IPR_ICU_GROUP12=114,
IPR_SCI12_SCIX0=122,IPR_SCI12_SCIX1=122,IPR_SCI12_SCIX2=122,IPR_SCI12_SCIX3=122,
IPR_MTU0_TGIA0=126,IPR_MTU0_TGIB0=126,IPR_MTU0_TGIC0=126,IPR_MTU0_TGID0=126,IPR_MTU0_TCIV0=130,IPR_MTU0_TGIE0=130,IPR_MTU0_TGIF0=130,
IPR_MTU1_TGIA1=133,IPR_MTU1_TGIB1=133,IPR_MTU1_TCIV1=135,IPR_MTU1_TCIU1=135,
IPR_MTU2_TGIA2=137,IPR_MTU2_TGIB2=137,IPR_MTU2_TCIV2=139,IPR_MTU2_TCIU2=139,
IPR_MTU3_TGIA3=141,IPR_MTU3_TGIB3=141,IPR_MTU3_TGIC3=141,IPR_MTU3_TGID3=141,IPR_MTU3_TCIV3=145,
IPR_MTU4_TGIA4=146,IPR_MTU4_TGIB4=146,IPR_MTU4_TGIC4=146,IPR_MTU4_TGID4=146,IPR_MTU4_TCIV4=150,
IPR_MTU5_TGIU5=151,IPR_MTU5_TGIV5=151,IPR_MTU5_TGIW5=151,
IPR_MTU6_TGIA6=154,IPR_MTU6_TGIB6=154,IPR_MTU6_TGIC6=154,IPR_MTU6_TGID6=154,IPR_MTU6_TCIV6=158,
IPR_MTU7_TGIA7=161,IPR_MTU7_TGIB7=161,IPR_MTU7_TGIC7=163,IPR_MTU7_TGID7=163,IPR_MTU7_TCIV7=165,
IPR_POE_OEI1=166,IPR_POE_OEI3=166,IPR_POE_OEI4=166,
IPR_CMP_CMP0=171,IPR_CMP_CMP1=172,IPR_CMP_CMP2=173,
IPR_RIIC0_EEI0=194,IPR_RIIC0_RXI0=194,IPR_RIIC0_TXI0=194,IPR_RIIC0_TEI0=194,
IPR_DMAC_DMAC0I=198,IPR_DMAC_DMAC1I=199,IPR_DMAC_DMAC2I=200,IPR_DMAC_DMAC3I=201,
IPR_SCI0_RXI0=214,IPR_SCI0_TXI0=214,IPR_SCI0_TEI0=214,
IPR_SCI1_RXI1=217,IPR_SCI1_TXI1=217,IPR_SCI1_TEI1=217,
IPR_GPT0_GTCIA0=226,IPR_GPT0_GTCIB0=226,IPR_GPT0_GTCIC0=226,IPR_GPT0_GTCIE0=229,IPR_GPT0_GTCIV0=229,IPR_GPT0_LOCOI=229,
IPR_GPT1_GTCIA1=232,IPR_GPT1_GTCIB1=232,IPR_GPT1_GTCIC1=232,IPR_GPT1_GTCIE1=235,IPR_GPT1_GTCIV1=235,
IPR_GPT2_GTCIA2=238,IPR_GPT2_GTCIB2=238,IPR_GPT2_GTCIC2=238,IPR_GPT2_GTCIE2=241,IPR_GPT2_GTCIV2=241,
IPR_GPT3_GTCIA3=244,IPR_GPT3_GTCIB3=244,IPR_GPT3_GTCIC3=244,IPR_GPT3_GTCIE3=247,IPR_GPT3_GTCIV3=247,
IPR_SCI12_RXI12=250,IPR_SCI12_TXI12=250,IPR_SCI12_TEI12=250,
IPR_BSC_=0,
IPR_CMT0_=4,
IPR_CMT1_=5,
IPR_CMT2_=6,
IPR_CMT3_=7,
IPR_CAC_=36,
IPR_DOC_=57,
IPR_SCI12_SCIX_=122,
IPR_MTU1_TGI=133,
IPR_MTU1_TCI=135,
IPR_MTU2_TGI=137,
IPR_MTU2_TCI=139,
IPR_MTU3_TGI=141,
IPR_MTU4_TGI=146,
IPR_MTU5_=151,
IPR_MTU5_TGI=151,
IPR_MTU6_TGI=154,
IPR_POE_=166,
IPR_RIIC0_=194,
IPR_SCI0_=214,
IPR_SCI1_=217,
IPR_SCI12_=250,
};

enum enum_grp {
GRP_SCI0_ERI0=12,GRP_SCI1_ERI1=12,GRP_SCI12_ERI12=12,GRP_RSPI0_SPEI0=12
};

enum enum_gen {
GEN_SCI0_ERI0=12,GEN_SCI1_ERI1=12,GEN_SCI12_ERI12=12,GEN_RSPI0_SPEI0=12
};

#define	IEN_BSC_BUSERR		IEN0
#define	IEN_FCU_FIFERR		IEN5
#define	IEN_FCU_FRDYI		IEN7
#define	IEN_ICU_SWINT		IEN3
#define	IEN_CMT0_CMI0		IEN4
#define	IEN_CMT1_CMI1		IEN5
#define	IEN_CMT2_CMI2		IEN6
#define	IEN_CMT3_CMI3		IEN7
#define	IEN_CAC_FERRF		IEN4
#define	IEN_CAC_MENDF		IEN5
#define	IEN_CAC_OVFF		IEN6
#define	IEN_RSPI0_SPRI0		IEN7
#define	IEN_RSPI0_SPTI0		IEN0
#define	IEN_RSPI0_SPII0		IEN1
#define	IEN_DOC_DOPCF		IEN1
#define	IEN_ICU_IRQ0		IEN0
#define	IEN_ICU_IRQ1		IEN1
#define	IEN_ICU_IRQ2		IEN2
#define	IEN_ICU_IRQ3		IEN3
#define	IEN_ICU_IRQ4		IEN4
#define	IEN_ICU_IRQ5		IEN5
#define	IEN_S12AD_S12ADI	IEN6
#define	IEN_S12AD_S12GBADI	IEN7
#define	IEN_ICU_GROUP12		IEN2
#define	IEN_SCI12_SCIX0		IEN2
#define	IEN_SCI12_SCIX1		IEN3
#define	IEN_SCI12_SCIX2		IEN4
#define	IEN_SCI12_SCIX3		IEN5
#define	IEN_MTU0_TGIA0		IEN6
#define	IEN_MTU0_TGIB0		IEN7
#define	IEN_MTU0_TGIC0		IEN0
#define	IEN_MTU0_TGID0		IEN1
#define	IEN_MTU0_TCIV0		IEN2
#define	IEN_MTU0_TGIE0		IEN3
#define	IEN_MTU0_TGIF0		IEN4
#define	IEN_MTU1_TGIA1		IEN5
#define	IEN_MTU1_TGIB1		IEN6
#define	IEN_MTU1_TCIV1		IEN7
#define	IEN_MTU1_TCIU1		IEN0
#define	IEN_MTU2_TGIA2		IEN1
#define	IEN_MTU2_TGIB2		IEN2
#define	IEN_MTU2_TCIV2		IEN3
#define	IEN_MTU2_TCIU2		IEN4
#define	IEN_MTU3_TGIA3		IEN5
#define	IEN_MTU3_TGIB3		IEN6
#define	IEN_MTU3_TGIC3		IEN7
#define	IEN_MTU3_TGID3		IEN0
#define	IEN_MTU3_TCIV3		IEN1
#define	IEN_MTU4_TGIA4		IEN2
#define	IEN_MTU4_TGIB4		IEN3
#define	IEN_MTU4_TGIC4		IEN4
#define	IEN_MTU4_TGID4		IEN5
#define	IEN_MTU4_TCIV4		IEN6
#define	IEN_MTU5_TGIU5		IEN7
#define	IEN_MTU5_TGIV5		IEN0
#define	IEN_MTU5_TGIW5		IEN1
#define	IEN_MTU6_TGIA6		IEN2
#define	IEN_MTU6_TGIB6		IEN3
#define	IEN_MTU6_TGIC6		IEN4
#define	IEN_MTU6_TGID6		IEN5
#define	IEN_MTU6_TCIV6		IEN6
#define	IEN_MTU7_TGIA7		IEN1
#define	IEN_MTU7_TGIB7		IEN2
#define	IEN_MTU7_TGIC7		IEN3
#define	IEN_MTU7_TGID7		IEN4
#define	IEN_MTU7_TCIV7		IEN5
#define	IEN_POE_OEI1		IEN6
#define	IEN_POE_OEI3		IEN0
#define	IEN_POE_OEI4		IEN1
#define	IEN_CMP_CMP0		IEN3
#define	IEN_CMP_CMP1		IEN4
#define	IEN_CMP_CMP2		IEN5
#define	IEN_RIIC0_EEI0		IEN2
#define	IEN_RIIC0_RXI0		IEN3
#define	IEN_RIIC0_TXI0		IEN4
#define	IEN_RIIC0_TEI0		IEN5
#define	IEN_DMAC_DMAC0I		IEN6
#define	IEN_DMAC_DMAC1I		IEN7
#define	IEN_DMAC_DMAC2I		IEN0
#define	IEN_DMAC_DMAC3I		IEN1
#define	IEN_SCI0_RXI0		IEN6
#define	IEN_SCI0_TXI0		IEN7
#define	IEN_SCI0_TEI0		IEN0
#define	IEN_SCI1_RXI1		IEN1
#define	IEN_SCI1_TXI1		IEN2
#define	IEN_SCI1_TEI1		IEN3
#define	IEN_GPT0_GTCIA0		IEN2
#define	IEN_GPT0_GTCIB0		IEN3
#define	IEN_GPT0_GTCIC0		IEN4
#define	IEN_GPT0_GTCIE0		IEN5
#define	IEN_GPT0_GTCIV0		IEN6
#define	IEN_GPT0_LOCOI		IEN7
#define	IEN_GPT1_GTCIA1		IEN0
#define	IEN_GPT1_GTCIB1		IEN1
#define	IEN_GPT1_GTCIC1		IEN2
#define	IEN_GPT1_GTCIE1		IEN3
#define	IEN_GPT1_GTCIV1		IEN4
#define	IEN_GPT2_GTCIA2		IEN6
#define	IEN_GPT2_GTCIB2		IEN7
#define	IEN_GPT2_GTCIC2		IEN0
#define	IEN_GPT2_GTCIE2		IEN1
#define	IEN_GPT2_GTCIV2		IEN2
#define	IEN_GPT3_GTCIA3		IEN4
#define	IEN_GPT3_GTCIB3		IEN5
#define	IEN_GPT3_GTCIC3		IEN6
#define	IEN_GPT3_GTCIE3		IEN7
#define	IEN_GPT3_GTCIV3		IEN0
#define	IEN_SCI12_RXI12		IEN2
#define	IEN_SCI12_TXI12		IEN3
#define	IEN_SCI12_TEI12		IEN4

#define	VECT_BSC_BUSERR		16
#define	VECT_FCU_FIFERR		21
#define	VECT_FCU_FRDYI		23
#define	VECT_ICU_SWINT		27
#define	VECT_CMT0_CMI0		28
#define	VECT_CMT1_CMI1		29
#define	VECT_CMT2_CMI2		30
#define	VECT_CMT3_CMI3		31
#define	VECT_CAC_FERRF		36
#define	VECT_CAC_MENDF		37
#define	VECT_CAC_OVFF		38
#define	VECT_RSPI0_SPRI0	39
#define	VECT_RSPI0_SPTI0	40
#define	VECT_RSPI0_SPII0	41
#define	VECT_DOC_DOPCF		57
#define	VECT_ICU_IRQ0		64
#define	VECT_ICU_IRQ1		65
#define	VECT_ICU_IRQ2		66
#define	VECT_ICU_IRQ3		67
#define	VECT_ICU_IRQ4		68
#define	VECT_ICU_IRQ5		69
#define	VECT_S12AD_S12ADI	102
#define	VECT_S12AD_S12GBADI	103
#define	VECT_ICU_GROUP12	114
#define	VECT_SCI12_SCIX0	122
#define	VECT_SCI12_SCIX1	123
#define	VECT_SCI12_SCIX2	124
#define	VECT_SCI12_SCIX3	125
#define	VECT_MTU0_TGIA0		126
#define	VECT_MTU0_TGIB0		127
#define	VECT_MTU0_TGIC0		128
#define	VECT_MTU0_TGID0		129
#define	VECT_MTU0_TCIV0		130
#define	VECT_MTU0_TGIE0		131
#define	VECT_MTU0_TGIF0		132
#define	VECT_MTU1_TGIA1		133
#define	VECT_MTU1_TGIB1		134
#define	VECT_MTU1_TCIV1		135
#define	VECT_MTU1_TCIU1		136
#define	VECT_MTU2_TGIA2		137
#define	VECT_MTU2_TGIB2		138
#define	VECT_MTU2_TCIV2		139
#define	VECT_MTU2_TCIU2		140
#define	VECT_MTU3_TGIA3		141
#define	VECT_MTU3_TGIB3		142
#define	VECT_MTU3_TGIC3		143
#define	VECT_MTU3_TGID3		144
#define	VECT_MTU3_TCIV3		145
#define	VECT_MTU4_TGIA4		146
#define	VECT_MTU4_TGIB4		147
#define	VECT_MTU4_TGIC4		148
#define	VECT_MTU4_TGID4		149
#define	VECT_MTU4_TCIV4		150
#define	VECT_MTU5_TGIU5		151
#define	VECT_MTU5_TGIV5		152
#define	VECT_MTU5_TGIW5		153
#define	VECT_MTU6_TGIA6		154
#define	VECT_MTU6_TGIB6		155
#define	VECT_MTU6_TGIC6		156
#define	VECT_MTU6_TGID6		157
#define	VECT_MTU6_TCIV6		158
#define	VECT_MTU7_TGIA7		161
#define	VECT_MTU7_TGIB7		162
#define	VECT_MTU7_TGIC7		163
#define	VECT_MTU7_TGID7		164
#define	VECT_MTU7_TCIV7		165
#define	VECT_POE_OEI1		166
#define	VECT_POE_OEI3		168
#define	VECT_POE_OEI4		169
#define	VECT_CMP_CMP0		171
#define	VECT_CMP_CMP1		172
#define	VECT_CMP_CMP2		173
#define	VECT_RIIC0_EEI0		194
#define	VECT_RIIC0_RXI0		195
#define	VECT_RIIC0_TXI0		196
#define	VECT_RIIC0_TEI0		197
#define	VECT_DMAC_DMAC0I	198
#define	VECT_DMAC_DMAC1I	199
#define	VECT_DMAC_DMAC2I	200
#define	VECT_DMAC_DMAC3I	201
#define	VECT_SCI0_RXI0		214
#define	VECT_SCI0_TXI0		215
#define	VECT_SCI0_TEI0		216
#define	VECT_SCI1_RXI1		217
#define	VECT_SCI1_TXI1		218
#define	VECT_SCI1_TEI1		219
#define	VECT_GPT0_GTCIA0	226
#define	VECT_GPT0_GTCIB0	227
#define	VECT_GPT0_GTCIC0	228
#define	VECT_GPT0_GTCIE0	229
#define	VECT_GPT0_GTCIV0	230
#define	VECT_GPT0_LOCOI		231
#define	VECT_GPT1_GTCIA1	232
#define	VECT_GPT1_GTCIB1	233
#define	VECT_GPT1_GTCIC1	234
#define	VECT_GPT1_GTCIE1	235
#define	VECT_GPT1_GTCIV1	236
#define	VECT_GPT2_GTCIA2	238
#define	VECT_GPT2_GTCIB2	239
#define	VECT_GPT2_GTCIC2	240
#define	VECT_GPT2_GTCIE2	241
#define	VECT_GPT2_GTCIV2	242
#define	VECT_GPT3_GTCIA3	244
#define	VECT_GPT3_GTCIB3	245
#define	VECT_GPT3_GTCIC3	246
#define	VECT_GPT3_GTCIE3	247
#define	VECT_GPT3_GTCIV3	248
#define	VECT_SCI12_RXI12	250
#define	VECT_SCI12_TXI12	251
#define	VECT_SCI12_TEI12	252

#define	MSTP_DMAC	SYSTEM.MSTPCRA.BIT.MSTPA28
#define	MSTP_DMAC0	SYSTEM.MSTPCRA.BIT.MSTPA28
#define	MSTP_DMAC1	SYSTEM.MSTPCRA.BIT.MSTPA28
#define	MSTP_DMAC2	SYSTEM.MSTPCRA.BIT.MSTPA28
#define	MSTP_DMAC3	SYSTEM.MSTPCRA.BIT.MSTPA28
#define	MSTP_DTC	SYSTEM.MSTPCRA.BIT.MSTPA28
#define	MSTP_S12AD	SYSTEM.MSTPCRA.BIT.MSTPA17
#define	MSTP_CMT0	SYSTEM.MSTPCRA.BIT.MSTPA15
#define	MSTP_CMT1	SYSTEM.MSTPCRA.BIT.MSTPA15
#define	MSTP_CMT2	SYSTEM.MSTPCRA.BIT.MSTPA14
#define	MSTP_CMT3	SYSTEM.MSTPCRA.BIT.MSTPA14
#define	MSTP_MTU7	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU6	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU5	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU4	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU3	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU2	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU1	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU0	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_MTU	SYSTEM.MSTPCRA.BIT.MSTPA9
#define	MSTP_GPT3	SYSTEM.MSTPCRA.BIT.MSTPA7
#define	MSTP_GPT2	SYSTEM.MSTPCRA.BIT.MSTPA7
#define	MSTP_GPT1	SYSTEM.MSTPCRA.BIT.MSTPA7
#define	MSTP_GPT0	SYSTEM.MSTPCRA.BIT.MSTPA7
#define	MSTP_GPT	SYSTEM.MSTPCRA.BIT.MSTPA7
#define	MSTP_SCI0	SYSTEM.MSTPCRB.BIT.MSTPB31
#define	MSTP_SMCI0	SYSTEM.MSTPCRB.BIT.MSTPB31
#define	MSTP_SCI1	SYSTEM.MSTPCRB.BIT.MSTPB30
#define	MSTP_SMCI1	SYSTEM.MSTPCRB.BIT.MSTPB30
#define	MSTP_CRC	SYSTEM.MSTPCRB.BIT.MSTPB23
#define	MSTP_RIIC0	SYSTEM.MSTPCRB.BIT.MSTPB21
#define	MSTP_RSPI0	SYSTEM.MSTPCRB.BIT.MSTPB17
#define	MSTP_DOC	SYSTEM.MSTPCRB.BIT.MSTPB6
#define	MSTP_SCI12	SYSTEM.MSTPCRB.BIT.MSTPB4
#define	MSTP_SMCI12	SYSTEM.MSTPCRB.BIT.MSTPB4
#define	MSTP_CAC	SYSTEM.MSTPCRC.BIT.MSTPC19
#define	MSTP_RAM0	SYSTEM.MSTPCRC.BIT.MSTPC0

#define	IS_SCI0_ERI0		IS0
#define	IS_SCI1_ERI1		IS1
#define	IS_SCI12_ERI12		IS4
#define	IS_RSPI0_SPEI0		IS5

#define	EN_SCI0_ERI0		EN0
#define	EN_SCI1_ERI1		EN1
#define	EN_SCI12_ERI12		EN4
#define	EN_RSPI0_SPEI0		EN5

#define	__IR( x )		ICU.IR[ IR ## x ].BIT.IR
#define	 _IR( x )		__IR( x )
#define	  IR( x , y )	_IR( _ ## x ## _ ## y )
#define	__DTCE( x )		ICU.DTCER[ DTCE ## x ].BIT.DTCE
#define	 _DTCE( x )		__DTCE( x )
#define	  DTCE( x , y )	_DTCE( _ ## x ## _ ## y )
#define	__IEN( x )		ICU.IER[ IER ## x ].BIT.IEN ## x
#define	 _IEN( x )		__IEN( x )
#define	  IEN( x , y )	_IEN( _ ## x ## _ ## y )
#define	__IPR( x )		ICU.IPR[ IPR ## x ].BIT.IPR
#define	 _IPR( x )		__IPR( x )
#define	  IPR( x , y )	_IPR( _ ## x ## _ ## y )
#define	__VECT( x )		VECT ## x
#define	 _VECT( x )		__VECT( x )
#define	  VECT( x , y )	_VECT( _ ## x ## _ ## y )
#define	__MSTP( x )		MSTP ## x
#define	 _MSTP( x )		__MSTP( x )
#define	  MSTP( x )		_MSTP( _ ## x )

#define	__IS( x )		ICU.GRP[ GRP ## x ].BIT.IS ## x
#define	 _IS( x )		__IS( x )
#define	  IS( x , y )	_IS( _ ## x ## _ ## y )
#define	__EN( x )		ICU.GEN[ GEN ## x ].BIT.EN ## x
#define	 _EN( x )		__EN( x )
#define	  EN( x , y )	_EN( _ ## x ## _ ## y )

#define TMOCNTL SARL0
#define TMOCNTU SARU0

#define	BSC		(*(volatile struct st_bsc    __evenaccess *)0x81300)
#define	CAC		(*(volatile struct st_cac    __evenaccess *)0x8B000)
#define	CMT		(*(volatile struct st_cmt    __evenaccess *)0x88000)
#define	CMT0	(*(volatile struct st_cmt0   __evenaccess *)0x88002)
#define	CMT1	(*(volatile struct st_cmt0   __evenaccess *)0x88008)
#define	CMT2	(*(volatile struct st_cmt0   __evenaccess *)0x88012)
#define	CMT3	(*(volatile struct st_cmt0   __evenaccess *)0x88018)
#define	CRC		(*(volatile struct st_crc    __evenaccess *)0x88280)
#define	DMAC	(*(volatile struct st_dmac   __evenaccess *)0x82200)
#define	DMAC0	(*(volatile struct st_dmac0  __evenaccess *)0x82000)
#define	DMAC1	(*(volatile struct st_dmac1  __evenaccess *)0x82040)
#define	DMAC2	(*(volatile struct st_dmac1  __evenaccess *)0x82080)
#define	DMAC3	(*(volatile struct st_dmac1  __evenaccess *)0x820C0)
#define	DOC		(*(volatile struct st_doc    __evenaccess *)0x8B080)
#define	DTC		(*(volatile struct st_dtc    __evenaccess *)0x82400)
#define	FLASH	(*(volatile struct st_flash  __evenaccess *)0x8C296)
#define	GPT		(*(volatile struct st_gpt    __evenaccess *)0xC2000)
#define	GPT0	(*(volatile struct st_gpt0   __evenaccess *)0xC2100)
#define	GPT1	(*(volatile struct st_gpt0   __evenaccess *)0xC2180)
#define	GPT2	(*(volatile struct st_gpt0   __evenaccess *)0xC2200)
#define	GPT3	(*(volatile struct st_gpt0   __evenaccess *)0xC2280)
#define	ICU		(*(volatile struct st_icu    __evenaccess *)0x87000)
#define	IWDT	(*(volatile struct st_iwdt   __evenaccess *)0x88030)
#define	MPC		(*(volatile struct st_mpc    __evenaccess *)0x8C11F)
#define	MPU		(*(volatile struct st_mpu    __evenaccess *)0x86400)
#define	MTU		(*(volatile struct st_mtu    __evenaccess *)0xC120A)
#define	MTU0	(*(volatile struct st_mtu0   __evenaccess *)0xC1300)
#define	MTU1	(*(volatile struct st_mtu1   __evenaccess *)0xC1380)
#define	MTU2	(*(volatile struct st_mtu2   __evenaccess *)0xC1400)
#define	MTU3	(*(volatile struct st_mtu3   __evenaccess *)0xC1200)
#define	MTU4	(*(volatile struct st_mtu4   __evenaccess *)0xC1200)
#define	MTU5	(*(volatile struct st_mtu5   __evenaccess *)0xC1C80)
#define	MTU6	(*(volatile struct st_mtu3   __evenaccess *)0xC1A00)
#define	MTU7	(*(volatile struct st_mtu7   __evenaccess *)0xC1A00)
#define	POE		(*(volatile struct st_poe    __evenaccess *)0x8C4C0)
#define	PORT0	(*(volatile struct st_port0  __evenaccess *)0x8C000)
#define	PORT1	(*(volatile struct st_port1  __evenaccess *)0x8C001)
#define	PORT2	(*(volatile struct st_port2  __evenaccess *)0x8C002)
#define	PORT3	(*(volatile struct st_port3  __evenaccess *)0x8C003)
#define	PORT4	(*(volatile struct st_port4  __evenaccess *)0x8C044)
#define	PORT7	(*(volatile struct st_port7  __evenaccess *)0x8C007)
#define	PORT9	(*(volatile struct st_port9  __evenaccess *)0x8C009)
#define	PORTA	(*(volatile struct st_porta  __evenaccess *)0x8C00A)
#define	PORTB	(*(volatile struct st_portb  __evenaccess *)0x8C00B)
#define	PORTD	(*(volatile struct st_portd  __evenaccess *)0x8C00D)
#define	PORTE	(*(volatile struct st_porte  __evenaccess *)0x8C04E)
#define	RIIC0	(*(volatile struct st_riic   __evenaccess *)0x88300)
#define	RSPI0	(*(volatile struct st_rspi   __evenaccess *)0x88380)
#define	S12AD	(*(volatile struct st_s12ad  __evenaccess *)0x89000)
#define	SCI0	(*(volatile struct st_sci0   __evenaccess *)0x8A000)
#define	SCI1	(*(volatile struct st_sci0   __evenaccess *)0x8A020)
#define	SCI12	(*(volatile struct st_sci12  __evenaccess *)0x8B300)
#define	SMCI0	(*(volatile struct st_smci0  __evenaccess *)0x8A000)
#define	SMCI1	(*(volatile struct st_smci0  __evenaccess *)0x8A020)
#define	SMCI12	(*(volatile struct st_smci12 __evenaccess *)0x8B300)
#define	SYSTEM	(*(volatile struct st_system __evenaccess *)0x80000)
#define	WDT		(*(volatile struct st_wdt    __evenaccess *)0x88020)
#pragma bit_order
#pragma packoption
#endif