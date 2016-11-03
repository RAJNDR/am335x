#ifndef __AM335X_H
#define __AM335X_H

#ifdef __cplusplus
	extern "C" {
#endif

//#include "soc_am335x.h"
//#include "hw_control_am335x.h"
#include <stdint.h>

#ifndef __GNUC__
	#define __GNUC__
#endif

/*#define ASMNAME(x) ASMNAME_(__USER_LABEL_PREFIX__, #x)
#define ASMNAME_(x,y) ASMNAME__(x, y)
#define ASMNAME__(x,y) __asm__(#x y)*/


#define __IO volatile
#define __I volatile const

typedef struct {
	union   {
	__IO uint32_t	THR;
	__I uint32_t 	RHR;
	__I uint32_t	DLL_r;
	__IO uint32_t	DLL_w;
	};
	union   {
	__IO uint32_t	IER_w;
	__I uint32_t 	IER_r;
	__I uint32_t	DLH_r;
	__IO uint32_t	DLH_w;
	};
	union   {
	__IO uint32_t	FCR_w;
	__I uint32_t 	IIR_r;
	__I uint32_t	EFR_r;
	__IO uint32_t	EFR_w;
	};
	union   {
	__IO uint32_t	LCR_w;
	__I uint32_t 	LCR_r;
	};
	union   {
	__IO uint32_t	MCR_w;
	__I uint32_t 	MCR_r;
	__I uint32_t	XON1_ADDR1_r;
	__IO uint32_t	XON1_ADDR1_w;
	};
	union   {
	__I uint32_t 	LSR_r;
	__I uint32_t	XON2_ADDR2_r;
	__IO uint32_t	XON2_ADDR2_w;
	};
	union   {
	__IO uint32_t	TCR_w;
	__I uint32_t 	MSR_TCR_r;
	__I uint32_t	XOFF1_TCR_r;
	__IO uint32_t	XOFF1_TCR_w;
	};
	union   {
	__IO uint32_t	SPR_TLR_w;
	__I uint32_t 	SPR_TLR_r;
	__I uint32_t	XOFF2_TLR_r;
	__IO uint32_t	XOFF2_TLR_w;
	};
	union   {
	__I uint32_t	MDR1_r;
	__IO uint32_t	MDR1_w;
	};
	union   {
	__I uint32_t	MDR2_r;
	__IO uint32_t	MDR2_w;
	};
	union   {
	__IO uint32_t	TXFLL_w;
	__I uint32_t 	SFLSR_r;
	};
	union   {
	__IO uint32_t	TXFLH_w;
	__I uint32_t 	RESUME_r;
	};
	union   {
	__IO uint32_t	RXFLL_w;
	__I uint32_t 	SFREGL_r;
	};
	union   {
	__IO uint32_t	RXFLH_w;
	__I uint32_t 	SFREGH_r;
	};
	union   {
	__IO uint32_t	BLR_w;
	__I uint32_t 	BLR_r;
	__I uint32_t	UASR;
	};
	union   {
	__IO uint32_t	ACREG_w;
	__I uint32_t 	ACREG_r;
	};
	union   {
	__IO uint32_t	SCR_w;
	__I uint32_t 	SCR_r;
	};
	union   {
	__IO uint32_t	SSR_w;
	__I uint32_t 	SSR_r;
	};
	union   {
	__IO uint32_t	EBLR_w;
	__I uint32_t 	EBLR_r;
	};
		 uint32_t	RESERVED0;
	__I uint32_t	MVR;
	union   {
	__IO uint32_t	SYSC_w;
	__I uint32_t 	SYSC_r;
	};
	__I uint32_t	SYSS;
	union   {
	__IO uint32_t	WER_w;
	__I uint32_t 	WER_r;
	};
	union   {
	__IO uint32_t	CFPS_w;
	__I uint32_t 	CFPS_r;
	};
	union   {
	__IO uint32_t	RXFIFO_LVL_w;
	__I uint32_t 	RXFIFO_LVL_r;
	};
	union   {
	__IO uint32_t	TXFIFO_LVL_w;
	__I uint32_t 	TXFIFO_LVL_r;
	};
	union   {
	__IO uint32_t	IER2_w;
	__I uint32_t 	IER2_r;
	};
	union   {
	__IO uint32_t	ISR2_w;
	__I uint32_t 	ISR2_r;
	};
	union   {
	__IO uint32_t	FREQ_SEL_w;
	__I uint32_t 	FREQ_SEL_r;
	};
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	union   {
	__IO uint32_t	MDR3_w;
	__I uint32_t 	MDR3_r;
	};
	union   {
	__IO uint32_t	TXDMA_w;
	__I uint32_t 	TXDMA_r;
	};
} UART_typedef;


/*------GPIO structure typedef------*/
typedef struct {
	__IO uint32_t 	REVISION;				/*address 0h*/
		 uint8_t 	RESERVED1[12];			/*address 4h to Fh*/
	__IO uint32_t	GPIO_SYSCONFIG;			/*address 10h*/
		 uint8_t	RESERVED2[12];			/*address 14h to 1F*/
	__IO uint32_t	EOI;
	__IO uint32_t	IRQSTATUS_RAW_0;
	__IO uint32_t	IRQSTATUS_RAW_1;
	__IO uint32_t	IRQSTATUS_0;
	__IO uint32_t	IRQSTATUS_1;
	__IO uint32_t	IRQSTATUS_SET_0;
	__IO uint32_t	IRQSTATUS_SET_1;
	__IO uint32_t	IRQSTATUS_CLR_0;
	__IO uint32_t	IRQSTATUS_CLR_1;
	__IO uint32_t	IRQWAKEN_0;
	__IO uint32_t	IRQWAKEN_1;
		 uint8_t	RESERVED3[200];
	__IO uint32_t	SYSSTATUS;
		 uint8_t	RESERVED4[24];
	__IO uint32_t	CTRL;
	__IO uint32_t	OE;
	__IO uint32_t	DATAIN;
	__IO uint32_t	DATAOUT;
	__IO uint32_t	LEVELDETECT0;
	__IO uint32_t	LEVELDETECT1;
	__IO uint32_t	RISINGDETECT;
	__IO uint32_t	FALLINGDETECT;
	__IO uint32_t	DEBOUNCENABLE;
	__IO uint32_t	DEBOUNCEINGTIME;
		 uint8_t	RESERVED5[56];
	__IO uint32_t	CLEARDATAOUT;
	__IO uint32_t	SETDATAOUT;
}GPIO_Typedef;

/*------CM_PER structure typedef --------*/

typedef struct {
	__IO uint32_t	L4LS_CLKSTCTRL;
	__IO uint32_t	L3S_CLKSTCTRL;
	__IO uint32_t	L4FW_CLKSTCTRL;
	__IO uint32_t	L3_CLKSTCTRL;
		 uint32_t	RESERVED1;
	__IO uint32_t	CPGMAC0_CLKCTRL;
	__IO uint32_t	LCDC_CLKCTRL;
	__IO uint32_t	USB0_CLKCTRL;
	__IO uint32_t	RESERVED2;
	__IO uint32_t	TPTC0_CLKCTRL;
	__IO uint32_t	EMIF_CLKCTRL;
	__IO uint32_t	OCMCRAM_CLKCTRL;
	__IO uint32_t	GPMC_CLKCTRL;
	__IO uint32_t	MCASO0_CLKCTRL;
	__IO uint32_t	UART5_CLKCTRL;
	__IO uint32_t	MMC0_CLKCTRL;
	__IO uint32_t	ELM_CLKCTRL;
	__IO uint32_t	I2C2_CLKCTRL;
	__IO uint32_t	I2C1_CLKCTRL;
	__IO uint32_t	SP0_CLKCTRL;
	__IO uint32_t	SP1_CLKCTRL;
		 uint8_t	RESERVED3[12];
	__IO uint32_t	L4LS_CLKCTRL;
	__IO uint32_t	L4FW_CLKCTRL;
	__IO uint32_t	MCASP1_CLKCTRL;
	__IO uint32_t	UART1_CLKCTRL;
	__IO uint32_t	UART2_CLKCTRL;
	__IO uint32_t	UART3_CLKCTRL;
	__IO uint32_t	UART4_CLKCTRL;
	__IO uint32_t	TIMER7_CLKCTRL;
	__IO uint32_t	TIMER2_CLKCTRL;
	__IO uint32_t	TIMER3_CLKCTRL;
	__IO uint32_t	TIMER4_CLKCTRL;
		 uint8_t	RESERVED4[32];
	__IO uint32_t	GPIO1_CLKCTRL;
	__IO uint32_t	GPIO2_CLKCTRL;
	__IO uint32_t	GPIO3_CLKCTRL;
		 uint32_t	RESERVED5;
	__IO uint32_t	TPCC_CLKCTRL;
	__IO uint32_t	DCAN0_CLKCTRL;
	__IO uint32_t	DCAN1_CLKCTRL;
		 uint32_t	RESERVED6;
	__IO uint32_t	EPWMSS1_CLKCTRL;
	__IO uint32_t	EMIF_FW_CLKCTRL;
	__IO uint32_t	EPWMSS0_CLKCTRL;
	__IO uint32_t	EPWMSS2_CLKCTRL;
	__IO uint32_t	L3_INSTR_CLKCTRL;
	__IO uint32_t	L3_CLKCTRL;
	__IO uint32_t	IEEE5000_CLKCTRL;
	__IO uint32_t	PRU_ICSS_CLKCTRL;
	__IO uint32_t	TIMER5_CLKCTRL;
	__IO uint32_t	TIMER6_CLKCTRL;
	__IO uint32_t	MMC1_CLKCTRL;
	__IO uint32_t	MMC2_CLKCTRL;
	__IO uint32_t	TPTC1_CLKCTRL;
	__IO uint32_t	TPTC2_CLKCTRL;
		 uint8_t	RESERVED7[8];
	__IO uint32_t	SPINLOCK_CLKCTRL;
	__IO uint32_t	MAILBOX0_CLKCTRL;
		 uint8_t	RESERVED8[8];
	__IO uint32_t	L4HS_CLKSTCTRL;
	__IO uint32_t	L4HS_CLKCTRL;
		 uint8_t	RESERVED9[8];
	__IO uint32_t	OCPWP_L3_CLKSTCTRL;
	__IO uint32_t	OCPWP_CLKCTRL;
		 uint8_t	RESERVED10[12];
	__IO uint32_t	PRU_ICSS_CLKSTCTRL;
	__IO uint32_t	CPSW_CLKSTCTRL;
	__IO uint32_t	LCDC_CLKSTCTRL;	 
	__IO uint32_t	CLKDIV32K_CLKCTRL;
	__IO uint32_t	CLK_24MHZ_CLKSTCTRL;
}CM_PER_Typedef;

typedef struct {
	__IO uint32_t	CLKSTCTRL;
	__IO uint32_t	CONTROL_CLKCTRL;
	__IO uint32_t	GPIO0_CLKCTRL;
	__IO uint32_t	L4WKUP_CLKCTRL;
	__IO uint32_t	TIMER0_CLKCTRL;
	__IO uint32_t	DEBUGSS_CLKCTRL;
	__IO uint32_t	CM_L3_AON_CLKSTCTRL;
	__IO uint32_t	CM_AUTOIDLE_DPLL_MPU;
	__IO uint32_t	CM_IDLEST_DPLL_MPU;
	__IO uint32_t	CM_SSC_DELTAMSTEP_DPLL_MPU;
	__IO uint32_t	CM_SSC_MODFREQDIV_DPLL_MPU;
	__IO uint32_t	CM_CLKSEL_DPLL_MPU;
	__IO uint32_t	CM_AUTOIDLE_DPLL_DDR;
	__IO uint32_t	CM_IDLEST_DPLL_DDR;
	__IO uint32_t	CM_SSC_DELTAMSTEP_DPLL_DDR;
	__IO uint32_t	CM_SSC_MODFREQDIV_DPLL_DDR;
	__IO uint32_t	CM_CLKSEL_DPLL_DDR;
	__IO uint32_t	CM_AUTOIDLE_DPLL_DISP;
	__IO uint32_t	CM_IDLEST_DPLL_DISP;
	__IO uint32_t	CM_SSC_DELTAMSTEP_DPLL_DISP;
	__IO uint32_t	CM_SSC_MODFREQDIV_DPLL_DISP;
	__IO uint32_t	CM_CLKSEL_DPLL_DISP;
	__IO uint32_t	CM_AUTOIDLE_DPLL_CORE;
	__IO uint32_t	CM_IDLEST_DPLL_CORE;
	__IO uint32_t	CM_SSC_DELTAMSTEP_DPLL_CORE;
	__IO uint32_t	CM_SSC_MODFREQDIV_DPLL_CORE;
	__IO uint32_t	CM_CLKSEL_DPLL_CORE;
	__IO uint32_t	CM_AUTOIDLE_DPLL_PER;
	__IO uint32_t	CM_IDLEST_DPLL_PER;
	__IO uint32_t	CM_SSC_DELTAMSTEP_DPLL_PER;
	__IO uint32_t	CM_SSC_MODFREQDIV_DPLL_PER;
	__IO uint32_t	CM_CLKDCOLDO_DPLL_PER;
	__IO uint32_t	CM_DIV_M4_DPLL_CORE;
	__IO uint32_t	CM_DIV_M5_DPLL_CORE;
	__IO uint32_t	CM_CLKMODE_DPLL_MPU;
	__IO uint32_t	CM_CLKMODE_DPLL_PER;
	__IO uint32_t	CM_CLKMODE_DPLL_CORE;
	__IO uint32_t	CM_CLKMODE_DPLL_DDR;
	__IO uint32_t	CM_CLKMODE_DPLL_DISP;
	__IO uint32_t	CM_CLKSEL_DPLL_PERIPH;
	__IO uint32_t	CM_DIV_M2_DPLL_DDR;
	__IO uint32_t	CM_DIV_M2_DPLL_DISP;
	__IO uint32_t	CM_DIV_M2_DPLL_MPU;
	__IO uint32_t	CM_DIV_M2_DPLL_PER;
	__IO uint32_t	WKUP_M3_CLKCTRL;
	__IO uint32_t	UART0_CLKCTRL;
	__IO uint32_t	I2C0_CLKCTRL;
	__IO uint32_t	ADC_TSC_CLKCTRL;
	__IO uint32_t	SMARTREFLEX0_CLKCTRL;
	__IO uint32_t	TIMER1_CLKCTRL;
	__IO uint32_t	SMARTREFLEX1_CLKCTRL;
	__IO uint32_t	CM_L4_WKUP_AON_CLKSTCTRL;
		 uint32_t	RESERVED0;					//Reserved in AM335x_Techical_reference_manual
	__IO uint32_t	WDT1_CLKCTRL;
	__IO uint32_t	CM_DIV_M6_DPLL_CORE;
}CM_WKUP_Typdef;	

/****************************************/
#define CM_PER_BASE ((uint32_t) 0x44E00000)
#define CM_WKUP_BASE ((uint32_t 0x44E00400)
#define CONTROL_BASE ((uint32_t 0x44E10000)
/**************Peripherial****************/
#define UART0_BASE	((uint32_t) 0x44E09000)
#define UART1_BASE	((uint32_t) 0x48022000)
#define UART2_BASE	((uint32_t) 0x48024000)
#define UART3_BASE	((uint32_t) 0x481A6000)
#define UART4_BASE	((uint32_t) 0x481A8000)
#define UART5_BASE	((uint32_t) 0x481AC000)
/***************GPIO**********************/
#define GPIO0_BASE 	((uint32_t) 0x44E07000)
#define GPIO1_BASE 	((uint32_t) 0x4804C000)
#define GPIO2_BASE 	((uint32_t) 0x481AC000)
#define GPIO3_BASE 	((uint32_t) 0x481AE000)

/***********structure pointers************/
#define CM_PER		((CM_PER_Typedef *) CM_PER_BASE)
#define CM_WKUP		((CM_WKUP_Typdef *) CM_WKUP_BASE)

#define GPIO0		((GPIO_Typedef *)	GPIO0_BASE)
#define GPIO1		((GPIO_Typedef *)	GPIO1_BASE)
#define GPIO2		((GPIO_Typedef *)	GPIO2_BASE)
#define GPIO3		((GPIO_Typedef *)	GPIO3_BASE)

#define UART0		((UART_typedef *)	UART0_BASE)
#define UART1		((UART_typedef *)	UART1_BASE)
#define UART2		((UART_typedef *)	UART2_BASE)
#define UART3		((UART_typedef *)	UART3_BASE)
#define UART4		((UART_typedef *)	UART4_BASE)
#define UART5		((UART_typedef *)	UART5_BASE)

#ifdef __cplusplus
	}
#endif

#endif
