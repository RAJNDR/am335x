#include "include/am335x.h"

//four leds, 0x0 - 0xf
#define LED_PATTERN     0x1

static inline void write32(uint32_t val, const void *addr)
{
    *(volatile uint32_t *)addr = val;
}


static inline void clrbits_le32(const void* addr, uint32_t pattern)
{
    uint32_t register_value = *(volatile uint32_t *) addr;
    register_value = register_value & ~pattern;
    *(volatile uint32_t *)addr = register_value;
}


static inline void setbits_le32(const void* addr, uint32_t pattern)
{
    uint32_t register_value = *(volatile uint32_t *) addr;
    register_value = register_value | pattern;
    *(volatile uint32_t *)addr = register_value;
}


static void __attribute__((optimize("O0"))) dummy_wait(uint32_t nr_of_nops)
{
    // compiler optimizations disabled for this function using __attribute__,
    // pragma is as well possible to protect bad code...
    // http://stackoverflow.com/questions/2219829/how-to-prevent-gcc-optimizing-some-statements-in-c

    // i need this statement that the bad code below is not optimized away
    asm("");

    uint32_t counter = 0;
    for (; counter < nr_of_nops; ++counter)
    {
        ;
    }
}


static inline void init_led_output()
{
    //enable the GPIO module
    //CM_PER Registers -> CM_PER_GPIO1_CLKCTRL
    //write32((0x2 << 0) | (1 << 18), (uint32_t *)(0x44e00000 + 0xac));
	CM_PER->GPIO1_CLKCTRL = (0x2 << 0) | (1 << 18);
    //GPIO1 -> GPIO_IRQSTATUS_CLR_0 (3Ch)
    //setbits_le32(GPIO1->IRQSTATUS_CLR_0, 0xf << 21);
	GPIO1->IRQSTATUS_CLR_0 |= (0xf << 21); 
    //enable output
    //GPIO1 -> GPIO_OE (134h)
    GPIO1->OE &=  ~(0xf << 21);
}


static inline void dummy_blink_forever(void)
{
    for (;;)
    {
        //clear led gpio values to 0
        //GPIO1 -> GPIO_DATAOUT (13ch)
        //clrbits_le32(GPIO1->DATAOUT, 0xf << 21);
        GPIO1->DATAOUT &= ~(LED_PATTERN << 21);

        dummy_wait(0x002BABE5);

        //set led gpio values to 1
        //GPIO1 -> GPIO_DATAOUT (13ch)
        //setbits_le32(GPIO1->DATAOUT, LED_PATTERN << 21);
		GPIO1->DATAOUT |= (LED_PATTERN << 21);
        dummy_wait(0x002BABE5);
    }
}


static inline void heartbeat_forever(uint32_t times)
{
	uint32_t i;
    for (i = 0;i < times;i++)
    {
        //on
        GPIO1->DATAOUT |= (LED_PATTERN << 21);
        dummy_wait(0x007ABE5);

        //off
        GPIO1->DATAOUT &= ~(LED_PATTERN << 21);
        dummy_wait(0x005BBE5);

        //on
        GPIO1->DATAOUT |= (LED_PATTERN << 21);
        dummy_wait(0x003ABE5);

        //off
        GPIO1->DATAOUT &= ~(LED_PATTERN << 21);
        dummy_wait(0x0024ABE5);
    }
}

static char * cTest = "hello world";
int c_entry(void)
{
	init_led_output();
	int i;
    uint32_t	lcr=0;
    uint32_t	efr=0;
    uint32_t 	tlr=0;
    heartbeat_forever(5);
    dummy_wait(0x0024ABE5 + 0x0024ABE5);
    
    *((uint32_t *)(0x44e00400 + 0xB4)) |= 0x2;    
    while(0x2 != *((uint32_t *)(0x44e00400 + 0xB4)) & 0x2);
    while(0x1000 != *((uint32_t *)(0x44e00400)) & 0x1000);
    while(0x0<<16 != *((uint32_t *)(0x44e00400 + 0xB4)) & 0x3<<16);
	*((uint32_t *)(0x44e10000 + 0x970)) = 0x10 | 0x20;
	*((uint32_t *)(0x44e10000 + 0x974)) = 0x10;
		
	UART0->SYSC_w |= 0x2;
	while(!(UART0->SYSS & 0x1));
	
    //CM_PER->UART0
    lcr = UART0->LCR_r;
    UART0->LCR_w = 0x00BF;
    efr = (UART0->EFR_r & 0x10);
    UART0->EFR_w |= 0x10;
    UART0->LCR_w = 0x0080;
    tlr = UART0->MCR_r & 0xFF40;
    UART0->MCR_w |= 0x40;
    UART0->FCR_w = 0;
    UART0->EFR_w = 0x00BF;
    UART0->SPR_TLR_w &= 0xFF00;
    UART0->SCR_w &= 0xFF00;
    UART0->EFR_w |= efr;
    UART0->LCR_w = 0x0080;
    UART0->MCR_w |= tlr;
    UART0->LCR_w = lcr;
    
    UART0->MDR1_w |= 0x7;
    UART0->LCR_w = 0x00BF;
    efr = UART0->EFR_r & (0x10);
    UART0->EFR_w |= 0x10;
    
    UART0->LCR_w = 0x0000;
	UART0->IER_w = 0x0000;
	
	UART0->LCR_w = 0x00BF;
	UART0->DLL_w &= 0xFF1A;
	UART0->DLL_w &= 0xFFF0;
	UART0->LCR_w = 0x0000;
	UART0->IER_w &= 0xFFF0;
	
	UART0->LCR_w = 0x00BF;
	UART0->EFR_w &= (0xFFFF & efr);
    UART0->LCR_w = 0x0003;
	UART0->MDR1_w |= 0x0;
	
	UART0->LCR_w &= 0x7F;
	for(i =0 ; i<5; i++)
	{
	 heartbeat_forever(1);
	 
	 while(!(UART0->LSR_r & 0x40));
		UART0->THR = 'a';
	}
    
    //heartbeat_forever();
	dummy_blink_forever();
    return 0;
}
