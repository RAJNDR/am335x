ENTRY(_asm_entry) /* entry point of the program, first instruction executed */
SECTIONS
{
 . = 0x402F0400;
 .text : { _text_start_ = .;
 	   startup.o (INTERRUPT_VECTOR)
 	  *(.text)
	   _text_end = .;
 }
 .data : { _data_start = .;
	   *(.data) 
	   _data_end = .;
} /* global and static variables initialized by the application code */

 .bss : {_bss_start = .;
	 *(.bss COMMON) 
	 _bss_end = .;} /* global and static variable that are not initialized by the application code. the compiler initialize this data to 0. */
 . = ALIGN(8);
 . = . + 0x1000; /* 4kB of stack memory */
 stack_top = .; /* stack grows upwards */
}
