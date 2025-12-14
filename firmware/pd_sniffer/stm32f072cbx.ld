/* Linker script for the STM32F072CB */

/* Memory regions */
MEMORY {
  rom (rx) : ORIGIN = 0x08000000, LENGTH = 128K
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Include the common libopencm3 linker script */
INCLUDE cortex-m-generic.ld
/* Firmware entry point*/
ENTRY(reset_handler)
/*
 * The libopencm3 'common-template.ld' file expects these variables to be defined:
 *    * __stack_size
 *    * __heap_size
 *    * __flash_start
 *    * __flash_len
 *    * __ram_start
 *    * __ram_len
 */