/*
 * (C) Copyright 2013
 * Andrey Mitrofanov, avmwww@gmail.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm",
	      "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)

#define NVM_BASE	(CONFIG_MEM_NVM_BASE + CONFIG_MEM_NVM_UBOOT_OFF)
#define NVM_LEN		(CONFIG_MEM_NVM_LEN - CONFIG_MEM_NVM_UBOOT_OFF)
#define RAM_BUF_BASE	(CONFIG_MEM_RAM_BASE + CONFIG_MEM_RAM_LEN)
#define MALLOC_BASE	(RAM_BUF_BASE + CONFIG_MEM_RAM_BUF_LEN)
#define STACK_BASE	(CONFIG_SRAM_BASE)

MEMORY
{
	NVM (r):	ORIGIN = NVM_BASE, \
			LENGTH = NVM_LEN
	RAM (rw):	ORIGIN = CONFIG_MEM_RAM_BASE, \
			LENGTH = CONFIG_MEM_RAM_LEN
	RAM_BUF (r):	ORIGIN = RAM_BUF_BASE, \
			LENGTH = CONFIG_MEM_RAM_BUF_LEN
	MALLOC (r):	ORIGIN = MALLOC_BASE, \
			LENGTH = CONFIG_MEM_MALLOC_LEN
	STACK (r):	ORIGIN = STACK_BASE, \
			LENGTH = CONFIG_MEM_STACK_LEN
}

SECTIONS
{
	.vectors NVM_BASE :
	{
		*(.vectors)
	} >NVM

	.text :
	{
		_text_start = .;
		*(.text)
		*(.rodata*)
		_text_end = .;

	} >NVM

	.data :
	{
		_data_start = .;
		_data_lma_start = LOADADDR(.data);
		*(.data)
		. = ALIGN(4);
		*(.ramcode)
		_data_end = .;
	} >RAM AT>NVM

	.u_boot_cmd :
	{
		. = ALIGN(4);
		__u_boot_cmd_start = .;
		*(.u_boot_cmd)
		__u_boot_cmd_end = .;
	} >NVM

	.ARM.extab : {
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} >NVM

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} >NVM
	__exidx_end = .;

	.bss :
	{
		_bss_start = .;
		*(.bss)
		*(COMMON)
		_bss_end = .;
	} >RAM

	.stack :
	{
		*(.stack)
	} >STACK

	/DISCARD/ :
	{
		*(*)
	}
}

/*
 * Export memory addresses
 */
_mem_nvm_base		= ORIGIN(NVM);
_mem_nvm_size		= LENGTH(NVM);
_mem_ram_base		= ORIGIN(RAM);
_mem_ram_size		= LENGTH(RAM);
_mem_ram_buf_base	= ORIGIN(RAM_BUF);
_mem_ram_buf_size	= LENGTH(RAM_BUF);
_mem_malloc_base	= ORIGIN(MALLOC);
_mem_malloc_size	= LENGTH(MALLOC);
_mem_stack_base		= ORIGIN(STACK);
_mem_stack_size		= LENGTH(STACK);
_mem_stack_end		= ORIGIN(STACK) + LENGTH(STACK);
