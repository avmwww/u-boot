/*
 * (C) Copyright 2013
 * Andrey Mitrofanov, <avmwww@gmail.com>
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

/*
 * Board specific code for the TPM-011 board
 */

#include <common.h>
#include <netdev.h>

#include <asm/arch/mdr32.h>

struct mdr32_clk {
	u32 CLOCK_STATUS;
	u32 PLL_CONTROL;
	u32 HS_CONTROL;
	u32 CPU_CLOCK;
	u32 USB_CLOCK;
	u32 ADC_MCO_CLOCK;
	u32 RTC_CLOCK;
	u32 PER_CLOCK;
	u32 CAN_CLOCK;
	u32 TIM_CLOCK;
	u32 UART_CLOCK;
	u32 SSP_CLOCK;
};

struct mdr32_port {
	u32 RXTX;
	u32 OE;
	u32 FUNC;
	u32 ANALOG;
	u32 PULL;
	u32 PD;
	u32 PWR;
	u32 GFEN;
};

#define RST_CLK_BASE				0x40020000
#define RST_CLK					((volatile struct mdr32_clk *) RST_CLK_BASE)
#define RST_CLK_CLOCK_STATUS_PLL_USB_RDY	0x1
#define RST_CLK_CLOCK_STATUS_PLL_CPU_RDY	0x2
#define RST_CLK_CLOCK_STATUS_HSE_RDY		0x4

#define RST_CLK_PLL_CONTROL_PLL_USB_ON		0x1
#define RST_CLK_PLL_CONTROL_PLL_USB_RLD		0x2
#define RST_CLK_PLL_CONTROL_PLL_CPU_ON		0x4
#define RST_CLK_PLL_CONTROL_PLL_CPU_PLD		0x8
#define RST_CLK_PLL_CONTROL_PLL_USB_MUL(x)	(((x) & 0x0f) << 4)
#define RST_CLK_PLL_CONTROL_PLL_CPU_MUL(x)	(((x) & 0x0f) << 8)

#define RST_CLK_HS_CONTROL_HSE_ON		0x1
#define RST_CLK_HS_CONTROL_HSE_BYP		0x2

#define RST_CLK_CPU_CLOCK_CPU_C1_SEL(x)		((x) & 0x03)
#define RST_CLK_CPU_CLOCK_CPU_C2_SEL		0x04
#define RST_CLK_CPU_CLOCK_CPU_C3_SEL		(((x) & 0x0f) << 4)
#define RST_CLK_CPU_CLOCK_HCLK_SEL(x)		(((x) & 0x03) << 8)

#define PORTA		((volatile struct mdr32_port *) PORTA_BASE)
#define PORTB		((volatile struct mdr32_port *) PORTB_BASE)
#define PORTC		((volatile struct mdr32_port *) PORTC_BASE)
#define PORTD		((volatile struct mdr32_port *) PORTD_BASE)
#define PORTE		((volatile struct mdr32_port *) PORTE_BASE)
#define PORTF		((volatile struct mdr32_port *) PORTF_BASE)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_ROM		0x1
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_RAM		0x2
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_NAND		0x4
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_CPOL		0x8
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_BUSY		0x80
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_WAIT_STATE(x)	(((x) & 0x0f) << 12)

struct mdr32_ext_bus_control {
	u32 RESERVED0[20];
	u32 NAND_CYCLES;
	u32 EXT_BUS_CONTROL;
};

struct mdr32_eeprom {
	u32 CMD;
	u32 ADR;
	u32 DI;
	u32 DO;
	u32 KEY;
};

#define EEPROM_BASE		0x40018000
#define EEPROM			((volatile struct mdr32_eeprom *) EEPROM_BASE)
#define EEPROM_CMD_Delay(x)	(((x) & 0x07) << 3)
#define EXT_BUS_CNTRL_BASE	0x400F0000
#define EXT_BUS_CNTRL		((volatile struct mdr32_ext_bus_control*)EXT_BUS_CNTRL_BASE)

DECLARE_GLOBAL_DATA_PTR;

/*
 * Early hardware init.
 */
int board_init(void)
{
	return 0;
}

void cpu_init_crit(void)
{
	/* Enable HSE generator. */
	RST_CLK->HS_CONTROL = RST_CLK_HS_CONTROL_HSE_ON;
	while (!(RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY))
		continue;

	/* Set up EEPROM waitstates */
#define SET_EEPROM_DELAY(w)	\
	do {			\
		EEPROM->CMD = (EEPROM->CMD & ~EEPROM_CMD_Delay(0)) | EEPROM_CMD_Delay(w);\
	} while (0)
#if (CONFIG_SYS_CPUCLOCK < 25000000)
	SET_EEPROM_DELAY(0);
#elif (CONFIG_SYS_CPUCLOCK < 50000000)
	SET_EEPROM_DELAY(1);
#elif (CONFIG_SYS_CPUCLOCK < 75000000)
	SET_EEPROM_DELAY(2);
#endif

	/* Enable clock to watchdog timer */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_IWDT;

	/* Use HSE for CPU_C1 clock. */
	RST_CLK->CPU_CLOCK = RST_CLK_CPU_CLOCK_CPU_C1_SEL(2);

	/* Setup PLL for CPU. */
	RST_CLK->PLL_CONTROL = RST_CLK_PLL_CONTROL_PLL_CPU_MUL(CONFIG_MDR32_PLL_M - 1);
	RST_CLK->PLL_CONTROL = RST_CLK_PLL_CONTROL_PLL_CPU_MUL(CONFIG_MDR32_PLL_M - 1) |
		RST_CLK_PLL_CONTROL_PLL_CPU_ON;
	RST_CLK->PLL_CONTROL = RST_CLK_PLL_CONTROL_PLL_CPU_MUL(CONFIG_MDR32_PLL_M - 1) |
		RST_CLK_PLL_CONTROL_PLL_CPU_ON | RST_CLK_PLL_CONTROL_PLL_USB_RLD;
	RST_CLK->PLL_CONTROL = RST_CLK_PLL_CONTROL_PLL_CPU_MUL(CONFIG_MDR32_PLL_M - 1) |
		RST_CLK_PLL_CONTROL_PLL_CPU_ON;
	while (! (RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY))
		continue;

	/* Use PLLCPUo for CPU_C2, CPU_C3 and HCLK. */
	RST_CLK->CPU_CLOCK = RST_CLK_CPU_CLOCK_CPU_C2_SEL |
		RST_CLK_CPU_CLOCK_CPU_C1_SEL(2) |
		RST_CLK_CPU_CLOCK_HCLK_SEL(1);


	/* Setup external RAM */
	RST_CLK->PER_CLOCK |= \
				RST_CLK_PER_CLOCK_PORTA | \
				RST_CLK_PER_CLOCK_PORTB | \
				RST_CLK_PER_CLOCK_PORTC | \
				RST_CLK_PER_CLOCK_PORTD | \
				RST_CLK_PER_CLOCK_PORTE | \
				RST_CLK_PER_CLOCK_PORTF;
	/* Set funct and power, digital */
	#define SETUP_PORT(PORT,F,A,P) (PORT->FUNC = F, PORT->ANALOG = A, PORT->PWR = P)
	#define SETUP_PORT_MAIN(PORT) SETUP_PORT(PORT,0x55555555,0xFFFF,0xFFFFFFFF)
	SETUP_PORT_MAIN(PORTA);
	SETUP_PORT_MAIN(PORTB);
	SETUP_PORT(PORTC,0xAA001554,0xFC7E,0xF0F03FFC);
	PORTC->OE = 0x0C00;
	PORTC->RXTX = 0x0000;
	SETUP_PORT(PORTD,0xC3FFE800,0x9FFF,0x03FFFC00);
	SETUP_PORT(PORTE,0x55555555,0xFFFF,0xFFFFFFFF);
	SETUP_PORT(PORTF,0x5555555F,0xFFFF,0xFFFFFFFF);

	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_EXT_BUS;
	EXT_BUS_CNTRL->EXT_BUS_CONTROL=EXT_BUS_CNTRL_EXT_BUS_CONTROL_RAM | \
		 EXT_BUS_CNTRL_EXT_BUS_CONTROL_WAIT_STATE(0x0f);
	/* Enable clock to watchdog timer */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_IWDT;
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PORTF;

	/* Redefine portf function */
	PORTF->FUNC |= 0x0f;
	/* PF0(UART2_RXD) and  PF1(UART2_TXD) digital pins */
	PORTF->ANALOG |= 3;
	PORTF->PWR &= ~0x0f;
	PORTF->PWR |= 0x05;


#if 0
	int rv;

#if !defined(CONFIG_SYS_NO_FLASH)
	if ((rv = fsmc_nor_psram_init(CONFIG_SYS_FLASH_CS, CONFIG_SYS_FSMC_FLASH_BCR,
			CONFIG_SYS_FSMC_FLASH_BTR,
			CONFIG_SYS_FSMC_FLASH_BWTR)))
		return rv;
#endif

#if defined(CONFIG_LCD)
	/*
	 * Configure FSMC for accessing the LCD controller
	 */
	if ((rv = fsmc_nor_psram_init(CONFIG_LCD_CS, CONFIG_LCD_FSMC_BCR,
			CONFIG_LCD_FSMC_BTR, CONFIG_LCD_FSMC_BWTR)))
		return rv;

	gd->fb_base = CONFIG_FB_ADDR;
#endif
#endif
}

/*
 * Dump pertinent info to the console.
 */
int checkboard(void)
{
	printf("Board: TPM-011 board, %s\n",
		CONFIG_SYS_BOARD_REV_STR);

	return 0;
}

/*
 * Setup external RAM.
 */
int dram_init(void)
{
	int				rv = 0;

#if 0
	static struct stm32f2_gpio_dsc	ctrl_gpio = {STM32F2_GPIO_PORT_I,
						     STM32F2_GPIO_PIN_9};

	rv = fsmc_nor_psram_init(CONFIG_SYS_RAM_CS,
			CONFIG_SYS_FSMC_PSRAM_BCR,
			CONFIG_SYS_FSMC_PSRAM_BTR,
#ifdef CONFIG_SYS_FSMC_PSRAM_BWTR
			CONFIG_SYS_FSMC_PSRAM_BWTR
#else
			(u32)-1
#endif
		);
	if (rv != 0)
		goto out;

	rv = stm32f2_gpio_config(&ctrl_gpio, STM32F2_GPIO_ROLE_GPOUT);
	if (rv != 0)
		goto out;

# if defined(CONFIG_SYS_RAM_BURST)
	/*
	 * FIXME: all this hardcoded stuff.
	 */

	/* Step.2 */
	stm32f2_gpout_set(&ctrl_gpio, 1);

	/* Step.3 */
	*(volatile u16 *)(CONFIG_SYS_RAM_BASE + 0x0010223E) = 0;

	/* Step.4-5 */
	stm32f2_gpout_set(&ctrl_gpio, 0);

	/* Step.6 */
	fsmc_nor_psram_init(CONFIG_SYS_RAM_CS, 0x00083115,
			0x0010FFFF, -1);

	/* Step.7 */
	rv = *(volatile u16 *)(CONFIG_SYS_RAM_BASE + 0x000000);

	/* Step.8 */
	fsmc_nor_psram_init(CONFIG_SYS_RAM_CS, 0x00005059,
			0x10000702, 0x10000602);

	/* Step.9 */
	stm32f2_gpout_set(&ctrl_gpio, 1);

	/* Step.10 */
	*(volatile u16 *)(CONFIG_SYS_RAM_BASE + 0x0110223E) = 0;

	/* Step.11 */
	stm32f2_gpout_set(&ctrl_gpio, 0);

	/* Step.12 */
	fsmc_nor_psram_init(CONFIG_SYS_RAM_CS, 0x00083115,
			0x0010FFFF, -1);

	/* Step.13 */
	rv = *(volatile u16 *)(CONFIG_SYS_RAM_BASE + 0x01000000);

# else
	/*
	 * Switch PSRAM in the Asyncronous Read/Write Mode
	 */
	stm32f2_gpout_set(&ctrl_gpio, 0);
# endif /* CONFIG_SYS_RAM_BURST */

#endif
	/*
	 * Fill in global info with description of SRAM configuration
	 */
	gd->bd->bi_dram[0].start = CONFIG_SYS_RAM_BASE;
	gd->bd->bi_dram[0].size  = CONFIG_SYS_RAM_SIZE;

	rv = 0;
out:
	return rv;
}

#ifdef CONFIG_K5600BG1_ETH
/*
 * Register ethernet driver
 */
int board_eth_init(bd_t *bis)
{
	return k5600bg1_eth_init(bis, CONFIG_K5600BG1_BASE_ADDR);
}
#endif

/*
 * Initialize internal Flash interface
 */
void envm_init(void)
{
}

/*
 * Write a data buffer to internal Flash.
 * Note that we need for this function to reside in RAM since it
 * will be used to self-upgrade U-boot in internal Flash.
 */
envm_write(u32 offset, void * buf, u32 size)
{
	s32 ret = 0;

	return ret;
}
