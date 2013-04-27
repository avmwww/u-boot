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
 * Board specific code for the STMicroelectronic STM3220G-EVAL board
 */

#include <common.h>
#include <netdev.h>
#include <ili932x.h>

#include <asm/arch/mdr32.h>
#include <asm/arch/stm32f2_gpio.h>
#include <asm/arch/fsmc.h>

#define RST_CLK_BASE        0x40020000

#define     __IO    volatile

typedef struct
{
  __IO uint32_t CLOCK_STATUS;
  __IO uint32_t PLL_CONTROL;
  __IO uint32_t HS_CONTROL;
  __IO uint32_t CPU_CLOCK;
  __IO uint32_t USB_CLOCK;
  __IO uint32_t ADC_MCO_CLOCK;
  __IO uint32_t RTC_CLOCK;
  __IO uint32_t PER_CLOCK;
  __IO uint32_t CAN_CLOCK;
  __IO uint32_t TIM_CLOCK;
  __IO uint32_t UART_CLOCK;
  __IO uint32_t SSP_CLOCK;
}RST_CLK_TypeDef;

typedef struct
{
  __IO uint32_t RXTX;
  __IO uint32_t OE;
  __IO uint32_t FUNC;
  __IO uint32_t ANALOG;
  __IO uint32_t PULL;
  __IO uint32_t PD;
  __IO uint32_t PWR;
  __IO uint32_t GFEN;
}PORT_TypeDef;


#define RST_CLK             ((RST_CLK_TypeDef*) RST_CLK_BASE)

#define RST_CLK_CLOCK_STATUS_PLL_USB_RDY_Pos    0
#define RST_CLK_CLOCK_STATUS_PLL_USB_RDY        ((uint32_t)0x00000001)

#define RST_CLK_CLOCK_STATUS_PLL_CPU_RDY_Pos    1
#define RST_CLK_CLOCK_STATUS_PLL_CPU_RDY        ((uint32_t)0x00000002)

#define RST_CLK_CLOCK_STATUS_HSE_RDY_Pos        2
#define RST_CLK_CLOCK_STATUS_HSE_RDY            ((uint32_t)0x00000004)


/** @} */ /* End of group Periph_RST_CLK_RST_CLK_CLOCK_STATUS_Bits */

/** @} */ /* End of group Periph_RST_CLK_Defines */

/** @defgroup Periph_RST_CLK_Defines Defines
  * @{
  */

/** @defgroup Periph_RST_CLK_RST_CLK_PLL_CONTROL_Bits RST_CLK_PLL_CONTROL
  * @{
  */

#define RST_CLK_PLL_CONTROL_PLL_USB_ON_Pos      0
#define RST_CLK_PLL_CONTROL_PLL_USB_ON          ((uint32_t)0x00000001)

#define RST_CLK_PLL_CONTROL_PLL_USB_RLD_Pos     1
#define RST_CLK_PLL_CONTROL_PLL_USB_RLD         ((uint32_t)0x00000002)

#define RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos      2
#define RST_CLK_PLL_CONTROL_PLL_CPU_ON          ((uint32_t)0x00000004)

#define RST_CLK_PLL_CONTROL_PLL_CPU_PLD_Pos     3
#define RST_CLK_PLL_CONTROL_PLL_CPU_PLD         ((uint32_t)0x00000008)

#define RST_CLK_PLL_CONTROL_PLL_USB_MUL_Pos     4
#define RST_CLK_PLL_CONTROL_PLL_USB_MUL_Msk     ((uint32_t)0x000000F0)

#define RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos     8
#define RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Msk     ((uint32_t)0x00000F00)


/** @} */ /* End of group Periph_RST_CLK_RST_CLK_PLL_CONTROL_Bits */

/** @} */ /* End of group Periph_RST_CLK_Defines */

/** @defgroup Periph_RST_CLK_Defines Defines
  * @{
  */

/** @defgroup Periph_RST_CLK_RST_CLK_HS_CONTROL_Bits RST_CLK_HS_CONTROL
  * @{
  */

#define RST_CLK_HS_CONTROL_HSE_ON_Pos           0
#define RST_CLK_HS_CONTROL_HSE_ON               ((uint32_t)0x00000001)

#define RST_CLK_HS_CONTROL_HSE_BYP_Pos          1
#define RST_CLK_HS_CONTROL_HSE_BYP              ((uint32_t)0x00000002)


/** @} */ /* End of group Periph_RST_CLK_RST_CLK_HS_CONTROL_Bits */

/** @} */ /* End of group Periph_RST_CLK_Defines */

/** @defgroup Periph_RST_CLK_Defines Defines
  * @{
  */

/** @defgroup Periph_RST_CLK_RST_CLK_CPU_CLOCK_Bits RST_CLK_CPU_CLOCK
  * @{
  */

#define RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos        0
#define RST_CLK_CPU_CLOCK_CPU_C1_SEL_Msk        ((uint32_t)0x00000003)

#define RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos        2
#define RST_CLK_CPU_CLOCK_CPU_C2_SEL            ((uint32_t)0x00000004)

#define RST_CLK_CPU_CLOCK_CPU_C3_SEL_Pos        4
#define RST_CLK_CPU_CLOCK_CPU_C3_SEL_Msk        ((uint32_t)0x000000F0)

#define RST_CLK_CPU_CLOCK_HCLK_SEL_Pos          8
#define RST_CLK_CPU_CLOCK_HCLK_SEL_Msk          ((uint32_t)0x00000300)

#define RST_CLK_PER_CLOCK_CAN1      (1 << 0)
#define RST_CLK_PER_CLOCK_CAN2      (1 << 1)
#define RST_CLK_PER_CLOCK_USB       (1 << 2)
#define RST_CLK_PER_CLOCK_EEPROM    (1 << 3)
#define RST_CLK_PER_CLOCK_RSTCLK    (1 << 4)
#define RST_CLK_PER_CLOCK_DMA       (1 << 5)
#define RST_CLK_PER_CLOCK_UART1     (1 << 6)
#define RST_CLK_PER_CLOCK_UART2     (1 << 7)
#define RST_CLK_PER_CLOCK_SSP1      (1 << 8)
#define RST_CLK_PER_CLOCK_I2C1      (1 << 10)
#define RST_CLK_PER_CLOCK_POWER     (1 << 11)
#define RST_CLK_PER_CLOCK_WWDT      (1 << 12)
#define RST_CLK_PER_CLOCK_IWDT      (1 << 13)
#define RST_CLK_PER_CLOCK_TIMER1    (1 << 14)
#define RST_CLK_PER_CLOCK_TIMER2    (1 << 15)
#define RST_CLK_PER_CLOCK_TIMER3    (1 << 16)
#define RST_CLK_PER_CLOCK_ADC       (1 << 17)
#define RST_CLK_PER_CLOCK_DAC       (1 << 18)
#define RST_CLK_PER_CLOCK_COMP      (1 << 19)
#define RST_CLK_PER_CLOCK_SSP2      (1 << 20)
#define RST_CLK_PER_CLOCK_PORTA     (1 << 21)
#define RST_CLK_PER_CLOCK_PORTB     (1 << 22)
#define RST_CLK_PER_CLOCK_PORTC     (1 << 23)
#define RST_CLK_PER_CLOCK_PORTD     (1 << 24)
#define RST_CLK_PER_CLOCK_PORTE     (1 << 25)
#define RST_CLK_PER_CLOCK_BKP       (1 << 27)
#define RST_CLK_PER_CLOCK_PORTF     (1 << 29)
#define RST_CLK_PER_CLOCK_EXT_BUS   (1 << 30)

#define PORTA_BASE          0x400A8000
#define PORTB_BASE          0x400B0000
#define PORTC_BASE          0x400B8000
#define PORTD_BASE          0x400C0000
#define PORTE_BASE          0x400C8000
#define PORTF_BASE          0x400E8000

#define PORTA               ((PORT_TypeDef*)    PORTA_BASE)
#define PORTB               ((PORT_TypeDef*)    PORTB_BASE)
#define PORTC               ((PORT_TypeDef*)    PORTC_BASE)
#define PORTD               ((PORT_TypeDef*)    PORTD_BASE)
#define PORTE               ((PORT_TypeDef*)    PORTE_BASE)
#define PORTF               ((PORT_TypeDef*)    PORTF_BASE)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_ROM_Pos   0
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_ROM       ((uint32_t)0x00000001)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_RAM_Pos   1
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_RAM       ((uint32_t)0x00000002)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_NAND_Pos  2
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_NAND      ((uint32_t)0x00000004)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_CPOL_Pos  3
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_CPOL      ((uint32_t)0x00000008)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_BUSY_Pos  7
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_BUSY      ((uint32_t)0x00000080)

#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_WAIT_STATE_Pos 12
#define EXT_BUS_CNTRL_EXT_BUS_CONTROL_WAIT_STATE_Msk ((uint32_t)0x0000F000)

typedef struct
{
       uint32_t RESERVED0[20];
  __IO uint32_t NAND_CYCLES;
  __IO uint32_t EXT_BUS_CONTROL;
}EXT_BUS_CNTRL_TypeDef;


#define EXT_BUS_CNTRL_BASE  0x400F0000

#define EXT_BUS_CNTRL       ((EXT_BUS_CNTRL_TypeDef*)EXT_BUS_CNTRL_BASE)



#define SYSCLOCK_MULTIPLIER	(48000000/8000000)



#if (CONFIG_NR_DRAM_BANKS > 0)
/*
 * Check if RAM configured
 */
# if !defined(CONFIG_SYS_RAM_CS) || !defined(CONFIG_SYS_FSMC_PSRAM_BCR) ||     \
     !defined(CONFIG_SYS_FSMC_PSRAM_BTR)
#  error "Incorrect PSRAM FSMC configuration."
# endif
#endif /* CONFIG_NR_DRAM_BANKS */

DECLARE_GLOBAL_DATA_PTR;

/*
 * Early hardware init.
 */
int board_init(void)
{
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
	return 0;
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
	// Setup external RAM
	RST_CLK->PER_CLOCK |= \
				RST_CLK_PER_CLOCK_PORTA | \
				RST_CLK_PER_CLOCK_PORTB | \
				RST_CLK_PER_CLOCK_PORTC | \
				RST_CLK_PER_CLOCK_PORTD | \
				RST_CLK_PER_CLOCK_PORTE | \
				RST_CLK_PER_CLOCK_PORTF; // Clock periferial devs
	// Set funct and power, digital
	#define SETUP_PORT(PORT,F,A,P) (PORT->FUNC = F, PORT->ANALOG = A, PORT->PWR = P)
	#define SETUP_PORT_MAIN(PORT) SETUP_PORT(PORT,0x55555555,0xFFFF,0xFFFFFFFF)
	SETUP_PORT_MAIN(PORTA);
	SETUP_PORT_MAIN(PORTB);
	SETUP_PORT(PORTC,0xAA001554,0xFC7E,0xF0F03FFC);
	PORTC->OE   = 0x0C00;
	PORTC->RXTX = 0x0000;
	SETUP_PORT(PORTD,0xC3FFE800,0x9FFF,0x03FFFC00);
	SETUP_PORT(PORTE,0x55555555,0xFFFF,0xFFFFFFFF);
	SETUP_PORT(PORTF,0x5555555F,0xFFFF,0xFFFFFFFF);


	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_EXT_BUS;
	EXT_BUS_CNTRL->EXT_BUS_CONTROL=EXT_BUS_CNTRL_EXT_BUS_CONTROL_RAM | \
		 (0xF<<EXT_BUS_CNTRL_EXT_BUS_CONTROL_WAIT_STATE_Pos);
	/* Enable clock to watchdog timer */
	RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_IWDT;


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

#ifdef CONFIG_STM32_ETH
/*
 * Register ethernet driver
 */
int board_eth_init(bd_t *bis)
{
	return stm32_eth_init(bis);
}
#endif

void eth_halt(void)
{
}

int eth_init(bd_t * bis)
{
	return 0;
}

int eth_send(volatile void *ptr, int len)
{
	return 0;
}

int eth_rx(void)
{
	return 0;
}

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
u32 __attribute__((section(".ramcode")))
	     __attribute__ ((long_call))
envm_write(u32 offset, void * buf, u32 size)
{
	s32 ret = 0;

	return ret;
}
