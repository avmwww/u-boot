/*
 * (C) Copyright 2011
 *
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
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

#include <common.h>

#include "clock.h"
#include "envm.h"

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


/*
 * STM32 Clock configuration is set by the number of CONFIG options.
 *
 * Source for system clock must be selected:
 * - CONFIG_STM32_SYS_CLK_HSI: HSI oscillator used as sys clock,
 *   CONFIG_STM32_SYS_CLK_HSE: HSE oscillator used as sys clock,
 *   CONFIG_STM32_SYS_CLK_PLL: PLL used as sys clock.
 *
 * In case of CONFIG_STM32_SYS_CLK_HSE or CONFIG_STM32_SYS_CLK_PLL with
 * CONFIG_STM32_PLL_SRC_HSE configurations, the following option must be set:
 * - CONFIG_STM32_HSE_HZ: HSE oscillator frequency value.
 *
 * In case of CONFIG_STM32_SYS_CLK_PLL configuration, the following options
 * must be set:
 * - CONFIG_STM32_PLL_SRC_HSI: HSI clock used as PLL clock entry,
 *   CONFIG_STM32_PLL_SRC_HSE: HSE clock used as PLL clock entry;
 * - CONFIG_STM32_PLL_M: division factor for PLL input clock;
 * - CONFIG_STM32_PLL_N: multiplication factor for VCO;
 * - CONFIG_STM32_PLL_P: division factor for main system clock;
 * - CONFIG_STM32_PLL_Q: division factor for USB OTG FS, SDIO and RNG clocks.
 * Then:
 *  Fsys = Fvco / P    : PLL general clock output;
 *  Fvco = Fin * N / M : VCO clock (Fin is the frequency of PLL source);
 *  Fout = Fvco / Q    : USB OTG FS, SDIO, RNG clock output.
 *
 * !!! Note, beside of the options specified above VCC voltage may limit the
 * acceptable range of system clock, below is assumed that VDD(V) is equal to
 * 3.3V (i.e. in range of [2.7; 3.6]).
 * See AN3362 "Clock configuration tool for STM32F2xx microcontrollers"
 * application note, and stm32f2xx_rcc.c code of STM32F2xx_StdPeriph_Driver
 * library from STM for details.
 */

/*
 * Check configuration params
 */
#if ((!!defined(CONFIG_STM32_SYS_CLK_HSI) +				       \
      !!defined(CONFIG_STM32_SYS_CLK_HSE) +				       \
      !!defined(CONFIG_STM32_SYS_CLK_PLL)) != 1)
# error "Incorrect SYS Clock source configuration."
#endif

#if defined(CONFIG_STM32_SYS_CLK_HSE) ||				       \
    (defined(CONFIG_STM32_SYS_CLK_PLL) &&				       \
     defined(CONFIG_STM32_PLL_SRC_HSE))
# if !defined(CONFIG_STM32_HSE_HZ)
#  error "External oscillator HSE value is not set."
# endif
#endif

#if defined(CONFIG_STM32_SYS_CLK_PLL)
# if ((!!defined(CONFIG_STM32_PLL_SRC_HSI) +				       \
       !!defined(CONFIG_STM32_PLL_SRC_HSE)) != 1)
#  error "Incorrect PLL clock source configuration."
# endif
# if ((!!defined(CONFIG_STM32_PLL_M) +					       \
       !!defined(CONFIG_STM32_PLL_N) +					       \
       !!defined(CONFIG_STM32_PLL_P) +					       \
       !!defined(CONFIG_STM32_PLL_Q)) != 4)
#  error "Incomplete PLL configuration."
# endif
# if (CONFIG_STM32_PLL_M < 2) || (CONFIG_STM32_PLL_M > 63)
#  error "Incorrect PLL_M value."
# endif
# if (CONFIG_STM32_PLL_N < 192) || (CONFIG_STM32_PLL_N > 432)
#  error "Incorrect PLL_N value."
# endif
# if (CONFIG_STM32_PLL_P != 2) && (CONFIG_STM32_PLL_P != 4) &&		       \
     (CONFIG_STM32_PLL_P != 6) && (CONFIG_STM32_PLL_P != 8)
#  error "Incorrect PLL_P value."
# endif
# if (CONFIG_STM32_PLL_Q < 4) || (CONFIG_STM32_PLL_Q > 15)
#  error "Incorrect PLL_Q value."
# endif
#endif

/*
 * Internal oscillator value
 */
#define STM32_HSI_HZ			16000000	/* 16 MHz	      */

/*
 * Get the SYS CLK value according to configuration
 */
#if defined(CONFIG_STM32_SYS_CLK_HSI)
# define STM32_SYS_CLK			STM32_HSI_HZ
#elif defined(CONFIG_STM32_SYS_CLK_HSE)
# define STM32_SYS_CLK			CONFIG_STM32_HSE_HZ
#else
# if defined(CONFIG_STM32_PLL_SRC_HSE)
#  define STM32_PLL_IN_HZ		CONFIG_STM32_HSE_HZ
# else
#  define STM32_PLL_IN_HZ		STM32_HSI_HZ
# endif
# define STM32_SYS_CLK			((STM32_PLL_IN_HZ *		       \
					  CONFIG_STM32_PLL_N) /		       \
					 (CONFIG_STM32_PLL_M *		       \
					  CONFIG_STM32_PLL_P))
#endif

/*
 * Get the Flash latency value for this SYS CLK
 */
#  if (STM32_SYS_CLK >        0) && (STM32_SYS_CLK <=  30000000)
# define STM32_FLASH_WS			0
#elif (STM32_SYS_CLK > 30000000) && (STM32_SYS_CLK <=  60000000)
# define STM32_FLASH_WS			1
#elif (STM32_SYS_CLK > 60000000) && (STM32_SYS_CLK <=  90000000)
# define STM32_FLASH_WS			2
#elif (STM32_SYS_CLK > 90000000) && (STM32_SYS_CLK <= 120000000)
# define STM32_FLASH_WS			3
#elif (STM32_SYS_CLK > 120000000) && (STM32_SYS_CLK <= 150000000)
# define STM32_FLASH_WS			4
#elif (STM32_SYS_CLK > 150000000) && (STM32_SYS_CLK <= 168000000)
# define STM32_FLASH_WS			5
#else
# error "Incorrect System clock value configuration."
# define STM32_FLASH_WS			0	/* to avoid compile-time err  */
#endif

/*
 * Offsets and bitmasks of some RCC regs
 */
#define STM32_RCC_CR_HSEON		(1 << 16) /* HSE clock enable	      */
#define STM32_RCC_CR_HSERDY		(1 << 17) /* HSE clock ready	      */
#define STM32_RCC_CR_PLLON		(1 << 24) /* PLL clock enable	      */
#define STM32_RCC_CR_PLLRDY		(1 << 25) /* PLL clock ready	      */

#define STM32_RCC_CFGR_SW_BIT		0	/* System clock switch	      */
#define STM32_RCC_CFGR_SW_MSK		0x3
#define STM32_RCC_CFGR_SWS_BIT		2	/* System clock switch status */
#define STM32_RCC_CFGR_SWS_MSK		0x3

#define STM32_RCC_CFGR_SWS_HSI		0x0
#define STM32_RCC_CFGR_SWS_HSE		0x1
#define STM32_RCC_CFGR_SWS_PLL		0x2

#define STM32_RCC_CFGR_HPRE_BIT		4	/* AHB prescaler	      */
#define STM32_RCC_CFGR_HPRE_MSK		0xF
#define STM32_RCC_CFGR_HPRE_DIVNO	0x0
#define STM32_RCC_CFGR_HPRE_DIV2	0x8
#define STM32_RCC_CFGR_HPRE_DIV4	0x9
#define STM32_RCC_CFGR_HPRE_DIV8	0xA
#define STM32_RCC_CFGR_HPRE_DIV16	0xB
#define STM32_RCC_CFGR_HPRE_DIV64	0xC
#define STM32_RCC_CFGR_HPRE_DIV128	0xD
#define STM32_RCC_CFGR_HPRE_DIV256	0xE
#define STM32_RCC_CFGR_HPRE_DIV512	0xF

#define STM32_RCC_CFGR_PPRE1_BIT	10	/* APB Low speed presc (APB1) */
#define STM32_RCC_CFGR_PPRE1_MSK	0x7
#define STM32_RCC_CFGR_PPRE1_DIV0	0x0
#define STM32_RCC_CFGR_PPRE1_DIV2	0x4
#define STM32_RCC_CFGR_PPRE1_DIV4	0x5
#define STM32_RCC_CFGR_PPRE1_DIV8	0x6
#define STM32_RCC_CFGR_PPRE1_DIV16	0x7

#define STM32_RCC_CFGR_PPRE2_BIT	13	/* APB high-speed presc (APB2)*/
#define STM32_RCC_CFGR_PPRE2_MSK	0x7
#define STM32_RCC_CFGR_PPRE2_DIVNO	0x0
#define STM32_RCC_CFGR_PPRE2_DIV2	0x4
#define STM32_RCC_CFGR_PPRE2_DIV4	0x5
#define STM32_RCC_CFGR_PPRE2_DIV8	0x6
#define STM32_RCC_CFGR_PPRE2_DIV16	0x7

#define STM32_RCC_PLLCFGR_HSESRC	(1 << 22) /* Main PLL entry clock src */

#define STM32_RCC_PLLCFGR_PLLM_BIT	0	/* Div factor for input clock */
#define STM32_RCC_PLLCFGR_PLLM_MSK	0x3F

#define STM32_RCC_PLLCFGR_PLLN_BIT	6	/* Mult factor for VCO	      */
#define STM32_RCC_PLLCFGR_PLLN_MSK	0x1FF

#define STM32_RCC_PLLCFGR_PLLP_BIT	16	/* Div factor for main sysclk */
#define STM32_RCC_PLLCFGR_PLLP_MSK	0x3

#define STM32_RCC_PLLCFGR_PLLQ_BIT	24	/* Div factor for USB,SDIO,.. */
#define STM32_RCC_PLLCFGR_PLLQ_MSK	0xF

/*
 * Timeouts (in cycles)
 */
#define STM32_HSE_STARTUP_TIMEOUT	0x0500

/*
 * Clock values
 */
static u32 clock_val[CLOCK_END];

#if 0//!defined(CONFIG_STM32_SYS_CLK_HSI)
/*
 * Set-up clock configuration.
 */
static void clock_setup(void)
{
	u32	val;
	int	i;

	/*
	 * Enable HSE, and wait it becomes ready
	 */
	STM32_RCC->cr |= STM32_RCC_CR_HSEON;
	for (i = 0; i < STM32_HSE_STARTUP_TIMEOUT; i++) {
		if (STM32_RCC->cr & STM32_RCC_CR_HSERDY)
			break;
	}

	if (!(STM32_RCC->cr & STM32_RCC_CR_HSERDY)) {
		/*
		 * Have no HSE clock
		 */
		goto out;
	}

	val = STM32_RCC->cfgr;

	/*
	 * HCLK = SYSCLK / 1
	 */
	val &= ~(STM32_RCC_CFGR_HPRE_MSK << STM32_RCC_CFGR_HPRE_BIT);
	val |= STM32_RCC_CFGR_HPRE_DIVNO << STM32_RCC_CFGR_HPRE_BIT;

	/*
	 * PCLK2 = HCLK / 2
	 */
	val &= ~(STM32_RCC_CFGR_PPRE2_MSK << STM32_RCC_CFGR_PPRE2_BIT);
	val |= STM32_RCC_CFGR_PPRE2_DIV2 << STM32_RCC_CFGR_PPRE2_BIT;

	/*
	 * PCLK1 = HCLK / 4
	 */
	val &= ~(STM32_RCC_CFGR_PPRE1_MSK << STM32_RCC_CFGR_PPRE1_BIT);
	val |= STM32_RCC_CFGR_PPRE1_DIV4 << STM32_RCC_CFGR_PPRE1_BIT;

	STM32_RCC->cfgr = val;

# if defined(CONFIG_STM32_SYS_CLK_PLL)
	/*
	 * Configure the main PLL
	 */
#  if defined(CONFIG_STM32_PLL_SRC_HSE)
	val = STM32_RCC_PLLCFGR_HSESRC;
#  else
	val = 0;
#  endif

	val |= CONFIG_STM32_PLL_M << STM32_RCC_PLLCFGR_PLLM_BIT;
	val |= CONFIG_STM32_PLL_N << STM32_RCC_PLLCFGR_PLLN_BIT;
	val |= ((CONFIG_STM32_PLL_P >> 1) - 1) << STM32_RCC_PLLCFGR_PLLP_BIT;
	val |= CONFIG_STM32_PLL_Q << STM32_RCC_PLLCFGR_PLLQ_BIT;

	STM32_RCC->pllcfgr = val;

	/*
	 * Enable the main PLL, and wait until main PLL becomes ready.
	 * Note: we wait infinitely here, since the max time necessary for
	 * PLL to lock is probably not a constant. There's no timeout here in
	 * STM lib code as well.
	 */
	STM32_RCC->cr |= STM32_RCC_CR_PLLON;
	while (STM32_RCC->cr & STM32_RCC_CR_PLLRDY);

	/*
	 * Select PLL as system source if it's setup OK, and HSE otherwise
	 */
	if (!(STM32_RCC->cr & STM32_RCC_CR_PLLRDY))
		val = STM32_RCC_CFGR_SWS_PLL;
	else
		val = STM32_RCC_CFGR_SWS_HSE;
# else
	/*
	 * Select HSE as system source
	 */
	val = STM32_RCC_CFGR_SWS_HSE;
# endif /* CONFIG_STM32_SYS_CLK_PLL */

	/*
	 * Configure Flash prefetch, Instruction cache, and wait
	 * latency.
	 */
	envm_config(STM32_FLASH_WS);

	/*
	 * Change system clock source, and wait (infinite!) till it done
	 */
	STM32_RCC->cfgr &= ~(STM32_RCC_CFGR_SW_MSK << STM32_RCC_CFGR_SW_BIT);
	STM32_RCC->cfgr |= val << STM32_RCC_CFGR_SW_BIT;
	while ((STM32_RCC->cfgr & (STM32_RCC_CFGR_SWS_MSK <<
				   STM32_RCC_CFGR_SWS_BIT)) !=
	       (val << STM32_RCC_CFGR_SWS_BIT));
out:
	return;
}
#endif /* CONFIG_STM32_SYS_CLK_HSI */

static void inline ext_ram_init(void)
{
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
}


/*
 * Initialize the reference clocks.
 */
void clock_init(void)
{
#if 0
	static u32 apbahb_presc_tbl[] = {0, 0, 0, 0, 1, 2, 3, 4,
					 1, 2, 3, 4, 6, 7, 8, 9};

	u32 tmp, presc, pllvco, pllp, pllm;

#if !defined(CONFIG_STM32_SYS_CLK_HSI)
	/*
	 * Set clocks to cfg, which is differs from the poweron default
	 */
	clock_setup();
#else
	/*
	 * For consistency with !HSI configs, enable prefetch and cache
	 */
	envm_config(STM32_FLASH_WS);
#endif

	/*
	 * Get SYSCLK
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_SWS_BIT;
	tmp &= STM32_RCC_CFGR_SWS_MSK;
	switch (tmp) {
	case STM32_RCC_CFGR_SWS_HSI:
		/* HSI used as system clock source */
		clock_val[CLOCK_SYSCLK] = STM32_HSI_HZ;
		break;
	case STM32_RCC_CFGR_SWS_HSE:
		/* HSE used as system clock source */
		clock_val[CLOCK_SYSCLK] = CONFIG_STM32_HSE_HZ;
		break;
	case STM32_RCC_CFGR_SWS_PLL:
		/* PLL used as system clock source */
		/*
		 * PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
		 * SYSCLK = PLL_VCO / PLLP
		 */
		pllm  = STM32_RCC->pllcfgr >> STM32_RCC_PLLCFGR_PLLM_BIT;
		pllm &= STM32_RCC_PLLCFGR_PLLM_MSK;

		if (STM32_RCC->pllcfgr & STM32_RCC_PLLCFGR_HSESRC) {
			/* HSE used as PLL clock source */
			tmp = CONFIG_STM32_HSE_HZ;
		} else {
			/* HSI used as PLL clock source */
			tmp = STM32_HSI_HZ;
		}
		pllvco  = STM32_RCC->pllcfgr >> STM32_RCC_PLLCFGR_PLLN_BIT;
		pllvco &= STM32_RCC_PLLCFGR_PLLN_MSK;
		pllvco *= tmp / pllm;

		pllp  = STM32_RCC->pllcfgr >> STM32_RCC_PLLCFGR_PLLP_BIT;
		pllp &= STM32_RCC_PLLCFGR_PLLP_MSK;
		pllp  = (pllp + 1) * 2;

		clock_val[CLOCK_SYSCLK] = pllvco / pllp;
		break;
	default:
		clock_val[CLOCK_SYSCLK] = STM32_HSI_HZ;
		break;
	}

	/*
	 * Get HCLK
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_HPRE_BIT;
	tmp &= STM32_RCC_CFGR_HPRE_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_HCLK] = clock_val[CLOCK_SYSCLK] >> presc;

	/*
	 * Get PCLK1
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_PPRE1_BIT;
	tmp &= STM32_RCC_CFGR_PPRE1_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_PCLK1] = clock_val[CLOCK_HCLK] >> presc;

	/*
	 * Get PCLK2
	 */
	tmp  = STM32_RCC->cfgr >> STM32_RCC_CFGR_PPRE2_BIT;
	tmp &= STM32_RCC_CFGR_PPRE2_MSK;
	presc = apbahb_presc_tbl[tmp];
	clock_val[CLOCK_PCLK2] = clock_val[CLOCK_HCLK] >> presc;

	/*
	 * Set SYSTICK. Divider "8" is the SOC hardcoded.
	 */
	 clock_val[CLOCK_SYSTICK] = clock_val[CLOCK_HCLK] / 8;
#endif

        /* Enable HSE generator. */
        RST_CLK->HS_CONTROL = RST_CLK_HS_CONTROL_HSE_ON;
        while (! (RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY));

	ext_ram_init();

        /* Use HSE for CPU_C1 clock. */
        RST_CLK->CPU_CLOCK = (2<<RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos);

        /* Setup PLL for CPU. */
        RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos);
        RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) |
                RST_CLK_PLL_CONTROL_PLL_CPU_ON;
        RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) |
                RST_CLK_PLL_CONTROL_PLL_CPU_ON | RST_CLK_PLL_CONTROL_PLL_USB_RLD;
        RST_CLK->PLL_CONTROL = ((SYSCLOCK_MULTIPLIER-1)<<RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos) |
                RST_CLK_PLL_CONTROL_PLL_CPU_ON;
        while (! (RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY));

        /* Use PLLCPUo for CPU_C2, CPU_C3 and HCLK. */
        RST_CLK->CPU_CLOCK = (1<<RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos) |
                                (2<<RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos) |
                                (1<<RST_CLK_CPU_CLOCK_HCLK_SEL_Pos);


	return;
}

/*
 * Return a clock value for the specified clock.
 * Note that we need this function in RAM because it will be used
 * during self-upgrade of U-boot into eNMV.
 * @param clck          id of the clock
 * @returns             frequency of the clock
 */
unsigned long  __attribute__((section(".ramcode")))
	       __attribute__ ((long_call))
	       clock_get(enum clock clck)
{
	return clock_val[clck];
}
