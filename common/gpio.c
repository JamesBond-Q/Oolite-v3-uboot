/******************************************************************************
* Filename : gpio.c
* This part is used to control LED and detect button-press
* 
******************************************************************************/

#include <common.h>
#include "../autoconf.h"
#include <configs/rt2880.h>
#include <rt_mmap.h>
#include <gpio.h>

#define ARRAY_SIZE(x)		(sizeof(x) / sizeof((x)[0]))

#define SYSCTR_ADDR	0xB0000000
#define IRQ_ADDR 	0xB0000200
#define PRGIO_ADDR 	0xB0000600

#define ra_inl(offset)		(*(volatile unsigned long *)(offset))
#define ra_outl(offset,val)	(*(volatile unsigned long *)(offset) = val)
#define ra_and(addr, value) ra_outl(addr, (ra_inl(addr) & (value)))
#define ra_or(addr, value) ra_outl(addr, (ra_inl(addr) | (value)))

//MT7620
#define RALINK_GPIOMODE_I2C             (1U << 0)	/* GPIO #1, #2 */
#define RALINK_GPIOMODE_UARTF		(7U << 2)	/* GPIO #7~#14 */
#define RALINK_GPIOMODE_UARTL		(1U << 5)	/* GPIO#15,#16 */
#define RALINK_GPIOMODE_MDIO		(3U << 7)	/* GPIO#22,#23 */
#define RALINK_GPIOMODE_RGMII1		(1U << 9)	/* GPIO#24~#35 */
#define RALINK_GPIOMODE_RGMII2		(1U << 10)	/* GPIO#60~#71 */
#define RALINK_GPIOMODE_SPI		(1U << 11)	/* GPIO#3~#6 */
#define RALINK_GPIOMODE_SPI_REFCLK	(1U << 12)	/* GPIO#37 */
#define RALINK_GPIOMODE_WLED		(1U << 13)	/* GPIO#72 */
#define RALINK_GPIOMODE_JTAG            (1U << 15)	/* GPIO#40~#44 */
#define RALINK_GPIOMODE_PERST		(3U << 16)	/* GPIO#36 */
#define RALINK_GPIOMODE_NAND_SD		(3U << 18)	/* GPIO#45~#59 */
#define RALINK_GPIOMODE_PA		(1U << 20)	/* GPIO#18~#21 */
#define RALINK_GPIOMODE_WDT		(3U << 21)	/* GPIO#17 */
#define RALINK_GPIOMODE_SUTIF		(3U << 30)	/* FIXME */



#define SYSCFG_OFFSET		0x10
#define UARTF_PCM_MODE_SHIFT	6
/*
6 R/W UARTF_PCM_MODE
0: Set 4 of UART-Full pins as normal UART function
1: Set 4 of UART-Full pins as normal PCM function
1¡¯b0
*/
#define GPIOMODE_OFFSET		0x60
#define UARTF_GPIO_MODE_SHIFT	1

/*
1 R/W UARTF_GPIO_MODE
0:Normal Mode
1:GPIO Mode
Control GPIO[6:3]
1¡¯b1
*/

#if 0
void uart_pcm_mode(unsigned x)
{
	ulong v = le32_to_cpu(*((volatile u_long *)(SYSCTR_ADDR + SYSCFG_OFFSET)));
   
	if (0 == x) {
		v &= ~(1 << UARTF_PCM_MODE_SHIFT);
		*((volatile u_long *)(SYSCTR_ADDR + SYSCFG_OFFSET)) = cpu_to_le32(v);
	}
	else if(1 == x) {
		v |= (1 << UARTF_PCM_MODE_SHIFT);
		*((volatile u_long *)(SYSCTR_ADDR + SYSCFG_OFFSET)) = cpu_to_le32(v);
	}

}


void uart_gpio_mode(unsigned x)
{
	ulong v = le32_to_cpu(*(volatile u_long *)(SYSCTR_ADDR + GPIOMODE_OFFSET));

	if (0 == x) {
		v &= ~(1 << UARTF_GPIO_MODE_SHIFT);
		*(volatile u_long *)(SYSCTR_ADDR + GPIOMODE_OFFSET) = cpu_to_le32(v);
	}
	else if(1 == x) {
		v |= (1 << UARTF_GPIO_MODE_SHIFT);
		*(volatile u_long *)(SYSCTR_ADDR + GPIOMODE_OFFSET) = cpu_to_le32(v);
	}
}
#endif

//---by bond #if defined(MT7620_MP)
#if defined(MT7620_MP)
const static struct gpio_reg_offset_s {
	unsigned short min_nr, max_nr;
	unsigned short int_offset;
	unsigned short edge_offset;
	unsigned short rmask_offset;
	unsigned short fmask_offset;
	unsigned short data_offset;
	unsigned short dir_offset;
	unsigned short pol_offset;
	unsigned short set_offset;
	unsigned short reset_offset;
	unsigned short tog_offset;
} s_gpio_reg_offset[] = {
	{  0, 23,  0x0,  0x4,  0x8,  0xc, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34 },
	{ 24, 39, 0x38, 0x3c, 0x40, 0x44, 0x48, 0x4c, 0x50, 0x54, 0x58, 0x5c },
	{ 40, 71, 0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x84 },
	{ 72, 72, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c, 0xa0, 0xa4, 0xa8, 0xac },
};
#endif

//rst-ok!!!
#if 0		
#if defined (MT7628_ASIC_BOARD)
const static struct gpio_reg_offset_s {
	unsigned short min_nr, max_nr;
	unsigned short int_offset;
	unsigned short edge_offset;	//gpio_edge
	unsigned short rmask_offset;//gpio_redge
	unsigned short fmask_offset;//gpio_fedge
	unsigned short data_offset;	//gpio_data
	unsigned short dir_offset;	//gpio_ctrl
	unsigned short pol_offset;	//gpio_pol
	unsigned short set_offset;	//gpio_dset
	unsigned short reset_offset;//gpio_dclr
	unsigned short tog_offset;
} s_gpio_reg_offset[] = {
	{  0, 31,  0x0,  0x4,  0x8,  0xc, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34 },
	{ 32, 63, 0x38, 0x3c, 0x40, 0x44, 0x24, 0x04, 0x50, 0x54, 0x58, 0x5c },//chang this by yourself
	{ 64, 95, 0x60, 0x64, 0x68, 0x6c, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x84 },
};
#endif
#endif

#if defined (MT7628_ASIC_BOARD)
const static struct gpio_reg_offset_s {
	unsigned short min_nr, max_nr;
	unsigned short int_offset;
	unsigned short edge_offset;	//gpio_edge
	unsigned short rmask_offset;//gpio_redge
	unsigned short fmask_offset;//gpio_fedge
	unsigned short data_offset;	//gpio_data	+
	unsigned short dir_offset;	//gpio_ctrl		+
	unsigned short pol_offset;	//gpio_pol
	unsigned short set_offset;	//gpio_dset
	unsigned short reset_offset;//gpio_dclr
	unsigned short tog_offset;
} s_gpio_reg_offset[] = {
	{  0, 31,  0x0,  0x4,  0x8,  0xc, 0x20, 0x00, 0x28, 0x2c, 0x30, 0x34 },
	{ 32, 63, 0x38, 0x3c, 0x40, 0x44, 0x24, 0x04, 0x50, 0x54, 0x58, 0x5c },//chang this by yourself
	{ 64, 95, 0x60, 0x64, 0x68, 0x6c, 0x28, 0x08, 0x78, 0x7c, 0x80, 0x84 },
};
#endif


static int is_valid_gpio_nr(unsigned short gpio_nr)
{
	return (gpio_nr > 72)? 0:1;
}

/* Query GPIO number belongs which item.
 * @gpio_nr:	GPIO number
 * @return:
 * 	NULL:	Invalid parameter.
 *  otherwise:	Pointer to a gpio_reg_offset_s instance.
 */
static const struct gpio_reg_offset_s *get_gpio_reg_item(unsigned short gpio_nr)
{
	int i;
	const struct gpio_reg_offset_s *p = &s_gpio_reg_offset[0], *ret = NULL;

	if (!is_valid_gpio_nr(gpio_nr))
		return ret;

	for (i = 0; !ret && i < ARRAY_SIZE(s_gpio_reg_offset); ++i, ++p) {
		if (gpio_nr < p->min_nr || gpio_nr > p->max_nr)
			continue;

		ret = p;
		//printf("\n--->gpio.c--->get_gpio_reg_item:ret = 0x%x\n",ret);
	}

	return ret;
}

/* Return bit-shift of a GPIO.
 * @gpio_nr:	GPIO number
 * @return:
 * 	0~31:	bit-shift of a GPIO pin in a register.
 *  	-1:	Invalid parameter.
 */
static int get_gpio_reg_bit_shift(unsigned short gpio_nr)
{
	const struct gpio_reg_offset_s *p;

	if (!(p = get_gpio_reg_item(gpio_nr)))
		return -1;

	return gpio_nr - p->min_nr;
}

/* Return specific GPIO register in accordance with GPIO number
 * @gpio_nr:	GPIO number
 * @return
 * 	0:	invalid parameter
 *  otherwise:	address of GPIO register
 */
unsigned int mtk7620_get_gpio_reg_addr(unsigned short gpio_nr, enum gpio_reg_id id)
{
	int ret = 0;
	const struct gpio_reg_offset_s *p;

	if (!(p = get_gpio_reg_item(gpio_nr)) || id < 0 || id >= GPIO_MAX_REG)
		return ret;

	switch (id) {
	case GPIO_INT:
		ret = p->int_offset;
		break;
	case GPIO_EDGE:
		ret = p->edge_offset;
		break;
	case GPIO_RMASK:
		ret = p->rmask_offset;
		break;
	case GPIO_MASK:
		ret = p->fmask_offset;
		break;
	case GPIO_DATA:
		ret = p->data_offset;
		//+++by bond
		//printf("--->gpio.c--->mtk7620_get_gpio_reg_addr:GPIO_DATA---ret = 0x%x\n",ret);
		break;
	case GPIO_DIR:
		ret = p->dir_offset;		
		//printf("--->gpio.c--->mtk7620_get_gpio_reg_addr:GPIO_DIR---ret = 0x%x\n",ret);
		break;
	case GPIO_POL:
		ret = p->pol_offset;
		break;
	case GPIO_SET:
		ret = p->set_offset;
		break;
	case GPIO_RESET:
		ret = p->reset_offset;
		break;
	case GPIO_TOG:
		ret = p->tog_offset;
		break;
	default:
		return 0;
	}
	ret += PRGIO_ADDR;

	return ret;
}

/* Set GPIO pin direction.
 * If a GPIO pin is multi-function pin, it would be configured as GPIO mode.
 * @gpio_nr:	GPIO number
 * @gpio_dir:	GPIO direction
 * 	0: 	output
 * 	1:	input
 *  otherwise:	input
 * @return:
 * 	0:	Success
 * 	-1:	Invalid parameter
 */
int mtk7620_set_gpio_dir(unsigned short gpio_nr, unsigned short gpio_dir)
{
	int shift;
	unsigned int mask, val;
	unsigned int reg;

	if (!is_valid_gpio_nr(gpio_nr))
		return -1;

	reg = mtk7620_get_gpio_reg_addr(gpio_nr, GPIO_DIR);	
	//printf("--->gpio.c-->mtk7620_set_gpio_dir:---reg = 0x%x\n",reg);
	shift = get_gpio_reg_bit_shift(gpio_nr);
	//printf("--->gpio.c-->mtk7620_set_gpio_dir:---shift = 0x%x\n",shift);

	if (!gpio_dir) {
		/* output */
		ra_or(reg, 1U << shift);//set "1"
	} else {
		/* input */
		ra_and(reg, ~(1U << shift));//set "0"
	}

	/* Handle special GPIO pin */
	shift = -1;
	mask = val = 1;

//+++by bond
#if defined (MT7628_ASIC_BOARD)
		if (gpio_nr == 38) {
			/* WDT_RST_N */
			shift = 14;
		}else if (gpio_nr >=0 && gpio_nr <=3) {
			/* I2S set to GPIO mode 2'b01*/
			shift = 6;
			mask = 3;
			val = 1;
		}
		else if (gpio_nr >=4 && gpio_nr <=5) {
			/* I2C set to GPIO mode 2'b01*/
			shift = 20;
			mask = 3;
		}
		else if (gpio_nr == 6) {
			/*  SPI_CS1 */
			shift = 4;
			mask = 3;
		}
		else if (gpio_nr == 11) {
			/*  GPIO */
			shift = 0;
			mask = 3;
		}
		else if (gpio_nr >=14 && gpio_nr <=17) {
			/* SPIS_MODE set to GPIO mode 2'b01*/
			shift = 2;
			mask = 3;
			val = 1;
			//debug("SPIS_MODE!\n");
		}
		else if (gpio_nr == 18) {
			/* PWM0_MODE set to GPIO mode 2'b01*/
			shift = 28;
			mask = 3;
		}
		else if (gpio_nr == 19) {
			/* PWM1_MODE set to GPIO mode 2'b01*/
			shift = 30;
			mask = 3;
		}
		else if (gpio_nr >=20 && gpio_nr <=21) {
			/* UART2_MODE set to GPIO mode 2'b01*/
			shift = 26;
			mask = 3;
		}
	    else if (gpio_nr >=22 && gpio_nr <=29) {
			/* SD_MODE set to GPIO mode 2'b01*/
			shift = 10;
			mask = 3;
		}
		else if (gpio_nr == 30) {
			/* P4_LED_KN_MODE */
			shift = 26;
			mask = 3;
		}
		else if (gpio_nr == 31) {
			/* P3_LED_KN_MODE */
			shift = 24;
			mask = 3;
		}
		else if (gpio_nr == 32) {
			/* P2_LED_KN_MODE */
			shift = 22;
			mask = 3;
		}
		else if (gpio_nr == 33) {
			/* P1_LED_KN_MODE */
			shift = 20;
			mask = 3;
		}
		else if (gpio_nr == 34) {
			/* P0_LED_KN_MODE */
			shift = 18;
			mask = 3;
		}
		else if (gpio_nr == 35) {
			/* WLED_KN_MODE */
			shift = 16;
		}
		else if (gpio_nr == 36) {
			/* PERST_MODE */
			shift = 16;
			mask = 3;
		}
		else if (gpio_nr == 37) {
			/* REFCLK_MODE */
			shift = 18;
		}
		else if (gpio_nr == 39) {
			/* P4_LED_AN_MODE */
			shift = 10;
			mask = 3;
		}
		else if (gpio_nr == 40) {
			/* P3_LED_AN_MODE */
			shift = 8;
			mask = 3;
		}
		else if (gpio_nr == 41) {
			/* P2_LED_AN_MODE */
			shift = 6;
			mask = 3;
		}
		else if (gpio_nr == 42) {
			/* P1_LED_AN_MODE */
			shift = 4;
			mask = 3;
		}
		else if (gpio_nr == 43) {
			/* P0_LED_AN_MODE */
			shift = 2;
			mask = 3;
		}
		else if (gpio_nr >=45 && gpio_nr <=46) {
			/* UART1 set to GPIO mode 2'b01*/
			shift = 24;
			mask = 3;
		}else if (gpio_nr == 44){
			/* WLED_AN_MODE */
			shift = 0;
			mask = 3;
			val = 1;
		}
#endif


#if defined(MT7620_MP)
	if (gpio_nr >= 1 && gpio_nr <= 2) {
		/* I2C */
		shift = 0;
	} else if (gpio_nr >=  7 && gpio_nr <= 14) {
		/* UARTF */
		shift = 2;
		mask = 7;
		val = 7;
	} else if (gpio_nr >= 15 && gpio_nr <= 16) {
		/* UARTL */
		shift = 5;
	} else if (gpio_nr >= 22 && gpio_nr <= 23) {
		/* MDIO */
		shift = 7;
		mask = 3;
		val = 3;
	} else if (gpio_nr >= 24 && gpio_nr <= 35) {
		/* RGMII1 */
		shift = 9;
	} else if (gpio_nr >= 60 && gpio_nr <= 71) {
		/* RGMII2 */
		shift = 10;
	} //---else if (gpio_nr >= 3 && gpio_nr <= 6) {
//+++by bond
		else if (gpio_nr >= 38 && gpio_nr <= 39) {
		/* SPI */
		shift = 11;
//---by bond
	} else if (gpio_nr == 37) {
		/* SPI_REFCLK */
		shift = 12;
	} else if (gpio_nr == 72) {
		/* WLED */
		shift = 13;
	} else if (gpio_nr >= 40 && gpio_nr <= 44) {
		/* JTAG/EPHY_LED */
		shift = 15;
	} else if (gpio_nr == 36) {
		/* PERST */
		shift = 16;
		mask = 3;
		val = 3;
	} else if (gpio_nr >= 45 && gpio_nr <= 59) {
		/* NAND/SD_BT */
		shift = 18;
		mask = 3;
		val = 3;
	} else if (gpio_nr >= 18 && gpio_nr <= 21) {
		/* PA */
		shift = 20;
	} else if (gpio_nr == 17) {
		/* WDT */
		shift = 21;
		mask = 3;
		val = 3;
	}
#endif	//#if defined(MT7620_MP)

#if 0 //7628-ok
	if (shift >= 0) {
		unsigned long old = ra_inl(RT2880_GPIOMODE_REG), new;//RT2880_GPIOMODE_REG = 0xb0000060

		ra_and(RT2880_GPIOMODE_REG, ~(mask << shift));//set "0"
		if (val)
			ra_or(RT2880_GPIOMODE_REG, val << shift);//set "1"
		if (old != (new = ra_inl(RT2880_GPIOMODE_REG))) {
			debug("GPIO#%d updated GPIOMODE register: %08lx -> %08lx\n",gpio_nr, old, new);//---by bond
		}
	}
#endif

#if 0
	/*This for SET the RT2880_SYSCFG0_REG(0x10000010) [8] to "1"*/
	if(gpio_nr >=30 && gpio_nr <=34)
	{
		unsigned long old = ra_inl(RT2880_SYSCFG0_REG), new;
		//debug("GPIO#%d updated RT2880_AGPIOCFG_REG register: %08lx -> %08lx\n",gpio_nr, old, new);

		ra_and(RT2880_SYSCFG0_REG, ~( 0x1<< 8));//set "0"
		ra_or(RT2880_SYSCFG0_REG, 0x1 << 8);//set "1"
		new = ra_inl(RT2880_SYSCFG0_REG);
		debug("GPIO#%d updated RT2880_SYSCFG0_REG register: %08lx -> %08lx\n",gpio_nr, old, new);
		
	}
#endif

	/*This for SET the RT2880_AGPIOCFG_REG(0x1000003c) [17:20] to "1"*/
	if(gpio_nr >=22 && gpio_nr <=29)
	{
		unsigned long old = ra_inl(RT2880_AGPIOCFG_REG), new;
		//debug("GPIO#%d updated RT2880_AGPIOCFG_REG register: %08lx -> %08lx\n",gpio_nr, old, new);

		ra_and(RT2880_AGPIOCFG_REG, ~( 0xf<< 17));//set "0"
		ra_or(RT2880_AGPIOCFG_REG, 0xf << 17);//set "1"
		new = ra_inl(RT2880_AGPIOCFG_REG);
		//debug("GPIO#%d updated RT2880_AGPIOCFG_REG register: %08lx -> %08lx\n",gpio_nr, old, new);
		
	}
	
	//GPIO1_MODE
	if ( (gpio_nr >=0 && gpio_nr <=3)|(gpio_nr >=4 && gpio_nr <=5)|(gpio_nr == 6)|(gpio_nr == 11)|(gpio_nr >=14 && gpio_nr <=17)|(gpio_nr == 18)|(gpio_nr == 19)|(gpio_nr >=20 && gpio_nr <=21)|(gpio_nr >=22 && gpio_nr <=29)|(gpio_nr == 36)|(gpio_nr == 37)|(gpio_nr == 38)|(gpio_nr >=45 && gpio_nr <=46) ) {
		unsigned long old = ra_inl(RT2880_GPIOMODE_REG), new;//RT2880_GPIOMODE_REG = 0xb0000060

		ra_and(RT2880_GPIOMODE_REG, ~(mask << shift));//set "0"
		if (val)
			ra_or(RT2880_GPIOMODE_REG, val << shift);//set "1"
		if (old != (new = ra_inl(RT2880_GPIOMODE_REG))) {
			//debug("GPIO#%d updated GPIOMODE register: %08lx -> %08lx\n",gpio_nr, old, new);//---by bond
		}
	}
	
	//GPIO2_MODE
	if ((gpio_nr >=30 && gpio_nr <=34)|(gpio_nr == 36)|(gpio_nr >=39 && gpio_nr <=43)|(gpio_nr == 44)) {
		unsigned long old = ra_inl(RT2880_GPIO2MODE_REG), new;//RT2880_GPIO2MODE_REG = 0xb0000064

		ra_and(RT2880_GPIO2MODE_REG, ~(mask << shift));//set "0"
		if (val)
			ra_or(RT2880_GPIO2MODE_REG, val << shift);//set "1"
		if (old != (new = ra_inl(RT2880_GPIO2MODE_REG))) {
			//debug("GPIO#%d updated GPIOMODE register: %08lx -> %08lx\n",gpio_nr, old, new);//---by bond
		}
	}
	
	return 0;
}

/* Read GPIO pin value.
 * @gpio_nr:	GPIO number
 * @return:	GPIO value
 * 	0/1:	Success
 * 	-1:	Invalid parameter
 */
int mtk7620_get_gpio_pin(unsigned short gpio_nr)
{
	int shift;
	unsigned int reg;
	unsigned long val = 0;

	if (!is_valid_gpio_nr(gpio_nr))
		return -1;

	reg = mtk7620_get_gpio_reg_addr(gpio_nr, GPIO_DATA);
	//+++ by bond
	//printf("--->gpio.c-->mtk7620_get_gpio_pin:---reg = 0x%x\n",reg);
	shift = get_gpio_reg_bit_shift(gpio_nr);
	//printf("--->gpio.c-->mtk7620_get_gpio_pin:---shift = 0x%x\n",shift);

	val = !!(ra_inl(reg) & (1U << shift));
	//printf("--->gpio.c-->mtk7620_get_gpio_pin:---value = 0x%x\n",val);

	return val;
}

/* Set GPIO pin value
 * @gpio_nr:	GPIO number
 * @val:
 * 	0:	Write 0 to GPIO pin
 *  otherwise:	Write 1 to GPIO pin
 * @return:
 * 	0:	Success
 * 	-1:	Invalid parameter
 */
int mtk7620_set_gpio_pin(unsigned short gpio_nr, unsigned int val)
{
	int shift;
	unsigned int reg;

	if (!is_valid_gpio_nr(gpio_nr))
		return -1;

	reg = mtk7620_get_gpio_reg_addr(gpio_nr, GPIO_DATA);
	shift = get_gpio_reg_bit_shift(gpio_nr);

	if (!val)
		ra_and(reg, ~(1U << shift));
	else
		ra_or(reg, 1U << shift);

	return 0;
}
//---by bond #endif	//#if defined(MT7620_MP)

void led_init( void ){
	int i, led[] = { GPIO43, GPIO44 };
	for ( i = 0; i < ARRAY_SIZE( led ); ++i ) {
		mtk7620_set_gpio_dir( led[i], 0 );	/* Set LED as output */
		mtk7620_set_gpio_pin( led[i], 0 );	/* turn on LED */
	}
#if defined(ALL_LED_OFF)
	mtk7620_set_gpio_dir(ALL_LED_OFF_GPIO_NR, 0);	/* Set LED as output */
	mtk7620_set_gpio_pin(ALL_LED_OFF_GPIO_NR, 0);	/* turn on LED */
#endif
}

#if defined(HWNAT_FIX)
void rst_fengine(void)
{
	printf("ppe reset\n");	
	ra_or(RT2880_RSTCTRL_REG, 0x1<<31); //rst ppe
	ra_or(RALINK_FRAME_ENGINE_BASE+0x4,0x1); //rst pse
	udelay(500000);
	ra_and(RT2880_RSTCTRL_REG, ~(0x1<<31)); 
	ra_or(RALINK_FRAME_ENGINE_BASE+0x4,~(0x1)); 
}
#endif

void gpio_init(void)
{
	//---by bond printf( "\n gpio init : WPS / RESET pin\n" );
	mtk7620_set_gpio_dir(WPS_BTN, 1);//if "1"set input
}

unsigned long DETECT(void)
{
	int key = 0;

	if(!mtk7620_get_gpio_pin(RST_BTN)) {
		key = 1;
		printf("reset buootn pressed!\n");
	}
	return key;
}

unsigned long DETECT_WPS(void)
{
	int key = 0;
	//+++by bond
	gpio_init();
	if(!mtk7620_get_gpio_pin(WPS_BTN)) {
		key = 1;
//		printf("wps buootn pressed!\n");
	}
	return key;
}
/*
void PWR_LEDON(void)
{
	mtk7620_set_gpio_pin(PWR_LED, 0);
}
*/
void LEDON( void )
{
	mtk7620_set_gpio_pin( GPIO43, 0 );
	mtk7620_set_gpio_pin( GPIO44, 0 );
}

void LEDOFF( void )
{
	mtk7620_set_gpio_pin( GPIO43, 1 );
	mtk7620_set_gpio_pin( GPIO44, 1 );
}

void ALL_LED_INIT(void)
{
	mtk7620_set_gpio_dir(GPIO0, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO1, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO2, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO3, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO4, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO5, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO6, 0);//if "0"set output
	
	mtk7620_set_gpio_dir(GPIO11, 0);//if "0"set output

	mtk7620_set_gpio_dir(GPIO14, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO15, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO16, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO17, 0);//if "0"set output
	
	mtk7620_set_gpio_dir(GPIO18, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO19, 0);//if "0"set output


	mtk7620_set_gpio_dir(GPIO20, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO21, 0);//if "0"set output

	mtk7620_set_gpio_dir(GPIO22, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO23, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO24, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO25, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO26, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO27, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO28, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO29, 0);//if "0"set output

	mtk7620_set_gpio_dir(GPIO30, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO31, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO32, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO33, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO34, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO35, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO36, 0);//if "0"set output
	
	mtk7620_set_gpio_dir(GPIO37, 0);//if "0"set output

	mtk7620_set_gpio_dir(GPIO39, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO40, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO41, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO42, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO43, 0);//if "0"set output
	
	mtk7620_set_gpio_dir(GPIO44, 0);//if "0"set output

	mtk7620_set_gpio_dir(GPIO45, 0);//if "0"set output
	mtk7620_set_gpio_dir(GPIO46, 0);//if "0"set output

}


//---by bond #if defined(ALL_LED_OFF
void ALL_LEDON(void)
{
	ALL_LED_INIT();
	//mtk7620_set_gpio_pin(ALL_LED_OFF_GPIO_NR, 0);	/* turn on LED */
	mtk7620_set_gpio_pin( GPIO0, 0 );
	mtk7620_set_gpio_pin( GPIO1, 1 );
	mtk7620_set_gpio_pin( GPIO2, 0 );
	mtk7620_set_gpio_pin( GPIO3, 0 );
	mtk7620_set_gpio_pin( GPIO4, 0 );
	mtk7620_set_gpio_pin( GPIO5, 0 );
	mtk7620_set_gpio_pin( GPIO6, 1 );

	mtk7620_set_gpio_pin( GPIO11, 0 );
	
	mtk7620_set_gpio_pin( GPIO14, 0 );
	mtk7620_set_gpio_pin( GPIO15, 0 );
	mtk7620_set_gpio_pin( GPIO16, 0 );
	mtk7620_set_gpio_pin( GPIO17, 0 );

	mtk7620_set_gpio_pin( GPIO18, 0 );
	mtk7620_set_gpio_pin( GPIO19, 0 );

	mtk7620_set_gpio_pin( GPIO20, 0 );
	mtk7620_set_gpio_pin( GPIO21, 0 );

	mtk7620_set_gpio_pin( GPIO22, 1 );
	mtk7620_set_gpio_pin( GPIO23, 1 );
	mtk7620_set_gpio_pin( GPIO24, 0 );
	mtk7620_set_gpio_pin( GPIO25, 0 );
	mtk7620_set_gpio_pin( GPIO26, 0 );
	mtk7620_set_gpio_pin( GPIO27, 0 );
	mtk7620_set_gpio_pin( GPIO28, 0 );
	mtk7620_set_gpio_pin( GPIO29, 0 );

	mtk7620_set_gpio_pin( GPIO30, 0 );
	mtk7620_set_gpio_pin( GPIO31, 0 );
	mtk7620_set_gpio_pin( GPIO32, 0 );
	mtk7620_set_gpio_pin( GPIO33, 0 );
	mtk7620_set_gpio_pin( GPIO34, 0 );
	mtk7620_set_gpio_pin( GPIO35, 0 );
	mtk7620_set_gpio_pin( GPIO36, 0 );
	
	mtk7620_set_gpio_pin( GPIO37, 0 );
	
	mtk7620_set_gpio_pin( GPIO39, 0 );
	mtk7620_set_gpio_pin( GPIO40, 0 );
	mtk7620_set_gpio_pin( GPIO41, 0 );
	mtk7620_set_gpio_pin( GPIO42, 0 );
	mtk7620_set_gpio_pin( GPIO43, 0 );

	
	mtk7620_set_gpio_pin( GPIO44, 0 );

	mtk7620_set_gpio_pin( GPIO45, 0 );
	mtk7620_set_gpio_pin( GPIO46, 0 );
}

void ALL_LEDOFF(void)
{
	ALL_LED_INIT();
	//mtk7620_set_gpio_pin(ALL_LED_OFF_GPIO_NR, 1);	/* turn off LED */
	mtk7620_set_gpio_pin( GPIO0, 1 );
	mtk7620_set_gpio_pin( GPIO1, 0 );
	mtk7620_set_gpio_pin( GPIO2, 1 );
	mtk7620_set_gpio_pin( GPIO3, 1 );
	mtk7620_set_gpio_pin( GPIO4, 1 );
	mtk7620_set_gpio_pin( GPIO5, 1 );
	mtk7620_set_gpio_pin( GPIO6, 0 );
	
	mtk7620_set_gpio_pin( GPIO11, 1 );
	
	mtk7620_set_gpio_pin( GPIO14, 1 );
	mtk7620_set_gpio_pin( GPIO15, 1 );
	mtk7620_set_gpio_pin( GPIO16, 1 );
	mtk7620_set_gpio_pin( GPIO17, 1 );

	mtk7620_set_gpio_pin( GPIO18, 1 );
	mtk7620_set_gpio_pin( GPIO19, 1 );

	mtk7620_set_gpio_pin( GPIO20, 1 );
	mtk7620_set_gpio_pin( GPIO21, 1 );

	mtk7620_set_gpio_pin( GPIO22, 0 );
	mtk7620_set_gpio_pin( GPIO23, 0 );
	mtk7620_set_gpio_pin( GPIO24, 1 );
	mtk7620_set_gpio_pin( GPIO25, 1 );
	mtk7620_set_gpio_pin( GPIO26, 1 );
	mtk7620_set_gpio_pin( GPIO27, 1 );
	mtk7620_set_gpio_pin( GPIO28, 1 );
	mtk7620_set_gpio_pin( GPIO29, 1 );

	mtk7620_set_gpio_pin( GPIO30, 1 );
	mtk7620_set_gpio_pin( GPIO31, 1 );
	mtk7620_set_gpio_pin( GPIO32, 1 );
	mtk7620_set_gpio_pin( GPIO33, 1 );
	mtk7620_set_gpio_pin( GPIO34, 1 );
	mtk7620_set_gpio_pin( GPIO35, 1 );
	mtk7620_set_gpio_pin( GPIO36, 1 );
	
	mtk7620_set_gpio_pin( GPIO37, 1 );
	
	mtk7620_set_gpio_pin( GPIO39, 1 );
	mtk7620_set_gpio_pin( GPIO40, 1 );
	mtk7620_set_gpio_pin( GPIO41, 1 );
	mtk7620_set_gpio_pin( GPIO42, 1 );
	mtk7620_set_gpio_pin( GPIO43, 1 );
	
	mtk7620_set_gpio_pin( GPIO44, 1 );
	mtk7620_set_gpio_pin( GPIO45, 1 );
	mtk7620_set_gpio_pin( GPIO46, 1 );
}
//#endif	//---by bondif defined(ALL_LED_OFF)
