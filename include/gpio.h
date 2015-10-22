#ifndef __GPIO_H__
#define __GPIO_H__

//#ifndef __NET_H__
//#define __NET_H__


//---by bond #if defined(MT7620_MP)
/* LED, Button GPIO# definition */
#define RST_BTN		12
//#define WPS_BTN		12
#define WPS_BTN		38

#if defined(MT7620_MP)
//#define PWR_LED	GND
#define WIFI_2G_LED	72
#define WAN_LED		44
#endif

#if defined (MT7628_ASIC_BOARD)
#define GPIO0		0
#define GPIO1		1
#define GPIO2		2
#define GPIO3		3
//I2C
#define GPIO4		4
#define GPIO5		5
//SPI_CS1
#define GPIO6		6

//GPIO
#define GPIO11		11

//SPIS
#define GPIO14		14
#define GPIO15		15
#define GPIO16		16
#define GPIO17		17

//PWM0
#define GPIO18		18
//PWM1
#define GPIO19		19



//UART2/eMMC
#define GPIO20		20
#define GPIO21		21

//SD/eMMC
#define GPIO22		22
#define GPIO23		23
#define GPIO24		24
#define GPIO25		25
#define GPIO26		26
#define GPIO27		27
#define GPIO28		28
#define GPIO29		29

//Px_LED_KN
#define GPIO30		30
#define GPIO31		31
#define GPIO32		32
#define GPIO33		33
#define GPIO34		34
//WLEN_KN
#define GPIO35		35
//PERST
#define GPIO36		36
//REFCLK
#define GPIO37		37		
//Px_LED_AN
#define GPIO39		39
#define GPIO40		40
#define GPIO41		41
#define GPIO42		42
#define GPIO43		43
//WLEN_AN
#define GPIO44		44
//UART1
#define GPIO45		45
#define GPIO46		46



#endif


enum gpio_reg_id {
	GPIO_INT = 0,
	GPIO_EDGE,
	GPIO_RMASK,
	GPIO_MASK,
	GPIO_DATA,
	GPIO_DIR,
	GPIO_POL,
	GPIO_SET,
	GPIO_RESET,
	GPIO_TOG,
	GPIO_MAX_REG
};

extern unsigned int mtk7620_get_gpio_reg_addr(unsigned short gpio_nr, enum gpio_reg_id id);
extern int mtk7620_set_gpio_dir(unsigned short gpio_nr, unsigned short gpio_dir);
extern int mtk7620_get_gpio_pin(unsigned short gpio_nr);
extern int mtk7620_set_gpio_pin(unsigned short gpio_nr, unsigned int val);
//#endif

extern void led_init(void);
extern void gpio_init(void);
extern void LEDON(void);
extern void LEDOFF(void);
extern unsigned long DETECT(void);
extern unsigned long DETECT_WPS(void);
extern void rst_fengine(void);

//#if defined(ALL_LED_OFF)
extern void ALL_LEDON(void);
extern void ALL_LEDOFF(void);
//#endif

//---by bond #endif	//#if defined(MT7620_MP)
#endif //__GPIO_H__
