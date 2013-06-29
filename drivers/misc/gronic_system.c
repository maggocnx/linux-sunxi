#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/posix-timers.h>
#include <linux/kthread.h>
#include <mach/platform.h>
#include <linux/delay.h>
#include <linux/input.h>
#include "gronic_system.h"


struct gronic_keypad_data *key_data = NULL;
__u32 e_port;

static struct input_dev *keypad_dev;

void config_pin_mode(__u32  port, __u32  pin, __u32 mode){
	__u32 reg_val = 0;

	reg_val = (PIO_REG_CFG_VALUE(port,pin/8)) & ~ (0x07<<((pin & 0x07)<<2));
	reg_val =  reg_val | (mode<<((pin & 0x07)<<2));
	*PIO_REG_CFG(port,pin/8) = reg_val;
}

void config_pin_pull(__u32  port, __u32  pin, __u32 mode){
	__u32 reg_val = 0;

	reg_val = (PIO_REG_PULL_VALUE(port,pin/16)) & ~ (0x03<<((pin & 0x0F)<<1));
	reg_val =  reg_val | (mode<<((pin & 0x0F)<<1));

	*PIO_REG_PULL(port,0) = reg_val;
}

void set_pin_value(__u32 port, __u32 pin, __u32 value){
	__u32 reg_val = PIO_REG_DATA_VALUE(port);
	if(value)
		reg_val =  reg_val | (0x01 << pin) ; 
	else 
		reg_val = reg_val & ~(0x01 << pin); 

	*PIO_REG_DATA(port) = reg_val;
}

char  get_pin_value(__u32 port,  __u32 pin){
	__u32 pin_msk = 0x01 << pin;
	if ((PIO_REG_DATA_VALUE(port) &  pin_msk)>0 )
		return 0;
	else 
		return 1;
}

static irqreturn_t key_pad_irq_handler(int irq, void *devid)
{
	printk("Gronic IRQ Handler \n");	
	set_pin_value(PIO_B,PER_RST,0);

	return IRQ_HANDLED;
}

struct gronic_keypad_data {
	char  key;
};

void init_gpio(void){
	int irq_err = 0;
	//led on 
	config_pin_mode(PIO_G,9,PIN_MODE_OUTPUT);
	set_pin_value(PIO_G,9,1);

	//config spi
	config_pin_mode(PIO_E, SPI_MOSI, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_MISO, PIN_MODE_INPUT);
	config_pin_mode(PIO_E, SPI_CS0, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_SCLK, PIN_MODE_OUTPUT);

	//Interupt key press
	config_pin_mode(PIO_B, KEYPAD_IRQ_PIN, PIN_MODE_INPUT);
	config_pin_pull(PIO_B, KEYPAD_IRQ_PIN, PULL_MODE_UP);


	config_pin_mode(PIO_E, 10, PIN_MODE_INPUT);
	config_pin_pull(PIO_E, 10, PULL_MODE_DOWN);

	//Init Select 
	config_pin_mode(PIO_E, SPI_SEL_0, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_SEL_1, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_B, PER_RST, PIN_MODE_OUTPUT);

	set_pin_value(PIO_B,PER_RST,1);

	irq_err = request_irq(KEYPAD_IRQ_NO, key_pad_irq_handler, IRQF_NO_SUSPEND, "gronic_keypad", key_data);

	printk("Gronic IRQ Req : %d\n", irq_err);

}

struct task_struct *task;


const ADR_REG KEYB_MCP[]={ 
   {0,0},
   { GPIO_A | MCP_IOCON, IOCON_BANK | IOCON_ODR }, // control     Bank, ODR
   { GPIO_B | MCP_IOCON, IOCON_BANK | IOCON_ODR }, // control     Bank, ODR
   { GPIO_A | MCP_IODIR, 0x00 }, // GPIO-A Direction     output 4 bit Row
   { GPIO_B | MCP_IODIR, 0x1F }, // GPIO-B Direction     input 5 bit Column
   { GPIO_A | MCP_GPPU, 0x00 }, // GPIO-A Pull Up Res 
   { GPIO_B | MCP_GPPU, 0x1F }, // GPIO-B Pull Up Res    Res Column on
   { GPIO_A | MCP_GPINTEN, 0x00 }, // GPIO-A Interrupt on changed
   { GPIO_B | MCP_GPINTEN, 0x1F }, // GPIO-B Interrupt on changed  Interr. Column on
   { GPIO_A | MCP_OLAT, 0x00 }, // GPIO-A default output level 
   {0xFF,0}
};


const ADR_REG CONTR_MCP[]={
   {3,0},
   { GPIO_A | MCP_IOCON, IOCON_BANK | IOCON_ODR }, // control     Bank, ODR
   { GPIO_B | MCP_IOCON, IOCON_BANK | IOCON_ODR }, // control     Bank, ODR
   { GPIO_A | MCP_IODIR, 0x02 }, // GPIO-A Direction     output Bit1 input
   { GPIO_B | MCP_IODIR, 0x00 }, // GPIO-B Direction     output LCD DB8..0
   { GPIO_A | MCP_GPPU, 0x00 }, // GPIO-A Pull Up Res 
   { GPIO_B | MCP_GPPU, 0xFF }, // GPIO-B Pull Up Res    Res DB on
   { GPIO_A | MCP_GPINTEN, 0x00 }, // GPIO-A Interrupt on changed
   { GPIO_B | MCP_GPINTEN, 0x00 }, // GPIO-B Interrupt on changed   
   { GPIO_A | MCP_OLAT, 0x60 }, // GPIO-A default output level 
   { GPIO_B | MCP_OLAT, 0xFF }, // GPIO-B default output level
   {0xFF,0
   }
};

// void spi_mcp23_config( void){
// 	*SPI_A13_REG(SPI_CTL) 	= 0x000A0013;		// DEL,PHA,MODE,ENABLE
// 	*SPI_A13_REG(SPI_WAIT)	= 0x00000080;		// WAIT Timer
// 	*SPI_A13_REG(SPI_CCTL)	= 0x00000010;		// Clock Freq.
// }

void spi_thermal_config( void){

	*SPI_A13_REG(SPI_CTL)	= 0x000A0013;		// DEL,PHA,MODE,ENABLE
	*SPI_A13_REG(SPI_WAIT)	= 0x00000080;		// WAIT Timer
	*SPI_A13_REG(SPI_CCTL)	= 0x00000010;		// Clock Freq.
}


static void spi_cs_set( u8 select){
	switch(select){
		case 0:						// MCP23S17  ext. GPIO
			set_pin_value(PIO_E, SPI_SEL_0, 0);
			set_pin_value(PIO_E, SPI_SEL_1, 0);
		break;
		
		case 1:						// MCP23S17  Thermal Printer Control
			set_pin_value(PIO_E, SPI_SEL_0, 0);
			set_pin_value(PIO_E, SPI_SEL_1, 1);
		break;

		case 2:						// MCP23S17  Front-LCD
			set_pin_value(PIO_E, SPI_SEL_0, 1);
			set_pin_value(PIO_E, SPI_SEL_1, 0);
		break;

		case 3:						// Thermal Printer Head
			set_pin_value(PIO_E, SPI_SEL_0, 1);
			set_pin_value(PIO_E, SPI_SEL_1, 1);
		break;
		
	}
	
}

u32 SPI_A13_SET(__u32 sdata){
	u32 smask = (0x00000080);
	u32 rdat  = (0x00000000);

										// CS low

	do{
		if((sdata & smask) == 0)		e_port = e_port & ~BB_SPI2_MOSI;
		else					e_port = e_port | BB_SPI2_MOSI;
		*PIO_REG_DATA( PIO_E) =  e_port;										// MOSI bit out
		*PIO_REG_DATA( PIO_E) = e_port | BB_SPI2_CLK;						// clk high
		if(PIO_REG_DATA_VALUE(PIO_E) & BB_SPI2_MISO) rdat = rdat | smask;
		smask = smask >> 1;
		*PIO_REG_DATA( PIO_E) = e_port;										// clk low
	}while(smask);
	
										// CS high
	return(rdat);
}


void MCPSPI_wr(u8 mcp, u8 mreg, u8 mvalu){								// MCP Register schreiben


	e_port = (PIO_REG_DATA_VALUE(PIO_E) & 0x000000F0) | BB_SPI2_CS0 | (mcp << 8) ;				// select ext. CS 
	*PIO_REG_DATA(PIO_E) = e_port;

	e_port = e_port & ~ BB_SPI2_CS0;
	*PIO_REG_DATA(PIO_E) =  e_port;		
	
	SPI_A13_SET(0x40);										// MCP Adr.
	SPI_A13_SET(mreg);										// MCP Reg 
	SPI_A13_SET(mvalu);										// Value

	e_port = e_port & ~BB_SPI2_MOSI;									// MOSI low
	e_port = e_port | BB_SPI2_CS0;
	*PIO_REG_DATA( PIO_E) = e_port;		
}

u32 MCPSPI_rd(u8 mcp, u8 mreg){											// MCP Register lesen
	__u32 ret;
	e_port = (PIO_REG_DATA_VALUE(PIO_E) & 0x000000F0) | BB_SPI2_CS0 | (mcp<<8);				// select ext. CS
	*PIO_REG_DATA(PIO_E) = e_port;

	e_port = e_port & ~ BB_SPI2_CS0;
	*PIO_REG_DATA(PIO_E) =  e_port;	

	SPI_A13_SET(0x41);										// MCP Adr.
	SPI_A13_SET(mreg);										// MCP Reg
	ret=SPI_A13_SET(0);		

	e_port = e_port & ~BB_SPI2_MOSI;									// MOSI low
	e_port = e_port | BB_SPI2_CS0;
	*PIO_REG_DATA( PIO_E) = e_port;
									// Value
	return(ret);
}

void mcp23_init( const ADR_REG *chip){
	u8 addr = chip->mcpreg;
	// spi_mcp23_config();
	chip++;
	while(chip->mcpreg != 0xFF){
		MCPSPI_wr(addr,chip->mcpreg,chip->mcpvalu);
		chip++;
	}
}

void thread_function(void *data){
	int cnt = 0;
	printk("Thread start");
	while(!kthread_should_stop()){
		
		input_report_key(keypad_dev, BTN_0,'a');
		input_sync(keypad_dev);


		MCPSPI_wr(3,GPIO_A | MCP_OLAT,0x40);
		// set_pin_value(PIO_G,9,cnt & 0x01);

		// config_pin_pull(PIO_B, KEYPAD_IRQ_PIN, cnt & 0x01);


		cnt++;
		msleep(10);
		MCPSPI_wr(3,GPIO_A | MCP_OLAT,0x01);
		msleep(10);
	}

	set_pin_value(PIO_B,PER_RST,0);
	set_pin_value(PIO_G,9,0);

 	input_unregister_device(keypad_dev);

}


void init_keypad(void){
	int error ;
	keypad_dev = input_allocate_device();
	if (!keypad_dev) {
		printk(KERN_ERR "button.c: Not enough memory\n");
	}

	keypad_dev->evbit[0] = BIT_MASK(EV_KEY);
	keypad_dev->keybit[BIT_WORD(BTN_0)] = BIT_MASK(BTN_0);

	error = input_register_device(keypad_dev);
	if (error) {
		printk( "button.c: Failed to register device\n");
	}

}

static int __init gronic_init(void) {
	int data = 20;

	pr_info("Gronic system driver init\n");
	
	init_gpio();
	init_keypad();

	// reg_val = get_pin_value(PIO_E, 10);
	// pr_info("Gronic  %X - \n" , reg_val);

	mcp23_init( KEYB_MCP );						// Tastatur initialisieren
	mcp23_init( CONTR_MCP );					// Drucker initialisieren

	task = kthread_run(&thread_function,(void *)data,"gronic-system");

	return 0;
}


static void __exit gronic_exit(void) {
	free_irq(KEYPAD_IRQ_NO,key_data);
	kthread_stop(task);

	pr_info("Gronic system driver exit \n");
}

module_init(gronic_init);
module_exit(gronic_exit);
MODULE_LICENSE("Dual BSD/GPL");