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


struct gronic_keypad_data {
	char  key;
};

void init_gpio(void){
	//led on 
	config_pin_mode(PIO_G,9,PIN_MODE_OUTPUT);

	//config spi
	config_pin_mode(PIO_E, SPI_MOSI, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_MISO, PIN_MODE_INPUT);
	config_pin_mode(PIO_E, SPI_CS0, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_SCLK, PIN_MODE_OUTPUT);

	//Interupt key press
	config_pin_pull(PIO_B, KEYPAD_IRQ_PIN, PULL_MODE_UP);
	// config_pin_mode(PIO_B, KEYPAD_IRQ_PIN, PIN_MODE_INT);
	config_pin_mode(PIO_B, KEYPAD_IRQ_PIN, PIN_MODE_INPUT);


	config_pin_pull(PIO_E, 3, PULL_MODE_UP);



	config_pin_mode(PIO_E, 10, PIN_MODE_INPUT);
	config_pin_pull(PIO_E, 10, PULL_MODE_DOWN);

	//Init Select 
	config_pin_mode(PIO_E, SPI_SEL_0, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_E, SPI_SEL_1, PIN_MODE_OUTPUT);
	config_pin_mode(PIO_B, PER_RST, PIN_MODE_OUTPUT);


	set_pin_value(PIO_B,PER_RST,1);
	set_pin_value(PIO_B,PER_RST,0);
	set_pin_value(PIO_B,PER_RST,1);


}

struct task_struct *task;


const ADR_REG KEYB_MCP[]={ 
   {0,0},
   { GPIO_A | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_B | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_A | MCP_IODIR, 0xF0 }, // GPIO-A Direction     output 4 bit Row
   { GPIO_B | MCP_IODIR, 0x1F}, // GPIO-B Direction     input 5 bit Column
   { GPIO_A | MCP_GPPU, 0x0F }, // GPIO-A Pull Up Res 
   { GPIO_B | MCP_GPPU, 0x1F }, // GPIO-B Pull Up Res    Res Column on
   { GPIO_A | MCP_GPINTEN, 0x00 }, // GPIO-A Interrupt on changed
   { GPIO_B | MCP_GPINTEN, 0x1F }, // GPIO-B Interrupt on changed  Interr. Column on
   { GPIO_A | MCP_OLAT, 0x00}, // GPIO-A default output level 
   { GPIO_A | MCP_IPOL, 0x00 }, 
   { GPIO_B | MCP_IPOL, 0x00 }, 
   {0xFF,0}
};


const ADR_REG CONTR_MCP[]={
   {3,0},
   { GPIO_A | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_B | MCP_IOCON,  IOCON_ODR }, // control     Bank, ODR
   { GPIO_A | MCP_IODIR, 0x02 }, // GPIO-A Direction     output Bit1 input
   { GPIO_B | MCP_IODIR, 0x00 }, // GPIO-B Direction     output LCD DB8..0
   { GPIO_A | MCP_GPPU, 0x00 }, // GPIO-A Pull Up Res 
   { GPIO_B | MCP_GPPU, 0xFF }, // GPIO-B Pull Up Res    Res DB on
   { GPIO_A | MCP_GPINTEN, 0x00 }, // GPIO-A Interrupt on changed
   { GPIO_B | MCP_GPINTEN, 0x00 }, // GPIO-B Interrupt on changed   
   { GPIO_A | MCP_OLAT, 0x61 }, // GPIO-A default output level 
   { GPIO_B | MCP_OLAT, 0xFF }, // GPIO-B default output level
    { GPIO_A | MCP_IPOL, 0x00 },
   { GPIO_B | MCP_IPOL, 0x00 },
   {0xFF,0   }
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

u32 SPI_MCP(__u8 mcp_chip, __u32 sdata){

	 u32 smask = (1<<23);
	 u32 rdat  = (0x00000000);

	 e_port = (PIO_REG_DATA_VALUE(PIO_E) & 0x000000F0) | (mcp_chip << 8)  ; // select MCP-Chip  CS low
	 *PIO_REG_DATA(PIO_E) = e_port;

	  do{
		if((sdata & smask) == 0)e_port = e_port & ~BB_SPI2_MOSI;   // clk low
		else     e_port = e_port |  BB_SPI2_MOSI;
		*PIO_REG_DATA( PIO_E) = e_port;          // MOSI bit out
		*PIO_REG_DATA( PIO_E) = e_port | BB_SPI2_CLK;      // clk high
		if(PIO_REG_DATA_VALUE(PIO_E) & BB_SPI2_MISO) rdat = rdat | smask;
		smask = smask >> 1;
	 }while(smask);
	 
	 e_port = e_port | BB_SPI2_CS0;  
	 *PIO_REG_DATA( PIO_E) = e_port;           // CS high
	 return(rdat);
}


void mcp23_init( const ADR_REG *chip ){
	u8 addr = chip->mcpreg;
	chip++;
	while(chip->mcpreg != 0xFF){
		SPI_MCP(addr, MCP_WRITE | (chip->mcpreg<<8) | chip->mcpvalu);
		chip++;
	}
}


void thread_function(void *data){
	char cnt = 0;
	char key;

	__u32 reg_val;
	__u8 col_reg, row_reg;
	printk("Thread start\n");


	set_pin_value(PIO_G,9,1);


	while(!kthread_should_stop()){
		reg_val = PIO_REG_DATA_VALUE(PIO_B) ;
		// printk("XX %X\n",reg_val);
		if((reg_val & (1<<KEYPAD_IRQ_PIN)) ==0) {

			col_reg = SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_INTCAP) << 8) | 0) & 0x1F;

			SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_IODIR) << 8) | 0xFF);
			SPI_MCP(0, MCP_WRITE | (( GPIO_B | MCP_IODIR) << 8) | 0xE0);
			SPI_MCP(0, MCP_WRITE | (( GPIO_B | MCP_OLAT) << 8) | 0x00);

			row_reg = SPI_MCP(0, MCP_READ | (( GPIO_A | MCP_GPIO) << 8) | 0) & 0x0F;
			
			SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_IODIR) << 8) | 0xF0);
			SPI_MCP(0, MCP_WRITE | (( GPIO_B | MCP_IODIR) << 8) | 0x1F);
			SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0x00);

			SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_INTCAP) << 8) | 0) & 0x1F;



			switch((__u32)(col_reg<<4)|row_reg){
				case 0x0FE : key = '+' ;break;
				case 0x0FD : key = 'M' ;break;
				case 0x0FB : key = '-' ;break;
				case 0x17E : key = '1' ;break;
				case 0x17D : key = '2' ;break;
				case 0x17B : key = '3' ;break;
				case 0x1BE : key = '4' ;break;
				case 0x1BD : key = '5' ;break;
				case 0x1BB : key = '6' ;break;
				case 0x1DE : key = '7' ;break;
				case 0x1DD : key = '8' ;break;
				case 0x1DB : key = '9' ;break;
				case 0x1ED : key = '0' ;break;
				case 0x1EE : key = '.' ;break;
				case 0x1EB : key = '?' ;break;
				case 0x177 : key = 'A' ;break;
				case 0x1B7: key = 'K' ;break;
				case 0x1D7: key = 'O' ;break;
				// case 0x1FF: key = 'R' ;break;
				default : key = 0; break;
			}

			if(key) printk("Key %c \n",key);


			cnt++;
		}


		// SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (cnt & 0xFF));
		if(cnt & 1 )
			SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0x40);
		else 
			SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0x00);

		// SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0x40);
		// SPI_MCP(0, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0xFF);


		// reg_val = SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_GPIO) << 8) | 0);

		// printk("%X\n",reg_val);

      		// set_pin_value(PIO_G,9,cnt & 0x01);

		// config_pin_pull(PIO_B, KEYPAD_IRQ_PIN, cnt & 0x01);


		// msleep(25);
		// SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | 0x00);

		msleep(50);
	}

	//LED off 
	set_pin_value(PIO_G,9,0);

	free_irq(KEYPAD_IRQ_NO,key_data);
 	input_unregister_device(keypad_dev);

}


static irqreturn_t key_pad_irq_handler(int irq, void *devid)
{
	__u32 reg_val;
	printk("Gronic IRQ Handler \n");	
	set_pin_value(PIO_B,PER_RST,0);

	reg_val = SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_INTCAP) << 8) | 0) & 0x1F;
	printk("Key %X\n",reg_val);

		// input_report_key(keypad_dev, BTN_0,'a');
		// input_sync(keypad_dev);

	return IRQ_HANDLED;
}


void init_keypad(void){
	int irq_err = 0;

	irq_err = request_irq(KEYPAD_IRQ_NO, key_pad_irq_handler, IRQF_NO_SUSPEND, "gronic_keypad", key_data);
	printk("Gronic IRQ Req : %d\n", irq_err);


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
	__u32 reg_val;
	pr_info("Gronic system driver init\n");
	
	init_gpio();
	init_keypad();



	mcp23_init( KEYB_MCP );						// Tastatur initialisieren
	mcp23_init( CONTR_MCP );					// Drucker initialisieren

	task = kthread_run(&thread_function,(void *)data,"gronic-system");

	return 0;
}


static void __exit gronic_exit(void) {
	kthread_stop(task);

	pr_info("Gronic system driver exit \n");
}

module_init(gronic_init);
module_exit(gronic_exit);
MODULE_LICENSE("Dual BSD/GPL");