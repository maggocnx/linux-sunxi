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

char*  lcd_buf;



static struct input_dev *keypad_dev;
static void __exit gronic_exit(void);

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



u8 cont_reg_a = 0x60 ;		// CS= high BLT= on

void disp_data(__u8 lcd_dat){
 	cont_reg_a |= 0x80;         // A0= high 
	SPI_MCP(3, MCP_WRITE | (( GPIO_B | MCP_OLAT) << 8) | lcd_dat);
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (cont_reg_a & ~0x20) );  // CS= low
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (cont_reg_a | 0x20) );  // CS= high
}

void disp_cmd(__u8 lcd_cmd){
	cont_reg_a &= ~0x80;         // A0= low 
	SPI_MCP(3, MCP_WRITE | (( GPIO_B | MCP_OLAT) << 8) | lcd_cmd);
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (cont_reg_a & ~0x20) );  // CS= low
	SPI_MCP(3, MCP_WRITE | (( GPIO_A | MCP_OLAT) << 8) | (cont_reg_a | 0x20) );  // CS= high
}

__u8 LCD_image(char * image){
__u8 page,i,j,v,mask_v,mask_h;
__u8 im_buf[128];
 
                for(page=0; page<8; page++){
                               memset(im_buf,0,128);
                               mask_v=0x01;
                               for(v=0;v<8;v++){
                                               for(j=0;j<16;j++){                                                                                                          // von horizontal pixel nach vertikal pixel wandeln
                                                               mask_h=0x80;
                                                               for(i=0;i<8;i++){
                                                               	      if(*image & mask_h) im_buf[(j*8)+i] |= mask_v;
                                                                              mask_h=mask_h>>1;
                                                   }
                                                               image++;
                                               }
                                               mask_v = mask_v<<1;
                               }
 
                               disp_cmd(START_LINE_SET);
                               disp_cmd(PAGE_ADDRESS_SET + page);
                               disp_cmd(COLUMN_ADDRESS_HIGH);
                               disp_cmd(COLUMN_ADDRESS_LOW);
                               for(i=0;i<NUMBER_OF_COLUMNS;i++){                                                // 0..127
                               disp_data(im_buf[i]);
                               }
                }
                return(0);
}

	
	
/******************************************************************************
This performs all of the necessary initialization of the SED-1565 controller
Init_Display();
******************************************************************************/
u8 LCD_vol=8;
#define  LCDVOL_MAX 25


int display_open(struct inode *inode, struct file *filp) {

  /* Success */
 	return 0;
}

int display_release(struct inode *inode, struct file *filp) {
 
  /* Success */
 	return 0;
}


ssize_t display_write( struct file *filp, char *buf, size_t count, loff_t *f_pos) {
	char *tmp;
	tmp=buf+count-1;

	if(count>BUF_SIZE) count = BUF_SIZE;
	copy_from_user(lcd_buf,buf,count);

	LCD_image(lcd_buf);
	return count;
}




struct file_operations lcd_fops = {
	write : display_write,
	open: display_open,
	release: display_release
};



void register_lcd(void){
	int result;

	 result = register_chrdev(BUF_SIZE, "backlcd", &lcd_fops);
  		if (result < 0) {
    			printk("<1>memory: cannot obtain major number %d\n", BUF_SIZE);
    		return result;
  	}

  	lcd_buf = kmalloc(BUF_SIZE, GFP_KERNEL); 
  	if (!lcd_buf) { 
    		result = -ENOMEM;
    		goto fail; 
  	} 
	memset(lcd_buf,0x88,BUF_SIZE);


  	printk("Gronic Inserting memory module Major Number : %d \n", BUF_SIZE); 
  	return 0;

  fail:
     	gronic_exit(); 
    	return result;

}


void Init_LCD(void){
	disp_cmd(0);
	disp_cmd(RESET_DISPLAY);
	disp_cmd(LCD_BIAS_1_9);
	disp_cmd(ADC_SELECT_NORMAL);
	disp_cmd(COMMON_OUTPUT_REVERSE);
	disp_cmd(V5_RESISTOR_RATIO);
	disp_cmd(ELECTRONIC_VOLUME_SET);
	disp_cmd(LCD_vol);
	disp_cmd(START_LINE_SET);
	disp_cmd((POWER_CONTROL_SET | VOLTAGE_REGULATOR |	VOLTAGE_FOLLOWER | BOOSTER_CIRCUIT));
	disp_cmd(DISPLAY_NORMAL);
	disp_cmd(DISPLAY_ON);
}

// void Off_Display(void){
// 	disp_cmd(0);
// 	disp_cmd(RESET_DISPLAY);
// 	disp_cmd(POWER_CONTROL_SET);
// 	disp_cmd(DISPLAY_OFF);
// }



void thread_function(void *data){
	char cnt = 0;
	__u32  key,last_key;

	__u32 reg_val;
	__u8 col_reg, row_reg;

	bool flg;

	printk("Gronic thread start\n");

	set_pin_value(PIO_G,9,1);


	while(!kthread_should_stop()){

		reg_val = PIO_REG_DATA_VALUE(PIO_B) ;
		
		input_report_key(keypad_dev, 0,1);

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
				case 0x0FE : key = KEY_UP ;break;
				case 0x0FD : key = KEY_F2 ;break;
				case 0x0FB : key = KEY_DOWN ;break;
				case 0x17E : key = KEY_1 ;break;
				case 0x17D : key = KEY_2 ;break;
				case 0x17B : key = KEY_3 ;break;
				case 0x1BE : key = KEY_4 ;break;
				case 0x1BD : key = KEY_5 ;break;
				case 0x1BB : key = KEY_6 ;break;
				case 0x1DE : key = KEY_7 ;break;
				case 0x1DD : key = KEY_8 ;break;
				case 0x1DB : key = KEY_9 ;break;
				case 0x1ED : key = KEY_0 ;break;
				case 0x1EE : key = KEY_DOT ;break;
				case 0x1EB : key = KEY_QUESTION;break;
				case 0x177 : key = KEY_A;break;
				case 0x1B7 :  key  = KEY_BACKSPACE ;break;
				case 0x1D7 :  key  = KEY_ENTER;break;
				// case 0x1FF :   key   = 0 ;break;
				default : key = 0; break;
			}
			


			if(key){
			LCD_image(lcd_buf);

				last_key = key;
				input_report_key(keypad_dev, key,true);

			}
			else{
				if(last_key)
					input_report_key(keypad_dev,last_key,false);
			}
			input_sync(keypad_dev);
			cnt++;
		}
		msleep(20);
	}
	set_pin_value(PIO_G,9,0);
}




// static irqreturn_t key_pad_irq_handler(int irq, void *devid)
// {
// 	__u32 reg_val;
// 	printk("Gronic IRQ Handler \n");	
// 	set_pin_value(PIO_B,PER_RST,0);

// 	reg_val = SPI_MCP(0, MCP_READ | (( GPIO_B | MCP_INTCAP) << 8) | 0) & 0x1F;
// 	printk("Key %X\n",reg_val);

// 	// input_report_key(keypad_dev, BTN_0,'a');
// 	// 	input_sync(keypad_dev);

// 	return IRQ_HANDLED;
// }


void init_keypad(void){
	int error;

	// if (request_irq(KEYPAD_IRQ_NO, key_pad_irq_handler, 0, "gronic_keypad", NULL)) {
 //                printk(KERN_ERR "Gronic Can't allocate irq %d\n", KEYPAD_IRQ_NO);
 //                return -EBUSY;
 //        	}
 //        	else{
	// 	printk("Gronic IRQ Req ");
 //        	}



	keypad_dev = input_allocate_device();
	if (!keypad_dev) {
		printk(KERN_ERR "Gronic : Not enough memory\n");
		error = -ENOMEM;
		goto err_free_irq;
	}

	keypad_dev->name = "Gronic Keypad";
	keypad_dev->evbit[0] = BIT_MASK(EV_KEY);
		
	keypad_dev->keybit[BIT_WORD(KEY_1 )]= BIT_MASK(KEY_1 )  |  BIT_MASK(KEY_2 ) | BIT_MASK(KEY_3 )| BIT_MASK(KEY_4 )| BIT_MASK(KEY_5 )| BIT_MASK(KEY_6 )| 
			        BIT_MASK(KEY_7 )| BIT_MASK(KEY_8 )| BIT_MASK(KEY_9 )| BIT_MASK(KEY_0)| BIT_MASK(KEY_BACKSPACE ); 
	keypad_dev->keybit[BIT_WORD(KEY_DOT )] |= BIT_MASK(KEY_DOT );
	keypad_dev->keybit[BIT_WORD(KEY_QUESTION)]|= BIT_MASK(KEY_QUESTION);
	keypad_dev->keybit[BIT_WORD(KEY_A )]|= BIT_MASK(KEY_A );
	keypad_dev->keybit[BIT_WORD(KEY_BACKSPACE)]|= BIT_MASK(KEY_BACKSPACE );
	keypad_dev->keybit[BIT_WORD(KEY_ENTER )]|= BIT_MASK(KEY_ENTER);
	keypad_dev->keybit[BIT_WORD(KEY_UP )] |= BIT_MASK(KEY_UP) | BIT_MASK(KEY_DOWN) ;
	keypad_dev->keybit[BIT_WORD(KEY_F2)]  |= BIT_MASK(KEY_F2);


	error = input_register_device(keypad_dev);
	if (error) {
		printk(KERN_ERR "Gronic : Failed to register device\n");
		goto err_free_dev;
	}

	return 0;

 err_free_dev:
	input_free_device(keypad_dev);
 err_free_irq:
	// free_irq(KEYPAD_IRQ_NO, key_pad_irq_handler);
	return error;
}


static int __init gronic_init(void) {
	int data = 20;
	__u32 reg_val;
	pr_info("Gronic system driver init\n");

	init_gpio();
	init_keypad();


	mcp23_init( KEYB_MCP );						// Tastatur initialisieren
	mcp23_init( CONTR_MCP );					// Drucker initialisieren
	
	register_lcd();
	Init_LCD();
	


	LCD_image((char*) lcd_buf);

	task = kthread_run(&thread_function,(void *)data,"gronic-system");

	return 0;
}


static void __exit gronic_exit(void) {

	unregister_chrdev(BUF_SIZE, "backlight");

 	input_unregister_device(keypad_dev);


	/* Freeing buffer memory */
	 if (lcd_buf) {
	     kfree(lcd_buf);
	 }


	kthread_stop(task);
	// exit();
	pr_info("Gronic system driver exit \n");
}

module_init(gronic_init);
module_exit(gronic_exit);
MODULE_LICENSE("Dual BSD/GPL");